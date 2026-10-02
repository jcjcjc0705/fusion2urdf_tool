# Author-syuntoku14
# Description-Generate URDF file from Fusion 360
#
# Requirement:
# - If ANY error happens (including early-return), rollback to the very beginning state
#   and DELETE all history (timeline items) created after the script started.
#
# Key point:
# - Fusion often cannot delete suppressed timeline items while the timeline marker is not at the end.
# - Therefore, on failure we:
#   (1) Move marker to END (make timeline items deletable)
#   (2) Delete items with index >= start_timeline_count (truncate)
#   (3) Move marker to END again (optional, stabilize UI)
#
# NOTE:
# - This works only when "Design History" is enabled (Parametric modeling).
# - If the design is in Direct Modeling mode, timeline deletion is not available.

import adsk
import adsk.core
import adsk.fusion
import traceback
import os
import shutil
from pathlib import Path

from .utils import utils
from .utils.xacro2urdf import convert_xacro_to_urdf
from .core import Link, Joint, Write

# -----------------------------
# Rollback / Truncate (GLOBAL)
# -----------------------------
_start_marker_index = None
_start_timeline_count = None
_need_rollback = False


def _get_timeline_marker_index(design: adsk.fusion.Design):
    """
    Record the current timeline marker position index.
    Fusion exposes markerPosition differently by version, so try multiple approaches.
    """
    try:
        tl = design.timeline
    except:
        return None

    # Preferred: markerPosition has an index
    try:
        marker = tl.markerPosition
        if hasattr(marker, 'index'):
            return int(marker.index)
    except:
        pass

    # Fallback: approximate using current timeline end
    try:
        return int(tl.count)
    except:
        return None


def _try_set_timeline_marker(design: adsk.fusion.Design, marker_index: int) -> bool:
    """
    Try to move timeline marker to marker_index.
    Returns True if success, False otherwise.
    """
    if marker_index is None:
        return False

    try:
        tl = design.timeline
    except:
        return False

    # Clamp index
    try:
        count = int(tl.count)
        if marker_index < 0:
            marker_index = 0
        if marker_index > count:
            marker_index = count
    except:
        pass

    # Some builds accept setting markerPosition to a timeline object
    try:
        tl.markerPosition = tl.item(marker_index)
        return True
    except:
        pass

    # Some builds accept setting markerPosition to an integer
    try:
        tl.markerPosition = marker_index
        return True
    except:
        pass

    return False


def _move_marker_to_end(design: adsk.fusion.Design):
    """
    Move timeline marker to the end. Needed to ensure timeline items are deletable.
    """
    tl = design.timeline
    c = int(tl.count)
    if c <= 0:
        return

    # Try to set marker to the last item (most compatible)
    try:
        tl.markerPosition = tl.item(c - 1)
        return
    except:
        pass

    # Fallback: set to count
    try:
        tl.markerPosition = c
    except:
        pass


def _delete_timeline_from_index_strict(design: adsk.fusion.Design, start_count: int):
    """
    Delete timeline items with index >= start_count.

    In some Fusion builds, timeline.item(i) returns TimelineObject without deleteMe().
    We must delete its underlying entity/object instead.
    """
    if start_count is None:
        return

    tl = design.timeline
    end_index = int(tl.count) - 1
    if end_index < start_count:
        return

    for i in range(end_index, start_count - 1, -1):
        tlo = tl.item(i)

        # Try common underlying object access patterns
        deleted = False

        # 1) entity.deleteMe()
        try:
            ent = getattr(tlo, 'entity', None)
            if ent is not None and hasattr(ent, 'deleteMe'):
                ent.deleteMe()
                deleted = True
        except:
            pass

        if deleted:
            continue

        # 2) object.deleteMe()
        try:
            obj = getattr(tlo, 'object', None)
            if obj is not None and hasattr(obj, 'deleteMe'):
                obj.deleteMe()
                deleted = True
        except:
            pass

        if deleted:
            continue

        # 3) If neither is deletable, try to suppress / remove via timeline itself (rare support)
        # Some builds support deleting by manipulating timeline groups; not always available.
        # If not possible, we skip.
        # (You can log here if you want to know which item cannot be deleted.)


# -----------------------------
# Main entry
# -----------------------------
def run(context):
    ui = None
    app = None
    product = None
    design = None
    root = None
    components = None

    success_msg = 'Successfully create URDF file'
    msg = success_msg

    global _need_rollback, _start_marker_index, _start_timeline_count
    _need_rollback = True
    _start_marker_index = None
    _start_timeline_count = None

    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        title = 'Fusion2URDF'
        if not design:
            if ui:
                ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent
        components = design.allComponents

        # Capture "start state" BEFORE any modification
        _start_marker_index = _get_timeline_marker_index(design)
        try:
            _start_timeline_count = int(design.timeline.count)
        except:
            _start_timeline_count = None

        # --------------------
        # naming
        robot_name = root.name.split()[0]
        package_name = robot_name + '_description'

        save_dir = utils.file_dialog(ui)
        if not save_dir:
            if ui:
                ui.messageBox('Fusion2URDF was canceled', title)
            return

        # Ask cleanup
        cleanup_result = ui.messageBox(
            'Do you want to automatically clean up copied components after URDF generation?\n\n'
            'YES: Clean up (recommended)\n'
            'NO: Keep components',
            title,
            adsk.core.MessageBoxButtonTypes.YesNoButtonType,
            adsk.core.MessageBoxIconTypes.QuestionIconType
        )
        cleanup_components = (cleanup_result == adsk.core.DialogResults.DialogYes)

        if os.path.basename(save_dir) != package_name:
            save_dir = os.path.join(save_dir, package_name)

        os.makedirs(save_dir, exist_ok=True)
        package_dir = os.path.join(os.path.dirname(__file__), 'package')

        # --------------------
        # generate dictionaries
        joints_dict, msg = Joint.make_joints_dict(root, msg)
        if msg != success_msg:
            if ui:
                ui.messageBox(msg, title)
            return

        inertial_dict, msg = Link.make_inertial_dict(root, msg)
        if msg != success_msg:
            if ui:
                ui.messageBox(msg, title)
            return

        if 'base_link' not in inertial_dict:
            if ui:
                ui.messageBox('There is no base_link. Please set base_link and run again.', title)
            return

        links_xyz_dict = {}

        # --------------------
        # write URDF / xacro / launch
        Write.write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_display_launch(package_name, robot_name, save_dir)
        Write.write_gazebo_launch(package_name, robot_name, save_dir)
        Write.write_control_launch(package_name, robot_name, save_dir, joints_dict)
        Write.write_yaml(package_name, robot_name, save_dir, joints_dict)

        utils.copy_package(save_dir, package_dir)
        utils.update_cmakelists(save_dir, package_name)
        utils.update_package_xml(save_dir, package_name)

        # --------------------
        # STL export (THIS MODIFIES DESIGN)
        utils.copy_occs(root)
        utils.export_stl(design, save_dir, components)

        # --------------------
        # Plain URDF (xacro includes expanded into a single self-contained file)
        # Any consumer that cannot process xacro wants this form:
        # Isaac Sim, PyBullet, MuJoCo, Unity, Drake ...
        plain_dir = Path(save_dir) / f'{robot_name}_urdf'
        plain_dir.mkdir(parents=True, exist_ok=True)

        xacro_file = os.path.join(save_dir, 'urdf', f'{robot_name}.xacro')
        convert_xacro_to_urdf(
            xacro_file=xacro_file,
            output_urdf_path=str(plain_dir / f'{robot_name}.urdf')
        )

        plain_package_dir = plain_dir / package_name
        plain_package_dir.mkdir(parents=True, exist_ok=True)

        src_meshes = Path(save_dir) / 'meshes'
        dst_meshes = plain_package_dir / 'meshes'
        if src_meshes.exists():
            shutil.copytree(src_meshes, dst_meshes, dirs_exist_ok=True)

        # --------------------
        # cleanup copied components
        if cleanup_components:
            utils.cleanup_copied_components(root)
            msg += '\nCopied components cleaned up.'

        msg += f'\nFiles saved to:\n{save_dir}'
        if ui:
            ui.messageBox(msg, title)

        # Success: do NOT rollback or delete history
        _need_rollback = False

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
        # Keep _need_rollback = True so we rollback in finally

    finally:
        # --------------------
        # On ANY failure or early-return -> rollback and truncate timeline
        try:
            if app is None:
                app = adsk.core.Application.get()

            if _need_rollback and design is not None:
                # Move marker to end first (make items deletable)
                try:
                    _move_marker_to_end(design)
                except:
                    pass

                # Delete all timeline items created after the script started (truncate)
                try:
                    if _start_timeline_count is not None:
                        _delete_timeline_from_index_strict(design, _start_timeline_count)
                except:
                    if ui:
                        ui.messageBox(
                            'Failed to delete timeline items:\n{}'.format(traceback.format_exc()),
                            'Fusion2URDF'
                        )

                # Optional: move marker to end again (end is now truncated)
                try:
                    _move_marker_to_end(design)
                except:
                    pass

                # Optional: also move marker back to the original start marker for "start state" viewing
                # If you prefer to visually show the start state, uncomment below:
                # try:
                #     if _start_marker_index is not None:
                #         _try_set_timeline_marker(design, _start_marker_index)
                # except:
                #     pass

            try:
                adsk.doEvents()
            except:
                pass

        except:
            # Never throw from finally
            pass
