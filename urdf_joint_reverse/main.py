import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


SUPPORTED_JOINT_TYPES = {"revolute", "continuous", "prismatic"}


def format_xyz(values):
    normalized = [0.0 if abs(value) < 1e-12 else value for value in values]
    return " ".join(f"{value:.6g}" for value in normalized)


def parse_xyz(text):
    parts = text.split()
    if len(parts) != 3:
        raise ValueError(f"Invalid xyz format: {text!r}")
    return [float(part) for part in parts]


def get_joint_name(joint, fallback_index):
    return joint.get("name") or f"<unnamed_{fallback_index}>"


def reverse_axis(axis_element):
    current_xyz = axis_element.get("xyz", "1 0 0")
    values = parse_xyz(current_xyz)
    axis_element.set("xyz", format_xyz([-value for value in values]))


def reverse_limit(limit_element):
    lower = limit_element.get("lower")
    upper = limit_element.get("upper")
    if lower is None or upper is None:
        return False

    lower_value = float(lower)
    upper_value = float(upper)
    limit_element.set("lower", f"{-upper_value:.6g}")
    limit_element.set("upper", f"{-lower_value:.6g}")
    return True


def reverse_mimic(mimic_element):
    multiplier = mimic_element.get("multiplier")
    if multiplier is None:
        mimic_element.set("multiplier", "-1")
        return

    mimic_element.set("multiplier", f"{-float(multiplier):.6g}")


def reverse_joint(joint):
    joint_type = joint.get("type", "").strip()
    if joint_type not in SUPPORTED_JOINT_TYPES:
        return False, f"joint type '{joint_type}' is not supported"

    axis_element = joint.find("axis")
    if axis_element is None:
        axis_element = ET.SubElement(joint, "axis")
        axis_element.set("xyz", "1 0 0")
    reverse_axis(axis_element)

    limit_updated = False
    if joint_type in {"revolute", "prismatic"}:
        limit_element = joint.find("limit")
        if limit_element is not None:
            limit_updated = reverse_limit(limit_element)

    mimic_element = joint.find("mimic")
    if mimic_element is not None:
        reverse_mimic(mimic_element)

    message = "axis reversed"
    if limit_updated:
        message += ", limit swapped and negated"
    if mimic_element is not None:
        message += ", mimic multiplier updated"
    return True, message


def save_tree(tree, target_path):
    ET.indent(tree, space="  ")
    tree.write(target_path, encoding="utf-8", xml_declaration=True)


def list_joints(joints):
    print("\nAvailable joints:")
    for index, joint in enumerate(joints, start=1):
        name = get_joint_name(joint, index)
        joint_type = joint.get("type", "")
        axis = joint.find("axis")
        axis_xyz = axis.get("xyz", "1 0 0") if axis is not None else "1 0 0 (default)"
        print(f"  [{index}] {name}  type={joint_type}  axis={axis_xyz}")


def find_joint(selection, joints):
    if selection.isdigit():
        index = int(selection)
        if 1 <= index <= len(joints):
            return joints[index - 1]
        return None

    for index, joint in enumerate(joints, start=1):
        if get_joint_name(joint, index) == selection:
            return joint
    return None


def interactive_reverse(target_path):
    try:
        tree = ET.parse(target_path)
    except ET.ParseError as error:
        print(f"Failed to parse URDF: {error}")
        return 1

    root = tree.getroot()
    joints = root.findall("joint")
    if not joints:
        print("No <joint> elements found.")
        return 1

    print(f"Loaded: {target_path}")
    print("Type a joint index or name and press Enter to reverse it. Type q to quit.")

    while True:
        list_joints(joints)
        selection = input("\nSelect joint index/name, or q to quit: ").strip()

        if selection.lower() == "q":
            print("Bye.")
            return 0

        if not selection:
            print("Please enter a joint index or name.")
            continue

        joint = find_joint(selection, joints)
        if joint is None:
            print(f"Joint not found: {selection}")
            continue

        joint_name = joint.get("name", "<unnamed>")
        changed, message = reverse_joint(joint)
        if not changed:
            print(f"{joint_name}: {message}")
            continue

        save_tree(tree, target_path)
        print(f"{joint_name}: {message}. Changes saved back to the original URDF.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Interactively reverse URDF joint axes in place."
    )
    parser.add_argument("urdf_path", help="Path to the target URDF file")
    return parser.parse_args()


def main():
    args = parse_args()
    target_path = Path(args.urdf_path).expanduser()

    if not target_path.exists():
        print(f"File not found: {target_path}")
        return 1

    if target_path.suffix.lower() != ".urdf":
        print(f"Warning: target suffix is not .urdf, but processing will continue: {target_path}")

    return interactive_reverse(target_path)


if __name__ == "__main__":
    sys.exit(main())
