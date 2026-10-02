# -*- coding: utf-8 -*-
"""
SetMassMaterial
===============
給選取的 body 指定材質與密度。兩種模式：

  設材質  —— 選一個材質套上去，密度欄會自動帶入該材質的密度。
             不動它就用材質原本的密度；改了就以你填的為準。

  設重量  —— 填目標總質量，腳本自動算出需要的密度。

為什麼要在 Fusion 這邊做而不是改 URDF
------------------------------------
fusion2urdf 的慣量張量是從「幾何 × 密度」算出來的（Link.py 的
getXYZMomentsOfInertia）。只在 URDF 裡手改 <mass>，<inertia> 還是錯的。
對步態模擬來說慣量比質量更難 debug，所以密度要在這裡設對。

密度怎麼保證準確
----------------
Fusion 材質的密度屬性內部單位不明確，所以不用猜：
先把材質套上去，量測實際的 mass / volume，再依比例縮放屬性值。
密度與質量是嚴格線性關係，所以一次修正就精確命中。

每次執行都會在文件材質庫裡建立一個新材質（不會改到原本的材質庫）。
"""

import adsk.core
import adsk.fusion
import traceback

# ----------------------------------------------------------------- 設定
DEFAULT_LIBRARY_HINT = 'Fusion'   # 預設選哪個材質庫（名稱包含這個字串）
DOC_LIB_LABEL = u'★ 本設計已有的材質'
MAX_MATERIALS = 400               # 單一材質庫最多列幾個
PREVIEW_BODY_LIMIT = 250          # 超過這個數量就不做即時量測
DENSITY_PROP_IDS = ('structural_Density', 'PhysMaterial_Density', 'Density')

CMD_ID = 'sixleg_set_mass_material'
CMD_NAME = u'Set Mass / Material'
EVENT_ID = 'sixleg_set_mass_material_reopen'

_handlers = []
_app = None
_ui = None
_preselected = []
_reopen = False
_event_registered = False
_prefilled_density = 0.0


# ----------------------------------------------------------------- helpers
def _walk_bodies(occ, out):
    stack = [occ]
    guard = 0
    while stack and guard < 200000:
        guard += 1
        cur = stack.pop()
        try:
            for i in range(cur.bRepBodies.count):
                out.append(cur.bRepBodies.item(i))
        except Exception:
            pass
        try:
            for j in range(cur.childOccurrences.count):
                stack.append(cur.childOccurrences.item(j))
        except Exception:
            pass


def _gather_bodies(sel_input):
    bodies = []
    seen = set()

    def add(b):
        try:
            t = b.entityToken
        except Exception:
            t = None
        if t is None or t not in seen:
            if t:
                seen.add(t)
            bodies.append(b)

    for i in range(sel_input.selectionCount):
        ent = sel_input.selection(i).entity
        body = adsk.fusion.BRepBody.cast(ent)
        if body:
            add(body)
            continue
        occ = adsk.fusion.Occurrence.cast(ent)
        if occ:
            tmp = []
            _walk_bodies(occ, tmp)
            for b in tmp:
                add(b)
    return bodies


def _measure(bodies, accuracy=None):
    """回傳 (總質量 kg, 總體積 cm3, 總表面積 cm2)。整個選取加總。"""
    if accuracy is None:
        accuracy = adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy
    mass = 0.0
    vol = 0.0
    area = 0.0
    for b in bodies:
        try:
            pp = b.getPhysicalProperties(accuracy)
            if pp:
                mass += pp.mass
                vol += pp.volume
                try:
                    area += pp.area
                except Exception:
                    pass
        except Exception:
            pass
    return mass, vol, area


def _update_info(inputs):
    """更新「目前選取」的即時摘要。"""
    info = inputs.itemById('info')
    sel = inputs.itemById('sel')
    if not info or not sel:
        return
    try:
        bodies = _gather_bodies(sel)
    except Exception:
        bodies = []

    if not bodies:
        info.formattedText = u'尚未選取'
        return
    if len(bodies) > PREVIEW_BODY_LIMIT:
        info.formattedText = u'選取 {} 個 body（數量過多，略過即時量測）'.format(
            len(bodies))
        return

    mass, vol, area = _measure(
        bodies, adsk.fusion.CalculationAccuracy.LowCalculationAccuracy)
    d = (mass / vol) * 1000.0 if vol > 0 else 0.0
    info.formattedText = (
        u'選取 {} 個 body　體積 {:.1f} cm3　表面積 {:.1f} cm2<br>'
        u'目前：{:.1f} g（{:.3f} kg）　密度 {:.4f} g/cm^3'.format(
            len(bodies), vol, area, mass * 1000.0, mass, d))


def _density_prop(material):
    """找出材質的密度屬性（FloatProperty），找不到回 None。"""
    try:
        props = material.materialProperties
    except Exception:
        return None

    for pid in DENSITY_PROP_IDS:
        try:
            p = props.itemById(pid)
            if p:
                fp = adsk.core.FloatProperty.cast(p)
                if fp:
                    return fp
        except Exception:
            pass

    try:
        for i in range(props.count):
            p = props.item(i)
            nm = (p.name or '') + (p.id or '')
            if 'ensity' in nm:
                fp = adsk.core.FloatProperty.cast(p)
                if fp:
                    return fp
    except Exception:
        pass
    return None


def _normalize_density(v):
    """材質屬性值換算成 g/cm^3 的粗估（只用於預填，實際值靠量測校正）。"""
    if v is None or v <= 0:
        return 0.0
    if v > 100.0:          # kg/m^3
        return v / 1000.0
    if v < 0.05:           # kg/cm^3
        return v * 1000.0
    return v               # 已經是 g/cm^3


def _libraries(design):
    """[(顯示名稱, Materials 物件), ...]，第一個是本設計的材質庫。"""
    out = [(DOC_LIB_LABEL, design.materials)]
    try:
        libs = _app.materialLibraries
        for i in range(libs.count):
            lib = libs.item(i)
            try:
                if lib.libraryType != adsk.core.LibraryTypes.MaterialLibraryType:
                    continue
            except Exception:
                pass
            try:
                out.append((lib.name, lib.materials))
            except Exception:
                pass
    except Exception:
        pass
    return out


def _unique_material_name(design, base):
    existing = set()
    try:
        for i in range(design.materials.count):
            existing.add(design.materials.item(i).name)
    except Exception:
        pass
    if base not in existing:
        return base
    i = 2
    while '{}_{}'.format(base, i) in existing:
        i += 1
    return '{}_{}'.format(base, i)


def _shutdown():
    global _event_registered
    try:
        if _event_registered:
            _app.unregisterCustomEvent(EVENT_ID)
            _event_registered = False
    except Exception:
        pass
    try:
        adsk.terminate()
    except Exception:
        pass


def _selected_material(inputs):
    """依目前的下拉選單取得 Material 物件。"""
    design = adsk.fusion.Design.cast(_app.activeProduct)
    lib_drop = inputs.itemById('lib')
    mat_drop = inputs.itemById('mat')
    if not lib_drop or not mat_drop:
        return None

    libs = _libraries(design)
    li = lib_drop.selectedItem
    mi = mat_drop.selectedItem
    if not li or not mi:
        return None

    materials = None
    for nm, mats in libs:
        if nm == li.name:
            materials = mats
            break
    if materials is None:
        return None

    try:
        return materials.itemByName(mi.name)
    except Exception:
        return None


def _fill_materials(inputs):
    """依材質庫下拉選單重新填入材質清單，並預填密度。"""
    global _prefilled_density
    design = adsk.fusion.Design.cast(_app.activeProduct)
    lib_drop = inputs.itemById('lib')
    mat_drop = inputs.itemById('mat')
    if not lib_drop or not mat_drop:
        return

    libs = _libraries(design)
    li = lib_drop.selectedItem
    materials = None
    for nm, mats in libs:
        if li and nm == li.name:
            materials = mats
            break
    mat_drop.listItems.clear()
    if materials is None:
        return

    names = []
    try:
        for i in range(min(materials.count, MAX_MATERIALS)):
            names.append(materials.item(i).name)
    except Exception:
        pass
    names.sort()
    for i, nm in enumerate(names):
        mat_drop.listItems.add(nm, i == 0)

    _update_density_prefill(inputs)


def _update_density_prefill(inputs):
    global _prefilled_density
    mat = _selected_material(inputs)
    dens_in = inputs.itemById('density')
    if not dens_in:
        return
    d = 0.0
    if mat:
        fp = _density_prop(mat)
        if fp:
            try:
                d = _normalize_density(fp.value)
            except Exception:
                d = 0.0
    _prefilled_density = d
    try:
        dens_in.value = d
    except Exception:
        pass


# ----------------------------------------------------------------- handlers
class _ValidateHandler(adsk.core.ValidateInputsEventHandler):
    def notify(self, args):
        try:
            inputs = args.firingEvent.sender.commandInputs
            sel = inputs.itemById('sel')
            mode = inputs.itemById('mode').selectedItem.name
            ok = sel.selectionCount > 0
            if ok and mode.startswith(u'設重量'):
                ok = inputs.itemById('mass').value > 0
            args.areInputsValid = ok
        except Exception:
            args.areInputsValid = False


class _InputChangedHandler(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        try:
            inputs = args.inputs
            cid = args.input.id
            if cid == 'sel':
                _update_info(inputs)
            elif cid == 'lib':
                _fill_materials(inputs)
            elif cid == 'mat':
                _update_density_prefill(inputs)
            elif cid == 'mode':
                is_mass = inputs.itemById('mode').selectedItem.name.startswith(u'設重量')
                inputs.itemById('density').isVisible = not is_mass
                inputs.itemById('mass').isVisible = is_mass
        except Exception:
            pass


class _ExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        global _reopen
        try:
            inputs = args.firingEvent.sender.commandInputs
            sel = inputs.itemById('sel')
            mode = inputs.itemById('mode').selectedItem.name
            want_density = inputs.itemById('density').value
            want_mass_g = inputs.itemById('mass').value
            _reopen = bool(inputs.itemById('keepOpen').value)

            design = adsk.fusion.Design.cast(_app.activeProduct)

            bodies = _gather_bodies(sel)
            if not bodies:
                _ui.messageBox(u'選取的東西裡面沒有任何 body。')
                return

            base_mat = _selected_material(inputs)
            if base_mat is None:
                _ui.messageBox(u'取不到選定的材質。')
                return

            is_mass_mode = mode.startswith(u'設重量')

            # ---- 建立文件內的材質副本並套用
            label = u'{}_{}g'.format(base_mat.name, int(round(want_mass_g))) \
                if is_mass_mode else u'{}_{:.4f}'.format(base_mat.name, want_density)
            new_name = _unique_material_name(design, label)

            try:
                new_mat = design.materials.addByCopy(base_mat, new_name)
            except Exception as e:
                _ui.messageBox(u'無法複製材質：{}'.format(e))
                return

            applied = 0
            for b in bodies:
                try:
                    b.material = new_mat
                    applied += 1
                except Exception:
                    pass

            mass0, vol0, _a0 = _measure(bodies)
            if vol0 <= 0:
                _ui.messageBox(u'選取的 body 體積為 0，無法設定密度。\n'
                               u'（曲面體沒有體積，請先換成實體）')
                return

            d0 = (mass0 / vol0) * 1000.0      # g/cm^3

            fp = _density_prop(new_mat)
            scaled = False
            note = u''

            if is_mass_mode:
                target_mass_kg = want_mass_g / 1000.0
                if fp and mass0 > 0:
                    try:
                        fp.value = fp.value * (target_mass_kg / mass0)
                        scaled = True
                    except Exception as e:
                        note = u'密度屬性寫入失敗：{}'.format(e)
                elif not fp:
                    note = u'找不到材質的密度屬性，無法調整。'
            else:
                changed = abs(want_density - _prefilled_density) > 1e-6
                if changed:
                    if fp and d0 > 0:
                        try:
                            fp.value = fp.value * (want_density / d0)
                            scaled = True
                        except Exception as e:
                            note = u'密度屬性寫入失敗：{}'.format(e)
                    elif not fp:
                        note = u'找不到材質的密度屬性，無法微調。'

            mass1, vol1, _a1 = _measure(bodies)
            d1 = (mass1 / vol1) * 1000.0 if vol1 > 0 else 0.0

            msg = [u'套用材質：{}'.format(new_name),
                   u'body：{} / {}　體積 {:.2f} cm3'.format(
                       applied, len(bodies), vol1),
                   u'',
                   u'密度：{:.4f} g/cm^3'.format(d1),
                   u'質量：{:.1f} g　（{:.3f} kg）'.format(mass1 * 1000.0, mass1)]
            if scaled:
                msg.append(u'')
                msg.append(u'（已由 {:.4f} 校正到 {:.4f} g/cm^3）'.format(d0, d1))
            elif not is_mass_mode:
                msg.append(u'')
                msg.append(u'（用材質原本的密度，未調整）')
            if note:
                msg += [u'', u'⚠️ ' + note]

            _ui.messageBox(u'\n'.join(msg), CMD_NAME)

        except Exception:
            _reopen = False
            if _ui:
                _ui.messageBox(u'Failed:\n{}'.format(traceback.format_exc()))


class _DestroyHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        global _reopen
        try:
            if _reopen:
                _reopen = False
                _app.fireCustomEvent(EVENT_ID)
            else:
                _shutdown()
        except Exception:
            _shutdown()


class _ReopenHandler(adsk.core.CustomEventHandler):
    def notify(self, args):
        try:
            cmd_def = _ui.commandDefinitions.itemById(CMD_ID)
            if cmd_def:
                cmd_def.execute()
            else:
                _shutdown()
        except Exception:
            _shutdown()


class _CreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = args.command
            cmd.isRepeatable = False
            cmd.okButtonText = u'套用'
            inputs = cmd.commandInputs
            design = adsk.fusion.Design.cast(_app.activeProduct)

            sel = inputs.addSelectionInput(
                'sel', u'Body / Component', u'選取要設定的 body 或 link')
            sel.addSelectionFilter('Bodies')
            sel.addSelectionFilter('Occurrences')
            sel.setSelectionLimits(1, 0)
            for ent in _preselected:
                try:
                    sel.addSelection(ent)
                except Exception:
                    pass

            mode = inputs.addDropDownCommandInput(
                'mode', u'模式', adsk.core.DropDownStyles.TextListDropDownStyle)
            mode.listItems.add(u'設材質（可微調密度）', True)
            mode.listItems.add(u'設重量（自動算密度）', False)

            lib_drop = inputs.addDropDownCommandInput(
                'lib', u'材質庫', adsk.core.DropDownStyles.TextListDropDownStyle)
            libs = _libraries(design)
            default_idx = 0
            for i, (nm, _m) in enumerate(libs):
                if DEFAULT_LIBRARY_HINT in nm:
                    default_idx = i
                    break
            for i, (nm, _m) in enumerate(libs):
                lib_drop.listItems.add(nm, i == default_idx)

            inputs.addDropDownCommandInput(
                'mat', u'材質', adsk.core.DropDownStyles.TextListDropDownStyle)

            d = inputs.addValueInput(
                'density', u'密度 (g/cm^3)', '',
                adsk.core.ValueInput.createByReal(0.0))
            m = inputs.addValueInput(
                'mass', u'目標總質量 (g，選取全部加總)', '',
                adsk.core.ValueInput.createByReal(0.0))
            m.isVisible = False

            info = inputs.addTextBoxCommandInput(
                'info', u'目前選取', u'尚未選取', 2, True)
            info.isFullWidth = True

            inputs.addBoolValueInput('keepOpen', u'套用後繼續設下一組',
                                     True, '', True)

            _fill_materials(inputs)
            _update_info(inputs)

            on_exec = _ExecuteHandler()
            cmd.execute.add(on_exec)
            _handlers.append(on_exec)

            on_changed = _InputChangedHandler()
            cmd.inputChanged.add(on_changed)
            _handlers.append(on_changed)

            on_valid = _ValidateHandler()
            cmd.validateInputs.add(on_valid)
            _handlers.append(on_valid)

            on_destroy = _DestroyHandler()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except Exception:
            if _ui:
                _ui.messageBox(u'Failed:\n{}'.format(traceback.format_exc()))


# ----------------------------------------------------------------- entry
def run(context):
    global _app, _ui, _preselected, _event_registered
    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface

        design = adsk.fusion.Design.cast(_app.activeProduct)
        if not design:
            _ui.messageBox(u'請先開啟一個 Fusion 設計檔。')
            return

        _preselected = []
        try:
            for i in range(_ui.activeSelections.count):
                _preselected.append(_ui.activeSelections.item(i).entity)
        except Exception:
            pass

        try:
            _app.unregisterCustomEvent(EVENT_ID)
        except Exception:
            pass
        custom_event = _app.registerCustomEvent(EVENT_ID)
        _event_registered = True
        on_reopen = _ReopenHandler()
        custom_event.add(on_reopen)
        _handlers.append(on_reopen)

        cmd_def = _ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()
        cmd_def = _ui.commandDefinitions.addButtonDefinition(
            CMD_ID, CMD_NAME, u'給選取的 body 指定材質與密度，或指定目標重量')

        on_created = _CreatedHandler()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)

        cmd_def.execute()
        adsk.autoTerminate(False)

    except Exception:
        if _ui:
            _ui.messageBox(u'Failed:\n{}'.format(traceback.format_exc()))


def stop(context):
    _shutdown()
