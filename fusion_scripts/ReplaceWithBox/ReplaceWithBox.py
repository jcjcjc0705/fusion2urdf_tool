# -*- coding: utf-8 -*-
"""
ReplaceWithBox
==============
把選取的 component / body 換成一個等大的外接方塊實體。

用途
----
廠商釋出的感測器模型（IMU、深度相機、光達、GPS…）常常是上百個曲面 patch，
縫不成實體 → 沒有質量，而且 fusion2urdf 的 visual / collision 共用同一個 STL，
那些碎片會直接變成 collision mesh，在 Isaac Sim 裡造成穿透與效能災難。

用一個外接方塊取代，同時解決三件事：
  * 變成實體 → 有體積、有質量、有合理的慣量
  * mesh 從數百個 patch 變成 12 個三角形
  * collision 的 convex decomposition 變成一步到位

做法
----
1. 算出選取幾何的軸對齊外接框（在該 component 的原生座標系）
2. 在同一個 component 裡建立一個等大的方塊
3. （可選）刪掉原本的 body

質量校正
--------
填「目標質量」後，結果視窗會告訴你這個方塊需要多少密度。
到 Modify > Physical Material，複製一個材質改密度，套到方塊上即可。
"""

import adsk.core
import adsk.fusion
import traceback

# ----------------------------------------------------------------- 設定
DELETE_SOURCE_DEFAULT = True    # 預設是否刪掉原本的 body
DEFAULT_TARGET_MASS_G = 0.0     # 目標質量（公克），0 = 不計算密度
BOX_NAME_SUFFIX = '_box'

CMD_ID = 'sixleg_replace_with_box'
CMD_NAME = u'Replace With Box'
EVENT_ID = 'sixleg_replace_with_box_reopen'

_handlers = []
_app = None
_ui = None
_preselected = []
_reopen = False
_event_registered = False


# ----------------------------------------------------------------- helpers
def _native(ent):
    """拿到 proxy 底下的原生物件。"""
    try:
        return ent.nativeObject or ent
    except Exception:
        return ent


def _collect_targets(sel_input):
    """回傳 {component: [原生 body, ...]}。

    用原生 body 是刻意的：方塊要建在 component 定義裡，這樣該 component
    的每一個實例都會同步換成方塊。
    """
    groups = {}

    def add_body(b):
        nb = _native(b)
        try:
            comp = nb.parentComponent
        except Exception:
            return
        key = comp.name
        if key not in groups:
            groups[key] = (comp, [])
        _, lst = groups[key]
        if all(x is not nb for x in lst):
            lst.append(nb)

    def walk_comp(comp):
        for i in range(comp.bRepBodies.count):
            add_body(comp.bRepBodies.item(i))
        for j in range(comp.occurrences.count):
            walk_comp(comp.occurrences.item(j).component)

    for i in range(sel_input.selectionCount):
        ent = sel_input.selection(i).entity

        body = adsk.fusion.BRepBody.cast(ent)
        if body:
            add_body(body)
            continue

        occ = adsk.fusion.Occurrence.cast(ent)
        if occ:
            walk_comp(occ.component)
            continue
    return groups


def _combined_bbox(bodies):
    """合併一組 body 的軸對齊外接框，回傳 (minPt, maxPt) tuple。"""
    lo = [None, None, None]
    hi = [None, None, None]
    for b in bodies:
        try:
            bb = b.boundingBox
        except Exception:
            continue
        if not bb:
            continue
        mn = (bb.minPoint.x, bb.minPoint.y, bb.minPoint.z)
        mx = (bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z)
        for k in range(3):
            lo[k] = mn[k] if lo[k] is None else min(lo[k], mn[k])
            hi[k] = mx[k] if hi[k] is None else max(hi[k], mx[k])
    if lo[0] is None:
        return None
    return (tuple(lo), tuple(hi))


def _make_box(design, comp, lo, hi, name):
    """在 comp 裡建立一個 lo~hi 的方塊實體，回傳新 body。"""
    dx = max(hi[0] - lo[0], 1e-6)
    dy = max(hi[1] - lo[1], 1e-6)
    dz = max(hi[2] - lo[2], 1e-6)

    center = adsk.core.Point3D.create((lo[0] + hi[0]) / 2.0,
                                      (lo[1] + hi[1]) / 2.0,
                                      (lo[2] + hi[2]) / 2.0)
    obb = adsk.core.OrientedBoundingBox3D.create(
        center,
        adsk.core.Vector3D.create(1, 0, 0),
        adsk.core.Vector3D.create(0, 1, 0),
        dx, dy, dz)

    tbm = adsk.fusion.TemporaryBRepManager.get()
    temp = tbm.createBox(obb)

    is_parametric = False
    try:
        is_parametric = (design.designType ==
                         adsk.fusion.DesignTypes.ParametricDesignType)
    except Exception:
        pass

    if is_parametric:
        bf = comp.features.baseFeatures.add()
        bf.startEdit()
        new_body = comp.bRepBodies.add(temp, bf)
        bf.finishEdit()
    else:
        new_body = comp.bRepBodies.add(temp)

    try:
        new_body.name = name
    except Exception:
        pass
    return new_body, (dx, dy, dz)


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


# ----------------------------------------------------------------- handlers
class _ValidateHandler(adsk.core.ValidateInputsEventHandler):
    def notify(self, args):
        try:
            inputs = args.firingEvent.sender.commandInputs
            args.areInputsValid = inputs.itemById('sel').selectionCount > 0
        except Exception:
            args.areInputsValid = False


class _ExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        global _reopen
        try:
            inputs = args.firingEvent.sender.commandInputs
            sel = inputs.itemById('sel')
            del_source = inputs.itemById('delSource').value
            target_g = inputs.itemById('targetMass').value
            _reopen = bool(inputs.itemById('keepOpen').value)

            design = adsk.fusion.Design.cast(_app.activeProduct)

            groups = _collect_targets(sel)
            if not groups:
                _ui.messageBox(u'選取的東西裡面沒有任何 body。')
                return

            lines = []
            total_boxes = 0
            total_removed = 0
            total_vol_cm3 = 0.0

            for key in sorted(groups.keys()):
                comp, bodies = groups[key]
                bb = _combined_bbox(bodies)
                if not bb:
                    lines.append(u'{}：算不出外接框，跳過'.format(comp.name))
                    continue
                lo, hi = bb

                try:
                    new_body, dims = _make_box(
                        design, comp, lo, hi, comp.name + BOX_NAME_SUFFIX)
                except Exception as e:
                    lines.append(u'{}：建立方塊失敗（{}）'.format(comp.name, e))
                    continue

                total_boxes += 1
                dx, dy, dz = dims
                vol_cm3 = dx * dy * dz          # Fusion 內部單位就是 cm
                total_vol_cm3 += vol_cm3

                removed = 0
                if del_source:
                    for b in bodies:
                        try:
                            b.deleteMe()
                            removed += 1
                        except Exception:
                            pass
                    total_removed += removed

                lines.append(
                    u'{}：{:.1f} x {:.1f} x {:.1f} mm，'
                    u'體積 {:.2f} cm3，刪除 {} 個 body'.format(
                        comp.name, dx * 10, dy * 10, dz * 10, vol_cm3, removed))

            msg = [u'建立方塊：{} 個'.format(total_boxes),
                   u'刪除原始 body：{} 個'.format(total_removed),
                   u'']
            msg.extend(lines)

            if target_g and target_g > 0 and total_vol_cm3 > 0:
                dens_g_cm3 = target_g / total_vol_cm3
                msg += [u'',
                        u'目標質量 {:.1f} g / 總體積 {:.2f} cm3'.format(
                            target_g, total_vol_cm3),
                        u'→ 需要的密度：{:.4f} g/cm^3'.format(dens_g_cm3),
                        u'   （= {:.1f} kg/m^3，看你的欄位是哪個單位）'.format(
                            dens_g_cm3 * 1000.0),
                        u'',
                        u'設定方式：選取方塊 → Modify > Physical Material →',
                        u'把材質拖到方塊上 → 在 In This Design 區對它右鍵 Edit →',
                        u'Physical 頁籤 → Density 改成上面的數值。']

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
            cmd.okButtonText = u'建立方塊'
            inputs = cmd.commandInputs

            sel = inputs.addSelectionInput(
                'sel', u'Component / Body',
                u'選取要換成方塊的感測器 component（或個別 body）')
            sel.addSelectionFilter('Bodies')
            sel.addSelectionFilter('Occurrences')
            sel.setSelectionLimits(1, 0)
            for ent in _preselected:
                try:
                    sel.addSelection(ent)
                except Exception:
                    pass

            inputs.addBoolValueInput('delSource', u'刪掉原本的 body',
                                     True, '', DELETE_SOURCE_DEFAULT)
            inputs.addValueInput(
                'targetMass', u'目標質量 (g，0 = 不算)',
                '', adsk.core.ValueInput.createByReal(DEFAULT_TARGET_MASS_G))
            inputs.addBoolValueInput('keepOpen', u'建立後繼續做下一個',
                                     True, '', True)

            on_exec = _ExecuteHandler()
            cmd.execute.add(on_exec)
            _handlers.append(on_exec)

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
            CMD_ID, CMD_NAME, u'把選取的幾何換成等大的外接方塊實體')

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
