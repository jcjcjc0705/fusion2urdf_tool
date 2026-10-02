# -*- coding: utf-8 -*-
"""
GroupBodiesToLink
=================
選一組 body（或整個 occurrence），建立一個新的「頂層 component」當作 URDF 的 link，
把選到的 body 放進去。

兩種模式
--------
MODE = 'copy'（預設，建議）
    用 BRepBody.copyToComponent()。作用在 proxy body 上，只複製那一個實例，
    而且保證保留世界座標。原本的 All:1 樹原封不動留著當來源。
    → 完全不需要先跑 MakeAllUnique，共用 component 也不會出事。
    → 全部 link 收完之後，把 All:1 整個刪掉即可。

MODE = 'move'
    用 BRepBody.moveToComponent()，直接把 body 搬走。
    只有在每個部件都已經是 :1 的情況下才安全。

為什麼 link 一定要在頂層：fusion2urdf 的 copy_occs() 只走 root.occurrences
且只抓該層的 bRepBodies，巢狀 component 裡的 body 不會被匯出。

用法
----
1. 在 Fusion 裡先（可選）框選要歸在一起的 body
2. Utilities > ADD-INS > Scripts and Add-Ins > GroupBodiesToLink > Run
3. 「Link 名稱」填 leg_rr 之類的名字，可以繼續補選 / 取消選取 body
4. 按「建立 Link」
5. 勾著「建立後繼續建下一個」，對話框會自動再開，一輪做完所有 link
"""

import adsk.core
import adsk.fusion
import traceback
import re

# ----------------------------------------------------------------- 設定
MODE = 'copy'                  # 'copy'（建議） | 'move'
DEFAULT_LINK_NAME = 'base_link'
HIDE_SOURCE_DEFAULT = True     # copy 模式：複製後把來源 body 隱藏，方便追蹤進度
SKIP_HIDDEN_DEFAULT = True     # copy 模式：收集時跳過已隱藏（＝已被認領）的 body
DELETE_EMPTY_DEFAULT = False   # move 模式：刪掉被搬空的來源 component
GROUND_NEW_LINK = False
VERIFY = True                  # 驗證複製出來的 body 位置有沒有跑掉
TOL_CM = 0.01                  # 位置容差，0.01cm = 0.1mm

CMD_ID = 'sixleg_group_bodies_to_link'
CMD_NAME = u'Group Bodies to Link'
EVENT_ID = 'sixleg_group_bodies_to_link_reopen'

_handlers = []
_app = None
_ui = None
_preselected = []
_last_name = DEFAULT_LINK_NAME
_reopen = False
_event_registered = False


# ----------------------------------------------------------------- helpers
def _sanitize(name):
    s = re.sub(r'[^A-Za-z0-9_]', '_', name.strip())
    s = re.sub(r'_+', '_', s).strip('_')
    return s


def _existing_names(design):
    return set(c.name for c in design.allComponents)


def _token(ent):
    try:
        return ent.entityToken
    except Exception:
        return None


def _bbox_center(ent):
    try:
        bb = ent.boundingBox
        if not bb:
            return None
        return ((bb.minPoint.x + bb.maxPoint.x) / 2.0,
                (bb.minPoint.y + bb.maxPoint.y) / 2.0,
                (bb.minPoint.z + bb.maxPoint.z) / 2.0)
    except Exception:
        return None


def _dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


def _center(ent):
    """位置比對用的基準點。

    優先用質心：它是精確算出來的。boundingBox 不能用 —— proxy body 的外接框
    是把原生 AABB 的角點變換後再取外接框，對旋轉過的 occurrence 會比實際幾何
    寬鬆很多，跟複製出來（未旋轉）的 body 比會出現幾公分的假偏差。
    """
    try:
        pp = ent.getPhysicalProperties(
            adsk.fusion.CalculationAccuracy.LowCalculationAccuracy)
        if pp and pp.mass > 0:
            c = pp.centerOfMass
            return (c.x, c.y, c.z)
    except Exception:
        pass
    return _bbox_center(ent)


def _gather_bodies(sel_input, skip_hidden=False):
    """把選取內容展開成一串 BRepBody（選到 occurrence 會連子階層一起收）。

    這裡拿到的是 proxy body，帶著各自的 assembly context，
    所以同一個 component 的不同實例會被當成不同的 body，不會互相蓋掉。

    skip_hidden=True 會跳過燈泡關掉的 body —— 搭配「複製後隱藏來源」使用，
    最後直接選 All:1 就能把所有還沒被認領的 body 收成 base_link。
    """
    bodies = []
    seen = set()

    def add(b):
        if skip_hidden:
            try:
                if not b.isLightBulbOn:
                    return
            except Exception:
                pass
        t = _token(b)
        if t is None:
            bodies.append(b)
            return
        if t not in seen:
            seen.add(t)
            bodies.append(b)

    def walk_occ(occ):
        for i in range(occ.bRepBodies.count):
            add(occ.bRepBodies.item(i))
        for j in range(occ.childOccurrences.count):
            walk_occ(occ.childOccurrences.item(j))

    for i in range(sel_input.selectionCount):
        ent = sel_input.selection(i).entity
        body = adsk.fusion.BRepBody.cast(ent)
        if body:
            add(body)
            continue
        occ = adsk.fusion.Occurrence.cast(ent)
        if occ:
            walk_occ(occ)
            continue
    return bodies


def _copy_body_into(design, target_comp, target_occ, body):
    """把 body 複製進 target_comp，回傳 (新 body, 使用的方法)。

    copyToComponent() 對深層巢狀、component 又被共用的 proxy body 會回傳
    非 None 但實際上什麼都沒建立。所以這裡不看回傳值，改看「目標 component
    的 body 數有沒有真的增加」；沒增加就改用 TemporaryBRepManager 自己複製
    一份塞進去（proxy body 的幾何已經是世界座標，直接 add 位置就是對的）。
    """
    before_n = target_comp.bRepBodies.count

    try:
        body.copyToComponent(target_occ)
    except Exception:
        pass
    if target_comp.bRepBodies.count > before_n:
        return (target_comp.bRepBodies.item(target_comp.bRepBodies.count - 1),
                'copyToComponent')

    tbm = adsk.fusion.TemporaryBRepManager.get()
    temp = tbm.copy(body)
    if temp is None:
        return None, None

    is_parametric = False
    try:
        is_parametric = (design.designType ==
                         adsk.fusion.DesignTypes.ParametricDesignType)
    except Exception:
        pass

    if is_parametric:
        bf = target_comp.features.baseFeatures.add()
        bf.startEdit()
        nb = target_comp.bRepBodies.add(temp, bf)
        bf.finishEdit()
    else:
        nb = target_comp.bRepBodies.add(temp)
    return nb, 'tempBRep'


def _delete_emptied(sources):
    uniq = {}
    for occ in sources:
        if occ is None:
            continue
        t = _token(occ)
        if t and t not in uniq:
            uniq[t] = occ

    def depth(o):
        try:
            return o.fullPathName.count('+')
        except Exception:
            return 0

    removed = 0
    for occ in sorted(uniq.values(), key=depth, reverse=True):
        try:
            if occ.bRepBodies.count == 0 and occ.childOccurrences.count == 0:
                occ.deleteMe()
                removed += 1
        except Exception:
            pass
    return removed


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
            sel = inputs.itemById('selBodies')
            name = inputs.itemById('linkName').value
            args.areInputsValid = sel.selectionCount > 0 and bool(_sanitize(name))
        except Exception:
            args.areInputsValid = False


class _ExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        global _last_name, _reopen
        try:
            inputs = args.firingEvent.sender.commandInputs
            sel = inputs.itemById('selBodies')
            raw_name = inputs.itemById('linkName').value
            extra = inputs.itemById('extraOpt').value
            keep_open = inputs.itemById('keepOpen').value

            _last_name = raw_name
            _reopen = bool(keep_open)

            design = adsk.fusion.Design.cast(_app.activeProduct)
            root = design.rootComponent

            name = _sanitize(raw_name)
            if not name:
                _ui.messageBox(u'Link 名稱無效。')
                return
            if name in _existing_names(design):
                _ui.messageBox(u'已經有一個叫 "{}" 的 component 了，請換個名字。'.format(name))
                return

            skip_hidden_in = inputs.itemById('skipHidden')
            skip_hidden = bool(skip_hidden_in.value) if skip_hidden_in else False

            bodies = _gather_bodies(sel, skip_hidden)
            if not bodies:
                _ui.messageBox(u'選取的東西裡面沒有任何 body。'
                               u'\n（有勾「忽略已隱藏的 body」的話，'
                               u'代表選到的都已經被收進其他 link 了）')
                return

            sources = []
            for b in bodies:
                try:
                    sources.append(b.assemblyContext)
                except Exception:
                    sources.append(None)

            new_occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            new_occ.component.name = name

            done = 0
            failed = []
            misplaced = []
            methods = {'copyToComponent': 0, 'tempBRep': 0}
            target_comp = new_occ.component

            for b in bodies:
                bname = 'body'
                try:
                    bname = b.name
                except Exception:
                    pass

                before = _center(b) if VERIFY else None

                try:
                    if MODE == 'copy':
                        nb, method = _copy_body_into(
                            design, target_comp, new_occ, b)
                        ok = nb is not None
                        if ok:
                            methods[method] = methods.get(method, 0) + 1
                            if VERIFY and before:
                                after = _center(nb)
                                if after:
                                    d = _dist(before, after)
                                    if d > TOL_CM:
                                        misplaced.append((d, bname))
                            if extra:      # 隱藏來源 body
                                try:
                                    b.isLightBulbOn = False
                                except Exception:
                                    pass
                    else:
                        ok = b.moveToComponent(new_occ)

                    if ok:
                        done += 1
                    else:
                        failed.append(bname)
                except Exception as e:
                    failed.append(u'{} ({})'.format(bname, e))

            if GROUND_NEW_LINK:
                try:
                    new_occ.isGrounded = True
                except Exception:
                    pass

            removed = 0
            if MODE == 'move' and extra:
                removed = _delete_emptied(sources)

            verb = u'複製' if MODE == 'copy' else u'搬移'
            try:
                actual = target_comp.bRepBodies.count
            except Exception:
                actual = -1

            msg = [u'Link "{}" 建立完成（{} 模式）。'.format(name, MODE),
                   u'{} body：{} / {}'.format(verb, done, len(bodies)),
                   u'component 裡實際的 body 數：{}'.format(actual)]
            if MODE == 'copy':
                msg.append(u'  copyToComponent：{}　tempBRep 後援：{}'.format(
                    methods.get('copyToComponent', 0), methods.get('tempBRep', 0)))
                if actual >= 0 and actual != len(bodies):
                    msg.append(u'  ⚠️ 實際數量與選取數量不符，請回報這個訊息')
            if removed:
                msg.append(u'刪掉搬空的來源 component：{} 個'.format(removed))
            if raw_name != name:
                msg.append(u'（名稱已自動清理：{} -> {}）'.format(raw_name, name))
            if misplaced:
                misplaced.sort(reverse=True)          # 偏差最大的排最前面
                worst = misplaced[0][0] * 10.0
                msg.append(u'')
                msg.append(u'位置偏差超過 {:.2f} mm 的 body：{} 個'.format(
                    TOL_CM * 10.0, len(misplaced)))
                for d, bname in misplaced[:10]:
                    msg.append(u'  {:>8.3f} mm   {}'.format(d * 10.0, bname))
                if worst < 1.0:
                    msg.append(u'')
                    msg.append(u'最大偏差 {:.3f} mm —— 這個量級是 bounding box 的'.format(worst))
                    msg.append(u'近似誤差，不是真的跑位，可以忽略。')
                else:
                    msg.append(u'')
                    msg.append(u'⚠️ 最大偏差 {:.1f} mm，這不是數值誤差。'.format(worst))
                    msg.append(u'把 All:1 關燈只看這個 link，確認幾何有沒有飛掉。')
            if failed:
                msg.append(u'')
                msg.append(u'失敗 {} 個（前 20 個）：'.format(len(failed)))
                msg.extend(failed[:20])

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
            cmd.okButtonText = u'建立 Link'
            inputs = cmd.commandInputs

            sel = inputs.addSelectionInput(
                'selBodies', u'Body / Component',
                u'選取要歸進這個 link 的 body（選 component 會把它底下的 body 全收）')
            sel.addSelectionFilter('Bodies')
            sel.addSelectionFilter('Occurrences')
            sel.setSelectionLimits(1, 0)
            for ent in _preselected:
                try:
                    sel.addSelection(ent)
                except Exception:
                    pass

            inputs.addStringValueInput('linkName', u'Link 名稱', _last_name)

            if MODE == 'copy':
                inputs.addBoolValueInput('extraOpt', u'複製後隱藏來源 body',
                                         True, '', HIDE_SOURCE_DEFAULT)
                inputs.addBoolValueInput('skipHidden', u'忽略已隱藏的 body',
                                         True, '', SKIP_HIDDEN_DEFAULT)
            else:
                inputs.addBoolValueInput('extraOpt', u'刪掉被搬空的來源 component',
                                         True, '', DELETE_EMPTY_DEFAULT)

            inputs.addBoolValueInput('keepOpen', u'建立後繼續建下一個',
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
    global _app, _ui, _preselected, _last_name, _event_registered
    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface

        design = adsk.fusion.Design.cast(_app.activeProduct)
        if not design:
            _ui.messageBox(u'請先開啟一個 Fusion 設計檔。')
            return

        if MODE not in ('copy', 'move'):
            _ui.messageBox(u'MODE 只能是 copy 或 move。')
            return

        _last_name = DEFAULT_LINK_NAME

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
            CMD_ID, CMD_NAME, u'把選取的 body 收進一個新的頂層 link component')

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
