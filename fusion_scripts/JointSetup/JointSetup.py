# -*- coding: utf-8 -*-
"""
JointSetup
==========
Joint 的一站式設定：解鎖 limit、擺零位姿態、設定 limit，
以及列出匯出後要在 URDF 手改的軸向。

開關
----
DO_FREE    解除所有 limit（獨占，其他階段不跑），用來手動擺姿勢
DO_POSE    把每個 joint 驅動到表裡「驅動到」欄的角度
DO_REVERSE 列出標記 True 的 joint 要在 URDF 改哪一行
DO_LIMITS  依表裡的 limit 設定上下限
DO_DUMP_POSE 把目前姿態輸出成姿態腳本（唯讀，會問檔名與資料夾）

同一次開多個時，順序固定是：
    FREE（獨占） → REVERSE → POSE → LIMITS
LIMITS 一定要在 POSE 之後：POSE 為了能自由驅動會先解除所有 limit，
由 LIMITS 收尾重新啟用。反過來的話 limit 會被 POSE 關掉。

座標系
------
Fusion 的 joint 零位 = 建立 joint 當下的姿態，沒有 API 能事後更改。
URDF 的零位 = 匯出當下的 CAD 姿態。

兩者的差距由匯出器吸收 —— fusion2urdf 的 Joint.py 已改成寫入
    URDF limit = Fusion 絕對 limit − 當下的 rotationValue
所以只要匯出時姿態停在零位角，URDF 拿到的就是表裡填的值。

⚠️ 匯出前務必跑一次 DRY_RUN，確認「現在角 = 零位角」且沒有不符警告。
"""

import adsk.core
import adsk.fusion
import traceback
import math
import re

# ----------------------------------------------------------------- 開關
DRY_RUN = False

DO_FREE    = False    # 解除所有 limit
DO_POSE    = True     # 驅動到下表的「零位角」
DO_REVERSE = True     # 列出標記 True 的 joint 要在 URDF 改哪一行
DO_LIMITS  = True     # 設定 limit
DO_DUMP_POSE = False   # 把「目前姿態」輸出成姿態腳本（會問檔名與資料夾）

# ----------------------------------------------------------------- joint 表
#  名稱                  零位角  limit (下,上)   反轉軸向  驅動到
#                         (度)   None=continuous            (度)
#
#  「零位角」＝ URDF 的 0 度在 Fusion 角度裡是幾度。固定的參考點，
#     DO_LIMITS 和 DO_DUMP_POSE 都拿它當基準，不要為了擺姿勢去改它。
#     目前對應「手腳張開攤平」的姿態。
#
#  「limit」的基準是零位角，也就是 URDF 會看到的範圍。
#     Fusion UI 顯示的是「零位角 + limit」，看起來不對稱是正常的。
#     例：leg_rf1_rf2 零位角 45、limit -110~110
#         → Fusion UI 顯示 -65 ~ +155 → URDF 寫入 -110 ~ +110
#
#  「驅動到」＝ DO_POSE 要把關節擺到哪（Fusion 角度）。
#     匯出前要跟零位角一致（攤平）。
#     要 dump 站立姿勢時全部填 0（原始組裝姿態就是站立），
#     dump 完再改回跟零位角一致。
JOINTS = [
    # ---- 接在 base_link 上（髖橫向 / 肩橫向）
    ('base_leg_rf',            0,    (-45,   45),   False,    0),
    ('base_leg_rr',            0,    (-60,   60),   False,    0),
    ('base_leg_lf',            0,    (-45,   45),   False,    0),
    ('base_leg_lr',            0,    (-60,   60),   False,    0),
    ('base_hand_r',            0,    (-60,   60),   False,    0),
    ('base_hand_l',            0,    (-60,   60),   False,    0),

    # ---- 右前腳
    ('leg_rf1_rf2',           45,    (-110, 110),   False,   45),
    ('leg_rf2_rf3',           45,    (-110, 110),   False,   45),
    ('leg_rf_wheel',           0,    None,          False,    0),

    # ---- 右後腳
    ('leg_rr1_rr2',           45,    (-110, 110),   False,   45),
    ('leg_rr2_rr3',           45,    (-110, 110),   False,   45),
    ('leg_rr_wheel',           0,    None,          False,    0),

    # ---- 左前腳
    ('leg_lf1_lf2',           45,    (-110, 110),   False,   45),
    ('leg_lf2_lf3',           45,    (-110, 110),   False,   45),
    ('leg_lf_wheel',           0,    None,          False,    0),

    # ---- 左後腳（leg_lr1_lr2 的旋轉軸跟其他三隻相反，所以零位角是 -45）
    ('leg_lr1_lr2',          -45,    (-110, 110),   True,   -45),
    ('leg_lr2_lr3',           45,    (-110, 110),   False,   45),
    ('leg_lr_wheel',           0,    None,          False,    0),

    # ---- 右手
    ('hand_r1_r2',            30,    (-110, 110),   False,   30),
    ('hand_r2_r3',            30,    (-110, 110),   True,    30),
    ('hand_r3_gripper_r1',     0,    (-110, 110),   True,     0),
    ('gripper_r1_r2',          0,    (-180, 180),   False,    0),

    # ---- 左手
    ('hand_l1_l2',            30,    (-110, 110),   False,   30),
    ('hand_l2_l3',            30,    (-110, 110),   True,    30),
    ('hand_l3_gripper_l1',     0,    (-110, 110),   False,    0),
    ('gripper_l1_l2',          0,    (-180, 180),   False,    0),
]

# JOINTS = [
#     # ---- 接在 base_link 上（髖橫向 / 肩橫向）
#     ('base_leg_rf',            0,    (-45,   45),   False,    0),
#     ('base_leg_rr',            0,    (-60,   60),   False,    0),
#     ('base_leg_lf',            0,    (-45,   45),   False,    0),
#     ('base_leg_lr',            0,    (-60,   60),   False,    0),
#     ('base_hand_r',            0,    (-60,   60),   False,    0),
#     ('base_hand_l',            0,    (-60,   60),   False,    0),

#     # ---- 右前腳
#     ('leg_rf1_rf2',           45,    (-110, 110),   False,    0),
#     ('leg_rf2_rf3',           45,    (-110, 110),   False,    0),
#     ('leg_rf_wheel',           0,    None,          False,    0),

#     # ---- 右後腳
#     ('leg_rr1_rr2',           45,    (-110, 110),   False,    0),
#     ('leg_rr2_rr3',           45,    (-110, 110),   False,    0),
#     ('leg_rr_wheel',           0,    None,          False,    0),

#     # ---- 左前腳
#     ('leg_lf1_lf2',           45,    (-110, 110),   False,    0),
#     ('leg_lf2_lf3',           45,    (-110, 110),   False,    0),
#     ('leg_lf_wheel',           0,    None,          False,    0),

#     # ---- 左後腳（leg_lr1_lr2 的旋轉軸跟其他三隻相反，所以零位角是 -45）
#     ('leg_lr1_lr2',          -45,    (-110, 110),   True,     0),
#     ('leg_lr2_lr3',           45,    (-110, 110),   False,    0),
#     ('leg_lr_wheel',           0,    None,          False,    0),

#     # ---- 右手
#     ('hand_r1_r2',            30,    (-110, 110),   False,  -15),
#     ('hand_r2_r3',            30,    (-110, 110),   True,    75),
#     ('hand_r3_gripper_r1',     0,    (-110, 110),   True,  -110),
#     ('gripper_r1_r2',          0,    (-180, 180),   False,    0),

#     # ---- 左手
#     ('hand_l1_l2',            30,    (-110, 110),   False,  -15),
#     ('hand_l2_l3',            30,    (-110, 110),   True,    75),
#     ('hand_l3_gripper_l1',     0,    (-110, 110),   False,  110),
#     ('gripper_l1_l2',          0,    (-180, 180),   False,    0),
# ]

#
#  ⚠️ limit 的基準是「零位角」，不是 Fusion UI 顯示的數字。
#     Fusion UI 會顯示 = 零位角 + limit，看起來不對稱是正常的。
#     匯出時 Joint.py 會扣掉零位角，所以 URDF 拿到的就是這裡填的值。
#
#     例：leg_rf1_rf2 零位角 45、limit -110~110
#         → Fusion UI 顯示 -65 ~ +155
#         → URDF 寫入     -110 ~ +110

# ----------------------------------------------------------------- helpers
def _joint_map(root):
    out = {}
    for i in range(root.joints.count):
        j = root.joints.item(i)
        out[j.name] = j
    return out


def _revolute(joint):
    return adsk.fusion.RevoluteJointMotion.cast(joint.jointMotion)


def _disable_limits(motion):
    try:
        lim = motion.rotationLimits
        lim.isMinimumValueEnabled = False
        lim.isMaximumValueEnabled = False
        return True
    except Exception:
        return False


def _norm_deg(d):
    """把角度收斂到 (-180, 180]，360 這種值才不會看不懂。"""
    d = math.fmod(d, 360.0)
    if d > 180.0:
        d -= 360.0
    elif d <= -180.0:
        d += 360.0
    return d


_ISAAC_TEMPLATE = u'''# -*- coding: utf-8 -*-
"""
{robot} / {pose_name}

由 Fusion 的 JointSetup 產生於 {stamp}。
角度基準是 URDF 的零位，不是 Fusion 的 joint 零位。
"""

# joint 名稱 -> 角度（度）。USD 的角度單位是度，不是弧度。
JOINT_POS_DEG = {{
{entries}
}}

# 給 Isaac Lab 的 ArticulationCfg.InitialStateCfg 用（弧度）：
# joint_pos={{
{entries_rad}
# }}

import omni.usd
from pxr import UsdPhysics, PhysxSchema

stage = omni.usd.get_context().get_stage()

found = {{}}
for prim in stage.Traverse():
    name = prim.GetName()
    if name in JOINT_POS_DEG and (prim.IsA(UsdPhysics.RevoluteJoint)
                                  or prim.IsA(UsdPhysics.PrismaticJoint)):
        found[name] = prim

ok_state, ok_drive, missing, errors = 0, 0, [], []

for name, deg in JOINT_POS_DEG.items():
    prim = found.get(name)
    if prim is None:
        missing.append(name)
        continue

    axis_token = 'angular' if prim.IsA(UsdPhysics.RevoluteJoint) else 'linear'

    # 1) 關節狀態 —— 決定 reset 之後回到哪裡
    try:
        state = PhysxSchema.JointStateAPI.Apply(prim, axis_token)
        state.CreatePositionAttr().Set(float(deg))
        ok_state += 1
    except Exception as e:
        errors.append('%s state: %s' % (name, e))

    # 2) drive 目標 —— 位置控制時追蹤的目標
    try:
        drive = UsdPhysics.DriveAPI.Get(prim, axis_token)
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, axis_token)
        drive.CreateTargetPositionAttr().Set(float(deg))
        ok_drive += 1
    except Exception as e:
        errors.append('%s drive: %s' % (name, e))

print('[{pose_name}] joint 表 %d 個，stage 找到 %d 個'
      % (len(JOINT_POS_DEG), len(found)))
print('  關節狀態設定 %d 個、drive 目標設定 %d 個' % (ok_state, ok_drive))
if missing:
    print('  stage 裡找不到 %d 個: %s' % (len(missing), ', '.join(missing)))
    print('  -> 確認 URDF 已匯入，而且 joint 名稱沒有被改過')
for e in errors[:10]:
    print('  錯誤: %s' % e)
'''


def _phase_dump_pose(root, ui, log, failures, notes):
    """把目前姿態輸出成姿態腳本。

    角度換算：JointSetup 的 rotationValue 是相對「建立 joint 當下」的姿態，
    而 URDF 的零位是表裡的零位角。兩者相減才是 URDF / Isaac 看到的角度。

    另外，第四欄標記反轉的 joint 在匯出後會被 urdf_joint_reverse 把
    <axis> 取負，所以這裡要再變一次號，輸出的值才跟 URDF 對得上。
    """
    import os
    import time

    jm = _joint_map(root)
    rows = []
    for name, pose, _lim, rev, _drive in JOINTS:
        j = jm.get(name)
        if j is None:
            failures.append(u'{}：找不到這個 joint，未寫入'.format(name))
            continue
        m = _revolute(j)
        if not m:
            continue
        try:
            cur = math.degrees(m.rotationValue)
        except Exception as e:
            failures.append(u'{}：讀不到角度（{}）'.format(name, e))
            continue
        q = _norm_deg(cur - float(pose))
        # 標記反轉的 joint，URDF 的 <axis> 會被取負（Fusion 端改不了），
        # 所以同一個物理姿勢在 URDF 裡的角度正負相反。
        if rev:
            q = -q
        rows.append((name, q))

    if not rows:
        failures.append(u'沒有可輸出的 joint。')
        return

    if DRY_RUN:
        log.append(u'[DUMP] (DRY_RUN) 可輸出 {} 個 joint，未存檔'.format(len(rows)))
        notes.append(u'  預計輸出（URDF 角度，度）：')
        for n, d in rows:
            notes.append(u'    {:<22} {:>8.3f}'.format(n, d))
        return

    dlg = ui.createFileDialog()
    dlg.title = u'儲存姿態腳本'
    dlg.filter = u'Python (*.py)'
    dlg.filterIndex = 0
    dlg.initialFilename = u'pose_stand.py'
    if dlg.showSave() != adsk.core.DialogResults.DialogOK:
        log.append(u'[DUMP] 使用者取消')
        return

    path = dlg.filename
    if not path.lower().endswith('.py'):
        path += '.py'
    pose_name = re.sub(r'[^A-Za-z0-9_\-]', '_',
                       os.path.splitext(os.path.basename(path))[0]) or 'pose'

    entries = u'\n'.join(
        u'    "{}": {:.4f},'.format(n, d) for n, d in rows)
    entries_rad = u'\n'.join(
        u'#     "{}": {:.6f},'.format(n, math.radians(d)) for n, d in rows)

    try:
        robot = root.parentDesign.parentDocument.name
    except Exception:
        robot = 'robot'

    text = _ISAAC_TEMPLATE.format(
        pose_name=pose_name,
        stamp=time.strftime('%Y-%m-%d %H:%M'),
        robot=robot,
        entries=entries,
        entries_rad=entries_rad)

    try:
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
    except Exception as e:
        failures.append(u'寫檔失敗：{}'.format(e))
        return

    log.append(u'[DUMP] 已輸出 {} 個 joint'.format(len(rows)))
    notes.append(u'  姿態腳本：{}'.format(path))


# ----------------------------------------------------------------- 各階段
def _phase_free(root, log):
    freed = 0
    for i in range(root.joints.count):
        m = _revolute(root.joints.item(i))
        if m and not DRY_RUN and _disable_limits(m):
            freed += 1
    log.append(u'[FREE] {}'.format(
        u'(DRY_RUN 未修改)' if DRY_RUN else u'解除 {} 個 joint 的 limit'.format(freed)))


def _phase_pose(root, log, failures):
    jm = _joint_map(root)
    moved = 0
    if not DRY_RUN:
        for i in range(root.joints.count):
            m = _revolute(root.joints.item(i))
            if m:
                _disable_limits(m)
    for name, pose, _lim, _rev, drive in JOINTS:
        j = jm.get(name)
        if j is None:
            continue
        m = _revolute(j)
        if not m:
            continue
        if DRY_RUN:
            continue
        try:
            m.rotationValue = math.radians(float(drive))
            moved += 1
        except Exception as e:
            failures.append(u'{} 驅動失敗：{}'.format(name, e))
    log.append(u'[POSE] {}'.format(
        u'(DRY_RUN 未修改)' if DRY_RUN else u'驅動 {} 個'.format(moved)))


def _phase_reverse(root, log, failures, notes):
    """產生「匯出後要在 URDF 改哪一行」的清單。

    Fusion 不允許在 joint 建立後更改旋轉軸：customRotationAxisEntity 和
    rotationAxis 兩個 setter 都會回 error 3 (Invalid parameter value)。
    所以軸向反轉只能在 URDF 端做 —— 把 <axis xyz> 的分量取負即可，
    limit 若對稱（例如 -110~+110）則不需要改。
    """
    jm = _joint_map(root)
    targets = [n for n, _p, _l, rev, _d in JOINTS if rev]

    for name in targets:
        j = jm.get(name)
        if j is None:
            failures.append(u'{}：找不到這個 joint'.format(name))
            continue
        m = _revolute(j)
        if not m:
            failures.append(u'{}：不是 revolute'.format(name))
            continue
        try:
            v = m.rotationAxisVector
            neg = u'{:g} {:g} {:g}'.format(-v.x, -v.y, -v.z)
            cur = u'{:g} {:g} {:g}'.format(v.x, v.y, v.z)
        except Exception as e:
            failures.append(u'{}：讀不到旋轉軸（{}）'.format(name, e))
            continue
        notes.append(u'  <joint name="{}">'.format(name))
        notes.append(u'    <axis xyz="{}"/>   →   <axis xyz="{}"/>'.format(
            cur, neg))

    log.append(u'[REVERSE] 需在 URDF 反轉 {} 個（見下方）'.format(len(targets)))


def _phase_limits(root, log, failures):
    jm = _joint_map(root)
    done = 0
    for name, pose, lim, _rev, _drive in JOINTS:
        j = jm.get(name)
        if j is None:
            continue
        m = _revolute(j)
        if not m or DRY_RUN:
            continue
        try:
            limits = m.rotationLimits
            if lim is None:
                limits.isMinimumValueEnabled = False
                limits.isMaximumValueEnabled = False
            else:
                if not (lim[0] <= 0 <= lim[1]):
                    failures.append(
                        u'{}：limit {}~{} 沒有涵蓋 0，零位角會落在範圍外'.format(
                            name, lim[0], lim[1]))
                # 基準是表裡的零位角，不是當下的 rotationValue ——
                # 姿態如果在中途被重置，用當下值會把 limit 算到錯的地方。
                base = math.radians(float(pose))
                limits.isMinimumValueEnabled = True
                limits.minimumValue = base + math.radians(lim[0])
                limits.isMaximumValueEnabled = True
                limits.maximumValue = base + math.radians(lim[1])

                if not (limits.isMinimumValueEnabled
                        and limits.isMaximumValueEnabled):
                    failures.append(u'{}：limit 寫入後沒有維持啟用狀態'.format(name))
            done += 1
        except Exception as e:
            failures.append(u'{} limit 失敗：{}'.format(name, e))
    log.append(u'[LIMITS] {}'.format(
        u'(DRY_RUN 未修改)' if DRY_RUN else u'設定 {} 個'.format(done)))


# ----------------------------------------------------------------- entry
def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox(u'請先開啟一個 Fusion 設計檔。')
            return
        root = design.rootComponent

        if root.joints.count == 0:
            ui.messageBox(u'這個設計裡沒有 joint。')
            return

        log = []
        failures = []
        notes = []

        if DO_FREE:
            _phase_free(root, log)
        else:
            if DO_REVERSE:
                _phase_reverse(root, log, failures, notes)
            if DO_POSE:
                _phase_pose(root, log, failures)
            if DO_DUMP_POSE:
                _phase_dump_pose(root, ui, log, failures, notes)
            if DO_LIMITS:
                # 一定要在 POSE 之後 —— _phase_pose 會先解除所有 limit 才能
                # 自由驅動，由 LIMITS 收尾重新啟用。
                _phase_limits(root, log, failures)

        # ---- 對照表
        jm = _joint_map(root)
        listed = set(n for n, _p, _l, _r, _d in JOINTS)
        missing = [n for n in listed if n not in jm]
        extra = [n for n in jm if n not in listed]

        rows = []
        mismatch = []
        for name, pose, lim, rev, drive in JOINTS:
            j = jm.get(name)
            if j is None:
                rows.append(u'{:<22} {:>8}   {:<12} {}'.format(
                    name[:22], u'缺少', u'--', u''))
                continue
            m = _revolute(j)
            try:
                cur = _norm_deg(math.degrees(m.rotationValue)) if m else 0.0
            except Exception:
                cur = 0.0
            lim_txt = u'--' if lim is None else u'{:+.0f}~{:+.0f}'.format(*lim)

            tag = u'反轉' if rev else u''
            off = abs(_norm_deg(cur - float(pose)))
            if off > 0.5:
                tag = (tag + u' ' if tag else u'') + u'⚠ 差 {:.1f}°'.format(off)
                mismatch.append(u'{}：現在 {:.1f}°，零位角 {:.1f}°'.format(
                    name, cur, float(pose)))

            drv = u'{:+.0f}'.format(float(drive))
            rows.append(u'{:<20} {:>7.1f} {:>7.1f} {:>6}  {:<11} {}'.format(
                name[:20], cur, float(pose), drv, lim_txt, tag))

        msg = [u'joint：設計裡 {} 個、表裡 {} 個'.format(
            root.joints.count, len(JOINTS))]
        if DRY_RUN:
            msg.append(u'[DRY_RUN] 沒有修改任何東西。')
        msg.append(u'')
        msg.extend(log)
        msg += [u'',
                u'名稱                     現在角   零位角  limit        備註',
                u'-' * 70]
        msg.extend(rows)

        if missing:
            msg += [u'', u'⚠️ 表裡有、設計裡沒有：']
            msg.extend(u'  ' + n for n in sorted(missing))
        if extra:
            msg += [u'', u'⚠️ 設計裡有、表裡沒有（不會被處理）：']
            msg.extend(u'  ' + n for n in sorted(extra))
        if mismatch:
            msg += [u'',
                    u'⚠️ 現在角與零位角不符 {} 個 —— 姿態沒有停在該停的地方。'.format(
                        len(mismatch)),
                    u'   把 DO_POSE 設成 True 單獨再跑一次，然後確認這一段消失。']
            msg.extend(u'  ' + x for x in mismatch[:12])
        if notes:
            msg += [u'', u'匯出後要在 URDF 改的行（Fusion 端無法反轉軸向）：']
            msg.extend(notes)
        if failures:
            msg += [u'', u'失敗：']
            msg.extend(u'  ' + f for f in failures)

        ui.messageBox(u'\n'.join(msg), u'JointSetup')

    except Exception:
        if ui:
            ui.messageBox(u'Failed:\n{}'.format(traceback.format_exc()))
