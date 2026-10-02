# -*- coding: utf-8 -*-
"""
URDF Set Effort / Velocity
==========================
fusion2urdf 把每個 joint 的 effort 和 velocity 寫死成 100
（Joint.py 的 make_joint_xml）。100 N·m 對一般致動器是天文數字，
在 Isaac Sim 裡等於給了無限扭矩，模擬結果不能反映真實能力。

這個工具依下面的表逐一改寫。用正規表示式就地替換，不經過 XML 解析，
所以排版、註解、屬性順序都不會被動到 —— diff 只會顯示真正改動的行。

continuous joint（輪子）原本沒有 <limit> 元素，本工具會補上一個
只含 effort / velocity 的 limit（URDF 規格允許 continuous 省略 lower/upper）。

用法
----
    python main.py path/to/robot.urdf

會先寫一份 <檔名>.bak 再就地修改。

單位
----
    effort   : N·m（旋轉關節）
    velocity : rad/s
"""

import os
import re
import shutil
import sys

# ----------------------------------------------------------------- 設定
# 名稱: (effort N·m, velocity rad/s)
#   None = 跳過不改，保留原值（用於規格還沒確認的關節）
#
# 已確認：DM-J10010-2EC 額定 50 N·m
#   直驅      effort = 50
#   1:3 皮帶  effort = 50 × 3 × 0.97 ≈ 145
#
# ⚠️ 下面標 TODO 的是還沒拿到規格的，目前設 None 會保留 100。
#    拿到數字填進去重跑即可。
DM_DIRECT = 50.0
DM_BELT = 145.0
DM_VEL = None          # TODO: DM-J10010-2EC 最高角速度 (rad/s)
DM_BELT_VEL = None     # TODO: 上面那個除以 3
MG_EFFORT = None       # TODO: MG4010E-i10 額定扭矩 (N·m)
MG_VEL = None          # TODO: MG4010E-i10 最高角速度 (rad/s)
WHEEL_EFFORT = None    # TODO: 4 吋輪轂電機額定扭矩 (N·m)
WHEEL_VEL = None       # TODO: 4 吋輪轂電機最高角速度 (rad/s)

# 兩個都是 None 時，才依上面的 JOINTS 表逐一設定。
# 填數字的話，URDF 裡每個 joint 都套用這個值（忽略表格）——
# 用於初期驗證：先讓扭矩和速度不要成為限制因素，單純確認結構、
# 質量、運動鏈是否正確。規格確認後改回 None 即可。
OVERRIDE_EFFORT = 1000.0      # N.m，None = 用表格
OVERRIDE_VELOCITY = 1000.0    # rad/s，None = 用表格

JOINTS = {
    # ---- 髖橫向 / 肩橫向（直驅 DM-J10010）
    'base_leg_rf':        (DM_DIRECT, DM_VEL),
    'base_leg_rr':        (DM_DIRECT, DM_VEL),
    'base_leg_lf':        (DM_DIRECT, DM_VEL),
    'base_leg_lr':        (DM_DIRECT, DM_VEL),
    'base_hand_r':        (DM_DIRECT, DM_VEL),
    'base_hand_l':        (DM_DIRECT, DM_VEL),

    # ---- 髖上下（直驅 DM-J10010）
    'leg_rf1_rf2':        (DM_DIRECT, DM_VEL),
    'leg_rr1_rr2':        (DM_DIRECT, DM_VEL),
    'leg_lf1_lf2':        (DM_DIRECT, DM_VEL),
    'leg_lr1_lr2':        (DM_DIRECT, DM_VEL),

    # ---- 膝（DM-J10010 + 1:3 皮帶）
    'leg_rf2_rf3':        (DM_BELT, DM_BELT_VEL),
    'leg_rr2_rr3':        (DM_BELT, DM_BELT_VEL),
    'leg_lf2_lf3':        (DM_BELT, DM_BELT_VEL),
    'leg_lr2_lr3':        (DM_BELT, DM_BELT_VEL),

    # ---- 肩上下（直驅 DM-J10010）
    'hand_r1_r2':         (DM_DIRECT, DM_VEL),
    'hand_l1_l2':         (DM_DIRECT, DM_VEL),

    # ---- 肘（DM-J10010 + 1:3 皮帶）
    'hand_r2_r3':         (DM_BELT, DM_BELT_VEL),
    'hand_l2_l3':         (DM_BELT, DM_BELT_VEL),

    # ---- 腕 pitch / roll（MG4010E-i10）
    'hand_r3_gripper_r1': (MG_EFFORT, MG_VEL),
    'hand_l3_gripper_l1': (MG_EFFORT, MG_VEL),
    'gripper_r1_r2':      (MG_EFFORT, MG_VEL),
    'gripper_l1_l2':      (MG_EFFORT, MG_VEL),

    # ---- 輪（4 吋輪轂電機，continuous）
    'leg_rf_wheel':       (WHEEL_EFFORT, WHEEL_VEL),
    'leg_rr_wheel':       (WHEEL_EFFORT, WHEEL_VEL),
    'leg_lf_wheel':       (WHEEL_EFFORT, WHEEL_VEL),
    'leg_lr_wheel':       (WHEEL_EFFORT, WHEEL_VEL),
}


# ----------------------------------------------------------------- 實作
def _fmt(v):
    """數字轉字串，整數不要拖小數點。"""
    return str(int(v)) if float(v).is_integer() else repr(float(v))


def _patch_block(block, effort, velocity):
    """改寫單一 <joint>…</joint> 區塊，回傳 (新區塊, 說明)。"""
    m = re.search(r'<limit\b[^>]*/>', block)

    if m:
        limit = m.group(0)
        new = limit
        if effort is not None:
            new = re.sub(r'effort="[^"]*"', 'effort="%s"' % _fmt(effort), new)
        if velocity is not None:
            new = re.sub(r'velocity="[^"]*"', 'velocity="%s"' % _fmt(velocity),
                         new)
        if new == limit:
            return block, 'unchanged'
        return block.replace(limit, new), 'limit updated'

    # continuous joint：沒有 <limit>，補一個只含 effort / velocity 的
    if effort is None and velocity is None:
        return block, 'no limit element, nothing to add'

    attrs = []
    if effort is not None:
        attrs.append('effort="%s"' % _fmt(effort))
    if velocity is not None:
        attrs.append('velocity="%s"' % _fmt(velocity))

    axis = re.search(r'([ \t]*)<axis\b[^>]*/>', block)
    indent = axis.group(1) if axis else '    '
    inserted = '\n%s<limit %s/>' % (indent, ' '.join(attrs))

    anchor = axis.group(0) if axis else None
    if anchor:
        return block.replace(anchor, anchor + inserted), 'limit inserted'
    return block.replace('</joint>', inserted.lstrip('\n') + '\n</joint>'), \
        'limit inserted'


def main(path):
    if not os.path.isfile(path):
        print('找不到檔案: %s' % path)
        return 1

    with open(path, 'r', encoding='utf-8') as f:
        text = f.read()

    # 只抓有 type 屬性的 joint，避開 transmission 裡同名的 <joint>。
    # 不依賴 name / type 的先後順序 —— 其他工具用 XML 庫重寫後順序可能會變。
    pattern = re.compile(
        r'<joint\b(?=[^>]*\btype=)[^>]*\bname="([^"]+)"[^>]*>.*?</joint>', re.S)

    seen = []
    changed = []
    skipped = []
    unknown = []

    override = OVERRIDE_EFFORT is not None or OVERRIDE_VELOCITY is not None

    def repl(m):
        name = m.group(1)
        seen.append(name)
        if override:
            effort, velocity = OVERRIDE_EFFORT, OVERRIDE_VELOCITY
        else:
            if name not in JOINTS:
                unknown.append(name)
                return m.group(0)
            effort, velocity = JOINTS[name]
        if effort is None and velocity is None:
            skipped.append(name)
            return m.group(0)
        block, how = _patch_block(m.group(0), effort, velocity)
        changed.append((name, effort, velocity, how))
        return block

    out = pattern.sub(repl, text)

    if out == text:
        print('沒有任何改動。')
    else:
        shutil.copyfile(path, path + '.bak')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(out)

    if override:
        print('[OVERRIDE] 全部 joint 套用 effort=%s velocity=%s，忽略 JOINTS 表'
              % (OVERRIDE_EFFORT, OVERRIDE_VELOCITY))
    print('URDF 裡的 joint: %d 個' % len(seen))
    print('已更新: %d 個' % len(changed))
    for name, e, v, how in changed:
        print('  %-22s effort=%-6s velocity=%-6s  (%s)' % (
            name,
            _fmt(e) if e is not None else '不變',
            _fmt(v) if v is not None else '不變',
            how))

    if skipped:
        print('\n跳過（規格未確認，保留原值）: %d 個' % len(skipped))
        for n in skipped:
            print('  %s' % n)
    if unknown:
        print('\n[!] URDF 裡有、表裡沒有: %d 個' % len(unknown))
        for n in unknown:
            print('  %s' % n)
    missing = [n for n in JOINTS if n not in seen]
    if missing:
        print('\n[!] 表裡有、URDF 裡沒有: %d 個' % len(missing))
        for n in missing:
            print('  %s' % n)

    if out != text:
        print('\n備份: %s.bak' % path)
    return 0


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('用法: python main.py path/to/robot.urdf')
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
