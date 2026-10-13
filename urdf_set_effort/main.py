# -*- coding: utf-8 -*-
"""
URDF Set Effort / Velocity（互動式）
==================================
fusion2urdf 把每個 joint 的 effort 和 velocity 寫死成 100，這個工具讓你逐一改成真實值。
操作方式跟 urdf_joint_reverse 一樣：列出所有關節和目前的 effort / velocity，
選一個（或多個）關節、輸入新值，馬上存回原檔再重新列出，直到輸入 q 離開。

用正規表示式就地替換，不經過 XML 函式庫重寫，所以排版、註解、屬性順序都不會被動到，
.xacro 也能用（xacro 命名空間不會被弄壞）。transmission 裡的同名 <joint> 不會被當成關節。

continuous joint（輪子）原本沒有 <limit>，會補一個只含 effort / velocity 的
（URDF 規格允許 continuous 省略 lower / upper）。

用法
----
    python main.py path/to/robot.urdf

單位
----
    revolute / continuous：effort N·m、velocity rad/s
    prismatic           ：effort N、  velocity m/s
"""

import argparse
import re
import sys
from pathlib import Path

EDITABLE_TYPES = {'revolute', 'continuous', 'prismatic'}

# 只抓開頭標籤有 type 的 <joint>，避開 transmission 裡的 <joint name="...">
JOINT_RE = re.compile(r'<joint\b(?=[^>]*\btype=)[^>]*>.*?</joint>', re.S)
LIMIT_RE = re.compile(r'<limit\b[^>]*/>')
AXIS_RE = re.compile(r'([ \t]*)<axis\b[^>]*/>')


def get_attr(tag, key):
    m = re.search(r'\b%s="([^"]*)"' % key, tag)
    return m.group(1) if m else None


def set_attr(tag, key, value):
    """改一個屬性；沒有的話加在 /> 前面（保留原本 /> 前有沒有空白的寫法）。"""
    if get_attr(tag, key) is not None:
        return re.sub(r'\b%s="[^"]*"' % key, '%s="%s"' % (key, value), tag)
    body = tag[:-2]
    trailing = ' ' if body != body.rstrip() else ''
    return '%s %s="%s"%s/>' % (body.rstrip(), key, value, trailing)


def fmt(value):
    """數字轉字串：整數不要拖小數點，其餘最多 4 位小數。"""
    v = round(float(value), 4)
    return str(int(v)) if v.is_integer() else repr(v)


def scan_joints(text):
    """回傳 [{name, type, start, end, effort, velocity}]，依檔案順序。"""
    joints = []
    for index, m in enumerate(JOINT_RE.finditer(text), start=1):
        head = text[m.start():text.index('>', m.start()) + 1]
        limit = LIMIT_RE.search(m.group(0))
        joints.append(dict(
            name=get_attr(head, 'name') or '<unnamed_%d>' % index,
            type=get_attr(head, 'type') or '',
            start=m.start(), end=m.end(),
            effort=get_attr(limit.group(0), 'effort') if limit else None,
            velocity=get_attr(limit.group(0), 'velocity') if limit else None))
    return joints


def apply_values(text, joint, effort, velocity):
    """把一個關節的 effort / velocity 改掉（None = 不改），回傳新的整份文字。"""
    block = text[joint['start']:joint['end']]
    limit = LIMIT_RE.search(block)
    if limit:
        tag = limit.group(0)
        if effort is not None:
            tag = set_attr(tag, 'effort', fmt(effort))
        if velocity is not None:
            tag = set_attr(tag, 'velocity', fmt(velocity))
        block = block[:limit.start()] + tag + block[limit.end():]
    else:
        # 沒有 <limit>（continuous）：補一個，effort / velocity 沒給的先不寫
        attrs = []
        if effort is not None:
            attrs.append('effort="%s"' % fmt(effort))
        if velocity is not None:
            attrs.append('velocity="%s"' % fmt(velocity))
        axis = AXIS_RE.search(block)
        indent = axis.group(1) if axis else '    '
        close = ' />' if axis and axis.group(0).endswith(' />') else '/>'   # 跟著 <axis> 的寫法
        new_tag = '<limit %s%s' % (' '.join(attrs), close)
        if axis:
            block = block[:axis.end()] + '\n' + indent + new_tag + block[axis.end():]
        else:
            close = block.rfind('</joint>')
            block = block[:close] + indent + new_tag + '\n' + block[close:]
    return text[:joint['start']] + block + text[joint['end']:]


def units(joint_type):
    return ('N', 'm/s') if joint_type == 'prismatic' else ('N·m', 'rad/s')


def list_joints(joints):
    print('\nAvailable joints:')
    width = max(len(j['name']) for j in joints)
    for index, j in enumerate(joints, start=1):
        if j['type'] in EDITABLE_TYPES:
            eu, vu = units(j['type'])
            effort = '%s %s' % (j['effort'], eu) if j['effort'] is not None else '-'
            velocity = '%s %s' % (j['velocity'], vu) if j['velocity'] is not None else '-'
            print('  [%2d] %-*s  type=%-10s  effort=%-12s  velocity=%s'
                  % (index, width, j['name'], j['type'], effort, velocity))
        else:
            print('  [%2d] %-*s  type=%-10s  (no effort/velocity)' % (index, width, j['name'], j['type']))


def select_joints(selection, joints):
    """'3'、'base_leg_rf'、'1,3,5'、'7-12'、'all' → 關節 list；有錯回傳 (None, 錯誤訊息)。"""
    if selection.lower() == 'all':
        return joints, None
    picked = []
    for token in re.split(r'[,\s]+', selection):
        if not token:
            continue
        m = re.fullmatch(r'(\d+)-(\d+)', token)
        if m:
            a, b = int(m.group(1)), int(m.group(2))
            if not (1 <= a <= b <= len(joints)):
                return None, 'Range out of bounds: %s' % token
            picked += joints[a - 1:b]
        elif token.isdigit():
            i = int(token)
            if not 1 <= i <= len(joints):
                return None, 'Index out of bounds: %s' % token
            picked.append(joints[i - 1])
        else:
            match = [j for j in joints if j['name'] == token]
            if not match:
                return None, 'Joint not found: %s' % token
            picked += match
    seen, out = set(), []
    for j in picked:
        if j['name'] not in seen:
            seen.add(j['name'])
            out.append(j)
    return out, None


def ask_value(label, unit, current, required):
    """回傳新值、None（不改）或 'cancel'。required：有關節還沒有這個值，不能留空。"""
    if required:
        hint = 'required'
    else:
        hint = 'Enter = keep %s' % (current if current is not None else 'current')
    while True:
        raw = input('  New %s [%s] (%s, q = cancel): ' % (label, unit, hint)).strip()
        if raw == '':
            if required:
                print('  Required: URDF <limit> needs both effort and velocity, '
                      'and some selected joints have no %s yet.' % label)
                continue
            return None
        if raw.lower() == 'q':
            return 'cancel'
        try:
            value = float(raw)
        except ValueError:
            print('  Not a number: %s' % raw)
            continue
        if value <= 0:
            print('  Must be greater than 0.')
            continue
        return value


def interactive_edit(path):
    text = path.read_text(encoding='utf-8')
    if not scan_joints(text):
        print('No <joint> elements found.')
        return 1

    print('Loaded: %s' % path)
    print('Select a joint by index or name (several: 1,3,5 or 7-12 or all), '
          'then enter new values. Type q to quit.')

    while True:
        joints = scan_joints(text)
        list_joints(joints)
        selection = input('\nSelect joint index/name, or q to quit: ').strip()

        if selection.lower() == 'q':
            print('Bye.')
            return 0
        if not selection:
            print('Please enter a joint index or name.')
            continue

        picked, error = select_joints(selection, joints)
        if error:
            print(error)
            continue
        skipped = [j['name'] for j in picked if j['type'] not in EDITABLE_TYPES]
        picked = [j for j in picked if j['type'] in EDITABLE_TYPES]
        if skipped:
            print('Skipped (no effort/velocity for this joint type): %s' % ', '.join(skipped))
        if not picked:
            continue

        if len(picked) == 1:
            j = picked[0]
            print('\n%s  type=%s  effort=%s  velocity=%s'
                  % (j['name'], j['type'], j['effort'] or '-', j['velocity'] or '-'))
            cur_e, cur_v = j['effort'], j['velocity']
        else:
            print('\n%d joints selected: %s' % (len(picked), ', '.join(j['name'] for j in picked)))
            cur_e = cur_v = None
        unit_set = {units(j['type']) for j in picked}
        eu, vu = unit_set.pop() if len(unit_set) == 1 else ('N·m or N', 'rad/s or m/s')

        effort = ask_value('effort', eu, cur_e, any(j['effort'] is None for j in picked))
        if effort == 'cancel':
            print('Cancelled.')
            continue
        velocity = ask_value('velocity', vu, cur_v, any(j['velocity'] is None for j in picked))
        if velocity == 'cancel':
            print('Cancelled.')
            continue
        if effort is None and velocity is None:
            print('No change.')
            continue

        # 一個一個改；每改一個都重新掃描，位置才不會跑掉
        for name in [j['name'] for j in picked]:
            joint = next(j for j in scan_joints(text) if j['name'] == name)
            text = apply_values(text, joint, effort, velocity)
        path.write_text(text, encoding='utf-8')

        changes = []
        if effort is not None:
            changes.append('effort=%s' % fmt(effort))
        if velocity is not None:
            changes.append('velocity=%s' % fmt(velocity))
        print('%s -> %s. Changes saved back to the original file.'
              % (', '.join(j['name'] for j in picked), ', '.join(changes)))


def parse_args():
    parser = argparse.ArgumentParser(
        description='Interactively set URDF joint effort / velocity limits in place.')
    parser.add_argument('urdf_path', help='Path to the target URDF (or xacro) file')
    return parser.parse_args()


def main():
    args = parse_args()
    target_path = Path(args.urdf_path).expanduser()
    if not target_path.exists():
        print('File not found: %s' % target_path)
        return 1
    if target_path.suffix.lower() not in ('.urdf', '.xacro'):
        print('Warning: target suffix is not .urdf/.xacro, but processing will continue: %s' % target_path)
    try:
        return interactive_edit(target_path)
    except (KeyboardInterrupt, EOFError):
        print('\nBye.')
        return 0


if __name__ == '__main__':
    sys.exit(main())
