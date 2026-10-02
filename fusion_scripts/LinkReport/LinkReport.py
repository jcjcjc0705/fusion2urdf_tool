# -*- coding: utf-8 -*-
"""
LinkReport
==========
唯讀。兩份報告：

1. component 清單
   root 底下每個 component 的 body 數量與質量。
   link 的「自持」和「含子階層」必須相等 —— fusion2urdf 的 copy_occs()
   只抓自持的 body，不相等代表有 body 卡在巢狀層級裡，匯出時會靜默消失。

2. 未認領清單（CHECK_UNCLAIMED = True 時）
   找出來源樹（All:1）裡沒有被任何 link 認領的 body。

   GroupBodiesToLink 是用複製的，所以每個被認領的 body 在某個 link 裡都會有
   一個幾何相同、位置也相同的副本。用「質心 + 體積」當指紋比對：兩者都是精確
   計算、與密度無關，複製也不會改變它們。比對不到的就是漏掉的。

   刪掉 All:1 之前務必跑一次 —— 刪了就回不來了。

Fusion 的 messageBox 有長度上限，所以報告分兩個對話框顯示。
"""

import adsk.core
import adsk.fusion
import traceback

# ----------------------------------------------------------------- 設定
SOURCE_NAME = 'All'       # 來源樹的 component 名稱
CHECK_UNCLAIMED = True    # 是否執行未認領比對（會慢一點，要算物理屬性）
POS_TOL_CM = 0.01         # 質心分桶精度，0.01cm = 0.1mm
VOL_REL_TOL = 1e-3        # 體積相對容差 0.1%
LIST_LIMIT = 25           # 未認領清單最多列幾筆
SHOW_COM = True           # 顯示每個 link 的質心與整機質心（取代「可見」欄）


class _Cancelled(Exception):
    pass


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


def _count_tree(occ):
    bodies = []
    _walk_bodies(occ, bodies)
    visible = 0
    for b in bodies:
        try:
            if b.isLightBulbOn:
                visible += 1
        except Exception:
            visible += 1
    return len(bodies), visible


def _props(body):
    try:
        pp = body.getPhysicalProperties(
            adsk.fusion.CalculationAccuracy.LowCalculationAccuracy)
        if not pp:
            return None
        c = pp.centerOfMass
        return (pp.volume, (c.x, c.y, c.z))
    except Exception:
        return None


def _key(center):
    return (round(center[0] / POS_TOL_CM),
            round(center[1] / POS_TOL_CM),
            round(center[2] / POS_TOL_CM))


def _neighbour_keys(k):
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                yield (k[0] + dx, k[1] + dy, k[2] + dz)


def _find_unclaimed(ui, source_occ, link_occs):
    link_bodies = []
    for occ in link_occs:
        _walk_bodies(occ, link_bodies)

    source_bodies = []
    _walk_bodies(source_occ, source_bodies)

    total = len(link_bodies) + len(source_bodies)
    progress = ui.createProgressDialog()
    progress.isCancelButtonShown = True
    progress.show(u'LinkReport', u'比對中 %v / %m ...', 0, max(total, 1))

    done = 0
    try:
        index = {}
        for b in link_bodies:
            if progress.wasCancelled:
                raise _Cancelled()
            done += 1
            progress.progressValue = done
            p = _props(b)
            if not p:
                continue
            vol, c = p
            index.setdefault(_key(c), []).append(vol)

        unclaimed = []
        no_props = 0
        for b in source_bodies:
            if progress.wasCancelled:
                raise _Cancelled()
            done += 1
            progress.progressValue = done

            p = _props(b)
            if not p:
                no_props += 1
                continue
            vol, c = p

            matched = False
            for k in _neighbour_keys(_key(c)):
                for v in index.get(k, ()):
                    if abs(v - vol) <= max(abs(vol), 1e-9) * VOL_REL_TOL:
                        matched = True
                        break
                if matched:
                    break

            if not matched:
                name = 'body'
                try:
                    name = b.name
                except Exception:
                    pass
                path = ''
                try:
                    path = b.assemblyContext.fullPathName
                except Exception:
                    pass
                unclaimed.append((vol, c, name, path))
    finally:
        progress.hide()

    return unclaimed, len(link_bodies), len(source_bodies), no_props


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

        rows = []
        grand_total = 0
        source_occ = None
        link_occs = []

        for i in range(root.occurrences.count):
            occ = root.occurrences.item(i)
            name = occ.name
            comp = occ.component

            if name.split(':')[0] == SOURCE_NAME:
                source_occ = occ
            else:
                link_occs.append(occ)

            try:
                direct = comp.bRepBodies.count
            except Exception:
                direct = -1

            total, visible = _count_tree(occ)
            grand_total += total

            mass_kg = 0.0
            com = None
            try:
                pp = occ.getPhysicalProperties(
                    adsk.fusion.CalculationAccuracy.LowCalculationAccuracy)
                if pp:
                    mass_kg = pp.mass
                    c = pp.centerOfMass
                    com = (c.x, c.y, c.z)
            except Exception:
                try:
                    mass_kg = comp.physicalProperties.mass
                except Exception:
                    pass

            rows.append((name, direct, total, visible, mass_kg, com))

        try:
            root_direct = root.bRepBodies.count
        except Exception:
            root_direct = 0
        grand_total += root_direct

        # ---------------------------------------------- 第一份：component 清單
        part1 = [u'root 底下的 component：{} 個'.format(root.occurrences.count),
                 u'實例 body 總數：{}'.format(grand_total)]
        if root_direct:
            part1.append(u'直接掛在 root 的 body：{}'.format(root_direct))
        if SHOW_COM:
            part1 += [u'',
                      u'名稱                      自持 / 含子階層   質量        質心 xyz (mm)',
                      u'-' * 76]
        else:
            part1 += [u'',
                      u'名稱                          自持 / 含子階層 / 可見    質量',
                      u'-' * 64]

        link_sum = 0
        total_mass = 0.0
        mom = [0.0, 0.0, 0.0]
        bad_structure = []
        for name, direct, total, visible, mass_kg, com in rows:
            mass_txt = u'{:.3f} kg'.format(mass_kg) if mass_kg else u'--'
            if SHOW_COM:
                com_txt = u'{:>7.1f} {:>7.1f} {:>7.1f}'.format(
                    com[0] * 10, com[1] * 10, com[2] * 10) if com else u'--'
                part1.append(u'{:<24} {:>5} / {:>6}   {:>10}   {}'.format(
                    name[:24], direct, total, mass_txt, com_txt))
            else:
                part1.append(u'{:<28} {:>5} / {:>6} / {:>5}   {}'.format(
                    name[:28], direct, total, visible, mass_txt))

            if name.split(':')[0] != SOURCE_NAME:
                link_sum += direct
                if direct != total:
                    bad_structure.append(name)
                if com and mass_kg:
                    total_mass += mass_kg
                    for k in range(3):
                        mom[k] += mass_kg * com[k]

        if SHOW_COM and total_mass > 0:
            part1 += [u'',
                      u'整機（不含來源樹）：{:.3f} kg'.format(total_mass),
                      u'整機質心 (mm)：{:.1f}, {:.1f}, {:.1f}'.format(
                          mom[0] / total_mass * 10,
                          mom[1] / total_mass * 10,
                          mom[2] / total_mass * 10)]

        if bad_structure:
            part1 += [u'',
                      u'⚠️ 自持 != 含子階層（body 卡在巢狀層級，匯出會漏）：']
            part1.extend(u'  ' + n for n in bad_structure)

        if source_occ is not None:
            s_row = [r for r in rows if r[0] == source_occ.name]
            s_total = s_row[0][2] if s_row else 0
            part1 += [u'',
                      u'link 自持加總：{}'.format(link_sum),
                      u'來源樹 "{}" body 數：{}'.format(SOURCE_NAME, s_total),
                      u'差：{}'.format(s_total - link_sum)]

        # ---------------------------------------------- 第二份：未認領比對
        part2 = []
        if CHECK_UNCLAIMED and source_occ is not None and link_occs:
            try:
                unclaimed, n_link, n_src, no_props = _find_unclaimed(
                    ui, source_occ, link_occs)
            except _Cancelled:
                part2 = [u'（未認領比對已取消）']
                unclaimed = None

            if part2 == []:
                part2 = [u'未認領比對',
                         u'link 側 {} 個 body，來源側 {} 個'.format(n_link, n_src),
                         u'未被任何 link 認領：{} 個'.format(len(unclaimed))]
                if no_props:
                    part2.append(u'（{} 個算不出物理屬性，已略過）'.format(no_props))

                if unclaimed:
                    unclaimed.sort(reverse=True)
                    part2 += [u'', u'體積大的在前，座標 mm：', u'']
                    for vol, c, nm, path in unclaimed:
                        part2.append(
                            u'{:9.3f} cm3  @ ({:8.1f},{:8.1f},{:8.1f})  {}'.format(
                                vol, c[0] * 10, c[1] * 10, c[2] * 10, nm))
                        if path:
                            part2.append(u'            {}'.format(path))
                    part2 += [u'',
                              u'確認這些都是你刻意不要的，才可以刪掉 {}:1。'.format(
                                  SOURCE_NAME)]
                else:
                    part2 += [u'',
                              u'每個來源 body 都在某個 link 裡找得到對應，',
                              u'可以安心刪掉 {}:1。'.format(SOURCE_NAME)]

        # ---------------------------------------------- 輸出
        ui.messageBox(u'\n'.join(part1), u'LinkReport 1/2　component 清單')

        if part2:
            head = part2[:3]
            body = part2[3:]
            shown = []
            for i, line in enumerate(body):
                if i >= LIST_LIMIT * 2:
                    shown.append(u'... 其餘未顯示，調大腳本開頭的 LIST_LIMIT')
                    break
                shown.append(line)
            ui.messageBox(u'\n'.join(head + shown),
                          u'LinkReport 2/2　未認領比對')

    except Exception:
        if ui:
            ui.messageBox(u'Failed:\n{}'.format(traceback.format_exc()))
