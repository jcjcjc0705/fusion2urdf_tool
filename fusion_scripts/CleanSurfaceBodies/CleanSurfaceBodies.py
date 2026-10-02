# -*- coding: utf-8 -*-
"""
CleanSurfaceBodies
==================
找出設計中所有「非實體」的 BRepBody（IGES/STEP 匯入後縫不起來的曲面碎片，
在瀏覽器裡通常被丟進 Unstitched 資料夾），並依 MODE 做處理。

為什麼要清掉：
- 曲面體沒有體積 → 對 getPhysicalProperties() 的 mass / inertia 貢獻 0
- 但匯出 STL 時會產生零厚度薄殼 → Isaac Sim 當 collision mesh 會穿透、抖動

建議流程
--------
1. MODE = 'report'  先看有幾個、分布在哪些 component
2. MODE = 'hide'    全部隱藏，轉一圈看模型有沒有缺角
3. MODE = 'delete'  確認沒問題後刪掉
   （想還原隱藏就用 MODE = 'show'）
"""

import adsk.core
import adsk.fusion
import traceback

# ----------------------------------------------------------------- 設定
MODE = 'report'       # 'report' | 'hide' | 'show' | 'delete'
LIST_LIMIT = 40       # 報告裡最多列幾個 component


class _Cancelled(Exception):
    pass


def _collect(design, progress):
    """回傳 (曲面體清單, 實體數量, {component 名稱: 曲面體數})。"""
    surfaces = []
    solid_count = 0
    per_comp = {}

    comps = [c for c in design.allComponents]
    if progress is not None:
        progress.maximumValue = max(len(comps), 1)

    for idx, comp in enumerate(comps):
        if progress is not None:
            if progress.wasCancelled:
                raise _Cancelled()
            progress.progressValue = idx + 1

        try:
            bodies = [comp.bRepBodies.item(i) for i in range(comp.bRepBodies.count)]
        except Exception:
            continue

        n = 0
        for b in bodies:
            try:
                if b.isSolid:
                    solid_count += 1
                else:
                    surfaces.append(b)
                    n += 1
            except Exception:
                pass

        if n:
            per_comp[comp.name] = n

    return surfaces, solid_count, per_comp


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox(u'請先開啟一個 Fusion 設計檔。')
            return

        if MODE not in ('report', 'hide', 'show', 'delete'):
            ui.messageBox(u'MODE 只能是 report / hide / show / delete。')
            return

        progress = ui.createProgressDialog()
        progress.isCancelButtonShown = True
        progress.show(u'CleanSurfaceBodies', u'掃描 component %v / %m ...', 0, 1)

        cancelled = False
        try:
            surfaces, solid_count, per_comp = _collect(design, progress)
        except _Cancelled:
            progress.hide()
            ui.messageBox(u'已取消。')
            return
        finally:
            progress.hide()

        total = len(surfaces)
        header = [u'實體 body：{} 個'.format(solid_count),
                  u'曲面 body：{} 個，分布在 {} 個 component'.format(total, len(per_comp))]

        if total == 0:
            ui.messageBox(u'\n'.join(header + [u'', u'沒有曲面體，不需要處理。']),
                          u'CleanSurfaceBodies')
            return

        if MODE == 'report':
            rows = sorted(per_comp.items(), key=lambda kv: kv[1], reverse=True)
            body = [u'', u'曲面體最多的 component（前 {}）：'.format(LIST_LIMIT)]
            for name, n in rows[:LIST_LIMIT]:
                body.append(u'  {} : {} 個'.format(name, n))
            if len(rows) > LIST_LIMIT:
                body.append(u'  ... 還有 {} 個'.format(len(rows) - LIST_LIMIT))
            body += [u'',
                     u'下一步：把腳本開頭的 MODE 改成 \'hide\' 再跑一次，',
                     u'轉一圈確認模型沒有缺角，然後改成 \'delete\'。']
            ui.messageBox(u'\n'.join(header + body), u'CleanSurfaceBodies')
            return

        # hide / show / delete
        done = 0
        failed = []

        progress = ui.createProgressDialog()
        progress.isCancelButtonShown = True
        progress.show(u'CleanSurfaceBodies', u'處理 %v / %m ...', 0, total)

        try:
            for i, b in enumerate(surfaces):
                if progress.wasCancelled:
                    cancelled = True
                    break
                progress.progressValue = i + 1

                bname = 'body'
                try:
                    bname = b.name
                except Exception:
                    pass
                try:
                    if MODE == 'delete':
                        b.deleteMe()
                    elif MODE == 'hide':
                        b.isLightBulbOn = False
                    else:
                        b.isLightBulbOn = True
                    done += 1
                except Exception as e:
                    failed.append(u'{} ({})'.format(bname, e))
        finally:
            progress.hide()

        verb = {'delete': u'刪除', 'hide': u'隱藏', 'show': u'顯示'}[MODE]
        msg = header + [u'', u'{}：{} / {} 個'.format(verb, done, total)]
        if cancelled:
            msg.append(u'⚠️ 已被使用者取消，已完成的部分保留。')
        if failed:
            msg.append(u'')
            msg.append(u'失敗 {} 個（前 20 個）：'.format(len(failed)))
            msg.extend(failed[:20])
        if MODE == 'hide':
            msg += [u'',
                    u'現在轉一圈看模型有沒有缺角。',
                    u'沒問題 → MODE 改 \'delete\'；有缺 → MODE 改 \'show\' 還原。']

        ui.messageBox(u'\n'.join(msg), u'CleanSurfaceBodies')

    except Exception:
        if ui:
            ui.messageBox(u'Failed:\n{}'.format(traceback.format_exc()))
