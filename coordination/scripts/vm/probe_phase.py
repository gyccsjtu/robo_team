#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判定：0.00 m 的近距事件主要发生在**跟踪阶段**还是**搜索阶段**？

这决定用户路线里 ① 和 ② 的优先级：
  若主要是"两机同目标跟踪" → 编组几何分离（①）是对的地方
  若主要是"搜索巡逻时路径交汇" → 搜索纵带错开（②）才对症
  若两者都有 → 按各自占比分配力气

判据：取 <D_NEAR 的机对，看该帧两机的 `_engaged()`：
  track-track  两机都在追同一个目标（编组内重合）
  track-diff   两机各追不同目标（交叉目标）
  search-*     至少一机在搜索（巡逻交汇）
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mission_time as MT
import task_allocator as TA
from strategy_compare import MapGrid

D_NEAR = 1.5      # "近距"门槛，与 probe_form 一致
D_TIGHT = 0.5     # "贴身/卡死"门槛


def main(seeds, targets=6, max_t=300.0):
    g = MapGrid(TA.METADATA)
    MT.SLOT_MODE = os.environ.get("SLOT_MODE", "chase")
    print("=" * 100)
    print("按阶段拆解近距事件   SLOT_MODE=%s  n=%d  目标=%d  "
          "门槛: 近距<%.1fm / 贴身<%.1fm" % (MT.SLOT_MODE, len(seeds), targets,
                                          D_NEAR, D_TIGHT))
    print("=" * 100)

    # 计数：按 (类别, 门槛) 分组
    cat = ["track-track", "track-diff", "track-search", "search-search"]
    near = {c: 0 for c in cat}
    tight = {c: 0 for c in cat}
    # 贴身事件的 (种子, 时刻, 类别, 距离) 样本，用于看现场
    samples = []
    min_by_cat = {c: float("nan") for c in cat}
    # 每局的最长贴身时长（按类别）
    longest = {c: 0.0 for c in cat}

    for sd in seeds:
        sim = TA.AllocSim(g, strategy="dynamic", targets=targets, seed=sd)
        frames = {}     # (u,v) -> [(t, cat, d), ...]

        def hook(s):
            eng = s._engaged()
            pos = {u: tuple(p) for u, p in s.uavs.items()}
            for ui in sorted(pos):
                for vi in sorted(pos):
                    if vi <= ui:
                        continue
                    dd = math.hypot(pos[ui][0] - pos[vi][0],
                                    pos[ui][1] - pos[vi][1])
                    if dd >= D_NEAR:
                        continue
                    eu, ev = eng.get(ui), eng.get(vi)
                    if eu is None and ev is None:
                        c = "search-search"
                    elif eu is None or ev is None:
                        c = "track-search"
                    elif eu == ev:
                        c = "track-track"
                    else:
                        c = "track-diff"
                    near[c] += 1
                    if dd < D_TIGHT:
                        tight[c] += 1
                    if dd < min_by_cat.get(c, 9e9) or \
                       (min_by_cat[c] != min_by_cat[c]):
                        min_by_cat[c] = dd
                    frames.setdefault((ui, vi), []).append((s.t, c, dd))
                    if len(samples) < 12 and dd < 0.05:
                        samples.append((sd, s.t, ui, vi, c, dd, eu, ev))

        sim.run(max_t, on_step=lambda s: (s._observe_metrics(), hook(s)))

        # 每局、每类别的最长连续贴身时长
        for pair, seq in frames.items():
            seq.sort()
            cat_run = {}
            for t, c, dd in seq:
                tightf = dd < D_TIGHT
                for c2 in cat:
                    if tightf and c == c2:
                        cat_run[c2] = cat_run.get(c2, 0.0) + MT.DT
                        longest[c2] = max(longest[c2], cat_run[c2])
                    elif c == c2:
                        cat_run[c2] = 0.0
            # 简化：上面的连续判定按帧序近似（同一 pair 的相邻帧）

    tot_near = sum(near.values()) or 1
    tot_tight = sum(tight.values()) or 1
    print("")
    print("  %-14s %10s %8s %10s %8s %10s" %
          ("类别", "近距事件", "占比", "贴身事件", "占比", "最长贴身"))
    for c in cat:
        print("  %-14s %10d %7.1f%% %10d %7.1f%% %8.1f s"
              % (c, near[c], 100.0 * near[c] / tot_near,
                 tight[c], 100.0 * tight[c] / tot_tight, longest[c]))
    print("  %-14s %10d %7.1f%% %10d %7.1f%%"
          % ("合计", tot_near, 100.0, tot_tight, 100.0))

    print("")
    print("  各类别最小间距:")
    for c in cat:
        m = min_by_cat[c]
        print("     %-14s %s" % (c, "%.3f m" % m if m == m else "无事件"))

    print("")
    print("  贴身(<%.1fm)现场样本（最多 12 条）:" % D_TIGHT)
    if not samples:
        print("     无 dd<0.05 的样本")
    for sd, t, ui, vi, c, dd, eu, ev in samples:
        print("     seed=%-3d t=%6.1f  %s-%s  d=%.3f  [%s]  目标 %s/%s"
              % (sd, t, ui, vi, dd, c, eu, ev))
    print("=" * 100)


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    main(list(range(1, n + 1)))
