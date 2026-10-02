#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""大样本判定：SEP 让位到底是"真的掉成功率"还是 n=10 的噪声？

同时报两个成功率口径：
  15s成功率   = 该局 6 个目标**全部**完成（全有全无，n=10 时噪声极大）
  目标级完成  = 完成的**目标数 / 总目标数**（更细、方差小得多）
判定标准：两个口径的方向是否一致。若 per-seed 掉而 target-level 几乎不动，
就说明 per-seed 那个 −30 点是全有全无放大的噪声。

命令行：python run_sep_big.py <sep> <outfile> <n_seeds>
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

sep = float(sys.argv[1])
out = sys.argv[2]
nmax = int(sys.argv[3]) if len(sys.argv) > 3 else 24

import mission_time as MT
import task_allocator as TA
from strategy_compare import MapGrid

MT.SEP_TARGET_M = sep
MT.SEP_PUSH_MAX_DEG = 30.0
MT.SLOT_MODE = "chase"

D_HIT = 1.5
D_TIGHT = 0.5


def run(seeds, targets=6, max_t=300.0):
    g = MapGrid(TA.METADATA)
    full = 0                    # 该局全部目标完成
    tgt_done = 0
    tgt_total = 0
    n_ev = 0
    min_all = []
    stuck_all = []

    for sd in seeds:
        sim = TA.AllocSim(g, strategy="dynamic", targets=targets, seed=sd)
        close = []

        def hook(s, close=close):
            eng = s._engaged()
            pos = {u: tuple(p) for u, p in s.uavs.items()}
            for ui in sorted(pos):
                for vi in sorted(pos):
                    if vi <= ui:
                        continue
                    dd = math.hypot(pos[ui][0] - pos[vi][0],
                                    pos[ui][1] - pos[vi][1])
                    if dd < D_HIT and eng.get(ui) is not None and eng.get(ui) == eng.get(vi):
                        close.append((s.t, ui, vi, dd))
        sim.run(max_t, on_step=lambda s: (s._observe_metrics(), hook(s)))

        done = len(sim.done)
        tgt_done += done
        tgt_total += targets
        if done == targets:
            full += 1
        n_ev += len(close)

        mn = min((d for _t, _u, _v, d in close), default=float("nan"))
        min_all.append(mn)
        bypair = {}
        for t, u, v, d in close:
            if d < D_TIGHT:
                bypair.setdefault((u, v), []).append(t)
        longest = 0.0
        for pair, ts in bypair.items():
            ts.sort()
            r0 = 0.0
            for i in range(1, len(ts)):
                r0 = r0 + MT.DT if abs(ts[i] - ts[i - 1] - MT.DT) < 1e-6 else 0.0
                longest = max(longest, r0 + MT.DT)
        stuck_all.append(longest)

        print("  seed=%-3d 完成 %d/%d  近距 %4d  最小 %.2f m  伴飞 %.1f s"
              % (sd, done, targets, len(close), mn, longest), flush=True)

    n = len(seeds)
    # nan 感知的 min：某局若**无同目标近距事件**，该局最小值是 nan。
    # 直接 min(list) 会被 nan 污染（nan 的比较全为 False，min 会把它当结果
    # 返回），于是"某一局零事件"会被误报成"全局最小间距 nan"。
    _vs = [v for v in min_all if v == v]
    print("-" * 96)
    print("汇总（n=%d, 每局 %d 目标）SEP_TARGET_M=%.1f" % (n, targets, sep))
    print("  15s成功率(全有全无) : %6.1f%% (%d/%d)" % (100.0 * full / n, full, n))
    print("  目标级完成率        : %6.1f%% (%d/%d)"
          % (100.0 * tgt_done / tgt_total, tgt_done, tgt_total))
    print("  同目标近距事件      : %6d 次 (平均每局 %.0f)" % (n_ev, n_ev / n))
    print("  最小机间距          : %6.2f m (有事件的局 %d/%d)"
          % (min(_vs) if _vs else float("nan"), len(_vs), n))
    print("  最长<0.5m 伴飞      : %6.1f s" % max(stuck_all, default=0.0))
    return dict(full=full, n=n, tgt_done=tgt_done, tgt_total=tgt_total,
                n_ev=n_ev, mind=min(_vs) if _vs else float("nan"),
                stuck=max(stuck_all, default=0.0))


if __name__ == "__main__":
    with open(out, "w", encoding="utf-8") as fh:
        real = sys.stdout
        sys.stdout = fh
        try:
            print("########## SEP=%.1f  seeds 1..%d ##########" % (sep, nmax),
                  flush=True)
            run(list(range(1, nmax + 1)))
        finally:
            sys.stdout = real
    print("SEP=%.1f 完成 -> %s" % (sep, out), flush=True)
