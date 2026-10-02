#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""seed=12 专项诊断：t4 从未发现，是否因为其他目标占用了过多 UAV。

问题（用户指定）：
  * t4 从未发现（known=False, progress=0.0%）是不是因为其他目标占用了
    过多 UAV，导致没有搜索兵力去扫 t4 所在的区域？

方法
----
复用 mission_time.Sim 本体（不修改已验证代码），仅通过 on_step 钩子逐帧
记录，并在局末叠加一个"上帝视角"对照：

  1. 逐帧时间线：每个已知目标当时占用几架 UAV（observers 集合）；
     以及每架 UAV 本帧是 tracker 还是 search（_engaged() 归属）。
  2. 每目标的关键时刻：discover / first_assign / 撤编 / 瞬移 / 消除。
  3. 无人值守时段：对每个 known 目标，统计 known 后没有任何 UAV 是它
     observer 的累计时长。
  4. 搜索兵力量化：每帧"空闲/搜索"机数（不在任何 observers 里的机数）。
     t4 未发现期间搜索兵力是否长期 =0 或很低？
  5. 上帝视角对照（关键）：t4 位置自始至终在移动。逐帧算"离 t4 最近的
     搜索机距离"。如果搜索机经常在 t4 的 SENSE_R=20m 附近但 known 仍为
     False，说明不是兵力问题而是搜索路径没扫到那片（覆盖盲区）；
     如果搜索机离 t4 一直很远，说明搜索兵力确实被抽调。
  6. 末尾回答"如果能预知 t4 位置，用当前兵力是否来得及消除"。

用法
----
    export ROBOCUP_WS="C:/Users/gycsjtu/AppData/Local/Temp/rcws"
    python diag_seed12.py [--seed 12] [--max-t 300] [--out out.txt]
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mission_time as MT
from strategy_compare import MapGrid, SENSE_R, DT

# 复现推荐配置（与 n=40 正式轮一致）
MT.SEP_TARGET_M = 3.0
MT.SEP_PUSH_MAX_DEG = 30.0
MT.SEARCH_PHASE = 6
MT.FRIEND_SAFE = 1.5
MT.FRIEND_K = 3.0
MT.FRIEND_MAX = 2.0
MT.SLOT_MODE = "los_plan"

import task_allocator as TA


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=12)
    ap.add_argument("--max-t", type=float, default=300.0)
    ap.add_argument("--out", default="diag_seed12.txt")
    ap.add_argument("--fr", type=float, default=1.5,
                    help="友机排斥强度 FRIEND_SAFE（1.5 推荐 / 3.0 复现旧 seed=12 失败）")
    ap.add_argument("--phase", type=int, default=6,
                    help="搜索相位错开（SEARCH_PHASE）")
    ap.add_argument("--tid", default=None,
                    help="要重点分析的未发现目标 id（默认自动找第一个 never-discovered）")
    args = ap.parse_args()
    MT.FRIEND_SAFE = args.fr
    MT.SEARCH_PHASE = args.phase

    g = MapGrid(TA.METADATA)
    sim = TA.AllocSim(g, strategy="dynamic", targets=6, seed=args.seed)

    # ---- 逐帧记录 ----
    hist = []           # (t, {tid: n_obs}, n_search, n_track, engaged_map)
    t4_prox = []        # (t, d_nearest_uav_to_t4, d_nearest_search_to_t4)
    unassigned = {}     # tid -> 累计无人值守时长（仅 known 后累计）
    was_known = {}
    known_since = {}    # tid -> 首次 known 时刻
    tgt_pos_log = {}    # tid -> [(t, x, y)]

    def hook(s):
        eng = s._engaged()
        pending = [tid for tid in s.tr.pending()
                   if tid not in s.done and s.tgt[tid]["known"]]
        # 每目标占用机数
        n_obs = {}
        for tid in pending:
            n_obs[tid] = len(s.tgt[tid].get("observers", []))
        # 搜索机 = 不在任何 observers 里的机
        busy = set()
        for tid in pending:
            busy |= set(s.tgt[tid].get("observers", []))
        search_uavs = [u for u in s.uavs if u not in busy]
        # 记录 t4 附近
        t4 = s.tgt.get("t4")
        if t4 is not None:
            d_near_uav = min(
                math.hypot(s.uavs[u][0] - t4["x"], s.uavs[u][1] - t4["y"])
                for u in s.uavs) if s.uavs else float("nan")
            d_near_search = min(
                math.hypot(s.uavs[u][0] - t4["x"], s.uavs[u][1] - t4["y"])
                for u in search_uavs) if search_uavs else float("nan")
            t4_prox.append((s.t, d_near_uav, d_near_search, len(search_uavs)))
        # 无人值守（known 后才累计）
        for tid in pending:
            if tid not in known_since:
                known_since[tid] = s.t
                unassigned.setdefault(tid, 0.0)
            if n_obs[tid] == 0:
                unassigned[tid] = unassigned.get(tid, 0.0) + DT
        hist.append((s.t, n_obs, len(search_uavs), len(busy), eng))
        # 记录 t4 位置轨迹
        for tid in s.tgt:
            tgt_pos_log.setdefault(tid, []).append(
                (s.t, s.tgt[tid]["x"], s.tgt[tid]["y"], s.tgt[tid]["known"]))

    sim.run(args.max_t, on_step=lambda s: (s._observe_metrics(), hook(s)))

    lines = []
    out = sys.stdout
    if args.out:
        out = open(args.out, "w", encoding="utf-8")
    pr = lambda *a: print(*a, file=out)

    pr("=" * 96)
    pr("seed=%d 专项诊断：t4 从未发现是否因其他目标占用过多 UAV" % args.seed)
    pr("配置 SEP=3 PUSH=30 PHASE=%d FR=%s | 完成 %d/6"
       % (args.phase, ("%.1f" % args.fr), len(sim.done)))
    pr("=" * 96)

    # ---- 各目标关键时刻 ----
    pr("")
    pr("[各目标关键时刻]")
    pr("  %-4s %-9s %-9s %-9s %-9s %-6s %s"
       % ("tid", "known", "assign", "unassigned_s", "elim", "evade", "初始位置"))
    for tid in sorted(sim.tgt):
        t = sim.tr.targets.get(tid)
        elim = None
        for (tt, kind, eid) in sim.events:
            if eid == tid and kind == "eliminated":
                elim = tt
        disc = known_since.get(tid)
        fa = sim._first_assign_time.get(tid)
        ev = sim._first_detect_time.get(tid)  # not used
        # find evade time
        evade_t = None
        for (tt, kind, eid) in sim.events:
            if eid == tid and kind == "evade":
                evade_t = tt
        # 初始位置（第一帧 tgt_pos_log）
        initp = tgt_pos_log[tid][0][1:3] if tgt_pos_log.get(tid) else (None, None)
        pr("  %-4s %-9s %-9s %-9s %-9s %-6s (%.1f, %.1f)"
           % (tid,
              ("%.1fs" % disc) if disc is not None else "NEVER",
              ("%.1fs" % fa) if fa is not None else "-",
              ("%.1fs" % unassigned.get(tid, 0.0)),
              ("%.1fs" % elim) if elim is not None else "-",
              ("%.1fs" % evade_t) if evade_t is not None else "-",
              initp[0] if initp[0] is not None else float("nan"),
              initp[1] if initp[1] is not None else float("nan")))

    # ---- 搜索兵力时间线（每 20s 采样一段）----
    pr("")
    pr("[搜索兵力量化] 每 20s 采样：本帧总机=6，跟踪中(忙)机数，搜索/空闲机数，"
       "以及离 t4 最近的搜索机距离")
    pr("  时间    忙机  搜索机  t4最近搜索机(m)  t4最近任意机(m)  t4 known")
    known_t4 = sim.tgt["t4"]["known"] if "t4" in sim.tgt else False
    last_t4_known = False
    for i in range(0, len(hist), int(20.0 / DT)):
        t, n_obs, n_search, n_track, eng = hist[i]
        # find t4 prox at this t (closest)
        dns = None
        dna = None
        for (tt, d1, d2, ns) in t4_prox:
            if abs(tt - t) < DT:
                dns = d2
                dna = d1
                break
        # t4 known at this time
        tk = False
        for (tt, x, y, k) in tgt_pos_log.get("t4", []):
            if abs(tt - t) < DT:
                tk = k
                break
        pr("  %6.1f  %3d   %3d     %8.1f      %8.1f       %s"
           % (t, n_track, n_search,
              dns if dns is not None else float("nan"),
              dna if dna is not None else float("nan"),
              "known" if tk else "未发现"))
        last_t4_known = tk

    # ---- 上帝视角可达性 ----
    pr("")
    pr("[上帝视角：若 t4 位置始终已知，当前兵力是否来得及消除？]")
    # 搜索机速度 = 5 m/s；t4 移动速度 ≤2 m/s（逃跑）或 1 m/s（游走）
    # 保守估计：最近搜索机到 t4 距离 / (5-2) 的接近速率
    if t4_prox:
        # 用 t4 已知前的最近搜索机距离最小值
        min_dns = min((p[1] for p in t4_prox), default=float("nan"))
        min_dna = min((p[2] for p in t4_prox), default=float("nan"))
        # 最小接近时间（若直线去追，追及速度 ≈ 5-2=3 m/s）
        t_reach = min_dns / 3.0 if not math.isnan(min_dns) else float("nan")
        pr("  t4 全程离最近搜索机的最小距离: %.1f m" % min_dns)
        pr("  t4 全程离最近任意机的最小距离: %.1f m" % min_dna)
        pr("  （若预知位置，追及所需时间 ≈ %.1f s，需 < 300s 且 < 30s 防瞬移）"
           % t_reach)
        pr("  解释：若该最小距离远大于 SENSE_R=20m，则搜索机从未靠近过 t4 区域"
           " → 兵力不足/搜索盲区；")
        pr("        若最小距离在 20m 内但 t4 仍未被发现 → 是搜索路径覆盖盲区，"
           "非兵力问题。")

    # ---- 每帧是否所有机都被占用（t4 未发现期间）----
    pr("")
    pr("[t4 未发现期间搜索兵力] 统计 t4 known=False 期间，搜索机数为 0 的帧占比")
    t4_unk_frames = 0
    t4_unk_zero_search = 0
    for i, (t, n_obs, n_search, n_track, eng) in enumerate(hist):
        tk = False
        for (tt, x, y, k) in tgt_pos_log.get("t4", []):
            if abs(tt - t) < DT:
                tk = k
                break
        if not tk:
            t4_unk_frames += 1
            if n_search == 0:
                t4_unk_zero_search += 1
    if t4_unk_frames:
        pr("  t4 未发现帧数: %d (%.0f%% of 全程)"
           % (t4_unk_frames, 100.0 * t4_unk_frames / len(hist)))
        pr("  其中搜索机=0 的帧: %d (%.1f%%) —— 该比例高说明确实无搜索兵力"
           % (t4_unk_zero_search, 100.0 * t4_unk_zero_search / t4_unk_frames))
    else:
        pr("  t4 全程已被发现（known）")

    if args.out:
        out.close()
    print("done -> %s" % args.out)


if __name__ == "__main__":
    main()
