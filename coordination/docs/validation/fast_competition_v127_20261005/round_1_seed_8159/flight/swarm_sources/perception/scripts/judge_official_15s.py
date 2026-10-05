#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按**官方裁判口径**判定一段快照录制的得分情况。

为什么需要它
------------
官方 `score_cal_master.py` 是两段式：
  L99  单帧报对（误差 <1m）即拿到 "发现分"（find_finish = 50）
  L137 自首次命中起**连续 15s 不断**才 delete_actor（再拿 100 分）
  L136 误差是与「接收瞬间的当前真值」比 ⇒ **链路滞后真的计入误差**
  L137 连续判定用 rospy.get_time()（仿真时间），间隔须 < 1.0s
⇒ 所以只看"平均误差"会严重高估成绩。**必须同时看「单帧达标占比」与「最长连续段」。**

本脚本把快照里的 dets 与同帧真值比对，直接算出这两个数，并给出 PASS/FAIL。

用法
----
  # 单轮
  python3 judge_official_15s.py /tmp/snap.jsonl --target a3 --speed 2.0

  # 多轮对照（同一个 --target / --speed 应用到所有文件）
  python3 judge_official_15s.py snap_v1.jsonl snap_c35.jsonl --target a3 --speed 2.0

  # 只想看全部类别、不关心实验条件
  python3 judge_official_15s.py /tmp/snap.jsonl

快照从哪来：`scripts/rec_snap.py`（订阅 /perception/debug_snapshot）。
字段依赖：每行 {t, truth:{actor:[x,y,...]}, dets:[{cls,dist:{actor:err},xyz:[x,y,z],coast}], cam:[x,y,...]}
"""
import argparse
import json
import math
import os
from collections import defaultdict

# ---- 官方常量（与 score_cal_master.py 对齐；改了要同步） ----
TAU = 2.0       # 检测坐标与某 actor 真值 <= 此距离即认作"报的是这个 actor"
THR = 1.0       # 官方定位误差判据：严格 < 1m
GAP = 1.0       # 官方：相邻两次报出间隔须 < 1s
NEED = 15.0     # 官方：连续 15s


def med(xs):
    s = sorted(xs)
    return s[len(s) // 2] if s else float("nan")


def under(xs, thr=THR):
    return 100.0 * sum(1 for x in xs if x < thr) / len(xs) if xs else float("nan")


def longest_run(seq, thr=THR, gap=GAP):
    """seq=[(t,err)] 升序；返回最长「err<thr 且相邻间隔<gap」的连续段时长"""
    best = 0.0
    cur_s = cur_e = prev = None
    for (t, e) in seq:
        if e >= thr:
            if cur_s is not None:
                best = max(best, cur_e - cur_s)
            cur_s = cur_e = prev = None
            continue
        if cur_s is None:
            cur_s = cur_e = t
        elif t - prev <= gap:
            cur_e = t
        else:
            best = max(best, cur_e - cur_s)
            cur_s = cur_e = t
        prev = t
    if cur_s is not None:
        best = max(best, cur_e - cur_s)
    return best


def dedup(seq):
    out = []
    for x in seq:
        if out and abs(out[-1][0] - x[0]) < 1e-9:
            continue
        out.append(x)
    return out


def speed_of(traj):
    """由真值轨迹反算速度（用仿真时间）—— 用来证明实验条件真的成立"""
    if len(traj) < 5:
        return float("nan")
    xs = [s[1] for s in traj]
    ys = [s[2] for s in traj]
    inst = []
    for i in range(1, len(traj)):
        dt = traj[i][0] - traj[i - 1][0]
        d = math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
        if dt > 1e-3:
            inst.append(d / dt)
    return med(inst)


def analyze(path, target=None, v_expect=None, tau=TAU):
    rows = [json.loads(l) for l in open(path, encoding="utf-8") if l.strip()]
    if len(rows) < 2:
        print("!! %s 行数不足 (%d)" % (path, len(rows)))
        return None

    # ---- 0) 实验条件自检 ----
    traj_all = defaultdict(list)
    for r in rows:
        for k, v in (r.get("truth") or {}).items():
            if v:
                traj_all[k].append((r["t"], v[0], v[1]))
    for k in traj_all:
        traj_all[k].sort()
    tgt = traj_all.get(target, []) if target else []
    span = rows[-1]["t"] - rows[0]["t"]
    if target:
        msg = "  [0] 条件自检  时长 %.1fs(仿真)   目标 %s 实测速度 %.3f m/s" % (
            span, target, speed_of(tgt))
        if v_expect is not None:
            msg += "（标称 %.1f）" % v_expect
        print(msg)
        if tgt:
            print("      目标轨迹 x=[%.1f, %.1f]  y=[%.1f, %.1f]"
                  % (min(s[1] for s in tgt), max(s[1] for s in tgt),
                     min(s[2] for s in tgt), max(s[2] for s in tgt)))
    else:
        print("  [0] 时长 %.1fs(仿真)   帧=%d" % (span, len(rows)))

    # ---- 1) 各类别的检测条目与官方判据 ----
    per_cls = defaultdict(list)
    for r in rows:
        for d in r.get("dets", []):
            cand = [(v, k) for k, v in (d.get("dist") or {}).items() if v is not None]
            if not cand:
                continue
            dmin, _ = min(cand)
            if dmin <= tau:
                cls = (d.get("cls") or "").rstrip("0123456789")
                per_cls[cls].append((r["t"], dmin, d.get("coast", 0)))

    print("  [1] 各类别的检测条目与官方判据")
    print("      %-7s %-26s %-10s %-12s" % ("类别", "条目(真实/含外推)", "误差中位", "单帧<1m"))
    result = {}
    for cls in sorted(per_cls):
        items = sorted(per_cls[cls])
        real = dedup([(t, dm) for (t, dm, c) in items if c == 0])
        allv = dedup([(t, dm) for (t, dm, c) in items])
        e_all = [x[1] for x in allv]
        r_real = longest_run(real)
        r_all = longest_run(allv)
        print("      %-7s %-26s %-10s %-12s"
              % (cls, "%d / %d" % (len(real), len(allv)),
                 "%.2fm" % med(e_all), "%.1f%%" % under(e_all)))
        print("              最长连续: 仅真实观测 %5.1fs | 含外推 %5.1fs  -> 官方15s %s"
              % (r_real, r_all, "PASS" if r_all >= NEED else "FAIL"))
        result[cls] = dict(n_real=len(real), n_all=len(allv),
                           med=med(e_all), under=under(e_all),
                           run_real=r_real, run_all=r_all)

    # ---- 2) 视野内召回（只对实验目标有意义） ----
    if target:
        exp = got = got_all = 0
        for r in rows:
            cam = r.get("cam")
            t = (r.get("truth") or {}).get(target)
            if not cam or not t:
                continue
            dx, dy = t[0] - cam[0], t[1] - cam[1]
            rng = math.hypot(dx, dy)
            az = math.degrees(math.atan2(dy, dx))
            if not (11.0 <= rng <= 60.0 and abs(az) <= 45.0):
                continue          # 几何可见区近似（fx=cx ⇒ 水平半视场 45°）
            exp += 1
            best = None
            for d in r.get("dets", []):
                dd = (d.get("dist") or {}).get(target)
                if dd is not None and dd <= tau:
                    c = d.get("coast", 0)
                    if best is None or c < best:
                        best = c
            if best is not None:
                got_all += 1
                if best == 0:
                    got += 1
        if exp:
            print("  [2] 目标 %s 视野内召回（水平 11~60m 且 |方位|<=45°）" % target)
            print("      期望可见 %d 帧次 -> 真实观测 %d (%.1f%%) | 含外推 %d (%.1f%%)"
                  % (exp, got, 100.0 * got / exp, got_all, 100.0 * got_all / exp))
        else:
            print("  [2] 目标 %s 未进入几何可见区（--target 或现场几何可能不对）" % target)
    return result


def main():
    ap = argparse.ArgumentParser(description="按官方 15s 判据分析快照录制")
    ap.add_argument("files", nargs="+", help="rec_snap.py 产出的 .jsonl")
    ap.add_argument("--target", default=None, help="实验目标 actor id，例如 a3（可选）")
    ap.add_argument("--speed", type=float, default=None,
                    help="目标标称速度，用于与真值实测值对照（可选）")
    ap.add_argument("--tau", type=float, default=TAU, help="归属阈值，默认 2.0 m")
    args = ap.parse_args()

    summary = []
    for f in args.files:
        print("=" * 78)
        print("%s   帧=%d" % (os.path.basename(f), sum(1 for _ in open(f, encoding="utf-8"))))
        res = analyze(f, args.target, args.speed, args.tau)
        if res is not None:
            summary.append((os.path.basename(f), res))
        print()

    if len(summary) > 1:
        print("=" * 78)
        print("汇总对照（官方 15s 判据；含外推口径 —— 与感知实际发布一致）")
        print("=" * 78)
        print("  %-24s %-8s %-8s %-10s %-12s %s"
              % ("文件", "类别", "条目", "误差中位", "最长连续", "判据"))
        for name, res in summary:
            for cls in sorted(res):
                rr = res[cls]
                print("  %-24s %-8s %-8d %-10s %-12s %s"
                      % (name, cls, rr["n_all"], "%.2fm" % rr["med"],
                         "%.1fs" % rr["run_all"],
                         "PASS" if rr["run_all"] >= NEED else "FAIL"))


if __name__ == "__main__":
    main()
