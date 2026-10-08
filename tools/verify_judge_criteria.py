#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""赛后验证：裁判硬判据「相邻播报间隔 ≤1s + 连续 15s + 误差 <1m」。

用法:
    python3 tools/verify_judge_criteria.py <log_dir> [--write]

数据源（<log_dir> 下）:
    03_judge.log         [FINAL] 行（streak 桶 / reset 原因 / 误差直方图）、
                         find actor_N、actor_N is OK、[SCORE_STATS] 周期行
    04b_yolo_bridge.log  [PUB_DBG] 桥端实际播报间隔、[GATE_DBG] 闸门拦截、
                         [EXTRAP_DBG] 外推量（注意：ed 是外推位移，不是裁判误差）
    start.log            [晚审计] 消息流检查、"裁判 ActorInfo 输入链路未通过审计"
                         （启动审计误杀证据）

判据出处: score_cal.py  err_threshold=1m / DETECTION_INTERVAL=1.0s /
          DETECTION_DURATION=15.0s（官方评分规则）
"""
import argparse
import os
import re
import sys
from collections import defaultdict

FINAL_RE = re.compile(
    r"\[FINAL\] score=(?P<score>-?\d+) find_finish=(?P<find>\d+) "
    r"uav_loss=(?P<loss>\d+) cb=(?P<cb>\d+) reset=(?P<reset>\d+) "
    r"\(dist/disc/no_pos=(?P<rd>\d+)/(?P<rdisc>\d+)/(?P<rnopos>\d+)\) "
    r"buckets\(lt1/1-5/5-10/10-15/ge15\)=(?P<b0>\d+)/(?P<b1>\d+)/(?P<b2>\d+)/"
    r"(?P<b3>\d+)/(?P<b4>\d+) "
    r"dist\(n/max/ge1\)=(?P<dn>\d+)/(?P<dmax>[\d.]+)m/(?P<dge1>\d+) "
    r"streak_max=(?P<smax>[\d.]+)s reason=(?P<reason>.*)")

STATS_RE = re.compile(
    r"\[SCORE_STATS\].*dist\(n/max/ge1\)=(?P<dn>\d+)/(?P<dmax>[\d.]+)m/(?P<dge1>\d+)")

PUB_RE = re.compile(
    r"\[PUB_DBG\] tag=(?P<tag>\w+) n=(?P<n>\d+) max_gap=(?P<max>[\d.]+)s "
    r"avg_gap=(?P<avg>[\d.]+)s")
GATE_RE = re.compile(r"\[GATE_DBG\] tag=(?P<tag>\w+) range=(?P<range>[\d.]+m|None)")
EXTRAP_RE = re.compile(r"\[EXTRAP_DBG\] tag=(?P<tag>\w+) .*ed=(?P<ed>[\d.]+)m")
FIND_RE = re.compile(r"^find actor_(\d+)")
OK_RE = re.compile(r"^actor_(\d+) is OK")
LATE_RE = re.compile(
    r"\[晚审计\] /actor_(?P<tag>\w+)_info 比赛尾段(?P<verdict>有消息流|仍无消息)")
AUDIT_KILL_RE = re.compile(r"裁判 ActorInfo 输入链路未通过审计")


def _read(path):
    try:
        with open(path, "r", errors="replace") as f:
            return f.read().splitlines()
    except OSError:
        return []


def analyze(log_dir):
    r = {
        "log_dir": log_dir,
        "final": None,            # 最后一条 [FINAL] 的 dict
        "stats_dist": {"n": 0, "max": 0.0, "ge1": 0},   # SCORE_STATS 聚合（fallback）
        "find": set(), "ok": set(),
        "pub": {},                # tag -> dict(n, max_gap, avg_gap)
        "gate": defaultdict(lambda: {"count": 0, "ranges": []}),
        "extrap": defaultdict(lambda: {"count": 0, "sum": 0.0, "max": 0.0}),
        "late": {},               # tag -> '有消息流'/'仍无消息'
        "audit_kill": False,
    }
    # ---- 03_judge.log ----
    for line in _read(os.path.join(log_dir, "03_judge.log")):
        m = FINAL_RE.search(line)
        if m:
            r["final"] = m.groupdict()
            continue
        m = STATS_RE.search(line)
        if m:
            r["stats_dist"]["n"] += int(m.group("dn"))
            r["stats_dist"]["ge1"] += int(m.group("dge1"))
            r["stats_dist"]["max"] = max(r["stats_dist"]["max"], float(m.group("dmax")))
            continue
        m = FIND_RE.match(line.strip())
        if m:
            r["find"].add(int(m.group(1)))
        m = OK_RE.match(line.strip())
        if m:
            r["ok"].add(int(m.group(1)))
    # ---- 04b_yolo_bridge.log ----
    for line in _read(os.path.join(log_dir, "04b_yolo_bridge.log")):
        m = PUB_RE.search(line)
        if m:
            tag = m.group("tag")
            prev = r["pub"].get(tag, {"n": 0, "max_gap": 0.0, "avg_gap": 0.0})
            prev.update({"n": int(m.group("n")),
                         "max_gap": max(prev["max_gap"], float(m.group("max"))),
                         "avg_gap": float(m.group("avg"))})
            r["pub"][tag] = prev
            continue
        m = GATE_RE.search(line)
        if m:
            tag = m.group("tag") or m.group("tag_none")
            g = r["gate"][tag]
            g["count"] += 1
            if m.group("range"):
                g["ranges"].append(float(m.group("range").rstrip("m")))
            continue
        m = EXTRAP_RE.search(line)
        if m:
            e = r["extrap"][m.group("tag")]
            ed = float(m.group("ed"))
            e["count"] += 1
            e["sum"] += ed
            e["max"] = max(e["max"], ed)
    # ---- start.log（可能在本目录，也可能在兄弟 run_match_<时间戳> 目录）----
    start_candidates = [os.path.join(log_dir, "start.log")]
    base = os.path.basename(os.path.abspath(log_dir))
    if base.startswith("logs_"):
        import glob as _glob
        start_candidates += _glob.glob(os.path.join(
            os.path.dirname(os.path.abspath(log_dir)),
            "run_match_" + base[len("logs_"):] + "*", "start.log"))
    start_lines = []
    for cand in start_candidates:
        start_lines = _read(cand)
        if start_lines:
            break
    for line in start_lines:
        m = LATE_RE.search(line)
        if m:
            r["late"][m.group("tag")] = m.group("verdict")
        if AUDIT_KILL_RE.search(line):
            r["audit_kill"] = True
    return r


def _verdict_line(name, ok, detail):
    tag = "PASS" if ok else "FAIL"
    return "[%-4s] %s %s" % (tag, name, detail)


def report(r):
    out = []
    w = out.append
    w("=" * 72)
    w("裁判硬判据赛后验证: %s" % r["log_dir"])
    w("判据: 相邻播报间隔 ≤1.0s ＋ 连续 15.0s ＋ 上报误差 <1m（score_cal.py）")
    w("=" * 72)

    fin = r["final"]
    # ---- 判据 1: 桥端播报连续性 ----
    w("")
    w("◆ 判据1 播报连续性（桥端 [PUB_DBG]，间隔 ≤1.0s）")
    if r["pub"]:
        v_ok = True
        for tag in sorted(r["pub"]):
            p = r["pub"][tag]
            ok = p["max_gap"] <= 1.0
            v_ok = v_ok and ok
            w("  %s: n=%d max_gap=%.2fs avg_gap=%.2fs %s"
              % (tag, p["n"], p["max_gap"], p["avg_gap"], "OK" if ok else "超1s!"))
        w(_verdict_line("判据1", v_ok, "全部 tag max_gap ≤1.0s" if v_ok else "存在断流窗口"))
    else:
        w(_verdict_line("判据1", False, "无 [PUB_DBG] 数据（旧版桥或全程零播报）"))

    # ---- 判据 2: 连续 15s ----
    w("")
    w("◆ 判据2 连续 15s（裁判端 streak）")
    if fin:
        smax = float(fin["smax"])
        ge15 = int(fin["b4"])
        ok = smax >= 15.0 or ge15 >= 1
        w("  streak_max=%.1fs streak_ge15=%d 桶(lt1/1-5/5-10/10-15/ge15)=%s"
          % (smax, ge15, "/".join(fin[k] for k in ("b0", "b1", "b2", "b3", "b4"))))
        w("  reset=%s (distance=%s discontinuous=%s no_pos=%s)"
          % (fin["reset"], fin["rd"], fin["rdisc"], fin["rnopos"]))
        w(_verdict_line("判据2", ok, "streak_max=%.1fs" % smax))
    else:
        elim = len(r["ok"])
        if elim:
            w(_verdict_line("判据2", True, "无 [FINAL] 但有 %d 个 actor 被消除（间接证明攒满 15s）" % elim))
        else:
            w(_verdict_line("判据2", False, "无 [FINAL] 且零消除（进程被杀或旧版 score_cal）"))

    # ---- 判据 3: 误差 <1m ----
    w("")
    w("◆ 判据3 误差 <1m（裁判端 上报 vs Gazebo 真值）")
    if fin and int(fin["dn"]) > 0:
        dn, dge1, dmax = int(fin["dn"]), int(fin["dge1"]), float(fin["dmax"])
        ratio = 100.0 * dge1 / dn
        w("  样本 n=%d 越界(≥1m)=%d (%.1f%%) 全程最远=%.2fm" % (dn, dge1, ratio, dmax))
        w(_verdict_line("判据3", dge1 == 0,
                        "0 次越界" if dge1 == 0 else
                        "越界 %d 次（每次都把 15s 计时清零）" % dge1))
    elif r["stats_dist"]["n"] > 0:
        sd = r["stats_dist"]
        w("  （无 [FINAL]，用 [SCORE_STATS] 聚合）样本 n=%d 越界=%d 最远=%.2fm"
          % (sd["n"], sd["ge1"], sd["max"]))
        w(_verdict_line("判据3", sd["ge1"] == 0, "见上"))
    else:
        w(_verdict_line("判据3", False, "无误差样本（裁判一条 ActorInfo 都没收到）"))

    # ---- 消除结果 ----
    w("")
    w("◆ 消除结果: find=%s eliminated=%s"
      % (sorted(r["find"]) if r["find"] else (fin["find"] if fin else "[]"),
         sorted(r["ok"]) if r["ok"] else "[]"))
    if fin:
        w("  score=%s uav_loss=%s reason=%s" % (fin["score"], fin["loss"], fin["reason"]))

    # ---- 根因线索 ----
    w("")
    w("◆ 根因线索")
    if r["audit_kill"]:
        w("  ⚠ start.log 有『裁判 ActorInfo 输入链路未通过审计』——本次运行被启动审计"
          " partial_fail 杀掉（2026-10-06 前的版本：发布者计数 grep 恒 0 + 启动期"
          "必然无消息，双误报）。")
    if r["gate"]:
        for tag in sorted(r["gate"]):
            g = r["gate"][tag]
            if g["ranges"]:
                w("  [GATE_DBG] %s 拦截 %d 次 range 中位=%.1fm 最远=%.1fm"
                  % (tag, g["count"], sorted(g["ranges"])[len(g["ranges"]) // 2],
                     max(g["ranges"])))
            else:
                w("  [GATE_DBG] %s 拦截 %d 次 range=None" % (tag, g["count"]))
        w("  → 闸门拦截说明检测到了但距离 >8m（enter 阈值），UAV 需抵近才能开始播报")
    if r["late"]:
        none_tags = [t for t, v in r["late"].items() if v == "仍无消息"]
        w("  [晚审计] %d/%d 路尾段仍无消息: %s"
          % (len(none_tags), len(r["late"]), ",".join(sorted(none_tags)) or "无"))
    if r["extrap"]:
        for tag in sorted(r["extrap"]):
            e = r["extrap"][tag]
            w("  [EXTRAP_DBG] %s n=%d 外推 ed 平均=%.2fm 最大=%.2fm（>1m 的外推会越界）"
              % (tag, e["count"], e["sum"] / e["count"], e["max"]))
    if not (r["gate"] or r["late"] or r["extrap"] or r["audit_kill"]):
        w("  （无异常线索）")

    w("")
    w("=" * 72)
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log_dir")
    ap.add_argument("--write", action="store_true", help="报告写入 <log_dir>/verify_judge_criteria.txt")
    args = ap.parse_args()
    if not os.path.isdir(args.log_dir):
        print("log_dir 不存在: %s" % args.log_dir, file=sys.stderr)
        return 2
    text = report(analyze(args.log_dir))
    print(text)
    if args.write:
        out_path = os.path.join(args.log_dir, "verify_judge_criteria.txt")
        with open(out_path, "w") as f:
            f.write(text + "\n")
        print("已写入: %s" % out_path, file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
