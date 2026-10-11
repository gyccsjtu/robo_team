#!/usr/bin/env python3
"""Summarize a match from saved logs without subscribing to referee topics."""

import argparse
from collections import defaultdict
from pathlib import Path
import re


ACTORS = ("green", "blue", "brown", "white", "red", "red")
TAGS = ("green", "blue", "brown", "white", "red1", "red2")
SIM_TIME = re.compile(r"\[[A-Z]+\] \[[^,]+, ([0-9.]+)\]:")
REPORT = re.compile(r"\[REPORT_POINT\] tag=(green|blue|brown|white|red1|red2)\b")
TRACK = re.compile(r"派追踪：([^ ]+) → 目标 t([0-5])")
DISPATCH = re.compile(r"dispatch target=t([0-5]).*\buav=([^ ]+)")
FOUND = re.compile(r"^find actor_([0-5])$")
ELIMINATED = re.compile(r"^actor_([0-5]) is OK$")
TIME_USAGE = re.compile(r"^Time usage: ([0-9.]+)$")
REPORT_MAX_GAP_S = 1.0


def longest_report_run(stamps):
    """Longest bridge publish span without a referee-breaking time gap.

    This only measures our publication timing; the referee also checks the
    actor ID and sub-metre position, so it is not an elimination verdict.
    """
    if not stamps:
        return 0.0
    start = last = stamps[0]
    longest = 0.0
    for stamp in stamps[1:]:
        if stamp < last or stamp - last > REPORT_MAX_GAP_S:
            longest = max(longest, last - start)
            start = stamp
        last = stamp
    return max(longest, last - start)


def summarize(log_dir):
    judge_path = log_dir / "03_judge.log"
    bridge_path = log_dir / "04b_yolo_bridge.log"
    manager_path = log_dir / "06_swarm_manager.log"
    found = defaultdict(int)
    eliminated = set()
    eliminated_at = {}
    pending = None
    for line in judge_path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        match = FOUND.match(line)
        if match:
            found[int(match.group(1))] += 1
        match = ELIMINATED.match(line)
        if match:
            pending = int(match.group(1))
            eliminated.add(pending)
        match = TIME_USAGE.match(line)
        if match and pending is not None:
            eliminated_at[pending] = float(match.group(1))
            pending = None
    reports = defaultdict(list)
    for line in bridge_path.read_text(encoding="utf-8", errors="replace").splitlines():
        match, stamp = REPORT.search(line), SIM_TIME.search(line)
        if match and stamp:
            reports[match.group(1)].append(float(stamp.group(1)))
    assigned = defaultdict(set)
    if manager_path.exists():
        for line in manager_path.read_text(encoding="utf-8", errors="replace").splitlines():
            match = TRACK.search(line)
            if match:
                assigned[int(match.group(2))].add(match.group(1))
            match = DISPATCH.search(line)
            if match:
                assigned[int(match.group(1))].add(match.group(2))
    return found, eliminated, eliminated_at, reports, assigned


def render(summary):
    found, eliminated, eliminated_at, reports, assigned = summary
    lines = ["actor 颜色   发现触发次数  官方消除  消除仿真秒  派往的无人机"]
    for actor_id, color in enumerate(ACTORS):
        lines.append("%-5d %-6s %-9s %-9s %-11s %s" % (
            actor_id, color, found[actor_id],
            "是" if actor_id in eliminated else "否",
            "%.1f" % eliminated_at[actor_id] if actor_id in eliminated_at else "-",
            ",".join(sorted(assigned[actor_id])) or "-"))
    lines.append("桥接已记录的真实视觉报告（本队 red1/red2 槽不能直接对应 actor4/5）：")
    if not any(reports.values()):
        lines.append("该版本没有 REPORT_POINT 日志，不能据此判定目标未识别。")
    for tag in TAGS:
        stamps = reports[tag]
        longest_gap = max((b - a for a, b in zip(stamps, stamps[1:])),
                          default=0.0)
        lines.append("%-6s %4d 条  首次=%-7s 最后=%-7s 最大间隔=%.1fs 连续发报最长=%.1fs" % (
            tag, len(stamps), "%.1f" % stamps[0] if stamps else "-",
            "%.1f" % stamps[-1] if stamps else "-", longest_gap,
            longest_report_run(stamps)))
    lines.append("官方消除 %d/6" % len(eliminated))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_dir", nargs="?", type=Path,
                        default=Path(__file__).resolve().parents[1] / "logs/latest")
    args = parser.parse_args()
    try:
        print(render(summarize(args.log_dir)))
    except OSError as exc:
        parser.error("cannot read match logs: %s" % exc)


if __name__ == "__main__":
    main()
