#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线重放：把**线上真实抓到的** target_report 流量喂给队友的协同核心。

为什么值得单独做：
  在 VM 上只盯着"话题有没有流量"是看不出核心到底消费了没有的 ——
  `coordination_node._submit()` 对不合规载荷只打一条 logwarn 就丢，
  不崩不退出。本脚本绕开 ROS，直接调队友自己那条入口
  `CoreBus.submit(source_id, kind, data, sim_s, wall_s)`（与节点逐行同路径），
  于是可以精确回答两件事：

  1. 核心到底收下没有？（被拒会抛 CoordinationError，逐条可数）
  2. 收下之后**做了什么**？core._target() 的第一句是
         if d['confidence'] < 1.: return   # v1 consumes confirmed reports
     也就是说 confidence < 1.0 的报告只进 observations 台账、
     **不会更新任务坐标**。本脚本把同一份流量跑两遍
     （原值 / 强制 1.0），把这条语义差变成可对比的数字。

用法：
    python3 replay_target_report.py <traffic.jsonl>
"""
import copy
import json
import math
import os
import sys

REPO_SRC = os.environ.get(
    "ROBO_REPO_SRC",
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "..", "robocup_repo", "src", "robocup_navigation", "src"))
sys.path.insert(0, os.path.abspath(REPO_SRC))

from robocup_navigation.coordination_adapter import CoreBus          # noqa: E402
from robocup_navigation.coordination.protocol import CoordinationError  # noqa: E402

# 与 coordination_node.DEV_LIMITS 逐项一致（核心要求键集合完全匹配）
LIMITS = dict(min_separation_m=1.0, arrival_tolerance_m=0.15, mission_timeout_s=180.0,
              deadlock_timeout_s=60.0, max_monitor_gap_s=1.1, state_timeout_s=3.0,
              lease_s=3.0, stop_speed_mps=0.05, required_clearance_m=0.6,
              tracking_bound_m=0.2, position_tolerance_m=0.05, nominal_speed_mps=0.3)


def load(path):
    recs = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                recs.append(json.loads(line))
    return recs


def build(recs):
    """按真实出现过的 target_id 建任务清单（核心要求 tasks 非空且 target_id 唯一）。"""
    tids = sorted({r["data"]["target_id"] for r in recs})
    tasks = {}
    for t in tids:
        first = next(r["data"] for r in recs if r["data"]["target_id"] == t)
        tasks["task_%s" % t] = {"target_id": t, "xyz": list(first["xyz"])}
    return tasks


def run(recs, conf_override=None, label=""):
    tasks = build(recs)
    fleet = ["uav_1"]
    roles = {u: ["VEHICLE_STATE", "COMMAND_ACK"] for u in fleet}
    roles.update(planner=["ROUTE_OFFER"], verifier=["RESOURCE_CLEAR"],
                 clock=["TICK"], perception=["TARGET_REPORT"])
    t0 = recs[0]["t"] if recs else 0.0
    bus = CoreBus("replay", fleet, tasks, dict(LIMITS), roles, "replay",
                  start_sim_s=t0, start_wall_s=t0)

    rejected = {}
    for r in recs:
        d = dict(r["data"])
        if conf_override is not None:
            d["confidence"] = conf_override
        try:
            bus.submit("perception", "TARGET_REPORT", d, r["t"], r["t"])
        except CoordinationError as exc:
            key = str(exc).split(":")[0]
            rejected[key] = rejected.get(key, 0) + 1
        except Exception as exc:                                     # noqa: BLE001
            key = type(exc).__name__
            rejected[key] = rejected.get(key, 0) + 1

    core = bus.core
    obs = len(core.observations)
    # 任务坐标有没有被报告推动
    moved = {}
    for tid, t in core.tasks.items():
        last = None
        for r in recs:
            if r["data"]["target_id"] == t["target_id"]:
                last = r["data"]["xyz"]
        moved[tid] = (last is not None and
                      math.dist(t["xyz"], last) < 1e-9)
    print("  [%s] 提交 %d 条 | 被拒 %d %s | observations=%d"
          % (label, len(recs), sum(rejected.values()),
             dict(rejected) if rejected else "", obs))
    for tid, ok in moved.items():
        print("        %-14s 任务坐标已跟随报告 = %s   xyz=%s"
              % (tid, "是 ✅" if ok else "否", [round(v, 2) for v in core.tasks[tid]["xyz"]]))
    return dict(rejected=sum(rejected.values()), obs=obs, moved=moved)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    path = sys.argv[1]
    recs = load(path)
    if not recs:
        print("流量文件是空的：%s" % path)
        return 2

    cs = sorted(float(r["data"]["confidence"]) for r in recs)
    tids = {}
    for r in recs:
        tids[r["data"]["target_id"]] = tids.get(r["data"]["target_id"], 0) + 1

    print("=" * 74)
    print("流量文件: %s" % os.path.abspath(path))
    print("条数: %d   跨度: %.1f s" % (len(recs), recs[-1]["t"] - recs[0]["t"]))
    print("target_id 分布: %s" % tids)
    print("confidence: min %.3f / 中位 %.3f / max %.3f" % (cs[0], cs[len(cs) // 2], cs[-1]))
    print("-" * 74)
    print("A) 按**线上原样**重放：")
    a = run(recs, None, "原样")

    print("B) 把同一份流量的 confidence 全改成 1.0 重放：")
    b = run(recs, 1.0, "conf=1.0")

    print("-" * 74)
    print("结论：")
    if a["rejected"] == 0 and b["rejected"] == 0:
        print("  ✅ 两种口径都被核心**全部接受** ⇒ 形状合规，接口连通。")
    else:
        print("  ❌ 存在被拒条目，形状或取值仍有问题。")
    drove_a = any(a["moved"].values())
    drove_b = any(b["moved"].values())
    if not drove_a and drove_b:
        print("  ⚠ **线上真实 confidence 不会驱动核心的任务坐标**，")
        print("     只有 confidence=1.0 才会（core._target 第一句就是这个判据）。")
        print("     ⇒ 需与队友确认：由感知对'已确认'目标发 1.0，还是核心改判据。")
    elif drove_a:
        print("  ✅ 线上原样 confidence 就能驱动任务坐标。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
