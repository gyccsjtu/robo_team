#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成合规的基础 metadata（空障碍图）—— 不读任何地图真值。

合规重构（2026-10-07，规则 §2.4/§2.5「无预读随机地图」）：
  旧实现解析官方 black_box.txt（每局随机化的建筑矩形真值）生成障碍图 +
  占用栅格 —— 比赛启动前预读地图 = 违规。
  新实现：本脚本生成**静态空图**，只含规则定值与队自定常量：
    - bounds        : 规则给定的 actor 活动范围（x[-55,135] y[-65,65]）
    - grid          : 全自由栅格（380x260 @0.5m，无任何占用）
    - obstacles     : 恒为空 []
    - spawn         : 队自定固定起飞区（城外西侧 x=-50 一线）
    - goal_candidates: 队自定训练目标点（与真值无关）
  真实占用栅格由各 swarm_agent 在比赛中用 2D 雷达实时 SLAM 构建
  （slam_grid.py：扫到→mark occupied），经 /swarm/occupancy_grid 汇总
  给 manager。actor 位置由 YOLO 视觉探测发现，不从任何文件预读。

用法：
  python3 black_box_to_metadata.py                 # 生成默认 robocup_base.json
  python3 black_box_to_metadata.py --out PATH      # 指定输出
  python3 black_box_to_metadata.py --verify PATH   # 校验是否为合规空图（比赛前硬检）
"""

import argparse
import json
import os
import sys

# 规则定值：actor 活动范围（官方规则 §2.2 场地描述，队内常量，非预读）
BOUNDS = {"x_min": -55.0, "x_max": 135.0, "y_min": -65.0, "y_max": 65.0}
GRID_RES = 0.5

# 队自定固定起飞区：城外西侧 x=-50 一线（避开固定灯柱 x>=-45；
# 房屋每局随机化，无法也不允许预读规避 —— 由 SLAM 建图 + 雷达守卫兜底）
SPAWN_CENTER = [-50.0, 0.0]

# 队自定训练目标点（覆盖训练用，与比赛真值无关）
GOAL_CANDIDATES = [
    {"id": "goal_0000", "center": [-20.0, -20.0], "radius_m": 0.6, "clearance_m": 2.0},
    {"id": "goal_0001", "center": [20.0, 20.0], "radius_m": 0.6, "clearance_m": 2.0},
    {"id": "goal_0002", "center": [-30.0, 10.0], "radius_m": 0.6, "clearance_m": 2.0},
    {"id": "goal_0003", "center": [40.0, -10.0], "radius_m": 0.6, "clearance_m": 2.0},
    {"id": "goal_0004", "center": [60.0, 25.0], "radius_m": 0.6, "clearance_m": 2.0},
    {"id": "goal_0005", "center": [-15.0, 35.0], "radius_m": 0.6, "clearance_m": 2.0},
]

DEFAULT_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "worlds", "generated", "robocup_base.json")


def build_metadata():
    """构造合规空图 metadata：障碍恒空、栅格全自由。"""
    w = int(round((BOUNDS["x_max"] - BOUNDS["x_min"]) / GRID_RES))
    h = int(round((BOUNDS["y_max"] - BOUNDS["y_min"]) / GRID_RES))
    return {
        "schema": "robocup_training_worlds/metadata/v1",
        "world_name": "robocup_base",
        "frame": {"convention": "ENU", "frame_id": "map",
                  "ground_z": 0.0, "units": "m"},
        "bounds": dict(BOUNDS),
        "grid": {
            "encoding": "rle_pairs",
            "frame_id": "map",
            "width": w,
            "height": h,
            "resolution_m": GRID_RES,
            "origin": [BOUNDS["x_min"], BOUNDS["y_min"]],
            "inflation_m": 0.0,
            # 合规说明：空图无占用；真实占用由运行时雷达 SLAM 构建
            "note": ("compliant empty map: no ground-truth pre-read "
                     "(rule 2.4/2.5). Occupancy is built at runtime by "
                     "lidar SLAM (slam_grid.py) in swarm_agent."),
            "data": [[0, w * h]],
        },
        # 障碍恒为空 —— 任何非空即为预读违规，--verify 会硬检
        "obstacles": [],
        "spawn": {
            "center": list(SPAWN_CENTER),
            "clearance_m": 2.0,
            "mode": "fixed_team_choice",
            "radius_m": 1.5,
            "yaw": 0.0,
        },
        "goal_candidates": [dict(g, expected_valid=True, outdoors=True,
                                 inside_obstacle=None, reachable=True,
                                 valid=True) for g in GOAL_CANDIDATES],
        "planning_altitude_m": 4.5,
        "safety": {"inflate_m": 1.0, "min_clearance_m": 1.0},
        "generator": {
            "name": "black_box_to_metadata.py (compliant empty map)",
            "source": "rule constants + team-chosen spawn (no black_box.txt)",
        },
    }


def verify_metadata(path):
    """校验 metadata 为合规空图。返回 0=合规，1=违规/不可用。"""
    try:
        with open(path, "r", encoding="utf-8") as f:
            md = json.load(f)
    except Exception as exc:
        print("[verify] FAIL 无法读取 %s: %s" % (path, exc))
        return 1
    errs = []
    obs = md.get("obstacles")
    if obs is None:
        errs.append("缺少 obstacles 字段")
    elif len(obs) != 0:
        errs.append("obstacles 非空（%d 项）—— 疑似预读地图真值，违规！" % len(obs))
    grid = md.get("grid") or {}
    data = grid.get("data")
    if not data or len(data) != 1 or data[0][0] != 0:
        errs.append("grid.data 不是全自由栅格（首 run 应为 [0, N]）")
    b = md.get("bounds") or {}
    for k in ("x_min", "x_max", "y_min", "y_max"):
        if k not in b:
            errs.append("bounds 缺少 %s" % k)
    if errs:
        print("[verify] FAIL %s" % path)
        for e in errs:
            print("  - %s" % e)
        return 1
    print("[verify] PASS %s：合规空图（obstacles=0，栅格全自由，bounds=%s）"
          % (path, b))
    return 0


def main():
    ap = argparse.ArgumentParser(description="生成合规空图 metadata（无预读）")
    ap.add_argument("--out", default=DEFAULT_OUT, help="输出路径")
    ap.add_argument("--verify", metavar="PATH", default=None,
                    help="校验指定 metadata 是否为合规空图后退出")
    args = ap.parse_args()

    if args.verify:
        sys.exit(verify_metadata(args.verify))

    md = build_metadata()
    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(md, f, ensure_ascii=False)
    g = md["grid"]
    print("[gen] 合规空图已生成: %s" % out)
    print("  bounds=%s grid=%dx%d @%.2fm obstacles=0（运行时雷达 SLAM 建图）"
          % (md["bounds"], g["width"], g["height"], g["resolution_m"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
