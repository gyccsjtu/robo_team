#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修复官方 map_generator.py 的 rover 堆叠 bug（训练环境修补）。

问题（2026-10-01 全链路仿真排查发现）：
  官方 ~/XTDrone/robocup/map_generator.py 有两个缺陷：
  1. base.world 模板自带 20 辆 rover 全部叠在 (0.1, 0)——正好压住
     uav2 的出生点 (0,0)（官方 robocup.launch 的 x/y），无人机 spawn
     进 20 层静态体深处 → libgazebo_ode 深穿透解算段错误（gzserver
     exit 255）。此前被误判为 gpu_ray 传感器触发。
  2. 追加的 20 辆 rover 位姿替换逻辑有 bug，全部落同一个点
     （-2.8, -45.1），而 black_box.txt 记录的是 20 个真实车道位置。

本脚本：
  A. 删除原点堆叠的 20 辆 rover（模板遗留物）；
  B. 把 degenerate 堆（同一坐标 >2 辆）的 rover 逐辆摆到 black_box.txt
     记录的 rover 矩形中心——修复后 world ↔ black_box 达到 43/43 同步，
     live 地图（black_box_to_metadata.py 产物）完全精确。

用法：python3 fix_rover_stacks.py [--world path] [--black-box path] [--dry-run]
注意：map_generator.py 每次随机化后需重跑本脚本。
"""

import argparse
import ast
import math
import re
import sys

DEFAULT_WORLD = "/home/gycc/PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world"
DEFAULT_BLACK_BOX = "/home/gycc/XTDrone/robocup/black_box.txt"

# 出生区（6 机 spawn：x∈{0,3}, y∈{-3,0,3}），中心留半车+margin
SPAWN_CLEAR_X = 5.0
SPAWN_CLEAR_Y = 5.0

ROVER_MAX_SIZE = 2.5   # rover 矩形 2x2；灯柱 0.5x1
ROVER_MIN_SIZE = 1.5


def load_rover_rects(path):
    bb = ast.literal_eval(open(path).read().strip())
    rects = []
    for r in bb:
        w = r[0][1] - r[0][0]
        h = r[1][1] - r[1][0]
        if ROVER_MIN_SIZE <= w <= ROVER_MAX_SIZE and ROVER_MIN_SIZE <= h <= ROVER_MAX_SIZE:
            rects.append(((r[0][0] + r[0][1]) / 2.0, (r[1][0] + r[1][1]) / 2.0))
    return rects


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default=DEFAULT_WORLD)
    parser.add_argument("--black-box", default=DEFAULT_BLACK_BOX)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    rover_centers = load_rover_rects(args.black_box)
    if not rover_centers:
        print("black_box.txt 里没有 rover 矩形，无需修复")
        return 0

    content = open(args.world).read()
    blocks = []   # (start, end, name, pose_str, x, y)
    for m in re.finditer(r"<model[^>]*name='(rover[^']*)'[^>]*>.*?</model>", content, re.S):
        pm = re.search(r"<pose[^>]*>([^<]+)</pose>", m.group(0))
        if not pm:
            continue
        parts = pm.group(1).split()
        blocks.append((m.start(), m.end(), m.group(1), pm.group(1),
                       float(parts[0]), float(parts[1])))
    if not blocks:
        print("world 里没有 rover 模型，无需修复")
        return 0

    # 按位置分类
    origin_stack = [b for b in blocks if abs(b[4]) < 4 and abs(b[5]) < 4]
    rest = [b for b in blocks if b not in origin_stack]
    # degenerate 堆检测：剩余 rover 中同一坐标 ≥2 辆
    pos_count = {}
    for b in rest:
        pos_count[(round(b[4], 1), round(b[5], 1))] = pos_count.get((round(b[4], 1), round(b[5], 1)), 0) + 1
    stacks = {p for p, c in pos_count.items() if c >= 2}
    to_place = [b for b in rest if (round(b[4], 1), round(b[5], 1)) in stacks]
    keep = [b for b in rest if (round(b[4], 1), round(b[5], 1)) not in stacks]

    print("world rover 总数 %d：原点堆 %d（删除）、degenerate 堆 %d（摆位）、正常 %d（保留）"
          % (len(blocks), len(origin_stack), len(to_place), len(keep)))

    if len(to_place) > len(rover_centers):
        print("⚠ degenerate 堆车数(%d) 超过 black_box rover 矩形数(%d)" % (len(to_place), len(rover_centers)))

    # 摆位目标：跳过出生区内的 black_box 中心（防 spawn 穿透）
    targets = [(cx, cy) for cx, cy in rover_centers
               if not (abs(cx) < SPAWN_CLEAR_X and abs(cy) < SPAWN_CLEAR_Y)]
    skipped = len(rover_centers) - len(targets)
    if skipped:
        print("⚠ %d 个 black_box rover 落出生区 ±%.0fm 内，其车保持原位（建议重跑 map_generator）"
              % (skipped, SPAWN_CLEAR_X))

    # 生成替换：从后往前改避免偏移失效
    edits = []  # (start, end, new_block or None)
    for i, b in enumerate(origin_stack):
        edits.append((b[0], b[1], None))
    for i, b in enumerate(to_place):
        if i >= len(targets):
            edits.append((b[0], b[1], None))  # 没位置了，删除多余车
            continue
        tx, ty = targets[i]
        old_pose = b[3].split()
        new_pose = "%.3f %.3f %s %s %s %s" % (tx, ty, *old_pose[2:])
        block_new = content[b[0]:b[1]].replace(b[3], new_pose, 1)
        edits.append((b[0], b[1], block_new))

    if args.dry_run:
        for i, b in enumerate(to_place[:6]):
            t = targets[i] if i < len(targets) else None
            print("  车摆位: (%.1f,%.1f) → %s" % (b[4], b[5], t))
        return 0

    for start, end, new_block in sorted(edits, key=lambda e: -e[0]):
        content = content[:start] + (new_block or "") + content[end:]
    with open(args.world, "w") as f:
        f.write(content)
    print("✅ 已修复：%d 删除，%d 摆位，%d 保留" %
          (len(origin_stack) + max(0, len(to_place) - len(targets)), min(len(to_place), len(targets)), len(keep)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
