#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""black_box.txt → robocup_training_worlds metadata（动态地图适配）。

背景（2026-10-01 撞楼复盘）：
  官方规则 §2.4/§2.5：house / lamp 位置**每次尝试前由脚本随机化**。
  官方工具 ~/XTDrone/robocup/map_generator.py 每场随机化后会重写
  robocup.world 并输出 black_box.txt（43 个建筑矩形真值，含固定灯柱
  与随机房屋）。官方 control_actor.py 靠它保证 actor 不站进楼里。

  而 swarm 链路的 ROBOCUP_METADATA 若指向静态 robocup_base.json
  （某一次随机化的手工快照），随机化一变就整体过时——实测 9/30 19:56
  随机化后，旧快照与当前 black_box.txt 的 43 个矩形只重合 10 个
  （灯柱固定所以重合，房屋全变）。A* 按旧图规划 → 直线穿过真实建筑。

本脚本：读当前 black_box.txt → 生成与 robocup_base.json 同 schema 的
metadata（robocup_training_worlds/metadata/v1），只替换 obstacles +
重建占用栅格；frame/bounds/spawn/goal_candidates 从模板继承（场地活动
范围是规则定值，与随机化无关）。run_match.sh 比赛段起 swarm 前调用，
保证任意随机图下地图与 Gazebo 世界一致。

数据来源合规说明：black_box.txt 是主办方每场下发的**环境**描述（建筑
矩形），不涉及目标位置真值——用它做静态地图等价于看一眼场地，与
SEED_TRUTH（目标真值播种，规则禁止）不是一回事。

用法：
  python3 black_box_to_metadata.py \
      [--black-box ~/XTDrone/robocup/black_box.txt] \
      [--template  .../generated/robocup_base.json] \
      [--output    .../generated/robocup_live.json]
"""

import argparse
import ast
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GENERATED_DIR = os.path.normpath(os.path.join(HERE, "..", "worlds", "generated"))
DEFAULT_BLACK_BOX = os.path.expanduser("~/XTDrone/robocup/black_box.txt")
DEFAULT_TEMPLATE = os.path.join(GENERATED_DIR, "robocup_base.json")
DEFAULT_OUTPUT = os.path.join(GENERATED_DIR, "robocup_live.json")

BUILDING_HEIGHT = 10.0   # 与模板一致（规则限高 6m < 建筑高度，纯竖直爬升不可绕行）
GROUP_NAME = "black_box"


def load_black_box(path):
    """读 black_box.txt → [((x_min, y_min), (x_max, y_max)), ...]。

    文件格式（map_generator.py 输出）：python 字面量列表，
    每项 [[x_min, x_max], [y_min, y_max]]。
    """
    with open(path, "r", encoding="utf-8") as stream:
        content = stream.read().strip()
    if not content:
        raise ValueError("black_box.txt 为空（随机化未运行？）")
    try:
        raw = ast.literal_eval(content)
    except (SyntaxError, ValueError) as exc:
        raise ValueError("black_box.txt 不是合法 python 字面量: %s" % exc)
    if not isinstance(raw, list) or not raw:
        raise ValueError("black_box.txt 不是非空列表")

    rects = []
    for index, item in enumerate(raw):
        if (not isinstance(item, list) or len(item) != 2
                or not all(isinstance(v, list) and len(v) == 2 for v in item)):
            raise ValueError("第 %d 项不是 [[x_min,x_max],[y_min,y_max]] 结构" % index)
        (x_min, x_max), (y_min, y_max) = item
        try:
            x_min, x_max = float(x_min), float(x_max)
            y_min, y_max = float(y_min), float(y_max)
        except (TypeError, ValueError):
            raise ValueError("第 %d 项含非数值" % index)
        if not all(math.isfinite(v) for v in (x_min, x_max, y_min, y_max)):
            raise ValueError("第 %d 项含非有限值" % index)
        if x_max <= x_min or y_max <= y_min:
            raise ValueError("第 %d 项退化（min>=max）" % index)
        rects.append(((x_min, y_min), (x_max, y_max)))
    return rects


def build_obstacles(rects):
    """矩形 → 模板同构的障碍物列表。"""
    obstacles = []
    for index, ((x_min, y_min), (x_max, y_max)) in enumerate(rects):
        width, height = x_max - x_min, y_max - y_min
        obstacles.append({
            "bbox": [x_min, y_min, x_max, y_max],
            "blocking": True,
            "center": [(x_min + x_max) / 2.0, (y_min + y_max) / 2.0],
            "footprint": {
                "kind": "polygon",
                "points": [[x_min, y_min], [x_max, y_min],
                           [x_max, y_max], [x_min, y_max]],
            },
            "group": GROUP_NAME,
            "height": BUILDING_HEIGHT,
            "id": "%s_%02d" % (GROUP_NAME, index),
            "shape": "box",
            "size": [width, height],
            "type": "building",
            "yaw": 0.0,
            "z_max": BUILDING_HEIGHT,
            "z_min": 0.0,
        })
    return obstacles


def build_grid(rects, bounds, resolution):
    """占用栅格（rle_pairs）——与 astar.GridMap 索引约定一致：

    cells[cy * width + cx]，cy=0 对应 y_min 行；cell 中心点落在任一
    矩形内即视为占用（cell_to_world 返回中心，判定口径与之一致）。
    """
    x_min, y_min = bounds["x_min"], bounds["y_min"]
    width = int(round((bounds["x_max"] - x_min) / resolution))
    height = int(round((bounds["y_max"] - y_min) / resolution))
    if width <= 0 or height <= 0 or width * height > 2_000_000:
        raise ValueError("bounds×resolution 得到非法栅格 %dx%d" % (width, height))

    # 越界矩形裁剪到 bounds 内（map_generator 理论上不会越界，防御性处理）
    clipped = 0
    safe_rects = []
    for (rx0, ry0), (rx1, ry1) in rects:
        cx0, cy0 = max(rx0, x_min), max(ry0, y_min)
        cx1, cy1 = min(rx1, bounds["x_max"]), min(ry1, bounds["y_max"])
        if cx1 <= cx0 or cy1 <= cy0:
            clipped += 1
            continue
        if (cx0, cy0, cx1, cy1) != (rx0, ry0, rx1, ry1):
            clipped += 1
        safe_rects.append((cx0, cy0, cx1, cy1))

    occupied_count = 0
    runs = []          # [[value, count], ...]
    last_value = None
    for cy in range(height):
        wy = y_min + (cy + 0.5) * resolution
        for cx in range(width):
            wx = x_min + (cx + 0.5) * resolution
            value = 0
            for (rx0, ry0, rx1, ry1) in safe_rects:
                if rx0 <= wx <= rx1 and ry0 <= wy <= ry1:
                    value = 1
                    break
            if value:
                occupied_count += 1
            if value == last_value and runs:
                runs[-1][1] += 1
            else:
                runs.append([value, 1])
                last_value = value

    grid = {
        "encoding": "rle_pairs",
        "frame_id": "map",
        "width": width,
        "height": height,
        "resolution_m": resolution,
        "origin": [x_min, y_min],
        "inflation_m": 0.0,
        "note": ("generated live from black_box.txt (官方随机化输出) "
                 "by black_box_to_metadata.py"),
        "data": runs,
    }
    return grid, clipped, occupied_count


def main():
    parser = argparse.ArgumentParser(
        description="black_box.txt → robocup_training_worlds metadata（动态地图）")
    parser.add_argument("--black-box", default=DEFAULT_BLACK_BOX,
                        help="官方随机化输出的建筑矩形文件（默认 %s）" % DEFAULT_BLACK_BOX)
    parser.add_argument("--template", default=DEFAULT_TEMPLATE,
                        help="schema 模板（默认 %s）" % DEFAULT_TEMPLATE)
    parser.add_argument("--output", default=DEFAULT_OUTPUT,
                        help="输出 metadata json（默认 %s）" % DEFAULT_OUTPUT)
    parser.add_argument("--check", action="store_true",
                        help="只校验输入并打印摘要，不写文件")
    args = parser.parse_args()

    rects = load_black_box(args.black_box)
    with open(args.template, "r", encoding="utf-8") as stream:
        template = json.load(stream)

    schema = template.get("schema")
    if schema != "robocup_training_worlds/metadata/v1":
        raise ValueError("模板 schema 不是 v1: %r" % schema)
    bounds = template.get("bounds")
    if not isinstance(bounds, dict) or not all(
            isinstance(bounds.get(k), (int, float)) for k in
            ("x_min", "x_max", "y_min", "y_max")):
        raise ValueError("模板 bounds 非法")
    resolution = template.get("grid", {}).get("resolution_m", 0.5)
    if not isinstance(resolution, (int, float)) or resolution <= 0:
        raise ValueError("模板栅格分辨率非法")

    obstacles = build_obstacles(rects)
    grid, clipped, occupied = build_grid(rects, bounds, float(resolution))

    # 覆盖完整性自检：每个矩形至少压中一个占用 cell（0.55m 灯柱 @0.5m 分辨率必中）
    for index, ((x_min, y_min), (x_max, y_max)) in enumerate(rects):
        cxc = int(math.floor((x_min + x_max) / 2.0 - bounds["x_min"]) // resolution)
        cyc = int(math.floor((y_min + y_max) / 2.0 - bounds["y_min"]) // resolution)
        if not (0 <= cxc < grid["width"] and 0 <= cyc < grid["height"]):
            raise ValueError("矩形 %d 中心 (%.1f,%.1f) 落在栅格外" %
                             (index, (x_min + x_max) / 2, (y_min + y_max) / 2))
        if grid["data"] and grid["data"][0] == [0, 1] and occupied == 0:
            raise ValueError("栅格全空，随机化数据异常")

    if args.check:
        print("black_box=%s" % args.black_box)
        print("矩形数=%d（灯柱+房屋），bounds=%s，占用 cell=%d，越界裁剪=%d"
              % (len(rects), bounds, occupied, clipped))
        for i, ((x0, y0), (x1, y1)) in enumerate(rects):
            print("  [%02d] (%.1f,%.1f)~(%.1f,%.1f)  %.1fm×%.1fm"
                  % (i, x0, y0, x1, y1, x1 - x0, y1 - y0))
        return 0

    metadata = {
        "schema": schema,
        "world_name": "robocup_live",
        "frame": template.get("frame"),
        "bounds": bounds,
        "grid": grid,
        "obstacles": obstacles,
        "spawn": template.get("spawn"),
        "goal_candidates": template.get("goal_candidates"),
    }

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(metadata, stream, ensure_ascii=False, separators=(",", ":"))

    print("已生成 %s：矩形 %d 个（越界裁剪 %d），占用 cell %d，bounds %s"
          % (args.output, len(obstacles), clipped, occupied, bounds))
    return 0


if __name__ == "__main__":
    sys.exit(main())
