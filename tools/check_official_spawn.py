#!/usr/bin/env python3
"""Check that a six-UAV scene, world offsets and map metadata share the official spawn."""

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET


OFFICIAL = ((0.0, -3.0), (3.0, -3.0), (0.0, 0.0),
            (3.0, 0.0), (0.0, 3.0), (3.0, 3.0))


def scene_points(path):
    root = ET.parse(path).getroot()
    groups = {group.get("ns"): group for group in root.findall("group")}
    points = []
    for i in range(6):
        name = "typhoon_h480_%d" % i
        group = groups.get(name)
        if group is None:
            raise ValueError("场景缺少 %s" % name)
        launch = next((node for node in group.findall("include")
                       if "single_vehicle_spawn" in node.get("file", "")), None)
        if launch is None:
            raise ValueError("%s 缺少飞行器 spawn include" % name)
        args = {item.get("name"): item.get("value") for item in launch.findall("arg")}
        if float(args["z"]) != 0.0 or float(args["Y"]) != 0.0:
            raise ValueError("%s 的出生高度或 yaw 偏离官方" % name)
        points.append((float(args["x"]), float(args["y"])))
    return tuple(points)


def offset_points(path):
    with open(path, encoding="utf-8") as stream:
        shell = stream.read()
    arrays = []
    for name in ("TAKEOFF_X", "TAKEOFF_Y"):
        match = re.search(r"^%s=\(([^)]*)\)$" % name, shell, re.MULTILINE)
        if match is None:
            raise ValueError("启动脚本缺少 %s 数组" % name)
        values = tuple(float(value) for value in match.group(1).split())
        if len(values) != 6:
            raise ValueError("%s 必须有 6 个值" % name)
        arrays.append(values)
    return tuple(zip(*arrays))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--run-script", required=True)
    parser.add_argument("--metadata", required=True)
    args = parser.parse_args()
    try:
        scene = scene_points(args.scene)
        offsets = offset_points(args.run_script)
        with open(args.metadata, encoding="utf-8") as stream:
            spawn = json.load(stream)["spawn"]
        center = tuple(float(value) for value in spawn["center"])
        if scene != OFFICIAL:
            raise ValueError("场景出生点 %r，官方为 %r" % (scene, OFFICIAL))
        if offsets != OFFICIAL:
            raise ValueError("MAVROS 世界坐标偏移 %r，官方为 %r" % (offsets, OFFICIAL))
        if center != (1.5, 0.0) or spawn.get("mode") != "fixed":
            raise ValueError("地图元数据出生区 %r 与官方六机阵列不符" % spawn)
    except (OSError, ET.ParseError, KeyError, TypeError, ValueError) as exc:
        print("[spawn] FAIL %s" % exc, file=sys.stderr)
        return 1
    print("[spawn] PASS 六机出生点、世界偏移和地图元数据均为官方固定阵列")
    return 0


if __name__ == "__main__":
    sys.exit(main())
