#!/usr/bin/env python3
"""Read-only, offline check of a local XTDrone RoboCup map fixture.

This diagnostic reads the generated world and the environment-side obstacle
files. It must not run in a contestant node or be used to plan a flight.
"""

import argparse
import ast
from collections import Counter
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


SOURCE_FILES = (
    "map_generator.py", "base1.world", "base2.world", "base3.world",
    "rover_static.world",
)
BUILDINGS = (
    "house_1_146", "house_3_156", "house_3_157", "house_3_158",
    "gas_station_73", "fast_food_93", "house_1_66", "house_1_67",
    "house_1_146_clone", "house_2_71", "house_2_125", "house_2_126",
    "house_3_68",
)
FIXED_BOXES = 14
ROVERS = 20
EXPECTED_BOXES = FIXED_BOXES + len(BUILDINGS) + ROVERS


def load_reference(root, ref):
    """Read pinned verified bytes or fetched Git objects; never use network."""
    try:
        from .official_reference import contract, pinned_blobs
    except ImportError:
        from official_reference import contract, pinned_blobs
    if ref == contract()['commit']:
        return pinned_blobs(root, SOURCE_FILES)
    blobs = {}
    for name in SOURCE_FILES:
        result = subprocess.run(
            ["git", "-C", str(root), "show", "%s:robocup/%s" % (ref, name)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
        )
        if result.returncode:
            detail = result.stderr.decode("utf-8", "replace").strip().splitlines()
            raise ValueError("无法读取官方参考 %s:robocup/%s：%s" %
                             (ref, name, detail[-1] if detail else "git show 失败"))
        blobs[name] = result.stdout
    return blobs


def check_sources(root, reference):
    errors = []
    for name, official in reference.items():
        path = root / "robocup" / name
        try:
            local = path.read_bytes()
        except OSError:
            errors.append("缺少官方地图源码/模板：%s" % path)
            continue
        if local != official:
            errors.append("地图源码/模板偏离官方参考：%s" % path)
    return errors


def _pair(values, context):
    if not isinstance(values, (list, tuple)) or len(values) != 2:
        raise ValueError("%s 应有两个坐标" % context)
    pair = (float(values[0]), float(values[1]))
    if not all(math.isfinite(value) for value in pair):
        raise ValueError("%s 包含非有限坐标" % context)
    return pair


def read_boxes(path):
    if path.stat().st_size > 10_000_000:
        raise ValueError("black_box.txt 异常过大")
    boxes = ast.literal_eval(path.read_text(encoding="utf-8"))
    if not isinstance(boxes, (list, tuple)):
        raise ValueError("black_box.txt 顶层不是矩形列表")
    result = []
    for index, box in enumerate(boxes):
        if not isinstance(box, (list, tuple)) or len(box) != 2:
            raise ValueError("black_box.txt 矩形 %d 格式错误" % index)
        x = _pair(box[0], "矩形 %d X" % index)
        y = _pair(box[1], "矩形 %d Y" % index)
        if x[0] >= x[1] or y[0] >= y[1]:
            raise ValueError("black_box.txt 矩形 %d 无正面积" % index)
        if x[1] - x[0] > 1000 or y[1] - y[0] > 1000:
            raise ValueError("black_box.txt 矩形 %d 尺寸异常" % index)
        result.append((x, y))
    return result


def _xy_from_pose(pose, label):
    if pose is None or not pose.text:
        raise ValueError("%s 缺少位姿" % label)
    parts = pose.text.split()
    if len(parts) < 2:
        raise ValueError("%s 位姿少于两个坐标" % label)
    return _pair(parts[:2], label)


def read_world(path):
    root = ET.parse(path).getroot()
    world = root if root.tag == "world" else root.find("world")
    if world is None:
        raise ValueError("robocup.world 缺少 <world>")
    actors = world.findall("actor")
    names = [actor.get("name") for actor in actors]
    if sorted(names) != ["actor_%d" % index for index in range(6)]:
        raise ValueError("robocup.world 应有 actor_0..5 各一名，实际为 %r" % names)
    for actor in actors:
        plugins = [plugin for plugin in actor.findall("plugin")
                   if "libros_actor_cmd_pose_plugin.so" in plugin.get("filename", "")]
        if len(plugins) != 1 or plugins[0].find("init_pose") is None:
            raise ValueError("%s 缺少官方运动插件及 init_pose" % actor.get("name"))
        _xy_from_pose(plugins[0].find("init_pose"), actor.get("name"))
    models = world.findall("model")  # Ignore Gazebo's duplicated <state><model>.
    model_names = [model.get("name", "") for model in models]
    # Gazebo Classic World::Init applies the first <state> after loading all
    # model definitions. Its pose is therefore the effective initial pose.
    first_state = world.find("state")
    state_models = ({model.get("name"): model for model in first_state.findall("model")}
                    if first_state is not None else {})
    rover_names = [name for name in model_names if name.startswith("rover_static")]
    duplicated_rovers = sorted(name for name, count in Counter(rover_names).items()
                               if count > 1)
    positions = {}
    for name in BUILDINGS:
        matching = [model for model in models if model.get("name") == name]
        if len(matching) != 1:
            raise ValueError("robocup.world 中 %s 数量为 %d，预期 1" %
                             (name, len(matching)))
        model = matching[0]
        saved = state_models.get(name)
        pose = saved.find("pose") if saved is not None else None
        # Gazebo's static gas station has its definition pose on the link.
        if pose is None:
            pose = model.find("pose")
        if pose is None:
            pose = model.find("link/pose")
        positions[name] = _xy_from_pose(pose, name)
    return positions, len(rover_names), duplicated_rovers


def _key(point):
    return round(point[0], 4), round(point[1], 4)


def expected_obstacle_points(boxes):
    """Mirror the official generator's 1 m sampling and perimeter points."""
    points = []
    for x in range(-50, 130):
        points.extend(((x + 0.5, 50.0), (x + 0.5, -50.0)))
    for y in range(-50, 50):
        points.extend(((-50.0, y + 0.5), (130.0, y + 0.5)))
    for (xmin, xmax), (ymin, ymax) in boxes:
        y = ymin + 0.5
        while y + 0.5 <= ymax:
            x = xmin + 0.5
            points.append((x, y))
            while x + 0.5 <= xmax:
                x += 1.0
                points.append((x, y))
            y += 1.0
    return {_key(point) for point in points}


def read_obstacle_points(path):
    points = set()
    with path.open(encoding="utf-8") as stream:
        for lineno, line in enumerate(stream, 1):
            if not line.strip():
                continue
            values = line.split()
            if len(values) != 2:
                raise ValueError("obstacle.txt 第 %d 行不是 X Y" % lineno)
            points.add(_key(_pair(values, "obstacle.txt 第 %d 行" % lineno)))
    return points


def check_artifacts(world_path, boxes_path, obstacle_path):
    """Return all detected contradictions without changing any file."""
    errors = []
    try:
        positions, rover_count, duplicated_rovers = read_world(world_path)
    except (OSError, ET.ParseError, TypeError, ValueError) as exc:
        errors.append("world 无法验证：%s" % exc)
        positions, rover_count, duplicated_rovers = None, None, []
    try:
        boxes = read_boxes(boxes_path)
    except (OSError, SyntaxError, TypeError, ValueError) as exc:
        errors.append("black_box.txt 无法验证：%s" % exc)
        boxes = None
    if rover_count is not None and rover_count != ROVERS:
        errors.append("world 有 %d 个 rover model 元素，生成器配置 %d 组；"
                      "官方原版 rover_static.world 可能使每组重复两次" %
                      (rover_count, ROVERS))
    if duplicated_rovers:
        errors.append("world 中 %d 个 rover 名称重复（例如 %s），Gazebo 实际解析有歧义" %
                      (len(duplicated_rovers), duplicated_rovers[0]))
    if boxes is not None:
        if len(boxes) != EXPECTED_BOXES:
            errors.append("black_box.txt 有 %d 个矩形，官方结构应为 %d 个"
                          "（固定 %d + 建筑 %d + rover %d）" %
                          (len(boxes), EXPECTED_BOXES, FIXED_BOXES,
                           len(BUILDINGS), ROVERS))
        # A locally modified generator may have a different fixed prefix.
        # Infer that prefix so mismatch examples still point to the matching
        # building records, but never accept it as official.
        prefix = len(boxes) - len(BUILDINGS) - ROVERS
        if positions is not None and 0 <= prefix <= 32:
            for index, name in enumerate(BUILDINGS):
                (xmin, xmax), (ymin, ymax) = boxes[prefix + index]
                center = ((xmin + xmax) / 2, (ymin + ymax) / 2)
                actual = positions[name]
                distance = math.hypot(actual[0] - center[0],
                                      actual[1] - center[1])
                if distance > 1.0:
                    errors.append("%s 坐标不匹配：world=(%.2f, %.2f)，"
                                  "black_box=(%.2f, %.2f)，偏差 %.2fm" %
                                  (name, actual[0], actual[1], center[0],
                                   center[1], distance))
    try:
        observed = read_obstacle_points(obstacle_path)
        if boxes is not None:
            expected = expected_obstacle_points(boxes)
            if observed != expected:
                missing = expected - observed
                excess = observed - expected
                errors.append("obstacle.txt 与 black_box.txt/官方边界采样不配套："
                              "缺 %d 点、多 %d 点；例：缺 %r、多 %r" %
                              (len(missing), len(excess), sorted(missing)[:2],
                               sorted(excess)[:2]))
    except (OSError, TypeError, ValueError) as exc:
        errors.append("obstacle.txt 无法验证：%s" % exc)
    return errors


def main(argv=None):
    home = Path.home()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xtdrone-root", type=Path, default=home / "XTDrone")
    parser.add_argument("--source-root", type=Path,
                        help="待比对源码所在目录（默认与 --xtdrone-root 相同；可检查隔离副本）")
    parser.add_argument("--world", type=Path,
                        default=home / "PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world")
    parser.add_argument("--black-box", type=Path,
                        default=home / "XTDrone/robocup/black_box.txt")
    parser.add_argument("--obstacle", type=Path,
                        default=home / "XTDrone/robocup/obstacle.txt")
    parser.add_argument("--official-ref", default="origin/master",
                        help="本地已获取的官方 Git 引用；工具不会联网")
    args = parser.parse_args(argv)
    source_errors = []
    try:
        reference = load_reference(args.xtdrone_root, args.official_ref)
        source_errors.extend(check_sources(args.source_root or args.xtdrone_root,
                                           reference))
    except (OSError, ValueError) as exc:
        source_errors.append(str(exc))
    artifact_errors = check_artifacts(args.world, args.black_box, args.obstacle)
    if not source_errors:
        print("[map:source] PASS 生成器和模板与本地官方 Git 引用一致", flush=True)
    if not artifact_errors:
        print("[map:artifact] PASS world、black_box.txt、obstacle.txt 一致", flush=True)
    if source_errors or artifact_errors:
        for error in source_errors:
            print("[map:source] FAIL %s" % error, file=sys.stderr)
        for error in artifact_errors:
            print("[map:artifact] RISK %s" % error, file=sys.stderr)
        return 1
    print("[map] PASS 官方源码/模板与 world、black_box.txt、obstacle.txt 一致")
    return 0


if __name__ == "__main__":
    sys.exit(main())
