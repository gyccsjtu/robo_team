#!/usr/bin/env python3
"""Safely stage an official XTDrone map set; only --apply changes live files.

The original official generator is kept byte-for-byte. A temporary runner
changes only its output directory so generation cannot overwrite the live
world before all files exist and have been backed up. This is an offline
environment maintenance tool, never a contestant runtime dependency.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

try:
    from .check_official_map import (SOURCE_FILES, check_artifacts, check_sources,
                                     load_reference, read_boxes,
                                     read_obstacle_points)
except ImportError:  # Direct invocation: python3 tools/prepare_official_map.py
    from check_official_map import (SOURCE_FILES, check_artifacts, check_sources,
                                    load_reference, read_boxes,
                                    read_obstacle_points)


OUTPUT_LINE = "output_path = os.path.expanduser('~/PX4_Firmware/Tools/sitl_gazebo/worlds/')"
SIMULATION_NAMES = {
    "gzserver", "gzclient", "rosmaster", "roscore", "roslaunch", "px4",
    "mavros_node", "control_actor.py", "control_actors.sh", "score_cal.py",
}


def active_simulation_processes():
    """Read /proc; catch ROS shell wrappers as well as Gazebo executables."""
    active = []
    proc = Path("/proc")
    for entry in proc.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            args = (entry / "cmdline").read_bytes().split(b"\0")
        except (OSError, PermissionError):
            continue
        names = {Path(arg.decode("utf-8", "replace")).name
                 for arg in args if arg and not arg.startswith(b"-")}
        hits = names & SIMULATION_NAMES
        if hits:
            active.append("%s:%s" % (entry.name, ",".join(sorted(hits))))
    return active


def _sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _destinations(xtdrone_root, world_path):
    result = {"source/" + name: xtdrone_root / "robocup" / name
              for name in SOURCE_FILES}
    result["world"] = world_path
    result["black_box"] = xtdrone_root / "robocup/black_box.txt"
    result["obstacle"] = xtdrone_root / "robocup/obstacle.txt"
    return result


def _back_up(destinations, stage):
    backup = {}
    backup_dir = stage / "backup"
    backup_dir.mkdir(parents=True)
    for key, destination in destinations.items():
        if destination.exists():
            saved = backup_dir / key.replace("/", "__")
            shutil.copy2(destination, saved)
            backup[key] = saved
        else:
            backup[key] = None
    return backup


def _stage_official_sources(blobs, stage):
    robocup = stage / "official/robocup"
    robocup.mkdir(parents=True)
    for name, body in blobs.items():
        (robocup / name).write_bytes(body)
    return robocup


def _run_generator(robocup, generated, timeout):
    generated.mkdir(parents=True, exist_ok=True)
    source = (robocup / "map_generator.py").read_text(encoding="utf-8")
    if source.count(OUTPUT_LINE) != 1:
        raise ValueError("官方生成器输出路径语句已变化，拒绝改写运行副本")
    runner = robocup / "_map_generator_staged.py"
    redirected = source.replace(
        OUTPUT_LINE, "output_path = %r" % (str(generated) + os.sep), 1)
    runner.write_text(redirected, encoding="utf-8")
    try:
        result = subprocess.run(
            [sys.executable, str(runner)], cwd=str(robocup),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            timeout=timeout, check=False,
        )
    except subprocess.TimeoutExpired as exc:
        (robocup.parent.parent / "generator.log").write_text(
            "生成超时：%ss\n%s\n%s" % (timeout, exc.stdout or "", exc.stderr or ""),
            encoding="utf-8",
        )
        raise RuntimeError("官方地图生成器超过 %ss；原文件未切换" % timeout)
    (robocup.parent.parent / "generator.log").write_text(
        "exit=%d\n--- stdout ---\n%s\n--- stderr ---\n%s\n" %
        (result.returncode, result.stdout, result.stderr), encoding="utf-8")
    if result.returncode:
        raise RuntimeError("官方地图生成器退出码 %d；参见 generator.log" %
                           result.returncode)


def _validate_generated(world, black_box, obstacle):
    for path in (world, black_box, obstacle):
        if not path.is_file() or path.stat().st_size == 0:
            raise ValueError("生成文件缺失或为空：%s" % path)
    root = ET.parse(world).getroot()
    element = root if root.tag == "world" else root.find("world")
    if element is None:
        raise ValueError("生成 world 缺少 <world>")
    actors = element.findall("actor")
    if sorted(actor.get("name") for actor in actors) != [
            "actor_%d" % index for index in range(6)]:
        raise ValueError("生成 world 未包含 actor_0..5 各一名")
    for actor in actors:
        plugins = [plugin for plugin in actor.findall("plugin")
                   if "libros_actor_cmd_pose_plugin.so" in plugin.get("filename", "")]
        if len(plugins) != 1 or plugins[0].find("init_pose") is None:
            raise ValueError("%s 缺少官方运动插件或初始位姿" % actor.get("name"))
    if not read_boxes(black_box) or not read_obstacle_points(obstacle):
        raise ValueError("black_box.txt 或 obstacle.txt 没有有效数据")


def _generate_validated(robocup, generated, stage, timeout, max_attempts=3):
    """Retry only an empty official world, within one total time budget.

    The official generator and its choices remain unchanged. Every failed
    generation is retained; no simulation starts from an invalid artifact.
    """
    if not isinstance(max_attempts, int) or not 1 <= max_attempts <= 5:
        raise ValueError("generation attempts must be between 1 and 5")
    started = time.monotonic()
    attempts = []
    world = generated / "robocup.world"
    black_box = robocup / "black_box.txt"
    obstacle = robocup / "obstacle.txt"
    for number in range(1, max_attempts+1):
        remaining = timeout-(time.monotonic()-started)
        if remaining <= 0.:
            raise RuntimeError("地图生成总时间预算耗尽；原文件未切换")
        _run_generator(robocup, generated, remaining)
        log = stage / ("generator_attempt_%02d.log" % number)
        shutil.copy2(stage / "generator.log", log)
        row = {"attempt": number, "log": str(log)}
        try:
            _validate_generated(world, black_box, obstacle)
        except (ValueError, ET.ParseError) as exc:
            row.update(valid=False, error=str(exc),
                       empty_world=not world.is_file() or world.stat().st_size == 0)
            attempts.append(row)
            (stage / "generation_attempts.json").write_text(
                json.dumps(attempts, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
            if not row['empty_world']:
                raise
            failed = stage / ('generation_failure_%02d' % number)
            failed.mkdir()
            for path in (world, black_box, obstacle):
                if path.is_file():
                    shutil.copy2(path, failed / path.name)
                    path.unlink()  # staged outputs only; never reuse another attempt's metadata
            if number == max_attempts:
                raise RuntimeError("官方生成器连续%d次产生空地图；原文件未切换" % number) from exc
            continue
        row.update(valid=True, world_sha256=_sha(world))
        attempts.append(row)
        (stage / "generation_attempts.json").write_text(
            json.dumps(attempts, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
        return world, black_box, obstacle, attempts


def install_one(staged, destination):
    """The only replacement point, kept separate for rollback regression tests."""
    os.replace(staged, destination)


def _install_transaction(destinations, sources, backup, stage):
    prepared = {}
    token = stage.name
    try:
        for key, destination in destinations.items():
            destination.parent.mkdir(parents=True, exist_ok=True)
            temp = destination.with_name(destination.name + "." + token + ".new")
            shutil.copy2(sources[key], temp)
            prepared[key] = temp
        for key, destination in destinations.items():
            install_one(prepared[key], destination)
    except Exception:
        # Restore every destination, including files that did not exist before.
        restore_errors = []
        for key, destination in destinations.items():
            saved = backup[key]
            try:
                if saved is None:
                    destination.unlink(missing_ok=True)
                else:
                    temp = destination.with_name(destination.name + "." + token + ".restore")
                    shutil.copy2(saved, temp)
                    os.replace(temp, destination)
            except OSError as exc:
                restore_errors.append("%s: %s" % (destination, exc))
        if restore_errors:
            raise RuntimeError("切换失败且部分还原失败；备份在 %s：%s" %
                               (stage / "backup", "; ".join(restore_errors)))
        raise
    finally:
        for temp in prepared.values():
            temp.unlink(missing_ok=True)


def prepare(xtdrone_root, world_path, official_ref="origin/master",
            stage_parent=None, timeout=90):
    """Apply a backed-up official source/artifact set; caller checks consent."""
    running = active_simulation_processes()
    if running:
        raise RuntimeError("仍有仿真/裁判进程，拒绝切换：%s" % ", ".join(running))
    blobs = load_reference(xtdrone_root, official_ref)
    destinations = _destinations(xtdrone_root, world_path)
    if any(destination.is_symlink() for destination in destinations.values()):
        raise RuntimeError("目标路径包含符号链接，拒绝替换；请先人工核对实际路径")
    if len({destination.resolve() for destination in destinations.values()}) != len(destinations):
        raise RuntimeError("目标路径重复，拒绝执行不明确的成套切换")
    parent = stage_parent or xtdrone_root.parent
    parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix="official_map_stage_", dir=str(parent)))
    try:
        backup = _back_up(destinations, stage)
        robocup = _stage_official_sources(blobs, stage)
        generated = stage / "generated"
        world, black_box, obstacle, generation_attempts = _generate_validated(
            robocup, generated, stage, timeout)
        risks = check_artifacts(world, black_box, obstacle)
        sources = {"source/" + name: robocup / name for name in SOURCE_FILES}
        sources.update(world=world, black_box=black_box, obstacle=obstacle)
        manifest = {
            "reference": official_ref,
            "generation_attempts": generation_attempts,
            "stage": str(stage),
            "backup": {key: str(path) if path else None for key, path in backup.items()},
            "official_source_sha256": {name: _sha(robocup / name) for name in SOURCE_FILES},
            "generated_sha256": {key: _sha(sources[key]) for key in
                                 ("world", "black_box", "obstacle")},
            "artifact_risks": risks,
        }
        (stage / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False,
                                                        indent=2) + "\n", encoding="utf-8")
        running = active_simulation_processes()
        if running:
            raise RuntimeError("生成期间仿真/裁判进程启动，拒绝切换：%s" % ", ".join(running))
        _install_transaction(destinations, sources, backup, stage)
        return stage, risks
    except Exception as exc:
        raise RuntimeError("准备失败，原文件保持或已还原；备份/日志在 %s：%s" %
                           (stage, exc)) from exc


def main(argv=None):
    home = Path.home()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true",
                        help="显式执行备份、官方版本恢复和成套地图切换；默认只预览")
    parser.add_argument("--xtdrone-root", type=Path, default=home / "XTDrone")
    parser.add_argument("--world", type=Path,
                        default=home / "PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world")
    parser.add_argument("--official-ref", default="origin/master")
    parser.add_argument("--stage-parent", type=Path)
    parser.add_argument("--timeout", type=int, default=90)
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("--timeout 必须为正数")
    try:
        reference = load_reference(args.xtdrone_root, args.official_ref)
        destinations = _destinations(args.xtdrone_root, args.world)
        if not args.apply:
            print("[prepare-map] DRY RUN：没有写入文件，也没有生成地图")
            print("[prepare-map] 官方参考：%s" % args.official_ref)
            for key, destination in destinations.items():
                print("[prepare-map] 备份并替换 %s：%s" % (key, destination))
            for issue in check_sources(args.xtdrone_root, reference):
                print("[prepare-map] 当前源码差异：%s" % issue)
            return 0
        stage, risks = prepare(args.xtdrone_root, args.world, args.official_ref,
                               args.stage_parent, args.timeout)
    except (OSError, ValueError, RuntimeError, ET.ParseError) as exc:
        print("[prepare-map] FAIL %s" % exc, file=sys.stderr)
        return 1
    print("[prepare-map] DONE 已恢复官方源码/模板和成套生成文件；备份：%s" %
          (stage / "backup"))
    print("[prepare-map] 生成日志与清单：%s" % stage)
    for risk in risks:
        print("[prepare-map] RISK 官方生成物：%s" % risk)
    return 0


if __name__ == "__main__":
    sys.exit(main())
