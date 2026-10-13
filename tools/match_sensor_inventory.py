#!/usr/bin/env python3
"""Inventory installed logical sensors for the six fixed RoboCup aircraft.

This offline tool reads launch/SDF files only. It does not contact ROS or infer
that a mounted but unused sensor is free. Pricing follows score_cal.py; stock
IMU/GPS/sonar are recorded separately because that formula has no such items.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


UAV_NAMES = tuple("typhoon_h480_%d" % i for i in range(6))
PRICES = dict(mono_cam=500, stereo_cam=1000, laser1d=200, laser2d=1000,
              laser3d=2000, gimbal=200, bino_cam=1000)
UNPRICED_TYPES = frozenset(("imu", "gps", "sonar", "magnetometer", "air_pressure"))
MODEL_NAME = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_-]*\Z")


def _counts():
    return dict.fromkeys(PRICES, 0)


def _model_path(models, name):
    if not MODEL_NAME.fullmatch(name):
        raise ValueError("unsafe or unresolved SDF model name: %r" % name)
    for filename in (name + ".sdf", "model.sdf"):
        path = models / name / filename
        resolved = path.resolve()
        try:
            resolved.relative_to(models)
        except ValueError:
            raise ValueError("SDF path escapes models directory: %s" % path)
        if resolved.is_file():
            return resolved
    raise ValueError("missing SDF for model %s" % name)


def _sample_count(sensor, axis):
    node = sensor.find("ray/scan/" + axis)
    if node is None and axis == "vertical":
        return 1
    raw = node.findtext("samples", "").strip() if node is not None else ""
    if not re.fullmatch(r"[0-9]+", raw) or int(raw) < 1:
        raise ValueError("%s: invalid %s scan samples" % (sensor.get("name"), axis))
    return int(raw)


def _inspect_model(models, name, snapshots, stack=()):
    path = _model_path(models, name)
    if path in stack:
        raise ValueError("cyclic SDF model include: %s" % path)
    if path not in snapshots:
        raw = path.read_bytes()
        root = ET.fromstring(raw)
        if root.tag != "sdf" or len(root.findall("model")) != 1:
            raise ValueError("expected one SDF model: %s" % path)
        snapshots[path] = (root, hashlib.sha256(raw).hexdigest())
    root, sha = snapshots[path]
    counts, devices, unpriced = _counts(), [], []
    resources = [{"model": name, "path": str(path), "sha256": sha}]

    def record(kind, link, sensors):
        counts[kind] += 1
        devices.append(dict(type=kind, model=name, link=link,
                            sensors=sensors, sdf_sha256=sha))

    seen_sensors = 0
    for link in root.findall(".//link"):
        sensors = link.findall("sensor")
        seen_sensors += len(sensors)
        names = [s.get("name") for s in sensors]
        if any(not n for n in names) or len(names) != len(set(names)):
            raise ValueError("missing or duplicate sensor name in %s/%s" %
                             (name, link.get("name")))
        pair = [s for s in sensors if s.get("name", "").startswith("stereo_camera_")]
        if pair:
            if (len(pair) != 2 or {s.get("name") for s in pair} !=
                    {"stereo_camera_left", "stereo_camera_right"} or
                    any(s.get("type") != "camera" or len(s.findall("camera")) != 1
                        for s in pair)):
                raise ValueError("orphan or unknown stereo pair in %s/%s" %
                                 (name, link.get("name")))
            record("stereo_cam", link.get("name"), sorted(s.get("name") for s in pair))
        for sensor in sensors:
            if sensor in pair:
                continue
            kind, sensor_name = sensor.get("type"), sensor.get("name")
            if kind == "multicamera":
                cameras = sensor.findall("camera")
                if len(cameras) != 2 or {c.get("name") for c in cameras} != {"left", "right"}:
                    raise ValueError("unknown multicamera layout: %s/%s" % (name, sensor_name))
                record("stereo_cam", link.get("name"), [sensor_name])
            elif kind == "camera":
                if (link.get("name") != "cgo3_camera_link" or sensor_name != "camera" or
                        len(sensor.findall("camera")) != 1):
                    raise ValueError("unknown camera layout: %s/%s" % (name, sensor_name))
                record("mono_cam", link.get("name"), [sensor_name])
            elif kind in ("ray", "gpu_ray"):
                horizontal = _sample_count(sensor, "horizontal")
                vertical = _sample_count(sensor, "vertical")
                dimensions = 1 + int(horizontal > 1) + int(vertical > 1)
                record("laser%dd" % dimensions, link.get("name"), [sensor_name])
            elif kind in UNPRICED_TYPES:
                unpriced.append(dict(type=kind, sensor=sensor_name, model=name,
                                     link=link.get("name")))
            else:
                raise ValueError("unknown sensor type: %s/%s=%r" % (name, sensor_name, kind))
    if seen_sensors != len(root.findall(".//sensor")):
        raise ValueError("sensor outside a supported link layout: %s" % name)

    for plugin in root.findall(".//plugin"):
        plugin_name = plugin.get("name", "")
        if plugin_name == "gimbal_controller":
            record("gimbal", None, [plugin_name])
        elif "gimbal" in plugin.get("filename", ""):
            raise ValueError("unknown gimbal controller: %s/%s" % (name, plugin_name))
    for include in root.findall(".//include"):
        uri = include.findtext("uri", "").strip()
        if not uri.startswith("model://"):
            raise ValueError("unsupported SDF include URI: %r" % uri)
        child_counts, child_devices, child_unpriced, child_resources = _inspect_model(
            models, uri[len("model://"):], snapshots, stack + (path,))
        for key in counts:
            counts[key] += child_counts[key]
        devices.extend(child_devices)
        unpriced.extend(child_unpriced)
        resources.extend(child_resources)
    return counts, devices, unpriced, resources


def judge_args(counts):
    """Return exactly the seven safe numeric private ROS startup parameters."""
    return " ".join("_%s:=%d" % (key, counts[key]) for key in PRICES)


def inventory(scene, models):
    scene, models = Path(scene).resolve(), Path(models).resolve()
    if not models.is_dir():
        raise ValueError("models directory does not exist: %s" % models)
    scene_bytes = scene.read_bytes()
    root = ET.fromstring(scene_bytes)
    if root.tag != "launch":
        raise ValueError("expected a ROS launch file: %s" % scene)
    groups = root.findall(".//group[@ns]")
    names = [g.get("ns") for g in groups]
    if len(names) != len(set(names)):
        raise ValueError("duplicate aircraft namespace in scene")
    if len(names) != 6 or set(names) != set(UAV_NAMES):
        raise ValueError("scene must contain exactly typhoon_h480_0 through typhoon_h480_5")
    by_name = dict(zip(names, groups))
    snapshots, aircraft, totals = {}, [], _counts()
    for uav in UAV_NAMES:
        sdf_args = by_name[uav].findall(".//arg[@name='sdf']")
        if len(sdf_args) != 1 or not sdf_args[0].get("value"):
            raise ValueError("%s requires one explicit SDF model argument" % uav)
        model = sdf_args[0].get("value")
        counts, devices, unpriced, resources = _inspect_model(models, model, snapshots)
        cost = sum(counts[k] * PRICES[k] for k in PRICES)
        aircraft.append(dict(uav=uav, model=model, sdf_path=resources[0]["path"],
                             sdf_sha256=resources[0]["sha256"], counts=counts, cost=cost,
                             devices=devices, unpriced_stock_sensors=unpriced,
                             model_resources=resources))
        for key in totals:
            totals[key] += counts[key]
    cost = sum(totals[k] * PRICES[k] for k in PRICES)
    return dict(schema_version=1, scene_path=str(scene),
                scene_sha256=hashlib.sha256(scene_bytes).hexdigest(), models_path=str(models),
                count_basis="installed logical devices across all six aircraft",
                aircraft=aircraft, totals=totals, unit_prices=PRICES.copy(),
                total_cost=cost, sensor_penalty=round(cost * 0.003, 3),
                judge_args=judge_args(totals))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=Path(__file__).resolve().parents[1] /
                        "launch/robocup_with_laser.launch")
    parser.add_argument("--models", type=Path, default=Path.home() /
                        "PX4_Firmware/Tools/sitl_gazebo/models")
    parser.add_argument("--output", type=Path, help="write the complete JSON inventory")
    parser.add_argument("--judge-args", action="store_true",
                        help="stdout contains only the seven judge startup parameters")
    args = parser.parse_args(argv)
    try:
        result = inventory(args.scene, args.models)
        report = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
        if args.output:
            protected = {Path(result["scene_path"])}
            protected.update(Path(r["path"]) for a in result["aircraft"]
                             for r in a["model_resources"])
            if args.output.resolve() in protected:
                raise ValueError("output must not overwrite a launch/SDF input")
            args.output.write_text(report, encoding="utf-8")
        print(result["judge_args"] if args.judge_args else report, end="\n" if args.judge_args else "")
    except (OSError, ValueError, ET.ParseError) as exc:
        print("[sensor-inventory] FAIL %s" % exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
