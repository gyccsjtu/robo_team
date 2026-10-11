#!/usr/bin/env python3
"""Compare the installed team sensor geometry with XTDrone's stock sensors.

The team may choose mounting positions and a CPU ray implementation.  The
optical/scan specifications checked here are fixed by the competition rules.
"""

import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


LASER_FIELDS = (
    "update_rate", "ray/scan/horizontal/samples",
    "ray/scan/horizontal/resolution", "ray/scan/horizontal/min_angle",
    "ray/scan/horizontal/max_angle", "ray/range/min", "ray/range/max",
    "ray/range/resolution",
)
CAMERA_FIELDS = (
    "horizontal_fov", "image/width", "image/height", "image/format",
    "clip/near", "clip/far",
)
PLUGIN_FIELDS = (
    "Fx", "Fy", "Cx", "Cy", "distortionK1", "distortionK2",
    "distortionK3", "distortionT1", "distortionT2",
)


def _value(node, path):
    item = node.find(path)
    if item is None or item.text is None:
        raise ValueError("missing %s" % path)
    raw = item.text.strip()
    try:
        return float(raw)
    except ValueError:
        return raw


def _same_spec(team, official, fields, label):
    for field in fields:
        actual, expected = _value(team, field), _value(official, field)
        if isinstance(expected, float):
            equal = isinstance(actual, float) and abs(actual - expected) <= 1e-6
        else:
            equal = actual == expected
        if not equal:
            raise ValueError("%s %s=%r, XTDrone=%r" %
                             (label, field, actual, expected))


def check_spec(team_path, mirror_path, hokuyo_path, stereo_path):
    if team_path.read_bytes() != mirror_path.read_bytes():
        raise ValueError("PX4 与 XTDrone 的 team sensor 模型不一致")
    team = ET.parse(team_path).getroot()
    hokuyo = ET.parse(hokuyo_path).getroot()
    stereo = ET.parse(stereo_path).getroot()
    laser = team.find(".//sensor[@name='laser_2d']")
    official_laser = hokuyo.find(".//sensor[@name='laser']")
    if laser is None or official_laser is None or laser.get("type") != "ray":
        raise ValueError("找不到 team CPU ray 或 XTDrone Hokuyo 激光传感器")
    _same_spec(laser, official_laser, LASER_FIELDS, "2D laser")
    laser_plugin = laser.find("plugin")
    if laser_plugin is None:
        raise ValueError("2D laser 缺少 ROS 插件")
    for team_field, reference_field in (("min_range", "ray/range/min"),
                                        ("max_range", "ray/range/max"),
                                        ("updateRate", "update_rate")):
        if _value(laser_plugin, team_field) != _value(official_laser, reference_field):
            raise ValueError("2D laser 插件 %s 与官方模型不一致" % team_field)
    official_stereo = stereo.find(".//sensor[@type='multicamera']")
    if official_stereo is None:
        raise ValueError("找不到 XTDrone stereo_camera")
    official_plugin = official_stereo.find("plugin")
    for side in ("left", "right"):
        camera = team.find(".//sensor[@name='stereo_camera_%s']" % side)
        official_camera = official_stereo.find("camera[@name='%s']" % side)
        if camera is None or official_camera is None or camera.get("type") != "camera":
            raise ValueError("缺少 %s 相机" % side)
        _same_spec(camera, official_stereo, ("update_rate",), side)
        _same_spec(camera.find("camera"), official_camera, CAMERA_FIELDS, side)
        _same_spec(camera.find("plugin"), official_plugin, PLUGIN_FIELDS, side)
    return True


def main(argv=None):
    home = Path.home()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--team", type=Path, default=home /
        "PX4_Firmware/Tools/sitl_gazebo/models/typhoon_h480_laser/typhoon_h480_laser.sdf")
    parser.add_argument("--mirror", type=Path, default=home /
        "XTDrone/sitl_config/models/typhoon_h480_laser/typhoon_h480_laser.sdf")
    parser.add_argument("--hokuyo", type=Path, default=home /
        "XTDrone/sitl_config/models/hokuyo_lidar/model.sdf")
    parser.add_argument("--stereo", type=Path, default=home /
        "XTDrone/sitl_config/models/stereo_camera/model.sdf")
    args = parser.parse_args(argv)
    try:
        check_spec(args.team, args.mirror, args.hokuyo, args.stereo)
    except (OSError, ValueError, ET.ParseError) as exc:
        print("[sensor] FAIL %s" % exc, file=sys.stderr)
        return 1
    print("[sensor] PASS 双目光学参数与 2D Hokuyo 扫描参数符合 XTDrone")
    return 0


if __name__ == "__main__":
    sys.exit(main())
