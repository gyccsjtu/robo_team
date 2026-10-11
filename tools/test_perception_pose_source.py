"""ROS-free regression tests for legal image-time camera localization."""
import ast
import math
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


SOURCE = Path(__file__).resolve().parents[1] / "perception/perception_real.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))
FUNCTIONS = {
    "quat_to_R", "R_to_quat", "interpolate_rotation",
    "camera_pose_from_local", "on_pose", "pose_at_stamp",
}


def load_pose_functions(pz_scale=None):
    nodes = [node for node in TREE.body
             if isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS]
    # Evaluate the real default, rather than baking an old calibration into
    # the test and silently approving it forever.
    config = next(node for node in TREE.body if isinstance(node, ast.Assign)
                  and any(isinstance(t, ast.Name) and t.id == 'PZ_SCALE'
                          for t in node.targets))
    settings = dict(os=SimpleNamespace(environ={}))
    exec(compile(ast.Module(body=[config], type_ignores=[]), str(SOURCE), 'exec'),
         settings)
    namespace = dict(math=math, np=np, threading=threading,
                     CAM_OFF_BL=np.array([0.12, 0.06, -0.05]),
                     PZ_SCALE=settings['PZ_SCALE'] if pz_scale is None else pz_scale,
                     _lock=threading.Lock(), _pose_history=[],
                     _pose_fault=None,
                     rospy=SimpleNamespace(logerr=lambda *args: None))
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"),
         namespace)
    return namespace


class Stamp:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


def local_pose(stamp, x, y, z, yaw):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=Stamp(stamp)),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z),
            orientation=SimpleNamespace(x=0.0, y=0.0,
                                        z=math.sin(yaw / 2.0),
                                        w=math.cos(yaw / 2.0))))


class ImageTimePoseTests(unittest.TestCase):
    def setUp(self):
        self.api = load_pose_functions()
        self.offset = (-50.0, -25.0, 0.0)

    def test_left_camera_extrinsic_and_fixed_spawn_offset_rotate_together(self):
        msg = local_pose(100.0, 2.0, 3.0, 4.0, math.pi / 2.0)
        x, y, z, rotation = self.api["camera_pose_from_local"](
            msg, self.offset)
        self.assertAlmostEqual(x, -48.06, places=6)
        self.assertAlmostEqual(y, -21.88, places=6)
        self.assertAlmostEqual(z, 3.95, places=6)
        # Optical center points along body +X; body yaw 90 degrees points north.
        ray = rotation @ np.array([1.0, 0.0, 0.0])
        np.testing.assert_allclose(ray, [0.0, 1.0, 0.0], atol=1e-7)

    def test_image_uses_its_own_pose_time_not_latest_position(self):
        for stamp, x in ((100.0, 0.0), (101.0, 1.0)):
            self.api["on_pose"](local_pose(stamp, x, 0.0, 4.0, 0.0),
                                self.offset)
        pose = self.api["pose_at_stamp"](100.8)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose[0], -50.0 + 0.8 + 0.12, places=6)
        self.assertAlmostEqual(pose[1], -25.0 + 0.06, places=6)
        self.assertAlmostEqual(pose[2], 3.95, places=6)

    def test_metric_camera_pose_does_not_shrink_ground_projection(self):
        # A known ground ray with 8 m horizontal range from a 4 m camera.
        # MAVROS and extrinsics are metres, so the inferred range must stay
        # 8 m at every height, without an empirical scale from an old camera.
        for height in (2.6, 3.5, 4.2):
            with self.subTest(height=height):
                pose = self.api['camera_pose_from_local'](
                    local_pose(100., 0., 0., height+.05, 0.), (0., 0., 0.))
                ray = np.array([8., 0., -height])
                ground = np.array(pose[:3]) - pose[2]/ray[2]*ray
                self.assertAlmostEqual(ground[0]-pose[0], 8., places=6)
                self.assertAlmostEqual(ground[2], 0., places=6)

    def test_explicit_calibration_remains_opt_in(self):
        api = load_pose_functions(pz_scale=.98)
        pose = api['camera_pose_from_local'](
            local_pose(100., 0., 0., 4., 0.), (0., 0., 0.))
        self.assertAlmostEqual(pose[2], 3.95*.98, places=6)

    def test_yaw_slerp_matches_image_time(self):
        self.api["on_pose"](local_pose(100.0, 0, 0, 4, 0), self.offset)
        self.api["on_pose"](local_pose(101.0, 0, 0, 4, math.pi / 2),
                            self.offset)
        rotation = self.api["pose_at_stamp"](100.8)[3]
        self.assertAlmostEqual(math.atan2(rotation[1, 0], rotation[0, 0]),
                               math.radians(72.0), places=6)
        np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-7)

    def test_stale_image_and_reordered_pose_cannot_create_new_observation(self):
        self.api["on_pose"](local_pose(100.0, 0, 0, 4, 0), self.offset)
        self.api["on_pose"](local_pose(101.0, 1, 0, 4, 0), self.offset)
        self.api["on_pose"](local_pose(100.5, 900, 0, 4, 0), self.offset)
        self.assertEqual(len(self.api["_pose_history"]), 2)
        self.assertIsNone(self.api["pose_at_stamp"](99.9))
        self.assertIsNone(self.api["pose_at_stamp"](101.1))

    def test_in_flight_ekf_jump_disables_camera_reports(self):
        for k in range(12):
            self.api["on_pose"](local_pose(100.0 + k * 0.1,
                                           k * 0.1, 0, 3, 0), self.offset)
        self.assertIsNotNone(self.api["pose_at_stamp"](101.05))
        self.api["on_pose"](local_pose(101.13, 3.5, 0, 3, 0), self.offset)
        self.assertIsNotNone(self.api["_pose_fault"])
        self.assertEqual(self.api["_pose_history"], [])
        self.api["on_pose"](local_pose(101.23, 1.23, 0, 3, 0), self.offset)
        self.assertIsNone(self.api["pose_at_stamp"](101.23))

    def test_ground_estimator_settling_does_not_latch_fault(self):
        for k in range(12):
            self.api["on_pose"](local_pose(100.0 + k * 0.1,
                                           k * 0.01, 0, 0, 0), self.offset)
        self.api["on_pose"](local_pose(101.13, 3.5, 0, 0, 0), self.offset)
        self.assertIsNone(self.api["_pose_fault"])

    def test_formal_perception_has_no_gazebo_truth_service(self):
        imports = [(node.module, alias.name)
                   for node in ast.walk(TREE)
                   if isinstance(node, ast.ImportFrom)
                   for alias in node.names]
        self.assertNotIn(("gazebo_msgs.srv", "GetLinkState"), imports)
        self.assertNotIn("/gazebo/get_link_state", SOURCE.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
