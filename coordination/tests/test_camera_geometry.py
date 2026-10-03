from pathlib import Path
import sys
import unittest
import ast
import threading
import numpy as np
sys.path.insert(0, str(Path(__file__).parents[2]/'perception'))
from camera_geometry import calibration, aligned_translation, vertical_extent


class CameraGeometryTests(unittest.TestCase):
    def test_vertical_height_at_image_edge_does_not_use_radial_range(self):
        # Same upright 1.8m actor on-axis and 45 degrees off-axis.
        for y in (0., 10., -10.):
            self.assertAlmostEqual(vertical_extent((0.,0.,4.5), (10.,y),
                (1.,y/10.,(1.8-4.5)/10.)), 1.8)

    def test_vertical_height_is_invariant_to_ray_scale_and_camera_position(self):
        # A pitched camera changes the ray's optical-depth normalization;
        # the world-space top and foot remain the same vertical line.
        for scale in (.2,1.,3.):
            ray = tuple(v*scale for v in (8.,5.,-2.7))
            self.assertAlmostEqual(vertical_extent((2.,-3.,4.5),(10.,2.),ray),1.8)

    def test_missing_or_degenerate_top_ray_cannot_supply_person_height(self):
        for ray in ((0.,0.,-1.),(float('nan'),1.,1.),(-1.,0.,-1.)):
            with self.assertRaises(ValueError):
                vertical_extent((0.,0.,4.5),(10.,0.),ray)

    def actual_pose_alignment(self, history):
        source = Path(__file__).parents[2]/'perception/perception_real.py'
        function = next(n for n in ast.parse(source.read_text(encoding='utf-8')).body
                        if isinstance(n, ast.FunctionDef) and n.name=='pose_at_stamp')
        scope = dict(_pose_history=history, _pose_history_lock=threading.Lock(),
                     aligned_translation=aligned_translation, np=np)
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), scope)
        return scope['pose_at_stamp']

    def test_delayed_frame_uses_bracketing_history_not_current_camera_pose(self):
        history = [(t, 2*t, 0., 4., np.eye(3)) for t in (10.,10.2,10.4,10.6,10.8,11.)]
        xyz_rotation = self.actual_pose_alignment(history)(10.15, (999.,999.,999.,np.eye(3)))
        self.assertAlmostEqual(xyz_rotation[0], 20.3)
        np.testing.assert_allclose(xyz_rotation[3], np.eye(3))

    def test_missing_history_never_substitutes_current_pose_for_old_image(self):
        pose = (1.,2.,4.,np.eye(3))
        for history in ([], [(10.,)+pose]):
            self.assertIsNone(self.actual_pose_alignment(history)(10., pose))

    def test_outside_retained_history_rejects_instead_of_fabricating_pose(self):
        history = [(t,0.,0.,4.,np.eye(3)) for t in (10.,10.2)]
        self.assertIsNone(self.actual_pose_alignment(history)(9., (0.,0.,4.,np.eye(3))))

    def test_original_radar_camera_and_stereo_are_distinct_calibrations(self):
        radar = calibration(640, 360, [205.47,0,320.5,0,205.47,180.5,0,0,1], [0]*5)
        stereo = calibration(752, 480, [376,0,376,0,376,240,0,0,1], [0]*5)
        self.assertNotEqual(radar, stereo)

    def test_invalid_or_distorted_raw_calibration_cannot_publish_geometry(self):
        for intrinsic, distortion in (([0]*9, []), ([1,0,2,0,1,2,0,0,1], [.1]),
                                      ([float('nan')]*9, [])):
            with self.assertRaises(ValueError):
                calibration(640, 360, intrinsic, distortion)

    def test_alignment_uses_time_delta_in_seconds(self):
        xyz = aligned_translation(10.15, (10.,0.,0.,4.), (10.2,.4,0.,4.))
        self.assertAlmostEqual(xyz[0], .3)
        # Old code produced -.1m here because it used velocity times a ratio.
        self.assertEqual(xyz[1:], (0.,4.))

    def test_unknown_stale_future_or_nonmonotonic_pose_evidence_is_rejected(self):
        for stamp, first, second in ((9., (10.,0,0,0), (10.2,0,0,0)),
                (11., (10.,0,0,0), (10.2,0,0,0)), (10., (10.,0,0,0), (10.,0,0,0))):
            with self.assertRaises(ValueError):
                aligned_translation(stamp, first, second)
