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

    def test_binding_records_actual_bracket_and_interpolated_rotation(self):
        import json
        angle = .2
        rotated = np.array([[np.cos(angle), -np.sin(angle), 0.],
                            [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
        history = [(10., 0., 0., 4., np.eye(3),
                    dict(before_s=9.99, after_s=10.01, link_name='uav2::camera')),
                   (10.2, .4, 0., 4., rotated,
                    dict(before_s=10.19, after_s=10.21, link_name='uav2::camera')),
                   (10.4, .8, 0., 4., rotated)]
        align = self.actual_pose_alignment(history)
        evidence = dict(schema_version=1)
        actual = align(10.1, None, evidence=evidence)
        unchanged = align(10.1, None)
        self.assertEqual(actual[:3], unchanged[:3])
        np.testing.assert_allclose(actual[3], unchanged[3])
        self.assertEqual(evidence['first']['sample_s'], 10.)
        self.assertEqual(evidence['second']['sample_s'], 10.2)
        self.assertEqual(evidence['first']['service']['link_name'], 'uav2::camera')
        np.testing.assert_allclose(evidence['camera_rotation'], actual[3].reshape(-1))
        json.dumps(evidence, allow_nan=False)

    def test_rejected_alignment_does_not_fabricate_binding(self):
        history = [(10., 0., 0., 4., np.eye(3)), (10.2, .4, 0., 4., np.eye(3))]
        evidence = {}
        self.assertIsNone(self.actual_pose_alignment(history)(9., None, evidence=evidence))
        self.assertEqual(evidence, {})

    def test_old_pose_samples_remain_compatible_with_optional_binding(self):
        history = [(10., 0., 0., 4., np.eye(3)), (10.2, .4, 0., 4., np.eye(3))]
        evidence = {}
        self.actual_pose_alignment(history)(10.1, None, evidence=evidence)
        self.assertNotIn('service', evidence['first'])

    def test_image_callback_copies_original_header_with_image_timestamp(self):
        from types import SimpleNamespace as NS
        source = Path(__file__).parents[2]/'perception/perception_real.py'
        function = next(n for n in ast.parse(source.read_text(encoding='utf-8')).body
                        if isinstance(n, ast.FunctionDef) and n.name == 'on_img')
        latest = {}
        image = np.zeros((2, 3, 3), dtype=np.uint8)
        scope = dict(_lock=threading.Lock(), _latest=latest,
                     CvBridge=lambda: NS(imgmsg_to_cv2=lambda msg, encoding: image),
                     time=NS(time=lambda: 1000.),
                     rospy=NS(Time=NS(now=lambda: NS(to_sec=lambda: 10.3))))
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), scope)
        header = NS(seq=7, frame_id='uav_3/cgo3_camera_optical_frame',
                    stamp=NS(to_sec=lambda: 10.1))
        scope['on_img'](NS(header=header))
        header.frame_id = 'uav_1/other'
        self.assertEqual(latest['header'], dict(seq=7, frame_id='uav_3/cgo3_camera_optical_frame'))
        self.assertEqual(latest['stamp'], 10.1)
        self.assertIs(latest['img'], image)

    def test_actual_pose_sampler_keeps_service_interval_and_link_identity(self):
        from types import SimpleNamespace as NS
        source = Path(__file__).parents[2]/'perception/perception_real.py'
        function = next(n for n in ast.walk(ast.parse(source.read_text(encoding='utf-8')))
                        if isinstance(n, ast.FunctionDef) and n.name == '_sample_camera_pose')
        times = iter([10., 10.04])
        history = []
        pose = NS(position=NS(x=1., y=2., z=4.), orientation=NS(x=0., y=0., z=0., w=1.))
        response = NS(success=True, link_state=NS(pose=pose,
                      link_name='typhoon_h480_2::cgo3_camera_link', reference_frame='world'))
        scope = dict(pose_sampler=lambda link, reference: response,
                     CAM_LINK=response.link_state.link_name, CAM_OFF_BL=np.array([0.,0.,-.162]),
                     quat_to_R=lambda *q: np.eye(3), _pose_history=history,
                     _pose_history_lock=threading.Lock(),
                     rospy=NS(Time=NS(now=lambda: NS(to_sec=lambda: next(times))),
                              logwarn_throttle=lambda *args: self.fail(str(args))))
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), scope)
        scope['_sample_camera_pose'](None)
        self.assertEqual(len(history), 1)
        self.assertAlmostEqual(history[0][0], 10.02)
        self.assertAlmostEqual(history[0][3], 3.838)
        self.assertEqual(history[0][5]['link_xyz'], [1.,2.,4.])
        self.assertEqual(history[0][5]['before_s'], 10.)
        self.assertEqual(history[0][5]['after_s'], 10.04)
        self.assertEqual(history[0][5]['reference_frame'], 'world')

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
