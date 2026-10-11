"""Yaw-only runtime gates, timestamped orientation and measured image motion."""
import ast
import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from perception.fast_visual_node import (PoseBuffer, YawGate, allowed_sensor_topics,
                                        load_calibration, normalized_quaternion,
                                        parse_args, slerp, world_bearing)
from perception.fast_visual_tracker import FastVisualTracker
from tools.fast_visual_shadow import Frame, ShadowState


def yaw_quat(angle):
    return (0., 0., math.sin(angle/2), math.cos(angle/2))


class FastVisualNodeTests(unittest.TestCase):
    def setUp(self):
        self.cal = load_calibration()
        self.track = dict(id='session:red:2', label='red', sample_s=10.1, seed_s=10.,
                          status='tracked', quality=.8, box=[356., 190., 396., 290.],
                          real_detection=False, report_eligible=False, footpoint_valid=False)

    def build(self, **kwargs):
        row = dict(source='typhoon_h480_0', track=self.track, frame_stamp=10.1,
                   now_s=10.15, quaternion=(0., 0., 0., 1.), width=752, height=480)
        row.update(kwargs)
        return YawGate(self.cal).build(**row)

    def test_calibration_comes_from_perception_literal_source(self):
        self.assertEqual((752, 480, 376., 376., 376., 240.),
                         (self.cal.width, self.cal.height, self.cal.fx, self.cal.fy, self.cal.cx, self.cal.cy))
        self.assertEqual(0., world_bearing([356, 190, 396, 290], yaw_quat(0), self.cal))
        self.assertAlmostEqual(-math.pi/4, world_bearing([742, 190, 762, 290], yaw_quat(0), self.cal))
        self.assertAlmostEqual(math.pi/2, world_bearing([356, 190, 396, 290], yaw_quat(math.pi/2), self.cal))

    def test_ray_matches_actual_perception_under_roll_pitch_and_yaw(self):
        # Execute just the existing pure ray/rotation definitions, not the ROS
        # node. Arbitrary attitudes catch axis-order and vertical-pixel errors.
        source = Path(__file__).resolve().parents[1]/'perception/perception_real.py'
        tree = ast.parse(source.read_text())
        functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                     and node.name in ('quat_to_R', 'image_world_bearing')]
        namespace = dict(np=np, math=math, M_OPT2LINK=np.array(self.cal.optical_to_body),
                         FX=self.cal.fx, FY=self.cal.fy, CX=self.cal.cx, CY=self.cal.cy)
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), namespace)
        for raw_q, box in (((.12, -.16, .3, .9), [100, 80, 160, 230]),
                           ((-.2, .1, -.4, .85), [560, 240, 620, 450]),
                           ((.3, .2, 0., .9), [330, 170, 420, 310])):
            q = normalized_quaternion(raw_q)
            expected = namespace['image_world_bearing']((box[0]+box[2])*.5,
                        (box[1]+box[3])*.5, namespace['quat_to_R'](*q))
            self.assertAlmostEqual(expected, world_bearing(box, q, self.cal), places=12)

    def test_dynamic_or_missing_calibration_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'cal.py'
            path.write_text('FX = FY = 376.\n')
            with self.assertRaises(ValueError):
                load_calibration(path)
            path.write_text('FX = FY = read_camera()\n')
            with self.assertRaises(ValueError):
                load_calibration(path)

    def test_orientation_is_interpolated_at_new_image_not_seed_or_latest_pose(self):
        history = PoseBuffer()
        history.add(10., yaw_quat(0.))
        history.add(10.1, yaw_quat(math.pi/2))
        q, reason = history.at(10.05)
        self.assertIsNone(reason)
        self.assertAlmostEqual(math.pi/4, world_bearing(self.track['box'], q, self.cal))
        self.assertAlmostEqual(math.pi/2, world_bearing(self.track['box'], history.at(10.1)[0], self.cal))

    def test_quaternion_wrap_uses_short_arc(self):
        q = slerp(yaw_quat(math.radians(179)), yaw_quat(math.radians(-179)), .5)
        self.assertAlmostEqual(math.pi, abs(world_bearing(self.track['box'], q, self.cal)))
        np.testing.assert_allclose(slerp((0, 0, 0, 1), (0, 0, 0, -1), .5), (0, 0, 0, 1))

    def test_pose_diagnostics_distinguish_interpolation_and_endpoint_reuse(self):
        history = PoseBuffer()
        history.add(10., yaw_quat(0.))
        history.add(10.1, yaw_quat(.2))
        for stamp, expected_mode, expected_delta in (
                (10., 'exact', 0.), (10.04, 'interpolated', .06),
                (9.97, 'endpoint_before', .03), (10.12, 'endpoint_after', .02)):
            q, reason, mode, delta = history.at_with_diagnostics(stamp)
            self.assertIsNone(reason)
            self.assertEqual(expected_mode, mode)
            self.assertAlmostEqual(expected_delta, delta)
            self.assertEqual((q, reason), history.at(stamp))

    def test_rejected_pose_diagnostics_do_not_mask_supporting_time_gap(self):
        history = PoseBuffer()
        self.assertEqual((None, 'pose_unavailable', 'unavailable', None),
                         history.at_with_diagnostics(10.))
        history.add(10., yaw_quat(0.))
        history.add(10.3, yaw_quat(.2))
        for stamp, reason, mode, delta in (
                (9.9, 'pose_outside_history', 'rejected_endpoint_before', .1),
                (10.4, 'pose_outside_history', 'rejected_endpoint_after', .1),
                (10.1, 'pose_sample_gap', 'rejected_gap', .2)):
            actual_q, actual_reason, actual_mode, actual_delta = history.at_with_diagnostics(stamp)
            self.assertIsNone(actual_q)
            self.assertEqual((reason, mode), (actual_reason, actual_mode))
            self.assertAlmostEqual(delta, actual_delta)
            self.assertEqual((actual_q, actual_reason), history.at(stamp))

    def test_missing_stale_and_gapped_orientation_rejected(self):
        history = PoseBuffer()
        self.assertEqual('pose_unavailable', history.at(10.)[1])
        history.add(10., yaw_quat(0.))
        self.assertIsNone(history.at(10.05)[1])
        self.assertEqual('pose_outside_history', history.at(10.051)[1])
        history.add(10.3, yaw_quat(.2))
        self.assertEqual('pose_sample_gap', history.at(10.15)[1])
        self.assertEqual('pose_unavailable', self.build(quaternion=None)[1])

    def test_invalid_duplicate_pose_cannot_change_history(self):
        history = PoseBuffer()
        history.add(10., yaw_quat(0.))
        self.assertEqual('non_increasing_pose', history.add(10., yaw_quat(2.)))
        self.assertEqual('non_increasing_pose', history.add(9., yaw_quat(2.)))
        for q in ((0, 0, 0, 0), (0, 0, 1), (0, 0, float('nan'), 1)):
            with self.assertRaises(ValueError):
                history.add(11., q)
        self.assertEqual(1, len(history.samples))

    def test_pose_cache_bounded_by_time_and_count(self):
        history = PoseBuffer()
        for index in range(400):
            history.add(10.+index*.005, yaw_quat(0.))
        self.assertEqual(180, len(history.samples))
        history.add(20., yaw_quat(0.))
        self.assertEqual(1, len(history.samples))

    def test_payload_separates_local_pixel_identity_and_source_session(self):
        payload, reason = self.build()
        self.assertIsNone(reason)
        self.assertEqual('session:red:2', payload['seed_id'])
        self.assertEqual('red:2', payload['seed_candidate_id'])
        self.assertEqual('session', payload['candidate_session'])
        for name in ('real_detection', 'report_eligible', 'footpoint_valid'):
            self.assertIs(payload[name], False)
        self.assertTrue(payload['yaw_only'])
        self.assertNotIn('position', payload)
        self.assertNotIn('target_id', payload)
        self.assertNotIn('confidence', payload)
        json.dumps(payload, allow_nan=False)

    def test_genuine_seed_and_same_picture_are_not_new_tracking(self):
        self.assertEqual('not_tracked', self.build(track=dict(self.track, status='seeded'))[1])
        self.assertEqual('invalid_eligibility', self.build(track=dict(self.track, real_detection=True))[1])
        self.assertEqual('not_a_new_tracked_image', self.build(track=dict(self.track, seed_s=10.1))[1])
        self.assertEqual('not_a_new_tracked_image', self.build(frame_stamp=10.2)[1])
        self.assertEqual('invalid_eligibility', self.build(track=dict(self.track, report_eligible=True))[1])

    def test_stale_images_and_expired_detector_anchor_never_publish(self):
        self.assertEqual('image_age', self.build(now_s=10.251)[1])
        self.assertEqual('image_age', self.build(now_s=10.)[1])
        self.assertEqual('seed_age', self.build(track=dict(self.track, seed_s=9.49))[1])
        self.assertEqual('seed_age', self.build(track=dict(self.track, seed_s=9.54))[1])
        self.assertEqual('invalid_timing_or_quality', self.build(now_s=float('nan'))[1])

    def test_quality_size_and_bounds_rejected(self):
        for quality in (.349, 1.01, float('nan')):
            self.assertIsNone(self.build(track=dict(self.track, quality=quality))[0])
        self.assertIsNotNone(self.build(track=dict(self.track, quality=.35))[0])
        self.assertEqual('image_size_changed', self.build(width=640)[1])
        for box in ([0, 0, 0, 10], [-1, 5, 10, 20], [0, 0, 800, 480], [1, 2, 3], [1, 2, 3, float('nan')]):
            self.assertIsNone(self.build(track=dict(self.track, box=box))[0])

    def test_ambiguous_identity_and_vertical_bearing_rejected(self):
        for identity in ('red:2', 's:blue:2', 's:red:other', 's:red:2:4', ':red:2'):
            self.assertEqual('invalid_identity', self.build(track=dict(self.track, id=identity))[1])
        pitch90 = (0, math.sin(math.pi/4), 0, math.cos(math.pi/4))
        self.assertEqual('near_vertical_bearing', self.build(quaternion=pitch90)[1])

    def test_duplicate_does_not_refresh_but_second_red_is_separate(self):
        gate = YawGate(self.cal)
        args = ('typhoon_h480_0', self.track, 10.1, 10.15, yaw_quat(0.), 752, 480)
        self.assertIsNotNone(gate.build(*args)[0])
        self.assertEqual('duplicate_or_old_yaw_image', gate.build(*args)[1])
        args = list(args)
        args[1] = dict(self.track, id='session:red:3')
        self.assertIsNotNone(gate.build(*args)[0])

    def test_only_explicit_yaw_flag_and_allowed_sensors(self):
        self.assertFalse(parse_args(['--output', '/tmp/unused']).publish_yaw)
        self.assertTrue(parse_args(['--output', '/tmp/unused', '--publish-yaw']).publish_yaw)
        topics = allowed_sensor_topics(['typhoon_h480_0'])
        self.assertEqual(['/typhoon_h480_0/stereo_camera/left/image_raw',
                          '/typhoon_h480_0/mavros/local_position/pose',
                          '/perception/fast_tracker_seeds'], topics)
        for args in (['--uavs', '0,0'], ['--uavs', '6'], ['--max-hz', '21'], ['--duration', 'nan']):
            with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                parse_args(['--output', '/tmp/unused']+args)

    def test_measured_new_image_motion_produces_bearing_then_fails_closed(self):
        old_threads = cv2.getNumThreads()
        cv2.setNumThreads(1)
        try:
            image = np.full((480, 752, 3), 35, dtype=np.uint8)
            rng = np.random.RandomState(314)
            image[160:256, 350:390] = cv2.GaussianBlur(rng.randint(50, 245, (96, 40, 3)).astype(np.uint8), (3, 3), .6)
            detection = dict(id='session:red:2', label='red', box=[350, 160, 390, 256], confidence=.85)
            state = ShadowState(lambda: FastVisualTracker(max_tracks=6), lambda frame: frame.payload)
            result, _, reason = state.seed([Frame(10., image)], 10., [detection])
            self.assertIsNone(reason)
            self.assertIsNone(result)
            shifted = cv2.warpAffine(image, np.float32([[1, 0, 12], [0, 1, 4]]), (752, 480), borderValue=(35, 35, 35))
            result = state.step([Frame(10., image), Frame(10.1, shifted)])
            track = result['tracks'][0]
            payload, reason = self.build(track=track)
            self.assertIsNone(reason)
            self.assertAlmostEqual(-math.atan2(6., 376.), payload['bearing'], delta=.002)
            self.assertEqual(10., payload['seed_s'])
            self.assertFalse(payload['real_detection'])
            lost = state.step([Frame(10.2, np.zeros_like(image))])['tracks'][0]
            self.assertEqual('lost', lost['status'])
            self.assertEqual('not_tracked', self.build(track=lost, frame_stamp=10.2, now_s=10.25)[1])
            self.assertEqual([], state.step([Frame(10.3, shifted)])['tracks'])
        finally:
            cv2.setNumThreads(old_threads)


if __name__ == '__main__':
    unittest.main()
