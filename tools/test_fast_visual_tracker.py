"""Image-level contract/regression tests; no ROS, truth, or flight interface."""
import json
import unittest

import cv2
import numpy as np

from perception.fast_visual_tracker import FastVisualTracker


class FastVisualTrackerTests(unittest.TestCase):
    def setUp(self):
        self.old_threads = cv2.getNumThreads()
        cv2.setNumThreads(1)
        self.image = np.full((240, 320, 3), 35, dtype=np.uint8)
        rng = np.random.RandomState(314)
        # A textured upright foreground on a plain background, so tracking
        # cannot be rescued by abundant features on a textured ground plane.
        patch = rng.randint(50, 245, (96, 40, 3)).astype(np.uint8)
        self.image[60:156, 100:140] = cv2.GaussianBlur(patch, (3, 3), .6)
        self.box = [100, 60, 140, 156]
        self.detection = dict(id='camera0:12', label='blue', box=self.box, confidence=.83)

    def tearDown(self):
        cv2.setNumThreads(self.old_threads)

    def shifted(self, dx, dy):
        return cv2.warpAffine(self.image, np.float32([[1, 0, dx], [0, 1, dy]]),
                              (320, 240), borderValue=(35, 35, 35))

    def test_new_image_motion_changes_box_without_claiming_footpoint_or_report(self):
        tracker = FastVisualTracker()
        seed = tracker.seed(self.image, 10., [self.detection])[0]
        self.assertEqual(seed['status'], 'seeded')
        for i, (dx, dy) in enumerate(((5, 2), (11, 4), (17, 6)), 1):
            result = tracker.step(self.shifted(dx, dy), 10. + i * .1)[0]
            self.assertEqual(result['status'], 'tracked', result)
            np.testing.assert_allclose(result['box'], np.array(self.box) + [dx, dy, dx, dy], atol=.35)
            self.assertEqual(result['seed_s'], 10.)
            self.assertFalse(result['real_detection'])
            self.assertFalse(result['footpoint_valid'])
            self.assertFalse(result['control_eligible'])
            self.assertFalse(result['report_eligible'])
            self.assertGreater(result['quality'], .7)
            json.dumps(result, allow_nan=False)
        self.assertFalse(seed['report_eligible'])

    def test_scale_update_is_measured_not_fixed_box_translation(self):
        tracker = FastVisualTracker()
        tracker.seed(self.image, 1., [self.detection])
        affine = cv2.getRotationMatrix2D((120, 108), 0, 1.08)
        current = cv2.warpAffine(self.image, affine, (320, 240), borderValue=(35, 35, 35))
        result = tracker.step(current, 1.1)[0]
        self.assertEqual(result['status'], 'tracked', result)
        self.assertGreater(result['box'][2] - result['box'][0], 42.)
        self.assertGreater(result['box'][3] - result['box'][1], 102.)

    def test_absolute_detector_age_expires_despite_successful_intermediate_images(self):
        tracker = FastVisualTracker(max_age_s=.6)
        tracker.seed(self.image, 10., [self.detection])
        for i in range(1, 7):
            self.assertEqual(tracker.step(self.shifted(i, 0), 10 + i * .1)[0]['status'], 'tracked')
        result = tracker.step(self.shifted(7, 0), 10.601)[0]
        self.assertEqual(result['status'], 'expired')
        self.assertEqual(result['reason'], 'detector_anchor_expired')
        self.assertEqual(tracker.step(self.shifted(8, 0), 10.7), [])

    def test_frame_gap_does_not_produce_prediction(self):
        tracker = FastVisualTracker(max_gap_s=.3)
        tracker.seed(self.image, 1., [self.detection])
        result = tracker.step(self.shifted(20, 0), 1.301)[0]
        self.assertEqual(result['reason'], 'image_gap')
        self.assertEqual(result['status'], 'expired')

    def test_duplicate_and_reverse_images_do_not_advance_evidence(self):
        tracker = FastVisualTracker()
        tracker.seed(self.image, 1., [self.detection])
        for stamp in (1., .9):
            result = tracker.step(self.shifted(40, 0), stamp)[0]
            self.assertEqual(result['status'], 'rejected')
            self.assertEqual(result['reason'], 'non_increasing_image')
        result = tracker.step(self.shifted(5, 0), 1.1)[0]
        self.assertEqual(result['status'], 'tracked')
        self.assertAlmostEqual(result['box'][0], 105., delta=.3)
        bad_seed = dict(self.detection, box=[200, 60, 240, 156])
        self.assertEqual(tracker.seed(self.image, 1., [bad_seed])[0]['status'], 'rejected')
        self.assertEqual(tracker.step(self.shifted(10, 0), 1.2)[0]['status'], 'tracked')

    def test_same_image_step_then_detector_correction_is_allowed_once(self):
        tracker = FastVisualTracker()
        tracker.seed(self.image, 1., [self.detection])
        frame = self.shifted(5, 0)
        tracker.step(frame, 1.1)
        corrected = dict(self.detection, box=[105, 60, 145, 156])
        self.assertEqual(tracker.seed(frame, 1.1, [corrected])[0]['status'], 'seeded')
        self.assertEqual(tracker.seed(frame, 1.1, [corrected])[0]['status'], 'rejected')
        result = tracker.step(self.shifted(10, 0), 1.2)[0]
        self.assertEqual(result['seed_s'], 1.1)
        self.assertEqual(result['status'], 'tracked')

    def test_textureless_small_and_outside_boxes_are_not_anchored(self):
        for image, box, reason in (
                (np.full_like(self.image, 80), self.box, 'insufficient_person_texture'),
                (self.image, [100, 60, 109, 80], 'small_box'),
                (self.image, [-1, 60, 139, 156], 'box_outside_image')):
            with self.subTest(reason=reason):
                tracker = FastVisualTracker()
                result = tracker.seed(image, 1., [dict(self.detection, box=box)])[0]
                self.assertEqual(result['reason'], reason)
                self.assertEqual(tracker.step(image, 1.1), [])

    def test_plain_uniform_can_use_head_and_arm_features_without_ground(self):
        image = np.full_like(self.image, 35)
        image[60:156, 100:140] = (160, 40, 30)
        rng = np.random.RandomState(42)
        image[60:69, 108:132] = rng.randint(30, 240, (9, 24, 3)).astype(np.uint8)
        image[88:115, 102:105] = rng.randint(30, 240, (27, 3, 3)).astype(np.uint8)
        tracker = FastVisualTracker()
        result = tracker.seed(image, 1., [self.detection])[0]
        self.assertEqual(result['status'], 'seeded', result)
        moved = cv2.warpAffine(image, np.float32([[1, 0, 4], [0, 1, 2]]),
                               (320, 240), borderValue=(35, 35, 35))
        result = tracker.step(moved, 1.1)[0]
        self.assertEqual(result['status'], 'tracked', result)
        np.testing.assert_allclose(result['box'], [104, 62, 144, 158], atol=.4)
        self.assertFalse(result['footpoint_valid'])

    def test_full_occlusion_loses_track_and_requires_genuine_reseed(self):
        tracker = FastVisualTracker()
        tracker.seed(self.image, 1., [self.detection])
        result = tracker.step(np.full_like(self.image, 35), 1.1)[0]
        self.assertEqual(result['status'], 'lost')
        self.assertEqual(tracker.step(self.image, 1.2), [])
        self.assertEqual(tracker.seed(self.image, 1.2, [self.detection])[0]['status'], 'seeded')

    def test_two_same_color_instances_keep_independent_ids(self):
        image = self.image.copy()
        image[70:166, 200:240] = image[60:156, 100:140][:, ::-1]
        second = dict(id='camera0:13', label='blue', box=[200, 70, 240, 166], confidence=.85)
        tracker = FastVisualTracker()
        tracker.seed(image, 1., [self.detection, second])
        new_image = cv2.warpAffine(image, np.float32([[1, 0, 5], [0, 1, 0]]), (320, 240))
        results = {r['id']: r for r in tracker.step(new_image, 1.1)}
        self.assertEqual(set(results), {'camera0:12', 'camera0:13'})
        self.assertAlmostEqual(results['camera0:12']['box'][0], 105., delta=.3)
        self.assertAlmostEqual(results['camera0:13']['box'][0], 205., delta=.3)
        # Correction of one ID must not silently reset the other's anchor age.
        tracker.seed(new_image, 1.1, [dict(self.detection, box=[105, 60, 145, 156])])
        out = {r['id']: r for r in tracker.step(new_image, 1.2)}
        self.assertEqual(out['camera0:12']['seed_s'], 1.1)
        self.assertEqual(out['camera0:13']['seed_s'], 1.)

    def test_changed_color_does_not_keep_same_texture_track(self):
        # Preserve luminance structure while changing clothing chroma.
        image = self.image.copy()
        intensity = image[60:156, 100:140, 0]
        image[60:156, 100:140] = np.stack((intensity, intensity // 5, intensity // 5), axis=2)
        tracker = FastVisualTracker()
        tracker.seed(image, 1., [self.detection])
        changed = image.copy()
        changed[60:156, 100:140] = changed[60:156, 100:140, ::-1]
        self.assertEqual(tracker.step(changed, 1.1)[0]['status'], 'lost')

    def test_capacity_and_invalid_arguments_are_explicit(self):
        tracker = FastVisualTracker(max_tracks=1)
        result = tracker.seed(self.image, 1., [self.detection, dict(self.detection, id='another')])
        self.assertEqual(result[1]['reason'], 'capacity')
        for kwargs in ({'max_age_s': float('nan')}, {'max_gap_s': 0}, {'max_tracks': 0}):
            with self.assertRaises(ValueError):
                FastVisualTracker(**kwargs)
        with self.assertRaises(ValueError):
            tracker.step(self.image, float('nan'))
        with self.assertRaises(ValueError):
            tracker.step(self.image[:, :, 0], 1.1)

    def test_repeated_rotation_and_return_does_not_inflate_box(self):
        tracker = FastVisualTracker()
        tracker.seed(self.image, 1., [self.detection])
        for i, angle in enumerate((4, 0, 4, 0, 4, 0), 1):
            affine = cv2.getRotationMatrix2D((120, 108), angle, 1.)
            frame = cv2.warpAffine(self.image, affine, (320, 240), borderValue=(35, 35, 35))
            result = tracker.step(frame, 1. + i * .08)[0]
            self.assertEqual(result['status'], 'tracked', result)
            if angle == 0:
                np.testing.assert_allclose(result['box'], self.box, atol=.4)

    def test_duplicate_spatial_instances_are_rejected_as_ambiguous(self):
        tracker = FastVisualTracker()
        tracker.seed(self.image, 1., [self.detection, dict(self.detection, id='camera0:13')])
        results = tracker.step(self.shifted(5, 0), 1.1)
        self.assertEqual(len(results), 2)
        for result in results:
            self.assertEqual(result['status'], 'lost')
            self.assertEqual(result['reason'], 'multiple_tracks_ambiguous')
            self.assertFalse(result['report_eligible'])
        self.assertEqual(tracker.step(self.shifted(10, 0), 1.2), [])

    def test_invalid_box_diagnostics_are_json_safe(self):
        tracker = FastVisualTracker()
        result = tracker.seed(self.image, 1., [dict(self.detection, box=[float('nan'), 60, 140, 156])])[0]
        self.assertEqual(result['reason'], 'invalid_box')
        json.dumps(result, allow_nan=False)

    def test_camera_motion_is_only_image_motion_not_world_person_motion(self):
        tracker = FastVisualTracker()
        tracker.seed(self.image, 1., [self.detection])
        result = tracker.step(self.shifted(8, -3), 1.1)[0]
        self.assertEqual(result['status'], 'tracked')
        self.assertNotIn('velocity', result)
        self.assertNotIn('world_position', result)


if __name__ == '__main__':
    unittest.main()
