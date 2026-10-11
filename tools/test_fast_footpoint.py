import json
import math
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from tools.evaluate_fast_footpoint import (audit, ground_xy, sensitivity_case,
                                         stereo_sensitivity)


class FootpointAuditTests(unittest.TestCase):
    def test_calibrated_level_camera_matches_independent_flat_ground_formula(self):
        # At height 4m/range 8m, the foot is 188 pixels below the horizon.
        np.testing.assert_allclose(ground_xy(376., 428., 4., np.eye(3)), [8., 0.])
        case = sensitivity_case(4., 8., 0., 5.)
        self.assertAlmostEqual(case['bottom_down_delta_m'], 8.-4.*376./193.)
        self.assertAlmostEqual(case['bottom_up_delta_m'], 4.*376./183.-8.)
        self.assertTrue(case['eligible_geometry'])

    def test_pitch_projection_does_not_change_known_ground_intersection(self):
        from tools.evaluate_fast_footpoint import rotation_pitch
        for pitch in (-8., 0., 8.):
            case = sensitivity_case(4., 8., pitch, 1.)
            u, v = case['projected_foot_px']
            np.testing.assert_allclose(ground_xy(u, v, 4., rotation_pitch(pitch)),
                                       [8., 0.], atol=1e-10)

    def test_horizon_nonfinite_and_impossible_geometry_do_not_gain_eligibility(self):
        self.assertIsNone(ground_xy(376., 240., 4., np.eye(3)))
        self.assertIsNone(ground_xy(376., float('nan'), 4., np.eye(3)))
        self.assertIsNone(ground_xy(376., 400., -1., np.eye(3)))
        self.assertFalse(sensitivity_case(3., 12., 0., 1.)['eligible_geometry'])
        self.assertFalse(sensitivity_case(6., 8., 0., 1.)['eligible_geometry'])

    def test_short_stereo_baseline_has_nontrivial_depth_sensitivity(self):
        row = stereo_sensitivity([8.])[0]
        self.assertAlmostEqual(row['disparity_px'], 5.64)
        self.assertGreater(row['worst_depth_change_1.0px_m'], 1.7)

    def fixture(self, directory):
        import hashlib
        image = directory / 'image.png'
        self.assertTrue(cv2.imwrite(str(image), np.zeros((480, 752, 3), dtype=np.uint8)))
        reference = dict(source_uav='typhoon_h480_0', image_stamp=10., image=str(image),
            image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
            detections=[dict(label='blue', box=[100., 200., 120., 300.])])
        event = dict(source_uav=reference['source_uav'], image_stamp=10., image=str(image),
            anchor=False, results=[dict(id='local:1', label='blue', status='tracked',
                                      box=[100., 200., 120., 305.])],
            matches=[dict(id='local:1', reference_index=0)])
        (directory / 'reference.jsonl').write_text(json.dumps(reference)+'\n')
        (directory / 'tracking.jsonl').write_text(json.dumps(event)+'\n')
        return reference

    def test_yolo_agreement_cannot_become_foot_or_world_ground_truth(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            self.fixture(directory)
            result = audit(directory, directory / 'result')
            self.assertEqual(result['tracker_vs_yolo_bottom_difference_px']['p50'], 5.)
            self.assertFalse(result['footpoint_evaluable'])
            self.assertFalse(result['world_error_evaluable'])
            self.assertFalse(result['report_eligible'])
            annotation = json.loads((directory/'result/annotation_template.jsonl').read_text())
            self.assertIsNone(annotation['foot_px'])
            self.assertFalse(annotation['reviewed'])

    def test_independent_reviewed_annotation_only_enables_pixel_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            reference = self.fixture(directory)
            annotation = dict(reference, track_id='local:1', reviewed=True,
                              same_person=True, foot_visible=True, foot_px=[110., 299.])
            path = directory / 'annotations.jsonl'
            path.write_text(json.dumps(annotation)+'\n')
            result = audit(directory, directory / 'result', path)
            self.assertEqual(result['tracker_bottom_vs_human_foot_px']['p50'], 6.)
            self.assertTrue(result['footpoint_evaluable'])
            self.assertFalse(result['world_error_evaluable'])
            annotation['foot_visible'] = False
            path.write_text(json.dumps(annotation)+'\n')
            result = audit(directory, directory / 'occluded_result', path)
            self.assertFalse(result['footpoint_evaluable'])

    def test_changed_native_image_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            reference = self.fixture(directory)
            Path(reference['image']).write_bytes(b'wrong image')
            with self.assertRaisesRegex(ValueError, 'hash mismatch'):
                audit(directory, directory / 'result')


if __name__ == '__main__':
    unittest.main()
