"""Calibrated perspective, independent of detector thresholds and height offsets."""
import math
import unittest
import numpy as np
from perception import perception_real as pr


class BodyProjectionTests(unittest.TestCase):
    def test_known_vertical_segments_at_offsets_and_camera_attitudes(self):
        for pitch, roll in ((0, 0), (30, 0), (0, 20), (12, -15)):
            a, b = map(math.radians, (pitch, roll))
            rotation = np.array([[math.cos(a),0,math.sin(a)], [0,1,0],
                                 [-math.sin(a),0,math.cos(a)]]) @ np.array(
                                 [[1,0,0], [0,math.cos(b),-math.sin(b)],
                                  [0,math.sin(b),math.cos(b)]])
            for y in (-5., 0., 5.):
                foot = np.array([7., y, -3.])
                head = foot+np.array([0., 0., 1.7])
                optical = (rotation @ pr.M_OPT2LINK).T @ head
                top = pr.CY+pr.FY*optical[1]/optical[2]
                self.assertAlmostEqual(pr.projected_body_height(foot, top, rotation), 1.7)

    def test_off_axis_slant_range_is_not_body_height_scale(self):
        foot = np.array([7., 5., -3.])
        top = pr.CY+pr.FY*1.3/7.
        pixels = pr.FY*1.7/7.
        self.assertGreater(pixels*np.linalg.norm(foot)/pr.FY, pr.H_MAX)
        self.assertAlmostEqual(pr.projected_body_height(foot, top, np.eye(3)), 1.7)

    def test_nonphysical_and_nonfinite_geometry_is_rejected(self):
        self.assertIsNone(pr.projected_body_height([7., 0., -3.], float('nan'), np.eye(3)))
        self.assertIsNone(pr.projected_body_height([7., 0., -3.], 1000., np.eye(3)))

    def test_quality_ranks_steeper_calibrated_view_and_rejects_bad_input(self):
        far = pr.ground_position_uncertainty(3., 1/3., 9., 100.)
        near = pr.ground_position_uncertainty(3., .5, 6., 150.)
        self.assertLess(near, .6*far)
        self.assertIsNone(pr.ground_position_uncertainty(3., .1, 9., 100.))
        self.assertIsNone(pr.ground_position_uncertainty(float('nan'), .5, 6., 150.))
