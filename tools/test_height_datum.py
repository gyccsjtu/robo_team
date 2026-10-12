"""Reference calibration uses only unarmed stable local poses and known geometry."""
import unittest
import math
from height_datum import GroundHeightDatum


class HeightDatumTests(unittest.TestCase):
    def calibrated(self):
        datum = GroundHeightDatum(.22)
        for k in range(41):
            datum.observe(100.+k*.05, -.08, False, 0.)
        return datum

    def test_last_ground_tilt_cannot_freeze_an_empty_reference(self):
        datum = self.calibrated()
        datum.observe(102.05, -.08, False, math.radians(3.1))
        datum.set_armed(True)
        self.assertAlmostEqual(datum.observe(102.1, 2.6, True, 0.), .30)
        self.assertEqual(datum.reason, 'frozen_valid')

    def test_unstable_candidates_do_not_replace_valid_reference(self):
        datum = self.calibrated()
        datum.observe(102.05, -.045, False, 0.)
        datum.observe(102.1, -.115, False, 0.)
        self.assertAlmostEqual(datum.observe(102.15, 2.6, True, 0.), .30)

    def test_real_ground_reference_shift_still_requires_recalibration(self):
        datum = self.calibrated()
        self.assertIsNone(datum.observe(102.05, .40, False, 0.))
        self.assertIsNone(datum.observe(102.10, 2.6, True, 0.))
        self.assertEqual(datum.reason, 'armed_without_ground_reference')

    def test_ground_reference_shift_can_recover_before_arming(self):
        datum = self.calibrated()
        for k in range(41):
            datum.observe(102.05+k*.05, .40, False, 0.)
        self.assertAlmostEqual(datum.observe(104.1, 2.6, True, 0.), -.18)

    def test_reordered_bad_sample_cannot_invalidate_current_reference(self):
        datum = self.calibrated()
        datum.observe(101.5, 80., False, math.radians(10.))
        self.assertAlmostEqual(datum.observe(102.05, 2.6, True, 0.), .30)

    def test_armed_state_precedes_delayed_unarmed_pose_callback(self):
        datum = self.calibrated()
        datum.set_armed(True)
        self.assertAlmostEqual(datum.observe(102.05, 3., False, 0.), .30)

    def test_individual_ground_bias_is_removed_and_then_frozen(self):
        for bias in (-.13, .03, .15):
            datum = GroundHeightDatum(.22)
            for k in range(41):
                datum.observe(100.+k*.05, bias, False, 0.)
            self.assertAlmostEqual(datum.offset+bias, .22)
            offset = datum.offset
            datum.observe(102.1, 2.6, True, 0.)
            datum.observe(110., 4., False, 0.)
            self.assertEqual(datum.offset, offset)

    def test_airborne_restart_and_unstable_ground_cannot_invent_reference(self):
        datum = GroundHeightDatum(.22)
        self.assertIsNone(datum.observe(100., 3., True, 0.))
        self.assertIsNone(datum.observe(101., 0., False, 0.))
        datum = GroundHeightDatum(.22)
        for k in range(60):
            self.assertIsNone(datum.observe(100.+k*.05, (k%2)*.1, False, 0.))

    def test_unknown_arming_and_repeated_time_are_not_calibration_samples(self):
        datum = GroundHeightDatum(.22)
        for _ in range(100):
            self.assertIsNone(datum.observe(100., 0., None, 0.))
            self.assertIsNone(datum.observe(100., 0., False, 0.))
