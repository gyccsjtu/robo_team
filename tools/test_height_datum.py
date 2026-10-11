"""Reference calibration uses only unarmed stable local poses and known geometry."""
import unittest
from height_datum import GroundHeightDatum


class HeightDatumTests(unittest.TestCase):
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
