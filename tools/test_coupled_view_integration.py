"""Assigned camera feedback may change observation height, never evidence."""
import copy
import unittest
from unittest.mock import patch

import test_tracking_view as fixtures


class CoupledViewIntegrationTests(unittest.TestCase):
    def run_crop(self, enabled, **changes):
        helper = fixtures.TrackingViewTests()
        agent = helper.agent()
        agent.local_z = 3.98
        agent.targets = {'t3': (6.48, 0., 1., 0.)}
        for i in range(21):
            sample = 100.+i*.2
            data = helper.payload(sample_s=sample, x=6.48,
                box=[358.5, 406.76, 400.1, 479.96],
                camera_tilt=.042605247317489436,
                report_ok=False, person_verdict_ok=True)
            data.update(changes)
            with patch.object(fixtures.sa, 'COUPLED_VIEW_GEOMETRY', enabled):
                helper.send(agent, data, now=sample+.1)
        return agent

    def test_enabled_candidate_reaches_assigned_feedback_without_new_evidence(self):
        baseline = self.run_crop(False)
        candidate = self.run_crop(True)
        self.assertEqual(baseline._tracking_altitude['goal'], 3.98)
        self.assertLess(candidate._tracking_altitude['goal'], 3.98)
        self.assertTrue(candidate._tracking_altitude['coupled_geometry'])
        self.assertEqual(candidate.targets, baseline.targets)
        self.assertEqual(candidate._visual_observations, {})
        self.assertEqual((candidate._confirm_start, candidate._last_confirm_t), (0., 0.))
        self.assertFalse(candidate._tracking_image['report_ok'])
        # The physical height has been held constant in this test: desired
        # lowering cannot claim that the measured viewing distance improved.
        self.assertAlmostEqual(candidate._tracking_image['range_floor'],
                               baseline._tracking_image['range_floor'])

    def test_failed_geometry_or_person_verdict_keeps_baseline_control(self):
        for changes in ({'geometry_ok': False}, {'person_verdict_ok': False},
                        {'report_ok': True}):
            with self.subTest(**changes):
                baseline = self.run_crop(False, **changes)
                candidate = self.run_crop(True, **changes)
                self.assertEqual(candidate._tracking_altitude,
                                 baseline._tracking_altitude)

    def test_stale_and_wrong_source_images_cannot_drive_candidate(self):
        helper = fixtures.TrackingViewTests()
        agent = self.run_crop(True)
        before = copy.deepcopy(agent._tracking_altitude)
        for change in ({'sample_s': 99.}, {'source_uav': 'u1'}):
            data = helper.payload(sample_s=104.2, x=6.48,
                box=[358.5, 406.76, 400.1, 479.96], report_ok=False)
            data.update(change)
            with patch.object(fixtures.sa, 'COUPLED_VIEW_GEOMETRY', True):
                helper.send(agent, data, now=104.3)
            self.assertEqual(agent._tracking_altitude, before)


if __name__ == '__main__':
    unittest.main()
