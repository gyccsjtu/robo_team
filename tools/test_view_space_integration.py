"""Real own-camera callbacks enable recovery without creating official evidence."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import test_tracking_view as fixture_module
import swarm_agent as sa


class ViewSpaceIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixture_module.TrackingViewTests()
        self.agent = self.fixture.agent()
        self.agent.local_z = 4.14
        self.agent.targets['t3'] = (8., 0., -.9, 0.)

    def send(self, sample, **changes):
        data = self.fixture.payload(sample_s=sample, box=[325., 300., 375., 480.],
            report_ok=False, person_verdict_ok=True, world_candidate_id='u0:session:7')
        data.update(changes)
        self.agent._t_seen = {'t3': sample}
        self.fixture.send(self.agent, data, now=sample+.1)

    def test_default_is_unchanged_and_three_new_frames_enable_recovery(self):
        for sample in (100., 100.2, 100.4):
            self.send(sample)
        with patch.object(sa, 'VIEW_SPACE_BACKOFF', False):
            self.assertNotIn('minimum_view_range', self.agent._view_motion_context(100.5))
        with patch.object(sa, 'VIEW_SPACE_BACKOFF', True):
            context = self.agent._view_motion_context(100.5)
            self.assertGreater(context['minimum_view_range'], 10.)
            vx, vy, urgent = sa.urgent_close_backoff_velocity(-.95, 0.,
                *context['own'], *context['target'], *context['velocity'],
                minimum_view_range=context['minimum_view_range'])
            self.assertTrue(urgent)
            self.assertAlmostEqual(vx, -1.35)
            self.assertEqual(vy, 0.)
        self.assertEqual(self.agent._visual_observations, {})
        self.assertFalse(self.agent._tracking_image['report_ok'])

    def test_single_frame_old_view_bad_person_and_new_identity_cannot_enable(self):
        self.send(100.)
        with patch.object(sa, 'VIEW_SPACE_BACKOFF', True):
            self.assertNotIn('minimum_view_range', self.agent._view_motion_context(100.1))
            self.assertIsNone(self.agent._view_motion_context(100.6))
            for sample in (100.2, 100.4):
                self.send(sample)
            self.send(100.6, person_verdict_ok=False)
            self.assertNotIn('minimum_view_range', self.agent._view_motion_context(100.7))
            self.send(100.8, world_candidate_id='u0:session:8')
            self.assertNotIn('minimum_view_range', self.agent._view_motion_context(100.9))


if __name__ == '__main__':
    unittest.main()
