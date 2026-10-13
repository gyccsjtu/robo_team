"""View-distance emergency intent; all requests still need final safety."""
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
import swarm_agent as agent


class ViewSpaceBackoffTests(unittest.TestCase):
    def intent(self, distance=8., velocity=-.9, required=None, requested=-.95):
        return agent.urgent_close_backoff_velocity(
            requested, 0., 0., 0., distance, 0., velocity, 0.,
            minimum_view_range=required)

    def test_missing_trusted_view_preserves_original_distance_trigger(self):
        self.assertEqual(self.intent(), (-.95, 0., False))
        self.assertEqual(self.intent(required=6.5), (-.95, 0., False))
        self.assertEqual(self.intent(distance=6.), (-1.35, 0., True))

    def test_sustained_cropped_view_uses_existing_backoff_formula_earlier(self):
        vx, vy, urgent = self.intent(required=10.5)
        self.assertTrue(urgent)
        self.assertAlmostEqual(vx, -1.35)
        self.assertEqual(vy, 0.)
        # Construction: a .9m/s approaching person outruns the available
        # viewing distance with a .95m/s retreat. This is not actor truth.
        self.assertAlmostEqual(abs(vx)-.9, .45)
        self.assertLessEqual(math.hypot(vx, vy), agent.CLOSE_BACKOFF_SPEED)

    def test_stationary_and_receding_people_do_not_gain_emergency_motion(self):
        for velocity in (0., .9):
            self.assertEqual(self.intent(velocity=velocity, required=10.5),
                             (-.95, 0., False))
        self.assertEqual(self.intent(distance=5., velocity=0., requested=0.),
                         (-.85, 0., False))

    def test_invalid_or_unbounded_view_distance_is_ignored(self):
        for value in (float('nan'), float('inf'), -1., 0.,
                      agent.VIEW_MOTION_RANGE+.01, True, '10.5', [], object()):
            with self.subTest(value=value):
                self.assertEqual(self.intent(required=value), (-.95, 0., False))

    def test_geometry_cannot_reduce_original_trigger_or_widen_past_motion_range(self):
        self.assertEqual(self.intent(distance=6., required=3.), (-1.35, 0., True))
        self.assertEqual(self.intent(distance=agent.VIEW_MOTION_RANGE+.1,
                                    required=agent.VIEW_MOTION_RANGE), (-.95, 0., False))
        self.assertTrue(self.intent(distance=11.9,
                                   required=agent.VIEW_MOTION_RANGE)[2])

    def test_original_speed_ceiling_also_caps_large_incoming_velocity(self):
        with patch.object(agent, 'CLOSE_BACKOFF_SPEED', 1.5):
            vx, vy, urgent = self.intent(velocity=-3., required=10.5)
            self.assertTrue(urgent)
            self.assertAlmostEqual(math.hypot(vx, vy), 1.5)

    def test_retreat_direction_follows_person_ray_not_fixed_world_axis(self):
        vx, vy, urgent = agent.urgent_close_backoff_velocity(
            0., -.95, 0., 0., 0., 8., 0., -.9, minimum_view_range=10.5)
        self.assertTrue(urgent)
        self.assertAlmostEqual(vx, 0.)
        self.assertAlmostEqual(vy, -1.35)
        # Coincident coordinates have no meaningful outward direction.
        self.assertEqual(agent.urgent_close_backoff_velocity(
            .1, .2, 0., 0., 0., 0., -.9, 0., minimum_view_range=10.5),
            (.1, .2, False))


if __name__ == '__main__':
    unittest.main()
