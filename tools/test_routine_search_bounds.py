"""Routine ground coverage and flight limits do not alter actor coordinates."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
sys.path.insert(0, str(ROOT/'coordination/src/robocup_swarm/scripts'))
import test_motion_safety as fixture_module
import swarm_agent as sa
import swarm_manager as sm
from routine_search import routine_point_allowed, routine_flight_bounds


class RoutineSearchBoundsTests(unittest.TestCase):
    def test_edge_and_corner_look_stay_available_from_inside(self):
        self.assertTrue(routine_point_allowed((121., 51.), (130., 60.)))
        self.assertFalse(routine_point_allowed((137.434, 20.5), (129.99, 20.5)))
        self.assertFalse(routine_point_allowed((121., 51.), (140., 60.)))
        for point in ((float('nan'), 0.), (0., float('inf')), (0.,), None):
            self.assertFalse(routine_point_allowed(point))

    def test_regular_boundary_waypoint_uses_actor_edge_not_distant_flight_edge(self):
        with patch.object(sm, 'ROUTINE_SEARCH_BOUNDS', False):
            self.assertEqual(sm.search_viewpoint(129.5, 20.5), (129.5, 20.5))
        with patch.object(sm, 'ROUTINE_SEARCH_BOUNDS', True):
            point = sm.search_viewpoint(129.5, 20.5)
            self.assertTrue(routine_point_allowed(point, (129.5, 20.5)))
            self.assertAlmostEqual(point[0], 121.)
        self.assertEqual(routine_flight_bounds((-50., 150., -60., 60.)),
                         (-50., 130., -60., 60.))

    def agent(self, x=124., task_type=0):
        fixture = fixture_module.OutputTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        agent = fixture.a
        agent.local_xy = (x, 0.)
        agent.assignment = NS(task_type=task_type)
        agent._takeoff_done = True
        agent._measured_velocity = (100., (3., 0.))
        agent._scan.ranges = [20.]*361
        agent._final_motion_gate = lambda vx, vy: (vx, vy)
        return agent

    def test_slow_search_intent_does_not_hide_fast_eastward_approach(self):
        agent = self.agent()
        with patch.object(sa, 'ROUTINE_SEARCH_BOUNDS', True):
            self.assertEqual(agent._output_motion_safety(.5, 0., 0., 0.)[:2], (0., 0.))

    def test_person_tracking_keeps_its_independent_safe_flight_area(self):
        agent = self.agent(task_type=1)
        with patch.object(sa, 'ROUTINE_SEARCH_BOUNDS', True):
            self.assertEqual(agent._output_motion_safety(.5, 0., 0., 0.)[:2], (.5, 0.))

    def test_outside_pose_allows_inward_return_but_stop_does_not_move(self):
        agent = self.agent(x=135.)
        agent._measured_velocity = (100., (0., 0.))
        with patch.object(sa, 'ROUTINE_SEARCH_BOUNDS', True):
            self.assertEqual(agent._output_motion_safety(-.5, 0., 0., 0.)[:2], (-.5, 0.))
            self.assertEqual(agent._output_motion_safety(.5, 0., 0., 0.)[:2], (0., 0.))
            self.assertEqual(agent._output_motion_safety(0., 0., 0., 0.)[:2], (0., 0.))


if __name__ == '__main__': unittest.main()
