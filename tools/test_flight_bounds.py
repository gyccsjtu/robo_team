"""Flight envelope regressions; source ROS and catkin environments before running."""

import math
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import PropertyMock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))

import swarm_agent
import swarm_manager


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FlightBoundsTests(unittest.TestCase):
    def setUp(self):
        self.now_patch = patch.object(
            swarm_agent.rospy.Time, 'now',
            return_value=swarm_agent.rospy.Time(100.0))
        self.now_patch.start()
        self.addCleanup(self.now_patch.stop)

    def test_official_spawn_is_legal_and_moves_inward(self):
        self.assertEqual(swarm_agent.flight_bounds(),
                         (-50.0, 130.0, -50.0, 50.0))
        self.assertEqual(swarm_agent.flight_waypoint(-50.0, -25.0),
                         (-48.0, -25.0))
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        with patch.object(swarm_agent.SwarmAgent, 'world_xy',
                          new_callable=PropertyMock, return_value=(-50.0, -25.0)):
            self.assertEqual(agent._map_guard_velocity(-6.0, 0.0), (0.0, 0.0))
            self.assertEqual(agent._map_guard_velocity(2.0, 0.0), (2.0, 0.0))
            self.assertIsNone(agent._bounds_recovery_velocity())

    def test_outside_map_recovery_and_soft_guard_are_symmetric(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'u0'
        for xy, velocity, sign in (
                ((-50.5, 0.0), (-6.0, 0.0), 1),
                ((130.5, 0.0), (6.0, 0.0), -1),
                ((0.0, -50.5), (0.0, -6.0), 1),
                ((0.0, 50.5), (0.0, 6.0), -1)):
            with patch.object(swarm_agent.SwarmAgent, 'world_xy',
                              new_callable=PropertyMock, return_value=xy):
                inward = agent._bounds_recovery_velocity()
                comp = inward[0] if xy[0] < -50 or xy[0] > 130 else inward[1]
                self.assertGreater(comp * sign, 0.0)
                guarded = agent._map_guard_velocity(*velocity)
                self.assertEqual(guarded, (0.0, 0.0))
        with patch.object(swarm_agent.SwarmAgent, 'world_xy',
                          new_callable=PropertyMock, return_value=(-43.0, -43.0)):
            vx, vy = agent._map_guard_velocity(-6.0, -6.0)
            self.assertAlmostEqual(vx, vy)
            self.assertGreater(abs(vx), .5)
            self.assertLessEqual(abs(vx)*.6 + vx*vx/2.4, 5.0+1e-9)

    def test_edge_actor_observed_from_inside_flight_box(self):
        for actor in ((130.0, 50.0), (-50.0, -50.0),
                      (130.0, 0.0), (0.0, 50.0)):
            wx, wy = swarm_agent.flight_waypoint(*actor)
            goal = swarm_agent.observation_ring_goal(
                wx, wy, actor[0], actor[1], 8.0,
                swarm_agent.flight_bounds(), margin=2.0)
            self.assertEqual(swarm_agent.flight_waypoint(*goal), goal)
            self.assertLessEqual(math.dist(goal, actor), 9.5)
        # No feasible point on a distant ring still yields an interior goal.
        impossible = swarm_agent.observation_ring_goal(
            0.0, 0.0, 100.0, 100.0, 8.0,
            (0.0, 10.0, 0.0, 10.0), margin=2.0)
        self.assertEqual(impossible, (8.0, 8.0))

    def test_padded_map_does_not_expand_search_and_dispatch_limits(self):
        self.assertEqual(swarm_manager._search_bounds(),
                         (-50.0, 130.0, -50.0, 50.0))
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._cell_waypoint = {}
        manager.assign_pub = _Publisher()
        manager._publish_assignment(
            'u0', (0, 0), SimpleNamespace(cx=128.0, cy=49.0))
        msg = manager.assign_pub.messages[-1]
        self.assertLessEqual(msg.target_x, 128.0)
        self.assertLessEqual(msg.target_y, 48.0)
        self.assertEqual(manager._sanitize_dispatch_xy(150.0, 60.0, 't0'),
                         (128.0, 48.0))

    def test_planner_and_position_output_cannot_target_outside(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent._plan_lock = threading.Lock()
        agent._plan_event = threading.Event()
        agent._request_plan((155.0, 65.0))
        self.assertEqual(agent._plan_pending, (128.0, 48.0))

        agent.local_xy = (0.0, 0.0)
        agent.offset = (-50.0, -25.0)
        agent.local_z = 2.5
        agent._landing = False
        agent._takeoff_done = False
        agent.pos_pub = _Publisher()
        agent._publish_pos_output(0.0, 0.0, 2.5, 0.0)
        self.assertEqual(agent.pos_pub.messages[-1].pose.position.x, 0.0)
        agent._takeoff_done = True
        agent.local_xy = (179.0, 0.0)  # world x=129, near east edge
        agent._publish_pos_output(6.0, 0.0, 2.5, 0.0)
        sp = agent.pos_pub.messages[-1]
        self.assertEqual(sp.pose.position.x + agent.offset[0], 128.0)


if __name__ == '__main__':
    unittest.main()
