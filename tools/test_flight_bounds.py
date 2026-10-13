"""Flight envelope regressions; source ROS and catkin environments before running."""

import math
import os
import subprocess
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
    def test_old_external_envelope_cannot_enlarge_actual_imported_flight_limits(self):
        code = ('import sys; sys.path.insert(0, %r); '
                'import swarm_agent as a, swarm_manager as m; '
                'assert a.flight_bounds()==(-50.,150.,-60.,60.); '
                'assert (m.FLIGHT_X_MIN,m.FLIGHT_X_MAX,m.FLIGHT_Y_MIN,m.FLIGHT_Y_MAX)'
                '==a.flight_bounds()') % str(ROOT/'coordination/src/robocup_swarm/scripts')
        env=dict(os.environ, FLIGHT_X_MIN='-100', FLIGHT_X_MAX='200',
                 FLIGHT_Y_MIN='-65', FLIGHT_Y_MAX='65')
        subprocess.run([sys.executable,'-c',code],env=env,check=True,
                       capture_output=True,text=True,timeout=15.)

    def setUp(self):
        self.now_patch = patch.object(
            swarm_agent.rospy.Time, 'now',
            return_value=swarm_agent.rospy.Time(100.0))
        self.now_patch.start()
        self.addCleanup(self.now_patch.stop)

    def test_official_spawn_is_legal_and_moves_inward(self):
        self.assertEqual(swarm_agent.flight_bounds(),
                         (-50.0, 150.0, -60.0, 60.0))
        self.assertEqual((swarm_manager.FLIGHT_Y_MIN, swarm_manager.FLIGHT_Y_MAX),
                         (-60.0, 60.0))
        self.assertEqual(swarm_agent.search_bounds(),(-50.,130.,-60.,60.))
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
                ((150.5, 0.0), (6.0, 0.0), -1),
                ((0.0, -60.5), (0.0, -6.0), 1),
                ((0.0, 60.5), (0.0, 6.0), -1)):
            with patch.object(swarm_agent.SwarmAgent, 'world_xy',
                              new_callable=PropertyMock, return_value=xy):
                inward = agent._bounds_recovery_velocity()
                comp = inward[0] if xy[0] < -50 or xy[0] > 150 else inward[1]
                self.assertGreater(comp * sign, 0.0)
                guarded = agent._map_guard_velocity(*velocity)
                self.assertEqual(guarded, (0.0, 0.0))
        with patch.object(swarm_agent.SwarmAgent, 'world_xy',
                          new_callable=PropertyMock, return_value=(-43.0, -53.0)):
            self.assertEqual(agent._map_guard_velocity(-6.0, -6.0),
                             (-0.5, -0.5))

    def test_edge_actor_observed_from_inside_flight_box(self):
        for actor in ((130.0, 60.0), (-50.0, -60.0),
                      (130.0, 0.0), (0.0, 60.0)):
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

    def test_actor_edge_position_is_retained_in_primary_and_backup_updates(self):
        # The search rectangle does not clip the observed person to the
        # aircraft's safe inset. Only the separate observation goal is legal.
        actor = (128.0, 60.0)
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        self.assertEqual(manager._sanitize_dispatch_xy(*actor, 't0'), actor)
        manager._tracking, manager._backup = {'t0': 'u0'}, {'t0': 'u1'}
        manager.status = {}
        manager.assign_pub = _Publisher()
        manager._update_tracker_position('t0', *actor)
        self.assertEqual(len(manager.assign_pub.messages), 2)
        for message in manager.assign_pub.messages:
            self.assertEqual((message.target_x, message.target_y), actor)
        goal = swarm_agent.observation_ring_goal(128., 58., *actor, 8.,
            swarm_agent.flight_bounds(), margin=2.)
        self.assertEqual(swarm_agent.flight_waypoint(*goal), goal)
        self.assertLessEqual(goal[1], 58.)
        self.assertAlmostEqual(math.dist(goal, actor), 8.)

    def test_position_beyond_flight_box_is_not_clipped_to_fabricate_report_range(self):
        actor=(128.,70.)
        manager=swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        self.assertEqual(manager._sanitize_dispatch_xy(*actor,'t0'),actor)
        goal=swarm_agent.observation_ring_goal(128.,58.,*actor,8.,
            swarm_agent.flight_bounds(),margin=2.)
        self.assertEqual(swarm_agent.flight_waypoint(*goal),goal)
        self.assertEqual(goal[1],58.)
        self.assertGreaterEqual(math.dist(goal, actor), 12.)
        self.assertGreater(math.dist(goal, actor), 9.5)

    def test_search_corner_assignment_uses_safe_viewpoint_with_wider_sensor_extent(self):
        # The actual shared coverage intersection is tested separately; a
        # real actor-region corner still needs an interior camera viewpoint.
        with patch.object(swarm_manager, '_search_bounds',
                          return_value=(-55.0, 155.0, -65.0, 65.0)):
            manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
            manager._cell_waypoint = {}
            manager.assign_pub = _Publisher()
            manager._publish_assignment(
                'u0', (25, 17), SimpleNamespace(cx=127.5, cy=59.5))
            msg = manager.assign_pub.messages[-1]
            self.assertEqual((msg.target_x, msg.target_y),
                             swarm_manager.search_viewpoint(127.5, 59.5))
            self.assertLessEqual(msg.target_x, 130.0)
            self.assertLessEqual(msg.target_y, 58.0)
            self.assertEqual(manager._sanitize_dispatch_xy(150.0, 60.0, 't0'),
                             (150.0, 60.0))

    def test_planner_and_position_output_cannot_target_outside(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent._plan_lock = threading.Lock()
        agent._plan_epoch = 0
        agent._plan_event = threading.Event()
        agent._request_plan((155.0, 65.0))
        self.assertEqual(agent._plan_pending, (148.0, 58.0))

        agent.local_xy = (0.0, 0.0)
        agent.offset = (-50.0, -25.0)
        agent.local_z = 2.5
        agent._landing = False
        agent._takeoff_done = False
        agent.pos_pub = _Publisher()
        agent._publish_pos_output(0.0, 0.0, 2.5, 0.0)
        self.assertEqual(agent.pos_pub.messages[-1].pose.position.x, 0.0)
        agent._takeoff_done = True
        agent.local_xy = (199.0, 0.0)  # world x=149, near east edge
        agent._publish_pos_output(6.0, 0.0, 2.5, 0.0)
        sp = agent.pos_pub.messages[-1]
        self.assertEqual(sp.pose.position.x + agent.offset[0], 149.0)

    def test_output_cannot_invent_unscanned_reverse_or_move_a_stop(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.local_xy, agent.offset, agent.local_z = (149., 0.), (0., 0.), 3.
        agent._landing, agent._takeoff_done, agent.yaw = False, True, 0.
        agent._scan_t = 100.
        agent._scan = SimpleNamespace(ranges=[12.]*720, range_min=.1, range_max=12.,
            angle_min=-math.radians(135), angle_increment=math.radians(270)/719.)
        agent.pos_pub = _Publisher()
        agent._publish_pos_output(0., .1, 3., 0.)
        point = agent.pos_pub.messages[-1].pose.position
        self.assertEqual((point.x, point.y), (149., .1))
        agent.local_xy = (150.5, 0.)
        agent._publish_pos_output(0., 0., 3., 0.)
        point = agent.pos_pub.messages[-1].pose.position
        self.assertEqual((point.x, point.y), (150.5, 0.))

    def test_negative_infinite_scan_is_not_open_space(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.yaw, agent._scan_t = 0., 100.
        agent._scan = SimpleNamespace(ranges=[-math.inf]*720, range_min=.1, range_max=12.,
            angle_min=-math.radians(135), angle_increment=math.radians(270)/719.)
        self.assertEqual(agent._final_scan_cap(2., 0.), (0., 0.))


if __name__ == '__main__':
    unittest.main()
