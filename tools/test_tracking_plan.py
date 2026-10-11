"""Target ownership, search interruption, and stable camera-view regressions.

Run after sourcing ROS and adding coordination/src/robocup_navigation/src to
PYTHONPATH, as for tools/test_match_blockers.py.
"""
import json
import math
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
import swarm_agent


class TrackingPlanTests(unittest.TestCase):
    def test_own_control_image_prevents_false_loss_without_qualified_report(self):
        agent, _ = self.make_agent()
        agent._reobserve_goal = lambda *args: None
        agent._orbit_entered_t = 60.
        agent._tracking_image = dict(target_id='t0',sample_s=99.9,
                                    box=(300.,200.,400.,350.),height=480,
                                    geometry_ok=False,pixel_only=True)
        aborts = []
        agent._abort_orbit = lambda *args, **kwargs: aborts.append(args)
        with patch.object(swarm_agent.rospy.Time,'now',return_value=swarm_agent.rospy.Time(100.)):
            agent._fly_orbit()
        self.assertEqual(aborts,[])
        self.assertEqual(agent._visual_observations,{})

    def test_short_image_gap_after_long_follow_does_not_abandon_target(self):
        agent, _ = self.make_agent()
        agent._orbit_entered_t = 60.
        agent._visual_observations['t0'] = dict(sample_s=98.8, eligible=True)
        aborts = []
        agent._abort_orbit = lambda *args, **kwargs: aborts.append(args)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.)):
            agent._fly_orbit()
        self.assertEqual(aborts, [])
        self.assertEqual(agent._orbit_target, 't0')

    def test_own_image_renews_loss_budget_but_team_coordinates_do_not(self):
        agent, _ = self.make_agent()
        agent._reobserve_goal = lambda *args: None
        agent._orbit_entered_t = 40.
        agent._visual_observations['t0'] = dict(sample_s=79.9, eligible=True)
        aborts = []
        agent._abort_orbit = lambda *args, **kwargs: aborts.append(args)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.)), \
                patch.object(swarm_agent, 'ORBIT_GIVEUP', 20.):
            agent._fly_orbit()  # _t_seen is fresh team evidence, not own vision
            self.assertEqual(len(aborts), 1)
            aborts.clear()
            agent._visual_observations['t0']['sample_s'] = 99.8
            agent._fly_orbit()
            self.assertEqual(aborts, [])

    def test_never_seen_target_still_has_a_bounded_reacquisition_deadline(self):
        agent, _ = self.make_agent()
        agent._reobserve_goal = lambda *args: None
        agent._orbit_entered_t = 79.9
        aborts = []
        agent._abort_orbit = lambda *args, **kwargs: aborts.append(args)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.)), \
                patch.object(swarm_agent, 'ORBIT_GIVEUP', 20.):
            agent._fly_orbit()
        self.assertEqual(len(aborts), 1)

    def test_visible_actor_does_not_exit_follow_when_residence_lock_expires(self):
        agent, commands = self.make_agent()
        agent._confirm_close = True
        agent._confirm_close_tid = 't0'
        agent._confirm_close_until = 99.
        agent._confirm_start = 80.
        agent._last_confirm_t = 100.
        agent._visual_observations['t0'] = dict(sample_s=100., eligible=True)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.)):
            agent._fly_orbit()
        self.assertTrue(agent._confirm_close)
        self.assertEqual(agent._confirm_close_tid, 't0')
        self.assertGreater(agent._confirm_close_until, 100.)
        self.assertEqual(agent._confirm_start, 80.)
        self.assertEqual(len(commands), 1)

    def test_missing_own_images_cannot_renew_close_follow_lock(self):
        agent, _ = self.make_agent()
        agent._confirm_close = True
        agent._confirm_close_tid = 't0'
        agent._confirm_close_until = 99.
        agent._visual_observations['t0'] = dict(sample_s=98., eligible=True)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.)):
            agent._fly_orbit()
        self.assertFalse(agent._confirm_close)
        self.assertIsNone(agent._confirm_close_tid)

    def test_intercept_prediction_cannot_break_an_existing_observation_ring(self):
        agent, commands = self.make_agent(x=0., y=0., tx=8.5, ty=0.,
                                          target_vx=2.)
        agent._landing = False
        agent._takeoff_done = True
        agent._target_to_orbit = (11.3, 1.)  # manager's future intercept
        agent._confirm_close = True
        agent._confirm_close_tid = 't0'
        agent._confirm_start = 90.
        follows, plans = [], []
        agent._fly_orbit = lambda: follows.append(True)
        agent._need_replan_track = lambda goal, tid=None: plans.append(goal) or False
        agent._pick_local_goal = lambda: (1., 0.)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.)):
            agent._control()
        self.assertEqual(follows, [True])
        self.assertEqual(plans, [])
        self.assertTrue(agent._confirm_close)
        self.assertEqual(agent._orbit_target, 't0')
        self.assertEqual(agent._confirm_start, 90.)
        self.assertEqual(agent._look_at, (8.5, 0.))

    def test_intercept_is_still_used_for_far_route_but_not_camera_bearing(self):
        agent, _ = self.make_agent(x=0., y=0., tx=20., ty=0.)
        agent._landing = False
        agent._takeoff_done = True
        agent._target_to_orbit = (23., 4.)
        agent._confirm_close = False
        agent._orbit_target = None
        plans = []
        agent._need_replan_track = lambda goal, tid=None: plans.append(goal) or False
        agent._pick_local_goal = lambda: (1., 0.)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.)):
            agent._control()
        self.assertEqual(plans, [(23., 4.)])
        self.assertEqual(agent._look_at, (20., 0.))

    def test_close_pursuit_keeps_up_without_destroying_observation_ring(self):
        # Reproduce the 01:34 run: approach already permits chasing, but
        # close-mode feedforward + final caps previously forced .95 m/s.
        agent, commands = self.make_agent(x=42.0, y=20.0, tx=50.0, ty=20.0,
                                          target_vx=2.0)
        agent.grid = SimpleNamespace(origin=(-50.0, -60.0),
                                     width=420, height=260, resolution=0.5)
        agent._confirm_close = True
        agent._confirm_close_tid = 't0'
        agent._confirm_close_until = 200.0
        agent._escape_spent = {'t0'}
        agent._last_eligible_image = {'t0': 100.0}
        agent._visual_observations['t0'] = {'sample_s': 100.0, 'eligible': True}
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)):
            agent._fly_orbit()
        self.assertGreater(commands[-1][0], 1.9)
        self.assertLessEqual(math.hypot(*commands[-1]), swarm_agent.FLEE_SHEPHERD_SPEED)
        self.assertAlmostEqual(commands[-1][1], 0.0)
        self.assertEqual(agent._look_at, (50.0, 20.0))

        # A simple obstacle-free controller rollout validates standoff,
        # not Gazebo dynamics, positioning accuracy, or official elimination.
        for step in range(1, 401):
            dt = 0.05
            agent.local_xy = (agent.local_xy[0] + commands[-1][0]*dt, 20.0)
            now_stamp = swarm_agent.rospy.Time.from_sec(100.0 + step*dt)
            now = now_stamp.to_sec()
            agent.targets['t0'] = (50.0 + 2.0*step*dt, 20.0, 2.0, 0.0)
            agent._t_seen['t0'] = now
            agent._last_eligible_image['t0'] = now
            agent._visual_observations['t0']['sample_s'] = now
            with patch.object(swarm_agent.rospy.Time, 'now',
                              return_value=now_stamp):
                agent._fly_orbit()
            distance = agent.targets['t0'][0] - agent.local_xy[0]
            self.assertTrue(7.5 <= distance <= 8.5, distance)

    def test_close_pursuit_cap_requires_decision_and_fresh_image(self):
        agent, _ = self.make_agent()
        self.assertEqual(agent._close_follow_cap('t0', .1), swarm_agent.CLOSE_TRANSIT_SPEED)
        agent._spend_mode = {'t0'}
        self.assertEqual(agent._close_follow_cap('t0', .1), swarm_agent.FLEE_SHEPHERD_SPEED)
        self.assertEqual(agent._close_follow_cap('t0', 1.01), swarm_agent.CLOSE_TRANSIT_SPEED)
        self.assertEqual(agent._close_follow_cap('t0', -.1), swarm_agent.CLOSE_TRANSIT_SPEED)
        self.assertEqual(agent._close_follow_cap('t1', .1), swarm_agent.CLOSE_TRANSIT_SPEED)

    def make_agent(self, x=8.5, y=0.0, tx=0.0, ty=0.0,
                   target_vx=0.0, target_vy=0.0):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'u0'
        agent.local_xy, agent.offset = (x, y), (0.0, 0.0)
        agent.grid = SimpleNamespace(origin=(-20.0, -20.0),
                                     width=80, height=80, resolution=0.5)
        agent.assignment = SimpleNamespace(task_type=1, target_id='t0')
        agent._orbit_target = 't0'
        agent.targets = {'t0': (tx, ty, target_vx, target_vy)}
        agent._t_seen = {'t0': 100.0}
        agent._giveup_until = {}
        agent._stale_search_until = {}
        agent._flee_run_t = {}
        agent._escape_spent = set()
        agent._confirm_close = False
        agent._confirm_close_tid = None
        agent._close_range_ref = None
        agent._orbit_entered_t = 100.0
        agent._confirm_start = agent._last_confirm_t = 0.0
        agent._visual_observations = {}
        agent._look_at = None
        agent.los = SimpleNamespace(visible=lambda *_args: False)
        agent._world_trusted = lambda: True
        agent._orbit_stale = lambda: False
        agent._approach_cap = lambda *_args: 6.0
        agent._apply_friend_avoidance = lambda vx, vy: (vx, vy)
        agent._update_orbit = lambda: None
        agent._publish_claim = lambda: None
        commands = []
        agent._send_vel = lambda vx, vy: commands.append((vx, vy))
        return agent, commands

    def test_searcher_cannot_self_orbit_even_when_target_is_close(self):
        agent, _ = self.make_agent(tx=1.0, ty=0.0)
        agent.assignment = SimpleNamespace(task_type=0, target_id='cell_2_3')
        agent._orbit_target = None
        agent._reset_t, agent._reset_n = {}, {}
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)):
            agent._check_start_orbit()
            self.assertIsNone(agent._orbit_target)
            # A task for another actor also cannot be stolen by proximity.
            agent.assignment = SimpleNamespace(task_type=1, target_id='t1')
            agent._check_start_orbit()
            self.assertIsNone(agent._orbit_target)
            # At a search waypoint the actual control loop scans its cell;
            # proximity alone cannot jump into the pursuit controller.
            agent.assignment = SimpleNamespace(task_type=0,
                                               target_x=8.5, target_y=0.0)
            agent._takeoff_done = True
            agent._landing = False
            agent.local_z = 3.0
            agent._hold_camera_candidate = lambda: False
            scans = []
            agent._search_scan_step = lambda: scans.append(True)
            agent._fly_orbit = lambda: self.fail('search stole an actor')
            agent._control()
            self.assertEqual(scans, [True])

    def test_new_tracking_task_discards_search_velocity_and_late_path(self):
        agent, commands = self.make_agent()
        agent.assignment = SimpleNamespace(task_type=0, target_id='cell_2_3')
        agent._track_assigned_id = None
        agent._team_tracks = {}
        agent._search_scan_key = (2, 3, 0.0, 0.0)
        agent._search_scan_done = False
        agent._search_scan_failed = False
        agent._plan_lock = threading.Lock()
        agent._plan_event = threading.Event()
        agent._plan_pending = (30.0, 0.0)
        agent._plan_epoch = 0
        agent._plan_fail_t = 99.0
        agent.path = [(0.0, 0.0), (30.0, 0.0)]
        agent.path_target = (30.0, 0.0)
        agent._last_local_goal = (30.0, 0.0)
        agent._last_flight_v = (4.0, 0.0)
        agent._confirm_close = False
        started, finish = threading.Event(), threading.Event()

        def old_search_plan(goal):
            started.set()
            self.assertTrue(finish.wait(1.0))
            return [(0.0, 0.0), goal], goal, True

        calls = [0]

        def shutdown_after_one_plan():
            calls[0] += 1
            return calls[0] > 1

        agent._compute_plan = old_search_plan
        agent._plan_event.set()
        msg = SimpleNamespace(task_type=1, target_id='t0', uav_id='u0',
                              target_x=5.0, target_y=0.0,
                              cell_ix=-1, cell_iy=-1)
        with patch.object(swarm_agent.rospy, 'is_shutdown',
                          side_effect=shutdown_after_one_plan):
            worker = threading.Thread(target=agent._planner_loop, daemon=True)
            worker.start()
            self.assertTrue(started.wait(1.0))
            agent._assign_cb(msg)
            self.assertEqual(agent._last_flight_v, (0.0, 0.0))
            agent._stream_last_v()
            self.assertEqual(commands[-1], (0.0, 0.0))
            self.assertEqual(agent._plan_fail_t, 0.0)
            self.assertEqual(agent.path, [])
            finish.set()
            worker.join(1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(agent.path, [])
        self.assertIsNone(agent.path_target)
        self.assertEqual(agent._target_to_orbit, (5.0, 0.0))

    def test_stationary_and_walking_actor_keep_fixed_view_bearing(self):
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)):
            still, still_cmd = self.make_agent()
            still._fly_orbit()
            self.assertEqual(len(still_cmd), 1)
            self.assertAlmostEqual(still_cmd[0][0], -0.4, places=4)
            self.assertAlmostEqual(still_cmd[0][1], 0.0, places=4)
            self.assertEqual(still._look_at, (0.0, 0.0))

            walking, walking_cmd = self.make_agent(target_vx=0.5)
            walking._fly_orbit()
            self.assertEqual(len(walking_cmd), 1)
            self.assertGreater(walking_cmd[0][0], 0.0)
            self.assertAlmostEqual(walking_cmd[0][1], 0.0, places=4)

    def test_same_target_position_refresh_preserves_visual_streak(self):
        agent, _ = self.make_agent()
        agent._team_tracks = {'t0': {'u0'}}
        agent._track_assigned_id = 't0'
        agent._search_scan_key = None
        agent._search_scan_done = False
        agent._search_scan_failed = False
        agent._target_to_orbit = (0.0, 0.0)
        agent._confirm_start = 90.0
        agent._last_confirm_t = 100.0
        agent.path = [(8.5, 0.0), (5.0, 0.0)]
        agent.path_target = (5.0, 0.0)
        msg = SimpleNamespace(task_type=1, target_id='t0', uav_id='u0',
                              target_x=0.5, target_y=0.0)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)):
            agent._assign_cb(msg)
        self.assertEqual(agent._target_to_orbit, (0.5, 0.0))
        self.assertEqual(agent._orbit_target, 't0')
        self.assertEqual((agent._confirm_start, agent._last_confirm_t),
                         (90.0, 100.0))
        self.assertEqual(agent.path_target, (5.0, 0.0))

    def test_stale_approach_stops_at_absolute_deadline_until_new_image(self):
        agent, commands = self.make_agent(x=0.0, y=0.0, tx=5.0)
        agent.local_z = 3.0
        agent._takeoff_done = True
        agent._landing = False
        agent._track_assigned_id = 't0'
        agent._target_to_orbit = (5.0, 0.0)
        agent._orbit_target = None
        agent._t_seen['t0'] = 80.0
        agent._team_tracks = {'t0': {'u0'}}
        agent._search_scan_key = None
        agent._search_scan_done = False
        agent._search_scan_failed = False
        agent._plan_lock = threading.Lock()
        agent._plan_epoch = 0
        agent._plan_pending = None
        agent._plan_fail_t = 0.0
        agent.path = [(0.0, 0.0), (5.0, 0.0)]
        agent.path_target = (5.0, 0.0)
        agent._last_flight_v = (2.0, 0.0)
        agent._last_local_goal = (5.0, 0.0)
        msg = SimpleNamespace(task_type=1, target_id='t0', uav_id='u0',
                              target_x=5.0, target_y=0.0)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)):
            agent._control()
            self.assertEqual(commands[-1], (0.0, 0.0))
            self.assertIsNone(agent._target_to_orbit)
            self.assertEqual(agent.path, [])
            agent._assign_cb(msg)
            self.assertIsNone(agent._target_to_orbit)
        agent._t_seen['t0'] = 100.2
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.3)):
            agent._assign_cb(msg)
        self.assertEqual(agent._target_to_orbit, (5.0, 0.0))

    def test_boundary_observation_moves_inward_even_inside_push_zone(self):
        # Flight bounds are independent of this fixture's small SLAM grid.
        agent, commands = self.make_agent(x=0.0, y=59.0, tx=0.0, ty=58.0)
        agent.grid = SimpleNamespace(origin=(-55.0, -65.0),
                                     width=420, height=260, resolution=0.5)
        agent._confirm_close = True
        agent._confirm_close_tid = 't0'
        agent._confirm_close_until = 140.0
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)):
            agent._fly_orbit()
        self.assertEqual(len(commands), 1)
        self.assertLess(commands[0][1], 0.0)
        self.assertLessEqual(math.hypot(*commands[0]),
                             swarm_agent.CLOSE_TRANSIT_SPEED + 1e-6)

    def test_expired_pixel_look_creates_one_camera_review_not_actor_report(self):
        agent, _ = self.make_agent(x=2.0, y=3.0)
        agent.assignment = SimpleNamespace(task_type=0)
        agent._search_scan_key = (1, 1, 2.0, 3.0)
        agent._camera_cue = dict(bearing=0.5, started=100.0,
                                 until=102.0, color='blue',
                                 scan_key=agent._search_scan_key)
        agent.yaw = 0.4
        published = []
        agent._candidate_missed_pub = SimpleNamespace(publish=published.append)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(102.1)):
            self.assertFalse(agent._hold_camera_candidate())
            self.assertFalse(agent._hold_camera_candidate())
        self.assertEqual(len(published), 1)
        payload = json.loads(published[0].data)
        self.assertEqual((payload['x'], payload['y'], payload['yaw']),
                         (2.0, 3.0, 0.4))
        self.assertNotIn('target_id', payload)

        agent._camera_cue = dict(bearing=0.5, started=103.0,
                                 until=105.0, color='blue',
                                 scan_key=agent._search_scan_key)
        agent._visual_observations['t1'] = {'sample_s': 103.5,
                                            'eligible': True}
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(105.1)):
            self.assertFalse(agent._hold_camera_candidate())
        self.assertEqual(len(published), 1)

        agent._camera_cue = dict(bearing=0.5, started=106.0,
                                 until=108.0, color='blue',
                                 scan_key=agent._search_scan_key)
        agent.assignment = SimpleNamespace(task_type=1, target_id='t1')
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(108.1)):
            self.assertFalse(agent._hold_camera_candidate())
        self.assertEqual(len(published), 1)


if __name__ == '__main__':
    unittest.main()
