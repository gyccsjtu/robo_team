"""Recorded blue-command replay and camera geometry, without simulator/truth."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa
from tracking_view import update_altitude_hold, view_height_window


class ViewMotionEnvelopeTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        p = patch.object(sa.rospy.Time, 'now',
                         side_effect=lambda: sa.rospy.Time.from_sec(self.now))
        p.start()
        self.addCleanup(p.stop)

    def view(self, altitude=3., distance=8., **changes):
        v = dict(target_id='t1', sample_s=self.now, position_sample_s=self.now,
                 height=480., width=752., fy=376., camera_tilt=0., bearing=0.,
                 geometry_ok=True, world_candidate_id='session:6',
                 box=(340., 240.+376.*(altitude-1.7)/distance,
                      375., 240.+376.*altitude/distance))
        v.update(changes)
        return v

    def agent(self, distance=8.54):
        a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id = 'u1'
        a.local_xy, a.offset, a.local_z = (50., 20.), (0., 0.), 3.
        a.altitude_layer, a.yaw = 3., 0.
        a.assignment = NS(task_type=1, target_id='t1', target_x=50.+distance, target_y=20.)
        a.targets = {'t1': (50.+distance, 20., 1., 0.)}
        a._t_seen = {'t1': self.now}
        a._tracking_image = self.view(distance=distance)
        a._orbit_target, a._confirm_close = 't1', True
        a._look_at = None
        a._takeoff_done, a._landing = True, False
        a._est_trustworthy = lambda: True
        a._bounds_recovery_velocity = lambda: None
        a._radar_guard_velocity = a._grid_guard_velocity = a._map_guard_velocity = lambda x, y: (x, y)
        a._oscillation_breakout = lambda x, y, **kw: (x, y)
        a._final_scan_cap = lambda x, y: (x, y)
        a._compute_desired_altitude = lambda: 0.
        a._last_cmd_v = (0., 0.)
        a._last_csv_t = 1e9
        a._climb_latch, a._climb_t0, a._alt_sag = False, None, 0.
        a._friend_positions, a._friend_seen = {}, {}
        a._scan_t = self.now
        a._scan = NS(angle_min=-math.pi, angle_increment=math.pi/180,
                     range_min=.1, range_max=20., ranges=[20.]*361)
        a.pos_pub, a.vel_pub = Mock(), Mock()
        return a

    def refresh(self, a):
        stamp = sa.rospy.Time.from_sec(self.now).to_sec()
        a._t_seen['t1'] = stamp
        a._tracking_image.update(sample_s=stamp, position_sample_s=stamp)
        a._scan_t = stamp

    def test_recorded_reversal_reaches_position_output_with_visual_slew(self):
        a = self.agent()
        # 2892.052 command then recorded braking/retreat directions. Values
        # are replay inputs; this checks setpoint constraints, not PX4 attitude.
        a._last_cmd_v = (-1.54576, 1.43530)
        for requested in [(0.02558, .72360)]*5+[(1.30316, -.51779)]*10+[(.848, .270)]*5:
            previous = a._last_cmd_v
            self.refresh(a)
            with patch.object(sa, 'FLIGHT_OUTPUT', 'pos'):
                a._send_vel(*requested)
            result = a._last_flight_v
            self.assertLessEqual(math.dist(previous, result), 1.2/20.+1e-9)
            position = a.pos_pub.publish.call_args.args[0].pose.position
            self.assertAlmostEqual(position.x-a.local_xy[0], result[0]*sa.POS_SP_LEAD)
            self.assertAlmostEqual(position.y-a.local_xy[1], result[1]*sa.POS_SP_LEAD)
            self.now += .05

    def test_every_hard_stop_stays_zero_and_cannot_revive_old_velocity(self):
        for guard in ('_radar_guard_velocity', '_grid_guard_velocity',
                      '_map_guard_velocity', '_final_scan_cap', 'requested_zero'):
            with self.subTest(guard=guard):
                a = self.agent()
                a._last_cmd_v = (2., .5)
                if guard != 'requested_zero':
                    setattr(a, guard, lambda x, y: (0., 0.))
                requested = (0., 0.) if guard == 'requested_zero' else (1., .2)
                with patch.object(sa, 'FLIGHT_OUTPUT', 'pos'):
                    a._send_vel(*requested)
                self.assertEqual(a._last_flight_v, (0., 0.))
                p = a.pos_pub.publish.call_args.args[0].pose.position
                self.assertEqual((p.x, p.y), a.local_xy)

    def test_missing_old_or_pixel_only_view_retains_legacy_slew(self):
        for change in ('missing', 'stale', 'pixel', 'geometry', 'old_position', 'search'):
            a = self.agent()
            if change == 'missing': a._tracking_image = None
            if change == 'stale': a._tracking_image['sample_s'] -= .501
            if change == 'pixel': a._tracking_image['pixel_only'] = True
            if change == 'geometry': a._tracking_image['geometry_ok'] = False
            if change == 'old_position': a._tracking_image['position_sample_s'] -= .501
            if change == 'search': a.assignment = NS(task_type=0, target_id='cell')
            with self.subTest(change=change):
                self.assertIsNone(a._view_motion_context(self.now))
                a._send_vel(1., 0.)
                self.assertAlmostEqual(a._last_flight_v[0], sa.MAX_ACC/sa.CTRL_RATE)

    def test_safety_turn_and_close_friend_keep_existing_response(self):
        for obstacle in ('radar', 'friend', 'bounds'):
            a = self.agent()
            a._last_cmd_v = (2., 0.)
            if obstacle == 'radar':
                a._radar_guard_velocity = lambda x, y: (.5, .5)
            elif obstacle == 'friend':
                a._friend_positions = {'u2': (51., 20., 3.)}
                a._friend_seen = {'u2': self.now}
            else:
                a._bounds_recovery_velocity = lambda: (.5, .5)
            a._send_vel(.5, .5)
            # Radar guard identical to the request is still explicit safety.
            if obstacle == 'radar':
                a._last_cmd_v = (2., 0.)
                a._send_vel(.9, .1)
            self.assertGreater(math.dist((2., 0.), a._last_flight_v), 1.2/20.+1e-5)

    def test_relative_braking_preserves_translation_and_tangent(self):
        # Full 2 m/s target translation is retained at the observation ring.
        self.assertEqual(sa.view_approach_velocity(2., .4, 0., 0., 8.25, 0., 2., .4),
                         (2., .4))
        velocity = sa.view_approach_velocity(2.6, .4, 0., 0., 8.54, 0., 1., 0.)
        self.assertAlmostEqual(velocity[1], .4)
        self.assertLess(velocity[0], 2.11)
        self.assertLessEqual(velocity[0]-1., math.sqrt(2.*1.2*(8.54-8.25))+1e-9)
        self.assertEqual(sa.view_approach_velocity(0., 0., 0., 0., 7., 0., -1., 0.),
                         (0., 0.))

    def test_routed_approach_and_orbit_share_pre_avoidance_braking(self):
        for orbit in (None, 't1'):
            a = self.agent()
            a._orbit_target = orbit
            a._adaptive_speed = lambda x, y: (x, y)
            result = a._apply_friend_avoidance(2.6, .4)
            self.assertLess(result[0], 2.11)
            self.assertAlmostEqual(result[1], .4)
            # A close friend still receives the final collision preference.
            a._friend_positions = {'u2': (51., 20., 3.)}
            a._friend_seen = {'u2': self.now}
            result = a._apply_friend_avoidance(2.6, .4)
            self.assertLess(result[0], 0.)

    def test_visual_entry_tolerance_captures_before_soft_brake_stops(self):
        a = self.agent(distance=8.4)
        a._confirm_close, a._confirm_close_tid = False, None
        a._target_to_orbit = (58.4, 20.)
        a._orbit_target = None
        a._giveup_until, a._stale_search_until = {}, {}
        a._world_trusted = lambda: True
        a._fly_orbit = Mock()
        a._need_replan_track = Mock(return_value=False)
        a._pick_local_goal = Mock(return_value=(51., 20.))
        a._control()
        a._fly_orbit.assert_called_once()
        a._need_replan_track.assert_not_called()
        self.assertTrue(a._confirm_close)

    def test_existing_csv_records_envelope_and_hard_stop_without_new_stream(self):
        a = self.agent()
        a._last_csv_t, a._last_orca_v, a._csv = 0., (0., 0.), Mock()
        a._tracking_image.update(needs_space=True, range_floor=9.)
        a._send_vel(1., .1)
        row = a._csv.write.call_args.kwargs
        self.assertTrue(row['visual_slew'])
        self.assertFalse(row['horizontal_stop'])
        self.assertTrue(row['view_needs_space'])
        self.assertEqual(row['view_range_floor'], 9.)
        self.now += .25
        self.refresh(a)
        a._map_guard_velocity = lambda x, y: (0., 0.)
        a._send_vel(1., .1)
        row = a._csv.write.call_args.kwargs
        self.assertFalse(row['visual_slew'])
        self.assertTrue(row['horizontal_stop'])

    def test_moving_target_approach_brakes_before_entering_push_zone(self):
        # A deterministic 20 Hz kinematic replay checks stopping distance.
        # It is deliberately not a claim about Gazebo/PX4 lag or judge success.
        own, target, velocity = 0., 11., 3.0
        for _ in range(180):
            preference = sa.view_approach_velocity(3., 0., own, 0., target, 0., 1., 0.)
            velocity, _ = sa.horizontal_slew_limit(*preference, (velocity, 0.),
                                                   close_mode=True, visual_mode=True)
            own, target = own+velocity*.05, target+.05
            self.assertGreaterEqual(target-own, 8.-1e-9)
        self.assertLess(target-own, 8.6)
        self.assertAlmostEqual(velocity, 1., delta=.02)

    def test_late_stationary_discovery_keeps_available_emergency_braking(self):
        # Full approach-cap -> pre-ORCA preference -> orbit entry -> send_vel
        # -> position-output chain. The flight state advances kinematically;
        # this reproduces late discovery without claiming PX4 dynamics.
        a = self.agent(distance=12.)
        a.targets['t1'] = (62., 20., 0., 0.)
        a._target_to_orbit = (62., 20.)
        a._orbit_target = None
        a._confirm_close, a._confirm_close_tid = False, None
        a._last_cmd_v = a._last_flight_v = (6., 0.)
        a._escape_spent, a._spend_mode, a._flee_run_t = {'t1'}, set(), {}
        a._giveup_until, a._stale_search_until = {}, {}
        a._world_trusted = lambda: True
        a._need_replan_track = lambda *args, **kw: False
        a._pick_local_goal = lambda: (62., 20.)
        a._adaptive_speed = lambda x, y: (x, y)
        a._orbit_stale = lambda: False
        a._obstructed_view_goal = lambda *args, **kw: None
        a._reobserve_goal = lambda *args: None
        a._update_orbit = a._publish_claim = lambda: None
        a._visual_observations = {}
        a.los = NS(visible=lambda *args: True)
        a._last_csv_t, a._last_orca_v, a._csv = 0., (0., 0.), Mock()
        self.assertEqual(a._approach_cap('t1', 12.), 6.)
        trace = []
        for _ in range(90):
            self.refresh(a)
            with patch.object(sa, 'FLIGHT_OUTPUT', 'pos'):
                a._control()
            p = a.pos_pub.publish.call_args.args[0].pose.position
            velocity = (p.x-a.local_xy[0])/sa.POS_SP_LEAD
            self.assertAlmostEqual(velocity, a._last_flight_v[0])
            if not trace:
                row = a._csv.write.call_args.kwargs
                self.assertFalse(row['visual_slew'])
                self.assertTrue(row['view_brake_fallback'])
                self.assertAlmostEqual(row['view_brake_room'], 3.75)
                self.assertAlmostEqual(row['view_brake_need'], 15.3)
            a.local_xy = (a.local_xy[0]+velocity*.05, 20.)
            trace.append((62.-a.local_xy[0], velocity))
            self.now += .05
        self.assertLessEqual(trace[0][1], 6.-sa.MAX_ACC/sa.CTRL_RATE+1e-9,
                             'late fresh imagery must not weaken the available brake')
        self.assertGreaterEqual(min(r for r, _ in trace), 7.5)
        self.assertLess(abs(trace[-1][1]), .2)

    def test_brake_budget_accounts_for_target_motion_and_one_control_step(self):
        room, need = sa.view_braking_distances((3., 0.), 0., 0., 12., 0., 0., 0.)
        self.assertAlmostEqual(room, 3.75)
        self.assertAlmostEqual(need, 3.9)  # Continuous stop fits; one step does not.
        self.assertGreater(need, room)
        room, need = sa.view_braking_distances((3., 0.), 0., 0., 12., 0., 2., 0.)
        self.assertAlmostEqual(need, 1./2.4+.05)
        self.assertLess(need, room)
        _, approaching = sa.view_braking_distances((3., 0.), 0., 0., 12., 0., -2., 0.)
        self.assertGreater(approaching, room)

    def test_late_tangent_request_cannot_hide_previous_inward_speed(self):
        a = self.agent(distance=12.)
        a.targets['t1'] = (62., 20., 0., 0.)
        a._orbit_target, a._confirm_close = None, False
        a._last_cmd_v = (6., 0.)
        a._send_vel(0., 2.)
        # Desired radial speed is zero, but last command still needs 15.3 m.
        self.assertAlmostEqual(math.dist((6., 0.), a._last_flight_v),
                               sa.MAX_ACC/sa.CTRL_RATE)

    def test_outward_speed_and_target_translation_retain_soft_slew(self):
        for previous, desired, target_v in ((-6., -5., 0.), (3., 2.5, 2.)):
            with self.subTest(previous=previous, target_v=target_v):
                a = self.agent(distance=12.)
                a.targets['t1'] = (62., 20., target_v, 0.)
                a._orbit_target, a._confirm_close = None, False
                a._last_cmd_v = (previous, 0.)
                a._send_vel(desired, 0.)
                self.assertAlmostEqual(abs(a._last_flight_v[0]-previous),
                                       sa.VIEW_MAX_ACC/sa.CTRL_RATE)

    def test_four_meter_height_replay_satisfies_both_camera_constraints(self):
        state, altitude = None, 4.
        for i in range(80):
            self.now = 100.+i*.2
            old = altitude
            state = update_altitude_hold(state, self.view(altitude, 8.), altitude, 8., 2.6, 4.2)
            altitude = state['goal']
            self.assertLessEqual(abs(altitude-old), .03+1e-8)
            self.assertGreaterEqual(altitude, 2.6)
        self.assertGreaterEqual(altitude/math.hypot(altitude, 8.), .33-1e-9)
        tilted_foot = 240.+376.*math.tan(math.atan2(altitude, 8.)+math.radians(10.))
        self.assertLessEqual(tilted_foot, .98*480.+1e-8)
        self.assertLessEqual(altitude, 3.15105)

    def test_infeasible_close_window_requests_space_without_descending(self):
        state = None
        for i in range(20):
            self.now = 100.+i*.2
            view = self.view(2.6, 6.3)
            lower, upper = view_height_window(view, 6.3, 2.6, 4.2)
            self.assertGreater(lower, upper)
            state = update_altitude_hold(state, view, 2.6, 6.3, 2.6, 4.2)
            self.assertEqual(state['goal'], 2.6)
            self.assertTrue(state['needs_space'])
            self.assertGreater(state['range_floor'], 6.3)
        a = self.agent(distance=6.3)
        a._tracking_image.update(needs_space=True, range_floor=10.2)
        self.assertEqual(a._view_motion_context(self.now)['radius'], 10.2)

    def test_tilt_and_repeated_images_cannot_integrate_height(self):
        view = self.view(4., 8.)
        state = update_altitude_hold(None, view, 4., 8., 2.6, 4.2)
        for _ in range(50):
            state = update_altitude_hold(state, view, 4., 8., 2.6, 4.2)
            self.assertEqual(state['goal'], 4.)
        for i in range(1, 30):
            view.update(sample_s=100.+i*.2, camera_tilt=math.radians(12.))
            state = update_altitude_hold(state, view, 4., 8., 2.6, 4.2)
            self.assertEqual(state['goal'], 4.)


if __name__ == '__main__':
    unittest.main()
