"""Replay startup progress and actual output; no simulator or ROS master needed."""
import math
import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                      'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa


class EarlyTakeoffProgressTests(unittest.TestCase):
    def setUp(self):
        self.time = 100.
        patches = [
            patch.dict(os.environ, {'TAKEOFF_PROGRESS_RECOVERY_EARLY': '1'}),
            patch.object(sa, 'MOTION_SAFETY', True),
            patch.object(sa, 'FLIGHT_OUTPUT', 'pos'),
            patch.object(sa.rospy.Time, 'now',
                         side_effect=lambda: sa.rospy.Time.from_sec(self.time)),
        ]
        self.param = Mock(return_value=NS(success=True, value=NS(real=2.5)))
        patches.append(patch.object(sa.rospy, 'ServiceProxy', return_value=self.param))
        for p in patches:
            p.start()
            self.addCleanup(p.stop)

    def agent(self):
        a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id = 'typhoon_h480_1'
        a.state = NS(connected=True, armed=True, mode='OFFBOARD', system_status=4)
        a.local_z, a.local_xy = -.1, (.02, -.01)
        a.roll = a.pitch = 0.
        a._anchor_done = True
        a._est_recent_speed = lambda: .02
        a.mode_srv, a.arm_srv, a._send_vel = Mock(), Mock(), Mock()
        a.mode_srv.return_value = NS(mode_sent=True)
        a._pose_sample_s = a._local_prev_t = self.time
        a._pos_sp, a._pos_sp_t = (.02, -.01, .4), self.time
        return a

    def tick(self, a, t, z=None, command=True, recover=False):
        self.time = t
        if z is not None:
            a.local_z = z
        stamp = sa.rospy.Time.from_sec(t).to_sec()
        a._pose_sample_s = a._local_prev_t = stamp
        if command:
            a._pos_sp, a._pos_sp_t = (*a.local_xy, a.local_z + .5), stamp
        a._check_climb_progress()
        return a._recover_stalled_takeoff() if recover else None

    def run_until(self, a, elapsed, z=None, command=True):
        for i in range(int(round(elapsed / .05)) + 1):
            t = 100. + i * .05
            self.tick(a, t, z=z(t) if callable(z) else z, command=command)

    def test_logged_grounded_misclassification_recovers_after_4_5s(self):
        a = self.agent()
        # PX4 reported airborne before its takeoff ramp, despite zero progress.
        a.landed_state = 2
        self.run_until(a, 4.5)
        self.assertTrue(a._recover_stalled_takeoff())
        a.mode_srv.assert_called_once_with(0, 'AUTO.TAKEOFF')
        self.param.assert_called_once_with('MIS_TAKEOFF_ALT')
        a.arm_srv.assert_not_called()
        self.assertEqual(a._takeoff_early_progress['started'], 100.)

    def test_normal_ramp_budget_is_not_shortened(self):
        a = self.agent()
        self.run_until(a, 4.)
        self.assertFalse(a._recover_stalled_takeoff())
        for i in range(1, 10):
            self.tick(a, 104. + i * .05)
        self.tick(a, 104.499)
        self.assertFalse(a._recover_stalled_takeoff())
        self.tick(a, 104.5)
        self.assertTrue(a._recover_stalled_takeoff())

    def test_normal_a2_measured_rise_does_not_request_mode(self):
        a = self.agent()
        # Same aircraft ULog: first climb starts near z=-.156; useful rise
        # appears during the normal three-second takeoff ramp, then climbs.
        def altitude(t):
            age = t - 100.
            return -.156 if age < 2. else -.156 + .3 * (age - 2.)
        for i in range(101):
            self.assertFalse(self.tick(a, 100. + i * .05,
                                       altitude(100. + i * .05), recover=True))
        a.mode_srv.assert_not_called()

    def test_actual_partial_rise_restarts_only_no_progress_window(self):
        a = self.agent()
        self.run_until(a, 4.5, z=lambda t: -.1 if t < 102. else .025)
        self.assertFalse(a._recover_stalled_takeoff())
        early = a._takeoff_early_progress
        self.assertEqual(early['started'], 100.)
        self.assertEqual(early['progress_s'], 102.)
        for i in range(1, 41):
            self.tick(a, 104.5 + i * .05)
        self.assertTrue(a._recover_stalled_takeoff())
        self.assertEqual(early['started'], 100.)

    def test_stale_or_repeated_output_cannot_establish_climb_intent(self):
        a = self.agent()
        self.run_until(a, 4.5, command=False)
        self.assertFalse(a._recover_stalled_takeoff())
        self.assertIsNone(a._takeoff_early_progress['intent_since'])
        self.param.assert_not_called()

    def test_held_output_breaks_intent_without_resetting_first_start(self):
        a = self.agent()
        self.run_until(a, 2.)
        self.time = 102.05
        a._pos_sp = (*a.local_xy, a.local_z)
        a._pos_sp_t = sa.rospy.Time.from_sec(self.time).to_sec()
        self.tick(a, self.time, command=False)
        for i in range(1, 50):
            self.tick(a, 102.05 + i * .05)
        self.assertFalse(a._recover_stalled_takeoff())
        self.assertEqual(a._takeoff_early_progress['started'], 100.)
        self.param.assert_not_called()

    def test_desired_climb_cannot_replace_a_safety_vetoed_output(self):
        a = self.agent()
        a.requested_vz = .5
        for i in range(91):
            t = 100. + i * .05
            self.time = t
            a._pos_sp, a._pos_sp_t = (*a.local_xy, a.local_z), sa.rospy.Time.from_sec(t).to_sec()
            self.tick(a, t, command=False)
        self.assertFalse(a._recover_stalled_takeoff())
        self.param.assert_not_called()

    def test_landing_failsafe_or_unsafe_motion_never_triggers_early(self):
        cases = [('state.mode', 'AUTO.LAND'), ('state.mode', 'AUTO.RTL'),
                 ('state.mode', 'MANUAL'), ('state.system_status', 5),
                 ('state.system_status', 6), ('state.armed', False),
                 ('state.connected', False), ('_landing', True),
                 ('_takeoff_done', True), ('_localization_fault', True),
                 ('_ekf_frozen', True), ('_anchor_done', False),
                 ('local_xy', (.51, 0.)), ('local_z', .31),
                 ('_takeoff_max_z', .31), ('roll', math.radians(11.)),
                 ('pitch', math.radians(11.)), ('roll', float('nan'))]
        for field, value in cases:
            with self.subTest(field=field, value=value):
                self.time = 100.
                a = self.agent()
                obj, name = (a.state, field[6:]) if field.startswith('state.') else (a, field)
                setattr(obj, name, value)
                self.run_until(a, 4.5)
                self.assertFalse(a._recover_stalled_takeoff())
                a.mode_srv.assert_not_called()
        a = self.agent()
        a._est_recent_speed = lambda: .16
        self.run_until(a, 4.5)
        self.assertFalse(a._recover_stalled_takeoff())

    def test_missing_stale_or_future_sample_breaks_early_evidence(self):
        for field, delta in [('_pose_sample_s', -.31), ('_pose_sample_s', .01),
                             ('_local_prev_t', -.31), ('_local_prev_t', .01),
                             ('_pos_sp_t', -.26), ('_pos_sp_t', .01)]:
            with self.subTest(field=field, delta=delta):
                self.time = 100.
                a = self.agent()
                for i in range(91):
                    self.tick(a, 100. + i * .05)
                    setattr(a, field, self.time + delta)
                    a._check_climb_progress()
                self.assertFalse(a._recover_stalled_takeoff())
                a.mode_srv.assert_not_called()

    def test_pose_or_output_can_expire_between_check_and_recovery(self):
        for field, delta in [('_pose_sample_s', -.31), ('_pos_sp_t', -.26)]:
            self.time = 100.
            a = self.agent()
            self.run_until(a, 4.5)
            setattr(a, field, self.time + delta)
            self.assertFalse(a._recover_stalled_takeoff())

    def test_failsafe_can_arrive_between_check_and_recovery(self):
        a = self.agent()
        self.run_until(a, 4.5)
        a.state.system_status = 5
        self.assertFalse(a._recover_stalled_takeoff())
        a.mode_srv.assert_not_called()

    def test_invalid_auto_height_preserves_parameter_safety_gate(self):
        for success, height in [(False, 2.5), (True, 1.), (True, 5.),
                                (True, float('nan'))]:
            with self.subTest(success=success, height=height):
                self.time = 100.
                a = self.agent()
                self.run_until(a, 4.5)
                self.param.return_value = NS(success=success, value=NS(real=height))
                self.assertFalse(a._recover_stalled_takeoff())
                self.assertTrue(a._takeoff_recovery_attempted)
                a.mode_srv.assert_not_called()

    def test_callback_during_parameter_read_cannot_hide_failsafe(self):
        a = self.agent()
        self.run_until(a, 4.5)
        def param_callback(*args):
            a.state.system_status = 5
            return NS(success=True, value=NS(real=2.5))
        self.param.side_effect = param_callback
        self.assertTrue(a._recover_stalled_takeoff())
        self.assertTrue(a._takeoff_recovery['failed'])
        a.mode_srv.assert_not_called()

    def test_valid_pose_callback_during_parameter_read_keeps_fixed_deadline(self):
        a = self.agent()
        self.run_until(a, 4.5)
        def param_callback(*args):
            self.time += .004
            a._local_prev_t = sa.rospy.Time.from_sec(self.time).to_sec()
            return NS(success=True, value=NS(real=2.5))
        self.param.side_effect = param_callback
        self.assertTrue(a._recover_stalled_takeoff())
        self.assertFalse(a._takeoff_recovery['failed'])
        self.assertEqual(a._takeoff_recovery['started'], 104.5)
        a.mode_srv.assert_called_once_with(0, 'AUTO.TAKEOFF')

    def test_clock_rewind_cannot_reuse_prior_early_budget(self):
        a = self.agent()
        self.run_until(a, 4.)
        self.tick(a, 99.)
        self.run_until(a, 5.)
        self.assertFalse(a._recover_stalled_takeoff())
        a.mode_srv.assert_not_called()

    def test_default_off_and_other_output_modes_keep_legacy_ten_seconds(self):
        for p in [patch.dict(os.environ, {'TAKEOFF_PROGRESS_RECOVERY_EARLY': '0'}),
                  patch.object(sa, 'MOTION_SAFETY', False),
                  patch.object(sa, 'FLIGHT_OUTPUT', 'vel')]:
            with p:
                self.time = 100.
                a = self.agent()
                self.run_until(a, 4.5)
                self.assertFalse(a._recover_stalled_takeoff())
                for i in range(1, 111):
                    self.tick(a, 104.5 + i * .05)
                self.assertTrue(a._recover_stalled_takeoff())
                a.mode_srv.assert_called_once_with(0, 'AUTO.TAKEOFF')


if __name__ == '__main__':
    unittest.main()
