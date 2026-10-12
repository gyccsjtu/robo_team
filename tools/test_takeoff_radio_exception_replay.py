"""FCU setup and recovery replay for the 2026-10-13 02:52 UAV3 stall.

Recorded evidence (ROS = ULog timestamp / 1e6 + 1925.620):
  logs/logs_20261013_025212/06_swarm_agent_3.log
  ~/.ros/sitl_typhoon_h480_3/log/2026-10-12/18_52_30.ulg

PX4 source explaining why COM_RCL_EXCEPT=4 failed:
  src/modules/commander/commander_params.c:889: bit 1 Hold, bit 2 Offboard.
  src/modules/commander/state_machine_helper.cpp:575-582: AUTO_TAKEOFF
  checks RCL_EXCEPT_HOLD before the no-RC-and-no-datalink failsafe.
  The DISABLED link-loss action at :731 leaves the old nav_state unchanged.

The fixture is an observed FCU state sequence, not a simulated successful
flight. A sent mode request must still time out if this sequence repeats.
"""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa


RECORDED_STALL = (
    # ROS time, commander main_state, actual nav_state, failsafe, altitude.
    (1958.816, 7, 14, False, -.098),
    (1968.812, 10, 14, True, -.130),
    (1969.808, 10, 14, True, -.130),
    (1983.812, 10, 14, True, -.144),
)
RECORDED_RADIO_FLAGS = dict(rc_signal_lost=True, data_link_lost=True,
                           rc_signal_found_once=False, landed=False,
                           local_position_valid=True, global_position_valid=True,
                           home_position_valid=True)


class TakeoffRadioExceptionReplayTests(unittest.TestCase):
    def setUp(self):
        # The first request's ROS stamp is 1968.796; using the watchdog's
        # rounded 1968.792 with literal float subtraction can be < 10 s.
        self.time = 1968.796
        self.params = {'NAV_RCL_ACT': 0, 'COM_RCL_EXCEPT': 4,
                       'MIS_TAKEOFF_ALT': 2.5,
                       'COM_OF_LOSS_T': 1., 'COM_OBL_ACT': 0,
                       'COM_OBL_RC_ACT': 4}
        self.initial_params = self.params.copy()
        self.wrong_readback = False
        self.pull = Mock(return_value=NS(success=True, param_received=600))
        self.get = Mock(side_effect=self.read_parameter)
        patches = (
            patch.object(sa.rospy.Time, 'now', side_effect=lambda:
                         sa.rospy.Time.from_sec(self.time)),
            patch.object(sa.rospy, 'wait_for_service'),
            patch.object(sa.rospy, 'ServiceProxy', side_effect=self.service_proxy),
        )
        for item in patches:
            item.start()
            self.addCleanup(item.stop)

    def service_proxy(self, name, _kind):
        if name.endswith('/param/pull'):
            return self.pull
        if name.endswith('/param/get'):
            return self.get
        raise AssertionError('unexpected service: ' + name)

    def read_parameter(self, name):
        value = self.params[name]
        if name == 'COM_RCL_EXCEPT' and self.wrong_readback:
            value = 4
        return NS(success=True, value=NS(integer=int(value), real=float(value)))

    def agent(self):
        a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id = 'typhoon_h480_3'
        a.state = NS(connected=True, armed=True, mode='OFFBOARD')
        a.local_z = -.13
        a.local_xy = (.01, .02)
        a.roll = a.pitch = 0.
        a._anchor_done = True
        a._local_prev_t = self.time
        a._climb_progress = (1958.792, -.13)
        a._est_recent_speed = lambda: .02
        a._send_vel = Mock()
        a.mode_srv = Mock(return_value=NS(mode_sent=True))
        a.arm_srv = Mock()
        def write_parameter(name, value):
            self.params[name] = value
            return True
        a._set_param = Mock(side_effect=write_parameter)
        return a

    def tick(self, a, stamp, mode='OFFBOARD', z=-.13):
        self.time = stamp
        a._local_prev_t = sa.rospy.Time.from_sec(stamp).to_sec()
        a.state.mode = mode
        a.local_z = z
        return a._recover_stalled_takeoff()

    def test_configure_verifies_hold_and_offboard_without_other_loss_changes(self):
        a = self.agent()
        a._configure_fcu()
        self.assertEqual(self.params['COM_RCL_EXCEPT'], 6)
        self.assertTrue(self.params['COM_RCL_EXCEPT'] & (1 << 1))
        self.assertTrue(self.params['COM_RCL_EXCEPT'] & (1 << 2))
        self.assertFalse(self.params['COM_RCL_EXCEPT'] & (1 << 0))
        self.get.assert_any_call('COM_RCL_EXCEPT')
        self.assertEqual(a._set_param.call_args_list,
                         [unittest.mock.call('NAV_RCL_ACT', 0),
                          unittest.mock.call('COM_RCL_EXCEPT', 6)])
        for name in ('COM_OF_LOSS_T', 'COM_OBL_ACT', 'COM_OBL_RC_ACT',
                     'MIS_TAKEOFF_ALT'):
            self.assertEqual(self.params[name], self.initial_params[name])

    def test_wrong_radio_readback_fails_startup(self):
        a = self.agent()
        self.wrong_readback = True
        with self.assertRaisesRegex(RuntimeError, 'FCU_CONFIGURATION_NOT_VERIFIED'):
            a._configure_fcu()
        self.assertEqual(self.pull.call_count, 3)
        self.assertEqual(self.get.call_args_list.count(
            unittest.mock.call('COM_RCL_EXCEPT')), 3)
        a.mode_srv.assert_not_called()
        a.arm_srv.assert_not_called()

    def test_actual_logged_states_distinguish_requested_and_executed_mode(self):
        self.assertTrue(RECORDED_RADIO_FLAGS['local_position_valid'])
        self.assertTrue(RECORDED_RADIO_FLAGS['rc_signal_lost'])
        self.assertTrue(RECORDED_RADIO_FLAGS['data_link_lost'])
        self.assertFalse(RECORDED_RADIO_FLAGS['landed'])
        self.assertFalse(4 & (1 << 1))  # PX4 AUTO_TAKEOFF's Hold exception.
        self.assertEqual(RECORDED_STALL[0][1:4], (7, 14, False))
        for _stamp, requested, executed, failsafe, _z in RECORDED_STALL[1:]:
            self.assertEqual((requested, executed, failsafe), (10, 14, True))
        a = self.agent()
        a._configure_fcu()
        self.assertTrue(a._recover_stalled_takeoff())
        for stamp, _main, _nav, _failsafe, z in RECORDED_STALL[1:]:
            self.assertTrue(self.tick(a, stamp, 'OFFBOARD', z))
            self.assertFalse(a._takeoff_recovery['auto_seen'])
        self.assertTrue(self.tick(a, 1983.840, 'OFFBOARD', -.144))
        self.assertTrue(a._takeoff_recovery['failed'])
        a.mode_srv.reset_mock()
        self.assertTrue(self.tick(a, 1984.000))
        a.mode_srv.assert_not_called()
        a._send_vel.assert_called_with(0., 0., vz=0.)
        a.arm_srv.assert_not_called()

    def test_real_mode_feedback_and_continuous_stream_allow_guarded_handback(self):
        a = self.agent()
        a._configure_fcu()
        self.assertTrue(a._recover_stalled_takeoff())
        a.mode_srv.assert_called_with(0, 'AUTO.TAKEOFF')
        self.assertTrue(self.tick(a, 1968.842))
        self.assertFalse(a._takeoff_recovery['auto_seen'])
        for i in range(2, 47):
            stamp = 1968.792 + i * .05
            z = min(2.1, -.13 + i * .055)
            self.assertTrue(self.tick(a, stamp, 'AUTO.TAKEOFF', z))
        a.mode_srv.assert_called_with(0, 'OFFBOARD')
        self.assertFalse(self.tick(a, 1971.142, 'OFFBOARD', 2.1))
        self.assertIsNone(a._takeoff_recovery)
        a.arm_srv.assert_not_called()
        self.assertEqual(self.params['MIS_TAKEOFF_ALT'], 2.5)

    def test_config_does_not_authorize_recovery_with_bad_pose_or_landing(self):
        for field, value in (('_localization_fault', True), ('_ekf_frozen', True),
                             ('_landing', True), ('_local_prev_t', 1967.),
                             ('roll', math.radians(15.))):
            with self.subTest(field=field):
                a = self.agent()
                a._configure_fcu()
                setattr(a, field, value)
                self.assertFalse(a._recover_stalled_takeoff())
                a.mode_srv.assert_not_called()
                a.arm_srv.assert_not_called()
        for mode in ('AUTO.LAND', 'AUTO.RTL', 'MANUAL'):
            with self.subTest(mode=mode):
                a = self.agent()
                a._configure_fcu()
                a.state.mode = mode
                self.assertFalse(a._recover_stalled_takeoff())
                a.mode_srv.assert_not_called()


if __name__ == '__main__':
    unittest.main()
