"""Replay an armed, grounded OFFBOARD climb stall without a simulator."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa


class StalledTakeoffRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.time=100.
        clock=patch.object(sa.rospy.Time,'now',side_effect=lambda:sa.rospy.Time.from_sec(self.time))
        clock.start();self.addCleanup(clock.stop)
        self.param=Mock(return_value=NS(success=True,value=NS(real=2.5)))
        proxy=patch.object(sa.rospy,'ServiceProxy',return_value=self.param)
        proxy.start();self.addCleanup(proxy.stop)

    def agent(self):
        a=sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id='typhoon_h480_0'
        a.state=NS(connected=True,armed=True,mode='OFFBOARD')
        a.local_z=-.1;a.local_xy=(.01,.02)
        a.roll=a.pitch=0.
        a._anchor_done=True;a._local_prev_t=self.time
        a._climb_progress=(89.,-.1)
        a._est_recent_speed=lambda:.02
        a._send_vel=Mock()
        a.mode_srv=Mock(return_value=NS(mode_sent=True))
        a.arm_srv=Mock()
        return a

    def tick(self,a,t,mode=None,z=None):
        self.time=t;a._local_prev_t=sa.rospy.Time.from_sec(t).to_sec()
        if mode is not None:a.state.mode=mode
        if z is not None:a.local_z=z
        return a._recover_stalled_takeoff()

    def test_logged_grounded_deadlock_requests_standard_takeoff_once_per_second(self):
        a=self.agent()
        self.assertTrue(a._recover_stalled_takeoff())
        self.param.assert_called_once_with('MIS_TAKEOFF_ALT')
        a.mode_srv.assert_called_once_with(0,'AUTO.TAKEOFF')
        self.assertTrue(self.tick(a,100.05))
        self.assertEqual(a.mode_srv.call_count,1)
        a.arm_srv.assert_not_called()
        a._send_vel.assert_called_with(0.,0.,vz=0.)

    def test_mode_acceptance_is_not_taken_as_success_and_normal_handback_needs_stream(self):
        a=self.agent();a._recover_stalled_takeoff()
        self.assertTrue(self.tick(a,100.05,'OFFBOARD',-.1))
        # Observed AUTO.TAKEOFF plus a continuously streamed ascent is needed.
        for i in range(1,46):self.tick(a,100.05+i*.05,'AUTO.TAKEOFF',min(2.,i*.05))
        a.mode_srv.assert_called_with(0,'OFFBOARD')
        self.assertTrue(self.tick(a,102.35,'AUTO.TAKEOFF',2.1))
        self.assertFalse(self.tick(a,102.40,'OFFBOARD',2.1))
        self.assertIsNone(a._takeoff_recovery)
        a.arm_srv.assert_not_called()

    def test_does_not_take_over_landing_or_failsafe_during_recovery(self):
        for mode in ('AUTO.LAND','AUTO.RTL','MANUAL'):
            a=self.agent();self.time=100.;a._local_prev_t=100.
            a._recover_stalled_takeoff();a.mode_srv.reset_mock();a._send_vel.reset_mock()
            self.assertTrue(self.tick(a,100.05,mode))
            self.assertTrue(a._takeoff_recovery['failed'])
            a.mode_srv.assert_not_called();a._send_vel.assert_not_called()

    def test_recovery_cannot_start_after_climb_or_when_pose_is_unsafe(self):
        for field,value in (('_takeoff_done',True),('_takeoff_max_z',.7),
                ('_localization_fault',True),('_ekf_frozen',True),
                ('local_xy',(1.,0.)),('local_xy',(float('nan'),0.)),
                ('local_z',.5),('local_z',float('nan')),
                ('roll',math.radians(15.)),('roll',float('nan')),('_local_prev_t',98.)):
            a=self.agent();setattr(a,field,value)
            self.assertFalse(a._recover_stalled_takeoff(),(field,value))
            a.mode_srv.assert_not_called();a.arm_srv.assert_not_called()
        a=self.agent();a._est_recent_speed=lambda:.4
        self.assertFalse(a._recover_stalled_takeoff())
        a.mode_srv.assert_not_called()
        a=self.agent();a._est_recent_speed=lambda:float('nan')
        self.assertFalse(a._recover_stalled_takeoff())
        a.mode_srv.assert_not_called()

    def test_unverified_or_excessive_auto_height_cannot_trigger_recovery(self):
        for success,height in ((False,2.5),(True,5.),(True,1.),(True,float('nan'))):
            a=self.agent();self.param.return_value=NS(success=success,value=NS(real=height))
            self.assertFalse(a._recover_stalled_takeoff())
            a.mode_srv.assert_not_called();a.arm_srv.assert_not_called()
            self.assertTrue(a._takeoff_recovery_attempted)

    def test_timeout_while_waiting_for_mode_cannot_retry_forever(self):
        a=self.agent();a._recover_stalled_takeoff();a.mode_srv.reset_mock()
        self.assertTrue(self.tick(a,116.,'OFFBOARD',-.1))
        self.assertTrue(a._takeoff_recovery['failed'])
        self.assertTrue(self.tick(a,117.))
        a.mode_srv.assert_not_called()
        a._send_vel.assert_called_with(0.,0.,vz=0.)

    def test_timeout_during_auto_ascent_requests_landing_instead_of_more_ascent(self):
        a=self.agent();a._recover_stalled_takeoff();a.mode_srv.reset_mock()
        self.assertTrue(self.tick(a,116.,'AUTO.TAKEOFF',.4))
        a.mode_srv.assert_called_once_with(0,'AUTO.LAND')

    def test_output_gap_delays_offboard_handback(self):
        a=self.agent();a._recover_stalled_takeoff();a.mode_srv.reset_mock()
        self.tick(a,103.,'AUTO.TAKEOFF',2.1)
        a.mode_srv.assert_not_called()
        for i in range(1,43):self.tick(a,103.+i*.05,'AUTO.TAKEOFF',2.1)
        a.mode_srv.assert_called_with(0,'OFFBOARD')


if __name__=='__main__':unittest.main()
