"""Green was kept near UAV2 but ordinary following triggered forced motion."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa


class TrackingBreakoutTests(unittest.TestCase):
    def agent(self, task=1):
        a=sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id='u2'; a.assignment=NS(task_type=task)
        return a

    def replay(self,a,guard_changed):
        for i in range(30):
            v=(.45 if i%2 else -.45,0.)
            with patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time.from_sec(100.+i*.3)):
                out=a._oscillation_breakout(*v,guard_changed=guard_changed)
            self.assertEqual(out,v)

    def test_following_reversals_never_trigger_forced_motion(self):
        self.replay(self.agent(),True)

    def test_search_turns_without_actual_guard_intervention_never_trigger(self):
        self.replay(self.agent(0),False)

    def test_new_tracking_or_camera_attention_cancels_old_breakout(self):
        for tracking in (True,False):
            a=self.agent(1 if tracking else 0)
            a._osc_break_until=200.;a._osc_break_hd=0.;a._osc_break_sp=1.2
            if not tracking:a._camera_cue={'until':110.}
            self.replay(a,True)
            self.assertEqual(a._osc_break_until,0.)

    def test_guard_stop_is_not_replaced_by_old_motion(self):
        a=self.agent(0);a._osc_break_until=200.;a._osc_break_hd=0.
        with patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time(100)):
            self.assertEqual(a._oscillation_breakout(0.,0.,True),(0.,0.))

    def test_search_deadlock_recovery_cannot_increase_guarded_speed(self):
        a=self.agent(0)
        for i in range(40):
            with patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time.from_sec(100.+i*.3)):
                out=a._oscillation_breakout(.3 if i%2 else -.3,0.,True)
                self.assertLessEqual(math.hypot(*out),.3+1e-9)
        self.assertGreater(getattr(a,'_osc_break_until',0.),0.)


if __name__=='__main__':unittest.main()
