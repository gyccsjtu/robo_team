"""A stopped flight intent may request another camera, never override safety."""
import json
import os
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import swarm_agent as sa
import swarm_manager as sm
from tracking_motion import TrackingMotionWindow, motion_feedback


class TrackingMotionTests(unittest.TestCase):
    def packet(self, t, output=0., **change):
        row = dict(uav_id='u0', target_id='t3', sample_s=t, assignment_s=100.,
                   intent_speed=2., output_speed=output)
        row.update(change)
        return row

    def test_continuous_real_stop_is_required(self):
        w = TrackingMotionWindow()
        for i in range(6):
            t=100.+i*.2
            self.assertTrue(w.offer(self.packet(t),t+.05,'u0',100.))
            self.assertEqual(w.blocked('u0','t3',t+.05,100.),i==5)
        self.assertFalse(w.blocked('u0','t3',101.7,100.))
        w.offer(self.packet(101.2,output=1.),101.25,'u0',100.)
        self.assertFalse(w.blocked('u0','t3',101.3,100.))

    def test_repeat_gap_wrong_owner_and_old_task_cannot_request_relief(self):
        w=TrackingMotionWindow()
        self.assertTrue(w.offer(self.packet(100.),100.1,'u0',100.))
        self.assertFalse(w.offer(self.packet(100.),100.2,'u0',100.))
        self.assertFalse(w.offer(self.packet(100.4),100.5,'u1',100.))
        self.assertFalse(w.offer(self.packet(100.4),100.5,'u0',100.2))
        self.assertFalse(w.offer(self.packet(99.),100.1,'u0',100.))
        self.assertTrue(w.offer(self.packet(101.2),101.3,'u0',100.))
        self.assertFalse(w.blocked('u0','t3',101.3,100.))
        self.assertFalse(w.blocked('u0','t3',101.3,101.))

    def test_feedback_is_not_an_observation_and_idle_is_not_blockage(self):
        a=NS(task_type=1,target_id='t3',header=NS(stamp=sm.rospy.Time(100.)))
        row=motion_feedback('u0',a,101.,0.,0.)
        self.assertFalse(row['blocked'])
        self.assertNotIn('x',row)
        self.assertNotIn('real_detection',row)
        self.assertIsNone(motion_feedback('u0',NS(task_type=0),101.,2.,0.))

    def test_clock_reset_starts_a_new_window(self):
        w=TrackingMotionWindow()
        w.offer(self.packet(100.),100.1,'u0',100.)
        row=self.packet(10.,assignment_s=10.)
        self.assertTrue(w.offer(row,10.1,'u0',10.))
        self.assertFalse(w.blocked('u0','t3',10.1,10.))

    def manager(self):
        m=sm.SwarmManager.__new__(sm.SwarmManager)
        m._tracking={'t3':'u0'};m._backup={};m._tracking_since={'t3':100.}
        m.uav_ids=['u0','u1','u2','u3']
        m.status={u:NS(connected=True,x=float(i*3),y=0.,z=3.) for i,u in enumerate(m.uav_ids)}
        m.last_report={u:sm.rospy.Time(101.1) for u in m.uav_ids}
        m._tracking_motion=TrackingMotionWindow()
        m._eligible_observations={('u0','t3'):dict(eligible=True,sample_s=101.,hits=8,since_s=100.)}
        m._visual_observations={};m._release_cooldown={};m._confirm_timeout_bl={};m._eliminated=set()
        m.tracker=NS(needs_backup=Mock(return_value=[]),targets={'t3':NS(eliminated=False,observers={'u0'})})
        m._get_target_pos=lambda *a:(8.,0.)
        m._camera_view_needs_relief=lambda *a:False
        m._patrol_accepts_target=lambda *a:True
        m._sole_first_pass_zone_searcher=lambda *a:False
        m._free_search_lease=Mock();m.assign_pub=Mock()
        m._task_lock=__import__('threading').RLock()
        return m

    def test_blocked_owner_keeps_task_while_one_idle_backup_is_reserved(self):
        m=self.manager()
        with patch.dict(os.environ,{'BLOCKED_TRACKING_RELIEF':'1'}), patch.object(sm.rospy.Time,'now',return_value=sm.rospy.Time(101.1)):
            for i in range(6):
                t=100.+i*.2
                m._tracking_motion.offer(self.packet(t),t+.05,'u0',100.)
            m._dispatch_backup()
            self.assertEqual(m._tracking,{'t3':'u0'})
            self.assertEqual(m._backup,{'t3':'u3'})
            self.assertEqual(m.assign_pub.publish.call_count,1)
            self.assertEqual(m.assign_pub.publish.call_args.args[0].task_type,1)
            m._dispatch_backup()
            self.assertEqual(m.assign_pub.publish.call_count,1)
            m._eligible_observations.clear()
            self.assertFalse(m._motion_needs_relief('t3',101.1))

    def test_orca_stop_preserves_current_tracking_intention(self):
        from test_orca_heading import OrcaHeadingTests
        a=OrcaHeadingTests()._agent((3.3,0.))
        a.assignment=NS(task_type=1,target_id='t3')
        a._friend_positions['u2']=(-3.3,0.,3.)
        a._friend_seen['u2']=100.
        with patch.dict(os.environ,{'BLOCKED_TRACKING_RELIEF':'1'}), patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time(100.)), patch.object(sa.rospy,'logwarn'),patch.object(sa.rospy,'loginfo_throttle'):
            self.assertEqual(a._apply_friend_avoidance(2.,0.),(0.,0.))
            self.assertEqual(a._friend_motion_intent,(100.,'t3',2.))

    def test_disabled_or_expired_visual_does_not_borrow_a_searcher(self):
        for enabled, sample in (('0',101.),('1',100.)):
            m=self.manager()
            m._eligible_observations[('u0','t3')]['sample_s']=sample
            for i in range(6):
                t=100.+i*.2
                m._tracking_motion.offer(self.packet(t),t+.05,'u0',100.)
            with patch.dict(os.environ,{'BLOCKED_TRACKING_RELIEF':enabled}), patch.object(sm.rospy.Time,'now',return_value=sm.rospy.Time(101.1)):
                m._dispatch_backup()
                self.assertFalse(m._backup)
                m.assign_pub.publish.assert_not_called()


if __name__ == '__main__': unittest.main()
