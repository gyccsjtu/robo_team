"""Early white contact: do not send its rescue into the same friend blockage."""
import os
import unittest
from unittest.mock import patch
import test_tracking_motion_relief as fixtures
from search_segments import SegmentBook
from boundary_patrol import SouthPatrol
import swarm_manager as sm


class ReliefRouteTests(unittest.TestCase):
    def manager(self):
        m=fixtures.TrackingMotionTests().manager()
        # Own-position layout representative of the captured early-white cluster.
        for u,(x,y) in {'u0':(3.,-.12),'u1':(6.4,-1.5),'u2':(0.,-.1),'u3':(0.,-3.6)}.items():
            m.status[u].x,m.status[u].y=x,y
        m._get_target_pos=lambda *a:(10.5,-8.3)
        m._eligible_observations[('u2','t3')]=dict(eligible=True,sample_s=101.,hits=4,since_s=100.)
        m._segments=SegmentBook()
        m._regional_patrols=[]
        for u,half,key in [('u1','north',(22,9)),('u3','south',(22,8))]:
            p=SouthPatrol((-50.,130.,-60.,60.),east=True,east_half=half)
            t=m._segments.reserve(u,[key],[(98.55,6.5 if half=='north' else -.5)],100.,10.,90.,purpose='south_patrol')
            p.uid,p.task_id=u,t.task_id
            m._regional_patrols.append(p)
        m.__dict__.pop('_patrol_accepts_target')
        for i in range(6):
            t=100.+.2*i
            m._tracking_motion.offer(fixtures.TrackingMotionTests().packet(t),t+.05,'u0',100.)
        return m

    def run_dispatch(self,m,enabled):
        with patch.dict(os.environ,{'BLOCKED_TRACKING_RELIEF':'1','RELIEF_CLEAR_APPROACH':enabled}),patch.object(sm.rospy.Time,'now',return_value=sm.rospy.Time(101.1)):
            m._dispatch_backup()

    def test_old_choice_sends_the_seeing_camera_into_friend_blockage(self):
        m=self.manager();self.run_dispatch(m,'0')
        self.assertEqual(m._backup,{'t3':'u2'})
        self.assertGreater(m._backup_approach_conflicts('u2',10.5,-8.3,101.1),0)

    def test_near_patrol_can_rescue_without_borrowing_both_patrols(self):
        m=self.manager();self.run_dispatch(m,'1')
        self.assertEqual(m._tracking,{'t3':'u0'})
        self.assertEqual(m._backup,{'t3':'u1'})
        self.assertFalse(m._patrol_motion_loan('u3',10.5,-8.3,101.1))
        self.assertEqual(m.assign_pub.publish.call_count,1)
        self.assertEqual(m.assign_pub.publish.call_args.args[0].task_type,1)

    def test_sole_expired_disconnected_or_unbound_patrol_cannot_be_borrowed(self):
        for reason in ('missing','expired','offline','wrong_task'):
            m=self.manager()
            if reason=='missing':m._segments.tasks.pop('u3')
            elif reason=='expired':m._segments.tasks['u3'].expires=100.5
            elif reason=='offline':m.status['u3'].connected=False
            else:m._regional_patrols[1].task_id='old'
            self.assertFalse(m._patrol_motion_loan('u1',10.5,-8.3,101.1),reason)

    def test_no_far_or_unqualified_rescue_and_no_primary_theft(self):
        m=self.manager()
        self.assertFalse(m._patrol_motion_loan('u1',30.,-8.3,101.1))
        m._eligible_observations.clear();self.run_dispatch(m,'1')
        self.assertFalse(m._backup)
        m.assign_pub.publish.assert_not_called()

    def test_conflicts_use_current_three_dimensional_peers_only(self):
        m=self.manager()
        before=m._backup_approach_conflicts('u2',10.5,-8.3,101.1)
        m.status['u0'].z=10.
        after=m._backup_approach_conflicts('u2',10.5,-8.3,101.1)
        self.assertLess(after,before)
        m.last_report['u0']=sm.rospy.Time(90.)
        self.assertEqual(m._backup_approach_conflicts('u2',10.5,-8.3,101.1),after)


if __name__=='__main__':unittest.main()
