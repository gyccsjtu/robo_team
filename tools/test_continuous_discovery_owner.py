"""A continuous discovery camera can own a task without judge-report credit."""
import json
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
import test_discovery_dispatch as fixture
from search_segments import ATTEND
import swarm_manager as sm


class ContinuousDiscoveryOwnerTests(unittest.TestCase):
    setUp = fixture.DiscoveryDispatchTests.setUp
    payload = fixture.DiscoveryDispatchTests.payload

    def observe(self, stamp, **changes):
        self.now=stamp
        for uid in self.m.uav_ids:
            self.m.last_report[uid]=sm.rospy.Time.from_sec(stamp)
            self.m.status[uid].header=NS(stamp=sm.rospy.Time.from_sec(stamp))
        self.m.status['u0'].search_state=ATTEND
        self.m._discovery_cb(NS(data=json.dumps(self.payload(**changes))))

    def test_three_fresh_same_instance_images_allow_current_camera_dispatch(self):
        self.observe(100.)
        self.assertTrue(self.m._blind_dispatch_interrupts_attention('u0','t1',100.))
        self.observe(100.2)
        self.assertTrue(self.m._blind_dispatch_interrupts_attention('u0','t1',100.2))
        self.observe(100.4)
        self.assertFalse(self.m._blind_dispatch_interrupts_attention('u0','t1',100.4))
        self.m._dispatch_target_batch([('t1',8.,0.)],100.4)
        self.assertEqual(self.m._tracking,{'t1':'u0'})
        self.assertEqual(self.m._eligible_observations,{})
        self.m.tracker.report.assert_not_called()

    def test_duplicate_old_gapped_or_other_instance_frames_never_build_sequence(self):
        for changes in ({'sample_s':100.}, {'candidate_id':'other'}, {'x':30.}):
            self.m._discovery_pool={}
            self.observe(100.)
            self.observe(100.2,**changes)
            self.observe(100.4,**changes)
            self.assertIsNone(self.m._continuous_discovery_view('u0','t1',100.4))
        self.m._discovery_pool={}
        self.observe(100.)
        self.observe(100.8)
        self.observe(101.)
        self.assertIsNone(self.m._continuous_discovery_view('u0','t1',101.))

    def test_conflicting_target_ambiguous_appearance_and_stale_view_do_not_preempt(self):
        for stamp in (100.,100.2,100.4):self.observe(stamp)
        self.assertTrue(self.m._blind_dispatch_interrupts_attention('u0','t0',100.4))
        self.assertIsNone(self.m._continuous_discovery_view('u0','t1',100.76))
        self.assertIsNone(self.m._continuous_discovery_view('u0','t1',100.4,30.,0.))
        for stamp in (100.6,100.8,101.):self.observe(stamp,appearance_status='ambiguous')
        self.assertIsNone(self.m._continuous_discovery_view('u0','t1',101.))

    def test_current_human_owner_is_not_stolen_for_a_new_discovery(self):
        for stamp in (100.,100.2,100.4):self.observe(stamp)
        self.m._tracking={'t0':'u0'}
        self.m._tracking_since={'t0':100.}
        self.m._dispatch_target_batch([('t1',8.,0.)],100.4)
        self.assertEqual(self.m._tracking['t0'],'u0')
        self.assertNotEqual(self.m._tracking.get('t1'),'u0')


if __name__=='__main__':unittest.main()
