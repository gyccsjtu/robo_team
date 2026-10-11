"""022050: a still-visible white ID was cancelled before bridge revalidation."""
import json
import unittest
from types import SimpleNamespace as NS
from test_discovery_dispatch import DiscoveryDispatchTests
import swarm_manager as sm


class PendingOwnerTests(unittest.TestCase):
    payload = DiscoveryDispatchTests.payload

    def setUp(self):
        DiscoveryDispatchTests.setUp(self)
        self.now = 2328.568
        m = self.m
        m._tracking = {'t3': 'u0'}
        m.status['u0'].x, m.status['u0'].y = 128.7, 33.8
        m.last_report['u0'] = sm.rospy.Time.from_sec(self.now)
        old = self.payload(target_id='t3', sample_s=2320.480, x=124.53, y=27.79,
                           candidate_id='session:19')
        m._visual_observations[('u0', 't3')] = dict(old, eligible=True)
        m._visual_source['t3'] = ('u0',2320.480,124.53,27.79)
        m._visual_near_seen['t3'] = (124.53,27.79,2320.480)
        m._last_detect['t3'] = 2320.480
        self.frame = dict(schema_version=2, purpose='discovery_only',
            source_uav='u0', sample_s=2328.048, real_detection=True, candidates=[
            self.payload(target_id='t3', sample_s=2328.048, candidate_id='session:21',
                         x=124.69,y=28.83,confidence=.815,hits=15,range_m=7.51,
                         appearance_status='actor',box=[387.4,348.365,420.,443.665])])

    def receive(self):
        self.m._discovery_cb(NS(data=json.dumps(self.frame)))

    def test_recorded_frame_keeps_owner_without_merging_identity_or_credit(self):
        self.receive()
        m = self.m
        self.assertFalse(m._hold_near_visual_target('t3',self.now))
        self.assertTrue(m._hold_pending_owner_instance('t3',self.now))
        self.assertEqual(m._visual_source['t3'][1],2320.480)
        self.assertEqual(m._tracking,{'t3':'u0'})
        m.tracker.report.assert_not_called()
        self.assertEqual(m._eligible_observations,{})

    def test_duplicate_frames_and_changed_ids_do_not_renew_deadline(self):
        self.receive()
        self.assertTrue(self.m._hold_pending_owner_instance('t3',self.now))
        until=self.m._pending_owner_instances['t3']['until']
        for dt in (1.,3.,6.):
            self.now=2328.568+dt
            self.m.last_report['u0']=sm.rospy.Time.from_sec(self.now)
            v=next(iter(self.m._discovery_pool.values()))
            v['sample_s']=self.now-.1
            v['candidate_id']='rebuilt:'+str(dt)
            active=self.m._hold_pending_owner_instance('t3',self.now)
            self.assertEqual(active,dt<6.)
            self.assertEqual(self.m._pending_owner_instances['t3']['until'],until)

    def test_no_hold_for_other_camera_ambiguous_instance_or_stale_frame(self):
        for change in ({'source_uav':'u1'}, {'appearance_status':'ambiguous'},
                       {'x':140.}, {'confidence':.5}, {'sample_s':2327.0}):
            self.m._discovery_pool={}
            row=dict(self.frame['candidates'][0],complete_frame=True,**change)
            self.m._discovery_pool[('t3',row['source_uav'],row['candidate_id'])]=row
            self.assertFalse(self.m._hold_pending_owner_instance('t3',self.now))

    def test_two_same_color_candidates_cannot_be_treated_as_id_repair(self):
        self.receive()
        first=next(iter(self.m._discovery_pool.values()))
        self.m._discovery_pool[('t3','u0','other')]=dict(first,candidate_id='other')
        self.assertFalse(self.m._hold_pending_owner_instance('t3',self.now))


if __name__=='__main__':unittest.main()
