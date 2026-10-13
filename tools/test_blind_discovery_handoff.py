"""Recorded white reacquisition and task continuity constraints."""
import json, unittest, os
from types import SimpleNamespace as NS
from unittest.mock import patch
import test_discovery_dispatch as fixture
from search_segments import ATTEND
sm=fixture.sm

class BlindDiscoveryHandoffTests(unittest.TestCase):
    setUp=fixture.DiscoveryDispatchTests.setUp

    def prepare(self):
        self.m.status['u0'].x,self.m.status['u0'].y=69.,.5
        self.m.status['u1'].x,self.m.status['u1'].y=5.,0.
        self.m.status['u2'].x,self.m.status['u2'].y=15.,0.
        self.m._tracking_since={}

    def observe(self,stamp,x,y,raw_confidence,**changes):
        self.now=stamp+.1
        for uid in self.m.uav_ids:
            self.m.last_report[uid]=sm.rospy.Time.from_sec(self.now)
            self.m.status[uid].header=NS(stamp=sm.rospy.Time.from_sec(self.now))
        self.m.status['u0'].search_state=ATTEND
        raw=dict(purpose='discovery_only',geometry_ok=True,official_view_ok=False,
                 real_detection=True,source_uav='u0',target_id='t3',sample_s=stamp,
                 candidate_id='recorded-white:2',hits=4,range_m=8.39,
                 confidence=raw_confidence,x=x,y=y,appearance_status='actor')
        raw.update(changes)
        self.m._discovery_cb(NS(data=json.dumps(raw)))
        tx,ty=self.m._get_target_pos('t3',self.now)
        self.m._dispatch_target_batch([('t3',tx,ty)],self.now)
        self.m._handoff_distant_tracker('t3',tx,ty,self.m.tracker.targets['t3'])

    def first_two(self):
        self.prepare()
        # Actual own reports from 402.868s white reacquisition.
        self.observe(2043.544,76.43,.69,.700)
        self.observe(2044.144,76.82,.43,.912)
        self.assertEqual(self.m._tracking['t3'],'u2')

    def third(self,**changes):
        self.observe(2044.728,77.5,.43,.721,**changes)

    def test_recorded_third_image_transfers_blind_task_without_report_credit(self):
        with patch.dict(os.environ,DISCOVERY_BLIND_HANDOFF='1'):
            self.first_two();self.third()
        self.assertEqual(self.m._tracking['t3'],'u0')
        self.assertEqual(self.m._eligible_observations,{})
        self.m.tracker.report.assert_not_called()
        messages=[(c.args[0].uav_id,c.args[0].target_id,c.args[0].task_type)
                  for c in self.m.assign_pub.publish.call_args_list]
        self.assertEqual(messages,[('u2','t3',1),('u0','t3',1),('u2','t3',255)])

    def test_disabled_feature_preserves_previous_owner(self):
        with patch.dict(os.environ,DISCOVERY_BLIND_HANDOFF='0'):
            self.first_two();self.third()
        self.assertEqual(self.m._tracking['t3'],'u2')

    def test_recent_primary_image_and_team_qualified_image_protect_owner(self):
        for field in ('_visual_observations','_eligible_observations'):
            with self.subTest(field=field),patch.dict(os.environ,DISCOVERY_BLIND_HANDOFF='1'):
                self.m._tracking={};self.m._discovery_pool={}
                self.first_two()
                setattr(self.m,field,{('u2','t3'):dict(sample_s=2044.6,eligible=True,x=77.5,y=.43,hits=1)})
                self.third();self.assertEqual(self.m._tracking['t3'],'u2')
                setattr(self.m,field,{})

    def test_new_instance_and_ambiguous_image_cannot_take_task(self):
        for change in ({'candidate_id':'other'}, {'appearance_status':'ambiguous'},
                       {'confidence':.54}, {'sample_s':2044.144}):
            with self.subTest(change=change),patch.dict(os.environ,DISCOVERY_BLIND_HANDOFF='1'):
                self.m._tracking={};self.m._discovery_pool={};self.first_two()
                self.third(**change);self.assertEqual(self.m._tracking['t3'],'u2')

    def test_other_live_target_is_not_taken(self):
        with patch.dict(os.environ,DISCOVERY_BLIND_HANDOFF='1'):
            self.first_two();self.m._tracking['t0']='u0'
            self.third()
            self.assertEqual(self.m._tracking['t0'],'u0')
            self.assertEqual(self.m._tracking['t3'],'u2')

    def test_far_and_grounded_camera_cannot_take_task(self):
        for position,height in ((0.,3.),(69.,.5)):
            with self.subTest(position=position,height=height),patch.dict(os.environ,DISCOVERY_BLIND_HANDOFF='1'):
                self.m._tracking={};self.m._discovery_pool={};self.first_two()
                self.m.status['u0'].x=position;self.m.status['u0'].z=height
                self.third();self.assertEqual(self.m._tracking['t3'],'u2')

    def test_two_same_color_instances_are_not_resolved_by_distance_alone(self):
        with patch.dict(os.environ,DISCOVERY_BLIND_HANDOFF='1'):
            self.first_two()
            self.m._discovery_pool[('t3','u0','second')]=dict(target_id='t3',source_uav='u0',candidate_id='second',appearance_status='actor',sample_s=2044.728,dispatch_since_s=2043.544,dispatch_hits=3,x=77.7,y=.6)
            self.third();self.assertEqual(self.m._tracking['t3'],'u2')

if __name__=='__main__':unittest.main()
