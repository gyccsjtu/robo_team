"""Visible moving people must not be held still while normal dispatch matures."""
import copy
import json
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

from tools.test_attention_priority import AttentionPriorityTests, sa
from tools.test_discovery_dispatch import DiscoveryDispatchTests, sm
from camera_attention import localized_follow_decision


class VisibleFollowTests(unittest.TestCase):
    def setUp(self):
        AttentionPriorityTests.setUp(self)
        self.a._target_to_orbit = None
        self.a._friend_positions = {}
        self.a.state = NS(mode='OFFBOARD')
        self.a._hypothesis_switch_s = {}
        self.a._scan = NS(angle_min=-math.pi, angle_increment=math.pi/180.,
                          ranges=[float('inf')]*361, range_min=.1, range_max=30.)
        self.a._scan_t = self.now
        self.mode = patch.dict(sa.os.environ, {'LOCALIZED_SEARCH_FOLLOW':'1'})
        self.mode.start(); self.addCleanup(self.mode.stop)

    def payload(self, x, **changes):
        raw = dict(source_uav='u0', target_id='t2', track_id=7,
                   world_candidate_id='epoch:7', candidate_id='brown:7',
                   real_detection=True, sample_s=self.now, confidence=.9, hits=4,
                   box=[340.,200.,380.,300.], width=752, height=480, fy=376.,
                   camera_tilt=0., geometry_ok=True, report_ok=True,
                   person_verdict_ok=True, person_verdict_reason='',
                   appearance_status='actor', bearing=0., x=x, y=0.)
        raw.update(changes)
        return raw

    def walking(self):
        for i in range(3):
            self.now = 100.+i*.2
            self.a._scan_t = self.now
            self.a._tracking_view_cb(NS(data=json.dumps(self.payload(8.+i*.2))))

    def test_real_callback_and_control_follow_walk_without_target_or_report_credit(self):
        self.walking()
        deadline=self.a._camera_cue['until']
        with patch.dict(sa.os.environ, {'LOCALIZED_SEARCH_FOLLOW':'0'}):
            self.a._control()
            self.assertEqual(self.commands[-1],(0.,0.))
        self.a._control()
        self.assertGreater(self.commands[-1][0], .95)
        self.assertEqual(self.commands[-1][1], 0.)
        self.assertEqual(self.a.assignment.task_type, 0)
        self.assertEqual(self.a._camera_cue['until'], deadline)
        self.assertEqual(self.a._visual_observations, {})
        self.assertEqual(self.a._eligible_observations, {})
        self.assertEqual(self.detections, [])

    def test_existing_tracking_person_cannot_be_replaced_by_new_search_view(self):
        self.a.assignment=NS(task_type=1,target_id='t0')
        self.a._tracking_view_cb(NS(data=json.dumps(self.payload(8.))))
        self.assertIsNone(self.a._camera_cue)
        self.assertEqual(self.a.assignment.target_id,'t0')

    def test_stationary_or_single_jump_cannot_produce_motion_feedforward(self):
        for values in ((8.,8.,8.), (8.,8.2,10.5)):
            self.a._camera_cue=None
            self.a._camera_cue_retry_after={}
            self.a._camera_attention_attempts=[]
            self.a._localized_search_view_samples={}
            for i,x in enumerate(values):
                self.now=100.+i*.2
                self.a._tracking_view_cb(NS(data=json.dumps(self.payload(x))))
            self.a._scan_t=self.now
            self.a._hold_camera_candidate()
            self.assertEqual(self.commands[-1],(0.,0.))

    def test_stale_frame_lidar_obstacle_partial_scan_and_budget_retain_stop(self):
        self.walking();base=copy.deepcopy(self.a._camera_cue)
        scans=[None,NS(angle_min=-math.pi,angle_increment=math.pi/180.,
            ranges=[float('nan')]*361,range_min=.1,range_max=30.),
            NS(angle_min=-math.pi,angle_increment=math.pi/180.,
            ranges=[3.9]*361,range_min=.1,range_max=30.),
            NS(angle_min=0.,angle_increment=math.pi/180.,
            ranges=[float('inf')]*181,range_min=.1,range_max=30.),
            NS(angle_min=float('nan'),angle_increment=math.pi/180.,
            ranges=[float('inf')]*361,range_min=.1,range_max=30.)]
        for scan in scans:
            self.assertEqual(localized_follow_decision(base,self.now,0.,(0.,0.),scan,self.now)[:2],(0.,0.))
        for now,xy,stamp in ((101.,(0.,0.),101.), (104.,(0.,0.),104.),
                             (100.4,(4.,0.),100.4), (100.4,(0.,0.),99.)):
            self.assertEqual(localized_follow_decision(base,now,0.,xy,self.a._scan,stamp)[:2],(0.,0.))

    def test_crop_new_instance_and_rejected_person_cannot_steer(self):
        self.walking();before=copy.deepcopy(self.a._camera_cue)
        self.now+=.2
        for change in (dict(world_candidate_id='other:1'), dict(source_uav='other')):
            self.a._tracking_view_cb(NS(data=json.dumps(self.payload(8.6,**change))))
            self.assertEqual(self.a._camera_cue,before)
        self.a._tracking_view_cb(NS(data=json.dumps(self.payload(8.6,box=[340.,200.,380.,480.]))))
        self.a._scan_t=self.now
        self.a._hold_camera_candidate()
        self.assertEqual(self.commands[-1],(0.,0.))

    def test_new_invalid_own_frame_immediately_withdraws_follow_permission(self):
        for change in (dict(person_verdict_ok=False), dict(report_ok=False),
                       dict(geometry_ok=False), dict(appearance_status='decoy')):
            self.a._camera_cue=None
            self.a._camera_cue_retry_after={}
            self.a._camera_attention_attempts=[]
            self.a._localized_search_view_samples={}
            self.walking()
            self.now+=.1
            self.a._tracking_view_cb(NS(data=json.dumps(self.payload(8.5,**change))))
            self.a._scan_t=self.now
            self.a._hold_camera_candidate()
            self.assertEqual(self.commands[-1],(0.,0.))


class ImmediateDispatchTests(unittest.TestCase):
    setUp=DiscoveryDispatchTests.setUp
    payload=DiscoveryDispatchTests.payload

    def test_busy_discovery_camera_keeps_own_person_and_calls_an_available_peer(self):
        self.m._tracking={'t0':'u0'}
        self.m._confirm_timeout_bl={}
        with patch.dict(sm.os.environ,{'IMMEDIATE_VISIBLE_DISPATCH':'1'}):
            self.m._discovery_cb(NS(data=json.dumps(self.payload(appearance_status='actor'))))
        self.assertEqual(self.m._tracking['t0'],'u0')
        self.assertIn(self.m._tracking['t1'],('u1','u2'))
        self.m.tracker.report.assert_not_called()
        self.assertEqual(self.m._eligible_observations,{})

    def test_ambiguous_candidates_cannot_trigger_immediate_dispatch(self):
        self.m._confirm_timeout_bl={}
        self.m.allocator=NS(queue_revisit=lambda *args,**kwargs:None)
        with patch.dict(sm.os.environ,{'IMMEDIATE_VISIBLE_DISPATCH':'1'}):
            self.m._discovery_cb(NS(data=json.dumps(self.payload(appearance_status='ambiguous'))))
        self.assertEqual(self.m._tracking,{})


if __name__=='__main__': unittest.main()
