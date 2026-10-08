"""Actual manager callback/ranking/dispatch, based on the live blue counterexample."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import unittest
import threading
import test_camera_takeover_adapter as fixture
from tracker_selection import tracker_rank
from visual_observation import VisualEvidence,TAG_TO_TID

path=Path(__file__).parents[1]/'src/robocup_swarm/scripts/swarm_manager.py'
tree=ast.parse(path.read_text(encoding='utf-8'))
methods=[n for c in tree.body if isinstance(c,ast.ClassDef) for n in c.body
         if isinstance(n,ast.FunctionDef) and n.name in ('_tracker_rank','_confirmed_visual_cb')]


class ApproachObserverDispatchTests(unittest.TestCase):
    def manager(self,original=1957.212):
        now=1957.856
        m=fixture.CameraTakeoverAdapterTests().manager()
        m._authority_lock=threading.RLock()
        m._last_detect={};m._visual_evidence=VisualEvidence('run',['live','blind','owner'])
        m.tracker=NS(targets={})
        m.status['live'].x=37.5;m.status['blind'].x=42.5
        clock=NS(now=lambda:NS(to_sec=lambda:now),from_sec=lambda s:s)
        scope=dict(json=json,TAG_TO_TID=TAG_TO_TID,tracker_rank=tracker_rank,DETECT_RADIUS=20.,
                   rospy=NS(Time=clock),TargetDetection=lambda:NS(header=NS()))
        exec(compile(ast.Module(body=methods,type_ignores=[]),str(path),'exec'),scope)
        m._tracker_rank=lambda tid,uid,d:scope['_tracker_rank'](m,tid,uid,d)
        m._detection_cb=Mock()
        payload=dict(schema_version=7,run_id='run',uav_id='live',seq=1,sample_s=original,
            target_id='blue',frame_id='world_enu',xyz=[0.,0.,1.25],confidence=.791,
            camera_xyz=[-37.5,0.,2.25],person_frame_verified=True,observation_id='run:live:1',
            evidence_kind='navigation_candidate',motion_identity_verified=False,
            candidate_reason='blue_approach_motion',person_hits=3,track_hits=4,approach_motion_verified=True)
        return m,scope,payload

    def test_received_live_candidate_prefers_search_observer_over_blind_idle(self):
        m,s,p=self.manager()
        s['_confirmed_visual_cb'](m,NS(data=json.dumps(p)))
        self.assertEqual(m._approach_observers,{('live','blue'):1957.212})
        m._detection_cb.assert_called_once()
        fixture.scope['_dispatch_tracker'](m,'t1',0.,0.)
        self.assertEqual(m._authorized_publish.call_args.args[0].uav_id,'live')
        self.assertEqual(m._tracking,{'t1':'live'})
        m._authority.withdraw.assert_not_called()  # Existing publisher owns the normal STOP handoff.

    def test_stale_rejected_candidate_does_not_prefer_observer(self):
        m,s,p=self.manager(original=1956.)
        s['_confirmed_visual_cb'](m,NS(data=json.dumps(p)))
        m._detection_cb.assert_not_called()
        fixture.scope['_dispatch_tracker'](m,'t1',0.,0.)
        self.assertEqual(m._authorized_publish.call_args.args[0].uav_id,'blind')

    def test_changed_original_or_navigation_ineligibility_keeps_existing_fences(self):
        for mode in ('changed_original','blocked','task_rejected'):
            m,s,p=self.manager();s['_confirmed_visual_cb'](m,NS(data=json.dumps(p)))
            if mode=='changed_original':m._visual_evidence.stamps[('live','blue')]=1957.5
            elif mode=='blocked':m._navigation_eligible=lambda uid:uid!='live'
            else:m._navigation_task_allowed=lambda uid,key:uid!='live'
            fixture.scope['_dispatch_tracker'](m,'t1',0.,0.)
            self.assertEqual(m._authorized_publish.call_args.args[0].uav_id,'blind')

    def test_qualified_original_clears_only_that_source_candidate_marker(self):
        m,s,p=self.manager();s['_confirmed_visual_cb'](m,NS(data=json.dumps(p)))
        p={k:v for k,v in p.items() if k not in ('evidence_kind','motion_identity_verified',
            'candidate_reason','person_hits','track_hits','approach_motion_verified')}
        p.update(schema_version=3,seq=2,sample_s=1957.5,observation_id='run:live:2')
        s['_confirmed_visual_cb'](m,NS(data=json.dumps(p)))
        self.assertEqual(m._approach_observers,{})
