import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
sys.path.insert(0,str(Path(__file__).parents[1]/'src/robocup_swarm/scripts'))
from visual_observation import VisualEvidence
from report_readiness import ReportReadiness
from yolo_target_bridge import TargetBridgeCore,YoloTargetBridge


def image(stamp,seq=1,tag='green',distance=10.,uid='uav_1',version=3,person=True):
    m=dict(schema_version=version,run_id='run',uav_id=uid,seq=seq,sample_s=stamp,
        target_id=tag,frame_id='world_enu',xyz=[distance,0.,1.25],confidence=.9,
        observation_id='run:%s:%d'%(uid,seq))
    if version==3:m.update(camera_xyz=[0.,0.,2.5],person_frame_verified=person)
    return m


class ApproachTests(unittest.TestCase):
    def test_v3_camera_proof_validated_and_v2_remains_internal_compatible(self):
        for m in (image(10.),image(10.,version=2)):
            self.assertEqual(VisualEvidence('run',['uav_1']).receive(m,10.),m)
        for extra in ({'camera_xyz':[math.nan,0.,2.]},{'camera_xyz':[0.,1.]},
                      {'person_frame_verified':1},{'camera_xyz':[True,0.,2.]}):
            self.assertIsNone(VisualEvidence('run',['uav_1']).receive(dict(image(10.),**extra),10.))
        self.assertIsNone(VisualEvidence('run',['uav_1']).receive(dict(image(10.),schema_version=2),10.))

    def test_far_discovery_does_not_upload_and_three_close_originals_do(self):
        gate=ReportReadiness()
        for i,t in enumerate((10.,10.3,10.6)):
            self.assertFalse(gate.observe(image(t,i+1,distance=15.),t,True))
        for i,t in enumerate((11.,11.3,11.6)):
            self.assertEqual(gate.observe(image(t,i+4),t,True),i==2)
        self.assertTrue(gate.allowed('green',11.8,11.6))
        self.assertFalse(gate.allowed('green',12.61,11.6))
        self.assertFalse(gate.observe(image(12.7,7),12.7,True))

    def test_duplicate_camera_mixing_short_span_and_v2_cannot_start(self):
        gate=ReportReadiness()
        gate.observe(image(10.),10.,True)
        gate.observe(image(10.,2),10.,True)
        gate.observe(image(10.3,3),10.3,True)
        self.assertFalse(gate.allowed('green',10.3,10.3))
        gate.observe(image(10.6,4,uid='uav_2'),10.6,True)
        self.assertFalse(gate.allowed('green',10.6,10.6))
        for i,t in enumerate((11.,11.1,11.2)):
            self.assertFalse(gate.observe(image(t,i+5,version=2),t,True))
        gate=ReportReadiness()
        for i,t in enumerate((10.,10.1,10.2)):
            self.assertFalse(gate.observe(image(t,i+1),t,True))

    def test_white_needs_verified_person_and_loss_or_clear_resets(self):
        gate=ReportReadiness()
        for i,t in enumerate((10.,10.3,10.6)):
            self.assertFalse(gate.observe(image(t,i+1,tag='white',person=False),t,True))
        for i,t in enumerate((11.,11.3,11.6)):
            gate.observe(image(t,i+4,tag='white'),t,True)
        self.assertTrue(gate.allowed('white',11.6,11.6))
        self.assertFalse(gate.observe(image(11.7,7,tag='white',person=False),11.7,True))
        self.assertFalse(gate.allowed('white',11.7,11.7))
        for i,t in enumerate((12.,12.3,12.6)):
            gate.observe(image(t,i+8,tag='white'),t,True)
        gate.clear('white')
        self.assertFalse(gate.allowed('white',12.6,12.6))

    def test_ongoing_tracking_uses_fresh_images_and_not_older_permission(self):
        gate=ReportReadiness()
        for i,t in enumerate((10.,10.3,10.6)):gate.observe(image(t,i+1),t,True)
        self.assertTrue(gate.observe(image(10.9,4,distance=18.),10.9,True))
        self.assertFalse(gate.allowed('green',10.9,11.))
        self.assertFalse(gate.allowed('green',9.,10.9))
        self.assertFalse(gate.allowed('green',10.9,10.9))

    def test_actual_bridge_keeps_internal_state_while_official_output_is_gated(self):
        b=YoloTargetBridge.__new__(YoloTargetBridge)
        b._emit_stats={};b._skip_trace_t={}
        b.core=TargetBridgeCore();b._approach_reporting=True;b._report_readiness=ReportReadiness()
        b._now=lambda:10.6;b._rospy=NS(Time=NS(from_sec=lambda t:t))
        b._TargetState=lambda:NS(header=NS());b._ActorInfo=NS
        internal,official=[],[];b.pub=NS(publish=internal.append)
        b._actor_pubs={'green':NS(publish=official.append)}
        b.core.tracks['green'].t_obs=10.6
        event=dict(tag='green',tid='t0',x=10.,y=0.,vx=0.,vy=0.,state=0,eliminated=False)
        b._emit(event);self.assertEqual(len(internal),1);self.assertFalse(official)
        for i,t in enumerate((10.,10.3,10.6)):b._report_readiness.observe(image(t,i+1),t,True)
        b._emit(event);self.assertEqual(len(official),1)
        b._now=lambda:11.61;b._emit(event)
        self.assertEqual(len(official),1);self.assertEqual(b.core.tracks['green'].t_obs,10.6)
