import json
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'))
from visual_observation import VisualEvidence
from report_readiness import ReportReadiness
from legacy_pursuit import LegacyPursuit
import yolo_target_bridge as bridge_module
import test_bridge_acceptance as acceptance_fixture


class LegacyMigrationTests(unittest.TestCase):
    def sample(self,seq=1,stamp=100.,**changes):
        value=dict(schema_version=5,evidence_kind='navigation_candidate',motion_identity_verified=False,run_id='run',
            uav_id='uav_1',seq=seq,sample_s=stamp,target_id='green',frame_id='world_enu',
            xyz=[30.,0.,1.25],confidence=.9,camera_xyz=[0.,0.,3.],
            person_frame_verified=True,observation_id='run:uav_1:%d'%seq)
        value.update(changes)
        return value

    def test_candidate_age_duplicate_and_kind(self):
        gate=VisualEvidence('run',['uav_1'])
        self.assertIsNotNone(gate.receive(self.sample(),100.))
        self.assertIsNone(gate.receive(self.sample(),100.))
        self.assertIsNone(gate.receive(self.sample(2,100.2),102.))
        self.assertIsNone(gate.receive(self.sample(2,100.2,evidence_kind='qualified'),100.2))

    def test_schema4_remains_compatible_without_motion_field(self):
        message=self.sample(schema_version=4)
        del message['motion_identity_verified']
        self.assertIsNotNone(VisualEvidence('run',['uav_1']).receive(message,100.))
        message['schema_version']=5
        self.assertIsNone(VisualEvidence('run',['uav_1']).receive(message,100.))

    def test_candidate_cannot_start_reporting_even_when_close(self):
        ready=ReportReadiness()
        for i in range(3):
            self.assertFalse(ready.observe(self.sample(i+1,100.+i*.4,xyz=[10.,0.,1.25]),100.+i*.4,True))
        self.assertFalse(ready.allowed('green',100.8,100.8))

    def test_actual_bridge_candidate_does_not_touch_official_core(self):
        fixture=acceptance_fixture.BridgeAcceptanceTests()
        b=fixture.bridge()
        b._candidate_core=bridge_module.TargetBridgeCore(brown_activation=True)
        with patch.object(bridge_module,'_msg_string_cls',return_value=NS):
            for i in range(3):
                b._visual_cb(NS(data=json.dumps(self.sample(i+1,100.+i*.2))))
        self.assertEqual(len(b.confirmed),1)
        self.assertEqual(len(b.detected),0)
        self.assertFalse(b.core.tracks['green'].alive)
        self.assertLess(b.core.tracks['green'].t_obs,0.)

    def test_motion_proof_only_blue_cue_never_official(self):
        for color,expected in [('blue',1),('white',0),('green',0)]:
            b=acceptance_fixture.BridgeAcceptanceTests().bridge()
            b._candidate_core=bridge_module.TargetBridgeCore(brown_activation=True)
            with patch.object(bridge_module,'_msg_string_cls',return_value=NS):
                for i in range(3):
                    b._visual_cb(NS(data=json.dumps(self.sample(i+1,100.+i*.2,
                        target_id=color,person_frame_verified=False,motion_identity_verified=True))))
            self.assertEqual(len(b.confirmed),expected)
            self.assertFalse(b.core.tracks[color].alive)

    def test_approach_timeout_and_qualified_recovery(self):
        p=LegacyPursuit()
        self.assertFalse(p.expired(1,'t0',100.))
        p.observe(self.sample())
        self.assertTrue(p.expired(1,'t0',130.))
        p.observe(dict(schema_version=3))
        self.assertTrue(p.expired(1,'t0',131.))
        self.assertFalse(p.expired(2,'t0',132.))
        p.observe(dict(schema_version=3))
        self.assertFalse(p.expired(2,'t0',162.))
