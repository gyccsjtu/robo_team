import json
import ast
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parents[2]/'perception'))
sys.path.insert(0,str(Path(__file__).parents[1]/'src/robocup_swarm/scripts'))
from white_discovery import white_discovery
from visual_observation import VisualEvidence
from report_readiness import ReportReadiness
from official_report_sources import OfficialReportSources
import yolo_target_bridge as B
import test_bridge_acceptance as acceptance_fixture


class WhiteEarlyTests(unittest.TestCase):
    def track(self,**changes):
        t=NS(cls='white',miss=0,hits=2,observed_s=100.,h=1.75,rng=6.65,score_ema=.8,
            raw_xy=(6.,0.),person_support=NS(hits=2,current_verified=True))
        for k,v in changes.items():setattr(t,k,v)
        return t

    def sample(self,**changes):
        m=dict(schema_version=6,run_id='run',uav_id='uav_1',seq=1,sample_s=100.,
            target_id='white',frame_id='world_enu',xyz=[6.,0.,1.25],confidence=.9,
            camera_xyz=[0.,0.,2.5],person_frame_verified=True,observation_id='run:uav_1:1',
            evidence_kind='navigation_candidate',motion_identity_verified=False,
            candidate_reason='white_early_proof',person_hits=2,track_hits=2)
        m.update(changes);return m

    def test_two_fresh_proofs_near_target_guide_before_full_verdict(self):
        self.assertTrue(white_discovery(self.track(),100.5,1.3,2.4,.2))
        for t in [self.track(miss=1),self.track(hits=1),self.track(rng=23.),
                  self.track(cls='blue'),self.track(h=3.),self.track(score_ema=.1),
                  self.track(raw_xy=(float('nan'),0.)),self.track(observed_s=98.)]:
            self.assertFalse(white_discovery(t,100.5,1.3,2.4,.2))
        t=self.track();t.person_support.hits=1
        self.assertFalse(white_discovery(t,100.5,1.3,2.4,.2))
        t.person_support.hits=2;t.person_support.current_verified=False
        self.assertFalse(white_discovery(t,100.5,1.3,2.4,.2))

    def test_actual_perception_branch_accepts_recorded_near_white_counterexample(self):
        path=Path(__file__).parents[2]/'perception/perception_real.py'
        tree=ast.parse(path.read_text(encoding='utf-8'))
        branch=next(n for n in ast.walk(tree) if isinstance(n,ast.If)
                    and any(isinstance(c,ast.Call) and isinstance(c.func,ast.Name)
                            and c.func.id=='white_discovery' for c in ast.walk(n.test)))
        tk=self.track(observed_s=1970.496,h=1.77,rng=6.65,score_ema=.4261447490906253)
        env=dict(tk=tk,now=1971.432,_reject_reason='hits',white_discovery=white_discovery,
            navigation_candidates={},VERDICT_H_MIN=1.3,VERDICT_H_MAX=2.4,VERDICT_SCORE=.2,
            os=NS(environ={'PR_WHITE_EARLY_DISCOVERY':'1'}))
        exec(compile(ast.Module(body=[branch],type_ignores=[]),str(path),'exec'),env)
        self.assertIs(env['navigation_candidates']['white'],tk)
        self.assertIs(tk.navigation_provisional,True)

    def test_schema_fences_proof_kind_counts_and_duplicate(self):
        gate=VisualEvidence('run',['uav_1'])
        self.assertIsNotNone(gate.receive(self.sample(),100.5))
        self.assertIsNone(gate.receive(self.sample(),100.5))
        for changes in [dict(person_frame_verified=False),dict(target_id='blue'),
                        dict(person_hits=1),dict(track_hits=True),dict(candidate_reason='qualified'),
                        dict(motion_identity_verified=True)]:
            self.assertIsNone(VisualEvidence('run',['uav_1']).receive(self.sample(**changes),100.5))

    def test_actual_bridge_forwards_first_early_cue_but_never_official(self):
        b=acceptance_fixture.BridgeAcceptanceTests().bridge();b._candidate_core=B.TargetBridgeCore()
        b._trace_record=lambda *a:None
        b._report_readiness=ReportReadiness()
        b._official_sources=OfficialReportSources(B.TargetBridgeCore,b._report_readiness)
        with patch.object(B,'_msg_string_cls',return_value=NS):
            b._visual_cb(NS(data=json.dumps(self.sample())))
        self.assertEqual(len(b.confirmed),1)
        self.assertEqual(json.loads(b.confirmed[0].data)['schema_version'],6)
        self.assertEqual(b.detected,[])
        self.assertFalse(b._official_sources.cores)
        self.assertFalse(b._report_readiness.sources)
        self.assertLess(b.core.tracks['white'].t_obs,0)

    def test_candidate_never_relabels_as_qualified_and_distance_gate_holds(self):
        b=acceptance_fixture.BridgeAcceptanceTests().bridge();b._candidate_core=B.TargetBridgeCore()
        b._trace_record=lambda *a:None
        with patch.object(B,'_msg_string_cls',return_value=NS):
            b._visual_cb(NS(data=json.dumps(self.sample(xyz=[23.,0.,1.25]))))
        self.assertEqual(b.confirmed,[])
        r=ReportReadiness()
        for i in range(3):
            m=self.sample(seq=i+1,sample_s=100.+i*.4,observation_id='run:uav_1:%d'%(i+1))
            self.assertFalse(r.observe(m,100.+i*.4,True))
        self.assertFalse(r.allowed('white',100.8,100.8))
