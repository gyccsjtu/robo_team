import ast
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parents[2]/'perception'))
sys.path.insert(0,str(Path(__file__).parents[1]/'src/robocup_swarm/scripts'))
from motion_identity import MotionIdentity
from white_discovery import blue_discovery
from visual_observation import VisualEvidence
from report_readiness import ReportReadiness
from official_report_sources import OfficialReportSources
from legacy_pursuit import LegacyPursuit
import yolo_target_bridge as B
import test_bridge_acceptance as fixture


class BlueEarlyTests(unittest.TestCase):
    def motion(self):
        m=MotionIdentity()
        for t,x in [(99.2,0.),(99.6,.5),(100.,1.)]:m.observe(t,x,0.)
        return m

    def track(self,**changes):
        t=NS(cls='blue',miss=0,hits=3,observed_s=100.,h=1.75,rng=40.,score_ema=.8,
             raw_xy=(1.,0.),person_support=NS(hits=2,current_verified=True),blue_identity=self.motion())
        for k,v in changes.items():setattr(t,k,v)
        return t

    def sample(self,**changes):
        m=dict(schema_version=7,run_id='run',uav_id='uav_1',seq=1,sample_s=100.,
               target_id='blue',frame_id='world_enu',xyz=[40.,0.,1.25],confidence=.9,
               camera_xyz=[0.,0.,2.5],person_frame_verified=True,observation_id='run:uav_1:1',
               evidence_kind='navigation_candidate',motion_identity_verified=False,
               candidate_reason='blue_approach_motion',person_hits=2,track_hits=3,
               approach_motion_verified=True)
        m.update(changes);return m

    def test_short_motion_never_qualifies_official_identity(self):
        m=self.motion()
        self.assertTrue(m.approach_allowed(100.))
        self.assertTrue(m.approach_allowed(100.8))
        self.assertFalse(m.allowed(100.))
        self.assertFalse(m.qualified)
        self.assertFalse(m.observe(100.,100.,0.))
        self.assertTrue(m.approach_allowed(100.))
        self.assertFalse(m.approach_allowed(101.1))
        m.observe(100.1,50.,0.)
        self.assertFalse(m.approach_allowed(100.1))

    def test_static_jitter_single_jump_and_excess_speed_reject(self):
        for xs in [(0.,0.,0.),(0.,.2,-.2),(0.,0.,1.),(0.,1.5,3.)]:
            m=MotionIdentity()
            for t,x in zip((99.2,99.6,100.),xs):m.observe(t,x,0.)
            self.assertFalse(m.approach_allowed(100.))

    def test_body_current_geometry_and_original_age_still_required(self):
        self.assertTrue(blue_discovery(self.track(),100.,1.3,2.4,.2))
        for kwargs in [dict(miss=1),dict(hits=1),dict(rng=46.),dict(h=3.),dict(score_ema=.1),
                       dict(cls='white'),dict(observed_s=98.),dict(blue_identity=None),
                       dict(raw_xy=(float('nan'),0.)),
                       dict(person_support=NS(hits=1,current_verified=True)),
                       dict(person_support=NS(hits=3,current_verified=False))]:
            self.assertFalse(blue_discovery(self.track(**kwargs),100.,1.3,2.4,.2))

    def test_recorded_uav4_three_original_window_survives_processing_latency(self):
        # CSV rounded raw originals; finite reconstruction, not full association replay.
        m=MotionIdentity()
        for row in [(1969.448,57.47,12.71),(1970.368,59.9,13.42),(1970.8,60.53,13.44)]:
            m.observe(*row)
        t=self.track(observed_s=1970.8,raw_xy=(60.53,13.44),rng=36.69,
                     h=1.91,score_ema=.6544561320776567,hits=13,blue_identity=m)
        self.assertTrue(blue_discovery(t,1971.284,1.3,2.4,.2))
        self.assertFalse(m.allowed(1971.284))
        self.assertFalse(blue_discovery(t,1971.801,1.3,2.4,.2))

    def test_actual_perception_branch_only_enabled_navigation(self):
        path=Path(__file__).parents[2]/'perception/perception_real.py'
        tree=ast.parse(path.read_text(encoding='utf-8'))
        branch=next(n for n in ast.walk(tree) if isinstance(n,ast.If)
                    and any(isinstance(c,ast.Call) and isinstance(c.func,ast.Name)
                            and c.func.id=='blue_discovery' for c in ast.walk(n.test)))
        for flag,reason,expected in [('1','range',True),('0','range',False),('1','height',False)]:
            tk=self.track();env=dict(tk=tk,now=100.,_reject_reason=reason,blue_discovery=blue_discovery,
                navigation_candidates={},VERDICT_H_MIN=1.3,VERDICT_H_MAX=2.4,VERDICT_SCORE=.2,
                os=NS(environ={'PR_BLUE_EARLY_DISCOVERY':flag}))
            exec(compile(ast.Module(body=[branch],type_ignores=[]),str(path),'exec'),env)
            self.assertEqual('blue' in env['navigation_candidates'],expected)

    def test_wire_rejects_missing_proof_false_motion_wrong_color_and_replay(self):
        gate=VisualEvidence('run',['uav_1']);self.assertIsNotNone(gate.receive(self.sample(),100.))
        self.assertIsNone(gate.receive(self.sample(),100.))
        for kw in [dict(target_id='white'),dict(approach_motion_verified=False),
                   dict(person_frame_verified=False),dict(person_hits=1),dict(track_hits=True),
                   dict(candidate_reason='qualified'),dict(evidence_kind='qualified')]:
            self.assertIsNone(VisualEvidence('run',['uav_1']).receive(self.sample(**kw),100.))

    def test_actual_bridge_forwards_first_cue_never_official(self):
        for distance,expected in [(40.,1),(10.,1),(46.,0)]:
            b=fixture.BridgeAcceptanceTests().bridge();b._candidate_core=B.TargetBridgeCore()
            b._trace_record=lambda *a:None;b._report_readiness=ReportReadiness()
            b._official_sources=OfficialReportSources(B.TargetBridgeCore,b._report_readiness)
            with patch.object(B,'_msg_string_cls',return_value=NS):
                b._visual_cb(NS(data=json.dumps(self.sample(xyz=[distance,0.,1.25]))))
            self.assertEqual(len(b.confirmed),expected)
            self.assertEqual(b.detected,[])
            self.assertFalse(b._official_sources.cores)
            self.assertFalse(b._report_readiness.sources)
            self.assertLess(b.core.tracks['blue'].t_obs,0.)

    def test_repeated_candidates_cannot_extend_bounded_approach_or_report(self):
        p=LegacyPursuit();r=ReportReadiness()
        self.assertFalse(p.expired(1,'t1',100.))
        for i in range(31):
            m=self.sample(seq=i+1,sample_s=100.+i,observation_id='run:uav_1:%d'%(i+1),xyz=[10.,0.,1.25])
            p.observe(m);self.assertFalse(r.observe(m,100.+i,True))
        self.assertTrue(p.expired(1,'t1',130.))
        self.assertFalse(r.allowed('blue',130.,130.))
