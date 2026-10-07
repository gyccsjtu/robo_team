import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).parents[2]/'perception'))
from stationary_person import allowed
from person_verifier import PersonVerifier
from shared_inference_client import SharedInferenceClient


class StationaryPersonTests(unittest.TestCase):
    def test_only_current_fresh_authorized_green_person_has_support(self):
        self.assertTrue(allowed('t0','green',True,0,10.,10.5))
        for target,color,verified,miss,stamp,now in [
                (None,'green',True,0,10.,10.5),('t2','green',True,0,10.,10.5),
                ('t0','blue',True,0,10.,10.5),('t0','green',False,0,10.,10.5),
                ('t0','green',True,1,10.,10.5),('t0','green',True,0,10.,11.01),
                ('t0','green',True,0,10.,9.9),('t0','green',True,0,float('nan'),10.)]:
            self.assertFalse(allowed(target,color,verified,miss,stamp,now))

    def test_person_and_color_filters_both_required(self):
        self.assertTrue(PersonVerifier('weights',color_check=True).green_proof_enabled())
        for verifier in [PersonVerifier(),PersonVerifier('weights'),
                         PersonVerifier('weights',color_check=True,classes=(3,))]:
            self.assertFalse(verifier.green_proof_enabled())

    def test_legacy_or_unsupported_shared_reply_cannot_grant_proof(self):
        client=SharedInferenceClient()
        fake_cv=SimpleNamespace(IMREAD_COLOR=1,IMREAD_UNCHANGED=-1,
            imencode=lambda *a:(True,SimpleNamespace(tobytes=lambda:b'raw')))
        with patch.dict(sys.modules,cv2=fake_cv):
            for extra,expected in [({},False),
                    ({'verification_version':3,'verified_green_person':True},False),
                    ({'verification_version':1,'verified_green_person':1},False),
                    ({'verification_version':1,'verified_green_person':True},True)]:
                client._rpc=lambda payload,extra=extra:dict(ok=True,dets=[],**extra)
                boxes,meta=client.infer(object())
                self.assertEqual(boxes,[])
                self.assertEqual(meta['verified_green_person'],expected)

    def verdict(self, proof, verified=False, **changes):
        source=Path(__file__).parents[2]/'perception/perception_real.py'
        track=next(n for n in ast.parse(source.read_text()).body
                   if isinstance(n,ast.ClassDef) and n.name=='Track')
        method=next(n for n in track.body if isinstance(n,ast.FunctionDef) and n.name=='verdict')
        env=dict(MAX_COAST_PUB=2,CONFIRM_HITS=3,VERDICT_ATTACH_ON=1,
            VERDICT_H_MIN=1.3,VERDICT_H_MAX=2.4,VERDICT_RANGE_MAX=22.,
            VERDICT_SP=.15,VERDICT_DISP=2.5,VERDICT_SCORE=.2)
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),env)
        values=dict(hits=10,miss=0,h=1.8,rng=10.,observed_s=10.,
            sp=0.,max_disp=0.,score_ema=.8,motion_factor=lambda now:None,
        attached_to_cam=lambda:False,recent_motion=SimpleNamespace(speed=lambda now:0.),blue_identity=None)
        values.update(changes)
        return env['verdict'](SimpleNamespace(**values),now=10.5,stationary_person=proof,verified_person=verified)

    def test_new_client_accepts_red_proof_only_from_version_two(self):
        client=SharedInferenceClient()
        fake_cv=SimpleNamespace(imencode=lambda *a:(True,SimpleNamespace(tobytes=lambda:b'raw')),
            IMREAD_COLOR=1,IMREAD_UNCHANGED=-1)
        with patch.dict(sys.modules,cv2=fake_cv):
            for version,colors in ((1,['green','white']),(2,['green','white','red']),(3,[])):
                client._rpc=lambda payload,version=version:dict(ok=True,dets=[],verification_version=version,
                    verified_person_colors=['green','white','red','unknown'])
                _,meta=client.infer(object())
                self.assertEqual(meta['verified_person_colors'],colors)

    def test_verified_person_avoids_static_attachment_veto_but_keeps_geometry(self):
        self.assertEqual(self.verdict(False,verified=True,hits=3,attached_to_cam=lambda:True),(True,''))
        self.assertEqual(self.verdict(False,verified=True,h=.5),(False,'height'))
        self.assertEqual(self.verdict(False,verified=True,rng=30.),(False,'range'))
        self.assertEqual(self.verdict(False,verified=True,miss=3),(False,'coast'))

    def test_actual_verdict_preserves_stationary_person_and_rejects_unverified_static(self):
        self.assertEqual(self.verdict(True),(True,''))
        self.assertEqual(self.verdict(False),(False,'static'))

    def test_static_support_does_not_disable_geometry_or_attachment(self):
        self.assertEqual(self.verdict(True,rng=30.),(False,'range'))
        self.assertEqual(self.verdict(True,h=.5),(False,'height'))
        self.assertEqual(self.verdict(True,attached_to_cam=lambda:True),(False,'self'))


if __name__ == '__main__':unittest.main()
