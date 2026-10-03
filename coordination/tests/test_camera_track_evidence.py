"""Exercise the actual tracker without importing ROS, Torch or a model."""
import ast
import math
from pathlib import Path
import unittest
import numpy as np


class CameraTrackEvidenceTests(unittest.TestCase):
    def track(self):
        source = Path(__file__).parents[2]/'perception/perception_real.py'
        node = next(n for n in ast.parse(source.read_text(encoding='utf-8')).body
                    if isinstance(n, ast.ClassDef) and n.name=='Track')
        scope = dict(math=math, np=np, cam_rel=lambda ux,uy,yaw,x,y:(x-ux,y-uy),
                     person_likelihood=lambda h:1., MOTION_FLOOR=.2, ALPHA=.5,
                     BETA=.15, V_MAX=2., MOTION_REF=1., LAG_COMP=0., PUB_EMA=0.,
                     wrap_pi=lambda angle:angle)
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), scope)
        return scope['Track']('green',0.,0.,.9,[10,20],[20,50],6.,1.7,10.)

    def test_replayed_image_cannot_accumulate_hits_or_change_position(self):
        track = self.track()
        for _ in range(10):
            track.update(4.,0.,.1,.9,[10,20],[20,50],6.,1.7,10.)
        self.assertEqual((track.hits,track.x,track.observed_s), (1,0.,10.))

    def test_prediction_advances_state_time_but_never_observation_evidence(self):
        track = self.track()
        track.vx = 1.
        track.coast(.5)
        track.coast(.5)
        self.assertEqual((track.x,track.t,track.observed_s,track.hits,track.miss), (1.,11.,10.,1,2))

    def test_new_frame_renews_only_its_original_camera_timestamp(self):
        track = self.track()
        track.update(.4,0.,.5,.9,[10,20],[20,50],6.,1.7,10.5)
        self.assertEqual((track.hits,track.observed_s), (2,10.5))
