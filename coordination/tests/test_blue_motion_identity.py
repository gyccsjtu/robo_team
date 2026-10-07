import ast
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

PERCEPTION=Path(__file__).parents[2]/'perception'
sys.path.insert(0,str(PERCEPTION))
from motion_identity import MotionIdentity
from recent_motion import RecentMotion
from fresh_person import FreshPerson


def actual_track_class(enabled):
    """Compile actual tracking methods without importing ROS or YOLO."""
    tree=ast.parse((PERCEPTION/'perception_real.py').read_text())
    constants={'ALPHA','BETA','V_MAX','H_REF','H_SIG','LAG_COMP','PUB_EMA',
        'MOTION_FLOOR','MOTION_REF','CONFIRM_HITS','VERDICT_H_MIN','VERDICT_H_MAX',
        'VERDICT_RANGE_MAX','VERDICT_SP','VERDICT_DISP','VERDICT_SCORE',
        'VERDICT_ATTACH_ON','ATTACH_MIN_VIEW','ATTACH_FRAC','RECENT_MOTION_WINDOW',
        'BLUE_MOTION_WINDOW','BLUE_MOTION_MIN_SPAN','FAST_GREEN_WHITE','WHITE_SHORT_MISS_RECOVERY'}
    selected=[n for n in tree.body if
        isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in constants for t in n.targets)
        or isinstance(n,ast.FunctionDef) and n.name in ('cam_rel','wrap_pi','person_likelihood')
        or isinstance(n,ast.ClassDef) and n.name=='Track']
    scope=dict(math=math,os=os,RecentMotion=RecentMotion,FreshPerson=FreshPerson,
        MotionIdentity=MotionIdentity,MAX_COAST_PUB=3,BLUE_IDENTITY_GUARD=enabled,
        np=NS(zeros=lambda *a,**kw:[0.]*32,float32=float))
    exec(compile(ast.Module(body=selected,type_ignores=[]),'actual_perception_track','exec'),scope)
    scope.update(RECENT_MOTION_WINDOW=4.,BLUE_MOTION_WINDOW=4.,BLUE_MOTION_MIN_SPAN=1.)
    return scope['Track']


class BlueMotionIdentityTests(unittest.TestCase):
    def test_jitter_and_single_jump_do_not_confirm_identity(self):
        for jump in (False,True):
            m=MotionIdentity()
            for i in range(100):
                m.observe(i*.1,(4. if jump and i>=30 else 0.)+.2*math.sin(i*.1),0.)
            self.assertFalse(m.allowed(9.9))

    def test_actual_track_can_move_then_remain_visible_and_stationary(self):
        cls=actual_track_class(True)
        track=cls('blue',0.,0.,.9,(50.,50.),(15.,30.),10.,1.8,10.)
        for i in range(1,191):
            stamp=10.+i*.1
            track.update(min(i*.1,3.5),0.,.1,.9,(50.,50.),(15.,30.),10.,1.8,stamp)
        self.assertTrue(track.verdict(29.,attach_check=False)[0])
        self.assertFalse(track.verdict(30.1,attach_check=False)[0])
        self.assertEqual(track.observed_s,29.)

    def test_loss_reassociation_and_replayed_images_cannot_borrow_identity(self):
        m=MotionIdentity()
        for i in range(41):m.observe(10.+i*.1,i*.1,0.)
        self.assertTrue(m.allowed(14.))
        self.assertFalse(m.observe(14.,100.,0.))
        self.assertFalse(m.observe(13.,100.,0.))
        self.assertTrue(m.allowed(14.))
        m.observe(14.1,50.,0.)
        self.assertFalse(m.allowed(14.1))
        for i in range(1,41):m.observe(14.1+i*.1,50.+i*.1,0.)
        self.assertTrue(m.allowed(18.1))
        m.observe(20.,54.,0.)
        self.assertFalse(m.allowed(20.))

    def test_default_and_other_colors_do_not_get_new_identity_gate(self):
        for enabled,color in [(False,'blue'),(True,'green'),(True,'white'),(True,'brown'),(True,'red')]:
            tr=actual_track_class(enabled)(color,0.,0.,.9,(50.,50.),(15.,30.),10.,1.8,10.)
            self.assertIsNone(tr.blue_identity)

    def test_actual_blue_track_rejects_background_drift_without_deleting_track(self):
        cls=actual_track_class(True)
        tr=cls('blue',0.,0.,.9,(50.,50.),(15.,30.),10.,1.8,10.)
        for i in range(1,81):
            tr.update(.4*math.sin(i*.1),0.,.1,.9,(50.,50.),(15.,30.),10.,1.8,10.+i*.1)
        self.assertEqual(tr.verdict(18.,attach_check=False),(False,'blue_identity'))
        self.assertEqual(tr.hits,81)
