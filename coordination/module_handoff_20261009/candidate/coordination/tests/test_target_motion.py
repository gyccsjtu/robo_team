import sys
import ast
import unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'))
from target_motion import TargetMotion


class TargetMotionTests(unittest.TestCase):
    def test_sustained_two_meter_motion_enables_chase(self):
        motion=TargetMotion()
        for t in (0.,.5,1.,1.5):result=motion.observe('t0','uav_1',t,2*t,0.)
        self.assertEqual(result,(2.,0.,True))

    def test_walking_does_not_enable_fast_chase(self):
        motion=TargetMotion()
        for t in (0.,.5,1.,1.5):result=motion.observe('t0','uav_1',t,t,0.)
        self.assertEqual(result,(1.,0.,False))

    def test_camera_handoff_cannot_create_velocity_from_calibration_offset(self):
        motion=TargetMotion()
        for t in (0.,.5,1.):motion.observe('t0','uav_1',t,t,0.)
        self.assertEqual(motion.observe('t0','uav_2',1.1,20.,0.),(0.,0.,False))

    def test_gap_jump_and_replay_do_not_create_flee_evidence(self):
        motion=TargetMotion()
        for t in (0.,.5,1.):motion.observe('t0','uav_1',t,2*t,0.)
        self.assertIsNone(motion.observe('t0','uav_1',1.,100.,0.))
        self.assertEqual(motion.observe('t0','uav_1',3.,100.,0.),(0.,0.,False))
        motion=TargetMotion()
        for t,x in ((0.,0.),(.5,0.),(1.,20.)):result=motion.observe('t0','uav_1',t,x,0.)
        self.assertEqual(result,(0.,0.,False))

    def test_actual_executor_rejects_stale_and_future_flee_state(self):
        source=Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts/swarm_agent.py'
        tree=ast.parse(source.read_text(encoding='utf-8'))
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_target_fleeing')
        scope=dict(FLEE_STATE_FRESH=3.,rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:10.))))
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),scope)
        for stamp,expected in ((9.,True),(6.9,False),(10.1,False)):
            node=SimpleNamespace(_target_state={'t0':(1,stamp)})
            self.assertEqual(scope['_target_fleeing'](node,'t0'),expected)
