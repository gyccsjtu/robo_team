import ast
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest

SCRIPTS = Path(__file__).parents[1]/'src/robocup_swarm/scripts'
sys.path.insert(0, str(SCRIPTS))
from own_visual_guidance import OwnVisualGuidance, tracking_input
from target_motion import TargetMotion
from visual_observation import VisualEvidence, TAG_TO_TID


class OwnGuidanceTests(unittest.TestCase):
    def test_own_fresh_image_wins_over_newer_foreign_point(self):
        own = OwnVisualGuidance('uav_1')
        own.observe('t0','uav_1',10.5,(10.,0.),(2.,0.),10.8)
        agent = NS(_own_visual_guidance=own, targets={'t0':(14.,0.,0.,0.)}, _t_seen={'t0':10.7})
        self.assertEqual(tracking_input(agent,'t0',10.8), ((10.,0.,2.,0.),10.5))
        self.assertEqual(tracking_input(agent,'t0',11.501), ((14.,0.,0.,0.),10.7))

    def test_duplicate_foreign_future_and_stale_do_not_renew_own_image(self):
        own = OwnVisualGuidance('uav_1')
        self.assertTrue(own.observe('t0','uav_1',10.,(10.,0.),(0.,0.),10.))
        for uid,stamp,now in (('uav_2',10.2,10.2),('uav_1',10.,10.2),
                              ('uav_1',11.,10.9),('uav_1',10.2,11.3)):
            self.assertFalse(own.observe('t0',uid,stamp,(20.,0.),(0.,0.),now))
        self.assertEqual(own.current('t0',10.9)[1],10.)
        self.assertIsNone(own.current('t0',11.01))
        own.clear('t0')
        self.assertIsNone(own.current('t0',10.9))

    def test_separate_targets_and_nonfinite_inputs(self):
        own = OwnVisualGuidance('uav_1')
        self.assertFalse(own.observe('t0','uav_1',10.,(float('nan'),0.),(0.,0.),10.))
        own.observe('t1','uav_1',10.,(10.,0.),(0.,0.),10.)
        self.assertIsNone(own.current('t0',10.))

    def callback(self, filename, agent):
        tree = ast.parse((SCRIPTS/filename).read_text(encoding='utf-8'))
        method = next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)
                      and n.name == '_confirmed_visual_cb')
        class Message:
            def __init__(self): self.header = NS()
        scope = dict(json=json,TAG_TO_TID=TAG_TO_TID,TargetState=Message,TargetDetection=Message,
            rospy=NS(Time=NS(now=lambda:NS(to_sec=lambda:10.9),from_sec=lambda t:t)))
        exec(compile(ast.Module(body=[method],type_ignores=[]),'actual_adapter','exec'),scope)
        return lambda m: scope['_confirmed_visual_cb'](agent,NS(data=json.dumps(m)))

    def observation(self, stamp, seq=1):
        return dict(schema_version=3,run_id='run',uav_id='uav_1',seq=seq,sample_s=stamp,
            target_id='green',frame_id='world_enu',xyz=[10.,0.,1.25],confidence=.9,
            camera_xyz=[0.,0.,2.5],person_frame_verified=True,observation_id='run:uav_1:%d'%seq)

    def test_actual_agent_keeps_delayed_own_evidence_without_regressing_global(self):
        forwarded = []
        agent = NS(_authority_lock=threading.RLock(),_visual_evidence=VisualEvidence('run',['uav_1']),
            _visual_motion=TargetMotion(),_own_visual_guidance=OwnVisualGuidance('uav_1'),
            _t_seen={'t0':10.8},_target_cb=forwarded.append)
        callback = self.callback('swarm_agent.py',agent)
        callback(self.observation(10.6))
        self.assertFalse(forwarded)
        self.assertEqual(agent._own_visual_guidance.current('t0',10.9)[1],10.6)
        self.assertEqual(agent._t_seen['t0'],10.8)
        self.assertEqual(agent._visual_motion.history[('t0','uav_1')][-1][0],10.6)

    def test_actual_manager_records_source_without_regressing_position_or_dispatch(self):
        reports, assignments, forwarded = [], [], []
        tracker = NS(targets={'t0':NS(observers=set())},
            report=lambda *a,**kw:reports.append((a,kw)),
            assign_observers=lambda *a:assignments.append(a))
        agent = NS(_authority_lock=threading.RLock(),_visual_evidence=VisualEvidence('run',['uav_1']),
            _last_detect={'t0':10.8},tracker=tracker,_eliminated=set(),_detection_cb=forwarded.append)
        callback = self.callback('swarm_manager.py',agent)
        callback(self.observation(10.6))
        self.assertFalse(forwarded)
        self.assertEqual(reports[0][0],('uav_1','t0',10.6,10.,0.))
        self.assertEqual(reports[0][1],{'truth':None})
        self.assertEqual(agent._last_detect['t0'],10.8)
        self.assertEqual(assignments,[('t0',['uav_1'])])
        callback(self.observation(10.6))
        self.assertEqual(len(reports),1)


if __name__ == '__main__':
    unittest.main()
