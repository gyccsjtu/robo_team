import ast
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest

SCRIPTS = Path(__file__).parents[1]/'src/robocup_swarm/scripts'
sys.path.insert(0,str(SCRIPTS))
from visual_observation import VisualEvidence, TAG_TO_TID
from target_motion import TargetMotion


class TargetMessage:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None,frame_id='')


class ConfirmedVisualAdaptersTests(unittest.TestCase):
    def adapter(self, file, previous=None):
        tree = ast.parse((SCRIPTS/file).read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name in ('SwarmAgent','SwarmManager'))
        function = next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_confirmed_visual_cb')
        clock = SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:10.8), from_sec=lambda t:t)
        scope = dict(json=json,rospy=SimpleNamespace(Time=clock),TAG_TO_TID=TAG_TO_TID,
                     TargetState=TargetMessage,TargetDetection=TargetMessage)
        exec(compile(ast.Module(body=[function],type_ignores=[]),str(SCRIPTS/file),'exec'),scope)
        received=[]
        instance=SimpleNamespace(_authority_lock=threading.RLock(),_visual_evidence=VisualEvidence('current',['uav_1']),
            _visual_motion=TargetMotion(),
            _target_cb=received.append,_detection_cb=received.append,_last_detect=previous or {},_t_seen=previous or {})
        return lambda m:scope['_confirmed_visual_cb'](instance,SimpleNamespace(data=json.dumps(m))),received

    def observation(self, **changes):
        message=dict(schema_version=2,run_id='current',uav_id='uav_1',seq=1,sample_s=10.5,
            target_id='red1',frame_id='world_enu',xyz=[2.,3.,1.25],confidence=.9,observation_id='current:uav_1:1')
        message.update(changes)
        return message

    def test_manager_and_executor_preserve_original_sample_mapping_and_coordinates(self):
        for file in ('swarm_manager.py','swarm_agent.py'):
            with self.subTest(file=file):
                callback,received=self.adapter(file)
                callback(self.observation())
                self.assertEqual(len(received),1)
                message=received[0]
                self.assertEqual((message.header.stamp,message.target_id,message.x,message.y),(10.5,'t5',2.,3.))

    def test_old_run_replay_same_frame_and_stale_data_never_reach_tracking(self):
        for file in ('swarm_manager.py','swarm_agent.py'):
            with self.subTest(file=file):
                callback,received=self.adapter(file)
                callback(self.observation(run_id='old',observation_id='old:uav_1:1'))
                self.assertEqual(received,[])
                callback(self.observation())
                callback(self.observation())
                callback(self.observation(seq=2,observation_id='current:uav_1:2'))
                callback(self.observation(seq=3,sample_s=9.,observation_id='current:uav_1:3'))
                self.assertEqual(len(received),1)

    def test_delayed_other_camera_cannot_regress_newer_target_state(self):
        for file in ('swarm_manager.py','swarm_agent.py'):
            callback,received=self.adapter(file, {'t5':10.6})
            callback(self.observation(sample_s=10.5))
            self.assertEqual(received,[])
            callback(self.observation(seq=2,sample_s=10.7,observation_id='current:uav_1:2'))
            self.assertEqual(len(received),1)

    def test_actual_executor_adapter_estimates_flee_motion_without_refreshing_image_time(self):
        callback,received=self.adapter('swarm_agent.py')
        for seq,stamp in enumerate((9.9,10.2,10.5,10.8),1):
            callback(self.observation(seq=seq,sample_s=stamp,xyz=[2*(stamp-9.9),0.,1.25],
                                     observation_id='current:uav_1:%d'%seq))
        self.assertEqual(received[-1].state,1)
        self.assertAlmostEqual(received[-1].vx,2.)
        self.assertEqual(received[-1].header.stamp,10.8)

    def test_approach_follows_fresh_assigned_target_without_other_target_or_stale_redirect(self):
        tree=ast.parse((SCRIPTS/'swarm_agent.py').read_text(encoding='utf-8'))
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SwarmAgent')
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_target_cb')
        scope=dict(TARGET_TTL=1.,rospy=SimpleNamespace(Time=SimpleNamespace(
            now=lambda:SimpleNamespace(to_sec=lambda:10.8))))
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'actual_target_cb','exec'),scope)
        a=SimpleNamespace(assignment=SimpleNamespace(task_type=1,target_id='t5'),
            targets={},_target_state={},_t_seen={},_target_to_orbit=(0.,0.))
        def feed(tid,stamp,xy):
            scope['_target_cb'](a,SimpleNamespace(eliminated=False,target_id=tid,
                header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:stamp)),
                x=xy[0],y=xy[1],vx=2.,vy=0.,state=1))
        feed('t5',10.5,(3.,4.));self.assertEqual(a._target_to_orbit,(3.,4.))
        feed('t3',10.6,(8.,9.));self.assertEqual(a._target_to_orbit,(3.,4.))
        feed('t5',8.,(20.,20.));self.assertEqual(a._target_to_orbit,(3.,4.))
