import ast
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
SCRIPTS = Path(__file__).parents[1]/'src/robocup_swarm/scripts'
sys.path.insert(0, str(SCRIPTS))
from source_aligned_fusion import SourceAlignedFusion
from white_reacquisition import WhiteReacquisition
from yolo_target_bridge import TargetBridgeCore
from navigation_feedback import accept
from visual_observation import TAG_TO_TID


class BrownWhiteRecoveryTests(unittest.TestCase):
    def test_source_change_is_not_velocity(self):
        f = SourceAlignedFusion()
        f.observe('a', 1., (0.,0.), .9, 'a1')
        r = f.observe('b', 1.2, (2.,0.), .9, 'b1')
        self.assertEqual(r['velocity'], (0.,0.))
        self.assertEqual(r['xy'], (2.,0.))
        self.assertEqual(r['reason'], 'SOURCE_DISAGREEMENT')

    def test_alignment_uses_original_time_and_rejects_replay(self):
        f = SourceAlignedFusion()
        f.observe('a', 1., (0.,0.), .9, '1')
        f.observe('a', 1.2, (.4,0.), .9, '2')
        r = f.observe('b', 1.4, (.8,0.), .9, '3')
        self.assertAlmostEqual(r['sources'][0]['aligned_xy'][0], .54)
        self.assertEqual(r['sources'][0]['original_s'], 1.2)
        self.assertIsNone(f.observe('a', 1.2, (9.,0.), .9, '4'))
        self.assertIsNone(f.observe('b', 1.5, (9.,0.), .9, '3'))
        r = f.observe('a', 3., (9.,0.), .9, '5')
        self.assertEqual(r['velocity'], (0.,0.))

    def test_other_colors_and_legacy_calls_unchanged(self):
        for tag in ('green','blue','white','red1','red2','brown'):
            old, new = TargetBridgeCore(), TargetBridgeCore(brown_alignment=True)
            for i in range(20):
                args=(10.+i*.2,tag,i*.15,2.,.95)
                old.report(*args)
                new.report(*args)
                self.assertEqual(old.tick(args[0]+.1),new.tick(args[0]+.1))

    def tracker(self):
        r=WhiteReacquisition();r.reset(2)
        self.assertTrue(r.observe(2,10.,(8.,0.),10.))
        return r

    def test_bounded_scan_and_late_frame_cannot_revive_failure(self):
        r=self.tracker()
        self.assertEqual(r.step(2,11.,(0.,0.))[0],'IDLE')
        for t, angle in ((11.1,0.),(12.1,.25),(13.1,-.25)):
            state,point=r.step(2,t,(0.,0.))
            self.assertEqual(state,'SCANNING')
            self.assertAlmostEqual(math.atan2(point[1],point[0]),angle)
        self.assertFalse(r.observe(2,14.1,(8.,0.),14.1))
        self.assertEqual(r.step(2,14.1,(0.,0.))[0],'FAILED')
        self.assertFalse(r.observe(2,15.,(8.,0.),15.))

    def test_fresh_own_image_resumes_once_without_refreshing_old_frame(self):
        r=self.tracker();r.step(2,11.1,(0.,0.))
        self.assertFalse(r.observe(2,10.,(8.,0.),11.2))
        self.assertFalse(r.observe(1,11.2,(8.,0.),11.2))
        self.assertTrue(r.observe(2,11.2,(9.,0.),11.3))
        self.assertEqual(r.step(2,11.3,(0.,0.)),('RESUMED',(9.,0.)))
        self.assertEqual(r.sample_s,11.2)
        self.assertEqual(r.step(2,13.,(0.,0.))[0],'IDLE')
        r.step(3,14.,(0.,0.))
        self.assertIsNone(r.sample_s)
        self.assertFalse(r.used)

    def test_stop_cancels_and_no_observation_does_not_search_blind(self):
        r=self.tracker();r.step(2,11.1,(0.,0.))
        self.assertEqual(r.step(2,11.2,(0.,0.),False),('IDLE',None))
        self.assertEqual(r.step(2,12.,(0.,0.)),('IDLE',None))

    def test_actual_agent_stop_and_failure_adapter(self):
        tree=ast.parse((SCRIPTS/'swarm_agent.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SwarmAgent')
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_control_white_reacquire')
        ns=dict(TAG_TO_TID=TAG_TO_TID,rospy=SimpleNamespace(loginfo=lambda *a:None))
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'actual_agent','exec'),ns)
        commands=[];feedback=[]
        a=SimpleNamespace(_white_reacquire=self.tracker(),_white_reacquire_phase=None,
            _gate=SimpleNamespace(generation=2,can_move=lambda now:True),world_xy=(0.,0.),
            uav_id='uav_1',_send_vel=lambda *v:commands.append(v),
            _report_blocked_plan=lambda *v:feedback.append(v))
        call=lambda t:ns['_control_white_reacquire'](a,t,TAG_TO_TID['white'],False)
        self.assertTrue(call(11.1));self.assertEqual(commands[-1],(0.,0.))
        self.assertTrue(call(14.2));self.assertIsNone(a._look_at)
        self.assertEqual(feedback[-1],('TARGET_VISUAL_LOST',14.2))

    def test_feedback_still_requires_tracking_six_seconds_and_stopped_motion(self):
        m=dict(schema_version=1,run_id='r',uav_id='u',generation=2,seq=1,sample_s=10.,
            blocked_since_s=4.,reason='TARGET_VISUAL_LOST',position_xy=[0.,0.])
        task={'u':dict(stopping=False,generation=2,task={'task_type':1})}
        motion={'u':dict(sample_s=10.,position_xy=[0.,0.],velocity_xy=[0.,0.])}
        self.assertTrue(accept(m,'r',['u'],task,motion,10.,{}))
        for field,value in [('blocked_since_s',4.1),('generation',1)]:
            bad=dict(m);bad[field]=value
            self.assertFalse(accept(bad,'r',['u'],task,motion,10.,{}))
        task['u']['task']['task_type']=0
        self.assertFalse(accept(m,'r',['u'],task,motion,10.,{}))

    def test_actual_camera_callback_ignores_foreign_and_old_generation_evidence(self):
        import json
        import threading
        from visual_observation import VisualEvidence
        from target_motion import TargetMotion
        tree=ast.parse((SCRIPTS/'swarm_agent.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SwarmAgent')
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_confirmed_visual_cb')
        clock=[10.]
        ns=dict(json=json,TAG_TO_TID=TAG_TO_TID,
            TargetState=lambda:SimpleNamespace(header=SimpleNamespace()),
            rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:clock[0]),from_sec=lambda t:t)))
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'actual_callback','exec'),ns)
        r=WhiteReacquisition();r.reset(2)
        a=SimpleNamespace(_authority_lock=threading.RLock(),_white_reacquire_enabled=True,
            _white_reacquire=r,uav_id='uav_1',_visual_evidence=VisualEvidence('r',['uav_1','uav_2']),
            _gate=SimpleNamespace(generation=2,can_move=lambda t:True,task={'task_type':1,'target_id':TAG_TO_TID['white']}),
            _t_seen={},_visual_motion=TargetMotion(),_target_cb=lambda m:None)
        def feed(uid,seq,t):
            msg=dict(schema_version=2,run_id='r',uav_id=uid,seq=seq,sample_s=t,target_id='white',
                frame_id='world_enu',xyz=[8.,0.,1.25],confidence=.9,observation_id='r:%s:%d'%(uid,seq))
            ns['_confirmed_visual_cb'](a,SimpleNamespace(data=json.dumps(msg)))
        feed('uav_2',1,10.);self.assertIsNone(r.sample_s)
        feed('uav_1',1,10.);self.assertEqual(r.sample_s,10.)
        r.step(2,11.1,(0.,0.));clock[0]=11.2
        feed('uav_2',2,11.2);self.assertEqual(r.step(2,11.2,(0.,0.))[0],'SCANNING')
        feed('uav_1',2,10.);self.assertEqual(r.sample_s,10.)
        # A newer other-camera global cache must not hide our valid own image.
        a._t_seen[TAG_TO_TID['white']]=11.2
        feed('uav_1',3,11.1);self.assertEqual(r.step(2,11.2,(0.,0.))[0],'RESUMED')
        r.reset(3);a._gate.generation=3
        feed('uav_1',3,11.1);self.assertIsNone(r.sample_s)

    def test_new_generation_requires_image_acquired_after_grant(self):
        r=WhiteReacquisition();r.reset(3,10.5)
        self.assertFalse(r.observe(3,10.4,(8.,0.),10.6))
        self.assertTrue(r.observe(3,10.5,(8.,0.),10.6))

    def test_other_colors_unchanged_with_sources(self):
        for tag in ('green','blue','white','red1','red2'):
            old,new=TargetBridgeCore(),TargetBridgeCore(brown_alignment=True)
            for i in range(30):
                args=(10.+i*.2,tag,.2*i,1.,.95,'u%d'%(i%2),str(i))
                old.report(*args);new.report(*args)
                self.assertEqual(old.tick(args[0]+.1),new.tick(args[0]+.1))
