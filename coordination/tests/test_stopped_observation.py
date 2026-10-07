import ast
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest

SCRIPTS=Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'
sys.path.insert(0,str(SCRIPTS))
from tracking_navigation import StoppedObservation
from visual_observation import VisualEvidence, TAG_TO_TID
from position_brake import PositionBrake


class StoppedObservationTests(unittest.TestCase):
    def test_foreign_old_future_invalid_and_duplicate_images_do_not_refresh(self):
        view=StoppedObservation('uav_5')
        self.assertTrue(view.observe('uav_5','t1',100.,(3.,4.),100.2))
        for uid,stamp,point in [('uav_4',100.5,(5.,6.)),('uav_5',100.,(5.,6.)),
                                ('uav_5',99.,(5.,6.)),('uav_5',101.,(5.,6.)),
                                ('uav_5',100.1,(math.nan,6.))]:
            self.assertFalse(view.observe(uid,'t1',stamp,point,100.5))
        self.assertEqual(view.images['t1'],(100.,(3.,4.)))
        self.assertEqual(view.look(108.,True,1),(3.,4.))
        self.assertIsNone(view.look(108.01,True,1))

    def test_only_real_new_own_image_extends_observation(self):
        view=StoppedObservation('uav_5')
        view.observe('uav_5','t1',100.,(3.,4.),100.2)
        self.assertEqual(view.look(107.9,True,1),(3.,4.))
        view.observe('uav_5','t1',108.,(4.,4.),108.2)
        self.assertEqual(view.look(110.284,True,1),(4.,4.))
        self.assertEqual(view.images['t1'][0],108.)

    def test_stopped_view_stays_on_same_person_and_resets_after_handoff(self):
        view=StoppedObservation('uav_5')
        view.observe('uav_5','t1',100.,(3.,4.),100.2)
        self.assertEqual(view.look(100.3,True,1),(3.,4.))
        view.observe('uav_5','t3',100.4,(9.,9.),100.6)
        self.assertEqual(view.look(100.7,True,1),(3.,4.))
        self.assertIsNone(view.look(100.8,False,1))
        self.assertEqual(view.look(100.9,True,2),(9.,9.))

    def test_no_stop_bad_pose_eliminated_and_backwards_time_disable_view(self):
        view=StoppedObservation('uav_5')
        view.observe('uav_5','t1',100.,(3.,4.),100.2)
        self.assertIsNone(view.look(100.3,False,1))
        self.assertIsNone(view.look(100.3,True,1,eligible=False))
        self.assertIsNone(view.look(100.3,True,1,blocked=('t1',)))
        self.assertIsNone(view.look(99.,True,1))

    def methods(self,now):
        source=(SCRIPTS/'swarm_agent.py').read_text()
        nodes=[n for n in ast.walk(ast.parse(source)) if isinstance(n,ast.FunctionDef)
               and n.name in ('_control','_confirmed_visual_cb')]
        scope=dict(math=math,json=json,TAG_TO_TID=TAG_TO_TID,
                   rospy=NS(loginfo=lambda *a:None,Time=NS(now=lambda:NS(to_sec=lambda:now[0]),
                                     from_sec=lambda s:NS(to_sec=lambda:s))),
                   TargetState=lambda:NS(header=NS()))
        exec(compile(ast.Module(body=nodes,type_ignores=[]),'actual_agent_methods','exec'),scope)
        return scope

    def test_actual_accepted_own_callback_and_stop_control_keep_yaw_request_zero_xyz(self):
        now=[1967.864];methods=self.methods(now)
        commands=[];view=StoppedObservation('uav_5')
        agent=NS(_authority_lock=threading.RLock(),_visual_evidence=VisualEvidence('run',['uav_4','uav_5']),
                 _stopped_observation=view,_t_seen={},_visual_motion=NS(observe=lambda *a:(0.,0.,False)),
                 _target_cb=lambda m:None,uav_id='uav_5',_gate=NS(task={'task_type':0},
                     stopping=True,generation=1,can_move=lambda t:False),
                 _giveup_until={},world_xy=(0.,0.),_takeoff_done=True,_landing=False,
                 _send_vel=lambda x,y:commands.append((x,y)))
        observation=dict(schema_version=2,run_id='run',uav_id='uav_5',seq=1,
            sample_s=1967.388,target_id='blue',frame_id='world_enu',xyz=[-26.77,13.35,1.25],
            confidence=.9,observation_id='run:uav_5:1')
        methods['_confirmed_visual_cb'](agent,NS(data=json.dumps(observation)))
        now[0]=1972.
        methods['_control'](agent)
        self.assertEqual(commands[-1],(0.,0.))
        self.assertEqual(agent._look_at,(-26.77,13.35))
        self.assertEqual(view.images['t1'][0],1967.388)
        now[0]=1978.148
        methods['_control'](agent)
        self.assertIsNone(agent._look_at)  # The historical image alone remains expired.
        agent._gate.stopping=False;now[0]=1972.
        methods['_control'](agent)
        self.assertIsNone(agent._look_at)  # Expired authority alone cannot activate it.
        agent._gate.stopping=True;agent._pose_quality=NS(usable=lambda t:False)
        methods['_control'](agent)
        self.assertIsNone(agent._look_at)
        self.assertEqual(commands[-1],(0.,0.))

    def test_actual_publisher_keeps_xyz_anchor_while_yaw_rate_changes(self):
        source=(SCRIPTS/'swarm_agent.py').read_text()
        node=next(n for n in ast.walk(ast.parse(source)) if isinstance(n,ast.FunctionDef)
                  and n.name=='_publish_command')
        def message():
            return NS(header=NS(),position=NS(),velocity=NS())
        scope=dict(rospy=NS(Time=NS(now=lambda:NS(to_sec=lambda:100.))),PositionTarget=message)
        exec(compile(ast.Module(body=[node],type_ignores=[]),'actual_publisher','exec'),scope)
        outputs=[]
        agent=NS(_position_brake=PositionBrake(),local_xy=(1.,2.),local_z=2.2,
            _pose_sample_s=100.,offset=(0.,0.),_xyz_stop_requested=True,
            vel_pub=NS(publish=outputs.append))
        cmd=NS(twist=NS(linear=NS(x=0.,y=0.,z=0.),angular=NS(z=.5)))
        scope['_publish_command'](agent,cmd)
        agent.local_xy=(1.3,2.4);agent.local_z=2.5;cmd.twist.angular.z=-.2
        scope['_publish_command'](agent,cmd)
        for output in outputs:
            self.assertEqual((output.position.x,output.position.y,output.position.z),(1.,2.,2.2))
            self.assertEqual((output.velocity.x,output.velocity.y,output.velocity.z),(0.,0.,0.))
            self.assertEqual(output.type_mask,1528)
        self.assertEqual([x.yaw_rate for x in outputs],[.5,-.2])


if __name__=='__main__':
    unittest.main()
