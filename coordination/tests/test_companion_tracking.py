import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

SCRIPTS=Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'
sys.path.insert(0,str(SCRIPTS))
from companion_tracking import companion_guidance
from own_visual_guidance import tracking_input


class CompanionTrackingTests(unittest.TestCase):
    def test_stationary_person_at_observation_distance_does_not_orbit(self):
        result=companion_guidance((0.,0.),(10.,0.),(0.,0.),.2)
        self.assertEqual(result['goal'],(0.,0.))
        self.assertEqual(result['speed_mps'],0.)
        self.assertEqual(result['look_at'],(10.,0.))

    def test_two_meter_target_has_long_plan_and_two_meter_following(self):
        result=companion_guidance((0.,0.),(10.,0.),(2.,0.),0.)
        self.assertEqual(result['goal'],(9.,0.))
        self.assertEqual(result['speed_mps'],2.)

    def test_navigation_projection_compensates_actual_image_age(self):
        result=companion_guidance((0.,0.),(9.4,0.),(2.,0.),.3)
        self.assertAlmostEqual(result['look_at'][0],10.)
        self.assertAlmostEqual(result['speed_mps'],2.)

    def test_distant_target_requests_existing_chase_cap(self):
        result=companion_guidance((0.,0.),(25.,0.),(2.,0.),0.)
        self.assertEqual(result['speed_mps'],2.6)

    def test_close_stationary_target_requests_retreat(self):
        result=companion_guidance((0.,0.),(5.,0.),(0.,0.),0.)
        self.assertEqual(result['goal'],(-5.,0.))

    def test_front_view_does_not_cross_the_person_to_force_behind_view(self):
        result=companion_guidance((20.,0.),(10.,0.),(2.,0.),0.)
        self.assertEqual(result['goal'],(29.,0.))

    def test_stale_future_nonfinite_and_coincident_inputs_rejected(self):
        for age in (-.1,1.51,float('nan')):
            self.assertIsNone(companion_guidance((0.,0.),(10.,0.),(2.,0.),age))
        self.assertIsNone(companion_guidance((0.,0.),(float('inf'),0.),(2.,0.),0.))
        self.assertIsNone(companion_guidance((0.,0.),(0.,0.),(0.,0.),0.))

    def test_actual_agent_requests_plan_and_preserves_friend_and_final_gates(self):
        source=(SCRIPTS/'swarm_agent.py').read_text(encoding='utf-8')
        node=next(n for n in ast.walk(ast.parse(source)) if isinstance(n,ast.FunctionDef)
                  and n.name=='_fly_companion')
        scope=dict(companion_guidance=companion_guidance,tracking_input=tracking_input,POS_KP=.8,MAX_SPEED=3.,FLEE_CHASE_SPEED=2.6,math=math)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),
                     'swarm_agent.py','exec'),scope)
        requests,commands,friends=[],[],[]
        agent=NS(targets={'t0':(10.,0.,2.,0.)},_t_seen={'t0':100.},world_xy=(0.,0.),
                 _gate=NS(can_move=lambda now:True),_plan_fail_t=0.,
                 _need_replan_track=lambda point:True,_request_plan=lambda point:requests.append(point),
                 _pick_local_goal=lambda:(4.,0.),_send_vel=lambda x,y:commands.append((x,y)))
        def friend(vx,vy):
            friends.append((vx,vy))
            return vx*.5,vy*.5
        agent._apply_friend_avoidance=friend
        scope['_fly_companion'](agent,100.,'t0')
        self.assertEqual(requests,[(9.,0.)])
        self.assertEqual(friends,[(2.,0.)])
        self.assertEqual(commands,[(1.,0.)])
        self.assertEqual(agent._t_seen['t0'],100.)
        agent._gate.can_move=lambda now:False
        scope['_fly_companion'](agent,100.1,'t0')
        self.assertEqual(commands[-1],(0.,0.))


if __name__=='__main__':
    unittest.main()
