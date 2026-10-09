"""Exercise the actual command branch for delayed FLEE and stale red slots."""
import ast
import math
import os
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import test_tracking_backoff as backoff
import test_reserved_orbit as orbit
from route_reservation import RouteGate
from online_radar_planner import OnlinePlanner
from robocup_navigation.astar import GridMap


class TargetChaseSpeedTests(unittest.TestCase):
    def setUp(self):
        fixture = backoff.TrackingBackoffTests()
        fixture.setUp()
        self.fixture = fixture
        fixture.scope.update(MAX_SPEED=3., FLEE_CHASE_SPEED=2.6, ORBIT_RADIUS=8.,
                             POS_KP=1., ARRIVE_TOL=1.)
        fixture.grant()
        self.agent = a = fixture.agent
        a._giveup_until.clear()
        a._resume_tracking_attempt = Mock(return_value=False)
        a._need_replan_track = Mock(return_value=False)
        a._pick_local_goal = Mock(return_value=(0., 8.))
        a._apply_friend_avoidance = lambda x, y: (x, y)
        a._target_fleeing = Mock(return_value=False)

    def test_approach_does_not_wait_for_flee_classification(self):
        self.agent._control()
        self.assertEqual(self.agent._send_vel.call_args.args, (0., 2.6))
        self.agent._target_fleeing.assert_not_called()

    def test_task_stop_still_blocks_chase(self):
        self.fixture.grant(seq=2, action='STOP')
        self.agent._control()
        self.assertEqual(self.agent._send_vel.call_args.args, (0., 0.))
        self.agent._pick_local_goal.assert_not_called()

    def test_chase_respects_lower_configured_maximum(self):
        self.fixture.scope['MAX_SPEED'] = 2.
        self.agent._control()
        self.assertEqual(self.agent._send_vel.call_args.args, (0., 2.))

    def test_search_not_slowed_by_stale_nearby_red_slot(self):
        a = self.agent
        a.assignment = SimpleNamespace(task_type=0, target_x=0., target_y=10.)
        a.path_target = (0., 10.)
        a._orbit_target = None
        a._update_orbit = Mock()
        a.targets = {'t5': (0., 1., 0., 0.)}
        a._t_seen = {'t5': 0.}
        a._nearest_target_dist = Mock(return_value=('t5', 1.))
        a._control()
        self.assertEqual(a._send_vel.call_args.args, (0., 3.))
        a._nearest_target_dist.assert_not_called()

    def test_small_tracking_error_is_not_forced_to_chase_speed(self):
        self.agent._pick_local_goal.return_value = (0., .3)
        self.agent._control()
        self.assertEqual(self.agent._send_vel.call_args.args, (0., .3))

    def test_downstream_orbit_limit_does_not_restore_one_point_five_cap(self):
        a = SimpleNamespace(_orbit_target='t5', _last_heading=None,
                            _target_fleeing=Mock(return_value=False))
        limited = orbit.scope['_adaptive_speed'](a, 0., 2.6)
        self.assertEqual(limited, (0., 2.6))
        self.assertGreater(math.hypot(*limited), 2.)
        a._target_fleeing.assert_not_called()


class ActualLookaheadTests(unittest.TestCase):
    def setUp(self):
        self.fixture=TargetChaseSpeedTests()
        self.fixture.setUp()
        self.a=self.fixture.agent
        self.scope=self.fixture.fixture.scope
        self.tree=ast.parse((backoff.scripts/'swarm_agent.py').read_text())

    def constants(self,lookahead):
        names={'POS_KP','LOOKAHEAD','MAX_SPEED','FLEE_CHASE_SPEED'}
        nodes=[n for n in self.tree.body if isinstance(n,ast.Assign)
               and any(isinstance(t,ast.Name) and t.id in names for t in n.targets)]
        nodes += [n for n in self.tree.body if isinstance(n,ast.If)
                  and any(isinstance(t,ast.Name) and t.id=='LOOKAHEAD' for t in ast.walk(n.test))]
        scope=dict(math=math,os=os)
        with patch.dict(os.environ,{'SWARM_MAX_SPEED':'3.0','FLEE_CHASE_SPEED':'2.6'}):
            if lookahead is None:os.environ.pop('SWARM_LOOKAHEAD_M',None)
            else:os.environ['SWARM_LOOKAHEAD_M']=str(lookahead)
            exec(compile(ast.Module(body=nodes,type_ignores=[]),'actual_guide_constants','exec'),scope)
        return {k:scope[k] for k in names}

    def path(self,points):
        self.scope.update(self.constants(4.))
        method=next(n for c in self.tree.body if isinstance(c,ast.ClassDef) and c.name=='SwarmAgent'
                    for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='_pick_local_goal')
        exec(compile(ast.Module(body=[method],type_ignores=[]),'actual_pick_local_goal','exec'),self.scope)
        a=self.a
        a._pick_local_goal=MethodType(self.scope['_pick_local_goal'],a)
        a.path=points
        a._route_gate=RouteGate('run','uav_4')
        self.assertTrue(a._route_gate.receive(dict(schema_version=1,run_id='run',uav_id='uav_4',
            generation=1,offer_id=1,points=points,seq=1,expires_s=30.),20.,1,1))
        a._online_planner=OnlinePlanner((0.,0.))
        a._online_safe_grid=GridMap(40,40,.5,(-10.,-10.),bytes(1600),'map')
        a._last_online_request_s=20.

    def test_real_default_and_invalid_environment(self):
        self.assertEqual(self.constants(None)['LOOKAHEAD'],3.)
        for value in [0,-1,'nan','inf']:
            with self.subTest(value=value),self.assertRaises(ValueError):self.constants(value)

    def test_real_four_meter_guide_reaches_existing_search_and_chase_caps(self):
        self.path([[0.,0.],[0.,2.],[0.,4.],[0.,8.],[0.,20.]])
        self.assertEqual(self.a._pick_local_goal(),(0.,4.))
        self.a._control()
        self.assertEqual(self.a._send_vel.call_args.args,(0.,2.6))
        a=self.a
        a.assignment=SimpleNamespace(task_type=0,target_x=0.,target_y=20.)
        a.path_target=(0.,20.);a._orbit_target=None;a._update_orbit=Mock()
        a._control()
        self.assertEqual(a._send_vel.call_args.args,(0.,3.))

    def test_turn_guide_and_short_reserved_leg_still_request_slower_motion(self):
        self.path([[0.,0.],[0.,2.],[2.,2.],[2.,20.]])
        self.assertEqual(self.a._pick_local_goal(),(0.,2.))
        self.a._control()
        self.assertEqual(self.a._send_vel.call_args.args,(0.,1.6))
        self.path([[0.,0.],[0.,.3]])
        self.a._control()
        self.assertEqual(self.a._send_vel.call_args.args,(0.,.24))

    def test_retained_snapshot_does_not_restart_a_stopped_task(self):
        self.path([[0.,0.],[0.,20.]])
        self.fixture.fixture.grant(seq=2,action='STOP')
        self.a._route_gate.receive_reservations(dict(schema_version=1,run_id='run',seq=1,
            reservations={'uav_4':dict(generation=1,segments=[[[0.,0.],[0.,20.]]])}),20.,1)
        self.a._control()
        self.assertEqual(self.a._send_vel.call_args.args,(0.,0.))
