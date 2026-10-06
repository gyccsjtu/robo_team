"""Actual generated orbit endpoint -> actual guide -> existing braking gate."""
import ast
import math
import os
from pathlib import Path
import sys
from types import MethodType
import unittest
from unittest.mock import Mock, patch

scripts = Path(__file__).parents[1]/'src/robocup_swarm/scripts'
sys.path.insert(0, str(scripts))
from orbit_geometry import orbit_goal
import test_reserved_orbit as legacy
import test_target_chase_speed as chase


class OrbitGeometryTests(unittest.TestCase):
    def test_default_endpoint_is_the_existing_point_two_radian_step(self):
        for position in [(8., 0.), (3., 4.), (0., 0.)]:
            angle = math.atan2(position[1], position[0])+.2
            self.assertEqual(orbit_goal(position, (0., 0.), 8.),
                             (8.*math.cos(angle), 8.*math.sin(angle)))

    def test_long_endpoint_stays_on_ring_at_requested_distance_near_ring(self):
        for position in [(8., 0.), (7.7, .1), (9., 0.), (2., 0.)]:
            goal = orbit_goal(position, (0., 0.), 8., 9.)
            self.assertAlmostEqual(math.hypot(*goal), 8.)
            self.assertAlmostEqual(math.dist(position, goal), 9.)
        for position in [(0., 0.), (.01, 0.), (30., 0.)]:
            goal = orbit_goal(position, (0., 0.), 8., 9.)
            self.assertTrue(all(math.isfinite(v) for v in goal))
            self.assertAlmostEqual(math.hypot(*goal), 8.)

    def test_measured_directions_precede_target_direction_and_both_are_considered(self):
        self.assertLess(orbit_goal((8., 0.), (0., 0.), 8., 9.,
                                  [(0., -1.), (.1, -1.)], (0., 2.))[1], 0.)
        # Raw twist alone favors positive Y. The opposing pose-rate direction
        # must participate; worst angular alignment chooses negative Y here.
        goal = orbit_goal((8., 0.), (0., 0.), 8., 9., [(-1., .1), (0., -2.)])
        self.assertLess(goal[1], 0.)
        self.assertLess(orbit_goal((8., 0.), (0., 0.), 8., 9.,
                                  [(0., .1), (0., 0.)], (0., -2.))[1], 0.)

    def test_invalid_geometry_is_not_a_motion_permission(self):
        for chord in [-1., 17., math.nan, math.inf]:
            with self.subTest(chord=chord), self.assertRaises(ValueError):
                orbit_goal((8., 0.), (0., 0.), 8., chord)
        with self.assertRaises(ValueError):
            orbit_goal((8., 0.), (0., 0.), 8., 9., [(math.nan, 0.)])

    def test_actual_environment_defaults_and_rejects_invalid_chord(self):
        tree = ast.parse((scripts/'swarm_agent.py').read_text())
        names = {'ORBIT_RADIUS', 'ORBIT_PLAN_CHORD'}
        nodes = [n for n in tree.body if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id in names for t in n.targets)]
        nodes += [n for n in tree.body if isinstance(n, ast.If)
                  and any(isinstance(t, ast.Name) and t.id == 'ORBIT_PLAN_CHORD' for t in ast.walk(n.test))]
        with patch.dict(os.environ):
            os.environ.pop('SWARM_ORBIT_PLAN_CHORD_M', None)
            scope = dict(math=math, os=os)
            exec(compile(ast.Module(body=nodes, type_ignores=[]), 'actual_orbit_constants', 'exec'), scope)
            self.assertEqual(scope['ORBIT_PLAN_CHORD'], 0.)
            for value in ['-1', '17', 'nan', 'inf']:
                os.environ['SWARM_ORBIT_PLAN_CHORD_M'] = value
                with self.subTest(value=value), self.assertRaises(ValueError):
                    exec(compile(ast.Module(body=nodes, type_ignores=[]), 'actual_orbit_constants', 'exec'), scope)


class ActualOrbitRouteTests(unittest.TestCase):
    def setUp(self):
        self.guide = chase.ActualLookaheadTests()
        self.guide.setUp()
        self.a = a = self.guide.a
        a.world_xy = (8., 0.)
        a.targets = {'t3': (0., 0., 0., 0.)}
        a._t_seen = {'t3': 19.9}
        a._orbit_target = 't3'
        a._request_plan = Mock()
        a._publish_claim = Mock()
        a._measured_motion = Mock(return_value=dict(velocity_candidates=[(0., 0.), (0., 0.)]))
        self.scope = dict(legacy.scope, orbit_goal=orbit_goal, POS_KP=.8,
                          rospy=self.guide.scope['rospy'])
        exec(compile(ast.Module(body=legacy.methods, type_ignores=[]), str(legacy.source), 'exec'), self.scope)
        a._orbit_stale = MethodType(self.scope['_orbit_stale'], a)
        a._fly_orbit = MethodType(self.scope['_fly_orbit'], a)
        self.guide.path([[8., 0.], [8., 1.]])
        a.path_target = None

    def granted_leg(self, chord, truncate=None):
        a = self.a
        a._orbit_plan_chord_m = chord
        a._fly_orbit()
        a._send_vel.assert_called_with(0., 0.)  # Merely proposing a point cannot fly.
        point = a._request_plan.call_args.args[0]
        a._send_vel.reset_mock()
        if truncate is not None:
            direction = tuple((point[k]-a.world_xy[k])/math.dist(point, a.world_xy) for k in (0, 1))
            point = tuple(a.world_xy[k]+truncate*direction[k] for k in (0, 1))
        # Synthetic observed-free straight route, using the real point and gates.
        self.guide.path([list(a.world_xy), list(point)])
        a.path_target = point
        a._fly_orbit()
        requested = a._send_vel.call_args.args
        evidence = a._measured_motion(20.)
        measured = evidence['velocity_candidates'] if evidence is not None else []
        limited = a._route_gate.limited_command(a.world_xy, requested, measured, 20., a._gate.generation)
        return point, requested, limited

    def test_real_short_leg_cannot_reach_the_fleeing_target_speed(self):
        point, requested, limited = self.granted_leg(0.)
        self.assertAlmostEqual(math.dist(self.a.world_xy, point), 1.5973346663492504)
        self.assertAlmostEqual(math.hypot(*requested), 1.2778677330794004)
        self.assertLess(math.hypot(*limited['velocity_xy']), 2.)

    def test_real_long_leg_guide_and_braking_gate_allow_existing_chase_cap(self):
        point, requested, limited = self.granted_leg(9.)
        self.assertAlmostEqual(math.dist(self.a.world_xy, point), 9.)
        self.assertAlmostEqual(math.dist(self.a.world_xy, self.a._pick_local_goal()), 4.)
        self.assertAlmostEqual(math.hypot(*requested), 2.6)
        self.assertAlmostEqual(math.hypot(*limited['velocity_xy']), 2.6)
        self.assertEqual(limited['reason'], 'ROUTE_CLEAR')

    def test_planner_truncated_leg_still_slows_at_actual_endpoint(self):
        point, requested, limited = self.granted_leg(9., truncate=.3)
        self.assertAlmostEqual(math.dist(self.a.world_xy, point), .3)
        self.assertAlmostEqual(math.hypot(*requested), .24)
        self.assertAlmostEqual(math.hypot(*limited['velocity_xy']), .24)

    def test_task_stop_still_blocks_the_long_orbit_request(self):
        self.granted_leg(9.)
        self.a._request_plan.reset_mock()
        self.guide.fixture.fixture.grant(seq=2, action='STOP')
        self.a._send_vel.reset_mock()
        self.a._control()
        self.a._send_vel.assert_called_once_with(0., 0.)
        self.a._request_plan.assert_not_called()

    def test_missing_measured_motion_cannot_use_long_route_to_move(self):
        self.a._measured_motion.return_value = None
        _, _, limited = self.granted_leg(9.)
        self.assertEqual(limited['velocity_xy'], (0., 0.))
        self.assertEqual(limited['reason'], 'ROUTE_MOTION_EVIDENCE_MISSING')
