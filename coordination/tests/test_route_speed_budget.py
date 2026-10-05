"""Route-bound speed and actual grant callback; no ROS or simulator required."""
import ast
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).parents[1]
SCRIPTS = ROOT/'src/robocup_swarm/scripts'
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(ROOT/'src/robocup_navigation/src'))
from route_reservation import RouteGate
from online_radar_planner import OnlinePlanner
from robocup_navigation.astar import GridMap


def grant(points, offer=1, seq=1, expires=10., generation=1):
    return dict(schema_version=1, run_id='r', uav_id='u', generation=generation,
                offer_id=offer, points=points, seq=seq, expires_s=expires)


def gate(points):
    g = RouteGate('r', 'u')
    assert g.receive(grant(points), 1., 1, 1)
    return g


class SpeedBudgetTests(unittest.TestCase):
    def test_long_straight_route_keeps_three_meters_per_second(self):
        g = gate([[0., 0.], [30., 0.]])
        r = g.limited_command((0., 0.), (3., 0.), [(2., 0.), (.2, 0.)], 1.1, 1)
        self.assertEqual(r['velocity_xy'], (3., 0.))
        self.assertEqual(r['reason'], 'ROUTE_CLEAR')

    def test_short_endpoint_scales_request_instead_of_all_or_nothing(self):
        g = gate([[0., 0.], [2.5, 0.]])
        self.assertFalse(g.command_clear((0., 0.), (3., 0.), (0., 0.), 1.1, 1))
        r = g.limited_command((0., 0.), (3., 0.), [(0., 0.)], 1.1, 1)
        self.assertGreater(r['velocity_xy'][0], 1.)
        self.assertLess(r['velocity_xy'][0], 3.)
        self.assertEqual(r['reason'], 'ROUTE_REQUEST_SCALED')
        self.assertTrue(g.command_clear((0., 0.), r['velocity_xy'], (0., 0.), 1.1, 1))

    def test_corner_reduces_diagonal_request_inside_same_route(self):
        g = gate([[0., 0.], [4., 0.], [4., 8.]])
        desired = (2.6/math.sqrt(2), 2.6/math.sqrt(2))
        r = g.limited_command((2., 0.), desired, [(1., 0.), (.2, 0.)], 1.1, 1)
        self.assertGreater(math.hypot(*r['velocity_xy']), .05)
        self.assertLess(math.hypot(*r['velocity_xy']), 2.6)
        self.assertAlmostEqual(r['velocity_xy'][0], r['velocity_xy'][1])
        self.assertTrue(g.command_clear((2., 0.), r['velocity_xy'], (1., 0.), 1.1, 1))

    def test_measured_overshoot_cannot_be_repaired_by_scaling_request(self):
        g = gate([[0., 0.], [4., 0.], [4., 8.]])
        r = g.limited_command((3.8, 0.), (0., 2.), [(2., 0.), (.1, 0.)], 1.1, 1)
        self.assertEqual(r['velocity_xy'], (0., 0.))
        self.assertEqual(r['reason'], 'ROUTE_MEASURED_ENVELOPE')

    def test_opposing_measured_vectors_are_not_averaged_away(self):
        g = gate([[0., 0.], [30., 0.]])
        r = g.limited_command((.2, 0.), (2., 0.), [(1., 0.), (-1., 0.)], 1.1, 1)
        self.assertEqual(r['velocity_xy'], (0., 0.))
        self.assertEqual(r['last_check']['velocity_xy'], [-1., 0.])

    def test_expired_old_generation_missing_motion_and_unknown_route_stop(self):
        for now, gen, measured in [(11., 1, [(0., 0.)]), (1.1, 2, [(0., 0.)]), (1.1, 1, [])]:
            g = gate([[0., 0.], [30., 0.]])
            self.assertEqual(g.limited_command((0., 0.), (2., 0.), measured, now, gen)['velocity_xy'], (0., 0.))
        g = RouteGate('r', 'u')
        self.assertEqual(g.limited_command((0., 0.), (2., 0.), [(0., 0.)], 1., 1)['reason'], 'ROUTE_MISSING')

    def test_nonfinite_request_or_time_never_selects_motion(self):
        for request, now in [((math.nan, 0.), 1.1), ((2., 0.), math.nan), ((2., 0.), .9)]:
            g = gate([[0., 0.], [30., 0.]])
            result = g.limited_command((0., 0.), request, [(0., 0.)], now, 1)
            self.assertEqual(result['velocity_xy'], (0., 0.))
            json.dumps(result, allow_nan=False)

    def test_guide_shortcut_cannot_cross_unreserved_inside_of_turn(self):
        g = gate([[0., 0.], [6., 0.], [6., 6.]])
        self.assertFalse(g.connector_clear((0., 0.), (6., 6.), 1.1, 1))
        self.assertTrue(g.connector_clear((0., 0.), (6., 0.), 1.1, 1))

    def test_exact_segment_cache_renews_and_replaces_without_expanding_geometry(self):
        points = [[i*.25,0.] for i in range(25)]+[[6.,i*.25] for i in range(25)]
        g = gate(points)
        self.assertEqual(len(g._segments),2)
        self.assertEqual(g.record['points'],points)
        self.assertFalse(g.connector_clear((0.,0.),(6.,6.),1.1,1))
        self.assertTrue(g.receive(grant(points,seq=2),1.2,1,1))
        self.assertEqual(len(g._segments),2)
        new=[[0.,0.],[-10.,0.]]
        self.assertTrue(g.receive(grant(new,offer=2,seq=3),1.3,1,2))
        self.assertFalse(g.connector_clear((0.,0.),(6.,0.),1.3,1))
        self.assertTrue(g.connector_clear((0.,0.),(-6.,0.),1.3,1))


class ActualCallbackTests(unittest.TestCase):
    def setUp(self):
        self.now = 1.1
        self.can_move = True
        ros = SimpleNamespace(Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:self.now)),
                              loginfo=lambda *a:None)
        scope = dict(math=math, json=json, rospy=ros, LOOKAHEAD=3.)
        tree = ast.parse((SCRIPTS/'swarm_agent.py').read_text(encoding='utf-8'))
        names = {'_route_grant_cb', '_pick_local_goal', '_route_guard_velocity'}
        methods = [n for c in tree.body if isinstance(c, ast.ClassDef) and c.name=='SwarmAgent'
                   for n in c.body if isinstance(n, ast.FunctionDef) and n.name in names]
        exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])),str(SCRIPTS/'swarm_agent.py'),'exec'),scope)
        self.scope = scope
        self.old = [[0., 0.], [10., 0.]]
        self.new = [[0., 0.], [0., 10.]]
        self.a = SimpleNamespace(uav_id='u',_authority_lock=threading.RLock(),
            _gate=SimpleNamespace(generation=1, can_move=lambda t:self.can_move),
            _route_gate=gate(self.old), _route_offer_id=1, _plan_ticket=2,
            _route_pending=(2,self.new,(0.,10.),1), path=self.old, path_target=(10.,0.))

    def receive(self, message):
        self.scope['_route_grant_cb'](self.a, SimpleNamespace(data=json.dumps(message)))

    def test_pending_plan_keeps_current_grant_renewed_then_commits_new_offer(self):
        self.receive(grant(self.old, seq=2, expires=2.3))
        self.assertEqual(self.a._route_gate.record['expires_s'], 2.3)
        self.assertIsNotNone(self.a._route_pending)
        self.now = 1.2
        self.receive(grant(self.new, offer=2, seq=3, expires=2.4))
        self.assertIsNone(self.a._route_pending)
        self.assertEqual(self.a.path, self.new)
        self.assertEqual(self.a._route_offer_id, 2)
        self.receive(grant(self.old, offer=1, seq=4, expires=3.))
        self.assertEqual(self.a.path, self.new)
        self.assertEqual(self.a._route_gate.record['offer_id'], 2)

    def test_pending_does_not_allow_stopped_old_generation_or_changed_old_polyline(self):
        for message, can_move in [(grant(self.old,seq=2,generation=2),True),
                                   (grant(self.old,seq=2),False),
                                   (grant(self.new,seq=2),True)]:
            self.can_move = can_move
            self.receive(message)
            self.assertEqual(self.a._route_gate.seq, 1)
            self.assertIsNotNone(self.a._route_pending)

    def test_actual_guide_uses_map_and_committed_route(self):
        a = self.a
        a.path = [[0.,0.],[6.,0.],[6.,6.]]
        a._route_gate = gate(a.path)
        a.world_xy = (5., 0.)
        a._online_planner = OnlinePlanner((0.,0.))
        a._online_safe_grid = GridMap(40,40,.5,(-10.,-10.),bytes(1600),'map')
        self.assertEqual(self.scope['_pick_local_goal'](a), (6.,0.))

    def test_actual_guard_preserves_both_measured_directions_and_stage_evidence(self):
        a = self.a
        a.world_xy = (.2,0.)
        a._measured_motion = lambda t:dict(velocity_candidates=[(1.,0.),(-1.,0.)])
        result = self.scope['_route_guard_velocity'](a,2.,0.,1.1,stage='before_acceleration')
        self.assertEqual(result,(0.,0.))
        self.assertEqual(a._final_stop_reason,'ROUTE_MEASURED_ENVELOPE')
        self.assertEqual(a._route_velocity_evidence['before_acceleration']['last_check']['velocity_xy'],[-1.,0.])


if __name__ == '__main__':
    unittest.main()
