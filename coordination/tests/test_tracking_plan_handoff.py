"""Exercise the actual asynchronous methods with camera updates during computation."""
import ast
import json
import math
from pathlib import Path
import threading
import traceback
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


SOURCE = Path(__file__).parents[1] / 'src/robocup_swarm/scripts/swarm_agent.py'
TREE = ast.parse(SOURCE.read_text(encoding='utf-8'))
METHODS = [n for cls in TREE.body if isinstance(cls, ast.ClassDef)
           for n in cls.body if isinstance(n, ast.FunctionDef)
           and n.name in ('_request_plan', '_planner_loop', '_route_grant_cb', '_need_replan_track')]


class TrackingPlanHandoffTests(unittest.TestCase):
    def setUp(self):
        self.now = 1963.336
        self.rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda: SimpleNamespace(
            to_sec=lambda: self.now)), is_shutdown=Mock(side_effect=[False, True]), logerr=Mock(), loginfo=Mock())
        self.scope = dict(rospy=self.rospy, json=json, traceback=traceback, math=math,
                          TRACK_REPLAN_MOVE=3., TRACK_REPLAN_SEC=2.,
                          String=lambda **kw: SimpleNamespace(**kw))
        exec(compile(ast.fix_missing_locations(ast.Module(body=METHODS, type_ignores=[])),
                     str(SOURCE), 'exec'), self.scope)
        self.a = SimpleNamespace(_authority_lock=threading.RLock(), _plan_lock=threading.Lock(),
            _plan_event=threading.Event(), _plan_ticket=7, _plan_pending=None, _plan_inflight=None,
            _route_pending=None, _route_pending_s=0., _route_offer_id=0, world_xy=(14., 5.),
            _gate=SimpleNamespace(generation=2, task={'task_type': 1}, run_id='run',
                                  can_move=lambda now: True), uav_id='uav_6',
            _route_offer_pub=Mock(), path=[], path_target=None, _last_flight_v=(0., 0.),
            grid=SimpleNamespace(world_to_cell=lambda point: (0, 0)),
            _route_gate=SimpleNamespace(receive=Mock(return_value=True), clear=Mock()))
        self.a._request_plan = lambda point: self.scope['_request_plan'](self.a, point)

    def run_worker(self, during_compute=lambda: None, ok=True):
        def compute(goal):
            during_compute()
            return [goal], goal, ok
        self.a._compute_plan = compute
        self.scope['_planner_loop'](self.a)

    def test_real_green_points_do_not_cancel_inflight_result_and_manager_can_commit(self):
        a = self.a
        a._request_plan((35.95, -6.61))
        self.run_worker(lambda: a._request_plan((35.63, -6.67)))
        self.assertEqual(a._plan_ticket, 8)
        a._route_offer_pub.publish.assert_called_once()
        offer = json.loads(a._route_offer_pub.publish.call_args.args[0].data)
        self.assertEqual(offer['offer_id'], 8)
        self.assertEqual(a.path, [])  # An offer alone never authorizes movement.
        a._request_plan((35.40, -6.70))
        self.assertEqual(a._plan_ticket, 8)
        self.scope['_route_grant_cb'](a, SimpleNamespace(data=json.dumps(offer)))
        self.assertEqual(a.path_target, (35.95, -6.61))
        a._request_plan((35.40, -6.70))
        self.assertEqual(a._plan_pending, ((35.40, -6.70), 2, 9))

    def test_tracking_pending_not_started_yet_is_preserved(self):
        self.a._request_plan((1., 1.))
        self.a._request_plan((2., 2.))
        self.assertEqual(self.a._plan_pending, ((1., 1.), 2, 8))

    def test_coalesced_updates_do_not_hide_old_route_from_replan_throttle(self):
        a = self.a
        a._request_plan((1., 1.))
        accepted_s = self.now
        self.now += .5
        self.run_worker(lambda: a._request_plan((5., 1.)))
        self.assertEqual(a._last_track_goal, (1., 1.))
        self.assertEqual(a._last_track_plan_t, accepted_s)
        offer = json.loads(a._route_offer_pub.publish.call_args.args[0].data)
        self.scope['_route_grant_cb'](a, SimpleNamespace(data=json.dumps(offer)))
        self.assertTrue(self.scope['_need_replan_track'](a, (5., 1.)))
        self.assertFalse(self.scope['_need_replan_track'](a, (1., 1.)))
        self.now = accepted_s + 2.
        self.assertTrue(self.scope['_need_replan_track'](a, (1., 1.)))

    def test_search_still_supersedes_different_goal(self):
        a = self.a
        a._gate.task = {'task_type': 0}
        a._request_plan((1., 1.))
        self.run_worker(lambda: a._request_plan((2., 2.)))
        a._route_offer_pub.publish.assert_not_called()
        self.assertEqual(a._plan_pending, ((2., 2.), 2, 9))

    def test_new_generation_can_queue_while_old_generation_computes(self):
        a = self.a
        a._request_plan((1., 1.))
        def change():
            a._gate.generation = 3
            a._request_plan((2., 2.))
        self.run_worker(change)
        a._route_offer_pub.publish.assert_not_called()
        self.assertEqual(a._plan_pending, ((2., 2.), 3, 9))
        self.assertTrue(a._plan_event.is_set())

    def test_stop_and_expiry_reject_completed_plan(self):
        for reason in ('STOP', 'EXPIRED'):
            with self.subTest(reason=reason):
                self.setUp()
                a = self.a
                a._request_plan((1., 1.))
                self.run_worker(lambda: setattr(a._gate, 'can_move', lambda now: False))
                a._route_offer_pub.publish.assert_not_called()
                self.assertIsNone(a._plan_inflight)
                self.assertIsNone(a._route_pending)

    def test_unanswered_route_offer_does_not_block_forever(self):
        a = self.a
        a._route_pending = (8, [[0., 0.], [1., 1.]], (1., 1.), 2)
        a._route_pending_s = self.now
        a._request_plan((2., 2.))
        self.assertIsNone(a._plan_pending)
        self.now += 1.
        a._request_plan((2., 2.))
        self.assertIsNotNone(a._plan_pending)

    def test_frontier_endpoint_different_from_requested_actor_does_not_cancel_offer(self):
        a = self.a
        a._request_plan((35.95, -6.61))
        a._compute_plan = lambda goal: ([(20., 3.)], (20., 3.), True)
        self.scope['_planner_loop'](a)
        self.assertEqual(a._route_pending[2], (20., 3.))
        a._request_plan((35.95, -6.61))
        self.assertEqual(a._plan_ticket, 8)
        self.assertIsNone(a._plan_pending)

    def test_failed_computation_releases_inflight_and_latest_point_can_retry(self):
        a = self.a
        a._request_plan((1., 1.))
        self.run_worker(lambda: a._request_plan((2., 2.)), ok=False)
        self.assertIsNone(a._plan_inflight)
        self.assertIsNone(a._route_pending)
        a._request_plan((2., 2.))
        self.assertEqual(a._plan_pending, ((2., 2.), 2, 9))

    def test_handoff_holds_authority_and_plan_locks_until_offer_exists(self):
        a = self.a
        a._request_plan((1., 1.))
        seen = []
        def publish(message):
            seen.append(a._plan_lock.acquire(blocking=False))
            if seen[-1]:
                a._plan_lock.release()
            self.assertIsNotNone(a._route_pending)
            self.assertIsNone(a._plan_inflight)
        a._route_offer_pub.publish.side_effect = publish
        self.run_worker()
        self.assertEqual(seen, [False])


if __name__ == '__main__':
    unittest.main()
