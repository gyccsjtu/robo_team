"""Official elimination and preempted search intent regressions; no ROS runtime."""
import ast
import io
import json
from pathlib import Path
import sys
import threading
from types import MethodType, SimpleNamespace
import unittest

SCRIPTS = Path(__file__).parents[1] / 'src/robocup_swarm/scripts'
sys.path.insert(0, str(SCRIPTS))
from task_authority import TaskAuthority
from search_occupancy import apply_authority_release, apply_intent_cancellation
from red_observations import actor_slot_remaining
from swarm_task import STATE_ASSIGNED, STATE_FREE, STATE_COVERED


def task(tid='t5', cell=None, xy=(10., 0.)):
    return dict(task_type=1 if cell is None else 0, target_id=tid if cell is None else '',
                cell_ix=-1 if cell is None else cell[0], cell_iy=-1 if cell is None else cell[1],
                target_x=xy[0], target_y=xy[1])


def stop(core, uid, start, xy=(10., 0.)):
    outputs = []
    for i in range(11):
        stamp = start + .1 * i
        outputs.extend(core.ack(dict(schema_version=2, run_id=core.run_id, uav_id=uid,
            generation=core.generation[uid], seq=core.ack_seq.get(uid, 0) + 1,
            sample_s=stamp, xyz=[*xy, 2.2], speed_mps=0., stopped_s=1.,
            status='STOPPED' if i == 10 else 'STATE'), stamp))
    return outputs


def manager_fixture(core, cells=None, tracking=None):
    tree = ast.parse((SCRIPTS / 'swarm_manager.py').read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SwarmManager')
    names = {'_release_finished', '_emit_authority', '_idle_uavs', '_left_cb'}
    scope = dict(json=json, apply_authority_release=apply_authority_release,
        apply_intent_cancellation=apply_intent_cancellation, actor_slot_remaining=actor_slot_remaining,
        STATE_ASSIGNED=STATE_ASSIGNED, parse_actor_list=lambda raw: json.loads(raw),
        String=lambda **kwargs: SimpleNamespace(**kwargs))
    clock = SimpleNamespace(now=core.last_s)
    scope['rospy'] = SimpleNamespace(loginfo=lambda *a: None, logwarn_throttle=lambda *a: None,
        Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: clock.now)))
    functions = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    exec(compile(ast.Module(body=functions, type_ignores=[]), 'swarm_manager.py', 'exec'), scope)
    published = []
    manager = SimpleNamespace(_authority=core, _authority_lock=threading.RLock(),
        _authority_log=io.StringIO(), _authority_event_cursor=0,
        _authority_pub=SimpleNamespace(publish=lambda msg: published.append(json.loads(msg.data))),
        _tracking=tracking or {}, _backup={}, _eliminated=set(),
        tracker=SimpleNamespace(targets={}), grid=SimpleNamespace(cells=cells or {}),
        _active_leases={}, uav_ids=list(core.fleet),
        status={uid: SimpleNamespace(connected=True) for uid in core.fleet},
        _navigation_eligible=lambda uid: True, _left_seen=True, _left_actors=[0, 1, 2, 3, 4, 5],
        _complete_official_run=lambda: None)
    for name in names:
        setattr(manager, name, MethodType(scope[name], manager))
    return manager, clock, published


class EliminationTests(unittest.TestCase):
    def setUp(self):
        self.core = TaskAuthority('run', ['uav_1', 'uav_2'])

    def test_eliminated_active_target_cannot_refresh_retry_or_be_offered_again(self):
        self.core.offer('uav_1', task(), 2180.)
        outputs = self.core.eliminate_target('t5', 2184.636)
        self.assertEqual(outputs[0]['action'], 'STOP')
        self.assertEqual(self.core.refresh('uav_1', 2201.840), [])
        self.assertEqual(stop(self.core, 'uav_1', 2201.928), [])
        self.assertEqual(self.core.offer('uav_2', task(), 2203.), [])
        self.assertEqual(self.core.tick(2300.), [])
        self.assertIn(('target', 't5'), self.core.locks)
        self.assertTrue(self.core.locks[('target', 't5')]['retired'])
        self.assertEqual(sum(e['event'] == 'TASK_GRANTED' for e in self.core.events), 1)

    def test_elimination_cancels_pending_target_without_freeing_active_search(self):
        self.core.offer('uav_1', task(cell=(15, 5)), 1.)
        self.core.offer('uav_1', task(), 2.)
        self.core.eliminate_target('t5', 2.1)
        self.assertNotIn('uav_1', self.core.pending)
        self.assertIn(('search', 15, 5), self.core.locks)
        self.assertEqual(stop(self.core, 'uav_1', 2.2), [])
        self.assertTrue(self.core.locks[('search', 15, 5)]['retired'])
        self.assertEqual(self.core.events[2]['event'], 'TASK_INTENT_CANCELLED')

    def test_active_elimination_preserves_valid_pending_search(self):
        self.core.offer('uav_1', task(), 1.)
        self.core.offer('uav_1', task(cell=(17, 13)), 2.)
        self.core.eliminate_target('t5', 2.1)
        grants = stop(self.core, 'uav_1', 2.2)
        self.assertEqual(grants[0]['task']['task_type'], 0)
        self.assertIn(('target', 't5'), self.core.locks)

    def test_elimination_preserves_other_target_and_rejects_bad_id_or_time(self):
        self.core.offer('uav_1', task('t0'), 1.)
        self.core.offer('uav_2', task('t5'), 1.1)
        self.core.eliminate_target('t5', 1.2)
        self.assertFalse(self.core.active['uav_1']['stopping'])
        for tid in ('', None, 4):
            with self.assertRaises(ValueError):
                self.core.eliminate_target(tid, 1.3)
        with self.assertRaises(ValueError):
            self.core.eliminate_target('t0', 1.)
        self.assertNotIn('t0', self.core.eliminated_targets)

    def test_cancel_events_cover_replace_withdraw_and_close_but_not_same_key_update(self):
        self.core.offer('uav_1', task(cell=(1, 1)), 0.)
        self.core.offer('uav_1', task(cell=(2, 2)), .1)
        self.core.offer('uav_1', task(cell=(2, 2), xy=(12., 0.)), .2)
        self.assertFalse(any(e['event'] == 'TASK_INTENT_CANCELLED' for e in self.core.events))
        self.core.offer('uav_1', task(), .3)
        self.core.withdraw('uav_1', .4)
        self.core.offer('uav_1', task('t0'), .5)
        self.core.close(.6)
        cancellations = [e for e in self.core.events if e['event'] == 'TASK_INTENT_CANCELLED']
        self.assertEqual([e['details']['reason'] for e in cancellations],
                         ['INTENT_REPLACED', 'TASK_WITHDRAWN', 'RUN_CLOSED'])
        self.assertIn(('search', 1, 1), self.core.locks)

    def test_real_manager_fences_stale_authority_even_without_business_tracking(self):
        self.core.offer('uav_2', task(), 2180.)
        manager, clock, published = manager_fixture(self.core)
        clock.now = 2184.636
        manager._left_cb(SimpleNamespace(data='[0, 1, 2, 3]'))
        self.assertEqual(published[-1]['action'], 'STOP')
        self.assertIn('t5', self.core.eliminated_targets)
        self.assertIn('t4', self.core.eliminated_targets)
        clock.now += .1
        manager._release_finished([0, 1, 2, 3])
        self.assertEqual(sum(e['event'] == 'STOP_REQUESTED' for e in self.core.events), 1)

    def test_real_manager_keeps_both_internal_red_slots_while_either_red_remains(self):
        self.core.offer('uav_1', task('t4'), 1.)
        self.core.offer('uav_2', task('t5'), 1.1)
        manager, clock, published = manager_fixture(self.core, tracking={'t4': 'uav_1', 't5': 'uav_2'})
        clock.now = 1.2
        manager._release_finished([0, 1, 2, 3, 5])
        self.assertEqual(self.core.eliminated_targets, set())
        self.assertEqual(published, [])
        manager.tracker.targets['t5'] = SimpleNamespace(eliminated=False)
        clock.now = 1.3
        manager._release_finished([0, 1, 2, 3])
        self.assertEqual(self.core.eliminated_targets, {'t4', 't5'})
        self.assertEqual(manager._tracking, {})
        self.assertTrue(manager.tracker.targets['t5'].eliminated)

    def test_real_manager_ignores_initial_empty_actor_list(self):
        self.core.offer('uav_1', task(), 1.)
        manager, _, published = manager_fixture(self.core)
        manager._left_seen = False
        manager._left_cb(SimpleNamespace(data='[]'))
        self.assertEqual(published, [])
        self.assertFalse(self.core.active['uav_1']['stopping'])

    def test_actual_search_replacement_then_elimination_sequence_recovers_auction(self):
        old = (15, 5)
        abandoned = (17, 13)
        self.core.offer('uav_2', task(cell=old, xy=(58.5, -21.5)), 2153.528)
        cells = {key: SimpleNamespace(state=STATE_ASSIGNED, owner='uav_2', lease_until=9999.)
                 for key in (old, abandoned)}
        manager, clock, published = manager_fixture(self.core, cells)
        manager._active_leases['uav_2'] = abandoned
        self.core.offer('uav_2', task(cell=abandoned, xy=(72.5, 34.5)), 2161.448)
        manager._emit_authority(self.core.offer('uav_2', task(), 2163.204))
        self.assertEqual(cells[abandoned].state, STATE_FREE)
        self.assertEqual(cells[old].state, STATE_ASSIGNED)
        self.assertNotIn('uav_2', manager._active_leases)
        manager._emit_authority(stop(self.core, 'uav_2', 2163.980, xy=(58.5, -21.5)))
        self.core.ack(dict(schema_version=2, run_id='run', uav_id='uav_2', seq=12,
            generation=self.core.generation['uav_2'], sample_s=2165.032,
            xyz=[64., -21.5, 2.2], speed_mps=0., stopped_s=0., status='STATE'), 2165.032)
        manager._emit_authority([])
        self.assertEqual(cells[old].state, STATE_FREE)
        manager._tracking['t5'] = 'uav_2'
        clock.now = 2184.636
        manager._release_finished([0, 1, 2, 3])
        self.assertIn('uav_2', manager._idle_uavs())
        manager._emit_authority(self.core.offer('uav_2', task(cell=abandoned), 2184.7))
        manager._emit_authority(stop(self.core, 'uav_2', 2184.8))
        self.assertEqual(self.core.active['uav_2']['key'], ('search', 17, 13))
        self.assertIn(('target', 't5'), self.core.locks)
        self.assertTrue(self.core.locks[('target', 't5')]['retired'])

    def test_real_idle_allows_exit_task_while_retired_search_cell_remains_occupied(self):
        self.core.offer('uav_1', task(cell=(15, 5)), 1.)
        self.core.offer('uav_1', task(), 2.)
        self.core.eliminate_target('t5', 2.1)
        stop(self.core, 'uav_1', 2.2)
        cell = SimpleNamespace(state=STATE_ASSIGNED, owner='uav_1', lease_until=9999.)
        manager, _, _ = manager_fixture(self.core, {(15, 5): cell})
        self.assertIn('uav_1', manager._idle_uavs())
        self.assertEqual(cell.state, STATE_ASSIGNED)
        self.core.offer('uav_1', task(cell=(15, 5)), 3.3)
        self.assertNotIn('uav_1', manager._idle_uavs())


class CancelClaimTests(unittest.TestCase):
    def event(self, **changes):
        result = dict(schema_version=2, run_id='run', event='TASK_INTENT_CANCELLED',
            uav_id='uav_1', details=dict(key=['search', 17, 13], reason='INTENT_REPLACED'))
        result.update(changes)
        return result

    def test_replaced_unexecuted_claim_returns_free_without_coverage(self):
        cell = SimpleNamespace(state=STATE_ASSIGNED, owner='uav_1', lease_until=9999.)
        grid = SimpleNamespace(cells={(17, 13): cell})
        self.assertTrue(apply_intent_cancellation(grid, self.event(), 'run', {}, {}))
        self.assertEqual((cell.state, cell.owner, cell.lease_until), (STATE_FREE, None, 0.))

    def test_cancel_never_clears_active_retired_reacquired_or_other_owner_claim(self):
        for state, owner, locks, pending in (
                (STATE_ASSIGNED, 'uav_1', {('search', 17, 13): dict(retired=False)}, {}),
                (STATE_ASSIGNED, 'uav_1', {('search', 17, 13): dict(retired=True)}, {}),
                (STATE_ASSIGNED, 'uav_1', {}, {'uav_2': task(cell=(17, 13))}),
                (STATE_ASSIGNED, 'uav_1', {}, {'uav_1': task(cell=(17, 13))}),
                (STATE_ASSIGNED, 'uav_2', {}, {}), (STATE_COVERED, 'uav_1', {}, {}),
                (STATE_ASSIGNED, 'uav_1', {}, {'uav_1': {}})):
            with self.subTest(state=state, owner=owner, locks=locks, pending=pending):
                cell = SimpleNamespace(state=state, owner=owner, lease_until=9999.)
                grid = SimpleNamespace(cells={(17, 13): cell})
                self.assertFalse(apply_intent_cancellation(grid, self.event(), 'run', locks, pending))
                self.assertEqual((cell.state, cell.owner, cell.lease_until), (state, owner, 9999.))

    def test_cancel_rejects_old_run_wrong_event_and_malformed_key(self):
        cell = SimpleNamespace(state=STATE_ASSIGNED, owner='uav_1', lease_until=9999.)
        grid = SimpleNamespace(cells={(17, 13): cell})
        for event in (self.event(run_id='old'), self.event(event='TASK_RETIRED'),
                      self.event(details=None), self.event(details=dict(key=['search', True, 13])),
                      self.event(details=dict(key=['target', 't5'])), self.event(schema_version=1)):
            self.assertFalse(apply_intent_cancellation(grid, event, 'run', {}, {}))
        self.assertEqual(cell.state, STATE_ASSIGNED)
