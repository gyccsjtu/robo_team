"""Fresh finite camera attention cannot be stolen by blind primary/backup jobs."""
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
import swarm_manager as sm
from robocup_swarm.msg import UavStatus
from search_segments import ATTEND, TRAVEL


class AttendDispatchTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        clock = patch.object(sm.rospy.Time, 'now',
                             side_effect=lambda: sm.rospy.Time.from_sec(self.now))
        clock.start()
        self.addCleanup(clock.stop)
        m = self.m = sm.SwarmManager.__new__(sm.SwarmManager)
        m.uav_ids, m.status, m.last_report = ['u0', 'u1'], {}, {}
        for name in ('_tracking', '_backup', '_release_cooldown',
                     '_visual_observations', '_eligible_observations', '_visual_source'):
            setattr(m, name, {})
        m._sanitize_dispatch_xy = lambda x, y, tid: (x, y)
        m._free_search_lease = Mock()
        m.assign_pub = Mock()
        m.tracker = NS(targets={tid: NS(observers=set(), eliminated=False)
                               for tid in ('t0', 't1', 't3', 't4', 't5')})
        self.status('u0', ATTEND, x=0.)
        self.status('u1', TRAVEL, x=10.)

    def status(self, uid, state, sample=None, x=0., y=0.):
        msg = UavStatus(uav_id=uid, connected=True, x=x, y=y, z=3., search_state=state)
        msg.header.stamp = sm.rospy.Time.from_sec(self.now if sample is None else sample)
        self.m._status_cb(msg)
        return msg

    def costs(self, tid='t0', uid='u0'):
        return self.m._target_assignment_costs({tid: (5., 0.)}, self.now, set(), [uid])[tid]

    def qualified(self, uid='u0', tid='t0', sample=None, **changes):
        view = dict(eligible=True, sample_s=self.now if sample is None else sample,
                    x=5., y=0., hits=4, since_s=self.now-.6, candidate_id='own-person')
        view.update(changes)
        self.m._eligible_observations[(uid, tid)] = view
        return view

    def backup(self):
        m = self.m
        m._tracking = {'t0': 'u1'}
        m._tracking_since = {'t0': self.now-10.}
        m.tracker.targets['t0'].observers = {'u1'}
        m.tracker.needs_backup = Mock(return_value=[('t0', 'view_lost')])
        m._backup_search_capacity = Mock(return_value=True)
        m._camera_view_needs_relief = Mock(return_value=False)
        m._sole_first_pass_zone_searcher = Mock(return_value=False)
        m._get_target_pos = Mock(return_value=(5., 0.))
        m._confirm_timeout_bl = {}
        with patch.object(sm, 'BACKUP_ENABLE', 1):
            m._dispatch_backup()

    def test_actual_050056_attention_survives_green_blind_dispatch(self):
        fixture = json.loads((ROOT / 'tools/fixtures/attend_blind_dispatch_050056.json').read_text())
        self.now = fixture['dispatch']['ros']
        m = self.m
        m.status.clear()
        m.last_report.clear()
        m.uav_ids = list(fixture['status_at_dispatch'])
        for event in fixture['status_at_dispatch'].values():
            row = event['data']
            self.status(row['uav'], row['state'], sample=row['stamp'], x=row['x'], y=row['y'])
        for event in fixture['attention_status']:
            row = event['data']
            self.status(row['uav'], row['state'], sample=row['stamp'], x=row['x'], y=row['y'])
        m._tracking = dict(fixture['existing_primary'])
        m._tracking_since = {'t3': 2071.912}
        m.tracker.targets['t3'].observers = {'typhoon_h480_3'}
        old_task = m.status['typhoon_h480_5'].search_state
        tx, ty = fixture['dispatch']['xy']
        busy = set(m._tracking.values())
        costs = m._target_assignment_costs({'t0': (tx, ty)}, self.now, busy,
                                          m._dispatch_available(self.now, busy))
        self.assertNotIn('typhoon_h480_5', costs['t0'])
        self.assertTrue(costs['t0'])  # other search aircraft can still dispatch
        m._dispatch_target_batch([('t0', tx, ty)], self.now)
        self.assertNotEqual(m._tracking['t0'], 'typhoon_h480_5')
        self.assertEqual(m._tracking['t3'], 'typhoon_h480_3')
        self.assertEqual(m.status['typhoon_h480_5'].search_state, old_task)
        self.assertNotIn('typhoon_h480_5', [c.args[0].uav_id
                                          for c in m.assign_pub.publish.call_args_list])

    def test_actual_same_camera_qualified_red_can_be_formally_assigned(self):
        fixture = json.loads((ROOT / 'tools/fixtures/attend_blind_dispatch_050056.json').read_text())
        view = fixture['qualified_red_tracking_view']['data']
        self.now = fixture['qualified_red_tracking_view']['ros']
        uid = view['source_uav']
        self.m.uav_ids = [uid]
        self.m.status, self.m.last_report = {}, {}
        self.status(uid, ATTEND, x=90.6, y=35.1)
        self.assertTrue(view['geometry_ok'] and view['report_ok'] and view['person_verdict_ok'])
        # Adapt logged qualified view to the existing manager accepted-image
        # contract; the runtime t5 binding comes from that view, not its colour.
        self.qualified(uid, view['target_id'], sample=view['sample_s'],
                       x=view['x'], y=view['y'], hits=view['hits'],
                       candidate_id=view['world_candidate_id'])
        self.m._dispatch_target_batch([(view['target_id'], view['x'], view['y'])], self.now)
        self.assertEqual(self.m._tracking, {view['target_id']: uid})

    def test_other_camera_qualified_image_cannot_exempt_attending_aircraft(self):
        self.qualified('u1')
        self.assertNotIn('u0', self.costs())

    def test_raw_unqualified_same_camera_is_not_an_exemption(self):
        self.m._visual_observations[('u0', 't0')] = dict(
            eligible=False, sample_s=self.now, x=5., y=0., hits=10)
        self.m._visual_source['t0'] = ('u0', self.now, 5., 0.)
        self.assertEqual(self.m._camera_candidates('t0', 5., 0., self.now, set()), [('u0', 5.)])
        self.assertNotIn('u0', self.costs())

    def test_own_qualified_current_image_allows_same_target_dispatch(self):
        self.qualified()
        self.assertIn('u0', self.costs())
        self.assertNotIn('u0', self.costs('t1'))  # another colour/instance cannot borrow it

    def test_own_stale_future_ineligible_image_cannot_exempt(self):
        for change in (dict(sample=self.now-1.01), dict(sample=self.now+.01),
                       dict(eligible=False)):
            with self.subTest(change=change):
                self.qualified(**change)
                self.assertNotIn('u0', self.costs())

    def test_bridge_accepted_sustained_own_distant_view_allows_dispatch(self):
        view = dict(eligible=False, sample_s=self.now-.1, x=5., y=0.,
                    dispatch_hits=3, dispatch_since_s=self.now-.6, candidate_id='far-person')
        self.m._visual_observations[('u0', 't0')] = view
        self.assertIn('u0', self.costs())
        view['dispatch_hits'] = 2
        self.assertNotIn('u0', self.costs())

    def test_attention_state_end_immediately_restores_blind_dispatch(self):
        self.assertNotIn('u0', self.costs())
        self.status('u0', TRAVEL)
        self.assertIn('u0', self.costs())

    def test_fresh_reordered_attention_status_does_not_block(self):
        self.status('u0', TRAVEL, sample=self.now)
        older = self.status('u0', ATTEND, sample=self.now-.1)
        self.assertIs(self.m.status['u0'], older)  # old callback semantics preserved
        self.assertIn('u0', self.costs())

    def test_duplicate_receipt_cannot_extend_old_attention_sample(self):
        original = self.m.status['u0']
        self.now += 1.001
        self.m._status_cb(original)
        self.assertIn('u0', self.costs())

    def test_bad_future_stale_or_missing_status_cannot_block_dispatch(self):
        status = self.m.status['u0']
        for sample in (float('nan'), float('inf'), 'bad', -1., 0., self.now+.01, self.now-1.01):
            with self.subTest(sample=sample):
                status.header = NS(stamp=NS(to_sec=lambda: sample))
                self.m._status_cb(status)
                self.assertIn('u0', self.costs())
        status.header = None
        self.m._status_cb(status)
        self.assertIn('u0', self.costs())
        status.search_state = TRAVEL
        self.assertIn('u0', self.costs())

    def test_existing_primary_is_not_reassigned_or_cancelled(self):
        self.m._tracking = {'t0': 'u0'}
        self.m.tracker.targets['t0'].observers = {'u0'}
        self.m._dispatch_target_batch([('t0', 5., 0.)], self.now)
        self.assertEqual(self.m._tracking, {'t0': 'u0'})
        self.m.assign_pub.publish.assert_not_called()

    def test_new_blind_backup_cannot_take_attention_but_expiry_restores_it(self):
        self.backup()
        self.assertEqual(self.m._tracking, {'t0': 'u1'})
        self.assertEqual(self.m._backup, {})
        self.m.assign_pub.publish.assert_not_called()
        self.now += 1.001
        self.backup()
        self.assertEqual(self.m._backup, {'t0': 'u0'})

    def test_new_same_target_qualified_backup_can_take_attention(self):
        self.qualified()
        self.backup()
        self.assertEqual(self.m._backup, {'t0': 'u0'})
        self.assertEqual(self.m._tracking, {'t0': 'u1'})

    def test_existing_backup_is_never_removed_by_new_attention_guard(self):
        self.m._backup = {'t0': 'u0'}
        self.backup()
        self.assertEqual(self.m._backup, {'t0': 'u0'})
        self.m.assign_pub.publish.assert_not_called()

    def test_clock_reset_does_not_leave_old_attention_highwater_forever(self):
        self.now = 10.
        self.status('u0', ATTEND)
        self.assertNotIn('u0', self.costs())
        self.status('u0', TRAVEL)
        self.assertIn('u0', self.costs())


if __name__ == '__main__':
    unittest.main()
