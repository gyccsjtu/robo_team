"""Spatial switches reallocate tasks without stealing another visible person."""
import json
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'coordination/src/robocup_swarm/scripts'))
import swarm_manager as sm
from cooperative_tracker import CooperativeTracker


class HypothesisDispatchTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        clock = patch.object(sm.rospy.Time, 'now',
                             side_effect=lambda: sm.rospy.Time.from_sec(self.now))
        clock.start()
        self.addCleanup(clock.stop)
        m = self.m = sm.SwarmManager.__new__(sm.SwarmManager)
        m.uav_ids = ['u%d' % i for i in range(6)]
        m.status = {uid: NS(x=-30., y=13., z=3., connected=True) for uid in m.uav_ids}
        m.status['u3'].x, m.status['u3'].y = 100., 19.
        m.status['u5'].x, m.status['u5'].y = 85., 19.
        m.last_report = {uid: sm.rospy.Time(100) for uid in m.uav_ids}
        for name in ('_release_cooldown', '_visual_observations', '_eligible_observations',
                     '_visual_source', '_last_detection_sample', '_visual_candidate',
                     '_last_detect', '_visual_near_seen', '_truth_cache', '_cur_targets',
                     '_tracking_since', '_target_hist', '_confirm_timeout_bl'):
            setattr(m, name, {})
        # 104229 roles: blue's old primary/backup are west; the new blue
        # camera is already primary on green. The one free aircraft is east.
        m._tracking = {'t2': 'u0', 't3': 'u1', 't1': 'u2', 't0': 'u3'}
        m._backup = {'t1': 'u4'}
        m._tracking_since['t1'] = 40.
        m._eliminated, m._blocked_cells = set(), set()
        m._quarantined_visual = lambda *args: False
        m.assign_pub = Mock()
        m._free_search_lease = Mock()
        m._sanitize_dispatch_xy = lambda x, y, tid: (x, y)
        m._update_tracker_position = Mock()
        m.tracker = CooperativeTracker(official_only=True, strict_samples=True)
        for tid, uid in m._tracking.items():
            ct = m.tracker.add_target(tid, 40.)
            ct.set_observers([uid] + ([m._backup[tid]] if tid in m._backup else []))
        m.tracker.report = Mock(wraps=m.tracker.report)
        jobs = {}
        def queue(kind, x, y, now, ttl_s, **kwargs):
            key = (10, 2)
            jobs[(kind, key)] = (now, now+max(1., ttl_s))
            return key
        m.allocator = NS(queue_revisit=Mock(side_effect=queue), revisit_jobs=jobs)
        self.event = dict(switch_s=100., old_x=-26., old_y=13., new_x=102., new_y=19.,
                          source_uav='u3', trial_until_s=125.)

    def send(self, **changes):
        data = dict(target_id='t1', source_uav='u3', sample_s=self.now,
                    x=102., y=19., real_detection=True, confidence=.9,
                    range_m=8., official_view_ok=True, hypothesis_switch=self.event)
        data.update(changes)
        if data['hypothesis_switch'] is None:
            data.pop('hypothesis_switch')
        self.m._visual_source_cb(NS(data=json.dumps(data)))

    def messages(self):
        return [(c.args[0].uav_id, c.args[0].target_id, c.args[0].task_type)
                for c in self.m.assign_pub.publish.call_args_list]

    def test_104229_busy_green_camera_dispatches_free_eastern_aircraft_immediately(self):
        self.send()
        m = self.m
        self.assertEqual(m._tracking, {'t2': 'u0', 't3': 'u1', 't1': 'u5', 't0': 'u3'})
        self.assertNotIn('t1', m._backup)
        self.assertEqual(self.messages(), [('u5', 't1', 1), ('u2', 't1', 255),
                                          ('u4', 't1', 255)])
        self.assertEqual(m._approach_leases['t1']['x'], 102.)
        self.assertEqual(m._tracking_since['t1'], 100.)
        self.assertEqual(m.tracker.report.call_count, 1)
        self.assertEqual(m._eligible_observations[('u3', 't1')]['hits'], 1)

    def test_no_free_aircraft_keeps_closer_old_blue_owner_and_protects_other_primaries(self):
        m = self.m
        m._tracking['t4'] = 'u5'
        m.status['u4'].x = -10.
        self.send()
        self.assertEqual(m._tracking['t1'], 'u4')
        self.assertEqual(m._tracking['t0'], 'u3')
        self.assertEqual(m._tracking['t4'], 'u5')
        self.assertEqual(self.messages(), [('u4', 't1', 1), ('u2', 't1', 255)])
        lease = m._approach_leases['t1']
        self.assertEqual((lease['x'], lease['y']), (102., 19.))
        self.assertGreater(lease['until'], 140.)
        self.assertLessEqual(lease['until'], 190.)

    def test_seeing_backup_borrows_only_after_new_assignment_and_leaves_primary(self):
        m = self.m
        m._backup['t0'] = 'u5'
        m.status['u5'].x = 100.
        m._eligible_observations[('u3', 't0')] = dict(sample_s=99.9, eligible=True)
        self.send(source_uav='u5')  # Later carriers need not be the original camera.
        self.assertEqual(m._tracking['t1'], 'u5')
        self.assertEqual(m._tracking['t0'], 'u3')
        self.assertNotIn('t0', m._backup)
        self.assertEqual(self.messages()[:2], [('u5', 't1', 1), ('u5', 't0', 255)])

    def test_backup_without_fresh_primary_cannot_be_borrowed(self):
        m = self.m
        m._backup['t0'] = 'u5'
        m.status['u5'].x = 100.
        m.status['u4'].x = -10.
        self.send(source_uav='u5')
        self.assertEqual(m._tracking['t1'], 'u4')
        self.assertEqual(m._backup['t0'], 'u5')

    def test_aircraft_safety_and_cooldown_still_apply_to_switch(self):
        for problem in ('stale', 'grounded', 'cooldown', 'disconnected'):
            with self.subTest(problem=problem):
                self.setUp()
                m = self.m
                m.status['u4'].x = -10.
                if problem == 'stale':
                    m.last_report['u5'] = sm.rospy.Time(96.)
                elif problem == 'grounded':
                    m.status['u5'].z = .5
                elif problem == 'cooldown':
                    m._release_cooldown['u5'] = sm.rospy.Time(102.)
                else:
                    m.status['u5'].connected = False
                self.send()
                self.assertEqual(m._tracking['t1'], 'u4')

    def test_no_available_owner_queues_one_revisit_without_stealing_green(self):
        m = self.m
        m._tracking['t4'] = 'u5'
        m.status['u2'].connected = m.status['u4'].connected = False
        self.send()
        self.assertNotIn('t1', m._tracking)
        self.assertEqual(m._tracking['t0'], 'u3')
        self.assertEqual(m.allocator.queue_revisit.call_count, 1)
        self.assertTrue(all(task == 255 for _, _, task in self.messages()))

    def test_new_hypothesis_clears_old_continuity_velocity_and_stale_callbacks(self):
        m = self.m
        ct = m.tracker.targets['t1']
        ct.confirm_since, ct.last_ok_t = 40., 99.9
        ct.fx, ct.fy, ct.vx, ct.vy = -26., 13., 5., 1.
        ct.confirmation_pending = True
        m._target_hist['t1'] = [(-26., 13., 99.)]
        m._eligible_observations[('u2', 't1')] = dict(sample_s=99.9, eligible=True)
        self.send()
        self.assertIsNone(ct.confirm_since)
        self.assertIsNone(ct.last_ok_t)  # Only tracker.step can process the one real report.
        self.assertEqual((ct.vx, ct.vy), (0., 0.))
        self.assertFalse(ct.confirmation_pending)
        self.assertNotIn('t1', m._target_hist)
        self.assertNotIn(('u2', 't1'), m._eligible_observations)
        self.now = 100.1
        self.send(source_uav='u2', sample_s=99.95, x=-26., y=13., hypothesis_switch=None)
        self.send(source_uav='u2', x=-26., y=13., hypothesis_switch=None)
        m._truth_cb(NS(target_id='t1', eliminated=False, x=-26., y=13.,
                       header=NS(stamp=sm.rospy.Time(100.1))))
        self.assertEqual(m._get_target_pos('t1', self.now), (102., 19.))
        self.assertEqual(m.tracker.report.call_count, 1)

    def test_duplicate_or_older_switch_does_not_reassign_or_extend_approach(self):
        self.send()
        lease = dict(self.m._approach_leases['t1'])
        self.send()
        self.now = 100.2
        self.send()
        self.send(hypothesis_switch=dict(self.event, switch_s=99., trial_until_s=124.))
        self.assertEqual(self.m._approach_leases['t1'], lease)
        self.assertEqual(len(self.messages()), 3)
        self.assertEqual(self.m.tracker.report.call_count, 2)

    def test_coordinate_jump_without_switch_does_not_reset_or_reallocate(self):
        self.send(hypothesis_switch=None)
        self.assertEqual(self.m._tracking['t1'], 'u2')
        self.assertEqual(self.m._backup['t1'], 'u4')
        self.assertEqual(self.messages(), [])

    def test_parallel_switch_and_pending_target_never_share_a_primary(self):
        m = self.m
        errors = []
        def run(fn):
            try:
                fn()
            except Exception as exc:
                errors.append(exc)
        workers = [threading.Thread(target=run, args=(self.send,)),
                   threading.Thread(target=run, args=(lambda: m._dispatch_target_batch(
                       [('t4', 80., 19.)], 100.),))]
        for thread in workers:
            thread.start()
        for thread in workers:
            thread.join(2.)
        self.assertEqual(errors, [])
        self.assertEqual(len(m._tracking), len(set(m._tracking.values())))
        self.assertEqual(m._tracking['t0'], 'u3')

    def test_lost_at_fourteen_seconds_queues_remaining_eleven_seconds_once(self):
        self.send()
        self.now = 114.
        self.m._release_tracker('t1', reason='lost')
        self.assertEqual(self.m.allocator.queue_revisit.call_args.kwargs['ttl_s'], 11.)
        self.m._release_tracker('t1', reason='lost')
        self.assertEqual(self.m.allocator.queue_revisit.call_count, 1)
        self.assertEqual(self.m.tracker.report.call_count, 1)
        self.assertEqual(self.m.allocator.revisit_jobs[('lost', (10, 2))][1], 125.)

    def test_revisit_last_tenth_second_is_not_rounded_up_and_expires(self):
        self.send()
        self.m._queue_lost_revisit('t1', 124.9)
        self.assertEqual(self.m.allocator.revisit_jobs[('lost', (10, 2))][1], 125.)
        self.m._queue_lost_revisit('t1', 125.)
        self.assertEqual(self.m.allocator.revisit_jobs, {})
        self.m._queue_lost_revisit('t1', 126.)
        self.assertEqual(self.m.allocator.queue_revisit.call_count, 1)

    def test_new_image_replaces_queued_priority_without_fabricating_a_report(self):
        self.send()
        self.m._queue_lost_revisit('t1', 114.)
        self.now = 114.2
        self.send(sample_s=114.1)
        self.assertEqual(self.m.allocator.revisit_jobs, {})
        self.assertEqual(self.m._reacquire_sites['t1']['expires_s'], 139.1)
        self.assertEqual(self.m.tracker.report.call_count, 2)

    def test_full_search_grid_retries_only_until_original_image_deadline(self):
        self.send()
        m = self.m
        m.allocator.queue_revisit.side_effect = [None, (10, 2)]
        self.now = 114.
        m._release_tracker('t1', reason='lost')
        self.assertFalse(m._reacquire_sites['t1']['queued'])
        # Existing other tasks need no work in this focused scheduler check.
        m._get_target_pos = lambda tid, now: (None, None)
        m._tracking.clear()
        self.now = 115.
        m._dispatch_pending_targets()
        self.assertEqual(m.allocator.queue_revisit.call_count, 2)
        self.assertEqual(m.allocator.queue_revisit.call_args.kwargs['ttl_s'], 10.)
        self.now = 125.
        m._dispatch_pending_targets()
        self.assertNotIn('t1', m._reacquire_sites)
        self.assertEqual(m.allocator.queue_revisit.call_count, 2)


if __name__ == '__main__':
    unittest.main()
