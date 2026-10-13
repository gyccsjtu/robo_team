"""Manager pause/reset/recovery regressions with mocked ROS transport only."""
import json
import os
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                     'coordination/src/robocup_swarm/scripts'))
import swarm_manager as sm


class ManagerHeaderClockTests(unittest.TestCase):
    def setUp(self):
        self.now, self.epoch, self.valid = 100., 1, True
        self.subscriber, self.publisher, self.reset_hook = Mock(), Mock(), Mock()
        self.publisher.side_effect = lambda *a, **k: NS(publish=Mock())
        patches = (
            patch.object(sm.rospy.Time, 'now', side_effect=lambda: sm.rospy.Time.from_sec(self.now)),
            patch.object(sm.rospy, 'get_time', side_effect=lambda: self.now),
            patch.object(sm.rospy, 'time_epoch', side_effect=lambda: self.epoch),
            patch.object(sm.rospy, 'is_time_valid', side_effect=lambda: self.valid),
            patch.object(sm.rospy, 'add_time_reset_callback', self.reset_hook),
            patch.object(sm.rospy, 'Subscriber', self.subscriber),
            patch.object(sm.rospy, 'Publisher', self.publisher),
            patch.object(sm.rospy, 'loginfo', Mock()),
            patch.object(sm.rospy, 'logwarn', Mock()),
            patch.object(sm.rospy, 'loginfo_throttle', Mock()),
            patch.object(sm, 'logger', return_value=NS(write=Mock())),
            patch.object(sm, '_search_bounds', return_value=(0., 14., 0., 14.)),
            patch.dict(os.environ, SEARCH_SEGMENTS='1', SOUTH_PATROL='0',
                       EAST_SPLIT_PATROL='0', SEED_TRUTH='0', IMMEDIATE_VISIBLE_DISPATCH='0'),
        )
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        self.ids = ['typhoon_h480_%d' % i for i in range(6)]
        self.m = sm.SwarmManager(self.ids)
        self.subscription_count = self.subscriber.call_count

    def status(self, uid=None, sample=None):
        return NS(uav_id=uid or self.ids[0], header=NS(stamp=sm.rospy.Time.from_sec(
            self.now if sample is None else sample)), connected=True, x=1., y=1., z=3.)

    def seed_old_attempt(self):
        m = self.m
        m._tracking, m._backup = {'t1': self.ids[0]}, {'t1': self.ids[1]}
        m._tracking_since['t1'] = 80.
        m.tracker.add_target('t1', 80.)
        m.tracker.targets['t1'].confirm_since = 80.
        m._truth_cache['t1'] = (5., 1., 99.9)
        m._eligible_observations[(self.ids[0], 't1')] = dict(sample_s=99.9, eligible=True)
        m._visual_observations.update(m._eligible_observations)
        m._release_cooldown[self.ids[4]] = sm.rospy.Time(120.)
        m.status[self.ids[0]] = self.status()
        m.last_report[self.ids[0]] = sm.rospy.Time(100.)
        m._processed_frame_times[self.ids[0]] = 99.9
        m._active_leases[self.ids[2]] = (0, 0)
        m.lease.grant(self.ids[2], (0, 0), 100., 40.)
        m._lease_hold_since[(self.ids[2], (0, 0))] = 80.
        m._segments.reserve(self.ids[3], [(1, 1)], [(10.5, 10.5)], 100., 40., 90.)
        m.grid.cells[(1, 0)].state = sm.STATE_COVERED
        m.grid.cells[(1, 0)].covered_t = 100.
        m.allocator.last_seen[(1, 0)] = 100.
        m.allocator._hot_targets['old'] = (1., 1., 100.)
        m.allocator.revisit_jobs[('lost', (1, 0))] = (100., 130.)
        m._search_view_budget = sm.ViewRevisitBudget()
        m._search_view_budget.views.append(dict(queued_s=100.))
        m._camera_point_seen = {((1, 0), 0): 100.}
        m._failed_views = {'t1': dict(sample_s=100.)}
        m._discovery_pool = {('t1', self.ids[0], 'old'): dict(sample_s=100.)}
        m._slam_cells, m._slam_w, m._slam_h = bytearray([1]), 1, 1
        m._slam_sources[self.ids[0]] = bytearray([1])
        m._map_evidence, m._g_infl = NS(hit=[100.]), NS(cells=[1])
        m._eliminated.add('t0')
        m._seen_target_slots = {'t0'}
        m._mission_finished = True

    def reset_epoch(self):
        self.now, self.epoch = 10., 2
        self.m._on_time_reset(self.epoch)
        return self.m._service_time_epoch()

    def test_epoch_clears_tasks_reservations_coverage_and_observation_caches(self):
        self.seed_old_attempt()
        m = self.m
        self.now, self.epoch = 10., 2
        m._on_time_reset(2)
        # Provider callbacks merely signal; the scheduler owns task mutations.
        self.assertEqual(m._tracking, {'t1': self.ids[0]})
        self.assertFalse(m._time_ready())
        self.assertTrue(m._service_time_epoch())
        for name in ('status', 'last_report', '_processed_frame_times', '_tracking',
                     '_backup', '_active_leases', '_lease_hold_since', '_truth_cache',
                     '_eligible_observations', '_visual_observations', '_target_hist',
                     '_release_cooldown', '_camera_point_seen', '_failed_views',
                     '_discovery_pool', '_slam_sources'):
            self.assertFalse(getattr(m, name), name)
        self.assertFalse(m.tracker.targets)
        self.assertFalse(m._segments.tasks)
        self.assertFalse(m._segments.reserved)
        self.assertTrue(all(c.state == sm.STATE_FREE and c.owner is None and c.covered_t == 0.
                            for c in m.grid.cells.values()))
        self.assertFalse(m.allocator.last_seen)
        self.assertFalse(m.allocator._hot_targets)
        self.assertFalse(m.allocator.revisit_jobs)
        self.assertFalse(m._search_view_budget.views)
        self.assertIsNone(m._map_evidence)
        self.assertIsNone(m._slam_cells)
        self.assertIsNone(m._g_infl)
        self.assertFalse(m._eliminated)
        self.assertFalse(m._seen_target_slots)
        self.assertFalse(m._mission_finished)
        cancels = [call.args[0] for call in m.assign_pub.publish.call_args_list]
        self.assertEqual({msg.uav_id for msg in cancels}, set(self.ids[:4]))
        self.assertTrue(all(msg.task_type == 255 and not msg.target_id and
                            msg.header.stamp.to_sec() == 10. for msg in cancels))
        m.confirm_pub.publish.assert_not_called()
        self.assertEqual(self.subscriber.call_count, self.subscription_count)
        self.assertEqual(self.reset_hook.call_count, 1)
        self.assertFalse(m._service_time_epoch())

    def test_invalid_or_pending_clock_blocks_dispatch_confirmation_and_ingestion(self):
        self.seed_old_attempt()
        m = self.m
        m.tracker.update = Mock(return_value=[])
        m._dispatch_target_batch = Mock()
        previous_tracking = dict(m._tracking)
        for pending in (None, 2):
            with self.subTest(pending=pending):
                self.valid = pending is not None
                m._pending_time_epoch = pending
                m._dispatch_tracker('t1', 5., 1.)
                m._dispatch_backup()
                m._allocate()
                m._update_targets()
                m._status_cb(self.status(uid=self.ids[5]))
                m._visual_source_cb(NS(data='{}'))
                m._occ_grid_cb(NS(data='{}'))
                self.assertNotIn(self.ids[5], m.status)
                self.assertEqual(m._tracking, previous_tracking)
                m._dispatch_target_batch.assert_not_called()
                m.tracker.update.assert_not_called()
                m.assign_pub.publish.assert_not_called()
                m.confirm_pub.publish.assert_not_called()

    def test_new_epoch_rejects_future_old_headers_and_recovers_from_new_evidence(self):
        self.seed_old_attempt()
        self.assertTrue(self.reset_epoch())
        m = self.m
        m.assign_pub.publish.reset_mock()
        m._status_cb(self.status(sample=100.))  # old epoch's future header
        m._status_cb(self.status(sample=0.))
        self.assertFalse(m.status)
        old_frame = dict(processed=True, source_uav=self.ids[0], sample_s=100.,
                         x=1., y=1., z=3., yaw=0., down_pitch=0.)
        m._processed_camera_frame_cb(NS(data=json.dumps(old_frame)))
        self.assertFalse(m._processed_frame_times)
        visual = dict(real_detection=True, target_id='t1', source_uav=self.ids[0],
                      sample_s=100., x=5., y=1., confidence=.9, range_m=4., official_view_ok=True)
        m._visual_source_cb(NS(data=json.dumps(visual)))
        self.assertFalse(m.tracker.targets)
        m._dispatch_tracker('t1', 5., 1.)
        m.assign_pub.publish.assert_not_called()
        self.now = 10.2
        m._status_cb(self.status(sample=10.1))
        self.assertAlmostEqual(m.last_report[self.ids[0]].to_sec(), 10.1, places=8)
        future = self.status(sample=10.21)
        future.x = 80.
        m._status_cb(future)
        self.assertEqual(m.status[self.ids[0]].x, 1.)
        old_frame['sample_s'] = 10.15
        m._processed_camera_frame_cb(NS(data=json.dumps(old_frame)))
        self.assertEqual(m._processed_frame_times[self.ids[0]], 10.15)
        visual['sample_s'] = 10.15
        m._visual_source_cb(NS(data=json.dumps(visual)))
        self.assertIn('t1', m.tracker.targets)
        self.assertEqual(m._eligible_observations[(self.ids[0], 't1')]['sample_s'], 10.15)
        m._dispatch_tracker('t1', 5., 1.)
        self.assertEqual(m._tracking, {'t1': self.ids[0]})
        self.assertEqual(m.assign_pub.publish.call_args.args[0].task_type, 1)
        self.assertEqual(self.subscriber.call_count, self.subscription_count)

    def test_epoch_arriving_during_prediction_cannot_publish_an_old_task(self):
        m = self.m
        m._tracking, m._backup = {'t1': self.ids[0]}, {'t1': self.ids[1]}
        m.status[self.ids[0]] = self.status()
        def reset_during_prediction(tid, x, y, lead):
            self.now, self.epoch = 10., 2
            m._on_time_reset(2)
            return x, y
        m._predict_target = Mock(side_effect=reset_during_prediction)
        with patch.object(sm, 'PREDICT_ACTOR_MOTION', 1):
            m._update_tracker_position('t1', 5., 1.)
        m._predict_target.assert_called_once()
        m.assign_pub.publish.assert_not_called()
        self.assertEqual(m._pending_time_epoch, 2)


if __name__ == '__main__':
    unittest.main()
