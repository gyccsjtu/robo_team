"""An EKF-faulted aircraft must not keep a tracking slot or backup lease."""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))

import cooperative_tracker
import swarm_manager


class UnavailableTrackingTests(unittest.TestCase):
    def setUp(self):
        now_patch = patch.object(swarm_manager.rospy.Time, 'now',
                                 return_value=swarm_manager.rospy.Time(100.0))
        now_patch.start()
        self.addCleanup(now_patch.stop)

    def _manager(self, main_connected, backup_connected=True):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.status = {
            'u0': SimpleNamespace(connected=main_connected),
            'u1': SimpleNamespace(connected=backup_connected),
        }
        manager.last_report = {uid: swarm_manager.rospy.Time(100.0)
                               for uid in manager.status}
        manager._tracking = {'t0': 'u0'}
        manager._backup = {'t0': 'u1'}
        manager._tracking_since = {'t0': 90.0}
        manager._approach_leases = {}
        manager._release_cooldown = {}
        manager._truth_cache = {}
        manager._free_search_lease = Mock()
        manager.assign_pub = Mock()
        manager.allocator = SimpleNamespace(clear_hot_target=Mock())
        manager.tracker = cooperative_tracker.CooperativeTracker(
            official_only=True, strict_samples=True)
        target = manager.tracker.add_target('t0', now=90.0)
        target.set_observers(['u0', 'u1'])
        return manager, target

    def test_failed_main_promotes_healthy_backup_without_cancelling_it(self):
        manager, target = self._manager(False, True)
        manager._release_unavailable_trackers(100.0)
        self.assertEqual(manager._tracking, {'t0': 'u1'})
        self.assertEqual(manager._backup, {})
        self.assertEqual(target.observers, {'u1'})
        cancels = [call.args[0] for call in
                   manager.assign_pub.publish.call_args_list]
        self.assertEqual([(msg.uav_id, msg.task_type) for msg in cancels],
                         [('u0', 255)])
        self.assertEqual(manager._tracking_since['t0'], 100.0)

    def test_failed_backup_is_freed_while_main_continues(self):
        manager, target = self._manager(True, False)
        manager._release_unavailable_trackers(100.0)
        self.assertEqual(manager._tracking, {'t0': 'u0'})
        self.assertEqual(manager._backup, {})
        self.assertEqual(target.observers, {'u0'})
        cancel = manager.assign_pub.publish.call_args.args[0]
        self.assertEqual((cancel.uav_id, cancel.task_type), ('u1', 255))

    def test_both_failed_release_target_for_next_dispatch(self):
        manager, target = self._manager(False, False)
        manager._release_unavailable_trackers(100.0)
        self.assertEqual(manager._tracking, {})
        self.assertEqual(manager._backup, {})
        cancels = [call.args[0] for call in
                   manager.assign_pub.publish.call_args_list]
        self.assertEqual({msg.uav_id for msg in cancels}, {'u0', 'u1'})

    def test_silent_main_is_released_after_three_seconds(self):
        manager, target = self._manager(True, True)
        manager.last_report['u0'] = swarm_manager.rospy.Time(96.0)
        manager._release_unavailable_trackers(100.0)
        self.assertEqual(manager._tracking, {'t0': 'u1'})

    def _backup_manager(self, primary_sample):
        manager, target = self._manager(True, True)
        manager._backup = {}
        manager.status['u1'].connected = False
        manager.status['u0'].x = manager.status['u0'].y = 0.0
        manager.status['u3'] = SimpleNamespace(connected=True, x=6.0, y=0.0)
        manager.status['u4'] = SimpleNamespace(connected=True, x=9.0, y=0.0)
        manager.last_report.update({
            'u3': swarm_manager.rospy.Time(100.0),
            'u4': swarm_manager.rospy.Time(100.0),
        })
        cells = {
            (3, 0): SimpleNamespace(state=swarm_manager.STATE_FREE, owner=None),
            (4, 0): SimpleNamespace(state=swarm_manager.STATE_COVERED, owner=None),
        }
        manager.grid = SimpleNamespace(
            zone_cells={3: [(3, 0)], 4: [(4, 0)]},
            visit_time={},
            get_uav_zone=lambda uid: int(uid[-1]),
            cell=lambda key: cells.get(key),
        )
        manager.allocator.visit_time = {}
        manager._blocked_cells = set()
        manager._visual_observations = {
            ('u0', 't0'): {'eligible': True, 'sample_s': primary_sample},
        }
        manager._truth_cache = {'t0': (0.0, 0.0, 100.0)}
        manager._confirm_timeout_bl = {}
        manager._eliminated = set()
        manager._backup_search_capacity = Mock(return_value=True)
        target.resets = 1
        return manager, target

    def test_historical_reset_does_not_pull_backup_during_fresh_primary_video(self):
        manager, target = self._backup_manager(primary_sample=99.5)
        manager._dispatch_backup()
        self.assertEqual(manager._backup, {})
        manager.assign_pub.publish.assert_not_called()

    def test_repeated_current_view_failure_gets_relief_despite_fresh_images(self):
        manager, target = self._backup_manager(primary_sample=99.8)
        target.resets = 4
        target.confirm_since, target.last_ok_t = 99.2, 99.8
        manager._tracking_since = {'t0': 70.}
        manager._dispatch_backup()
        # It may borrow a nearby sole zone searcher, while the global search
        # reserve and one-primary/one-backup constraints still apply.
        self.assertEqual(manager._backup, {'t0': 'u3'})
        self.assertEqual(manager._tracking, {'t0': 'u0'})
        self.assertFalse(target.eliminated)

    def test_old_failures_do_not_interrupt_current_continuous_tracking(self):
        manager, target = self._backup_manager(primary_sample=99.8)
        target.resets = 12
        target.confirm_since, target.last_ok_t = 90., 99.8
        manager._tracking_since = {'t0': 70.}
        manager._dispatch_backup()
        self.assertEqual(manager._backup, {})

    def test_relief_waits_for_one_full_confirmation_attempt(self):
        manager, target = self._backup_manager(primary_sample=99.8)
        target.resets = 4
        target.confirm_since, target.last_ok_t = 99.2, 99.8
        manager._tracking_since = {'t0': 90.}
        manager._dispatch_backup()
        self.assertEqual(manager._backup, {})

    def test_stale_primary_preserves_only_unseen_zone_searcher(self):
        manager, target = self._backup_manager(primary_sample=95.0)
        self.assertTrue(manager._sole_first_pass_zone_searcher('u3', {'u0'}))
        manager._dispatch_backup()
        self.assertEqual(manager._backup, {'t0': 'u4'})
        msg = manager.assign_pub.publish.call_args.args[0]
        self.assertEqual((msg.uav_id, msg.task_type), ('u4', 1))

    def test_no_other_backup_keeps_unseen_zone_searcher_on_search(self):
        manager, target = self._backup_manager(primary_sample=95.0)
        manager.status['u4'].connected = False
        manager._dispatch_backup()
        self.assertEqual(manager._backup, {})
        manager.assign_pub.publish.assert_not_called()

    def test_only_currently_qualified_slots_allow_remote_relief(self):
        manager, target = self._backup_manager(primary_sample=95.)
        manager.status['u4'].connected = False
        manager.status['u3'].x = -40.
        manager._truth_cache['t0'] = (130.,0.,100.)
        manager._seen_target_slots = {'t%d'%i for i in range(6)}
        manager._visual_observations[('u2','t0')] = dict(eligible=True,sample_s=100.)
        manager._tracking_since = {'t0':50.}
        manager._view_recovery = {'t0':85.}
        manager._dispatch_backup()
        self.assertEqual(manager._backup, {})  # historical sightings cannot consume search
        for i in range(1, 6):
            manager._visual_observations[('u2', 't%d'%i)] = dict(
                eligible=True, sample_s=100.)
        manager._dispatch_backup()
        self.assertEqual(manager._backup, {'t0':'u3'})
        self.assertEqual(manager._tracking_since['t0'],50.)
        self.assertEqual(manager._view_recovery['t0'],85.)
        self.assertGreater(manager._backup_approach_leases['t0']['until'],150.)

    def test_remote_relief_wait_is_bounded_and_requires_current_camera_evidence(self):
        manager, target = self._backup_manager(primary_sample=95.)
        manager._backup = {'t0':'u3'}
        manager.status['u3'].x = -40.
        manager._record_approach('t0','u3',130.,0.,100.,backup=True)
        manager._view_recovery = {'t0':100.}
        for now in (104.,108.,112.,116.,120.,124.):
            manager.status['u3'].x += 8.
            self.assertTrue(manager._hold_approaching_target('t0',now,backup=True))
        manager._visual_observations[('u2','t0')] = dict(eligible=True,sample_s=125.)
        manager.last_report['u3'] = swarm_manager.rospy.Time(125.)
        self.assertTrue(manager._recover_tracking_view('t0',125.,'unverified_visual'))
        manager._view_recovery = {'t0':100.}
        self.assertFalse(manager._recover_tracking_view('t0',190.,'unverified_visual'))


if __name__ == '__main__':
    unittest.main()
