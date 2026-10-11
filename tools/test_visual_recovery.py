"""Regressions for 00:59 run: premature cancellation and fixed blind views."""
import sys
import json
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
import cooperative_tracker
import swarm_manager
import swarm_agent


class VisualRecoveryTests(unittest.TestCase):
    def setUp(self):
        clock = patch.object(swarm_manager.rospy.Time, 'now',
                             return_value=swarm_manager.rospy.Time(100.))
        clock.start()
        self.addCleanup(clock.stop)

    def manager(self):
        m = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        m._tracking, m._backup = {'t0': 'u0'}, {'t0': 'u1'}
        m._tracking_since = {'t0': 10.0}
        m._visual_observations, m._eligible_observations = {}, {}
        m._view_recovery = {}
        m.status = {uid: SimpleNamespace(connected=True) for uid in ('u0', 'u1')}
        m.last_report = {uid: swarm_manager.rospy.Time(100.0) for uid in m.status}
        m._release_cooldown = {}
        m._free_search_lease = Mock()
        m.assign_pub = Mock()
        m._blocked_cells = set()
        m.allocator = SimpleNamespace(queue_revisit=Mock())
        m._visual_source = {'t0': ('u0', 120.0, 50.0, 20.0)}
        m.tracker = cooperative_tracker.CooperativeTracker(official_only=True, strict_samples=True)
        ct = m.tracker.add_target('t0', now=10.0)
        ct.set_observers(['u0', 'u1'])
        return m

    def test_repeated_fifteen_second_streak_cannot_renew_task_forever(self):
        m = self.manager()
        m._tracking_since = {'t0': 0.}
        m.status['u0'] = SimpleNamespace(x=0., y=0., z=3., connected=True)
        m._backup, m._confirm_done_t, m._confirm_timeout_bl = {}, {}, {}
        m._truth_cache, m._cur_targets = {}, {}
        m._eliminated = set()
        m._csv, m.confirm_pub, m._auction_cycle = Mock(), Mock(), 0
        m._get_target_pos = lambda tid, now: (5., 0.)
        m._has_fresh_visual = lambda tid, now: True
        m._recent_eligible = lambda *a: True
        m._stable_owner_view = lambda *a: True
        for name in ('_release_unavailable_trackers', '_release_unverified_visuals',
                     '_dispatch_pending_targets', '_dispatch_backup'):
            setattr(m, name, Mock())
        m._release_tracker = Mock()
        ct = m.tracker.targets['t0']
        with patch.object(swarm_manager.rospy.Time, 'now') as clock:
            for i in range(1, 401):
                at = i*.5
                clock.return_value = swarm_manager.rospy.Time.from_sec(at)
                # Repeatedly reach 15 s then lose the view for 1.5 s.
                if at % 17. < 15.5:
                    m.tracker.report('u0', 't0', at, 5., 0., truth=None)
                m._update_targets()
                if m._release_tracker.called:
                    break
        self.assertTrue(m._release_tracker.called)
        self.assertLess(at, 100.)
        self.assertFalse(ct.eliminated)

    def test_timeout_keeps_primary_with_accepted_image_under_later_bad_box(self):
        m = self.manager()
        m._eligible_observations[('u0', 't0')] = {'sample_s': 99.6, 'eligible': True}
        m._visual_observations[('u0', 't0')] = {'sample_s': 99.9, 'eligible': False}
        for reason in ('tracking_tenure', 'confirm_hold_timeout', 'unverified_visual'):
            self.assertTrue(m._recover_tracking_view('t0', 100.0, reason))
        m.assign_pub.publish.assert_not_called()
        self.assertEqual(m._tracking['t0'], 'u0')
        self.assertEqual(m.tracker.targets['t0'].last_ok_t, None)  # no fabricated report

    def test_handoff_requires_multiple_images_and_cancels_only_old_owner(self):
        m = self.manager()
        image = {'sample_s': 99.8, 'eligible': True, 'hits': 1, 'since_s': 99.8}
        m._eligible_observations[('u1', 't0')] = image
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(100.0)):
            self.assertTrue(m._recover_tracking_view('t0', 100.0, 'tracking_tenure'))
            m.assign_pub.publish.assert_not_called()
            image.update(hits=3, since_s=99.2)
            self.assertTrue(m._recover_tracking_view('t0', 100.0, 'tracking_tenure'))
        self.assertEqual(m._tracking, {'t0': 'u1'})
        self.assertEqual(m._backup, {})
        self.assertEqual(m.tracker.targets['t0'].observers, {'u1'})
        messages = [c.args[0] for c in m.assign_pub.publish.call_args_list]
        self.assertEqual([(msg.uav_id, msg.task_type) for msg in messages], [('u0', 255)])

    def test_bad_images_cannot_hold_two_aircraft_forever(self):
        m = self.manager()
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(100.0)):
            self.assertTrue(m._recover_tracking_view('t0', 100.0, 'unverified_visual'))
            self.assertTrue(m._recover_tracking_view('t0', 119.9, 'unverified_visual'))
            self.assertFalse(m._recover_tracking_view('t0', 120.0, 'unverified_visual'))
        m.allocator.queue_revisit.assert_called_once()
        self.assertNotIn('t0', m._view_recovery)

    def test_long_failed_owner_hands_off_even_if_it_still_has_images(self):
        m = self.manager()
        m._eligible_observations[('u0', 't0')] = dict(
            sample_s=99.8, eligible=True, hits=30, since_s=90.)
        m._eligible_observations[('u1', 't0')] = dict(
            sample_s=99.8, eligible=True, hits=3, since_s=99.2)
        self.assertTrue(m._recover_tracking_view('t0', 100., 'tracking_tenure'))
        self.assertEqual(m._tracking, {'t0': 'u1'})
        self.assertEqual(m._backup, {})
        self.assertIsNone(m.tracker.targets['t0'].last_ok_t)
        cancel = m.assign_pub.publish.call_args.args[0]
        self.assertEqual((cancel.uav_id, cancel.task_type), ('u0', 255))

    def test_fragmented_primary_does_not_block_a_sustained_backup_view(self):
        m = self.manager()
        m.tracker.targets['t0'].resets = 4
        m._eligible_observations[('u0', 't0')] = dict(
            sample_s=99.8, eligible=True, hits=2, since_s=99.4)
        m._eligible_observations[('u1', 't0')] = dict(
            sample_s=99.8, eligible=True, hits=20, since_s=96.)
        self.assertTrue(m._recover_tracking_view('t0', 100., 'unverified_visual'))
        self.assertEqual(m._tracking, {'t0': 'u1'})
        self.assertEqual(m._tracking_since['t0'], 100.)
        self.assertIsNone(m.tracker.targets['t0'].last_ok_t)

    def test_healthy_primary_or_short_backup_series_cannot_trigger_relief_handoff(self):
        for primary_since, backup_since, task_since in ((90., 96., 10.),
                                                       (99.4, 99.2, 10.),
                                                       (99.4, 96., 95.)):
            with self.subTest(primary_since=primary_since, backup_since=backup_since,
                              task_since=task_since):
                m = self.manager()
                m._tracking_since['t0'] = task_since
                m.tracker.targets['t0'].resets = 4
                m._eligible_observations[('u0', 't0')] = dict(
                    sample_s=99.8, eligible=True, hits=20, since_s=primary_since)
                m._eligible_observations[('u1', 't0')] = dict(
                    sample_s=99.8, eligible=True, hits=20, since_s=backup_since)
                self.assertTrue(m._recover_tracking_view('t0', 100., 'unverified_visual'))
                self.assertEqual(m._tracking, {'t0': 'u0'})
                m.assign_pub.publish.assert_not_called()

    def test_isolated_good_frames_cannot_extend_failed_attempt_forever(self):
        m = self.manager()
        for t in (100., 104., 108., 112., 116.):
            m._eligible_observations[('u0', 't0')] = dict(
                sample_s=t-.1, eligible=True, hits=1, since_s=t-.1)
            self.assertTrue(m._recover_tracking_view('t0', t, 'unverified_visual'))
        m._eligible_observations[('u0', 't0')]['sample_s'] = 119.9
        self.assertFalse(m._recover_tracking_view('t0', 120., 'unverified_visual'))
        self.assertEqual(m._tracking_since['t0'], 10.)

    def test_continuous_real_view_can_recover_a_short_failed_attempt(self):
        m = self.manager()
        self.assertTrue(m._recover_tracking_view('t0', 100., 'unverified_visual'))
        m._eligible_observations[('u0', 't0')] = dict(
            sample_s=119.9, eligible=True, hits=20, since_s=110.)
        self.assertTrue(m._recover_tracking_view('t0', 120., 'unverified_visual'))
        self.assertNotIn('t0', m._view_recovery)

    def test_hard_tenure_not_renewed_by_continuous_classifier_detections(self):
        m = self.manager()
        for t in (100., 110.):
            m._eligible_observations[('u0', 't0')] = dict(
                sample_s=t-.1, eligible=True, hits=30, since_s=90.)
            self.assertTrue(m._recover_tracking_view('t0', t, 'tracking_tenure'))
            self.assertTrue(m._recover_tracking_view('t0', t, 'unverified_visual'))
        m._eligible_observations[('u0', 't0')]['sample_s'] = 119.9
        self.assertFalse(m._recover_tracking_view('t0', 120., 'tracking_tenure'))
        self.assertEqual(m._tracking_since['t0'], 10.)

    def test_failed_location_cannot_immediately_recapture_released_uav(self):
        m = self.manager()
        m._visual_source['t0'] = ('u0', 100., 50., 20.)
        m._defer_failed_view('t0', ('u0', 'u1'), 100.)
        self.assertFalse(m._failed_view_dispatch_ok('t0', 50.1, 20., 106.))
        self.assertTrue(m._failed_view_dispatch_ok('t0', 50., 20., 115.))
        self.assertFalse(m.tracker.targets['t0'].eliminated)
        self.assertTrue(m._failed_view_dispatch_ok('t0', 70., 20., 106.))
        self.assertFalse(m._failed_view_dispatch_ok('t1', 50., 20., 106.))
        self.assertTrue(m._failed_view_dispatch_ok('t1', 52., 20., 106.))

    def test_failed_location_backoff_is_bounded_and_increases_on_repeat(self):
        m = self.manager()
        for t, delay in ((100., 15.), (130., 30.), (170., 60.), (240., 60.)):
            m._visual_source['t0'] = ('u0', t, 50., 20.)
            m._defer_failed_view('t0', ('u0',), t)
            self.assertEqual(m._failed_views['t0']['until'], t+delay)
        m._visual_source['t0'] = ('u0', 250., 80., 20.)
        m._defer_failed_view('t0', ('u0',), 250.)
        self.assertEqual(m._failed_views['t0']['attempts'], 1)

    def test_new_camera_can_recover_stationary_actor_but_old_evidence_cannot(self):
        m = self.manager()
        m._visual_source['t0'] = ('u0', 100., 50., 20.)
        m._defer_failed_view('t0', ('u0',), 100.)
        evidence = dict(sample_s=105.9, hits=20, since_s=100.2,
                        eligible=True, x=50., y=20.)
        m._eligible_observations[('u0', 't0')] = evidence.copy()
        self.assertFalse(m._failed_view_dispatch_ok('t0', 50., 20., 106.))
        m._eligible_observations[('u1', 't0')] = dict(evidence, since_s=99.)
        self.assertFalse(m._failed_view_dispatch_ok('t0', 50., 20., 106.))
        m._eligible_observations[('u1', 't0')] = evidence
        self.assertTrue(m._failed_view_dispatch_ok('t0', 50., 20., 106.))
        self.assertIsNone(m.tracker.targets['t0'].last_ok_t)

    def test_long_good_view_requests_reimage_without_cancel_or_report_credit(self):
        m = self.manager()
        m._eligible_observations[('u0', 't0')] = {'sample_s': 99.6, 'eligible': True}
        m.reobserve_pub = Mock()
        self.assertTrue(m._recover_tracking_view('t0', 100.0, 'tracking_tenure'))
        self.assertTrue(m._recover_tracking_view('t0', 100.1, 'tracking_tenure'))
        m.reobserve_pub.publish.assert_called_once()
        request = json.loads(m.reobserve_pub.publish.call_args.args[0].data)
        self.assertEqual((request['uav_id'], request['target_id']), ('u0', 't0'))
        m.assign_pub.publish.assert_not_called()
        self.assertIsNone(m.tracker.targets['t0'].last_ok_t)

    def agent(self):
        a = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        a.uav_id = 'u0'
        a.local_xy, a.offset = (58.0, 20.0), (0.0, 0.0)
        a._orbit_target, a._orbit_entered_t = 't0', 90.0
        a._t_seen = {'t0': 100.0}
        a._visual_observations = {}
        a.grid = SimpleNamespace(origin=(-50.0, -60.0), width=420, height=260,
                                 resolution=0.5, world_to_cell=lambda xy: xy,
                                 is_free=lambda c: True)
        a.los = SimpleNamespace(visible=lambda *args: True)
        return a

    def test_blind_fixed_view_moves_laterally_then_stops_on_good_image(self):
        a = self.agent()
        goal = a._reobserve_goal(50.0, 20.0, 100.0)
        self.assertIsNotNone(goal)
        self.assertGreater(abs(goal[1]-20.0), 3.0)
        self.assertEqual(goal, a._reobserve_goal(50.0, 20.0, 100.5))
        a._last_eligible_image = {'t0': 100.5}
        self.assertIsNone(a._reobserve_goal(50.0, 20.0, 100.6))

    def test_reobserve_rejects_obstacles_on_segment_and_map_exit(self):
        a = self.agent()
        a.grid.is_free = lambda c: abs(c[1]-20.0) < 0.5 or abs(c[1]-20.0) > 2.0
        self.assertIsNone(a._reobserve_goal(50.0, 20.0, 100.0))
        a.grid.is_free = lambda c: True
        with patch.object(swarm_agent, 'flight_bounds', return_value=(0, 60, 0, 40)):
            self.assertFalse(a._safe_reobserve_segment((58.0, 20.0), (62.0, 20.0)))

    def test_stale_team_target_or_close_blind_zone_does_not_trigger_side_move(self):
        a = self.agent()
        a._t_seen['t0'] = 90.0
        self.assertIsNone(a._reobserve_goal(50.0, 20.0, 100.0))
        a._t_seen['t0'] = 100.0
        a.local_xy = (53.0, 20.0)
        self.assertIsNone(a._reobserve_goal(50.0, 20.0, 100.0))

    def test_reimage_request_is_fresh_owned_bounded_and_deduplicated(self):
        a = self.agent()
        a.assignment = SimpleNamespace(task_type=1, target_id='t0')
        a._last_eligible_image = {'t0': 100.0}
        data = dict(uav_id='u0', target_id='t1', sample_s=100.0)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)):
            a._reobserve_cb(SimpleNamespace(data=json.dumps(data)))
            self.assertIsNone(a._reobserve_goal(50.0, 20.0, 100.0))
            data['target_id'] = 't0'
            a._reobserve_cb(SimpleNamespace(data=json.dumps(data)))
            self.assertIsNotNone(a._reobserve_goal(50.0, 20.0, 100.0))
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.5)):
            a._reobserve_cb(SimpleNamespace(data=json.dumps(data)))
        self.assertEqual(a._reobserve_forced, ('t0', 106.0))
        a._last_eligible_image['t0'] = 106.0
        self.assertIsNone(a._reobserve_goal(50.0, 20.0, 106.0))

    def test_bad_frame_cannot_reset_local_streak_before_actual_image_gap(self):
        a = self.agent()
        a._confirm_start, a._last_confirm_t = 90.0, 99.5
        a._visual_observations['t0'] = {'sample_s': 99.9, 'eligible': False}
        a._eligible_observations = {'t0': {'sample_s': 99.5, 'eligible': True}}
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)):
            a._update_orbit()
        self.assertEqual((a._confirm_start, a._last_confirm_t), (90.0, 99.5))
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.51)):
            a._update_orbit()
        self.assertEqual((a._confirm_start, a._last_confirm_t), (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
