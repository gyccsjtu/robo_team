"""Multi-person dispatch and image-qualified handoff, without a simulator."""
import os
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
import cooperative_tracker
import yolo_target_bridge as bridge


class JointDispatchTests(unittest.TestCase):
    def setUp(self):
        self.clock = patch.object(sm.rospy.Time, 'now', return_value=sm.rospy.Time(100))
        self.clock.start()
        self.addCleanup(self.clock.stop)
        m = self.m = sm.SwarmManager.__new__(sm.SwarmManager)
        m.uav_ids = ['u0', 'u1']
        m.status = {uid: NS(x=x, y=0., z=3., connected=True)
                    for uid, x in [('u0', 0.), ('u1', 10.)]}
        m.last_report = {uid: sm.rospy.Time(100) for uid in m.uav_ids}
        m._tracking, m._backup, m._release_cooldown = {}, {}, {}
        m._visual_observations, m._eligible_observations, m._visual_source = {}, {}, {}
        m._sanitize_dispatch_xy = lambda x, y, tid: (x, y)
        m._free_search_lease = Mock()
        m.assign_pub = Mock()
        m.tracker = NS(targets={tid: NS(observers=set()) for tid in ('t0', 't1')})

    def test_matching_reserves_unique_near_aircraft_for_second_person(self):
        with patch.dict(os.environ, {'DISPATCH_FAR_LIMIT': '12'}):
            self.m._dispatch_target_batch([('t0', 1., 0.), ('t1', -4., 0.)], 100.)
        self.assertEqual(self.m._tracking, {'t0': 'u1', 't1': 'u0'})
        self.assertEqual(self.m.assign_pub.publish.call_count, 2)
        self.assertEqual(self.m._free_search_lease.call_count, 2)

    def test_west_aircraft_can_take_east_target_with_finite_travel_budget(self):
        m = self.m
        m.status['u0'].x, m.status['u1'].x = -30., -40.
        with patch.dict(os.environ, {'DISPATCH_FAR_LIMIT':'0'}):
            m._dispatch_target_batch([('t0',130.,0.)],100.)
        self.assertEqual(m._tracking, {'t0':'u0'})
        self.assertGreater(m._approach_leases['t0']['until'],150.)
        self.assertLessEqual(m._approach_leases['t0']['until'],190.)
        for now in (104.,108.,112.,116.,120.,124.):
            m.status['u0'].x += 8.
            self.assertTrue(m._hold_approaching_target('t0',now))
        self.assertFalse(m._hold_approaching_target('t0',190.))

    def test_repeated_lost_same_place_is_deferred_but_new_place_is_not(self):
        m = self.m
        m._tracking = {'t1':'u0'}
        m._visual_source['t1'] = ('u0',97.,8.,0.)
        m._note_lost_view('t1',100.)
        self.assertTrue(m._failed_view_dispatch_ok('t1',8.,0.,101.))
        m._visual_source['t1'] = ('u0',117.,8.,0.)
        m._note_lost_view('t1',120.)
        self.assertFalse(m._failed_view_dispatch_ok('t1',8.,0.,121.))
        self.assertTrue(m._failed_view_dispatch_ok('t1',18.,0.,121.))
        self.assertTrue(m._failed_view_dispatch_ok('t1',8.,0.,136.))

    def test_historical_six_sightings_do_not_permanently_remove_search_reserve(self):
        m = self.m
        m._eligible_observations = {('u0','t%d'%i):dict(eligible=True,sample_s=50.)
                                    for i in range(6)}
        self.assertEqual(len(m._observed_target_slots()),6)
        m._eligible_observations.clear()
        self.assertFalse(m._backup_search_capacity())
        self.assertEqual(len(m._observed_target_slots()),6)

    def test_search_reserve_uses_current_qualified_images_and_official_removals(self):
        m = self.m
        m._seen_target_slots = {'t%d'%i for i in range(6)}
        m._eliminated = {'t0', 't1', 't2'}
        m._eligible_observations = {
            ('u0', 't3'): dict(eligible=True, sample_s=99.5),
            ('u0', 't4'): dict(eligible=True, sample_s=98.5),
            ('u0', 't5'): dict(eligible=True, sample_s=100.1),
        }
        self.assertEqual(m._currently_qualified_target_slots(100.),
                         {'t0', 't1', 't2', 't3'})
        self.assertFalse(m._backup_search_capacity())
        m._eligible_observations[('u0', 't4')]['sample_s'] = 99.8
        self.assertTrue(m._backup_search_capacity())

    def test_new_close_primary_keeps_recent_unreportable_view_for_three_seconds(self):
        m = self.m
        m.status['u0'].x, m.status['u1'].x = 0., 10.
        m._tracking = {'t0': 'u0'}
        m._tracking_since = {'t0': 99.8}
        m.tracker.targets['t0'].observers = {'u0'}
        m._visual_observations[('u0', 't0')] = dict(
            sample_s=99.9, eligible=False, x=5., y=0., candidate_id='person')
        m._eligible_observations[('u1', 't0')] = dict(
            sample_s=99.9, eligible=True, x=5., y=0.,
            candidate_id='peer', hits=4, since_s=99.3)
        m._handoff_distant_tracker('t0', 5., 0., m.tracker.targets['t0'])
        self.assertEqual(m._tracking, {'t0': 'u0'})
        m.assign_pub.publish.assert_not_called()

        # Fresh raw frames do not extend the assignment grace indefinitely.
        m._visual_observations[('u0', 't0')]['sample_s'] = 103.0
        m._eligible_observations[('u1', 't0')]['sample_s'] = 103.0
        m.last_report = {uid: sm.rospy.Time(103.1) for uid in m.uav_ids}
        with patch.object(sm.rospy.Time, 'now', return_value=sm.rospy.Time(103.1)):
            m._handoff_distant_tracker('t0', 5., 0., m.tracker.targets['t0'])
        self.assertEqual(m._tracking, {'t0': 'u1'})
        self.assertEqual([(call.args[0].uav_id, call.args[0].task_type)
                          for call in m.assign_pub.publish.call_args_list],
                         [('u1', 1), ('u0', 255)])

    def _assert_raw_does_not_block_handoff(self, raw_sample, raw_id):
        m = self.m
        m._tracking, m._tracking_since = {'t0': 'u0'}, {'t0': 99.8}
        m.tracker.targets['t0'].observers = {'u0'}
        m._visual_observations[('u0', 't0')] = dict(
            sample_s=raw_sample, eligible=False, x=5., y=0., candidate_id=raw_id)
        m._eligible_observations[('u0', 't0')] = dict(
            sample_s=98.5, eligible=True, x=5., y=0., candidate_id='person')
        m._eligible_observations[('u1', 't0')] = dict(
            sample_s=99.9, eligible=True, x=5., y=0.,
            candidate_id='peer', hits=4, since_s=99.3)
        m._handoff_distant_tracker('t0', 5., 0., m.tracker.targets['t0'])
        self.assertEqual(m._tracking['t0'], 'u1')

    def test_stale_raw_view_cannot_block_handoff(self):
        self._assert_raw_does_not_block_handoff(98.9, 'person')

    def test_different_instance_raw_view_cannot_block_handoff(self):
        self._assert_raw_does_not_block_handoff(99.9, 'other')

    def test_recorded_red_slot_rename_keeps_search_capacity_for_second_person(self):
        root = Path(__file__).resolve().parents[1]
        fixture = json.loads((root/'tools/fixtures/red_slot_rename_093017.json').read_text())
        m = self.m
        m.uav_ids = ['typhoon_h480_%d' % i for i in range(6)]
        m.status = {uid: NS(x=48., y=-12., z=3., connected=True)
                    for uid in m.uav_ids}
        m._tracking = {'t%d' % i: m.uav_ids[i] for i in (0, 1, 2, 3, 5)}
        m._seen_target_slots = {'t0', 't1', 't2', 't3'}
        m._visual_quarantine, m._visual_candidate = {}, {}
        m._last_detection_sample, m._last_detect = {}, {}
        m._visual_near_seen, m._truth_cache, m._cur_targets = {}, {}, {}
        m._eliminated = set()
        m._update_tracker_position = Mock()
        m.tracker = cooperative_tracker.CooperativeTracker(
            official_only=True, strict_samples=True)
        for tid in m._seen_target_slots:
            m.tracker.add_target(tid, now=fixture['reports'][0]['t'])
        node = bridge._mk_bridge()
        # Deliver real bridge messages through the manager's evidence and
        # tracker callbacks. Neither geometry nor a fabricated judge event
        # may count a renamed camera slot as a sixth observed person.
        node.source_pub = NS(publish=m._visual_source_cb)
        with patch.object(sm.rospy.Time, 'now') as now:
            for r in fixture['reports']:
                now.return_value = sm.rospy.Time.from_sec(r['t']+.1)
                node._advance(r['t']+.1-node._now())
                node._report_cb(bridge._FakeString(json.dumps(dict(
                    target_id=r['tag'], xyz=[r['x'], r['y'], 1.25],
                    confidence=r['conf'], range_m=r['range_m'], sample_s=r['t'],
                    source_uav=r['source'], real_detection=True,
                    official_view_ok=r['official_ok']))))
        self.assertIn('t5', m.tracker.targets)
        self.assertNotIn('t4', m.tracker.targets)
        self.assertEqual(m._visual_source['t5'][1], 2567.26)
        self.assertEqual(m._observed_target_slots(), {'t0', 't1', 't2', 't3', 't5'})
        self.assertFalse(m._backup_search_capacity())
        self.assertTrue(m._update_tracker_position.called)

    def test_actual_blue_hypothesis_switch_hands_task_to_camera_at_new_site(self):
        root = Path(__file__).resolve().parents[1]
        rows = json.loads((root/'tools/fixtures/blue_disjoint_102820.json').read_text())['reports']
        m = self.m
        m.uav_ids = ['typhoon_h480_%d' % i for i in range(6)]
        m.status = {uid: NS(x=-30., y=13., z=3., connected=True) for uid in m.uav_ids}
        m.status['typhoon_h480_5'].x, m.status['typhoon_h480_5'].y = 122., 46.
        m._tracking = {'t1':'typhoon_h480_2'}
        m._tracking_since = {'t1':2651.2}
        m._visual_quarantine, m._visual_candidate = {}, {}
        m._last_detection_sample, m._last_detect = {}, {}
        m._visual_near_seen, m._truth_cache, m._cur_targets = {}, {}, {}
        m._eliminated = set()
        m._update_tracker_position = Mock()
        m.tracker = cooperative_tracker.CooperativeTracker(official_only=True,strict_samples=True)
        node = bridge._mk_bridge()
        node.source_pub = NS(publish=m._visual_source_cb)
        with patch.object(sm.rospy.Time, 'now') as now:
            for r in rows:
                at = r['receive']
                now.return_value = sm.rospy.Time.from_sec(at)
                m.last_report = {uid:sm.rospy.Time.from_sec(at) for uid in m.uav_ids}
                node._advance(at-node._now())
                node._report_cb(bridge._FakeString(json.dumps(dict(
                    target_id='blue',xyz=[r['x'],r['y'],1.25],confidence=r['confidence'],
                    range_m=r['range_m'],sample_s=r['sample'],source_uav=r['source'],
                    real_detection=True,official_view_ok=r['official_ok']))))
                if 't1' in m.tracker.targets:
                    tx,ty = m._get_target_pos('t1',at)
                    m._handoff_distant_tracker('t1',tx,ty,m.tracker.targets['t1'])
        self.assertEqual(m._tracking['t1'],'typhoon_h480_5')
        self.assertGreater(m._get_target_pos('t1',rows[-1]['receive'])[0],100.)
        self.assertTrue(any(c.args[0].uav_id=='typhoon_h480_2' and c.args[0].task_type==255
                            for c in m.assign_pub.publish.call_args_list))
        self.assertFalse(m._eliminated)

    def test_failed_view_is_not_immediately_reassigned_by_joint_dispatch(self):
        m = self.m
        m._visual_source['t0'] = ('u0', 100., 1., 0.)
        m._defer_failed_view('t0', ('u0',), 100.)
        m.last_report = {uid: sm.rospy.Time(106) for uid in m.uav_ids}
        m._dispatch_target_batch([('t0', 1., 0.), ('t1', 9., 0.)], 106.)
        self.assertEqual(m._tracking, {'t1': 'u1'})
        # A newly seen same-colour person elsewhere is not blocked by this.
        m._dispatch_target_batch([('t0', -5., 0.)], 106.)
        self.assertEqual(m._tracking, {'t1': 'u1', 't0': 'u0'})

    def test_recovery_camera_evidence_cannot_send_old_failed_owner_back(self):
        m = self.m
        m._visual_source['t0'] = ('u0', 100., 1., 0.)
        m._defer_failed_view('t0', ('u0',), 100.)
        m.last_report = {uid: sm.rospy.Time(106) for uid in m.uav_ids}
        m._eligible_observations[('u1', 't0')] = dict(
            sample_s=105.9, since_s=100.2, hits=20, eligible=True, x=1., y=0.)
        m._dispatch_target_batch([('t0', 1., 0.)], 106.)
        self.assertEqual(m._tracking, {'t0': 'u1'})

    def test_matching_minimizes_total_cost_and_is_order_independent(self):
        costs = {'t0': {'u0': 1., 'u1': 2.}, 't1': {'u0': 2., 'u1': 50.}}
        expected = {'t0': 'u1', 't1': 'u0'}
        self.assertEqual(sm.joint_target_assignment(costs), expected)
        self.assertEqual(sm.joint_target_assignment(dict(reversed(list(costs.items())))), expected)
        self.assertEqual(sm.joint_target_assignment({}), {})

    def test_periodic_runtime_dispatch_uses_joint_matching(self):
        m = self.m
        m._eliminated, m._confirm_timeout_bl = set(), {}
        m.allocator = NS()
        positions = {'t0': (1., 0.), 't1': (-4., 0.)}
        m._get_target_pos = lambda tid, now: positions[tid]
        m._update_tracker_position = Mock()
        for ct in m.tracker.targets.values(): ct.eliminated = False
        with patch.dict(os.environ, {'DISPATCH_FAR_LIMIT': '12'}):
            m._dispatch_pending_targets()
            self.assertEqual(m._tracking, {'t0': 'u1', 't1': 'u0'})
            m._dispatch_pending_targets()
        self.assertEqual(m.assign_pub.publish.call_count, 2)
        self.assertEqual(m._update_tracker_position.call_count, 2)

    def test_six_targets_get_six_distinct_owners(self):
        costs = {f't{i}': {f'u{j}': abs(i-j) for j in range(6)} for i in range(6)}
        self.assertEqual(sm.joint_target_assignment(costs), {f't{i}': f'u{i}' for i in range(6)})

    def test_busy_latest_camera_does_not_hide_free_observer(self):
        m = self.m
        m._tracking = {'t1': 'u0'}
        m._visual_source = {'t0': ('u0', 100., 5., 0.)}
        m._visual_observations = {('u1', 't0'): dict(sample_s=99.8, x=5., y=0.)}
        self.assertEqual(m._preferred_camera('t0', 5., 0., 100., {'u0'}), ('u1', 5.))
        m._dispatch_target_batch([('t0', 5., 0.), ('t1', 2., 0.)], 100.)
        self.assertEqual(m._tracking, {'t1': 'u0', 't0': 'u1'})

    def test_stale_or_cooling_aircraft_cannot_take_new_person(self):
        self.m.last_report['u0'] = sm.rospy.Time(96)
        self.m._release_cooldown['u1'] = sm.rospy.Time(102)
        self.m._dispatch_target_batch([('t0', 5., 0.)], 100.)
        self.assertEqual(self.m._tracking, {})
        self.m._free_search_lease.assert_not_called()

    def test_concurrent_dispatch_does_not_double_assign_one_aircraft(self):
        self.m.uav_ids = ['u0']
        errors = []
        def dispatch(tid):
            try:
                self.m._dispatch_target_batch([(tid, 5., 0.)], 100.)
            except Exception as exc:
                errors.append(exc)
        threads = [threading.Thread(target=dispatch, args=(tid,)) for tid in ('t0', 't1')]
        for thread in threads: thread.start()
        for thread in threads: thread.join(2.)
        self.assertEqual(errors, [])
        self.assertEqual(len(self.m._tracking), 1)

    def test_nearer_aircraft_without_continuous_images_cannot_cancel_owner(self):
        m = self.m
        m._tracking = {'t0': 'u0'}
        m.status['u1'].x = 38.
        ct = m.tracker.targets['t0']
        ct.observers = {'u0'}
        for evidence in (None, dict(sample_s=100., since_s=100., hits=1, x=40., y=0.),
                         dict(sample_s=98., since_s=97., hits=5, x=40., y=0.)):
            m._eligible_observations = {('u1', 't0'): evidence} if evidence else {}
            m._handoff_distant_tracker('t0', 40., 0., ct)
            self.assertEqual(m._tracking, {'t0': 'u0'})
        m.assign_pub.publish.assert_not_called()
        m._eligible_observations = {('u1', 't0'): dict(
            sample_s=100., since_s=99.4, hits=3, eligible=True, x=40., y=0.)}
        m._tracking_since = {'t0': 50.}
        m._handoff_distant_tracker('t0', 40., 0., ct)
        self.assertEqual(m._tracking, {'t0': 'u1'})
        self.assertEqual(m._tracking_since['t0'], 100.)
        self.assertEqual([(c.args[0].uav_id, c.args[0].task_type)
                          for c in m.assign_pub.publish.call_args_list], [('u1', 1), ('u0', 255)])

    def test_camera_nine_metres_away_wins_over_blind_aircraft_at_five(self):
        m = self.m
        m.status['u0'].x, m.status['u1'].x = 0., 4.
        m._visual_observations[('u0', 't0')] = dict(sample_s=100., x=9., y=0.)
        m._dispatch_target_batch([('t0', 9., 0.)], 100.)
        self.assertEqual(m._tracking, {'t0': 'u0'})

    def _east_visual_owner_scenario(self):
        """Random-map run: the red2 task took u0 just before it saw white."""
        m = self.m
        m.uav_ids = ['u%d' % i for i in range(6)]
        m.status = {uid: NS(x=x, y=y, z=4., connected=True)
                    for uid, x, y in [('u0', 135., 20.), ('u1', 45., -20.),
                                      ('u2', 0., 0.), ('u3', 70., 0.),
                                      ('u4', 10., 0.), ('u5', 125., 38.)]}
        m.last_report = {uid: sm.rospy.Time(100) for uid in m.uav_ids}
        m._tracking = {'t4': 'u0', 't5': 'u5', 't3': 'u3', 't0': 'u1'}
        m._tracking_since = {'t4': 90., 't5': 90., 't3': 99., 't0': 90.}
        m._confirm_done_t = {}
        m._eliminated = set()
        m._visual_candidate = {}
        m._last_detect = {}
        m._visual_near_seen = {}
        m._truth_cache = {}
        m._cur_targets = {}
        m._last_detection_sample = {}
        m._quarantined_visual = lambda *args: False
        m._appearance_rejected = lambda *args: False
        m._update_tracker_position = Mock()
        m.tracker = cooperative_tracker.CooperativeTracker(
            official_only=True, strict_samples=True)
        for tid, uid in m._tracking.items():
            m.tracker.add_target(tid, now=90.)
            m.tracker.targets[tid].observers.add(uid)
        m.tracker.report = Mock(wraps=m.tracker.report)
        return m

    def _east_white_frame(self, m, sample, candidate='white-person', x=121.,
                          accepted=True, source='u0', distance=13.5,
                          purpose=None):
        # Exact /swarm/visual_source schema emitted by yolo_target_bridge.
        payload = dict(target_id='t3', source_uav=source,
                       candidate_id=candidate, local_target_id='t3',
                       sample_s=sample, x=x, y=24., real_detection=True,
                       confidence=.75, range_m=distance,
                       official_view_ok=accepted)
        if purpose is not None:
            payload['purpose'] = purpose
        m.last_report = {uid: sm.rospy.Time.from_sec(sample) for uid in m.uav_ids}
        with patch.object(sm.rospy.Time, 'now',
                          return_value=sm.rospy.Time.from_sec(sample+0.05)):
            m._visual_source_cb(NS(data=json.dumps(payload)))
            m._handoff_distant_tracker('t3', x, 24., m.tracker.targets['t3'])

    def test_continuous_distant_camera_retakes_blind_red_task_for_white(self):
        m = self._east_visual_owner_scenario()
        for sample in (100., 100.4):
            self._east_white_frame(m, sample)
            self.assertEqual(m._tracking['t3'], 'u3')
        self._east_white_frame(m, 100.8)
        self.assertFalse(m._visual_observations[('u0', 't3')]['eligible'])
        self.assertEqual(m._visual_observations[('u0', 't3')]['dispatch_hits'], 3)
        m.tracker.report.assert_not_called()  # 13 m never becomes judge evidence
        self.assertEqual(m._tracking['t3'], 'u0')
        self.assertNotIn('t4', m._tracking)  # red2 returns to pending
        self.assertIn('t4', m.tracker.targets)
        self.assertEqual(m._tracking['t5'], 'u5')  # red1 camera is protected
        self.assertEqual(m._tracking['t0'], 'u1')
        self.assertFalse(m._eliminated)
        self.assertEqual([(c.args[0].uav_id, c.args[0].target_id,
                           c.args[0].task_type)
                          for c in m.assign_pub.publish.call_args_list],
                         [('u0', 't3', 1), ('u0', 't4', 255), ('u3', 't3', 255)])

    def test_retask_clears_old_view_recovery_without_completing_target(self):
        m = self._east_visual_owner_scenario()
        m._view_recovery, m._tenure_recovery = {'t4': {'first_s': 90.}}, {'t4': {}}
        for sample in (100., 100.4, 100.8):
            self._east_white_frame(m, sample)
        self.assertNotIn('t4', m._view_recovery)
        self.assertNotIn('t4', m._tenure_recovery)
        self.assertIn('t4', m.tracker.targets)
        self.assertFalse(m.tracker.targets['t4'].eliminated)

    def test_active_old_image_streak_cannot_be_stolen_by_other_person(self):
        m = self._east_visual_owner_scenario()
        m.tracker.targets['t4'].last_ok_t = 100.6
        m._eligible_observations[('u0', 't4')] = dict(
            sample_s=100.6, since_s=89., hits=40, eligible=True,
            x=122., y=33., candidate_id='red2-person')
        for sample in (100., 100.4, 100.8):
            self._east_white_frame(m, sample)
        self.assertEqual(m._tracking['t4'], 'u0')
        self.assertEqual(m._tracking['t3'], 'u3')
        m.assign_pub.publish.assert_not_called()

    def test_changed_camera_instance_or_jump_cannot_retake_owner(self):
        m = self._east_visual_owner_scenario()
        self._east_white_frame(m, 100., candidate='person-A')
        self._east_white_frame(m, 100.4, candidate='person-B')
        self._east_white_frame(m, 100.8, candidate='person-B')
        self.assertEqual(m._tracking['t3'], 'u3')
        self._east_white_frame(m, 101.2, candidate='person-B', x=130.)
        self.assertEqual(m._tracking['t3'], 'u3')

    def test_bridge_rejected_frames_cannot_borrow_backup_or_primary(self):
        m = self._east_visual_owner_scenario()
        m.status['u4'].x, m.status['u4'].y = 135., 20.
        m._backup = {'t5': 'u4'}
        m.tracker.targets['t5'].observers.add('u4')
        for sample in (100., 100.4, 100.8):
            self._east_white_frame(m, sample, accepted=False, source='u4')
            self._east_white_frame(m, sample, accepted=False, source='u0')
        self.assertEqual(m._backup['t5'], 'u4')
        self.assertEqual(m._tracking['t4'], 'u0')
        self.assertEqual(m._tracking['t3'], 'u3')
        self.assertFalse(m._visual_observations[('u4', 't3')].get('dispatch_hits'))
        m.assign_pub.publish.assert_not_called()

    def test_bridge_approved_far_view_does_not_expand_backup_handoff(self):
        m = self._east_visual_owner_scenario()
        m.status['u4'].x, m.status['u4'].y = 135., 20.
        m._backup = {'t5': 'u4'}
        m.tracker.targets['t5'].observers.add('u4')
        for sample in (100., 100.4, 100.8):
            self._east_white_frame(m, sample, accepted=True, source='u4')
        self.assertTrue(m._sustained_distant_view('u4', 't3', 100.85))
        self.assertEqual(m._backup['t5'], 'u4')
        self.assertEqual(m._tracking['t3'], 'u3')
        m.assign_pub.publish.assert_not_called()

    def test_free_aircraft_can_handoff_with_coherent_bridge_accepted_far_images(self):
        m = self._east_visual_owner_scenario()
        m.status['u2'].x, m.status['u2'].y = 135., 20.
        for accepted in (False, True):
            for sample in (100., 100.4, 100.8):
                self._east_white_frame(m, sample + (1. if accepted else 0.),
                                       source='u2', accepted=accepted)
        self.assertEqual(m._tracking['t3'], 'u2')
        self.assertNotIn(('u2', 't3'), m._eligible_observations)
        self.assertIsNone(m.tracker.targets['t3'].confirm_since)

    def test_distant_same_frame_upgrade_does_not_create_extra_evidence(self):
        m = self._east_visual_owner_scenario()
        self._east_white_frame(m, 100., accepted=False, purpose='discovery_only')
        self._east_white_frame(m, 100.)
        self._east_white_frame(m, 100.)
        image = m._visual_observations[('u0', 't3')]
        self.assertEqual(image['dispatch_hits'], 1)
        self.assertFalse(m._sustained_distant_view('u0', 't3', 100.05))
        # Preserve the pre-existing same-frame eligibility upgrade contract.
        self._east_white_frame(m, 100., distance=11.5)
        self._east_white_frame(m, 100., distance=11.5)
        self.assertTrue(m._visual_observations[('u0', 't3')]['eligible'])
        self.assertEqual(m._last_eligible('u0', 't3')['hits'], 1)
        self.assertEqual(m._tracking['t3'], 'u3')
        m.assign_pub.publish.assert_not_called()

    def test_qualified_near_view_can_retask_blind_primary(self):
        m = self._east_visual_owner_scenario()
        m.status['u0'].x, m.status['u0'].y = 132., 24.
        for sample in (100., 100.4, 100.8):
            self._east_white_frame(m, sample, distance=11.)
        self.assertEqual(m._tracking['t3'], 'u0')
        self.assertNotIn('t4', m._tracking)

    def test_blind_owner_within_25m_allows_continuous_camera_handoff(self):
        m = self.m
        m._tracking, m._tracking_since = {'t0':'u0'}, {'t0':50.}
        m._eligible_observations[('u1','t0')] = dict(
            sample_s=100., since_s=99.4, hits=4, eligible=True, x=5., y=0.)
        m._handoff_distant_tracker('t0', 5., 0., m.tracker.targets['t0'])
        self.assertEqual(m._tracking, {'t0':'u1'})

    def test_prediction_is_independent_of_dispatch_rate_and_duplicate_frames(self):
        for dispatch_rate in (1, 5, 20):
            m = self.m
            m._target_hist = {}
            with patch.object(sm.rospy.Time, 'now') as clock:
                for t in range(1, 7):
                    m._record_target_observation('t0', 'u0', t, float(t), 0.)
                    m._record_target_observation('t0', 'u0', t, 100., 0.)
                    m._record_target_observation('t0', 'u0', t-.2, -100., 0.)
                    for k in range(dispatch_rate):
                        clock.return_value = sm.rospy.Time.from_sec(t+k/dispatch_rate)
                        prediction = m._predict_target('t0', float(t), 0., 1.)
                self.assertEqual(prediction, (7., 0.))
                clock.return_value = sm.rospy.Time.from_sec(7.1)
                self.assertEqual(m._predict_target('t0', 6., 0., 1.), (6., 0.))

    def test_camera_change_cannot_create_motion_from_source_offset(self):
        m = self.m
        for t in (99., 99.5, 100.):
            m._record_target_observation('t0', 'u0', t, 0., 0.)
            m._record_target_observation('t0', 'u1', t, 1.2, 0.)
        self.assertEqual(m._predict_target('t0', 1.2, 0., 1.), (1.2, 0.))

    def test_healthy_primary_is_not_replaced_just_because_peer_is_closer(self):
        m = self.m
        m._tracking = {'t0': 'u0'}
        m.status['u1'].x = 38.
        m._eligible_observations = {(uid, 't0'): dict(
            sample_s=100., since_s=99.4, hits=3, eligible=True, x=40., y=0.)
            for uid in m.uav_ids}
        m._handoff_distant_tracker('t0', 40., 0., m.tracker.targets['t0'])
        self.assertEqual(m._tracking, {'t0': 'u0'})
        m.assign_pub.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
