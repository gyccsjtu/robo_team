"""End-to-end manager callbacks for same-color actor/distractor frames."""
import json
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock

import test_discovery_dispatch as fixtures


class CandidateDispatchTests(unittest.TestCase):
    setUp = fixtures.DiscoveryDispatchTests.setUp
    payload = fixtures.DiscoveryDispatchTests.payload

    def batch(self, *items, **changes):
        packet = dict(schema_version=2, source_uav='u0', sample_s=self.now,
                      real_detection=True, purpose='discovery_only', candidates=list(items))
        packet.update(changes)
        self.m._discovery_cb(NS(data=json.dumps(packet)))

    def source(self, **changes):
        data = self.payload(official_view_ok=True, **changes)
        data.pop('purpose')
        self.m._visual_source_cb(NS(data=json.dumps(data)))

    def test_frame_order_cannot_hide_better_same_color_actor(self):
        first = self.payload(candidate_id='statue', confidence=.65, x=8.)
        second = self.payload(candidate_id='person', confidence=.9, x=20.)
        for items in ((first, second), (second, first)):
            self.m._discovery_pool = {}
            self.m._discovery_selected_s = {}
            self.m._visual_observations = {}
            self.m._visual_source = {}
            self.m._get_target_pos = lambda *args: (None, None)
            self.m._discovery_state_pub.reset_mock()
            self.batch(*items)
            self.assertEqual(len(self.m._discovery_pool), 2)
            states = self.m._discovery_state_pub.publish.call_args_list
            self.assertEqual(len(states), 1)
            self.assertEqual((states[0].args[0].x, states[0].args[0].y), (20., 0.))

    def test_ambiguous_object_gets_bounded_look_not_tracking_credit(self):
        queue = Mock(return_value=(1, 2))
        self.m.allocator = NS(queue_revisit=queue)
        item = self.payload(appearance_status='ambiguous')
        self.batch(item)
        self.assertEqual(self.m.tracker.targets, {})
        self.m.tracker.report.assert_not_called()
        queue.assert_called_once()
        self.assertEqual(queue.call_args.kwargs['ttl_s'], 8.)
        self.now += .2
        self.batch(self.payload(appearance_status='ambiguous'))
        queue.assert_called_once()
        self.assertEqual(self.m._candidate_looks[('u0', '10')]['expires_s'], 108.)

    def test_active_owner_kept_but_other_person_gets_look(self):
        self.batch(self.payload())
        self.m._tracking['t1'] = 'u0'
        self.m.allocator = NS(queue_revisit=Mock(return_value=(2, 2)))
        self.now += .2
        self.batch(self.payload(candidate_id='second', x=30.))
        self.assertEqual(self.m._tracking, {'t1': 'u0'})
        self.assertEqual(self.m._get_target_pos('t1', self.now), (8., 0.))
        self.m.allocator.queue_revisit.assert_called_once()
        self.assertEqual(self.m._eligible_observations, {})

    def test_near_high_confidence_same_camera_instance_cannot_move_active_anchor(self):
        self.batch(self.payload(candidate_id='person'))
        self.m._tracking['t1'] = 'u0'
        self.m.allocator = NS(queue_revisit=Mock(return_value=(2, 2)))
        self.now += .2
        self.batch(self.payload(candidate_id='statue', x=9., confidence=.99),
                   self.payload(candidate_id='person', x=8., confidence=.7))
        self.assertEqual(self.m._get_target_pos('t1', self.now), (8., 0.))
        self.assertEqual(self.m._visual_observations[('u0', 't1')]['candidate_id'], 'person')
        self.assertEqual(self.m._discovery_state_pub.publish.call_args.args[0].x, 8.)
        self.assertIn(('u0', 'statue'), self.m._candidate_looks)
        self.assertEqual(self.m._tracking, {'t1': 'u0'})

    def test_near_new_camera_waits_for_bridge_qualification(self):
        self.batch(self.payload(candidate_id='person'))
        self.m._tracking['t1'] = 'u0'
        self.m.allocator = NS(queue_revisit=Mock(return_value=(2, 2)))
        self.now += .2
        self.batch(self.payload(candidate_id='unbound', source_uav='u1',
                                x=8.1, confidence=.99), source_uav='u1')
        self.assertEqual(self.m._get_target_pos('t1', self.now), (8., 0.))
        self.assertEqual(self.m._visual_source['t1'][0], 'u0')
        self.assertNotIn(('u1', 't1'), self.m._visual_observations)
        self.assertIn(('u1', 'unbound'), self.m._candidate_looks)

    def test_near_candidates_use_distance_before_confidence_without_live_owner(self):
        self.batch(self.payload(candidate_id='person'))
        self.now += .2
        self.batch(self.payload(candidate_id='statue', x=10., confidence=.99),
                   self.payload(candidate_id='person', x=8.2, confidence=.7))
        self.assertEqual(self.m._get_target_pos('t1', self.now), (8.2, 0.))
        self.assertEqual(self.m._visual_observations[('u0', 't1')]['candidate_id'], 'person')

    def test_bridge_different_instance_upgrades_same_frame_discovery_everywhere(self):
        self.batch(self.payload(candidate_id='statue', x=9., confidence=.99),
                   self.payload(candidate_id='person', x=8., confidence=.8))
        self.assertEqual(self.m._get_target_pos('t1', self.now), (9., 0.))
        self.source(candidate_id='person', x=8.)
        self.source(candidate_id='person', x=8.)
        self.assertEqual(self.m._get_target_pos('t1', self.now), (8., 0.))
        self.assertEqual(self.m._visual_source['t1'], ('u0', self.now, 8., 0.))
        self.assertEqual(self.m._reacquire_sites['t1']['x'], 8.)
        self.assertEqual(self.m._cur_targets['t1'], (8., 0., 'u0'))
        self.assertEqual(self.m._eligible_observations[('u0', 't1')]['candidate_id'], 'person')
        self.assertTrue(self.m._visual_observations[('u0', 't1')]['eligible'])
        self.assertEqual(self.m.tracker.report.call_count, 1)

    def test_bridge_first_then_same_frame_discovery_keeps_qualified_instance(self):
        self.source(candidate_id='person', x=8.)
        self.batch(self.payload(candidate_id='statue', x=9., confidence=.99),
                   self.payload(candidate_id='person', x=8., confidence=.8))
        self.assertEqual(self.m._get_target_pos('t1', self.now), (8., 0.))
        self.assertEqual(self.m._visual_source['t1'], ('u0', self.now, 8., 0.))
        self.assertEqual(self.m._visual_observations[('u0', 't1')]['candidate_id'], 'person')
        self.assertTrue(self.m._visual_observations[('u0', 't1')]['eligible'])
        self.assertEqual(self.m.tracker.report.call_count, 1)
        self.m._discovery_state_pub.publish.assert_not_called()

    def test_newer_discovery_cannot_downgrade_a_bridge_qualified_source(self):
        self.source(candidate_id='person', x=8.)
        self.now += .6  # beyond the old half-second protective shortcut
        self.batch(self.payload(candidate_id='person', x=8.5))
        self.assertTrue(self.m._visual_observations[('u0', 't1')]['eligible'])
        self.assertEqual(self.m._visual_observations[('u0', 't1')]['sample_s'], 100.)
        self.assertEqual(self.m._get_target_pos('t1', self.now), (8., 0.))
        self.m._discovery_state_pub.publish.assert_not_called()

    def test_expired_qualified_peer_cannot_authorize_a_new_active_anchor(self):
        self.source(source_uav='u1', candidate_id='peer', x=8.)
        self.now = 159.9
        raw = self.payload(source_uav='u1', candidate_id='peer', x=9.)
        raw.pop('purpose')  # fresh but unqualified bridge view, not discovery
        self.m._visual_source_cb(NS(data=json.dumps(raw)))
        self.now = 160.
        self.source(candidate_id='primary', x=8.)
        self.m._tracking['t1'] = 'u0'
        self.m.allocator = NS(queue_revisit=Mock(return_value=(2, 2)))
        self.now += .25
        self.batch(self.payload(source_uav='u1', candidate_id='peer', x=9.,
                                confidence=.99), source_uav='u1')
        self.assertEqual(self.m._get_target_pos('t1', self.now), (8., 0.))
        self.assertEqual(self.m._visual_source['t1'][0], 'u0')
        self.assertIn(('u1', 'peer'), self.m._candidate_looks)
        self.assertEqual(self.m._eligible_observations[('u1', 't1')]['sample_s'], 100.)
        self.m._discovery_state_pub.publish.assert_not_called()

    def test_expired_qualified_image_does_not_swallow_same_instance_rediscovery(self):
        self.source(candidate_id='person', x=8.)
        self.m._tracking['t1'] = 'u0'
        self.now += 1.25
        self.batch(self.payload(candidate_id='person', x=8.5))
        self.assertEqual(self.m._get_target_pos('t1', self.now), (8.5, 0.))
        self.assertEqual(self.m._visual_source['t1'][1], self.now)
        self.assertFalse(self.m._visual_observations[('u0', 't1')]['eligible'])
        self.assertEqual(self.m._eligible_observations[('u0', 't1')]['sample_s'], 100.)
        self.assertEqual(self.m.tracker.report.call_count, 1)
        self.assertEqual(self.m._discovery_state_pub.publish.call_args.args[0].x, 8.5)

    def test_legacy_same_frame_upgrade_still_requires_identical_position(self):
        data = self.payload()
        data.pop('candidate_id')
        self.m._visual_source_cb(NS(data=json.dumps(data)))
        self.source(candidate_id='new', x=9.)
        self.assertFalse(self.m._visual_observations[('u0', 't1')]['eligible'])
        self.m.tracker.report.assert_not_called()
        self.source(candidate_id='new', x=8.)
        self.assertTrue(self.m._visual_observations[('u0', 't1')]['eligible'])
        self.assertEqual(self.m.tracker.report.call_count, 1)

    def test_three_people_in_one_frame_enter_distinct_targets(self):
        self.batch(*(self.payload(target_id=tid, candidate_id=str(i), x=8.+i*8.)
                     for i, tid in enumerate(('t0', 't1', 't2'))))
        self.assertEqual(set(self.m.tracker.targets), {'t0', 't1', 't2'})
        self.assertEqual(self.m._discovery_state_pub.publish.call_count, 3)
        self.m.tracker.report.assert_not_called()

    def test_frame_cannot_claim_a_different_camera_or_time(self):
        self.batch(self.payload(source_uav='u1'))
        self.batch(self.payload(sample_s=99.9))
        self.assertEqual(self.m.tracker.targets, {})

    def test_duplicate_instance_frame_rejected_instead_of_last_wins(self):
        self.batch(self.payload(x=8.), self.payload(x=80.))
        self.assertEqual(self.m.tracker.targets, {})

    def test_empty_frame_neither_creates_nor_refreshes_evidence(self):
        self.batch()
        self.assertEqual(self.m.tracker.targets, {})
        self.assertEqual(self.m._visual_source, {})

    def test_same_failed_object_cannot_restart_by_changing_color(self):
        m = self.m
        m._visual_source['t1'] = ('u0', 100., 8., 0.)
        m._defer_failed_view('t1', ('u0',), 100.)
        self.assertFalse(m._failed_view_dispatch_ok('t3', 8.2, 0., 106.))
        self.assertTrue(m._failed_view_dispatch_ok('t3', 10., 0., 106.))
        m._visual_source['t3'] = ('u0', 116., 8.1, 0.)
        m._defer_failed_view('t3', ('u0',), 116.)
        self.assertEqual(m._failed_views['t3']['attempts'], 2)
        self.assertEqual(m._failed_views['t3']['until'], 146.)
        self.assertTrue(m._failed_view_dispatch_ok('t1', 8., 0., 146.))

    def test_two_simultaneously_visible_people_do_not_share_failure(self):
        m = self.m
        m._visual_source['t1'] = ('u0', 100., 8., 0.)
        m._defer_failed_view('t1', ('u0',), 100.)
        for tid in ('t1', 't3'):
            m._eligible_observations[('u0', tid)] = dict(
                sample_s=105.9, eligible=True, x=8., y=0., hits=10, since_s=103.)
        self.assertTrue(m._failed_view_dispatch_ok('t3', 8., 0., 106.))


if __name__ == '__main__':
    unittest.main()
