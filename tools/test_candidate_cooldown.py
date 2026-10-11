"""A failed visual instance cannot cool down its independently visible neighbor."""
import json
import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

os.environ.setdefault('ROBOCUP_LOG_DIR', '/tmp/robocup_decoy_tests')
sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_discovery_dispatch as fixtures


class CandidateCooldownTests(unittest.TestCase):
    setUp = fixtures.DiscoveryDispatchTests.setUp
    payload = fixtures.DiscoveryDispatchTests.payload
    old_box = [100., 120., 160., 280.]
    new_box = [180., 120., 240., 280.]

    def failure(self):
        m = self.m
        m._discovery_pool = {}
        m._visual_source['t1'] = ('u0', 100., 8., 0.)
        m._visual_observations[('u0', 't1')] = dict(
            source_uav='u0', candidate_id='session:old', sample_s=100.,
            x=8., y=0., confidence=.8, eligible=False)
        m._defer_failed_view('t1', ('u0',), 100.)
        self.now = 106.

    def batch(self, *items, source='u0', sample=None):
        packet = dict(schema_version=2, source_uav=source,
                      sample_s=self.now if sample is None else sample,
                      real_detection=True, purpose='discovery_only', candidates=list(items))
        self.m._discovery_cb(NS(data=json.dumps(packet)))

    def pair(self, other_color=False, **changes):
        old = self.payload(candidate_id='session:old', x=8., box=self.old_box,
                           appearance_status='ambiguous')
        new = self.payload(candidate_id='session:new', x=8.2,
                           target_id='t3' if other_color else 't1', box=self.new_box,
                           appearance_status='actor')
        new.update(changes)
        return old, new

    def allowed(self, tid='t1', cid='session:new', source='u0', x=8.2):
        return self.m._failed_view_dispatch_ok(
            tid, x, 0., self.now, candidate_source=source, candidate_id=cid)

    def test_failure_remembers_original_camera_instance_and_image_time(self):
        self.failure()
        record = self.m._failed_views['t1']
        self.assertEqual((record['source_uav'], record['candidate_id'], record['image_sample_s']),
                         ('u0', 'session:old', 100.))
        self.assertEqual((record['attempts'], record['until']), (1, 115.))

    def test_same_colour_neighbor_in_complete_frame_bypasses_only_other_failure(self):
        self.failure()
        self.batch(*self.pair())
        self.assertTrue(self.allowed())
        self.assertFalse(self.allowed(cid='session:old', x=8.))
        self.assertEqual(self.m._visual_source['t1'][2:4], (8.2, 0.))
        self.assertEqual(self.m._visual_observations[('u0', 't1')]['candidate_id'], 'session:new')
        self.m.tracker.report.assert_not_called()

    def test_complete_frame_order_does_not_change_cooldown_exception(self):
        self.failure()
        self.batch(*self.pair()[::-1])
        self.assertTrue(self.allowed())
        self.assertEqual(self.m._visual_source['t1'][2:4], (8.2, 0.))

    def test_cross_colour_discovery_needs_no_prior_official_qualification(self):
        self.failure()
        self.batch(*self.pair(other_color=True))
        self.assertTrue(self.allowed(tid='t3'))
        self.assertIn('t3', self.m.tracker.targets)
        self.assertEqual(self.m._eligible_observations, {})
        self.m.tracker.report.assert_not_called()

    def test_changing_id_alone_at_failed_place_cannot_bypass(self):
        self.failure()
        self.batch(self.payload(candidate_id='session:new', x=8.2, box=self.old_box))
        self.assertFalse(self.allowed())
        self.assertEqual(self.m.tracker.targets, {})

    def test_same_object_double_box_with_two_ids_is_not_independence(self):
        self.failure()
        self.batch(*self.pair(box=self.old_box))
        self.assertFalse(self.allowed())

    def test_contained_box_cannot_masquerade_as_second_person(self):
        self.failure()
        self.batch(*self.pair(box=[110., 140., 150., 220.]))
        self.assertFalse(self.allowed())

    def test_slight_overlap_between_distinct_people_is_allowed(self):
        self.failure()
        self.batch(*self.pair(box=[140., 120., 200., 280.]))
        self.assertTrue(self.allowed())

    def test_distinct_images_must_not_be_combined_into_one_frame(self):
        self.failure()
        old, _ = self.pair()
        self.batch(old)
        self.now += .2
        _, new = self.pair()
        self.batch(new)
        self.assertFalse(self.allowed())

    def test_legacy_separate_messages_cannot_claim_complete_frame_proof(self):
        self.failure()
        for row in self.pair():
            self.m._discovery_cb(NS(data=json.dumps(row)))
        self.assertFalse(self.allowed())

    def test_forged_child_source_or_timestamp_cannot_create_proof(self):
        self.failure()
        for change in (dict(source_uav='u1'), dict(sample_s=105.9)):
            self.batch(*self.pair(**change))
            self.assertFalse(self.allowed())
        self.assertEqual(self.m._discovery_pool, {})

    def test_unknown_camera_or_future_image_cannot_create_proof(self):
        self.failure()
        rows = [dict(row, source_uav='unknown') for row in self.pair()]
        self.batch(*rows, source='unknown')
        future = [dict(row, sample_s=107.) for row in self.pair()]
        self.batch(*future, sample=107.)
        self.assertFalse(self.allowed())
        self.assertEqual(self.m.tracker.targets, {})

    def test_nonfinite_or_malformed_boxes_never_exempt_cooldown(self):
        for box in ([float('nan'), 120., 240., 280.],
                    [180., 120., float('inf'), 280.],
                    [240., 120., 180., 280.], [180., 120., 180., 280.],
                    [180., 120., 240.], '1234', None):
            self.m._failed_views = {}
            self.failure()
            with self.subTest(box=box):
                self.batch(*self.pair(box=box))
                self.assertFalse(self.allowed())

    def test_verified_independent_identity_survives_when_failed_object_leaves_view(self):
        self.failure()
        self.batch(*self.pair())
        self.now += .4
        _, new = self.pair()
        self.batch(new)
        self.assertTrue(self.allowed())
        # The normal joint-dispatch path has no explicit candidate argument;
        # it recovers exactly the selected camera identity, not all nearby IDs.
        self.assertTrue(self.m._failed_view_dispatch_ok('t1', 8.2, 0., self.now))

    def test_verified_exception_cannot_be_inherited_by_a_rebuilt_id(self):
        self.failure()
        self.batch(*self.pair())
        self.now += .4
        _, new = self.pair(candidate_id='session:rebuilt')
        self.batch(new)
        self.assertFalse(self.allowed(cid='session:rebuilt'))
        self.assertTrue(self.allowed())

    def test_new_neighbor_failure_does_not_increment_old_objects_attempts(self):
        self.failure()
        self.batch(*self.pair())
        self.m._defer_failed_view('t1', ('u0',), self.now)
        self.assertEqual(self.m._failed_views['t1']['attempts'], 1)
        self.assertEqual(self.m._failed_views['t1']['candidate_id'], 'session:new')

    def test_color_flash_at_different_times_is_not_simultaneous_eligible_evidence(self):
        self.failure()
        for tid, sample in (('t1', 105.8), ('t3', 105.9)):
            self.m._eligible_observations[('u0', tid)] = dict(
                sample_s=sample, eligible=True, x=8., y=0., hits=10, since_s=103.)
        self.assertFalse(self.m._failed_view_dispatch_ok('t3', 8., 0., self.now))

    def test_different_camera_colors_do_not_claim_one_camera_saw_both(self):
        self.failure()
        for source, tid in (('u0', 't1'), ('u1', 't3')):
            self.m._eligible_observations[(source, tid)] = dict(
                sample_s=105.9, eligible=True, x=8., y=0., hits=10, since_s=103.)
        self.assertFalse(self.m._failed_view_dispatch_ok('t3', 8., 0., self.now))

    def test_empty_frame_cannot_make_or_refresh_independence(self):
        self.failure()
        self.batch()
        self.assertFalse(self.allowed())
        self.assertNotIn('independent_instances', self.m._failed_views['t1'])


if __name__ == '__main__':
    unittest.main()
