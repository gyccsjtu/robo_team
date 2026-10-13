"""Own publication-flow diagnostics cannot create observations or authority."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                     'coordination/src/robocup_swarm/scripts'))
from formal_report_flow import ReportFlowHealth


class ReportFlowHealthTests(unittest.TestCase):
    def setUp(self):
        self.book = ReportFlowHealth(['u0', 'u1'])

    @staticmethod
    def image(sample=100., **changes):
        data = dict(target_id='t3', source_uav='u0', candidate_id='camera:17',
                    sample_s=sample, x=129.19, y=35.97, eligible=True,
                    range_m=10.8)
        data.update(changes)
        return data

    @staticmethod
    def publication(sample=100., **changes):
        data = dict(event='published', target_id='t3', tag='white',
                    source_uav='u0', candidate_id='camera:17', sample_s=sample,
                    report_s=sample+.1, x=129.19, y=35.97,
                    report_policy='fresh')
        data.update(changes)
        return data

    def test_white_range_blocked_eligible_images_are_not_publications(self):
        for i in range(30):
            sample = 100.+i*.2
            self.assertTrue(self.book.remember_image(self.image(sample), sample+.1))
        self.assertIsNone(self.book.latest('t3', 106.))

    def test_matched_real_publication_retains_evidence_and_measured_ages(self):
        self.book.remember_image(self.image(), 100.05)
        self.assertTrue(self.book.remember_publish(self.publication(), 100.12))
        result = self.book.latest('t3', 100.2)
        self.assertTrue(result['fresh'])
        self.assertAlmostEqual(result['report_age_s'], .1)
        result['x'] = 0.  # The caller cannot mutate the book.
        old = self.book.latest('t3', 130.2)
        self.assertEqual(old['x'], 129.19)
        self.assertFalse(old['fresh'])
        self.assertAlmostEqual(old['report_age_s'], 30.1)

    def test_publication_can_arrive_before_its_exact_qualified_image(self):
        self.assertFalse(self.book.remember_publish(self.publication(), 100.12))
        self.assertIsNone(self.book.latest('t3', 100.13))
        self.book.remember_image(self.image(), 100.18)
        self.assertEqual(self.book.latest('t3', 100.2)['report_s'], 100.1)

    def test_pending_deadline_is_fixed_and_fit_age_is_checked_again(self):
        payload = self.publication(report_policy='fit')
        self.book.remember_publish(payload, 100.1)
        deadline = next(iter(self.book._pending['t3'].values()))['deadline_s']
        for now in (100.2, 100.3, 100.4):
            self.assertFalse(self.book.remember_publish(payload, now))
            self.assertEqual(next(iter(self.book._pending['t3'].values()))['deadline_s'], deadline)
        self.book.remember_image(self.image(), 100.6001)
        self.assertIsNone(self.book.latest('t3', 100.61))

    def test_old_image_cannot_corroborate_a_fresh_looking_report(self):
        self.book.remember_image(self.image(99.), 100.)
        self.assertFalse(self.book.remember_publish(self.publication(99., report_s=100.), 100.1))
        self.assertIsNone(self.book.latest('t3', 100.2))

    def test_repeated_image_and_repeated_or_retimed_publication_do_not_refresh(self):
        self.book.remember_image(self.image(), 100.05)
        self.book.remember_publish(self.publication(), 100.1)
        self.assertFalse(self.book.remember_image(self.image(), 100.15))
        for report in (self.publication(), self.publication(report_s=100.2)):
            self.assertFalse(self.book.remember_publish(report, 100.2))
        self.assertEqual(self.book.latest('t3', 100.3)['report_s'], 100.1)

    def test_reversed_report_or_sample_cannot_replace_newer_matched_point(self):
        self.book.remember_image(self.image(100.2), 100.22)
        self.book.remember_publish(self.publication(100.2), 100.3)
        self.assertFalse(self.book.remember_publish(self.publication(100., report_s=100.32), 100.33))
        self.book.remember_image(self.image(100.3), 100.34)
        self.assertFalse(self.book.remember_publish(self.publication(100.3, report_s=100.3), 100.35))
        self.assertEqual(self.book.latest('t3', 100.4)['sample_s'], 100.2)

    def test_changed_instance_does_not_inherit_or_refresh_old_publication(self):
        self.book.remember_image(self.image(), 100.05)
        self.book.remember_publish(self.publication(), 100.1)
        self.book.remember_image(self.image(100.2, candidate_id='camera:18'), 100.22)
        old = self.book.latest('t3', 100.23)
        self.assertFalse(old['identity_current'])
        self.assertFalse(old['fresh'])
        self.assertFalse(self.book.remember_publish(self.publication(100.2), 100.3))
        self.assertTrue(self.book.remember_publish(self.publication(100.2, candidate_id='camera:18'), 100.31))

    def test_delayed_old_instance_image_cannot_undo_current_identity(self):
        self.book.remember_image(self.image(100.2, candidate_id='new'), 100.22)
        self.assertFalse(self.book.remember_image(self.image(100.), 100.24))
        self.assertFalse(self.book.remember_publish(self.publication(100.), 100.3))
        self.assertIsNone(self.book.latest('t3', 100.31))

    def test_two_red_streams_keep_separate_qualified_images_and_records(self):
        people = (('t4', 'red2', 'red:a', 10.), ('t5', 'red1', 'red:b', 20.))
        for tid, tag, candidate, x in people:
            self.book.remember_image(self.image(target_id=tid, candidate_id=candidate, x=x), 100.05)
        for tid, tag, candidate, x in people:
            self.assertTrue(self.book.remember_publish(self.publication(target_id=tid,
                tag=tag, candidate_id=candidate, x=x), 100.1))
        self.assertFalse(self.book.remember_publish(self.publication(100.2,
            target_id='t4', tag='red1', candidate_id='red:b', x=20.), 100.3))
        self.assertEqual(self.book.latest('t4', 100.4)['candidate_id'], 'red:a')
        self.assertEqual(self.book.latest('t5', 100.4)['candidate_id'], 'red:b')

    def test_zero_point_contradiction_rejected_but_real_origin_is_legal(self):
        self.book.remember_image(self.image(), 100.05)
        self.assertFalse(self.book.remember_publish(self.publication(x=0., y=0.), 100.1))
        self.book.remember_image(self.image(100.2, x=0., y=0.), 100.22)
        self.assertTrue(self.book.remember_publish(self.publication(100.2, x=0., y=0.), 100.3))

    def test_legal_short_prediction_and_stationary_smoothing_remain_matchable(self):
        for offset, policy in ((.15, 'fresh'), (.5, 'fit'), (.75, 'fit')):
            with self.subTest(offset=offset, policy=policy):
                book = ReportFlowHealth(['u0'])
                book.remember_image(self.image(), 100.05)
                self.assertTrue(book.remember_publish(self.publication(x=129.19+offset,
                                                report_policy=policy), 100.1))
        self.assertFalse(book.remember_publish(self.publication(100.1, x=130.1), 100.2))

    def test_unqualified_unknown_camera_and_wrong_original_image_do_not_match(self):
        for change in (dict(eligible=False), dict(source_uav='u2'), dict(candidate_id=''),
                       dict(sample_s=True), dict(x=float('nan'))):
            with self.subTest(change=change):
                book = ReportFlowHealth(['u0'])
                self.assertFalse(book.remember_image(self.image(**change), 100.1))
                self.assertFalse(book.remember_publish(self.publication(), 100.2))
                self.assertIsNone(book.latest('t3', 100.3))
        self.book.remember_image(self.image(100.1), 100.11)
        self.assertFalse(self.book.remember_publish(self.publication(), 100.2))

    def test_telemetry_events_and_invalid_time_fields_do_not_grant_health(self):
        self.book.remember_image(self.image(), 100.05)
        for changes in (dict(event='visual_lost'), dict(report_s=100.3, sample_s=100.4),
                        dict(report_s=101.), dict(report_s=True), dict(sample_s=float('nan')),
                        dict(x=float('inf')), dict(report_policy='unknown'),
                        dict(target_id='t9'), dict(source_uav='u2')):
            with self.subTest(changes=changes):
                self.assertFalse(self.book.remember_publish(self.publication(**changes), 100.2))
        self.assertIsNone(self.book.latest('t3', 100.3))

    def test_fresh_and_fit_have_distinct_original_image_age_limits(self):
        for policy, report, now, matched in (
                ('fresh', 100.4, 100.6, True),
                ('fresh', 100.40001, 100.5, False),
                ('fit', 100.6, 100.7, True),
                ('fit', 100.60001, 100.7, False)):
            with self.subTest(policy=policy, report=report, now=now):
                book = ReportFlowHealth(['u0'])
                book.remember_image(self.image(), 100.05)
                self.assertEqual(book.remember_publish(self.publication(
                    report_s=report, report_policy=policy), now), matched)

    def test_delayed_actual_publication_matches_but_is_not_currently_fresh(self):
        self.book.remember_image(self.image(), 100.05)
        self.assertTrue(self.book.remember_publish(self.publication(report_s=100.3), 100.55))
        result = self.book.latest('t3', 100.55)
        self.assertEqual(result['report_s'], 100.3)
        self.assertFalse(result['fresh'])
        self.assertAlmostEqual(result['report_age_s'], .25)
        self.assertAlmostEqual(result['sample_age_s'], .55)

    def test_delayed_subscriber_delivery_has_a_finite_receipt_budget(self):
        self.book.remember_image(self.image(), 100.05)
        self.assertFalse(self.book.remember_publish(self.publication(report_s=100.3), 100.90001))
        self.assertIsNone(self.book.latest('t3', 100.91))

    def test_expired_pending_retransmission_cannot_create_another_deadline(self):
        payload = self.publication(report_s=100.2)
        self.assertFalse(self.book.remember_publish(payload, 100.2))
        self.book.latest('t3', 100.701)
        self.assertFalse(self.book._pending['t3'])
        # Still inside the .6 s delivery budget, but already received once.
        self.assertFalse(self.book.remember_publish(payload, 100.71))
        self.assertFalse(self.book._pending['t3'])
        self.book.remember_image(self.image(), 100.72)
        self.assertIsNone(self.book.latest('t3', 100.73))

    def test_receipt_water_cannot_block_its_own_pending_resolution(self):
        payload = self.publication(report_s=100.3)
        self.assertFalse(self.book.remember_publish(payload, 100.55))
        self.book.remember_image(self.image(), 100.6)
        result = self.book.latest('t3', 100.61)
        self.assertEqual(result['report_s'], 100.3)
        self.assertFalse(result['fresh'])

    def test_reset_preserves_water_marks_and_discards_pending_and_old_images(self):
        self.book.remember_image(self.image(), 100.05)
        self.book.remember_publish(self.publication(), 100.1)
        self.book.remember_publish(self.publication(100.2), 100.3)
        self.book.reset('t3', barrier_s=100.15)
        self.assertIsNone(self.book.latest('t3', 100.31))
        self.assertFalse(self.book.remember_image(self.image(), 100.32))
        self.assertFalse(self.book.remember_publish(self.publication(), 100.33))
        self.book.remember_image(self.image(100.4), 100.42)
        self.assertTrue(self.book.remember_publish(self.publication(100.4), 100.5))
        self.assertFalse(self.book.remember_publish(self.publication(100.2), 100.51))

    def test_reset_without_barrier_blocks_pre_reset_unmatched_publication(self):
        self.book.remember_publish(self.publication(), 100.12)
        self.book.reset('t3')
        self.assertFalse(self.book.remember_image(self.image(), 100.15))
        self.assertFalse(self.book.remember_publish(self.publication(), 100.2))

    def test_new_hypothesis_barrier_may_precede_its_first_image_callback(self):
        self.book.reset('t3', barrier_s=100.)
        self.assertTrue(self.book.remember_image(self.image(candidate_id='new'), 100.1))
        self.assertFalse(self.book.remember_publish(self.publication(candidate_id='old'), 100.12))
        self.assertFalse(self.book.remember_publish(self.publication(source_uav='u1',
                                      candidate_id='old'), 100.13))
        self.assertTrue(self.book.remember_publish(self.publication(candidate_id='new'), 100.14))
        self.book.remember_image(self.image(100.2), 100.22)
        self.assertTrue(self.book.remember_publish(self.publication(100.2), 100.3))

    def test_storage_is_bounded_and_old_images_and_pending_expire(self):
        for i in range(80):
            sample = 100.+i*.003
            self.book.remember_image(self.image(sample), sample+.001)
        self.assertEqual(len(self.book._images['t3']), 16)
        for i in range(80):
            sample = 100.3+i*.001
            self.book.remember_publish(self.publication(sample, source_uav='u1'), sample+.1)
        self.assertEqual(len(self.book._pending['t3']), 16)
        self.book.latest('t3', 103.)
        self.assertFalse(self.book._images['t3'])
        self.assertFalse(self.book._pending['t3'])
        self.assertEqual(len(self.book._images), 6)


if __name__ == '__main__':
    unittest.main()
