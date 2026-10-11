"""Physical instances survive frame batching and appearance conflict handling."""
import json
import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

os.environ.setdefault('ROBOCUP_LOG_DIR', '/tmp/robocup_decoy_tests')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from perception import perception_real as pr


class AppearanceInstanceTests(unittest.TestCase):
    names = pr.CLASSES + ['distractor_person']

    def setUp(self):
        self.mapping = pr.target_class_mapping(self.names)
        self.resolver = pr.ActorDistractorResolver()

    def box(self, cls=2, conf=.7, xy=(300., 150., 360., 310.)):
        return NS(cls=cls, conf=conf, xyxy=[list(xy)])

    def resolve(self, boxes, sample):
        return self.resolver.resolve(boxes, self.mapping, self.names, sample)[0]

    def test_five_class_model_retains_all_same_colour_instances(self):
        a, b = self.box(), self.box(xy=(390., 150., 450., 310.))
        result, evidence = self.resolver.resolve(
            [a, b], pr.target_class_mapping(pr.CLASSES), pr.CLASSES, 100.)
        self.assertEqual(len(result), 2)
        self.assertTrue(all(b.appearance_status == 'actor' for b in result))
        self.assertEqual(evidence, [])
        self.assertFalse(hasattr(a, 'appearance_status'))

    def test_increasing_camera_rate_preserves_candidate_and_negative_evidence(self):
        for hz in (5, 10, 12, 20):
            tracker = pr.PixelCueTracker()
            resolver = pr.ActorDistractorResolver()
            cues = []
            for k in range(hz+1):
                sample = 100.+k/hz
                cues = tracker.update(sample, [dict(cls='white', conf=.95,
                    u=376., v=260., h=60., bearing=0., cam_x=0., cam_y=0.)])
                boxes, _ = resolver.resolve([self.box(), self.box(cls=5, conf=.96)],
                    self.mapping, self.names, sample)
            self.assertTrue(cues, hz)
            self.assertEqual(boxes, [], hz)

    def test_distractor_only_keeps_diagnostic_evidence_without_actor(self):
        result, evidence = self.resolver.resolve(
            [self.box(cls=5, conf=.98)], self.mapping, self.names, 100.)
        self.assertEqual(result, [])
        self.assertEqual(evidence[0]['status'], 'distractor')

    def test_single_strong_distractor_conflict_remains_attention_only(self):
        result = self.resolve([self.box(), self.box(cls=5, conf=.96)], 100.)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].appearance_status, 'ambiguous')
        self.assertEqual(result[0].distractor_confidence, .96)

    def test_only_three_distinct_sustained_frames_can_suppress_same_object(self):
        boxes = [self.box(), self.box(cls=5, conf=.96)]
        for sample in (100., 100., 100., 100.2):
            self.assertEqual(len(self.resolve(boxes, sample)), 1)
        self.assertEqual(self.resolve(boxes, 100.4), [])

    def test_too_short_series_or_large_gap_cannot_suppress(self):
        boxes = [self.box(), self.box(cls=5, conf=.96)]
        for sample in (100., 100.05, 100.1, 101., 101.2):
            self.assertEqual(len(self.resolve(boxes, sample)), 1)

    def test_small_confidence_margin_never_suppresses(self):
        boxes = [self.box(conf=.86), self.box(cls=5, conf=.97)]
        for sample in (100., 100.2, 100.4, 100.6):
            result = self.resolve(boxes, sample)
            self.assertEqual(result[0].appearance_status, 'ambiguous')

    def test_sustained_actor_margin_resolves_conflict_without_motion(self):
        boxes = [self.box(conf=.96), self.box(cls=5, conf=.6)]
        for sample in (100., 100.2, 100.4):
            result = self.resolve(boxes, sample)
        self.assertEqual(result[0].appearance_status, 'actor')

    def test_disagreeing_new_frame_resets_class_evidence(self):
        strong = [self.box(), self.box(cls=5, conf=.96)]
        self.resolve(strong, 100.)
        self.resolve(strong, 100.2)
        self.resolve([self.box(conf=.94), self.box(cls=5, conf=.96)], 100.4)
        self.assertEqual(len(self.resolve(strong, 100.6)), 1)

    def test_adjacent_overlapping_real_actor_survives_in_both_input_orders(self):
        # The true blue overlaps the statue by 1/3; global class-agnostic
        # suppression would erase useful independent-instance evidence.
        for reverse in (False, True):
            self.resolver = pr.ActorDistractorResolver()
            boxes = [self.box(), self.box(cls=5, conf=.96),
                     self.box(conf=.82, xy=(330., 150., 390., 310.))]
            if reverse:
                boxes.reverse()
            for sample in (100., 100.2, 100.4):
                result = self.resolve(boxes, sample)
            self.assertEqual(len(result), 1)
            self.assertEqual(result[0].xyxy[0], [330., 150., 390., 310.])
            self.assertEqual(result[0].appearance_status, 'actor')

    def test_nonunique_near_identical_instances_are_not_suppressed(self):
        boxes = [self.box(), self.box(conf=.6, xy=(301., 150., 361., 310.)),
                 self.box(cls=5, conf=.99)]
        for sample in (100., 100.2, 100.4, 100.6):
            result = self.resolve(boxes, sample)
            self.assertEqual(len(result), 2)
            self.assertTrue(all(b.appearance_status == 'ambiguous' for b in result))

    def test_weak_distractor_score_does_not_block_true_actor(self):
        result = self.resolve([self.box(conf=.95), self.box(cls=5, conf=.4)], 100.)
        self.assertEqual(result[0].appearance_status, 'actor')

    def test_two_nearby_instances_cannot_inherit_one_old_class_history(self):
        boxes = [self.box(), self.box(cls=5, conf=.96)]
        self.resolve(boxes, 100.)
        self.resolve(boxes, 100.2)
        # Both current physical instances can geometrically match the old
        # appearance track. Neither gets its previous two classification votes.
        boxes += [self.box(xy=(320., 150., 380., 310.)),
                  self.box(cls=5, conf=.96, xy=(320., 150., 380., 310.))]
        result = self.resolve(boxes, 100.4)
        self.assertEqual(len(result), 2)
        self.assertTrue(all(b.appearance_status == 'ambiguous' for b in result))

    def test_ambiguous_pixel_cue_keeps_existing_bounded_attention(self):
        tracker = pr.PixelCueTracker()
        box = dict(cls='blue', conf=.95, u=330., v=230., h=160., bearing=0.,
                   appearance_status='ambiguous', distractor_confidence=.96)
        for sample in (100., 100.2, 100.4):
            cues = tracker.update(sample, [box])
        self.assertEqual(len(cues), 1)
        self.assertTrue(cues[0]['attention_only'])
        self.assertEqual(cues[0]['appearance_status'], 'ambiguous')


class CandidateFrameTests(unittest.TestCase):
    def test_association_distance_gate_and_appearance_rank_are_independent(self):
        choices = [(.20, .60), (.15, .95)]
        for order in (choices, choices[::-1]):
            winner = min(order, key=lambda p: pr.association_score(*p, 1., True))
            self.assertEqual(winner, (.15, .95))
        self.assertIsNone(pr.association_score(1.1, 1., 1., True))

    def test_confirmed_track_preserves_model_confidence(self):
        for confidence in (.4, .8):
            row = self.batch([self.track(confidence=confidence)])['candidates'][0]
            self.assertAlmostEqual(row['confidence'], confidence)
            self.assertTrue(row['track_confirmed'])

    def track(self, cls='blue', x=8., confidence=.7, **kwargs):
        tr = pr.Track(cls, x, 0., confidence, (376., 400.), (60., 140.),
                      8., 1.75, 100., **kwargs)
        for i in range(1, 6):
            tr.update(x, 0., .2, confidence, tr.uv, tr.wh, 8., 1.75,
                      100.+i*.2, **kwargs)
        return tr

    def batch(self, tracks, session='session-a'):
        return pr.report_candidate_frame(
            [(tr.cls, 0, tr) for tr in tracks], 'u0', 101., session, (0., 0.))

    def test_same_frame_same_colour_candidates_are_order_independent(self):
        a, b = self.track(x=8.), self.track(x=20., confidence=.97)
        first, second = self.batch([a, b]), self.batch([b, a])
        self.assertEqual(first, second)
        self.assertEqual(len(first['candidates']), 2)
        self.assertEqual({c['xyz'][0] for c in first['candidates']}, {8., 20.})
        self.assertEqual(len({c['candidate_id'] for c in first['candidates']}), 2)
        self.assertEqual(first['schema_version'], 2)
        self.assertTrue(all(c['sample_s'] == first['sample_s'] and
                            c['source_uav'] == first['source_uav']
                            for c in first['candidates']))
        json.dumps(first, allow_nan=False)

    def test_candidate_identity_stays_stable_and_changes_with_process_session(self):
        tr = self.track()
        a = self.batch([tr])['candidates'][0]
        again = self.batch([tr])['candidates'][0]
        restarted = self.batch([tr], 'session-b')['candidates'][0]
        self.assertEqual(a['candidate_id'], again['candidate_id'])
        self.assertNotEqual(a['candidate_id'], restarted['candidate_id'])
        self.assertNotEqual(a['observation_id'], restarted['observation_id'])

    def test_stationary_blue_and_white_report_without_motion_exception(self):
        for cls in ('blue', 'white'):
            tr = self.track(cls=cls)
            self.assertEqual(tr.verdict(101.)[1], 'static')
            self.assertTrue(tr.verdict(101., motion_independent=True)[0])
            self.assertTrue(self.batch([tr])['candidates'][0]['official_view_ok'])

    def test_ambiguous_instance_survives_but_has_no_official_qualification(self):
        tr = self.track(appearance_status='ambiguous', distractor_confidence=.92)
        report = self.batch([tr])['candidates'][0]
        self.assertEqual(report['appearance_status'], 'ambiguous')
        self.assertFalse(report['official_view_ok'])
        self.assertFalse(tr.can_report_image())
        self.assertEqual(len(pr.control_tracks([tr], [], 101.)), 1)

    def test_fresh_clear_image_can_resolve_prior_ambiguity(self):
        tr = self.track(appearance_status='ambiguous')
        tr.update(8., 0., .2, .8, tr.uv, tr.wh, 8., 1.75, 101.2)
        self.assertEqual(tr.appearance_status, 'actor')
        self.assertTrue(tr.can_report_image())

    def test_single_image_never_becomes_an_official_candidate(self):
        tr = pr.Track('blue', 8., 0., .99, (376., 400.), (60., 140.), 8., 1.75, 101.)
        self.assertEqual(self.batch([tr])['candidates'], [])

    def test_bad_geometry_low_confidence_clipping_and_attachment_still_block(self):
        changes = [dict(observation_loc_ok=False), dict(loc_suspect=True),
                   dict(conf=.1), dict(uv=(376., 480.)), dict(h=3.)]
        for change in changes:
            tr = self.track()
            for name, value in change.items():
                setattr(tr, name, value)
            with self.subTest(change=change):
                self.assertFalse(self.batch([tr])['candidates'][0]['official_view_ok'])
        tr = self.track()
        with patch.object(tr, 'attached_to_cam', return_value=True):
            self.assertEqual(tr.verdict(101., motion_independent=True)[1], 'self')
            self.assertFalse(self.batch([tr])['candidates'][0]['official_view_ok'])

    def test_coasting_stale_and_confirmed_distractor_do_not_report(self):
        for changes in (dict(miss=1), dict(observed_s=100.8),
                        dict(appearance_status='distractor')):
            tr = self.track()
            for name, value in changes.items():
                setattr(tr, name, value)
            self.assertEqual(self.batch([tr])['candidates'], [])

    def test_discovery_has_same_id_and_all_instances_but_no_official_evidence(self):
        a, b = self.track(), self.track(x=10., appearance_status='ambiguous')
        entries = [('blue', 0, a), ('blue', 0, b)]
        batch = pr.discovery_candidate_frame(entries, 'u0', 101., 'session-a')
        self.assertEqual(batch, pr.discovery_candidate_frame(entries[::-1], 'u0', 101., 'session-a'))
        self.assertEqual(len(batch['candidates']), 2)
        self.assertEqual({c['candidate_id'] for c in batch['candidates']},
                         {c['candidate_id'] for c in self.batch([a, b])['candidates']})
        self.assertTrue(all(not c['official_view_ok'] for c in batch['candidates']))
        self.assertEqual(batch['purpose'], 'discovery_only')

    def test_red_uses_existing_bound_slots_and_stays_out_of_discovery(self):
        a, b = self.track(cls='red', x=8.), self.track(cls='red', x=20.)
        entries = [('red', 1, a), ('red', 0, b)]
        batch = pr.report_candidate_frame(entries, 'u0', 101., 'session-a', (0., 0.))
        self.assertEqual([(c['target_id'], c['xyz'][0]) for c in batch['candidates']],
                         [('red1', 20.), ('red2', 8.)])
        self.assertEqual(pr.discovery_candidate_frame(entries, 'u0', 101., 'session-a')['candidates'], [])


if __name__ == '__main__':
    unittest.main()
