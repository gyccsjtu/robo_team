"""Pinned real BoT-SORT regressions plus pure image/world identity checks.

From the repository root, run real tracker cases with the prepared vision env:
    ROBOCUP_REAL_MOT=1 /root/robo_team_build/botsort_env_20261011/bin/python \
        -B -m unittest tools.test_image_mot -v
Without ROBOCUP_REAL_MOT=1, only the pure image/world checks run; the ten
real tracker tests are explicitly skipped. With it, missing dependencies fail.
Only the background warp is a controlled fixture. Association, Kalman state,
low-score recovery and lifecycle run the vendored BoT-SORT implementation.
Synthetic rectangles test wiring; they do not measure image/GMC accuracy.
"""
import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from perception.image_mot import ImageMOT, image_world_matches


class KnownWarp:
    """One deterministic background transform per actual image."""
    def __init__(self, method=None):
        self.calls = 0
        self.dx = 0.
        self.last_boxes = None

    def apply(self, image, boxes):
        self.calls += 1
        self.last_boxes = np.asarray(boxes).copy()
        return np.array([[1., 0., self.dx], [0., 1., 0.]], dtype=np.float32)


@unittest.skipUnless(os.environ.get('ROBOCUP_REAL_MOT') == '1',
                     'Set ROBOCUP_REAL_MOT=1 to run the pinned real tracker')
class RealImageMOTTests(unittest.TestCase):
    def setUp(self):
        self.image = np.zeros((480, 752, 3), dtype=np.uint8)

    def mot(self, **change):
        # Deliberately omit tracker_factory: this must import real vendor BOTSORT.
        args = dict(classes=['red', 'green', 'blue', 'white', 'brown'],
                    source='u0', session='fixture', gmc_factory=KnownWarp)
        args.update(change)
        mot = ImageMOT(**args)
        self.assertTrue(type(mot.trackers['red']).__module__.endswith(
            'vendor.ultralytics_trackers.bot_sort'))
        return mot

    @staticmethod
    def box(x=100., cls=0, conf=.9, width=60.):
        return NS(cls=cls, conf=conf, xyxy=[[x, 120., x+width, 280.]])

    @staticmethod
    def cue(box):
        x1, y1, x2, y2 = box.xyxy[0]
        return dict(image_track_id=box.image_track_id, conf=box.conf,
                    u=(x1+x2)/2., v=(y1+y2)/2., h=y2-y1, bearing=0.,
                    box=list(box.xyxy[0]), width=752, height=480)

    def test_two_same_color_people_keep_distinct_ids_when_box_order_reverses(self):
        mot = self.mot()
        a, b = self.box(100.), self.box(180.)
        mot.annotate(self.image, [a, b], 100.)
        ids = a.image_track_id, b.image_track_id
        self.assertIsNotNone(ids[0])
        self.assertIsNotNone(ids[1])
        self.assertNotEqual(*ids)
        for k in range(1, 5):
            a, b = self.box(100.+k*4.), self.box(180.-k*4.)
            mot.annotate(self.image, [b, a], 100.+k*.2)
            self.assertEqual((a.image_track_id, b.image_track_id), ids)

    def test_same_box_of_different_colors_cannot_share_or_steal_identity(self):
        mot = self.mot()
        red, blue = self.box(), self.box(cls=2)
        mot.annotate(self.image, [red, blue], 100.)
        red_id, blue_id = red.image_track_id, blue.image_track_id
        self.assertTrue(red_id.startswith('red:fixture:0:'))
        self.assertTrue(blue_id.startswith('blue:fixture:0:'))
        self.assertNotEqual(red_id, blue_id)
        blue = self.box(cls=2)
        mot.annotate(self.image, [blue], 100.2)
        self.assertEqual(blue.image_track_id, blue_id)
        self.assertTrue(mot.trackers['red'].lost_stracks)

    def test_low_score_detection_keeps_old_id_but_cannot_create_new_person(self):
        mot = self.mot()
        old = self.box()
        mot.annotate(self.image, [old], 100.)
        identity = old.image_track_id
        for k in range(1, 4):
            low_new, low_old = self.box(400., conf=.2), self.box(100.+k, conf=.2)
            # Test second-stage indices in the full input, not high-score subset.
            mot.annotate(self.image, [low_new, low_old], 100.+k*.2)
            self.assertEqual(low_old.image_track_id, identity)
            self.assertIsNone(low_new.image_track_id)
            self.assertEqual(low_old.conf, .2)
            self.assertEqual(mot.cues(100.+k*.2, [self.cue(low_old)]), [])

    def test_duplicate_and_out_of_order_images_do_not_advance_ids_or_cue_hits(self):
        mot = self.mot()
        for k in range(3):
            box = self.box(100.+k)
            sample = 100.+k*.2
            self.assertTrue(mot.annotate(self.image, [box], sample))
            cues = mot.cues(sample, [self.cue(box)])
        self.assertEqual(len(cues), 1)
        self.assertEqual(cues[0]['hits'], 3)
        frames, calls, matched = mot.stats['frames'], mot.gmc.calls, mot.stats['matched']
        histories = {key: list(value) for key, value in mot.cue_history.items()}
        for sample in (100.4, 100.4, 100.1, 99.):
            repeated = self.box(102.)
            self.assertFalse(mot.annotate(self.image, [repeated], sample))
            self.assertIsNone(repeated.image_track_id)
            self.assertEqual(mot.cues(sample, [self.cue(repeated)]), [])
        self.assertEqual((mot.stats['frames'], mot.gmc.calls, mot.stats['matched']),
                         (frames, calls, matched))
        self.assertEqual(mot.cue_history, histories)

    def test_empty_real_frame_marks_lost_without_publishing_predicted_detection(self):
        mot = self.mot()
        box = self.box()
        mot.annotate(self.image, [box], 100.)
        identity, matched = box.image_track_id, mot.stats['matched']
        self.assertTrue(mot.annotate(self.image, [], 100.4))
        self.assertEqual(mot.trackers['red'].tracked_stracks, [])
        self.assertTrue(mot.trackers['red'].lost_stracks)
        self.assertEqual(mot.stats['matched'], matched)
        self.assertEqual(mot.observations[identity], 100.)
        self.assertEqual(mot.cues(100.4, [self.cue(box)]), [])
        reacquired = self.box(102.)
        mot.annotate(self.image, [reacquired], 100.6)
        self.assertEqual(reacquired.image_track_id, identity)

    def test_expiration_uses_image_seconds_before_frame_buffer_is_exhausted(self):
        mot = self.mot(max_absence_s=1., max_frame_gap_s=3.)
        box = self.box()
        mot.annotate(self.image, [box], 100.)
        old = box.image_track_id
        for sample in (100.2, 100.4, 100.6, 100.8, 101.01):
            mot.annotate(self.image, [], sample)
        self.assertEqual(mot.trackers['red'].lost_stracks, [])
        self.assertNotIn(old, mot.observations)
        fresh = self.box()
        mot.annotate(self.image, [fresh], 101.2)
        self.assertIsNotNone(fresh.image_track_id)
        self.assertNotEqual(fresh.image_track_id, old)
        self.assertEqual(mot.epoch, 0)

    def test_large_image_gap_starts_new_epoch_even_if_local_numeric_id_repeats(self):
        mot = self.mot(max_frame_gap_s=1.)
        first = self.box()
        mot.annotate(self.image, [first], 100.)
        previous_gmc = mot.gmc
        mot.annotate(self.image, [], 100.4)
        fresh = self.box()
        mot.annotate(self.image, [fresh], 102.)
        self.assertNotEqual(fresh.image_track_id, first.image_track_id)
        self.assertTrue(fresh.image_track_id.startswith('red:fixture:1:'))
        self.assertEqual(mot.epoch, 1)
        self.assertEqual(mot.stats['resets'], 1)
        self.assertIsNot(mot.gmc, previous_gmc)

    def test_high_frame_rate_does_not_expire_before_absence_seconds_budget(self):
        mot = self.mot(max_absence_s=3.)
        first = self.box()
        mot.annotate(self.image, [first], 100.)
        for k in range(1, 121):
            mot.annotate(self.image, [], 100.+k/60.)
        # More than the vendor's old 30-frame buffer, but only 2 image seconds.
        self.assertTrue(mot.trackers['red'].lost_stracks)
        fresh = self.box(102.)
        mot.annotate(self.image, [fresh], 102.1)
        self.assertEqual(fresh.image_track_id, first.image_track_id)
        self.assertEqual(mot.stats['resets'], 0)

    def test_shared_background_warp_runs_once_and_preserves_all_color_ids(self):
        mot = self.mot()
        boxes = [self.box(100., cls=i) for i in range(5)]
        mot.annotate(self.image, boxes, 100.)
        identities = [box.image_track_id for box in boxes]
        self.assertEqual(mot.gmc.calls, 1)
        self.assertEqual(mot.gmc.last_boxes.shape, (5, 4))
        mot.gmc.dx = 60.
        shifted = [self.box(160., cls=i) for i in range(5)]
        mot.annotate(self.image, shifted, 100.2)
        self.assertEqual(mot.gmc.calls, 2)
        self.assertEqual([box.image_track_id for box in shifted], identities)

    def test_invalid_background_warp_counts_degradation_without_new_evidence(self):
        mot = self.mot()
        first = self.box()
        mot.annotate(self.image, [first], 100.)
        mot.gmc.dx = float('nan')
        fresh = self.box()
        mot.annotate(self.image, [fresh], 100.2)
        self.assertEqual(mot.stats['gmc_failures'], 1)
        self.assertEqual(fresh.image_track_id, first.image_track_id)
        # Fallback is only identity propagation, not person or geometry proof.
        self.assertEqual(fresh.conf, .9)
        self.assertEqual(mot.observations[fresh.image_track_id], 100.2)


class ImageWorldIdentityTests(unittest.TestCase):
    @staticmethod
    def track(identity='red:s:0:1', tid=1, xy=(0., 0.), **change):
        values = dict(id=tid, image_track_id=identity, cls='red', hits=5, h=1.7,
                      observed_s=100., x=2.2, y=0., t=101.,
                      observation_xy=lambda: xy)
        values.update(change)
        return NS(**values)

    @staticmethod
    def detection(identity='red:s:0:1', xy=(0., 0.), **change):
        values = dict(image_track_id=identity, cls='red', h=1.7, strict=True,
                      xyz=list(xy), official_view_ok=False)
        values.update(change)
        return values

    def match(self, tracks, detections, sample=100.2):
        return image_world_matches(tracks, detections, sample,
                                   confirm_hits=3, height_gate=1.5)

    def test_world_association_is_order_independent_and_never_greedily_steals_image_id(self):
        a, b = self.track(tid=1), self.track('red:s:0:2', tid=2, xy=(1., 0.))
        da = self.detection(xy=(-1., 0.))
        db = self.detection('red:s:0:2', xy=(.9, 0.))
        for tracks in ([a, b], [b, a]):
            for dets in ([da, db], [db, da]):
                result = self.match(tracks, dets)
                self.assertEqual(set(result), {1, 2})
                self.assertIs(dets[result[1][0]], da)
                self.assertIs(dets[result[2][0]], db)
        self.assertFalse(da['official_view_ok'])
        self.assertFalse(db['official_view_ok'])

    def test_turn_after_occlusion_uses_last_real_observation_not_coasted_filter(self):
        old = self.track(x=2.2, t=101.)
        result = self.match([old], [self.detection(xy=(-2.2, 0.))], 101.1)
        self.assertIn(old.id, result)
        self.assertAlmostEqual(result[old.id][1], 2.2)
        self.assertGreater(result[old.id][2], 2.2)

    def test_short_interval_projection_jump_does_not_gain_authority_from_image_id(self):
        old = self.track()
        self.assertIn(old.id, self.match([old], [self.detection(xy=(2., 0.))]))
        self.assertEqual(self.match([old], [self.detection(xy=(2.5, 0.))]), {})

    def test_wrong_id_color_height_stale_image_or_projection_jump_cannot_bind(self):
        old = self.track()
        for detection in (self.detection('red:s:0:9'), self.detection(cls='blue'),
                          self.detection(h=4.), self.detection(xy=(100., 0.)),
                          self.detection(xy=(float('nan'), 0.))):
            self.assertEqual(self.match([old], [detection]), {})
        for sample in (99.9, 100., 103.01):
            self.assertEqual(self.match([old], [self.detection()], sample), {})

    def test_duplicate_image_identity_has_no_world_owner_or_detection(self):
        old = self.track()
        other = self.track(tid=2)
        self.assertEqual(self.match([old, other], [self.detection()]), {})
        self.assertEqual(self.match([old], [self.detection(), self.detection()]), {})

    def test_unconfirmed_world_track_still_requires_strict_geometry(self):
        old = self.track(hits=1)
        self.assertEqual(self.match([old], [self.detection(strict=False)]), {})
        self.assertIn(old.id, self.match([old], [self.detection(strict=True)]))

    def test_missing_image_identity_cannot_fall_back_to_position_only_matching(self):
        self.assertEqual(self.match([self.track(identity=None)], [self.detection()]), {})
        self.assertEqual(self.match([self.track()], [self.detection(identity=None)]), {})


if __name__ == '__main__':
    unittest.main()
