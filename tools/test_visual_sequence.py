"""Regressions from the 17:32 hydrant/scaffold and moving-red image sequences."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from perception import perception_real as pr


class VisualSequenceTests(unittest.TestCase):
    def detection(self, **changes):
        d = dict(loc_ok=False, cls='red', uv=(376., 400.), wh=(32., 88.),
                 range=8., h=1.7, sin_down=.36, conf=.65,
                 range_method='foot_ground', appearance_status='actor')
        d.update(changes)
        return d

    def established(self):
        tr = pr.Track('red', 8., 0., .9, (376., 400.), (50., 90.),
                      8., 1.7, 99.8)
        for i in range(1, 5):
            tr.update(8.+.2*i, 0., .2, .9, tr.uv, tr.wh, 8., 1.7, 99.8+.2*i)
        return tr

    def test_known_walking_person_survives_width_and_confidence_changes(self):
        tr = self.established()
        self.assertTrue(pr.continuation_view_ok(tr, self.detection(), 100.8, .3))
        self.assertTrue(pr.continuation_view_ok(tr, self.detection(conf=.4), 100.8, .3))

    def test_continuation_does_not_admit_crops_shallow_rays_or_new_identity(self):
        for change in (dict(uv=(376., 479.)), dict(uv=(2., 400.)),
                       dict(range_method='head_height'), dict(sin_down=.29),
                       dict(h=1.0), dict(conf=.39), dict(cls='blue'),
                       dict(appearance_status='ambiguous'), dict(wh=(32., 50.))):
            with self.subTest(change=change):
                self.assertFalse(pr.continuation_view_ok(
                    self.established(), self.detection(**change), 100.8, .3))
        tr = self.established()
        tr.hits = 1
        self.assertFalse(pr.continuation_view_ok(tr, self.detection(), 100.8, .3))
        for at, residual in ((100.6, .3), (101.4, .3), (100.8, 1.3), (100.8, math.nan)):
            self.assertFalse(pr.continuation_view_ok(self.established(), self.detection(), at, residual))

    def test_continuation_cannot_renew_its_own_identity_evidence(self):
        tr = self.established()
        strict_stamp = tr.last_strict_s
        for i in range(1, 11):
            at = 100.6+i*.2
            valid = pr.continuation_view_ok(tr, self.detection(), at, .2)
            self.assertTrue(valid)
            tr.update(8.6+i*.2, 0., .2, .65, (376., 400.), (32., 88.),
                      8., 1.7, at, loc_ok=False, view_ok=valid)
            self.assertEqual(tr.last_strict_s, strict_stamp)
            self.assertTrue(tr.observation_view_ok)
            self.assertFalse(tr.can_report_image())
        self.assertFalse(pr.continuation_view_ok(tr, self.detection(), 102.8, .2))

    def test_isolated_scaffold_hits_do_not_establish_world_dispatch(self):
        tr = pr.Track('blue', 5.47, 35., .69, (590., 260.), (17., 55.),
                      12., 1.58, 2111.432)
        tr.update(3.31, 34.81, 1.592, .509, tr.uv, tr.wh, 12., 1.56, 2113.024)
        tr.update(3.29, 34.86, .204, .752, tr.uv, tr.wh, 12., 1.57, 2113.228)
        self.assertEqual(tr.hits, 3)
        self.assertFalse(tr.world_candidate_ready())
        batch = pr.report_candidate_frame([('blue', 0, tr)], 'u4', 2113.228,
                                          'session', (0., 0.))
        self.assertEqual(batch['candidates'], [])

    def test_continuous_moving_and_stationary_people_ready_at_all_camera_rates(self):
        for hz in (5, 10, 20):
            for speed in (0., 1., 2.):
                tr = pr.Track('white', 8., 0., .7, (376., 400.), (45., 100.),
                              8., 1.7, 100.)
                for i in range(1, int(hz*.6)+1):
                    at = 100.+i/hz
                    tr.update(8.+speed*i/hz, 0., 1/hz, .7, tr.uv, tr.wh, 8., 1.7, at)
                self.assertTrue(tr.world_candidate_ready(), (hz, speed))

    def test_pixel_teleport_cannot_mature_as_continuous_world_track(self):
        tr = self.established()
        tr.world_ready = False
        tr.recent_observed_xy = [(100., 8., 0.), (100.2, 11., 0.), (100.4, 11.1, 0.)]
        tr.observed_s = 100.4
        self.assertFalse(tr.world_candidate_ready())

    def test_three_full_height_contradictions_suppress_hydrant_attention(self):
        tracker = pr.PixelCueTracker()
        for i, height in enumerate((1.028, 1.040, 1.051)):
            cues = tracker.update(100.+.2*i, [dict(cls='red', conf=.69,
                u=707., v=436., h=68., bearing=.2, body_height_m=height)])
        self.assertEqual(cues, [])
        self.assertEqual(len(tracker.rejections), 1)

    def test_crop_unknown_height_or_one_bad_frame_cannot_veto_real_attention(self):
        for heights in ((None, None, None), (1.7, 1.7, 1.7), (1.0, 1.7, 1.0)):
            tracker = pr.PixelCueTracker()
            for i, height in enumerate(heights):
                cues = tracker.update(100.+.2*i, [dict(cls='red', conf=.69,
                    u=376., v=300., h=80., bearing=0., body_height_m=height)])
            self.assertEqual(len(cues), 1, heights)
        self.assertIsNone(pr.cue_body_height((300., 300., 350., 480.), 4., np.eye(3)))
        self.assertIsNone(pr.cue_body_height((300., 150., 350., 240.), 4., np.eye(3)))
        # The image box ends at the physical foot, not at an invented shadow.
        y1 = 240.+376.*(4.-1.7)/8.
        foot = 240.+376.*4./8.
        y2 = foot
        self.assertAlmostEqual(pr.cue_body_height((350., y1, 390., y2), 4., np.eye(3)), 1.7)


if __name__ == '__main__':
    unittest.main()
