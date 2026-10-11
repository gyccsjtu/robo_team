"""A decoy contradiction cannot become a person from one unopposed frame."""
import unittest

from perception import perception_real as pr


class RecoveryTests(unittest.TestCase):
    def test_ordinary_stationary_actor_needs_no_new_gate(self):
        state = pr.AppearanceRecovery()
        self.assertTrue(state.observe(100., 'actor', 0.))

    def test_fresh_sustained_actor_evidence_recovers_without_motion(self):
        state = pr.AppearanceRecovery()
        self.assertFalse(state.observe(100., 'ambiguous', .95))
        for sample in (100.2, 100.2, 100.1, 100.4):
            self.assertFalse(state.observe(sample, 'actor', 0.))
        self.assertTrue(state.observe(100.6, 'actor', 0.))

    def test_intermittent_negative_or_sparse_primary_does_not_manufacture_recovery(self):
        state = pr.AppearanceRecovery()
        for sample, status in ((100., 'ambiguous'), (100.2, 'actor'),
                               (100.4, 'ambiguous'), (100.6, 'actor'),
                               (102., 'actor'), (102.2, 'actor')):
            self.assertFalse(state.observe(sample, status, .95 if status == 'ambiguous' else 0.))
        self.assertTrue(state.observe(102.4, 'actor', 0.))

    def test_short_burst_or_invalid_time_cannot_clear_contradiction(self):
        state = pr.AppearanceRecovery()
        state.observe(100., 'ambiguous', .95)
        for sample in (100.01, 100.02, 100.03, float('nan'), 100.03):
            self.assertFalse(state.observe(sample, 'actor', 0.))


class PipelineRecoveryTests(unittest.TestCase):
    def box(self, negative=False, **changes):
        b = dict(cls='blue', conf=.95, u=376., v=320., h=80., bearing=0.,
                 cam_x=0., cam_y=0., box=[356.,280.,396.,360.],
                 appearance_status='ambiguous' if negative else 'actor',
                 distractor_confidence=.95 if negative else 0.)
        b.update(changes)
        return b

    def test_pending_frames_do_not_create_pixel_attention_or_recovery_anchor(self):
        tracker = pr.PixelCueTracker()
        for sample in (100., 100.2, 100.4):
            self.assertEqual(tracker.update(sample, [self.box(True)]), [])
        self.assertEqual(tracker.update(100.6, [self.box()]), [])
        self.assertNotIn('normal_anchor_s', tracker.tracks[0])
        self.assertEqual(tracker.update(100.8, [self.box()]), [])
        cues = tracker.update(101., [self.box()])
        self.assertEqual(cues[0]['candidate_id'], 'blue:0')
        self.assertEqual(tracker.tracks[0]['normal_anchor_s'], 101.)

    def test_other_same_colour_person_is_independent_of_negative_instance(self):
        tracker = pr.PixelCueTracker()
        for sample in (100., 100.2, 100.4):
            cues = tracker.update(sample, [self.box(True), self.box(u=650., bearing=.65)])
        self.assertEqual(len(cues), 1)
        self.assertEqual(cues[0]['u'], 650.)

    def test_new_id_after_gap_does_not_make_negative_frames_valid(self):
        tracker = pr.PixelCueTracker()
        for start in (100., 103., 106.):
            for offset in (0., .2, .4):
                self.assertEqual(tracker.update(start+offset, [self.box(True)]), [])
            self.assertEqual(tracker.update(start+.6, [self.box()]), [])
        self.assertEqual(tracker.next_id, 3)

    def test_world_report_cannot_recover_on_single_primary_image(self):
        track = pr.Track('blue', 6., 0., .95, (376.,360.), (40.,80.), 6.,1.7,100.)
        self.assertTrue(track.can_report_image())
        for sample, negative, expected in ((100.2,True,False), (100.4,False,False),
                                            (100.6,False,False), (100.8,False,True)):
            self.assertTrue(track.update(6.,0.,.2,.95,(376.,360.),(40.,80.),6.,1.7,sample,
                appearance_status='ambiguous' if negative else 'actor',
                distractor_confidence=.95 if negative else 0.))
            self.assertEqual(track.can_report_image(), expected)
            self.assertEqual(pr.appearance_fields(track)['appearance_status'],
                             'actor' if expected else 'ambiguous')


if __name__ == '__main__':
    unittest.main()
