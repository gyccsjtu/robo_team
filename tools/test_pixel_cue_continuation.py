"""Small real detector boxes may continue a bounded, established look only."""
import unittest

from perception import perception_real as pr


class PixelCueContinuationTests(unittest.TestCase):
    def test_recovery_metadata_uses_same_image_and_mutually_unique_association(self):
        tracker=self.established(self.box(fy=376.,foot_down_sin=.27))
        cue=tracker.update(100.6,[self.box(fy=376.,foot_down_sin=.28)])[0]
        self.assertTrue(cue['association_unique'])
        self.assertEqual(cue['foot_down_sin'],.28)
        self.assertEqual(cue['fy'],376.)
        tracker=self.established(self.box(color='red',u=350.,bearing=-.08),
                                 self.box(color='red',u=402.,bearing=.08))
        cues=tracker.update(100.6,[self.box(color='red',u=376.,bearing=0.)])
        self.assertTrue(cues)
        self.assertFalse(cues[0]['association_unique'])

    def box(self, height=44., color='brown', u=376., bearing=0., confidence=.9,
            continuation=None, **kwargs):
        width = 16.
        item = dict(cls=color, conf=confidence, u=u, v=320., h=height,
                    bearing=bearing, cam_x=0., cam_y=0.,
                    box=[u-width/2, 320.-height/2, u+width/2, 320.+height/2],
                    width=752, height=480, appearance_status='actor',
                    small_box_continuation_only=(height < 40. if continuation is None else continuation))
        item.update(kwargs)
        return item

    def established(self, *boxes):
        tracker = pr.PixelCueTracker()
        boxes = boxes or (self.box(),)
        for stamp in (100., 100.2, 100.4):
            cues = tracker.update(stamp, list(boxes))
        self.assertEqual(len(boxes), len(cues))
        return tracker

    def test_input_opt_in_keeps_ordinary_gate_and_geometry_limits(self):
        for height in (32., 35., 39.999):
            box = self.box(height)
            self.assertFalse(pr.pixel_cue_box_ok('brown', .9, *box['box']))
            self.assertTrue(pr.pixel_cue_box_ok('brown', .9, *box['box'], allow_small_continuation=True))
        for height in (20., 31.999):
            self.assertFalse(pr.pixel_cue_box_ok('brown', .9, *self.box(height)['box'], allow_small_continuation=True))
        self.assertTrue(pr.pixel_cue_box_ok('brown', .9, *self.box(40.)['box']))
        for args in (('brown', .4, 360., 300., 376., 335.),
                     ('brown', .9, 360., 300., 365., 335.),
                     ('brown', .9, 360., 1., 376., 36.)):
            self.assertFalse(pr.pixel_cue_box_ok(*args, allow_small_continuation=True))

    def test_small_hydrant_or_new_colour_cannot_establish_a_candidate(self):
        tracker = pr.PixelCueTracker()
        for stamp in (100., 100.2, 100.4, 100.6, 100.8):
            self.assertEqual([], tracker.update(stamp, [self.box(35., color='red')]))
        self.assertEqual(0, tracker.next_id)
        self.assertEqual([], tracker.tracks)
        tracker = self.established()
        self.assertEqual([], tracker.update(100.6, [self.box(35., color='green')]))
        self.assertEqual(1, tracker.next_id)

    def test_only_an_established_normal_size_instance_can_continue(self):
        for normals in ((100.,), (100., 100.2), (100., 100.05, 100.1)):
            tracker = pr.PixelCueTracker()
            for stamp in normals:
                self.assertEqual([], tracker.update(stamp, [self.box()]))
            self.assertEqual([], tracker.update(normals[-1]+.2, [self.box(35.)]))
        tracker = self.established()
        cues = tracker.update(100.6, [self.box(35.)])
        self.assertEqual(1, len(cues))
        self.assertEqual('brown:0', cues[0]['candidate_id'])
        self.assertTrue(cues[0]['small_box_continuation'])
        self.assertTrue(cues[0]['attention_only'])
        self.assertFalse(cues[0]['report_eligible'])
        self.assertFalse(cues[0]['footpoint_valid'])
        self.assertEqual(100.4, cues[0]['normal_anchor_s'])

    def test_duplicate_reverse_nonfinite_and_image_gap_do_not_refresh_anchor(self):
        tracker = self.established()
        for stamp in (100.4, 100.3, float('nan')):
            self.assertEqual([], tracker.update(stamp, [self.box(35.)]))
        self.assertEqual(100.4, tracker.tracks[0]['sample_s'])
        self.assertEqual([], tracker.update(100.751, [self.box(35.)]))
        self.assertEqual(100.4, tracker.tracks[0]['sample_s'])

    def test_small_images_never_renew_two_second_normal_anchor(self):
        tracker = self.established()
        for index in range(1, 11):
            cues = tracker.update(100.4+index*.2, [self.box(35.)])
            self.assertEqual(1, len(cues), index)
            self.assertEqual(100.4, cues[0]['normal_anchor_s'])
        self.assertEqual([], tracker.update(102.401, [self.box(35.)]))
        self.assertEqual([], tracker.update(102.6, [self.box(35.)]))

    def test_normal_size_recovery_reestablishes_anchor_without_new_id(self):
        tracker = self.established()
        for stamp in (100.6, 100.8, 101., 101.2, 101.4, 101.6):
            self.assertTrue(tracker.update(stamp, [self.box(35.)]))
        for stamp in (101.8, 102., 102.2):
            cues = tracker.update(stamp, [self.box(40.)])
            self.assertEqual('brown:0', cues[0]['candidate_id'])
            self.assertNotIn('small_box_continuation', cues[0])
        cues = tracker.update(102.4, [self.box(35.)])
        self.assertEqual(102.2, cues[0]['normal_anchor_s'])

    def test_two_established_instances_make_middle_small_box_ambiguous(self):
        tracker = self.established(self.box(u=350., bearing=-.08),
                                   self.box(u=402., bearing=.08))
        self.assertEqual([], tracker.update(100.6, [self.box(35., u=376., bearing=0.)]))
        self.assertEqual([100.4, 100.4], [tr['sample_s'] for tr in tracker.tracks])

    def test_two_small_boxes_cannot_choose_one_old_instance_by_confidence(self):
        for reverse in (False, True):
            tracker = self.established()
            boxes = [self.box(35., u=372., bearing=-.015, confidence=.92),
                     self.box(35., u=380., bearing=.015, confidence=.86)]
            if reverse:
                boxes.reverse()
            self.assertEqual([], tracker.update(100.6, boxes))
            self.assertEqual(100.4, tracker.tracks[0]['sample_s'])

    def test_small_box_does_not_compete_with_normal_detection_in_same_image(self):
        tracker = self.established()
        cues = tracker.update(100.6, [self.box(35., confidence=.95), self.box(44., confidence=.9)])
        self.assertEqual(1, len(cues))
        self.assertNotIn('small_box_continuation', cues[0])

    def test_low_confidence_appearance_conflict_bad_bounds_and_missing_pose_fail(self):
        alternatives = [self.box(35., confidence=.44), self.box(35., appearance_status='ambiguous'),
                        self.box(35., appearance_status='distractor'), self.box(35., bearing=float('nan')),
                        self.box(35., continuation=False), self.box(35., box=[1.,2.,3.,4.]),
                        self.box(31.9), self.box(40., continuation=True)]
        for box in alternatives:
            tracker = self.established()
            self.assertEqual([], tracker.update(100.6, [box]), box)
            self.assertEqual(100.4, tracker.tracks[0]['sample_s'])

    def test_three_contradictory_full_body_frames_cannot_anchor_a_small_hydrant(self):
        tracker = pr.PixelCueTracker()
        for stamp in (100., 100.2, 100.4):
            self.assertEqual([], tracker.update(stamp, [self.box(body_height_m=.9)]))
        self.assertEqual([], tracker.update(100.6, [self.box(35.)]))

    def test_small_cue_never_promotes_geometry_even_if_attention_flag_is_lost(self):
        tracker = self.established()
        cue = tracker.update(100.6, [self.box(35.)])[0]
        cue['attention_only'] = False
        self.assertFalse(pr.pixel_promoted_strict([cue], 'brown', 376., 320., .95,
                         8., 1.8, 2., 80., 350., 390., 280., 752))
        seeds = pr.fast_tracker_seed_frame([cue], 'typhoon_h480_3', 100.6, 'session')
        self.assertFalse(seeds['report_eligible'])
        self.assertFalse(seeds['control_eligible'])
        self.assertEqual('session:brown:0', seeds['detections'][0]['id'])


if __name__ == '__main__':
    unittest.main()
