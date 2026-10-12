"""A confirmed decoy hold is finite, per camera and uniquely associated.

The 060353 case uses logged matched-primary boxes and exact image times.
Secondary prediction boxes were not logged: mocked equal boxes reproduce the
recorded strong-decoy decisions, not a replay of unknown model tensors.
"""
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock

from perception.distractor_verifier import DistractorVerifier, SecondaryAppearance, same_object


NAMES = ['red', 'green', 'blue', 'white', 'brown', 'distractor_person']


def box(cls=2, conf=.8, xy=(100., 100., 130., 180.)):
    return NS(cls=cls, conf=conf, xyxy=[list(xy)], appearance_status='actor',
              distractor_confidence=0.)


class ConfirmedDistractorHoldTests(unittest.TestCase):
    def setUp(self):
        self.guard = SecondaryAppearance.__new__(SecondaryAppearance)
        self.guard.mode = 'enforce'
        self.guard.histories = {}
        self.guard.model = Mock()

    def run_filter(self, sample, primary=None, secondary=None, camera='u5'):
        if primary is None:
            primary = [box()]
        self.guard.model.predict.return_value = [NS(names=NAMES, boxes=secondary or [])]
        return self.guard.filter(None, primary, NAMES, sample, camera)

    def confirm(self, xy=(100., 100., 130., 180.), camera='u5'):
        for sample in (1., 1.15, 1.30):
            kept, _ = self.run_filter(sample, [box(xy=xy)], [box(5, .95, xy)], camera)
        self.assertEqual(kept[0].appearance_status, 'distractor')

    def test_three_frame_confirmation_survives_empty_images_then_expires(self):
        self.confirm()
        self.run_filter(1.5, [], [])
        kept, evidence = self.run_filter(2.912)
        self.assertEqual(kept[0].appearance_status, 'distractor')
        self.assertTrue(kept[0].secondary_decoy_confirmed)
        self.assertEqual(evidence[0]['status'], 'secondary_distractor_hold')
        kept, _ = self.run_filter(3.300001)
        self.assertEqual(kept[0].appearance_status, 'actor')

    def test_pending_and_held_boxes_do_not_renew_the_deadline(self):
        self.confirm()
        kept, evidence = self.run_filter(3.1, secondary=[box(5, .95)])
        self.assertEqual(kept[0].appearance_status, 'distractor')
        self.assertEqual(evidence[0]['status'], 'secondary_distractor_hold')
        self.run_filter(3.2)
        kept, _ = self.run_filter(3.31)
        self.assertEqual(kept[0].appearance_status, 'actor')

    def test_one_error_does_not_create_a_hold_for_white_or_green(self):
        for cls in (1, 3):
            with self.subTest(cls=cls):
                self.guard.histories.clear()
                self.run_filter(1., [box(cls)], [box(5, .95)])
                kept, _ = self.run_filter(1.2, [box(cls)])
                self.assertEqual(kept[0].appearance_status, 'actor')
                self.assertFalse(self.guard.histories['u5'].confirmed)

    def test_fresh_unique_white_or_green_evidence_immediately_releases_hold(self):
        for cls in (1, 3):
            with self.subTest(cls=cls):
                self.guard.histories.clear()
                self.confirm()
                kept, _ = self.run_filter(1.5, [box(cls)], [box(cls, .8)])
                self.assertEqual(kept[0].appearance_status, 'actor')
                self.assertTrue(kept[0].secondary_actor_confirmed)
                self.assertFalse(self.guard.histories['u5'].confirmed)
                kept, _ = self.run_filter(1.7, [box(cls)])
                self.assertEqual(kept[0].appearance_status, 'actor')

    def test_nearby_real_people_and_other_cameras_never_borrow_the_hold(self):
        self.confirm()
        for cls in (1, 3):
            near = (125., 100., 155., 180.)
            kept, _ = self.run_filter(1.5 + cls/100.,
                                     [box(), box(cls, xy=near)], [box(cls, xy=near)])
            self.assertEqual([b.appearance_status for b in kept], ['distractor', 'actor'])
            self.assertTrue(kept[1].secondary_actor_confirmed)
        kept, _ = self.run_filter(1.6, camera='u0')
        self.assertEqual(kept[0].appearance_status, 'actor')

    def test_one_old_box_cannot_reject_two_overlapping_people(self):
        self.confirm()
        kept, _ = self.run_filter(1.5, [box(), box(3, xy=(103., 100., 133., 180.))])
        self.assertEqual([b.appearance_status for b in kept], ['actor', 'actor'])

    def test_one_current_box_cannot_choose_between_two_confirmed_objects(self):
        left, right = (100., 100., 130., 180.), (112., 100., 142., 180.)
        for sample in (1., 1.15, 1.3):
            self.run_filter(sample, [box(xy=left), box(xy=right)],
                            [box(5, .95, left), box(5, .95, right)])
        self.assertEqual(len(self.guard.histories['u5'].confirmed), 2)
        kept, _ = self.run_filter(1.5, [box(xy=(106., 100., 136., 180.))])
        self.assertEqual(kept[0].appearance_status, 'actor')

    def test_conflicting_fresh_actor_prevents_held_rejection(self):
        self.confirm()
        kept, _ = self.run_filter(1.5, secondary=[box(3, .8)])
        self.assertEqual(kept[0].appearance_status, 'actor')
        self.assertFalse(kept[0].secondary_actor_confirmed)
        self.assertTrue(self.guard.histories['u5'].confirmed)

    def test_hold_does_not_move_along_a_sequence_of_new_boxes(self):
        self.confirm()
        kept, _ = self.run_filter(1.5, [box(xy=(106., 100., 136., 180.))])
        self.assertEqual(kept[0].appearance_status, 'distractor')
        kept, _ = self.run_filter(1.7, [box(xy=(112., 100., 142., 180.))])
        self.assertEqual(kept[0].appearance_status, 'actor')
        self.assertEqual(self.guard.histories['u5'].confirmed[0]['box'],
                         [100., 100., 130., 180.])

    def test_duplicate_old_and_nonfinite_times_do_not_hold_or_refresh(self):
        self.confirm()
        for sample in (1.3, math.nan, math.inf):
            kept, _ = self.run_filter(sample)
            self.assertEqual(kept[0].appearance_status, 'actor')
        kept, _ = self.run_filter(.9)
        self.assertEqual(kept[0].appearance_status, 'actor')
        self.assertFalse(self.guard.histories['u5'].confirmed)
        kept, _ = self.run_filter(1.5)
        self.assertEqual(kept[0].appearance_status, 'actor')

    def test_shadow_records_hold_without_rejecting_primary(self):
        self.guard.mode = 'shadow'
        for sample in (1., 1.15, 1.30):
            self.run_filter(sample, secondary=[box(5, .95)])
        kept, evidence = self.run_filter(1.5)
        self.assertEqual(kept[0].appearance_status, 'actor')
        self.assertEqual(evidence[0]['status'], 'secondary_distractor_hold')

    def test_actual_060353_full_and_half_frame_remains_scoped(self):
        # Exact CSV matched-primary boxes; no actor truth or world locations.
        negatives = [
            (2165.376, .918, [366.3962707519531, 309.154296875,
                             398.8646240234375, 407.4518127441406]),
            (2165.572, .930, [364.99957275390625, 308.68914794921875,
                             398.4740295410156, 408.0736083984375]),
            (2165.756, .936, [364.7109069824219, 308.7005920410156,
                             397.9471130371094, 395.5929870605469]),
            (2165.964, .928, [365.15771484375, 305.7037048339844,
                             397.5671691894531, 391.4706726074219]),
        ]
        for sample, confidence, xy in negatives:
            self.run_filter(sample, [box(xy=xy)], [box(5, confidence, xy)])
        anchor = self.guard.histories['u5'].confirmed[0]
        self.assertEqual(anchor['confirmed_s'], 2165.964)
        for sample in (2166.152, 2166.388, 2166.576, 2166.764):
            self.run_filter(sample, [])
        pending = [
            (2166.964, .933, [361.38751220703125, 309.8331604003906,
                             396.8983459472656, 411.3833923339844]),
            (2167.388, .934, [361.9615478515625, 313.056396484375,
                             398.0067443847656, 413.24212646484375]),
        ]
        for sample, confidence, xy in pending:
            self.run_filter(sample, [box(3, xy=xy)], [box(5, confidence, xy)])
            if sample == 2166.964:
                self.run_filter(2167.156, [])
        full = [363.3912048339844, 322.90325927734375,
                396.2178955078125, 426.2430725097656]
        half = [363.3, 323.105, 396.1, 385.205]
        self.assertTrue(same_object(half, negatives[-1][2]))
        self.assertFalse(same_object(full, negatives[-1][2]))
        self.assertFalse(same_object(half, full))
        kept, evidence = self.run_filter(2167.576,
                                        [box(xy=full), box(conf=.575, xy=half)],
                                        [box(5, .925, full)])
        self.assertEqual([b.appearance_status for b in kept], ['ambiguous', 'distractor'])
        self.assertEqual(evidence[-1]['status'], 'secondary_distractor_hold')
        self.assertEqual(self.guard.histories['u5'].confirmed[0]['confirmed_s'], 2165.964)
        kept, _ = self.run_filter(2167.965, [box(xy=half)])
        self.assertEqual(kept[0].appearance_status, 'actor')


if __name__ == '__main__':
    unittest.main()
