"""Positive secondary evidence is current, same-colour and uniquely owned."""
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock

from perception.distractor_verifier import SecondaryAppearance


NAMES = ['red', 'green', 'blue', 'white', 'brown', 'distractor_person']


def box(cls=2, conf=.8, xy=(100., 100., 130., 180.)):
    return NS(cls=cls, conf=conf, xyxy=[list(xy)], appearance_status='actor',
              distractor_confidence=0.)


class SecondaryActorEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.guard = SecondaryAppearance.__new__(SecondaryAppearance)
        self.guard.mode = 'enforce'
        self.guard.histories = {}
        self.guard.model = Mock()

    def run_filter(self, primary, secondary, sample=1., camera='u5'):
        self.guard.model.predict.return_value = [NS(names=NAMES, boxes=secondary)]
        return self.guard.filter(None, primary, NAMES, sample, camera)

    def test_unique_actor_has_exact_same_image_source_and_secondary_box(self):
        for cls in range(5):
            with self.subTest(cls=cls):
                self.guard.histories.clear()
                original = box(cls, .95)
                positive = box(cls, .30, (101., 101., 131., 181.))
                kept, _ = self.run_filter([original], [positive])
                self.assertIs(kept[0], original)
                self.assertTrue(original.secondary_actor_confirmed)
                self.assertEqual(original.secondary_actor_label, NAMES[cls])
                self.assertEqual(original.secondary_actor_sample_s, 1.)
                self.assertEqual(original.secondary_actor_camera, 'u5')
                self.assertEqual(original.secondary_actor_confidence, .30)
                self.assertEqual(original.secondary_actor_box, positive.xyxy[0])
        self.assertEqual(self.guard.model.predict.call_count, 5)

    def test_different_colour_decoy_and_multiple_matches_are_not_proof(self):
        cases = ([box(3)], [box(2), box(5)], [box(2), box(3)],
                 [box(2), box(2)], [box(2, .299)], [box(2, math.nan)])
        for secondary in cases:
            with self.subTest(secondary=secondary):
                self.guard.histories.clear()
                original = box()
                self.run_filter([original], secondary)
                self.assertFalse(original.secondary_actor_confirmed)

    def test_one_secondary_cannot_certify_two_primary_instances(self):
        for primary in ([box(), box()], [box(), box(3)],
                        [box(), box(xy=(103., 100., 133., 180.))]):
            with self.subTest(primary=primary):
                self.guard.histories.clear()
                self.run_filter(primary, [box()])
                self.assertTrue(all(not b.secondary_actor_confirmed for b in primary))

    def test_nearby_independent_white_green_keep_their_own_proof(self):
        primary = [box(3), box(1, xy=(180., 100., 210., 180.))]
        self.run_filter(primary, [box(3), box(1, xy=(180., 100., 210., 180.))])
        self.assertTrue(all(b.secondary_actor_confirmed for b in primary))

    def test_reused_boxes_clear_proof_on_missing_error_and_no_model(self):
        original = box()
        self.run_filter([original], [box()])
        self.assertTrue(original.secondary_actor_confirmed)
        self.run_filter([original], [], 1.2)
        self.assertFalse(original.secondary_actor_confirmed)
        self.assertEqual(original.secondary_actor_label, '')
        self.assertIsNone(original.secondary_actor_sample_s)
        self.assertEqual(original.secondary_actor_camera, '')
        self.run_filter([original], [box()], 1.4)
        self.guard.model.predict.side_effect = RuntimeError('unavailable')
        self.guard.filter(None, [original], NAMES, 1.6, 'u5')
        self.assertFalse(original.secondary_actor_confirmed)
        self.guard.model = None
        original.secondary_actor_confirmed = True
        self.guard.filter(None, [original], NAMES, 1.8, 'u5')
        self.assertFalse(original.secondary_actor_confirmed)

    def test_empty_result_repeated_or_old_image_has_no_current_proof(self):
        original = box()
        self.run_filter([original], [box()])
        for sample in (1., .9, math.nan, math.inf):
            self.run_filter([original], [box()], sample)
            self.assertFalse(original.secondary_actor_confirmed)
        self.guard.model.predict.return_value = []
        self.guard.filter(None, [original], NAMES, 1.2, 'u5')
        self.assertFalse(original.secondary_actor_confirmed)

    def test_primary_appearance_conflict_does_not_get_actor_proof(self):
        for status in ('ambiguous', 'distractor'):
            self.guard.histories.clear()
            primary = box()
            primary.appearance_status = status
            self.run_filter([primary], [box()])
            self.assertFalse(primary.secondary_actor_confirmed)


if __name__ == '__main__':
    unittest.main()
