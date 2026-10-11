"""Offline verifier must not suppress neighboring or ambiguously matched actors."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('detector_pair', Path(__file__).parent/'training/evaluate_detector_pair.py')
pair = importlib.util.module_from_spec(spec); spec.loader.exec_module(pair)


class DetectorPairTest(unittest.TestCase):
    def setUp(self):
        self.policy = dict(iou=.65, minimum_decoy_confidence=.90,
                           same_model_class_margin=.20, width_height_ratio=[.70, 1.43],
                           center_dx_fraction=.20, center_dy_fraction=.15)
        self.actor = (2, [100., 100., 120., 160.], .99)
        self.decoy = (5, [98., 98., 122., 162.], .95)

    def veto(self, actors, secondary):
        return pair.proposed_vetoes(actors, secondary, self.policy, lambda a, b: False)[0]

    def test_cross_model_confidence_is_not_subtracted(self):
        self.assertEqual(self.veto([self.actor], [self.decoy]), [0])

    def test_same_verifier_actor_conflict_abstains(self):
        competing = (2, self.actor[1], .85)
        self.assertEqual(self.veto([self.actor], [self.decoy, competing]), [])

    def test_nearby_real_actor_is_kept(self):
        nearby = (2, [126., 100., 146., 160.], .85)
        self.assertEqual(self.veto([self.actor, nearby], [self.decoy]), [0])

    def test_two_overlapping_primary_instances_abstain(self):
        other = (3, [104., 100., 124., 160.], .85)
        self.assertEqual(self.veto([self.actor, other], [self.decoy]), [])

    def test_partial_primary_overlap_abstains(self):
        other = (4, [103., 110., 119., 145.], .85)
        self.assertEqual(self.veto([self.actor, other], [self.decoy]), [])

    def test_multiple_possible_decoys_abstain(self):
        self.assertEqual(self.veto([self.actor], [self.decoy, self.decoy]), [])

    def test_weak_or_absent_secondary_does_not_remove(self):
        self.assertEqual(self.veto([self.actor], []), [])
        self.assertEqual(self.veto([self.actor], [(5, self.decoy[1], .89)]), [])

    def test_invalid_secondary_does_not_remove(self):
        self.assertEqual(self.veto([self.actor], [(5, [1., 1., 0., 0.], .99)]), [])
        self.assertEqual(self.veto([self.actor], [(5, self.decoy[1], float('nan'))]), [])


if __name__ == '__main__':
    unittest.main()
