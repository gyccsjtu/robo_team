"""Keep true actors and original localization when applying a second classifier."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from perception.appearance_verifier import AppearanceVerifier, same_instance

NAMES = ['red', 'green', 'blue', 'white', 'brown', 'distractor_person']


def box(cls=2, xy=(100., 100., 120., 180.), conf=.9):
    return NS(cls=cls, xyxy=[list(xy)], conf=conf, appearance_status='actor',
              appearance_confidence=conf, distractor_confidence=0.)


class AppearanceVerifierTests(unittest.TestCase):
    def test_original_actor_box_color_and_confidence_are_untouched(self):
        primary=box(); before=(primary.cls, list(primary.xyxy[0]), primary.conf)
        kept, evidence=AppearanceVerifier().apply([primary], [box()], NAMES, 10.)
        self.assertIs(kept[0], primary)
        self.assertEqual((primary.cls, primary.xyxy[0], primary.conf), before)
        self.assertEqual(evidence, [])

    def test_three_unique_strong_images_classify_without_replacing_coordinates(self):
        verifier=AppearanceVerifier()
        for sample, status in [(10., 'ambiguous'), (10.2, 'ambiguous'), (10.4, 'distractor')]:
            primary=box(conf=.99)  # Different-model scores must not be compared.
            kept, evidence=verifier.apply([primary], [box(cls=5,conf=.8)], NAMES, sample)
            self.assertEqual(primary.appearance_status, status)
            self.assertEqual(primary.conf, .99)
            self.assertEqual(primary.xyxy[0], [100.,100.,120.,180.])
            self.assertEqual(len(kept), int(status!='distractor'))

    def test_moderate_negative_is_attention_only_never_definite_sculpture(self):
        verifier=AppearanceVerifier()
        for sample in [10.,10.2,10.4,10.6]:
            primary=box()
            kept,_=verifier.apply([primary],[box(cls=5,conf=.6)],NAMES,sample)
            self.assertEqual(len(kept),1)
            self.assertEqual(primary.appearance_status,'ambiguous')

    def test_verifier_actor_evidence_wins_over_weaker_negative(self):
        primary=box()
        kept,evidence=AppearanceVerifier().apply([primary],
            [box(conf=.85),box(cls=5,conf=.6)],NAMES,10.)
        self.assertEqual(primary.appearance_status,'actor')
        self.assertEqual(evidence,[])

    def test_neighbor_and_two_owners_cannot_inherit_a_negative(self):
        verifier=AppearanceVerifier()
        for sample in [10.,10.2,10.4]:
            primary=[box(),box(xy=(122.,100.,142.,180.))]
            kept,_=verifier.apply(primary,[box(cls=5,conf=.8)],NAMES,sample)
            self.assertIn(primary[1],kept)
            self.assertEqual(primary[1].appearance_status,'actor')
        overlap=[box(),box(cls=3,xy=(100.5,100.,120.5,180.))]
        kept,_=verifier.apply(overlap,[box(cls=5,conf=.95)],NAMES,10.6)
        self.assertEqual(kept,overlap)
        self.assertTrue(all(b.appearance_status=='actor' for b in kept))

    def test_duplicate_and_old_frames_do_not_confirm(self):
        verifier=AppearanceVerifier()
        for sample in [10.,10.,9.9,10.2]:
            primary=box()
            kept,_=verifier.apply([primary],[box(cls=5,conf=.9)],NAMES,sample)
            self.assertEqual(len(kept),1)
            self.assertEqual(primary.appearance_status,'ambiguous')

    def test_gap_or_instance_jump_breaks_confirmation(self):
        for third_sample,third_box in [(11.,(100.,100.,120.,180.)),
                                       (10.4,(140.,100.,160.,180.))]:
            verifier=AppearanceVerifier()
            for t in [10.,10.2]:verifier.apply([box()],[box(cls=5,conf=.9)],NAMES,t)
            primary=box(xy=third_box)
            kept,_=verifier.apply([primary],[box(cls=5,xy=third_box,conf=.9)],NAMES,third_sample)
            self.assertEqual(len(kept),1)
            self.assertEqual(primary.appearance_status,'ambiguous')

    def test_invalid_geometry_never_matches(self):
        for b in [(0.,0.,0.,10.),(0.,0.,float('nan'),10.)]:
            self.assertFalse(same_instance((0.,0.,10.,10.),b))


if __name__=='__main__':unittest.main()
