import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'perception'))
from distractor_verifier import DistractorVerifier, SecondaryAppearance, strong_decoys


def box(label='blue', confidence=.8, rect=(100,100,130,180)):
    return dict(label=label,confidence=confidence,box=list(rect))


class VerifierTests(unittest.TestCase):
    def setUp(self):
        self.v=DistractorVerifier()
        self.p=[box()];self.d=[box('distractor_person',.9)]

    def test_three_distinct_frames_with_span(self):
        self.assertFalse(self.v.update(self.p,self.d,1.))
        self.assertFalse(self.v.update(self.p,self.d,1.15))
        self.assertEqual(self.v.update(self.p,self.d,1.30),{0})

    def test_duplicate_and_reverse_samples_do_not_vote(self):
        for t in (1.,1.,.9,1.,1.1):self.assertFalse(self.v.update(self.p,self.d,t))

    def test_missing_or_weak_secondary_is_not_evidence(self):
        for s in ([],[box('distractor_person',.84)],[box('blue',.9)]):
            self.assertFalse(strong_decoys(self.p,s))

    def test_colour_flip_cannot_reset_physical_decoy_evidence(self):
        for t,c in ((1.,'blue'),(1.15,'white')):
            self.assertFalse(self.v.update([box(c)],self.d,t))
        self.assertEqual(self.v.update(self.p,self.d,1.3),{0})

    def test_nearby_real_person_is_not_suppressed(self):
        primary=self.p+[box('white',.8,(125,100,155,180))]
        self.assertEqual(set(strong_decoys(primary,self.d)),{0})

    def test_secondary_actor_conflict_abstains(self):
        self.assertFalse(strong_decoys(self.p,self.d+[box('green',.31)]))

    def test_primary_scores_not_compared_across_models(self):
        self.assertEqual(set(strong_decoys([box(confidence=.99)],self.d)),{0})

    def test_two_overlapping_people_make_identity_ambiguous(self):
        p=self.p+[box('white',.8,(105,100,135,180))]
        self.assertFalse(strong_decoys(p,self.d))

    def test_duplicate_colour_boxes_do_not_count_as_multiple_frames(self):
        p=self.p+[box('white')]
        self.assertFalse(self.v.update(p,self.d,1.))
        self.assertFalse(self.v.update(p,self.d,1.01))
        self.assertEqual(self.v.update(p,self.d,1.3),{0,1})

    def test_temporal_gap_or_disagreement_resets(self):
        self.v.update(self.p,self.d,1.);self.v.update(self.p,self.d,1.2)
        self.assertFalse(self.v.update(self.p,self.d,2.))
        self.v.update(self.p,[],2.1)
        self.assertFalse(self.v.update(self.p,self.d,2.3))

    def test_cropped_subpart_cannot_be_vetoed_by_neighboring_large_box(self):
        self.assertFalse(strong_decoys([box(rect=(104,110,120,140))],self.d))

    def test_nan_or_malformed_box_abstains(self):
        for rect in ((100,100,float('nan'),180),(130,100,100,180),(100,100,130)):
            self.assertFalse(strong_decoys([box(rect=rect)],self.d))

    def test_runtime_enforce_and_shadow_preserve_primary_geometry(self):
        for mode in ('shadow','enforce'):
            guard=SecondaryAppearance.__new__(SecondaryAppearance)
            guard.mode=mode;guard.histories={};guard.model=Mock()
            decoy=NS(cls=5,conf=.9,xyxy=[[100,100,130,180]])
            guard.model.predict.return_value=[NS(names={5:'distractor_person'},boxes=[decoy])]
            actor=NS(cls=0,conf=.95,xyxy=[[100,100,130,180]])
            for t in (1.,1.15,1.3):
                kept,evidence=guard.filter(None,[actor],['blue'],t,'u0')
            self.assertEqual(len(kept),int(mode=='shadow'))
            self.assertEqual(actor.xyxy,[[100,100,130,180]])
            self.assertEqual(evidence[0]['status'],'secondary_distractor')
            # Evidence from UAV0 may not suppress the first picture from UAV1.
            kept,_=guard.filter(None,[actor],['blue'],1.3,'u1')
            self.assertEqual(kept,[actor])

    def test_runtime_no_primary_box_clears_evidence_without_inference(self):
        guard=SecondaryAppearance.__new__(SecondaryAppearance)
        guard.mode='enforce';guard.model=Mock();guard.histories={'stereo':self.v}
        self.v.update(self.p,self.d,1.)
        self.assertEqual(guard.filter(None,[],['blue'],1.1),([],[]))
        self.assertEqual(self.v.history,[])
        guard.model.predict.assert_not_called()


if __name__=='__main__':unittest.main()
