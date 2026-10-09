from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parents[2]/'perception'))
from person_verifier import PersonVerifier,overlap,frame_verified


def box(cls,xyxy):
    return SimpleNamespace(cls=cls,conf=.85,xyxy=[xyxy])


class PersonVerifierTests(unittest.TestCase):
    def verifier(self, people):
        verifier=PersonVerifier('configured-local-weights',device='cpu')
        self.calls=[]
        def predict(image,**kwargs):
            self.calls.append((image,kwargs))
            return [SimpleNamespace(boxes=[box(0,p) for p in people])]
        verifier.model=predict
        return verifier

    def test_same_frame_person_retains_original_green_coordinates_and_confidence(self):
        green=box(1,[10.,20.,30.,60.]);image=object()
        verifier=self.verifier([[11.,21.,29.,59.]])
        result=verifier.filter_boxes(image,[green])
        self.assertIs(result[0],green)
        self.assertIs(self.calls[0][0],image)
        self.assertEqual(self.calls[0][1]['classes'],[0])

    def test_garbage_bin_without_person_is_vetoed_but_other_colors_unchanged(self):
        green=box(1,[10.,20.,30.,60.]);red=box(0,[40.,20.,60.,60.])
        verifier=self.verifier([])
        self.assertEqual(verifier.filter_boxes(object(),[green,red]),[red])

    def test_distant_person_does_not_validate_unrelated_green_box(self):
        verifier=self.verifier([[40.,20.,60.,60.]])
        self.assertEqual(verifier.filter_boxes(object(),[box(1,[10.,20.,30.,60.])]),[])

    def test_disabled_or_non_green_path_does_not_load_second_model(self):
        red=box(0,[10.,20.,30.,60.])
        for verifier,boxes in ((PersonVerifier(),[box(1,[10.,20.,30.,60.])]),
                               (PersonVerifier('nonexistent'),[red])):
            self.assertEqual(verifier.filter_boxes(object(),boxes),boxes)
            self.assertIsNone(verifier.model)

    def test_overlap_rejects_degenerate_and_nonfinite_boxes(self):
        for rectangle in ([0,0,0,10],[10,10,0,0],[float('nan'),0,10,10]):
            self.assertEqual(overlap(rectangle,[0,0,10,10]),0.)
        self.assertEqual(overlap([0,0,10,10],[0,0,10,10]),1.)

    def test_white_proof_is_per_box_and_does_not_remove_legacy_white_detections(self):
        verifier=self.verifier([[11.,21.,29.,59.]])
        verifier.color_check=True
        verifier.proof_classes={1,3}
        verified=box(3,[10.,20.,30.,60.])
        unrelated=box(3,[40.,20.,60.,60.])
        jersey=SimpleNamespace(filter_boxes=lambda image,boxes:boxes)
        with patch.dict(sys.modules,jersey_color=jersey):
            self.assertEqual(verifier.filter_boxes(object(),[verified,unrelated]),[verified,unrelated])
            self.assertTrue(frame_verified(verifier.verified_boxes,3,verified.xyxy[0]))
            self.assertFalse(frame_verified(verifier.verified_boxes,3,unrelated.xyxy[0]))
            self.assertFalse(frame_verified(verifier.verified_boxes,1,verified.xyxy[0]))
            verifier.filter_boxes(object(),[unrelated])
            self.assertEqual(verifier.verified_boxes,[])

    def test_red_proof_requires_same_frame_person_and_red_shirt(self):
        red=box(0,[10.,20.,30.,60.]);verifier=self.verifier([[11.,21.,29.,59.]])
        verifier.color_check=True;verifier.proof_classes={1,3,0}
        fractions=dict(red=.7,green=0.,blue=0.,white=0.,brown=.1)
        with patch('jersey_color.torso_fractions',return_value=fractions):
            self.assertEqual(verifier.filter_boxes(object(),[red]),[red])
            self.assertTrue(frame_verified(verifier.verified_boxes,0,red.xyxy[0]))
        for invalid in (None,dict(fractions,red=.02,blue=.6)):
            with patch('jersey_color.torso_fractions',return_value=invalid):
                self.assertEqual(verifier.filter_boxes(object(),[red]),[red])
                self.assertFalse(frame_verified(verifier.verified_boxes,0,red.xyxy[0]))
        verifier.model=lambda *a,**k:[SimpleNamespace(boxes=[])]
        with patch('jersey_color.torso_fractions',return_value=fractions):
            self.assertEqual(verifier.filter_boxes(object(),[red]),[red])
            self.assertEqual(verifier.verified_boxes,[])
