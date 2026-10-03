from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).parents[2]/'perception'))
from person_verifier import PersonVerifier,overlap


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
