from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch
import unittest
sys.path.insert(0,str(Path(__file__).parents[2]/'perception'))
from jersey_color import supported,filter_boxes
from person_verifier import PersonVerifier


def fractions(**changes):
    result=dict(red=0.,green=0.,blue=0.,white=0.,brown=0.)
    result.update(changes)
    return result


class JerseyColorTests(unittest.TestCase):
    def test_actual_blue_person_reported_white_is_rejected(self):
        self.assertFalse(supported('white',fractions(blue=.2778,white=.0556,brown=.2778)))
        self.assertFalse(supported('white',fractions(blue=.4167,white=.1389)))

    def test_actual_brown_person_reported_green_is_rejected(self):
        self.assertFalse(supported('green',fractions(red=.1688,green=.026,brown=.3896)))

    def test_correct_white_and_green_with_background_remain(self):
        self.assertTrue(supported('white',fractions(white=.0938,brown=.0312)))
        self.assertTrue(supported('green',fractions(green=.5952,brown=.3452)))

    def test_dark_unsupported_white_is_rejected_but_tiny_crop_is_undecided(self):
        self.assertFalse(supported('white',fractions(white=.025,brown=.05)))
        self.assertTrue(supported('white',None))

    def test_veto_preserves_original_class_confidence_and_rectangle(self):
        white=SimpleNamespace(cls=3,conf=.435,xyxy=[[0,0,10,20]])
        red=SimpleNamespace(cls=0,conf=.84,xyxy=[[20,0,30,20]])
        with patch('jersey_color.torso_fractions',return_value=fractions(white=.3)):
            self.assertEqual(filter_boxes(object(),[white,red]),[white,red])
        self.assertEqual((white.cls,white.conf,white.xyxy),(3,.435,[[0,0,10,20]]))

    def test_local_fallback_uses_same_veto_without_loading_torch(self):
        wrong=SimpleNamespace(cls=3,conf=.8,xyxy=[[0,0,10,20]])
        with patch('jersey_color.torso_fractions',return_value=fractions(blue=.6)):
            verifier=PersonVerifier(color_check=True)
            self.assertEqual(verifier.filter_boxes(object(),[wrong]),[])
            self.assertIsNone(verifier.model)
