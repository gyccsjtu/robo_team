"""New distractor classes must never become camera cues or actor reports."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from perception import perception_real as vision


class DetectorClassMappingTests(unittest.TestCase):
    def test_existing_five_class_weights_preserve_ids(self):
        self.assertEqual(vision.target_class_mapping(vision.CLASSES),
                         dict(enumerate(range(5))))

    def test_reordered_six_class_weights_map_colors_and_exclude_distractor(self):
        names = ['distractor_person', 'white', 'red', 'blue', 'brown', 'green']
        mapping = vision.target_class_mapping(names)
        boxes = [NS(cls=i, conf=.95, xyxy=[[1., 2., 30., 100.]])
                 for i in range(6)]
        targets = vision.target_boxes_only(boxes, mapping)
        self.assertEqual([b.cls for b in targets], [3, 0, 2, 4, 1])
        self.assertEqual([b.cls for b in boxes], list(range(6)))
        self.assertEqual(vision.target_boxes_only(boxes[:1], mapping), [])

    def test_appended_distractor_does_not_reach_geometry_or_pixel_attention(self):
        mapping = vision.target_class_mapping(vision.CLASSES+['distractor_person'])
        self.assertEqual(vision.target_boxes_only(
            [NS(cls=5, conf=.99, xyxy=[[100., 100., 160., 450.]])], mapping), [])

    def test_missing_and_duplicate_actor_labels_fail_at_model_load(self):
        for names in (['person', 'car'], vision.CLASSES+['blue'], None):
            with self.subTest(names=names), self.assertRaises(ValueError):
                vision.target_class_mapping(names)


if __name__ == '__main__':
    unittest.main()
