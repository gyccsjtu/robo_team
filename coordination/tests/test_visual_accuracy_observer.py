import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
from observe_visual_accuracy import paired_error


class VisualAccuracyObserverTests(unittest.TestCase):
    def test_coordinates_compare_at_original_image_time(self):
        truth = [(10.,0.,0.), (10.1,.08,0.)]
        self.assertAlmostEqual(paired_error(10.05,[.04,0.],truth),0.)
        self.assertAlmostEqual(paired_error(10.05,[.54,0.],truth),.5)

    def test_missing_old_future_or_sparse_truth_is_not_zero_error(self):
        for truth, stamp in (([],10.), ([(10.,0.,0.)],10.),
                ([(10.,0.,0.),(10.1,.08,0.)],9.),
                ([(10.,0.,0.),(10.1,.08,0.)],11.),
                ([(10.,0.,0.),(11.,.8,0.)],10.5)):
            self.assertIsNone(paired_error(stamp,[0.,0.],truth))
