import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('selection',Path(__file__).parent/'training/select_distractor_checkpoint.py')
selection=importlib.util.module_from_spec(spec);spec.loader.exec_module(selection)


def report(value):
    recalls={n:value for n in selection.training.NAMES[:5]}
    return dict(recall=dict(recalls,distractor_person=.8),usable_actor_recall=recalls,
                false_decoy_frames=0)


class SelectionTests(unittest.TestCase):
    def test_average_gain_cannot_hide_one_color_regression(self):
        baseline=report(.9);candidate=report(.99)
        candidate['recall']['green']=candidate['usable_actor_recall']['green']=.85
        self.assertFalse(selection.evaluation.compare(baseline,candidate)['image_acceptance'])
        failures=selection.failure_reasons('validation',baseline,candidate)
        self.assertEqual({r['color'] for r in failures},{'green'})

    def test_correct_raw_color_with_appearance_conflict_still_fails(self):
        baseline=report(.9);candidate=report(.99)
        candidate['usable_actor_recall']['white']=.8
        self.assertFalse(selection.evaluation.compare(baseline,candidate)['image_acceptance'])
        self.assertEqual(selection.failure_reasons('validation',baseline,candidate)[0]['metric'],'usable_actor_recall')

    def test_declared_ranking_prefers_sculpture_recall_then_earlier_tie(self):
        old=report(.9);actor=report(.95);a=report(.95);b=report(.95)
        a['recall']['distractor_person']=.9;b['recall']['distractor_person']=.8
        self.assertGreater(selection.rank_pass(actor,a,old,30),selection.rank_pass(actor,b,old,20))
        self.assertGreater(selection.rank_pass(actor,a,old,20),selection.rank_pass(actor,a,old,30))


if __name__=='__main__':
    unittest.main()
