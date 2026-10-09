"""Execute the actual production candidate loop without loading ROS/models."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

SOURCE = Path(__file__).parents[2]/'perception/perception_real.py'
TREE = ast.parse(SOURCE.read_text(encoding='utf-8'))
LOOP = next(n for n in ast.walk(TREE) if isinstance(n, ast.For)
            and isinstance(n.iter, ast.Call) and isinstance(n.iter.func, ast.Name)
            and n.iter.func.id == 'enumerate' and len(n.iter.args) == 1
            and isinstance(n.iter.args[0], ast.Name) and n.iter.args[0].id == 'dets'
            and any(isinstance(c, ast.Name) and c.id == 'appearance_similarity' for c in ast.walk(n)))
INITIAL = next(n for n in ast.walk(TREE) if isinstance(n, ast.Assign) and LOOP.lineno-10 <= n.lineno < LOOP.lineno
               and isinstance(n.targets[0], ast.Tuple)
               and [getattr(c,'id',None) for c in n.targets[0].elts] == ['best','bd'])
CODE = compile(ast.Module(body=[INITIAL, LOOP], type_ignores=[]), str(SOURCE), 'exec')


def candidate(distance, appearance=0., **extra):
    return dict(cls='green', strict=True, h=1.75, xyz=(distance, 0.),
                appearance=appearance, **extra)


def select(detections, confirmed=True, used=None):
    scope = dict(math=math, tk=NS(cls='green', h=1.75, appearance=1.),
        gate=3., confirmed=confirmed, H_GATE=.5, APP_W=.3, tx=0., ty=0.,
        used=used or [False]*len(detections), dets=detections,
        dyn_rej=0, best_appear_score=0., appearance_similarity=lambda a,b:b)
    exec(CODE, scope)
    return scope['best'], scope['bd']


class AssociationTests(unittest.TestCase):
    def test_better_score_is_not_pruned_by_previous_weighted_score(self):
        # Old loop chooses index0: raw1.3 is larger than previous score1.2.
        # Correct ranking compares score1.021 against1.2, both within gate3.
        index, score = select([candidate(2.1, 1.), candidate(1.3, .31)])
        self.assertEqual(index, 1)
        self.assertAlmostEqual(score, 1.021)

    def test_unique_winner_is_order_independent(self):
        a, b = candidate(2.1, 1.), candidate(1.3, .31)
        self.assertEqual(select([a,b])[0], 1)
        self.assertEqual(select([b,a])[0], 0)

    def test_unconfirmed_track_keeps_nearest_strict_candidate(self):
        self.assertEqual(select([candidate(2.1,1.), candidate(1.3,.31)], False)[0], 1)

    def test_appearance_never_expands_physical_gate(self):
        self.assertEqual(select([candidate(3.,1.), candidate(3.1,1.)])[0], -1)

    def test_identity_height_and_used_filters_remain(self):
        detections = [candidate(.1), candidate(.2), candidate(.3), candidate(1.)]
        detections[0]['cls'] = 'blue'
        detections[1]['h'] = 2.5
        self.assertEqual(select(detections, used=[False,False,True,False])[0], 3)

    def test_unconfirmed_loose_detection_still_rejected(self):
        a = candidate(.1); a['strict'] = False
        self.assertEqual(select([a, candidate(1.)], False)[0], 1)

    def test_weak_appearance_candidate_cannot_overwrite_lower_score(self):
        self.assertEqual(select([candidate(1.,1.), candidate(.5,.1)])[0], 0)


if __name__ == '__main__':
    unittest.main()
