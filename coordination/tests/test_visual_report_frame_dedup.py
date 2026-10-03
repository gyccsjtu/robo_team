"""Run the actual high-rate perception publish block with recorded frame times."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

source = Path(__file__).parents[2]/'perception/perception_real.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
block = next(n for n in ast.walk(tree) if isinstance(n, ast.If)
             and isinstance(n.test, ast.BoolOp)
             and len(n.test.values) >= 2
             and isinstance(n.test.values[0], ast.Name) and n.test.values[0].id == 'COORD_ON'
             and isinstance(n.test.values[1], ast.Name) and n.test.values[1].id == 'visual_pub_list')
code = compile(ast.fix_missing_locations(ast.Module(body=[block], type_ignores=[])),
               str(source), 'exec')


class VisualReportFrameDedupTests(unittest.TestCase):
    def test_fast_publish_checks_cannot_multiply_original_images_or_coast(self):
        emitted = []
        track = SimpleNamespace(miss=0, observed_s=10., x=2., y=3., conf=.6,
                                pub_xy=lambda: (2., 3.), uv=(None, None))
        scope = dict(COORD_ON=True, COORD_HZ=10., now=10., _coord_t=0., _obs_seq=0,
            visual_pub_list=[('white', 0, track)], pub_list=[], pubs={'white': [None]},
            _published_visual_samples={}, json=json, TARGET_Z=1.25, UAV='model',
            os=SimpleNamespace(environ={'ROBOCUP_RUN_ID':'run', 'PR_LOGICAL_UAV_ID':'uav_1'}),
            String=lambda **kw: SimpleNamespace(**kw), coord=Mock(),
            visual_coord=SimpleNamespace(publish=lambda msg: emitted.append(json.loads(msg.data))),
            rospy=SimpleNamespace(loginfo_throttle=Mock()), frame_stamp=10.)
        exec(code, scope)
        scope['now'] = 10.2
        exec(code, scope)
        self.assertEqual(len(emitted), 1)
        track.miss = 1
        scope['now'] = 10.4
        exec(code, scope)
        self.assertEqual(len(emitted), 1)
        track.miss, track.observed_s = 0, 10.6
        scope['now'] = 10.8
        exec(code, scope)
        self.assertEqual([x['sample_s'] for x in emitted], [10., 10.6])
        self.assertEqual([x['confidence'] for x in emitted], [.6, .6])
        self.assertGreater(emitted[1]['seq'], emitted[0]['seq'])
        track.observed_s = 12.
        scope['now'] = 11.
        exec(code, scope)
        track.observed_s = 10.6
        scope['now'] = 12.
        exec(code, scope)
        self.assertEqual(len(emitted), 2)
