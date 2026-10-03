"""Execute the real ROS publication block without loading YOLO or Gazebo."""
import ast
import json
from pathlib import Path
import unittest
from types import SimpleNamespace


class PerceptionDiscoveryTests(unittest.TestCase):
    def publish(self, observed_s=9.8, miss=0):
        source = Path(__file__).parents[2] / 'perception/perception_real.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))
        block = next(n for n in ast.walk(tree) if isinstance(n, ast.If)
                     and isinstance(n.test, ast.BoolOp)
                     and isinstance(n.test.values[0], ast.Name)
                     and n.test.values[0].id == 'COORD_ON')
        visual, legacy = [], []
        tk = SimpleNamespace(miss=miss, observed_s=observed_s, x=1., y=2., conf=.87,
                             pub_xy=lambda: (99., 99.), uv=(30., 40.))
        env = dict(COORD_ON=True, COORD_HZ=2., now=10., _coord_t=0., _obs_seq=0,
            _published_visual_samples={},
            pub_list=[], visual_pub_list=[('green', 0, tk)], pubs={'green': [None]},
            coord=SimpleNamespace(publish=legacy.append),
            visual_coord=SimpleNamespace(publish=visual.append),
            rospy=SimpleNamespace(loginfo_throttle=lambda *a: None),
            os=SimpleNamespace(environ={'ROBOCUP_RUN_ID': 'run', 'PR_LOGICAL_UAV_ID': 'uav_1'}),
            String=SimpleNamespace, json=json, TARGET_Z=0., UAV='uav_1', frame_stamp=observed_s)
        exec(compile(ast.Module(body=[block], type_ignores=[]), str(source), 'exec'), env)
        return [json.loads(m.data) for m in visual], legacy

    def test_unassigned_person_discovered_with_original_evidence(self):
        visual, legacy = self.publish()
        self.assertEqual(len(visual), 1)
        self.assertEqual(legacy, [])
        self.assertEqual(visual[0]['xyz'], [1., 2., 0.])
        self.assertEqual(visual[0]['sample_s'], 9.8)
        self.assertEqual(visual[0]['confidence'], .87)
        self.assertEqual(visual[0]['run_id'], 'run')

    def test_unassigned_coast_or_old_frame_still_cannot_create_evidence(self):
        self.assertEqual(self.publish(miss=1)[0], [])
        self.assertEqual(self.publish(observed_s=8.8)[0], [])
