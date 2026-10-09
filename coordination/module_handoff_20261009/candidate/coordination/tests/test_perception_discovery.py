"""Execute the real ROS publication block without loading YOLO or Gazebo."""
import ast
import json
from pathlib import Path
import unittest
import sys
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).parents[2]/'perception'))
from camera_geometry import image_time_position


class PerceptionDiscoveryTests(unittest.TestCase):
    def publish(self, observed_s=9.8, miss=0, camera_s=None, color='green', original_colors='', candidate=False, provisional=False):
        source = Path(__file__).parents[2] / 'perception/perception_real.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))
        block = next(n for n in ast.walk(tree) if isinstance(n, ast.If)
                     and isinstance(n.test, ast.BoolOp)
                     and isinstance(n.test.values[0], ast.Name)
                     and n.test.values[0].id == 'COORD_ON')
        visual, legacy = [], []
        tk = SimpleNamespace(miss=miss, observed_s=observed_s, x=1., y=2., conf=.87,
                             raw_xy=(3.,4.),
                             pub_xy=lambda: (99., 99.), uv=(30., 40.),
                             camera_s=observed_s if camera_s is None else camera_s,
                             camera_xyz=[-3.,2.,2.5],
                             person_support=SimpleNamespace(current_verified=True,hits=2),
                             hits=2,navigation_provisional=provisional)
        tk.blue_identity=(SimpleNamespace(allowed=lambda now:False,approach_allowed=lambda now:True)
                          if color=='blue' and provisional else None)
        env = dict(COORD_ON=True, COORD_HZ=2., now=10., _coord_t=0., _obs_seq=0,
            _published_visual_samples={}, navigation_candidates={color:tk} if candidate else {},
            pub_list=[], visual_pub_list=[(color, 0, tk)], pubs={color: [None]},
            coord=SimpleNamespace(publish=legacy.append),
            visual_coord=SimpleNamespace(publish=visual.append),
            rospy=SimpleNamespace(loginfo_throttle=lambda *a: None),
            image_time_position=image_time_position,
            os=SimpleNamespace(environ={'ROBOCUP_RUN_ID': 'run', 'PR_LOGICAL_UAV_ID': 'uav_1',
                                       'PR_ORIGINAL_REPORT_COLORS':original_colors}),
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
        self.assertEqual(visual[0]['schema_version'], 3)
        self.assertEqual(visual[0]['camera_xyz'], [-3.,2.,2.5])
        self.assertIs(visual[0]['person_frame_verified'], True)

    def test_unassigned_coast_or_old_frame_still_cannot_create_evidence(self):
        self.assertEqual(self.publish(miss=1)[0], [])
        self.assertEqual(self.publish(observed_s=8.8)[0], [])
        self.assertEqual(self.publish(camera_s=9.9)[0], [])

    def test_candidate_is_explicit_and_uses_original_coordinate(self):
        visual,legacy=self.publish(candidate=True)
        self.assertEqual(legacy,[])
        self.assertEqual(visual[0]['schema_version'],5)
        self.assertEqual(visual[0]['evidence_kind'],'navigation_candidate')
        self.assertEqual(visual[0]['xyz'],[3.,4.,0.])

    def test_early_white_publish_stays_candidate_with_original_proof(self):
        visual,legacy=self.publish(color='white',candidate=True,provisional=True)
        self.assertEqual(legacy,[])
        self.assertEqual(visual[0]['schema_version'],6)
        self.assertEqual(visual[0]['candidate_reason'],'white_early_proof')
        self.assertEqual(visual[0]['person_hits'],2)
        self.assertEqual(visual[0]['track_hits'],2)
        self.assertEqual(visual[0]['xyz'],[3.,4.,0.])

    def test_original_blue_position_keeps_same_image_time_and_proof(self):
        visual,_=self.publish(color='blue',original_colors='blue')
        self.assertEqual(visual[0]['xyz'],[3.,4.,0.])
        self.assertEqual(visual[0]['sample_s'],9.8)
        self.assertIs(visual[0]['person_frame_verified'],True)
        self.assertEqual(self.publish(color='white',original_colors='blue')[0][0]['xyz'],[1.,2.,0.])
        self.assertEqual(self.publish(color='blue',original_colors='blue',miss=1)[0],[])

    def test_short_blue_motion_publishes_navigation_only_original(self):
        visual,legacy=self.publish(color='blue',candidate=True,provisional=True)
        self.assertEqual(legacy,[])
        m=visual[0]
        self.assertEqual(m['schema_version'],7)
        self.assertEqual(m['candidate_reason'],'blue_approach_motion')
        self.assertIs(m['motion_identity_verified'],False)
        self.assertIs(m['approach_motion_verified'],True)
        self.assertEqual(m['xyz'],[3.,4.,0.])
        self.assertEqual(m['sample_s'],9.8)
