"""Sensor geometry, report integration and adversarial instance/time cases."""
import json
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/
                       'coordination/src/robocup_swarm/scripts'))
import yolo_target_bridge as bridge
from visual_triangulation import camera_bearing, validate_bearing, intersection, MultiViewTrack


def row(source, sample, position=(0., 0.), bias=(.9, 0.), candidate=None):
    camera = (-8., 0.) if source == 'a' else (0., -8.)
    dx, dy = position[0]-camera[0], position[1]-camera[1]
    norm = math.hypot(dx, dy)
    return dict(source_uav=source, sample_s=sample, tag='blue', local_tag='blue',
                candidate_id=candidate or source+'-person', official_ok=True,
                appearance_status='actor', conf=.95, range_m=8.,
                x=position[0]+bias[0], y=position[1]+bias[1],
                bearing=dict(origin=camera, direction=(dx/norm, dy/norm)))


class TriangulationTests(unittest.TestCase):
    def test_camera_quaternion_and_complete_box(self):
        for yaw in (0., .7, -2.):
            camera = (2., 3., 4., 0., 0., math.sin(yaw/2), math.cos(yaw/2))
            result = camera_bearing(camera, (350., 160., 402., 320.))
            self.assertEqual(result['origin'], [2., 3.])
            self.assertAlmostEqual(result['direction'][0], math.cos(yaw))
            self.assertAlmostEqual(result['direction'][1], math.sin(yaw))
            self.assertIsNone(camera_bearing(camera, (350., 160., 402., 478.)))
        self.assertIsNone(camera_bearing(None, (1., 2., 3., 4.)))
        self.assertIsNone(validate_bearing(dict(origin=[0,0], direction=[0,0])))
        self.assertIsNone(validate_bearing(dict(origin=[float('nan'),0], direction=[1,0])))

    def test_intersection_removes_range_bias(self):
        result = intersection(row('a', 100.), row('b', 100.1))
        self.assertEqual(result, (100.05, 0., 0.))

    def test_degenerate_async_same_camera_and_disjoint_rejected(self):
        a, b = row('a', 100.), row('b', 100.)
        for change in (dict(sample_s=99.8), dict(source_uav='a'), dict(tag='white'),
                       dict(x=10.), dict(bearing=dict(origin=(0.,-8.),direction=(1.,0.))),
                       dict(bearing=dict(origin=(0.,8.),direction=(0.,1.)))):
            self.assertIsNone(intersection(a, dict(b, **change)))

    def feed(self, tracker, moving=False):
        for i in range(8):
            t = 100.+i*.2
            p = (1.2*(t-100.), .4*(t-100.)) if moving else (0.,0.)
            tracker.add(row('a',t,p))
            tracker.add(row('b',t,p))

    def test_moving_target_predicted_to_publication(self):
        tracker = MultiViewTrack(); self.feed(tracker, moving=True)
        result = tracker.point_at(101.6, bridge._robust_official_fit, (2.82,.64))
        self.assertAlmostEqual(result[0], 1.92)
        self.assertAlmostEqual(result[1], .64)
        self.assertIsNone(tracker.point_at(102.01, bridge._robust_official_fit, (3.,1.)))

    def test_repeated_image_never_builds_a_fit(self):
        tracker = MultiViewTrack(); tracker.add(row('a',100.))
        for i in range(20):
            tracker.add(row('b',100.+i*.005))
        self.assertEqual(len(next(iter(tracker.pairs.values()))[0]), 1)
        self.assertIsNone(tracker.point_at(100.1, bridge._robust_official_fit, (.9,0.)))

    def test_candidate_change_invalidates_old_pair(self):
        tracker = MultiViewTrack(); self.feed(tracker)
        self.assertIsNotNone(tracker.point_at(101.5,bridge._robust_official_fit,(.9,0.)))
        tracker.add(row('a',101.5,candidate='different'))
        self.assertIsNone(tracker.point_at(101.5,bridge._robust_official_fit,(.9,0.)))

    def test_recent_monocular_report_wins_over_old_intersection(self):
        tracker = MultiViewTrack(); self.feed(tracker)
        self.assertIsNone(tracker.point_at(101.8,bridge._robust_official_fit,(.9,0.)))

    def test_new_bad_intersection_does_not_use_previous_fit(self):
        tracker = MultiViewTrack(); self.feed(tracker)
        tracker.add(row('a',101.6,(2.,0.))); tracker.add(row('b',101.6,(2.,0.)))
        self.assertIsNone(tracker.point_at(101.7,bridge._robust_official_fit,(2.9,0.)))

    def test_core_integration_fallback_and_red_independence(self):
        core = bridge.TargetBridgeCore()
        with patch.object(bridge,'MULTIVIEW_REPORT',True):
            for i in range(8):
                for source in ('a','b'):
                    core.report_frame([row(source,100.+i*.2)])
            tr = core.tracks['blue']
            self.assertAlmostEqual(bridge.official_point_at(tr,101.5)[0],0.)
            self.assertEqual(tr.official_method,'two_camera')
            with patch.object(bridge,'MULTIVIEW_REPORT',False):
                self.assertAlmostEqual(bridge.official_point_at(tr,101.5)[0],.9)
            tr.tag = 'red1'
            self.assertAlmostEqual(bridge.official_point_at(tr,101.5)[0],.9)
            self.assertIsNone(bridge.official_point_at(tr,102.1))

    def test_rejected_instance_cannot_supply_a_ray(self):
        core = bridge.TargetBridgeCore()
        for i in range(8):
            core.report_frame([row('a',100.+i*.2)])
        for i in range(8):
            core.report_frame([row('b',101.5+i*.1,(8.,0.), candidate='statue')])
        self.assertNotIn('b',core.tracks['blue'].multiview.views)

    def test_invalid_optional_bearing_preserves_original_report(self):
        data = dict(schema_version=2,source_uav='a',sample_s=100.,real_detection=True,
                    candidates=[dict(target_id='blue',candidate_id='person',xyz=[8,0,1],
                                     camera_bearing={'origin':[1,2]},range_m=8.,
                                     confidence=.9,official_view_ok=True)])
        parsed = bridge.parse_report_frame(json.dumps(data),100.1)
        self.assertEqual(len(parsed),1)
        self.assertIsNone(parsed[0]['bearing'])


if __name__ == '__main__':
    unittest.main()
