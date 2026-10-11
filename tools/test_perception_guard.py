"""Offline regressions for fixed-object cross-color visual confusion."""

import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))

import yolo_target_bridge as bridge


class CrossColorGuardTests(unittest.TestCase):
    def setUp(self):
        self.guard = bridge.CrossColorLocationGuard()

    def test_replays_same_camera_blue_then_white_fixed_object(self):
        # 22:45 run: UAV3 blue at 107/110 s, then white at 114/115 s,
        # always at approximately (-27.1, 13.2).  This cannot be a reliable
        # class identity for an official judge report.
        self.assertIsNone(self.guard.observe(107.24, 'u3', 'blue',
                                             -27.09, 13.10, .876, True))
        self.assertIsNone(self.guard.observe(110.056, 'u3', 'blue',
                                             -27.08, 13.17, .812, True))
        self.assertIsNone(self.guard.observe(114.26, 'u3', 'white',
                                             -26.94, 13.47, .602, True))
        event = self.guard.observe(115.528, 'u3', 'white',
                                   -27.11, 13.16, .645, True)
        self.assertIsNotNone(event)
        self.assertEqual(event['target_ids'], ['t1', 't3'])
        self.assertEqual(event['source_uav'], 'u3')
        self.assertTrue(self.guard.blocked('blue', -27.03, 13.2, 115.6))
        self.assertTrue(self.guard.blocked('white', -27.03, 13.2, 115.6))
        self.assertFalse(self.guard.blocked('blue', 150.0, 38.0, 115.6))
        self.assertFalse(self.guard.blocked('red1', -27.03, 13.2, 115.6))
        self.assertIsNone(self.guard.observe(116.336, 'u3', 'white',
                                             -27.03, 13.32, .765, True))
        self.assertFalse(self.guard.blocked('white', -27.03, 13.2, 128.0))

    def test_stationary_real_actor_remains_reportable_without_other_color(self):
        for t in (100.0, 100.4, 100.8, 101.2):
            self.assertIsNone(self.guard.observe(t, 'u3', 'white',
                                                 150.0, 38.0, .83, True))
        self.assertFalse(self.guard.blocked('white', 150.0, 38.0, 101.3))

    def test_one_frame_other_color_and_different_camera_do_not_quarantine(self):
        self.guard.observe(100.0, 'u3', 'blue', 0, 0, .8, True)
        self.guard.observe(100.4, 'u3', 'blue', 0, 0, .8, True)
        self.assertIsNone(self.guard.observe(100.5, 'u3', 'white',
                                             0, 0, .8, True))
        self.assertIsNone(self.guard.observe(100.9, 'u4', 'white',
                                             0, 0, .8, True))
        self.assertIsNone(self.guard.observe(101.2, 'u4', 'white',
                                             0, 0, .8, True))
        self.assertFalse(self.guard.blocked('blue', 0, 0, 101.3))

    def test_two_nearby_real_people_seen_together_are_not_quarantined(self):
        # Close people may project to effectively the same ground coordinate.
        # Concurrent detections of both colors are evidence of two objects,
        # not a single fixed object whose YOLO class switched over time.
        for t, tag in ((100.0, 'blue'), (100.1, 'white'),
                       (100.4, 'blue'), (100.5, 'white'),
                       (100.8, 'blue'), (100.9, 'white')):
            self.assertIsNone(self.guard.observe(t, 'u3', tag,
                                                 -27.1, 13.2, .88, True))
        self.assertFalse(self.guard.blocked('blue', -27.1, 13.2, 101.0))
        self.assertFalse(self.guard.blocked('white', -27.1, 13.2, 101.0))

    def test_short_switch_gap_is_not_enough_to_quarantine_nearby_people(self):
        for t, tag in ((100.0, 'blue'), (100.4, 'blue'),
                       (100.8, 'white'), (101.2, 'white')):
            self.assertIsNone(self.guard.observe(t, 'u3', tag,
                                                 -27.1, 13.2, .88, True))
        self.assertFalse(self.guard.blocked('white', -27.1, 13.2, 101.3))

    def test_unusable_or_moving_observation_does_not_make_conflict(self):
        self.guard.observe(100.0, 'u3', 'blue', 0, 0, .8, True)
        self.guard.observe(100.5, 'u3', 'blue', 1.2, 0, .8, True)
        self.guard.observe(101.0, 'u3', 'white', 1.2, 0, .8, True)
        self.assertIsNone(self.guard.observe(101.5, 'u3', 'white',
                                             1.2, 0, .8, True))
        self.assertIsNone(self.guard.observe(102.0, 'u3', 'blue',
                                             1.2, 0, .4, True))
        self.assertIsNone(self.guard.observe(102.5, 'u3', 'blue',
                                             1.2, 0, .8, False))
        self.assertFalse(self.guard.blocked('white', 1.2, 0, 102.6))

    def test_bridge_blocks_only_conflicted_official_point(self):
        b = bridge._mk_bridge()

        def feed(t, tag, x, y):
            b._advance(t - b._now())
            b._report_cb(SimpleNamespace(data=json.dumps({
                'target_id': tag, 'frame_id': 'world_enu',
                'xyz': [x, y, 1.2], 'confidence': 1.0,
                'visual_confidence': .82, 'range_m': 8.0,
                'source_uav': 'u3', 'sample_s': t,
                'real_detection': True, 'official_view_ok': True,
            })))

        for t in (100.0, 100.4, 100.8):
            feed(t, 'blue', -27.1, 13.2)
        b._tick(None)
        self.assertEqual(len(b._actor_pubs['blue'].msgs), 1)

        for t in (105.0, 105.4, 105.8):
            feed(t, 'white', -27.1, 13.2)
        self.assertEqual(len(b.conflict_pub.msgs), 1)
        signal = json.loads(b.conflict_pub.msgs[0].data)
        self.assertEqual(signal['target_ids'], ['t1', 't3'])
        b._tick(None)
        self.assertEqual(len(b._actor_pubs['white'].msgs), 0)
        b._publish_actor('blue', -27.1, 13.2)
        self.assertEqual(len(b._actor_pubs['blue'].msgs), 1)

        # A genuine person of either class at another position remains free
        # to report, and the judge's three-field ActorInfo contract is intact.
        b._publish_actor('white', 150.0, 38.0)
        self.assertEqual(len(b._actor_pubs['white'].msgs), 1)
        msg = b._actor_pubs['white'].msgs[0]
        self.assertEqual((msg.cls, msg.x, msg.y), ('white', 150.0, 38.0))


if __name__ == '__main__':
    unittest.main()
