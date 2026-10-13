"""Capture times stay immutable across header delays, outages and epochs."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'coordination/src/robocup_swarm/scripts'))
from perception import perception_real as pr
import yolo_target_bridge as bridge


class CaptureTimeTests(unittest.TestCase):
    def frame(self, stamp=100., epoch=0):
        return dict(img=object(), stamp=stamp, time_epoch=epoch)

    def test_small_pose_delay_waits_for_original_capture_without_restamping(self):
        frame = self.frame(100.02)
        ready, pending = pr.sampled_image_ready(frame, None, 100., 0)
        self.assertIsNone(ready)
        self.assertIs(pending, frame)
        # A later camera callback cannot relabel the waiting frame as new.
        newer = self.frame(100.1)
        ready, pending = pr.sampled_image_ready(newer, pending, 100.03, 0)
        self.assertIs(ready, frame)
        self.assertIsNone(pending)
        self.assertEqual(ready['stamp'], 100.02)

    def test_stale_future_zero_and_old_epoch_are_not_observations(self):
        for stamp, epoch in ((99.49, 0), (100.06, 0), (0., 0),
                             (float('nan'), 0), (100., 1)):
            with self.subTest(stamp=stamp, epoch=epoch):
                self.assertEqual(pr.sampled_image_ready(self.frame(stamp, epoch), None,
                                                        100., 0), (None, None))

    def test_future_pending_frame_does_not_survive_reset(self):
        pending = self.frame(100.02)
        self.assertEqual(pr.sampled_image_ready(self.frame(5., 1), pending, 5., 1),
                         (None, None))

    def test_missing_capture_time_and_invalid_clock_never_convert_or_refresh(self):
        for sample, valid in ((0., True), (100., False)):
            with self.subTest(sample=sample, valid=valid), \
                    patch.object(pr.rospy, 'is_time_valid', return_value=valid), \
                    patch.object(pr, 'CvBridge') as converter:
                pr.on_img(NS(header=NS(stamp=NS(to_sec=lambda: sample))))
                converter.assert_not_called()

    def test_reset_during_image_conversion_cannot_create_new_epoch_image(self):
        message = NS(header=NS(stamp=NS(to_sec=lambda: 100.)))
        old = dict(pr._latest)
        with patch.object(pr.rospy, 'is_time_valid', return_value=True), \
                patch.object(pr.rospy, 'time_epoch', side_effect=[0, 1]), \
                patch.object(pr.rospy.Time, 'now', return_value=pr.rospy.Time.from_sec(100.)), \
                patch.object(pr, 'CvBridge') as converter:
            converter.return_value.imgmsg_to_cv2.return_value = object()
            pr.on_img(message)
        self.assertEqual(pr._latest, old)


class BridgeTimeTests(unittest.TestCase):
    def node(self):
        node = bridge._mk_bridge()
        node._time_gate_enabled = True
        node._time_epoch = 0
        node._rospy.is_time_valid = lambda: True
        node._rospy.time_epoch = lambda: 0
        return node

    def test_invalid_source_stops_input_and_all_output(self):
        node = self.node()
        node._rospy.is_time_valid = lambda: False
        with patch.object(bridge, 'parse_report_frame') as parse:
            node._report_frame_cb(NS(data='{}'))
            parse.assert_not_called()
        node._publish_actor('blue', 10., 20.)
        node._emit(dict(tid=2))
        node._tick_locked(None)
        self.assertEqual(node.pub.msgs, [])
        self.assertEqual(node._actor_pubs['blue'].msgs, [])

    def test_changed_epoch_blocks_old_output_even_with_new_valid_clock(self):
        node = self.node()
        node._rospy.time_epoch = lambda: 1
        node._publish_actor('blue', 10., 20.)
        self.assertEqual(node._actor_pubs['blue'].msgs, [])

    def test_reset_clears_old_evidence_and_cannot_resend_old_points(self):
        node = self.node()
        for index in range(6):
            node.core.report(100.+index*.2, 'blue', 10., 20., .9, 8., source_uav='u0')
        old_core = node.core
        node._gate_live['blue'] = True
        node._pub_last['blue'] = 101.
        node._last_source_sample[('u0', 'blue')] = 101.
        node._red_consumed.add('red1')
        node._rospy.time_epoch = lambda: 1
        node._on_time_reset(1)
        self.assertIsNot(node.core, old_core)
        self.assertFalse(node.core.tracks['blue'].alive)
        self.assertEqual(node._gate_live, {})
        self.assertEqual(node._pub_last, {})
        self.assertEqual(node._last_source_sample, {})
        self.assertEqual(node._red_consumed, set())
        node._advance(5.)
        node._tick_locked(None)
        self.assertEqual(node._actor_pubs['blue'].msgs, [])

    def test_clock_failure_during_publish_preparation_blocks_final_report(self):
        node = self.node()
        node._rospy.is_time_valid = unittest.mock.Mock(side_effect=[True, False])
        node._publish_actor('blue', 10., 20.)
        self.assertEqual(node._actor_pubs['blue'].msgs, [])
        self.assertEqual(node._pub_last, {})


if __name__ == '__main__':
    unittest.main()
