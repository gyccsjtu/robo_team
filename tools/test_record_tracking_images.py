"""Replay quota exhaustion and source-clock faults without a ROS master."""
import math
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import record_tracking_images as recorder
from record_tracking_images import (COLORS, SOURCES, ImageQuota, evidence_color,
                                    matching_image, message_callerid, parse_args)


class ImageEvidenceTests(unittest.TestCase):
    def test_original_default_per_target_cadence_and_total_limit(self):
        q = ImageQuota(3, 650.)
        self.assertEqual(q.reserve(SOURCES[0], 't4', 100., 1.), 0)
        self.assertIsNone(q.reserve(SOURCES[0], 't4', 101., 2.))
        self.assertEqual(q.reserve(SOURCES[0], 't5', 100., 2.), 1)
        self.assertEqual(q.reserve(SOURCES[1], 't4', 100., 2.), 2)
        self.assertIsNone(q.reserve(SOURCES[0], 't4', 104., 4.))
        args = parse_args(['/tmp/evidence'])
        self.assertEqual((args.limit, args.seconds, args.fair_quota), (180, 650., False))
        self.assertFalse(args.spread_quota)

    def test_red_flood_cannot_spend_blue_or_late_or_other_camera_slots(self):
        q = ImageQuota(900, 650., fair=True)
        # Same failure shape as 031135: early repeated red views dominate arrivals.
        for i in range(100):
            q.reserve(SOURCES[0], 'red:42', 100.+2*i, 20.)
        self.assertEqual(q.count, 10)
        self.assertIsNotNone(q.reserve(SOURCES[1], 'red:1', 500., 20.))
        for color in ('blue', 'white', 'green', 'brown'):
            self.assertIsNotNone(q.reserve(SOURCES[0], color+':1', 500., 600.))
        self.assertIsNotNone(q.reserve(SOURCES[0], 'red:42', 502., 600.))
        self.assertEqual(q.caps[(2, 'blue', SOURCES[5])], 10)

    def test_total_bound_and_remainder_for_all_thirty_camera_colour_pairs(self):
        q = ImageQuota(311, 650., fair=True)
        self.assertEqual(sum(q.caps.values()), 311)
        stamps = dict.fromkeys(SOURCES, 100.)
        for slot, cap in q.caps.items():
            window, color, source = slot
            for _ in range(cap):
                stamps[source] += 2.
                self.assertIsNotNone(q.reserve(source, color, stamps[source],
                                              (window+.5)*650./3))
            stamps[source] += 2.
            self.assertIsNone(q.reserve(source, color, stamps[source],
                                        (window+.5)*650./3))
        self.assertEqual(q.count, 311)

    def test_source_reset_out_of_order_and_invalid_time_are_bounded(self):
        q = ImageQuota(90, 650., fair=True)
        self.assertIsNotNone(q.reserve(SOURCES[0], 't1', 100., 1.))
        # Full early blue slot still observes the newest source timestamp.
        self.assertIsNone(q.reserve(SOURCES[0], 't1', 200., 2.))
        self.assertIsNone(q.reserve(SOURCES[0], 't3', 150., 300.))
        self.assertIsNone(q.reserve(SOURCES[0], 't3', 1., 500.))
        self.assertEqual(q.rejected['source_stamp_regressed'], 2)
        self.assertIsNotNone(q.reserve(SOURCES[1], 't1', 1., 500.))
        for stamp in (0., -1., math.inf, math.nan):
            self.assertIsNone(q.reserve(SOURCES[2], 't1', stamp, 500.))
        for elapsed in (-1., 650., math.inf, math.nan):
            self.assertIsNone(q.reserve(SOURCES[2], 't1', 200., elapsed))
        self.assertEqual(q.count, 2)

    def test_red_aliases_share_fair_quota_but_distinct_colours_are_kept(self):
        self.assertEqual([evidence_color(x) for x in ('red1', 'red2', 'red:1', 't4', 't5')],
                         ['red']*5)
        q = ImageQuota(90, 650., fair=True)
        self.assertIsNotNone(q.reserve(SOURCES[0], 't4', 100., 1.))
        self.assertIsNone(q.reserve(SOURCES[0], 't5', 104., 1.))
        self.assertIsNotNone(q.reserve(SOURCES[0], 't1', 104., 1.))
        self.assertIsNone(q.reserve('arbitrary_source', 'blue', 104., 1.))
        self.assertIsNone(q.reserve(SOURCES[0], 'arbitrary_color', 104., 1.))

    def test_spread_opens_at_twelve_seconds_and_resets_only_at_window_boundary(self):
        q = ImageQuota(1800, 720., fair=True, windows=6, spread=True)
        self.assertIsNotNone(q.reserve(SOURCES[0], 'red:1', 100., 0.))
        self.assertIsNone(q.reserve(SOURCES[0], 'red:1', 111.999, 11.999))
        self.assertEqual(q.rejected['fair_slot_wait'], 1)
        for elapsed in range(12, 109, 12):
            self.assertIsNotNone(q.reserve(SOURCES[0], 'red:1', 100.+elapsed, elapsed))
        self.assertEqual(q.used[(0, 'red', SOURCES[0])], 10)
        self.assertIsNone(q.reserve(SOURCES[0], 'red:1', 219.999, 119.999))
        self.assertIsNotNone(q.reserve(SOURCES[0], 'red:1', 220., 120.))
        self.assertEqual(q.used[(1, 'red', SOURCES[0])], 1)
        self.assertIsNone(q.reserve(SOURCES[0], 'red:1', 221., 121.))
        self.assertIsNotNone(q.reserve(SOURCES[0], 'red:1', 232., 132.))
        self.assertIsNone(q.reserve(SOURCES[0], 'red:1', 820., 720.))

    def test_spread_early_flood_cannot_exhaust_late_budget_but_old_fair_can(self):
        spread = ImageQuota(1800, 720., fair=True, windows=6, spread=True)
        old = ImageQuota(1800, 720., fair=True, windows=6)
        for elapsed in range(0, 21, 2):
            spread.reserve(SOURCES[0], 'red:1', 100.+elapsed, elapsed)
            old.reserve(SOURCES[0], 'red:1', 100.+elapsed, elapsed)
        self.assertEqual(spread.used[(0, 'red', SOURCES[0])], 2)
        self.assertEqual(old.used[(0, 'red', SOURCES[0])], 10)
        self.assertIsNotNone(spread.reserve(SOURCES[0], 'red:1', 208., 108.))
        self.assertIsNone(old.reserve(SOURCES[0], 'red:1', 208., 108.))

    def test_spread_colour_source_and_window_slots_open_independently(self):
        q = ImageQuota(1800, 720., fair=True, windows=6, spread=True)
        self.assertIsNotNone(q.reserve(SOURCES[0], 't4', 100., 0.))
        self.assertIsNotNone(q.reserve(SOURCES[0], 't1', 102., 0.))
        self.assertIsNotNone(q.reserve(SOURCES[1], 't4', 100., 0.))
        self.assertIsNone(q.reserve(SOURCES[0], 't5', 104., 2.))
        self.assertIsNotNone(q.reserve(SOURCES[0], 't5', 220., 120.))
        self.assertEqual(q.count, 4)
        self.assertEqual(sum(q.caps.values()), 1800)

    def test_spread_new_allowance_never_renews_duplicate_frame_or_two_second_cadence(self):
        q = ImageQuota(1800, 720., fair=True, windows=6, spread=True)
        self.assertIsNotNone(q.reserve(SOURCES[0], 'red:1', 100., 0.))
        self.assertIsNone(q.reserve(SOURCES[0], 'red:2', 100., 12.))
        self.assertEqual(q.count, 1)
        self.assertEqual(q.rejected['sample_interval'], 1)
        self.assertIsNone(q.reserve(SOURCES[0], 'red:2', 101.999999, 24.))
        self.assertIsNotNone(q.reserve(SOURCES[0], 'red:2', 102., 24.))
        self.assertIsNone(q.reserve(SOURCES[0], 'red:3', 102., 36.))
        # Crossing a wall window does not reset acquisition-time spacing.
        self.assertIsNone(q.reserve(SOURCES[0], 'red:4', 103.999999, 120.))
        self.assertIsNotNone(q.reserve(SOURCES[0], 'red:4', 104., 120.))

    def test_spread_wait_observes_ros_reset_and_invalid_wall_time_remains_bounded(self):
        q = ImageQuota(1800, 720., fair=True, windows=6, spread=True)
        self.assertIsNotNone(q.reserve(SOURCES[0], 'blue:1', 100., 0.))
        self.assertIsNone(q.reserve(SOURCES[0], 'blue:1', 200., 1.))
        self.assertIsNone(q.reserve(SOURCES[0], 'white:1', 150., 12.))
        self.assertIsNone(q.reserve(SOURCES[0], 'blue:1', 1., 120.))
        self.assertEqual(q.rejected['source_stamp_regressed'], 2)
        for elapsed in (-1., 720., math.nan, math.inf):
            self.assertIsNone(q.reserve(SOURCES[1], 'blue:1', 500., elapsed))
        self.assertEqual(q.count, 1)

    def test_spread_cli_requires_fair_and_preserves_requested_next_run_budget(self):
        with self.assertRaises(ValueError):
            ImageQuota(1800, 720., spread=True)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(['/tmp/evidence', '--spread-quota'])
        args = parse_args(['/tmp/evidence', '--fair-quota', '--spread-quota',
                           '--limit', '1800', '--seconds', '720', '--quota-windows', '6'])
        self.assertTrue(args.fair_quota and args.spread_quota)
        self.assertEqual((args.limit, args.seconds, args.quota_windows), (1800, 720., 6))

    def test_spread_total_and_remainder_remain_bounded_after_all_slots_open(self):
        q = ImageQuota(1801, 720., fair=True, windows=6, spread=True)
        self.assertEqual(sum(q.caps.values()), 1801)
        stamps = dict.fromkeys(SOURCES, 100.)
        for (window, color, source), cap in q.caps.items():
            for _ in range(cap):
                stamps[source] += 2.
                self.assertIsNotNone(q.reserve(source, color, stamps[source],
                                              (window+1)*120.-.001))
            stamps[source] += 2.
            self.assertIsNone(q.reserve(source, color, stamps[source],
                                        (window+1)*120.-.001))
        self.assertEqual(q.count, 1801)

    def test_matching_is_exact_not_nearest_and_respects_acquisition_stamp(self):
        def frame(stamp):
            return NS(header=NS(stamp=NS(to_sec=lambda: stamp)))
        original, newer = frame(100.), frame(100.2)
        self.assertIs(matching_image([original, newer], 100.), original)
        self.assertIsNone(matching_image([original, newer], 100.1))
        self.assertIsNone(matching_image([newer], 100.))

    def test_cli_fair_budget_cannot_omit_camera_colour_time_slots(self):
        self.assertEqual(len(COLORS)*len(SOURCES)*3, 90)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(['/tmp/evidence', '--fair-quota', '--limit', '89'])
        for value in ('0', '-1', 'nan', 'inf'):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(['/tmp/evidence', '--seconds', value])

    def test_callerid_is_optional_transport_value_not_payload_identity(self):
        fake_payload = json.dumps({'publisher_callerid': '/forged'})
        self.assertEqual(message_callerid(NS(data=fake_payload,
            _connection_header={'callerid': '/actual_perception'})), '/actual_perception')
        self.assertIsNone(message_callerid(NS(data=fake_payload)))
        self.assertIsNone(message_callerid(NS(_connection_header={'callerid': 42})))

    def test_candidate_and_view_recorders_save_exact_image_and_transport_identity(self):
        # Exercise actual recorder callbacks and files with a fake read-only ROS bus.
        for candidates, spread in ((False, False), (True, False),
                                   (False, True), (True, True)):
            subscribers = []
            def subscribe(topic, kind, callback, callback_args=None, **kwargs):
                subscribers.append(topic)
                if topic.startswith('/typhoon_h480_0/'):
                    for stamp in (200., 202.):
                        image = NS(header=NS(stamp=NS(to_sec=lambda s=stamp: s)), data=stamp)
                        callback(image, callback_args)
                elif topic.startswith('/swarm/'):
                    for stamp, color, target in ((200., 'blue', 't1'),
                                                  (201., 'white', 't3'),
                                                  (202., 'red', 't4')):
                        payload = dict(source_uav=SOURCES[0], sample_s=stamp,
                            candidate_id=color+':7', target_id=target,
                            publisher_callerid='/forged')
                        callback(NS(data=json.dumps(payload),
                            _connection_header={'callerid': '/actual_perception'}))
                return NS(unregister=lambda: None)
            def write_image(path, image, options):
                Path(path).write_text(str(image))
                return True
            modules = dict(rospy=NS(init_node=lambda *a, **k: None,
                Subscriber=subscribe, is_shutdown=lambda: True,
                logwarn_throttle=lambda *a: None),
                cv2=NS(imwrite=write_image, IMWRITE_JPEG_QUALITY=1),
                cv_bridge=NS(CvBridge=lambda: NS(imgmsg_to_cv2=lambda m, **k: m.data)),
                sensor_msgs=NS(), **{'sensor_msgs.msg': NS(Image=object)},
                std_msgs=NS(), **{'std_msgs.msg': NS(String=object)})
            with tempfile.TemporaryDirectory() as directory:
                argv = ['record_tracking_images.py', directory, '--fair-quota', '--limit', '90']
                if candidates:
                    argv.append('--candidates')
                if spread:
                    argv.append('--spread-quota')
                with patch.dict(sys.modules, modules), patch.object(sys, 'argv', argv):
                    recorder.main()
                rows = [json.loads(line) for line in (Path(directory)/'images.jsonl').read_text().splitlines()]
                self.assertEqual([row['capture_s'] for row in rows], [200., 202.])
                self.assertTrue(all(row['publisher_callerid']=='/actual_perception' for row in rows))
                self.assertEqual((Path(directory)/'0000.jpg').read_text(), '200.0')
                summary = json.loads((Path(directory)/'summary.json').read_text())
                config = json.loads((Path(directory)/'config.json').read_text())
                self.assertEqual(config['spread_quota'], spread)
                self.assertEqual(summary['exact_image_missing'], 1)
                self.assertEqual(summary['images_written'], 2)
                self.assertEqual(subscribers[-1], '/swarm/visual_candidate' if candidates
                                 else '/swarm/tracking_view')


if __name__ == '__main__':
    unittest.main()
