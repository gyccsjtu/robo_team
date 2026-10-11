#!/usr/bin/env python3
"""Offline tests for the ROS fleet readiness probe."""

import argparse
import contextlib
import io
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))

from wait_match_ready import (DEFAULT_UAVS, ReadyTracker, expected_topics,
                              is_real_message, parse_arguments, positive_seconds)


class ReadyProbeTest(unittest.TestCase):
    def test_default_topics_cover_six_uavs_and_three_kinds(self):
        args = parse_arguments([])
        topics = expected_topics(args.kinds, args.uavs)
        self.assertEqual(len(topics), 18)
        self.assertEqual(args.uavs, DEFAULT_UAVS)
        self.assertEqual(topics[("camera", "typhoon_h480_5")],
                         "/typhoon_h480_5/stereo_camera/left/image_raw")

    def test_selected_kinds_and_uavs(self):
        args = parse_arguments(["--kind", "scan", "--kind", "state", "--uavs",
                                "typhoon_h480_1,typhoon_h480_3", "--timeout", "2.5"])
        self.assertEqual(args.kinds, ("state", "scan"))
        self.assertEqual(args.uavs, ("typhoon_h480_1", "typhoon_h480_3"))
        self.assertEqual(args.timeout, 2.5)
        self.assertEqual(len(expected_topics(args.kinds, args.uavs)), 4)

    def test_invalid_timeout_and_duplicate_uav_rejected(self):
        for value in ("0", "-1", "nan", "inf", "oops"):
            with self.assertRaises(argparse.ArgumentTypeError):
                positive_seconds(value)
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parse_arguments(["--uavs", "typhoon_h480_0", "typhoon_h480_0"])

    def test_real_message_requirements(self):
        self.assertFalse(is_real_message("state", SimpleNamespace(connected=False)))
        self.assertTrue(is_real_message("state", SimpleNamespace(connected=True)))
        self.assertFalse(is_real_message("camera", SimpleNamespace(
            width=640, height=480, data=b"")))
        self.assertTrue(is_real_message("camera", SimpleNamespace(
            width=640, height=480, data=b"x")))
        self.assertFalse(is_real_message("scan", SimpleNamespace(ranges=[])))
        self.assertTrue(is_real_message("scan", SimpleNamespace(ranges=[1.0])))

    def test_tracker_reports_only_missing_real_messages(self):
        topics = expected_topics(("state", "scan"), ("typhoon_h480_0",))
        tracker = ReadyTracker(topics)
        tracker.on_message(SimpleNamespace(connected=False), ("state", "typhoon_h480_0"))
        tracker.on_message(SimpleNamespace(ranges=[2.0]), ("scan", "typhoon_h480_0"))
        self.assertEqual(tracker.missing(), [
            (("state", "typhoon_h480_0"), "/typhoon_h480_0/mavros/state")])


if __name__ == "__main__":
    unittest.main()
