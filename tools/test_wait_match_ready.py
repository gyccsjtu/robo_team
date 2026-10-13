#!/usr/bin/env python3
"""Offline tests for the ROS fleet readiness probe."""

import argparse
import contextlib
import io
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace, ModuleType
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))

from wait_match_ready import (DEFAULT_UAVS, ReadyTracker, expected_topics,
                              is_real_message, parse_arguments, positive_seconds)
import wait_match_ready as probe


class ReadyProbeTest(unittest.TestCase):
    def test_connection_probe_needs_no_pose_clock_and_disables_clock_subscription(self):
        transport = ModuleType('rospy')
        calls = []
        transport.init_node = lambda *args, **kwargs: calls.append(kwargs)
        transport.ROSException = RuntimeError
        transport.is_shutdown = lambda: False
        def subscribe(topic, cls, callback, callback_args, **kwargs):
            self.assertEqual(topic, '/typhoon_h480_0/mavros/state')
            callback(SimpleNamespace(connected=True), callback_args)
            return SimpleNamespace(unregister=lambda: None)
        transport.Subscriber = subscribe
        with patch.dict(sys.modules, rospy=transport), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(probe.main(['--kind', 'state', '--uavs', 'typhoon_h480_0']), 0)
        self.assertTrue(calls[0]['disable_rostime'])
        self.assertNotIn('clock_sources', calls[0])

    def test_airborne_probe_compares_original_status_with_header_time(self):
        transport = ModuleType('team_time_ros')
        calls = []
        transport.init_node = lambda *args, **kwargs: calls.append(kwargs)
        transport.ROSException = RuntimeError
        transport.is_shutdown = lambda: False
        transport.get_time = lambda: 100.
        def subscribe(topic, cls, callback, callback_args, **kwargs):
            self.assertEqual(topic, '/swarm/uav_status')
            callback(SimpleNamespace(uav_id='typhoon_h480_0', connected=True, z=2.5,
                header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 99.9))), callback_args)
            return SimpleNamespace(unregister=lambda: None)
        transport.Subscriber = subscribe
        with patch.dict(sys.modules, team_time_ros=transport), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(probe.main(['--kind', 'airborne', '--uavs', 'typhoon_h480_0']), 0)
        self.assertEqual(calls[0]['clock_sources'], ('typhoon_h480_0',))
        self.assertLessEqual(calls[0]['clock_timeout'], 30.)

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

    def test_airborne_requires_each_aircraft_currently_above_ground(self):
        args = parse_arguments(['--kind', 'airborne', '--uavs', 'u0', 'u1'])
        tracker = ReadyTracker(expected_topics(args.kinds, args.uavs), ros_now=lambda:100.)
        def status(uid, z=2.5, connected=True, sample=100.):
            return SimpleNamespace(uav_id=uid, z=z, connected=connected,
                header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:sample)))
        for z,connected,sample in ((.1,True,100.),(2.5,False,100.),
                                   (2.5,True,98.),(2.5,True,101.)):
            tracker.on_message(status('u0', z, connected, sample), ('airborne','u0'))
            self.assertEqual(len(tracker.ready), 0)
        with patch('wait_match_ready.time.monotonic', return_value=10.):
            tracker.on_message(status('u0'), ('airborne','u0'))
            tracker.on_message(status('u0'), ('airborne','u1'))
            self.assertEqual(len(tracker.missing()), 1)
            tracker.on_message(status('u1'), ('airborne','u1'))
            self.assertEqual(tracker.missing(), [])
            tracker.on_message(status('u1',connected=False), ('airborne','u1'))
            self.assertEqual(len(tracker.missing()), 1)
        with patch('wait_match_ready.time.monotonic', return_value=11.1):
            self.assertEqual(len(tracker.missing()), 2)

    def perception_tracker(self, names=('u0',)):
        return ReadyTracker(expected_topics(('perception',), names), ros_now=lambda:100.)

    def frame(self, uid='u0', sample=99.9, **changes):
        data = dict(source_uav=uid, processed=True, sample_s=sample,
                    x=2., y=3., z=4., yaw=0., down_pitch=0.)
        data.update(changes)
        return SimpleNamespace(data=json.dumps(data))

    def test_perception_requires_two_new_aligned_airborne_frames(self):
        args = parse_arguments(['--kind', 'perception', '--uavs', 'u0'])
        self.assertEqual(args.kinds, ('perception',))
        self.assertEqual(expected_topics(args.kinds,args.uavs)[('perception','u0')],
                         '/swarm/processed_camera_frame')
        tracker = self.perception_tracker()
        key = ('perception','u0')
        tracker.on_message(self.frame(sample=99.8), key)
        self.assertEqual(tracker.ready,set())
        tracker.on_message(self.frame(sample=99.9), key)
        self.assertEqual(tracker.ready,{key})

    def test_frozen_ground_camera_is_not_ready_despite_airborne_status(self):
        tracker = self.perception_tracker()
        key = ('perception','u0')
        for sample in (99.8,99.9,100.):
            tracker.on_message(self.frame(sample=sample,z=.13), key)
        self.assertEqual(tracker.ready,set())
        self.assertEqual(len(tracker.missing()),1)

    def test_other_aircraft_frames_cannot_fill_missing_camera(self):
        tracker = self.perception_tracker(('u0','u1'))
        for sample in (99.8,99.9):
            for uid in ('u0','u1'):
                tracker.on_message(self.frame(sample=sample), ('perception',uid))
        self.assertEqual(tracker.ready,{('perception','u0')})
        self.assertEqual(tracker.missing(),[(('perception','u1'),'/swarm/processed_camera_frame')])

    def test_duplicate_and_reordered_frames_do_not_renew_validity(self):
        tracker = self.perception_tracker()
        key = ('perception','u0')
        with patch('wait_match_ready.time.monotonic', return_value=10.):
            for sample in (99.8,99.9):tracker.on_message(self.frame(sample=sample), key)
        with patch('wait_match_ready.time.monotonic', return_value=10.9):
            for sample in (99.9,99.8):tracker.on_message(self.frame(sample=sample), key)
        with patch('wait_match_ready.time.monotonic', return_value=11.1):
            self.assertEqual(len(tracker.missing()),1)

    def test_bad_geometry_stale_future_or_unprocessed_frames_cannot_pass(self):
        key = ('perception','u0')
        cases = ({'x':float('nan')},{'z':float('inf')},{'sample':99.4},
                 {'sample':100.1},{'sample_s':True},{'z':True},{'processed':False})
        for changes in cases:
            with self.subTest(changes=changes):
                tracker=self.perception_tracker()
                tracker.on_message(self.frame(sample=99.8),key)
                tracker.on_message(self.frame(**changes),key)
                self.assertEqual(tracker.ready,set())
        for payload in ('{','[]','null'):
            tracker.on_message(SimpleNamespace(data=payload),key)
            self.assertEqual(tracker.ready,set())

    def test_invalid_frame_clears_old_readiness_and_requires_new_sequence(self):
        tracker=self.perception_tracker();key=('perception','u0')
        for sample in (99.7,99.8):tracker.on_message(self.frame(sample=sample),key)
        tracker.on_message(self.frame(sample=99.9,z=.13),key)
        self.assertEqual(tracker.ready,set())
        tracker.on_message(self.frame(sample=99.9),key)
        self.assertEqual(tracker.ready,set())
        tracker.on_message(self.frame(sample=100.),key)
        self.assertEqual(tracker.ready,{key})


if __name__ == "__main__":
    unittest.main()
