#!/usr/bin/env python3
"""Wait for real MAVROS, camera, or laser messages from a UAV fleet.

This is a startup probe only. It does not subscribe to actor, judge, or Gazebo
ground-truth topics. The wall-clock timeout includes ROS initialization.
"""

import argparse
import json
import math
import re
import sys
import threading
import time
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1]/'coordination/src/robocup_swarm/scripts'
sys.path.insert(0, str(SCRIPTS))


DEFAULT_UAVS = tuple("typhoon_h480_%d" % index for index in range(6))
TOPIC_SUFFIX = {
    "state": "/mavros/state",
    "camera": "/stereo_camera/left/image_raw",
    "scan": "/scan",
}
AIRBORNE_KIND = 'airborne'
PERCEPTION_KIND = 'perception'
NAME_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def positive_seconds(value):
    try:
        seconds = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("timeout must be a number") from exc
    if not math.isfinite(seconds) or seconds <= 0:
        raise argparse.ArgumentTypeError("timeout must be finite and positive")
    return seconds


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--kind", action="append", choices=("state", "camera", "scan", "airborne", "perception", "all"),
        help="readiness kind; repeat to check several kinds (default: all)")
    parser.add_argument(
        "--uavs", nargs="+", default=DEFAULT_UAVS, metavar="UAV",
        help="UAV namespaces, separated by spaces or commas (default: six typhoon_h480 UAVs)")
    parser.add_argument(
        "--timeout", type=positive_seconds, default=30.0, metavar="SECONDS",
        help="overall wall-clock deadline, including ROS initialization (default: 30)")
    args = parser.parse_args(argv)
    args.kinds = tuple(TOPIC_SUFFIX) if not args.kind or "all" in args.kind else tuple(
        kind for kind in (*TOPIC_SUFFIX, AIRBORNE_KIND, PERCEPTION_KIND) if kind in args.kind)
    names = [part for value in args.uavs for part in value.split(",")]
    if not names or any(not NAME_PATTERN.fullmatch(name) for name in names):
        parser.error("--uavs must contain nonempty ROS namespace names")
    if len(set(names)) != len(names):
        parser.error("--uavs contains a duplicate namespace")
    args.uavs = tuple(names)
    return args


def expected_topics(kinds, uavs):
    """Return {(kind, UAV): absolute ROS topic} without requiring ROS imports."""
    shared = {AIRBORNE_KIND: '/swarm/uav_status',
              PERCEPTION_KIND: '/swarm/processed_camera_frame'}
    return {(kind, uav): (shared[kind] if kind in shared else
                         "/%s%s" % (uav, TOPIC_SUFFIX[kind]))
            for kind in kinds for uav in uavs}


def is_real_message(kind, message):
    """Require a connected FCU or a nonempty sensor frame."""
    if kind == "state":
        return bool(getattr(message, "connected", False))
    if kind == AIRBORNE_KIND:
        # Agent's connected flag requires actual climb completion, OFFBOARD,
        # armed, and no localization/landing/crash fault. A heartbeat alone
        # is insufficient. Explicit altitude rejects grounded status too.
        return (bool(getattr(message, 'connected', False)) and
                math.isfinite(getattr(message, 'z', float('nan'))) and
                getattr(message, 'z', 0.) >= 2.)
    if kind == "camera":
        return (getattr(message, "width", 0) > 0
                and getattr(message, "height", 0) > 0
                and len(getattr(message, "data", ())) > 0)
    if kind == "scan":
        return len(getattr(message, "ranges", ())) > 0
    raise ValueError("unknown readiness kind: %s" % kind)


class ReadyTracker:
    def __init__(self, topics, ros_now=None):
        self.topics = topics
        self.ready = set()
        self.received = {}
        self.perception_samples = {}
        self.ros_now = ros_now
        self.condition = threading.Condition()

    def on_message(self, message, key):
        with self.condition:
            if key[0] == PERCEPTION_KIND:
                self._on_processed_frame(message, key)
                self.condition.notify_all()
                return
            valid = is_real_message(key[0], message)
            if key[0] == AIRBORNE_KIND:
                if getattr(message, 'uav_id', None) != key[1]:
                    return
                sample = message.header.stamp.to_sec()
                valid = (valid and self.ros_now is not None and
                         0. <= self.ros_now()-sample <= 1.)
                if not valid:
                    self.ready.discard(key)
            if valid:
                self.ready.add(key)
                self.received[key] = time.monotonic()
            self.condition.notify_all()

    def _on_processed_frame(self, message, key):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict) or data.get('source_uav') != key[1]:
                return
            sample = data.get('sample_s')
            numbers = [data.get(name) for name in ('sample_s', 'x', 'y', 'z', 'yaw', 'down_pitch')]
            valid = (data.get('processed') is True and
                     all(not isinstance(v, bool) and isinstance(v, (int, float))
                         and math.isfinite(v) for v in numbers) and
                     self.ros_now is not None and 0. <= self.ros_now()-sample <= .5 and
                     data['z'] >= 2.)
        except (ValueError, TypeError, AttributeError):
            return
        if not valid:
            self.ready.discard(key)
            self.perception_samples.pop(key, None)
            return
        first, last = self.perception_samples.get(key, (sample, -math.inf))
        if sample <= last:
            return  # repeated images cannot renew readiness
        if sample-last > .7:
            first = sample
            self.ready.discard(key)
        self.perception_samples[key] = (first, sample)
        self.received[key] = time.monotonic()
        if sample-first >= .1-1e-6:
            self.ready.add(key)

    def _expire_airborne(self):
        now = time.monotonic()
        for key in tuple(self.ready):
            if key[0] in (AIRBORNE_KIND, PERCEPTION_KIND) and now-self.received[key] > 1.:
                self.ready.discard(key)

    def wait(self, deadline, is_shutdown):
        with self.condition:
            while True:
                self._expire_airborne()
                if len(self.ready) == len(self.topics):
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0 or is_shutdown():
                    break
                self.condition.wait(min(remaining, 0.25))
            return len(self.ready) == len(self.topics)

    def missing(self):
        with self.condition:
            self._expire_airborne()
            return [(key, topic) for key, topic in self.topics.items()
                    if key not in self.ready]


def main(argv=None):
    args = parse_arguments(argv)
    deadline = time.monotonic() + args.timeout
    topics = expected_topics(args.kinds, args.uavs)
    needs_header_time = AIRBORNE_KIND in args.kinds or PERCEPTION_KIND in args.kinds
    try:
        if needs_header_time:
            import team_time_ros as rospy
        else:
            import rospy
        from mavros_msgs.msg import State
        from sensor_msgs.msg import Image, LaserScan
        if AIRBORNE_KIND in args.kinds:
            from robocup_swarm.msg import UavStatus
        if PERCEPTION_KIND in args.kinds:
            from std_msgs.msg import String
    except ImportError as exc:
        print("[ready] ROS Python packages unavailable: %s; source ROS and catkin setup.bash" % exc,
              file=sys.stderr)
        return 2

    subscribers = []
    try:
        if needs_header_time:
            rospy.init_node("wait_match_ready", anonymous=True, disable_signals=False,
                            clock_sources=args.uavs,
                            clock_timeout=max(.001, deadline-time.monotonic()),
                            disable_rosout=True)
        else:
            # FCU connection and first sensor frames must not wait for EKF pose.
            # This transport-only probe never reads native ROS time.
            rospy.init_node("wait_match_ready", anonymous=True, disable_signals=False,
                            disable_rostime=True, disable_rosout=True)
        tracker = ReadyTracker(topics, ros_now=rospy.get_time if needs_header_time else None)
        classes = {"state": State, "camera": Image, "scan": LaserScan}
        if AIRBORNE_KIND in args.kinds:
            classes[AIRBORNE_KIND] = UavStatus
        if PERCEPTION_KIND in args.kinds:
            classes[PERCEPTION_KIND] = String
        for key, topic in topics.items():
            # Camera images can exceed rospy's small default TCP buffer. Never log
            # image payloads; the callback only checks that a nonempty frame arrived.
            buffer_bytes = 16 * 1024 * 1024 if key[0] == "camera" else 65536
            subscribers.append(rospy.Subscriber(
                topic, classes[key[0]], tracker.on_message,
                callback_args=key, queue_size=1, buff_size=buffer_bytes))
        complete = tracker.wait(deadline, rospy.is_shutdown)
    except (rospy.ROSException, OSError) as exc:
        print("[ready] ROS setup failed: %s" % exc, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("[ready] interrupted", file=sys.stderr)
        return 130
    finally:
        for subscriber in subscribers:
            subscriber.unregister()

    if complete:
        print("[ready] PASS %d/%d message gates: %s for %s" % (
            len(topics), len(topics), ",".join(args.kinds), ",".join(args.uavs)))
        return 0
    missing = tracker.missing()
    print("[ready] FAIL %d/%d message gates before %.1fs wall-clock deadline" % (
        len(topics) - len(missing), len(topics), args.timeout), file=sys.stderr)
    for (kind, uav), topic in missing:
        print("[ready] MISSING %-6s %-18s %s" % (kind, uav, topic), file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
