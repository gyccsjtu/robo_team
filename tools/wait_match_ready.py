#!/usr/bin/env python3
"""Wait for real MAVROS, camera, or laser messages from a UAV fleet.

This is a startup probe only. It does not subscribe to actor, judge, or Gazebo
ground-truth topics. The wall-clock timeout includes ROS initialization.
"""

import argparse
import math
import re
import sys
import threading
import time


DEFAULT_UAVS = tuple("typhoon_h480_%d" % index for index in range(6))
TOPIC_SUFFIX = {
    "state": "/mavros/state",
    "camera": "/stereo_camera/left/image_raw",
    "scan": "/scan",
}
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
        "--kind", action="append", choices=("state", "camera", "scan", "all"),
        help="readiness kind; repeat to check several kinds (default: all)")
    parser.add_argument(
        "--uavs", nargs="+", default=DEFAULT_UAVS, metavar="UAV",
        help="UAV namespaces, separated by spaces or commas (default: six typhoon_h480 UAVs)")
    parser.add_argument(
        "--timeout", type=positive_seconds, default=30.0, metavar="SECONDS",
        help="overall wall-clock deadline, including ROS initialization (default: 30)")
    args = parser.parse_args(argv)
    args.kinds = tuple(TOPIC_SUFFIX) if not args.kind or "all" in args.kind else tuple(
        kind for kind in TOPIC_SUFFIX if kind in args.kind)
    names = [part for value in args.uavs for part in value.split(",")]
    if not names or any(not NAME_PATTERN.fullmatch(name) for name in names):
        parser.error("--uavs must contain nonempty ROS namespace names")
    if len(set(names)) != len(names):
        parser.error("--uavs contains a duplicate namespace")
    args.uavs = tuple(names)
    return args


def expected_topics(kinds, uavs):
    """Return {(kind, UAV): absolute ROS topic} without requiring ROS imports."""
    return {(kind, uav): "/%s%s" % (uav, TOPIC_SUFFIX[kind])
            for kind in kinds for uav in uavs}


def is_real_message(kind, message):
    """Require a connected FCU or a nonempty sensor frame."""
    if kind == "state":
        return bool(getattr(message, "connected", False))
    if kind == "camera":
        return (getattr(message, "width", 0) > 0
                and getattr(message, "height", 0) > 0
                and len(getattr(message, "data", ())) > 0)
    if kind == "scan":
        return len(getattr(message, "ranges", ())) > 0
    raise ValueError("unknown readiness kind: %s" % kind)


class ReadyTracker:
    def __init__(self, topics):
        self.topics = topics
        self.ready = set()
        self.condition = threading.Condition()

    def on_message(self, message, key):
        if not is_real_message(key[0], message):
            return
        with self.condition:
            self.ready.add(key)
            self.condition.notify_all()

    def wait(self, deadline, is_shutdown):
        with self.condition:
            while len(self.ready) != len(self.topics):
                remaining = deadline - time.monotonic()
                if remaining <= 0 or is_shutdown():
                    break
                self.condition.wait(min(remaining, 0.25))
            return len(self.ready) == len(self.topics)

    def missing(self):
        with self.condition:
            return [(key, topic) for key, topic in self.topics.items()
                    if key not in self.ready]


def main(argv=None):
    args = parse_arguments(argv)
    deadline = time.monotonic() + args.timeout
    topics = expected_topics(args.kinds, args.uavs)
    try:
        import rospy
        from mavros_msgs.msg import State
        from sensor_msgs.msg import Image, LaserScan
    except ImportError as exc:
        print("[ready] ROS Python packages unavailable: %s; source ROS and catkin setup.bash" % exc,
              file=sys.stderr)
        return 2

    tracker = ReadyTracker(topics)
    subscribers = []
    try:
        rospy.init_node("wait_match_ready", anonymous=True, disable_signals=False,
                        disable_rostime=True, disable_rosout=True)
        classes = {"state": State, "camera": Image, "scan": LaserScan}
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
