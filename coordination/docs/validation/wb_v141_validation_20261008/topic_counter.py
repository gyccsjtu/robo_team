#!/usr/bin/env python3
"""Read-only topic counter (development diagnostic).

Purpose: resolve the question my offline analysis could not — does the bridge's
10 Hz timer actually run? It subscribes only, PUBLISHES NOTHING, never calls a
service, and never touches routes/tasks/setpoints/reporting.

What it separates:
  * /swarm/target_states flowing   -> yolo_target_bridge._emit() IS being called
  * /actor_<tag>_info silent       -> the publish branch inside _emit is skipped
  * both silent                    -> the timer / core.tick() is not running

Output: one JSON line per second per topic with a per-second count, appended to
--out. Exit on result file terminal state, SIGINT/SIGTERM, or wall limit.

Example:
  python3 topic_counter.py --out <round>/observers/topic_counter.jsonl \
      --result-file <root>/budget.json --wall-seconds 5400
"""
import argparse
import collections
import json
import os
import signal
import sys
import time

TOPICS = ["/swarm/target_states", "/swarm/detection", "/swarm/visual_observation",
          "/left_actors", "/swarm/authorized_assignment"]
ACTOR_TOPICS = ["/actor_green_info", "/actor_blue_info", "/actor_brown_info",
                "/actor_white_info", "/actor_red1_info", "/actor_red2_info"]


def result_finished(path):
    """True only when the result file exists AND reports a terminal attempt."""
    if not path:
        return False
    try:
        with open(path, encoding="utf-8") as fh:
            d = json.load(fh)
    except Exception:
        return False
    st = None
    if isinstance(d, dict):
        if isinstance(d.get("attempts"), list) and d["attempts"]:
            st = d["attempts"][-1].get("status")
        elif isinstance(d.get("attempts"), dict):
            st = d["attempts"].get("status")
        else:
            st = d.get("status")
    return st in ("ENDED", "FAILED", "STOPPED_AFTER_CONTACT")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--result-file", default="")
    ap.add_argument("--wall-seconds", type=float, default=5400.0)
    args = ap.parse_args()

    import rospy
    from std_msgs.msg import String
    from robocup_swarm.msg import TargetState

    stopping = {"flag": False}

    def _stop(signum, frame):
        stopping["flag"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    rospy.init_node("wb_topic_counter", anonymous=True)
    counts = collections.Counter()
    last_stamp = {}

    def make_cb(topic):
        def cb(msg):
            counts[topic] += 1
            if topic == "/swarm/target_states":
                last_stamp[topic] = (getattr(msg, "target_id", None),
                                     round(float(msg.x), 2), round(float(msg.y), 2),
                                     int(getattr(msg, "state", -1)),
                                     round(msg.header.stamp.to_sec(), 3))
            elif topic in ACTOR_TOPICS:
                last_stamp[topic] = (round(float(msg.x), 2), round(float(msg.y), 2))
        return cb

    subs = []
    from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
    for t in TOPICS:
        subs.append(rospy.Subscriber(t, TargetState if t == "/swarm/target_states" else String,
                                     make_cb(t), queue_size=200))
    for t in ACTOR_TOPICS:
        subs.append(rospy.Subscriber(t, ActorInfo, make_cb(t), queue_size=200))

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fh = open(args.out, "a", encoding="utf-8", buffering=1)
    fh.write(json.dumps(dict(kind="start", wall=time.time(),
                             ros=rospy.Time.now().to_sec(),
                             topics=TOPICS + ACTOR_TOPICS)) + "\n")

    deadline = time.monotonic() + args.wall_seconds
    while not rospy.is_shutdown() and not stopping["flag"]:
        if result_finished(args.result_file):
            break
        if time.monotonic() > deadline:
            break
        row = dict(kind="second", wall=round(time.time(), 3),
                   ros=rospy.Time.now().to_sec(),
                   counts=dict((k, v) for k, v in counts.items() if v))
        row["last"] = dict((k, v) for k, v in last_stamp.items())
        fh.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
        counts.clear()
        # Wall-clock sleep: keep sampling (and keep exiting) even if /clock stops.
        time.sleep(1.0)

    fh.write(json.dumps(dict(kind="stop", wall=time.time(),
                             ros=rospy.Time.now().to_sec())) + "\n")
    fh.close()
    for s in subs:
        s.unregister()
    return 0


if __name__ == "__main__":
    sys.exit(main())
