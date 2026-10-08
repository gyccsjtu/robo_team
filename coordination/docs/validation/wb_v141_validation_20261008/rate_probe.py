#!/usr/bin/env python3
"""Measure camera and pipeline rates on the SIMULATION clock, plus image age.

`rostopic hz` measures window time on the wall clock; this simulation runs at
RTF ~0.16, so wall-clock rates are ~6x lower than sim-clock rates and the two
cannot be compared directly. This counts messages against rospy.Time (sim time)
so the numbers are meaningful, and also reports the age of each image at the
moment the subscriber receives it.

Read-only: subscribes and prints. Publishes nothing.

Run: python3 rate_probe.py --seconds 20
"""
import argparse
import collections
import json
import sys
import time

UAVS = ["uav_1", "uav_2", "uav_3", "uav_4", "uav_5", "uav_6"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()
    import rospy
    from sensor_msgs.msg import Image

    rospy.init_node("wb_rate_probe", anonymous=True)
    count = collections.Counter()
    first_stamp = {}
    last_stamp = {}
    ages = collections.defaultdict(list)
    wall_first = time.monotonic()

    def make_cb(key):
        def cb(msg):
            count[key] += 1
            t = msg.header.stamp.to_sec()
            if t > 0:
                if key not in first_stamp:
                    first_stamp[key] = t
                last_stamp[key] = t
                ages[key].append(rospy.Time.now().to_sec() - t)
        return cb

    subs = []
    for u in UAVS:
        subs.append(rospy.Subscriber("/%s/cgo3_camera/image_raw" % u, Image,
                                     make_cb(u), queue_size=1))
    subs.append(rospy.Subscriber("/swarm/processed_camera_frame", Image,
                                 make_cb("processed"), queue_size=1))

    t0 = rospy.Time.now().to_sec()
    time.sleep(args.seconds)
    t1 = rospy.Time.now().to_sec()
    wall = time.monotonic() - wall_first
    sim_span = t1 - t0
    print("window: sim %.2f s  wall %.2f s  RTF %.3f" % (sim_span, wall,
                                                         sim_span / max(wall, 1e-9)))
    print()
    print("%-12s %8s %10s %10s %10s" % ("topic", "count", "per_sim_s", "per_wall_s", "age_p50"))
    total_sim = 0.0
    for key in UAVS + ["processed"]:
        c = count[key]
        per_sim = c / sim_span if sim_span > 0 else 0.0
        per_wall = c / wall if wall > 0 else 0.0
        a = sorted(ages[key])
        total_sim += per_sim
        print("%-12s %8d %10.2f %10.2f %10s"
              % (key, c, per_sim, per_wall,
                 ("%.3f" % a[len(a) // 2]) if a else "-"))
    print()
    print("camera aggregate: %.1f frames per SIM second (6 cameras)" % total_sim)
    print("processed topic : %.1f frames per SIM second"
          % (count["processed"] / sim_span if sim_span > 0 else 0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
