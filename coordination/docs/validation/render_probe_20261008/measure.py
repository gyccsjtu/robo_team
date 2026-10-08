#!/usr/bin/env python3
"""Measure camera frame rate and image age from the probe, and save frames.

Sim time is used for the frame rate (that is what the sensor's update_rate is
expressed in); wall time is reported alongside. Image age = sim now - header
stamp, which is the quantity that feeds the whole latency chain.
"""
import argparse
import json
import os
import time

import numpy as np
import rospy
from sensor_msgs.msg import Image

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--seconds", type=float, default=20.0)
args = ap.parse_args()

rospy.init_node("probe_measure", anonymous=True, disable_signals=True)

recs = []          # (wall_recv, sim_stamp, w, h, data, encoding, sim_now)
last = {"msg": None}


def cb(msg):
    recs.append((time.time(), msg.header.stamp.to_sec(), msg.width, msg.height,
                 msg.data, msg.encoding, rospy.Time.now().to_sec()))
    last["msg"] = msg


sub = rospy.Subscriber("/probe/camera/image_raw", Image, cb, queue_size=3)

t0 = time.time()
r = rospy.Rate(2.0)
while not rospy.is_shutdown() and time.time() - t0 < args.seconds:
    r.sleep()

n = len(recs)
res = {"backend_note": os.environ.get("LIBGL_ALWAYS_SOFTWARE", "<unset>"),
       "frames": n, "wall_seconds": round(time.time() - t0, 2)}
if n >= 2:
    sim = [x[1] for x in recs]
    wall = [x[0] for x in recs]
    ages = [x[6] - x[1] for x in recs if x[6] > 0]
    sim_span = sim[-1] - sim[0]
    wall_span = wall[-1] - wall[0]
    res.update({
        "sim_span": round(sim_span, 3),
        "wall_span": round(wall_span, 3),
        "sim_fps": round((n - 1) / sim_span, 3) if sim_span > 0 else None,
        "wall_fps": round((n - 1) / wall_span, 3) if wall_span > 0 else None,
        "rtf": round(sim_span / wall_span, 4) if wall_span > 0 else None,
        "image_age_p50": round(float(np.percentile(ages, 50)), 3) if ages else None,
        "image_age_max": round(float(max(ages)), 3) if ages else None,
        "size": "%dx%d" % (recs[0][2], recs[0][3]),
        "encoding": recs[0][5],
    })
print(json.dumps(res, indent=1))
with open(os.path.join(args.out, "measure.json"), "w") as fh:
    json.dump(res, fh, indent=1)

# save up to 3 frames, middle of the run, as PNG if possible else PPM
def save(idx, tag):
    wall, stamp, w, h, data, enc, simnow = recs[idx]
    arr = np.frombuffer(data, dtype=np.uint8)
    arr = arr.reshape(h, w, -1) if arr.size == w * h * 3 else arr.reshape(h, w)
    try:
        import cv2
        path = os.path.join(args.out, "frame_%s.png" % tag)
        cv2.imwrite(path, arr[:, :, ::-1] if arr.ndim == 3 else arr)
        return path
    except Exception:
        pass
    try:
        from PIL import Image as PILImage
        path = os.path.join(args.out, "frame_%s.png" % tag)
        PILImage.fromarray(arr).save(path)
        return path
    except Exception:
        path = os.path.join(args.out, "frame_%s.ppm" % tag)
        with open(path, "wb") as f:
            f.write(b"P6\n%d %d\n255\n" % (w, h))
            f.write(data)
        return path


if n:
    for idx, tag in ((max(0, n // 2), "mid"), (n - 1, "last")):
        print("saved", save(idx, tag))
