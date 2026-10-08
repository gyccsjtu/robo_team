#!/usr/bin/env python3
"""Development-only read-only evidence observer (white actor visibility).

Purpose: capture original camera images at instants when the DEVELOPMENT truth
says the actor is geometrically inside the field of view, so that later offline
review can ask "was the actor actually rendered / visible / large enough".

Hard isolation rules (see WORKBUDDY_BATCH_TASK_20261008.md):
- This process PUBLISHES NOTHING. No topics, no services. It never calls any
  MAVROS/FCU service and never touches setpoints, routes, tasks or reporting.
- Truth is used ONLY to decide whether to save an image. Truth values are never
  written into perception, bridges, the manager or any control path, and the
  perception node (perception_real.py) is not modified or involved.
- Same-instant pairing: the camera pose and the truth sample must belong to the
  same original-image time. Stale poses are rejected; a fresh pose is never
  paired with an old image.
- Bounded: per-aircraft at most one frame per second, a global cap per run, and
  a bounded in-memory queue. Disk/encode failures are recorded, not queued.
- Clean exit on result file, rospy shutdown, SIGINT/SIGTERM.

Default DISABLED (--enable required). Not run as part of this task.

Example (documentation only, do not run against a live simulation without
Codex scheduling it):

  python3 white_evidence_observer.py --enable \
      --out-dir /root/robocup_runs/<round>/observers/white_dev \
      --result-file /root/robocup_runs/<round>/budget.json \
      --truth-model actor_3 --min-expected-px 4 --max-range-m 30 \
      --total-cap 120
"""
import argparse
import hashlib
import json
import math
import os
import signal
import sys
import time
from collections import deque

import numpy as np

M_OPT2LINK = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
PERSON_M = 1.75
GROUND_Z = 0.0


def sha256_file(path):
    h = hashlib.sha256()
    try:
        with open(path, "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                h.update(chunk)
    except OSError:
        return ""
    return h.hexdigest()


class BoundedCapture(object):
    """Bounded capture state machine, testable without ROS."""

    def __init__(self, out_dir, per_uav_per_sec=1.0, total_cap=120,
                 min_expected_px=4.0, max_range_m=30.0, pose_max_age_s=0.5,
                 pair_tol_s=0.02, queue_max=64, enabled=False):
        self.out_dir = out_dir
        self.per_uav_per_sec = float(per_uav_per_sec)
        self.total_cap = int(total_cap)
        self.min_expected_px = float(min_expected_px)
        self.max_range_m = float(max_range_m)
        self.pose_max_age_s = float(pose_max_age_s)
        self.pair_tol_s = float(pair_tol_s)
        self.enabled = bool(enabled)
        self.queue = deque(maxlen=int(queue_max))
        self.saved = 0
        self.skipped = {}
        self.last_saved = {}            # uav -> wall time
        self.seen_keys = set()          # (uav, rounded image time) dedupe
        self.index_path = os.path.join(out_dir, "index.jsonl") if out_dir else ""

    def _skip(self, reason):
        """Record a rejection and return the reason (callers return it too)."""
        self.skipped[reason] = self.skipped.get(reason, 0) + 1
        return reason

    def should_capture(self, uav, image_time, pose, pose_time, truth_xy,
                       intr, size, wall_now, image_sha=""):
        """Decide and record. Returns (capture_bool, reason, payload)."""
        if not self.enabled:
            return False, "disabled", None
        if self.saved >= self.total_cap:
            return False, self._skip("total_cap_reached"), None
        key = (uav, round(float(image_time), 3))
        if key in self.seen_keys:
            return False, self._skip("duplicate_image_time"), None
        if pose is None or intr is None or size is None:
            return False, self._skip("missing_camera_fields"), None
        if pose_time is None or abs(float(pose_time) - float(image_time)) > self.pair_tol_s:
            # stale or mismatched pose: never pair a fresh pose with an old image
            return False, self._skip("pose_image_time_mismatch"), None
        if wall_now - float(pose_time) > self.pose_max_age_s:
            return False, self._skip("pose_too_old"), None
        if truth_xy is None:
            return False, self._skip("no_truth"), None
        try:
            fx, fy, cx, cy = [float(v) for v in intr]
            W, H = int(size[0]), int(size[1])
            R = np.asarray(pose["rotation"], float).reshape(3, 3)
            cam = np.asarray(pose["xyz"], float)
        except Exception:
            return False, self._skip("invalid_transform"), None
        d = M_OPT2LINK.T @ R.T @ (np.array([truth_xy[0], truth_xy[1], GROUND_Z],
                                           float) - cam)
        depth = float(d[2])
        if not math.isfinite(depth) or depth <= 0:
            return False, self._skip("behind_camera"), None
        u = fx * float(d[0]) / depth + cx
        v = fy * float(d[1]) / depth + cy
        if not (0 <= u < W and 0 <= v < H):
            return False, self._skip("outside_image"), None
        horiz = math.hypot(truth_xy[0] - cam[0], truth_xy[1] - cam[1])
        exp_h = PERSON_M * fx / depth
        if exp_h < self.min_expected_px and horiz > self.max_range_m:
            return False, self._skip("below_size_and_range_thresholds"), None
        last = self.last_saved.get(uav)
        if last is not None and (wall_now - last) < 1.0 / self.per_uav_per_sec:
            return False, self._skip("per_uav_rate_limit"), None
        payload = dict(uav=uav, image_time=float(image_time), pose_time=float(pose_time),
                       truth=dict(x=float(truth_xy[0]), y=float(truth_xy[1]),
                                  z=GROUND_Z, assumption="ground z=0, 1.75 m ref"),
                       camera_xyz=[float(x) for x in cam], rotation=[float(x) for x in R.flatten()],
                       intrinsics=[fx, fy, cx, cy], size=[W, H],
                       depth=round(depth, 3), uv=[round(u, 1), round(v, 1)],
                       expected_h_px=round(exp_h, 2), horiz_m=round(horiz, 2),
                       image_sha256=image_sha, truth_triggered=True,
                       non_control_use=True)
        return True, "capture", payload

    def commit(self, image, payload, wall_now=None):
        """Write PNG + index line. Failures are recorded, never raised."""
        try:
            import cv2
            os.makedirs(self.out_dir, exist_ok=True)
            name = "white_dev_%s_%d.png" % (payload["uav"], int(payload["image_time"] * 1000))
            path = os.path.join(self.out_dir, name)
            ok, buf = cv2.imencode(".png", image)
            if not ok:
                return self._skip("encode_failed")
            with open(path, "wb") as fh:
                fh.write(buf.tobytes())
            row = dict(schema_version=1, png=name, png_sha256=sha256_file(path),
                       wall_time=wall_now or time.time(), **payload)
            with open(self.index_path, "a", encoding="utf-8") as fh:
                json.dump(row, fh, ensure_ascii=False, default=str)
                fh.write("\n")
            self.saved += 1
            self.seen_keys.add((payload["uav"], round(payload["image_time"], 3)))
            self.last_saved[payload["uav"]] = wall_now or time.time()
            return True
        except Exception as error:
            self.skipped["write_failed"] = self.skipped.get("write_failed", 0) + 1
            print("[white_dev] write failed: %s" % error, flush=True)
            return False

    def stats(self):
        return dict(saved=self.saved, cap=self.total_cap, skipped=dict(self.skipped),
                    unique_keys=len(self.seen_keys))


def result_finished(path):
    """True only when the result file exists AND reports a terminal attempt."""
    if not path:
        return False
    try:
        with open(path, encoding="utf-8") as fh:
            d = json.load(fh)
    except Exception:
        return False          # not created yet, or unreadable/old: keep waiting
    st = None
    if isinstance(d, dict):
        if isinstance(d.get("attempts"), list) and d["attempts"]:
            st = d["attempts"][-1].get("status")
        elif isinstance(d.get("attempts"), dict):
            st = d["attempts"].get("status")
        else:
            st = d.get("status")
    return st in ("ENDED", "FAILED", "STOPPED_AFTER_CONTACT")


def build_arg_parser():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--enable", action="store_true",
                    help="required; without it the observer does nothing")
    ap.add_argument("--out-dir", default="")
    ap.add_argument("--result-file", default="", help="terminate when this reports ENDED/FAILED")
    ap.add_argument("--truth-model", default="actor_3",
                    help="Gazebo model name of the dev-truth actor (white)")
    ap.add_argument("--uavs", default="uav_1,uav_2,uav_3,uav_4,uav_5,uav_6")
    ap.add_argument("--per-uav-per-sec", type=float, default=1.0)
    ap.add_argument("--total-cap", type=int, default=120)
    ap.add_argument("--min-expected-px", type=float, default=4.0,
                    help="sampling heuristic only; NOT a competition criterion")
    ap.add_argument("--max-range-m", type=float, default=30.0,
                    help="sampling heuristic only; NOT a competition criterion")
    ap.add_argument("--pair-tol-s", type=float, default=0.02)
    ap.add_argument("--pose-max-age-s", type=float, default=0.5)
    ap.add_argument("--queue-max", type=int, default=64)
    ap.add_argument("--poll-s", type=float, default=0.2)
    ap.add_argument("--self-check", action="store_true",
                    help="print the resolved configuration and exit (no ROS)")
    return ap


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    script_sha = sha256_file(os.path.abspath(__file__))
    cfg = dict(vars(args))
    cfg["script_sha256"] = script_sha
    if args.self_check or not args.enable:
        print(json.dumps(dict(config=cfg, enabled=bool(args.enable),
                              note="observer publishes nothing; truth only gates image saving"),
                         ensure_ascii=False, indent=1))
        if not args.enable:
            print("not enabled: pass --enable to run")
        return 0

    if not args.out_dir:
        print("--out-dir is required when enabled")
        return 2

    cap = BoundedCapture(args.out_dir, per_uav_per_sec=args.per_uav_per_sec,
                         total_cap=args.total_cap,
                         min_expected_px=args.min_expected_px,
                         max_range_m=args.max_range_m,
                         pose_max_age_s=args.pose_max_age_s,
                         pair_tol_s=args.pair_tol_s, queue_max=args.queue_max,
                         enabled=True)

    import rospy
    from cv_bridge import CvBridge
    from gazebo_msgs.msg import ModelStates
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image

    stopping = {"flag": False}

    def _stop(signum, frame):
        stopping["flag"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    rospy.init_node("white_dev_observer", anonymous=False)
    state = {"truth": None, "pose": {}, "image": {}, "intr": {}}

    def models_cb(msg):
        try:
            i = msg.name.index(args.truth_model)
        except ValueError:
            return
        p = msg.pose[i].position
        state["truth"] = (float(p.x), float(p.y), float(p.z))

    def make_pose_cb(uav):
        def cb(msg):
            q = msg.pose.pose.orientation
            x, y, z, w = q.x, q.y, q.z, q.w
            n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
            x, y, z, w = x / n, y / n, z / n, w / n
            R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            p = msg.pose.pose.position
            state["pose"][uav] = dict(xyz=[float(p.x), float(p.y), float(p.z)],
                                      rotation=R.flatten().tolist(),
                                      stamp=msg.header.stamp.to_sec())
        return cb

    def make_image_cb(uav):
        bridge = CvBridge()

        def cb(msg):
            try:
                img = bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception:
                cap._skip("cv_bridge_failed")
                return
            state["image"][uav] = (msg, img)
        return cb

    def make_info_cb(uav):
        def cb(msg):
            state["intr"][uav] = ([msg.K[0], msg.K[4], msg.K[2], msg.K[5]],
                                  [msg.width, msg.height])
        return cb

    subs = [rospy.Subscriber("/gazebo/model_states", ModelStates, models_cb, queue_size=1)]
    for uav in [u for u in args.uavs.split(",") if u]:
        subs.append(rospy.Subscriber("/%s/cgo3_camera/image_raw" % uav, Image,
                                     make_image_cb(uav), queue_size=1))
        subs.append(rospy.Subscriber("/%s/cgo3_camera/camera_info" % uav, CameraInfo,
                                     make_info_cb(uav), queue_size=1))
        subs.append(rospy.Subscriber("/%s/mavros/local_position/pose" % uav, Odometry,
                                     make_pose_cb(uav), queue_size=1))

    print("[white_dev] enabled; out=%s cap=%d px>=%.1f range<=%.1f m"
          % (args.out_dir, args.total_cap, args.min_expected_px, args.max_range_m), flush=True)
    rate = rospy.Rate(1.0 / args.poll_s)
    while not rospy.is_shutdown() and not stopping["flag"]:
        if result_finished(args.result_file):
            print("[white_dev] result file reports a terminal state; exiting", flush=True)
            break
        for uav, (msg, img) in list(state["image"].items()):
            pose = state["pose"].get(uav)
            if pose is None or uav not in state["intr"]:
                continue
            intr, size = state["intr"][uav]
            img_t = msg.header.stamp.to_sec()
            ok, reason, payload = cap.should_capture(
                uav, img_t, dict(xyz=pose["xyz"], rotation=pose["rotation"]),
                pose["stamp"], state["truth"], intr, size, time.time())
            if ok:
                cap.commit(img, payload, time.time())
        rate.sleep()

    print("[white_dev] exit; %s" % json.dumps(cap.stats(), ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())