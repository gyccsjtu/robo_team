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
- Bounded: per-aircraft at most one frame per second, a global cap per run, and
  a bounded in-memory queue. Disk/encode failures are recorded, not queued.
- Clean exit on result file, rospy shutdown, SIGINT/SIGTERM.

Default DISABLED (--enable required).

POSE CONTRACT (corrected 2026-10-08 after an offline backtest showed the first
version would have saved ZERO frames):

  The first version waited for `/uav_N/mavros/local_position/pose` and demanded
  that the newest pose stamp match the image stamp within pair_tol_s = 0.02 s.
  Measured image latency in a real round is p50 0.472 s (min 0.216, max 0.884),
  so that contract rejects 100% of frames. It also guarded on
  `wall_now - pose_stamp <= pose_max_age_s`, which cannot distinguish "no pose
  for this instant" from "the image is old".

  The production perception node does not use a pose topic at all: when an
  image arrives it pulls the camera link pose from the Gazebo service
  /gazebo/get_link_state and pairs it with the image's own header stamp,
  accepting frames with `0 <= now - stamp <= 1`. This observer now mirrors that
  exact pattern (--pose-source service, the default):
    * pose pulled at image arrival, offset by PR_CAM_OFF_BL (0,0,-0.162)
    * the only freshness gate is the ORIGINAL image age (<= 1 s, same as prod)
    * the pose pull delay is recorded per row, because the pose is sampled
      slightly after capture and therefore carries a small position error
  --pose-source topic remains available for environments without the service;
  in that mode the pose is looked up in a bounded history by nearest stamp
  (not "newest pose"), and the pair tolerance is enforced.

Example (documentation only; scheduling is Codex's call):

  python3 white_evidence_observer.py --enable \
      --wiring /root/robocup_runs/<round>/flight/wiring.json \
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
DEFAULT_CAM_OFF_BL = "0,0,-0.162"        # same value city_swarm_run.py sets
IMAGE_MAX_AGE_S = 1.0                    # same gate as perception_real.py


def sha256_file(path):
    h = hashlib.sha256()
    try:
        with open(path, "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                h.update(chunk)
    except OSError:
        return ""
    return h.hexdigest()


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def model_to_uav(model_name, rows=None):
    """uav_id for a gazebo model name. `rows` is wiring['uavs'] when available."""
    for r in rows or []:
        if r.get("model_name") == model_name:
            return r.get("uav_id")
    return None


def load_wiring(path):
    """Return (uavs, {uav_id: model_name}). Missing/invalid file -> empty."""
    if not path or not os.path.isfile(path):
        return [], {}
    try:
        with open(path, encoding="utf-8") as fh:
            d = json.load(fh)
    except Exception:
        return [], {}
    rows = d.get("uavs") or []
    mapping = {}
    for r in rows:
        uid, mod = r.get("uav_id"), r.get("model_name")
        if uid and mod:
            mapping[uid] = mod
    return rows, mapping


class BoundedCapture(object):
    """Bounded capture state machine, testable without ROS."""

    def __init__(self, out_dir, per_uav_per_sec=1.0, total_cap=120,
                 min_expected_px=4.0, max_range_m=30.0, pose_max_age_s=2.0,
                 pair_tol_s=0.02, queue_max=64, enabled=False,
                 image_max_age_s=IMAGE_MAX_AGE_S, pose_history_s=3.0,
                 truth_max_age_s=1.0):
        self.out_dir = out_dir
        self.per_uav_per_sec = float(per_uav_per_sec)
        self.total_cap = int(total_cap)
        self.min_expected_px = float(min_expected_px)
        self.max_range_m = float(max_range_m)
        self.pose_max_age_s = float(pose_max_age_s)
        self.pair_tol_s = float(pair_tol_s)
        self.image_max_age_s = float(image_max_age_s)
        self.pose_history_s = float(pose_history_s)
        self.truth_max_age_s = float(truth_max_age_s)
        self.enabled = bool(enabled)
        self.queue = deque(maxlen=int(queue_max))
        self.saved = 0
        self.skipped = {}
        self.last_saved = {}            # uav -> wall time
        self.seen_keys = set()          # (uav, rounded image time) dedupe of saves
        self.poses = {}                 # uav -> deque[(stamp, payload)] (topic mode)
        self.index_path = os.path.join(out_dir, "index.jsonl") if out_dir else ""

    # -- pose history (topic mode) ------------------------------------------
    def note_pose(self, uav, stamp, xyz, rotation, now=None):
        """Remember a stamped pose so an image can be matched by nearest stamp."""
        if stamp is None:
            return
        stamp = float(stamp)
        dq = self.poses.setdefault(uav, deque(maxlen=512))
        dq.append((stamp, dict(xyz=[float(v) for v in xyz],
                               rotation=[float(v) for v in rotation])))
        horizon = stamp - self.pose_history_s
        while dq and dq[0][0] < horizon:
            dq.popleft()

    def pick_pose(self, uav, image_time):
        """Nearest-stamp pose to the image instant. Returns (payload, stamp)."""
        dq = self.poses.get(uav)
        if not dq:
            return None, None
        best, best_dt = None, None
        for stamp, payload in dq:
            dt = abs(stamp - float(image_time))
            if best_dt is None or dt < best_dt:
                best, best_dt = payload, dt
        return best, (None if best is None else
                      next(s for s, p in dq if p is best))

    # -- decision ------------------------------------------------------------
    def _skip(self, reason):
        """Record a rejection and return the reason (callers return it too)."""
        self.skipped[reason] = self.skipped.get(reason, 0) + 1
        return reason

    def should_capture(self, uav, image_time, pose, pose_time, truth_xy,
                       intr, size, wall_now, image_sha="", truth_age_s=None,
                       pose_pull_delay_s=None):
        """Decide and record. Returns (capture_bool, reason, payload)."""
        if not self.enabled:
            return False, "disabled", None
        if self.saved >= self.total_cap:
            return False, self._skip("total_cap_reached"), None
        if image_time is None or float(image_time) <= 0.0:
            return False, self._skip("no_image_stamp"), None
        key = (uav, round(float(image_time), 3))
        if key in self.seen_keys:
            return False, self._skip("duplicate_image_time"), None
        # The gate that actually matters: never save an ancient original.
        age = float(wall_now) - float(image_time)
        if age < -0.5:
            return False, self._skip("image_time_in_future"), None
        if age > self.image_max_age_s:
            return False, self._skip("image_too_old"), None
        if pose is None:
            pose, pose_time = self.pick_pose(uav, image_time)
        if pose is None or intr is None or size is None:
            return False, self._skip("missing_camera_fields"), None
        if pose_time is not None and abs(float(pose_time) - float(image_time)) > self.pair_tol_s:
            # stale or mismatched pose: never pair a fresh pose with an old image
            return False, self._skip("pose_image_time_mismatch"), None
        if pose_time is not None and (wall_now - float(pose_time)) > self.pose_max_age_s:
            return False, self._skip("pose_too_old"), None
        if truth_age_s is not None and float(truth_age_s) > self.truth_max_age_s:
            return False, self._skip("truth_too_old"), None
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
        payload = dict(uav=uav, image_time=float(image_time), pose_time=(
                           None if pose_time is None else float(pose_time)),
                       image_age_s=round(age, 4),
                       pose_pull_delay_s=(None if pose_pull_delay_s is None
                                          else round(float(pose_pull_delay_s), 4)),
                       truth_age_s=(None if truth_age_s is None
                                    else round(float(truth_age_s), 4)),
                       truth=dict(x=float(truth_xy[0]), y=float(truth_xy[1]),
                                  z=GROUND_Z, assumption="ground z=0, 1.75 m ref"),
                       camera_xyz=[float(x) for x in cam], rotation=[float(x) for x in R.flatten()],
                       intrinsics=[fx, fy, cx, cy], size=[W, H],
                       depth=round(depth, 3), uv=[round(u, 1), round(v, 1)],
                       expected_h_px=round(exp_h, 2), horiz_m=round(horiz, 2),
                       image_sha256=image_sha, truth_triggered=True,
                       non_control_use=True,
                       pose_alignment=("unstamped_current_service_approximation"
                                       if pose_time is None else "timestamp_matched"),
                       image_pose_delay_s=(age if pose_time is None else
                                          float(pose_time)-float(image_time)))
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
    ap.add_argument("--wiring", default="",
                    help="<round>/flight/wiring.json; authoritative uav_id<->model_name")
    ap.add_argument("--uavs", default="uav_1,uav_2,uav_3,uav_4,uav_5,uav_6")
    ap.add_argument("--model-template", default="typhoon_h480_%d",
                    help="fallback model name for uav_N when --wiring is absent")
    ap.add_argument("--cam-link-suffix", default="::cgo3_camera_link",
                    help="same link the production node uses")
    ap.add_argument("--cam-off-bl", default=DEFAULT_CAM_OFF_BL,
                    help="camera offset in base_link, same value as production")
    ap.add_argument("--pose-source", choices=("service", "topic"), default="service",
                    help="service = mirror production (pull get_link_state at "
                         "arrival); topic = mavros pose with nearest-stamp history")
    ap.add_argument("--pose-topic", default="/%s/mavros/local_position/pose")
    ap.add_argument("--per-uav-per-sec", type=float, default=1.0)
    ap.add_argument("--total-cap", type=int, default=120)
    ap.add_argument("--min-expected-px", type=float, default=4.0,
                    help="sampling heuristic only; NOT a competition criterion")
    ap.add_argument("--max-range-m", type=float, default=30.0,
                    help="sampling heuristic only; NOT a competition criterion")
    ap.add_argument("--pair-tol-s", type=float, default=0.02)
    ap.add_argument("--pose-max-age-s", type=float, default=2.0)
    ap.add_argument("--image-max-age-s", type=float, default=IMAGE_MAX_AGE_S,
                    help="reject originals older than this (production uses 1.0)")
    ap.add_argument("--truth-max-age-s", type=float, default=1.0)
    ap.add_argument("--pose-history-s", type=float, default=3.0)
    ap.add_argument("--queue-max", type=int, default=64)
    ap.add_argument("--poll-s", type=float, default=0.2)
    ap.add_argument("--self-check", action="store_true",
                    help="print the resolved configuration and exit (no ROS)")
    return ap


def resolve_links(args):
    """uav_id -> (model_name, camera_link). Wiring wins; template is fallback."""
    rows, mapping = load_wiring(args.wiring)
    out = {}
    for uav in [u for u in args.uavs.split(",") if u]:
        model = mapping.get(uav)
        if model is None and uav.startswith("uav_"):
            try:
                model = args.model_template % (int(uav.split("_")[1]) - 1)
            except Exception:
                model = None
        out[uav] = (model, (model + args.cam_link_suffix) if model else None)
    return out, rows


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    script_sha = sha256_file(os.path.abspath(__file__))
    links, wiring_rows = resolve_links(args)
    cfg = dict(vars(args))
    cfg["script_sha256"] = script_sha
    cfg["resolved_links"] = {k: v[1] for k, v in links.items()}
    cfg["wiring_uavs"] = len(wiring_rows)
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
    missing = [u for u, (m, l) in links.items() if not l]
    if missing:
        print("cannot resolve camera link for: %s" % ",".join(missing))
        return 2

    cap = BoundedCapture(args.out_dir, per_uav_per_sec=args.per_uav_per_sec,
                         total_cap=args.total_cap,
                         min_expected_px=args.min_expected_px,
                         max_range_m=args.max_range_m,
                         pose_max_age_s=args.pose_max_age_s,
                         pair_tol_s=args.pair_tol_s,
                         image_max_age_s=args.image_max_age_s,
                         pose_history_s=args.pose_history_s,
                         truth_max_age_s=args.truth_max_age_s,
                         queue_max=args.queue_max, enabled=True)

    import rospy
    from cv_bridge import CvBridge
    from gazebo_msgs.srv import GetLinkState, GetModelState
    from sensor_msgs.msg import CameraInfo, Image

    stopping = {"flag": False}

    def _stop(signum, frame):
        stopping["flag"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    rospy.init_node("white_dev_observer", anonymous=False)
    state = {"truth": None, "truth_wall": None, "image": {}, "intr": {}}
    off_bl = np.array([float(v) for v in args.cam_off_bl.split(",")])
    gls = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    def make_image_cb(uav):
        bridge = CvBridge()

        def cb(msg):
            try:
                # Keep only the latest message; decode only a selected frame.
                state["image"][uav] = msg
            except Exception:
                cap._skip("cv_bridge_failed")
                return
        return cb

    def make_info_cb(uav):
        def cb(msg):
            state["intr"][uav] = ([msg.K[0], msg.K[4], msg.K[2], msg.K[5]],
                                  [msg.width, msg.height])
        return cb

    def pull_pose(uav):
        """Camera pose at arrival, exactly like perception_real.py does it."""
        model, link = links[uav]
        response = gls(link, "world")
        if not response.success:
            raise ValueError("camera link pose unavailable for %s" % link)
        ls = response.link_state
        p = ls.pose.position
        o = ls.pose.orientation
        R = quat_to_R(o.x, o.y, o.z, o.w)
        off = R @ off_bl
        xyz = [p.x + off[0], p.y + off[1], p.z + off[2]]
        return dict(xyz=xyz, rotation=R.flatten().tolist())

    subs = []
    if args.pose_source == "topic":
        from geometry_msgs.msg import PoseStamped

        def make_pose_cb(uav):
            def cb(msg):
                q = msg.pose.pose.orientation if hasattr(msg.pose, "pose") else msg.pose.orientation
                pos = msg.pose.pose.position if hasattr(msg.pose, "pose") else msg.pose.position
                R = quat_to_R(q.x, q.y, q.z, q.w)
                off = R @ off_bl
                cap.note_pose(uav, msg.header.stamp.to_sec(),
                              [pos.x + off[0], pos.y + off[1], pos.z + off[2]],
                              R.flatten().tolist())
            return cb

    for uav in links:
        subs.append(rospy.Subscriber("/%s/cgo3_camera/image_raw" % uav, Image,
                                     make_image_cb(uav), queue_size=1))
        subs.append(rospy.Subscriber("/%s/cgo3_camera/camera_info" % uav, CameraInfo,
                                     make_info_cb(uav), queue_size=1))
        if args.pose_source == "topic":
            subs.append(rospy.Subscriber(args.pose_topic % uav, PoseStamped,
                                         make_pose_cb(uav), queue_size=1))

    print("[white_dev] enabled; out=%s cap=%d px>=%.1f range<=%.1f m pose=%s "
          "image_max_age=%.1f s"
          % (args.out_dir, args.total_cap, args.min_expected_px, args.max_range_m,
             args.pose_source, args.image_max_age_s), flush=True)
    while not rospy.is_shutdown() and not stopping["flag"]:
        if result_finished(args.result_file):
            print("[white_dev] result file reports a terminal state; exiting", flush=True)
            break
        if cap.saved < cap.total_cap:
            before = rospy.Time.now().to_sec()
            try:
                response = gms(args.truth_model, "world")
                after = rospy.Time.now().to_sec()
                if response.success and 0 <= after-before <= .05:
                    p = response.pose.position
                    state["truth"] = (float(p.x), float(p.y), float(p.z))
                    state["truth_wall"] = (before+after)/2.
                else:
                    state["truth"] = None
                    cap._skip("truth_query_unavailable")
            except Exception:
                state["truth"] = None
                cap._skip("truth_query_failed")
        for uav, msg in list(state["image"].items()):
            if uav not in state["intr"]:
                continue
            intr, size = state["intr"][uav]
            img_t = msg.header.stamp.to_sec()
            # Image headers use ROS time (/clock in this simulation), not UTC.
            wall = rospy.Time.now().to_sec()
            if cap.saved >= cap.total_cap:
                continue
            last = cap.last_saved.get(uav)
            if last is not None and wall-last < 1.0/cap.per_uav_per_sec:
                continue
            if wall-img_t > cap.image_max_age_s:
                cap._skip("image_too_old")
                continue
            pose, pose_time, pull_delay = None, None, None
            if args.pose_source == "service":
                t_pull = time.time()
                try:
                    pose = pull_pose(uav)
                except Exception as error:
                    cap._skip("pose_pull_failed")
                    print("[white_dev] pose pull failed (%s): %s" % (uav, error), flush=True)
                    continue
                pull_delay = time.time() - t_pull
                pose_time = None      # service pose has no independent stamp
            truth_wall = state["truth_wall"]
            truth_age = None if truth_wall is None else (wall - truth_wall)
            ok, reason, payload = cap.should_capture(
                uav, img_t, pose, pose_time, state["truth"], intr, size, wall,
                truth_age_s=truth_age, pose_pull_delay_s=pull_delay)
            if ok:
                payload["truth_source"] = "/gazebo/get_model_state"
                payload["truth_sample_s"] = state["truth_wall"]
                try:
                    img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                except Exception:
                    cap._skip("cv_bridge_failed")
                    continue
                cap.commit(img, payload, wall)
        # Poll termination even if Gazebo dies and /clock stops advancing.
        time.sleep(args.poll_s)

    print("[white_dev] exit; %s" % json.dumps(cap.stats(), ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
