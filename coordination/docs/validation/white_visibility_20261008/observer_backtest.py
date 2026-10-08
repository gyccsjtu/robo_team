#!/usr/bin/env python3
"""Offline backtest of white_evidence_observer's capture decision.

The observer itself needs a live simulation (it subscribes to topics and calls
the Gazebo link-state service), so it cannot be exercised offline. Its DECISION
FUNCTION can be: this replays the recorded v1.40 search-camera records through
the real BoundedCapture class and asks, for every record, whether the observer
would have saved that frame.

Why this exists: the first version of the observer paired the NEWEST pose stamp
against the image stamp with a 20 ms tolerance, and guarded staleness as
wall_now - pose_stamp. Replaying real records showed that contract rejects
100% of frames: measured image latency is min 0.216 / p50 0.472 / max 0.884 s.
The observer now mirrors perception_real.py (pull the camera link pose at
arrival, gate only on image age <= 1 s), and this script measures the yield of
both contracts on the same records.

Run:
  python3 observer_backtest.py --round <round_dir> [--json-out out.json]
"""
import argparse
import importlib.util
import json
import os
import sys

OBSERVER = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "..", "scripts", "white_evidence_observer.py")
WHITE_ACTOR_ID = 3

VARIANTS = {
    # name -> (pose_time policy, comment)
    "v1_latest_pose": ("latest_pose", "the first version: newest pose stamp vs image"),
    "production_mirror": ("service", "current: pose pulled at arrival, image-age gate"),
}


def load_observer(path):
    spec = importlib.util.spec_from_file_location("white_evidence_observer", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def load_jsonl(path):
    out = []
    if not os.path.isfile(path):
        return out
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except Exception:
                pass
    return out


def percentile(values, q):
    if not values:
        return None
    v = sorted(values)
    i = min(len(v) - 1, max(0, int(round((len(v) - 1) * q))))
    return v[i]


def latency_of(record):
    for fld in ("received_s", "sample_s"):
        if record.get(fld) is not None:
            return float(record[fld]) - float(record["image_s"])
    return 0.0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--round", required=True)
    ap.add_argument("--json-out", default="")
    ap.add_argument("--truth-tol", type=float, default=0.06)
    args = ap.parse_args()

    R = args.round
    mod = load_observer(os.path.abspath(OBSERVER))

    truth = []
    for r in load_jsonl(os.path.join(R, "observers",
                                     "independent_visual_accuracy_white.jsonl")):
        s = r.get("sample")
        if (r.get("kind") or "") != "actor_truth":
            continue
        if not (isinstance(s, (list, tuple)) and len(s) >= 4):
            continue
        if int(s[3]) != WHITE_ACTOR_ID:
            continue
        truth.append((float(s[0]), float(s[1]), float(s[2])))
    truth.sort(key=lambda x: x[0])

    def nearest_truth(t):
        best, best_dt = None, None
        for row in truth:
            dt = abs(row[0] - t)
            if best_dt is None or dt < best_dt:
                best, best_dt = row, dt
        if best_dt is not None and best_dt <= args.truth_tol:
            return best
        return None

    cams = load_jsonl(os.path.join(R, "flight", "algorithm",
                                   "search_camera_frames.jsonl"))
    seen = set()
    recs = []
    for c in cams:
        key = (c["uav_id"], round(float(c["image_s"]), 3))
        if key in seen:
            continue
        seen.add(key)
        recs.append(c)

    lat = [latency_of(c) for c in recs]
    print("=== recorded image latency (arrival - capture) ===")
    print("n=%d  min=%.3f  p50=%.3f  p90=%.3f  max=%.3f"
          % (len(lat), min(lat), percentile(lat, .5), percentile(lat, .9), max(lat)))

    results = {}
    for name, (policy, comment) in VARIANTS.items():
        for caps in ("unlimited", "production"):
            cap = mod.BoundedCapture(
                "/tmp/backtest_unused",
                per_uav_per_sec=(1e9 if caps == "unlimited" else 1.0),
                total_cap=(10 ** 9 if caps == "unlimited" else 120),
                min_expected_px=4.0, max_range_m=30.0, enabled=True)
            for c in recs:
                t_img = float(c["image_s"])
                lag = latency_of(c)
                wall = t_img + lag
                tr = nearest_truth(t_img)
                if tr is None:
                    # no truth within tolerance: still exercise the age gate
                    cap.should_capture(c["uav_id"], t_img, None, None, None,
                                       c.get("intrinsics"), c.get("size"), wall)
                    continue
                pose = dict(xyz=c["camera_xyz"], rotation=c["camera_rotation"])
                if policy == "latest_pose":
                    pose_t = t_img + lag          # newest available pose
                else:
                    pose_t = None                 # service pose: no stamp
                ok, _, _ = cap.should_capture(c["uav_id"], t_img, pose, pose_t,
                                              (tr[1], tr[2]), c.get("intrinsics"),
                                              c.get("size"), wall)
                if ok:
                    cap.saved += 1
                    cap.seen_keys.add((c["uav_id"], round(t_img, 3)))
                    cap.last_saved[c["uav_id"]] = wall
            results["%s/%s" % (name, caps)] = cap.stats()
            print("=== %s (%s) ===" % (name, caps))
            print("  %s" % comment)
            print("  stored           : %d / %d" % (cap.stats()["saved"], len(recs)))
            print("  skip reasons     : %s" % (json.dumps(cap.skipped, ensure_ascii=False)))

    payload = dict(round=R, records=len(recs),
                   latency=dict(min=min(lat), p50=percentile(lat, .5),
                                p90=percentile(lat, .9), max=max(lat)),
                   results=results)
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=1)
        print("\nwrote %s" % args.json_out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
