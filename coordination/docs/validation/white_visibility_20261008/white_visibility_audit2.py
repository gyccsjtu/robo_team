#!/usr/bin/env python3
"""White visibility audit, corrected per codex review 13e7a40 (v2).

Corrections applied:
- Align truth to camera frames by IMAGE time (image_s), not sample_s. sample_s
  is roughly 0.5 s later (the processing instant).
- actor_truth rows are [time, x, y, actor_id] - the last field is the ACTOR ID,
  not Z. This script reads (time, x, y) and states the ground-plane z=0 and
  1.75 m reference-height assumptions explicitly.
- search_camera_frames only covers frames actually processed for a search task;
  conclusions are limited to these search records, not "whole flight".
- Saved-image matching requires the SAME aircraft and an exact original-image
  time (+-0.01 s) with the file present. Loose +-0.3 s neighbours are reported
  separately and are NOT treated as originals.

Usage:
  python3 white_visibility_audit2.py --round <round_dir> [--json-out <path>]
"""
import argparse
import json
import math
import os
import sys

import numpy as np

M_OPT2LINK = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])
DEFAULT_FX = 205.46963709898583
DEFAULT_CX, DEFAULT_CY = 320.5, 180.5
DEFAULT_W, DEFAULT_H = 640, 360
PERSON_M = 1.75            # reference height assumption (animation not verified)
GROUND_Z = 0.0             # ground-plane assumption, stated explicitly


def load_jsonl(path):
    out = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def truth_xy(path):
    """Return [(t, x, y, actor_id)] - last field is actor id, NOT z."""
    rows = []
    for r in load_jsonl(path):
        s = r.get("sample")
        if isinstance(s, (list, tuple)) and len(s) >= 3:
            rows.append((float(s[0]), float(s[1]), float(s[2]),
                         int(s[3]) if len(s) > 3 else -1))
    rows.sort(key=lambda r: r[0])
    return rows


def nearest(rows, t, tol):
    best, best_dt = None, None
    lo, hi = 0, len(rows) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        dt = rows[mid][0] - t
        if best_dt is None or abs(dt) < best_dt:
            best_dt, best = abs(dt), rows[mid]
        if dt < 0:
            lo = mid + 1
        else:
            hi = mid - 1
    if best is None or best_dt > tol:
        return None
    return best


def project(cam_xyz, rot9, x, y, z, fx, fy, cx, cy):
    R = np.asarray(rot9, dtype=float).reshape(3, 3)
    d = M_OPT2LINK.T @ R.T @ (np.array([x, y, z], dtype=float)
                              - np.asarray(cam_xyz, dtype=float))
    depth = float(d[2])
    if abs(depth) < 1e-9:
        return depth, None, None
    return depth, fx * float(d[0]) / depth + cx, fy * float(d[1]) / depth + cy


def saved_images(round_dir):
    """Indexed original images: [(uav_id, image_time_s, path_or_name)]."""
    out = []
    fj = os.path.join(round_dir, "observers", "frames", "frames.jsonl")
    if os.path.isfile(fj):
        for r in load_jsonl(fj):
            o = r.get("observation", {})
            t = r.get("image_sample_s")
            if t is None:
                t = o.get("sample_s")
            if t is not None and o.get("uav_id"):
                name = r.get("image", "")
                full = os.path.join(round_dir, "observers", "frames", name)
                out.append((o["uav_id"], float(t),
                            full if (name and os.path.isfile(full)) else ""))
    alg = os.path.join(round_dir, "flight", "algorithm")
    if os.path.isdir(alg):
        for d in sorted(os.listdir(alg)):
            if not d.startswith("evidence_"):
                continue
            idx = os.path.join(alg, d, "index.jsonl")
            if not os.path.isfile(idx):
                continue
            # evidence_typhoon_h480_N[_suffix] -> uav_(N+1)
            parts = d.split("_")
            model = parts[2] if len(parts) > 2 else ""
            try:
                uav = "uav_%d" % (int(model.split("h480_")[1]) + 1)
            except Exception:
                uav = "?"
            for row in load_jsonl(idx):
                t = row.get("image_stamp")
                png = row.get("png", "")
                if t is None:
                    continue
                full = os.path.join(alg, d, png)
                out.append((uav, float(t), full if os.path.isfile(full) else ""))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--round", required=True)
    ap.add_argument("--json-out", default="")
    ap.add_argument("--truth-tol", type=float, default=0.06)
    ap.add_argument("--exact-tol", type=float, default=0.01)
    ap.add_argument("--loose-tol", type=float, default=0.30)
    args = ap.parse_args()

    R = args.round
    truth = truth_xy(os.path.join(R, "observers",
                                 "independent_visual_accuracy_white.jsonl"))
    cams = load_jsonl(os.path.join(R, "flight", "algorithm",
                                   "search_camera_frames.jsonl"))
    saved = saved_images(R)
    print("truth samples: %d  (format time,x,y,actor_id)" % len(truth))
    print("search camera frames: %d  (search records only)" % len(cams))
    print("indexed original images: %d  (file present)"
          % sum(1 for s in saved if s[2]))
    actor_ids = sorted({t[3] for t in truth})
    print("actor ids seen in truth:", actor_ids)

    frames = []
    for c in cams:
        t_img = float(c.get("image_s", 0.0))
        tr = nearest(truth, t_img, args.truth_tol)
        fx = fy = DEFAULT_FX
        cx, cy = DEFAULT_CX, DEFAULT_CY
        W, H = DEFAULT_W, DEFAULT_H
        intr = c.get("intrinsics")
        if isinstance(intr, (list, tuple)) and len(intr) == 4:
            fx, fy, cx, cy = [float(v) for v in intr]
        size = c.get("size")
        if isinstance(size, (list, tuple)) and len(size) == 2:
            W, H = int(size[0]), int(size[1])
        rec = dict(uav=c.get("uav_id"), t_image=t_img,
                   t_sample=float(c.get("sample_s", 0.0)), has_truth=bool(tr))
        if tr is None:
            rec.update(in_fov=False, reason="no_truth_within_tol")
            frames.append(rec)
            continue
        tt, tx, ty, aid = tr
        rec["truth"] = dict(t=tt, x=tx, y=ty, actor_id=aid)
        depth, u, v = project(c.get("camera_xyz"), c.get("camera_rotation"),
                              tx, ty, GROUND_Z + PERSON_M * 0.5,
                              fx, fy, cx, cy)
        rec.update(depth=depth, u=u, v=v, forward=depth > 0,
                   horiz_m=math.hypot(tx - float(c["camera_xyz"][0]),
                                      ty - float(c["camera_xyz"][1])))
        inside = bool(depth > 0 and u is not None and 0 <= u < W and 0 <= v < H)
        rec["in_fov"] = inside
        rec["expected_h_px"] = (PERSON_M * fx / depth) if depth > 0 else None
        rec["reason"] = "in_fov" if inside else (
            "behind_camera" if depth <= 0 else "outside_image")
        # original-image matching, SAME aircraft required
        exact = [s for s in saved if s[0] == rec["uav"] and s[2]
                 and abs(s[1] - t_img) <= args.exact_tol]
        loose = [s for s in saved if s[0] == rec["uav"] and s[2]
                 and abs(s[1] - t_img) <= args.loose_tol]
        rec["saved_exact"] = [s[2] for s in exact]
        rec["saved_loose_only"] = [s[2] for s in loose] if not exact else []
        frames.append(rec)

    in_fov = [f for f in frames if f.get("in_fov")]
    fwd = [f for f in frames if f.get("forward")]
    behind = [f for f in frames if f.get("reason") == "behind_camera"]
    outside = [f for f in frames if f.get("reason") == "outside_image"]
    exact_hits = [f for f in in_fov if f["saved_exact"]]
    loose_only = [f for f in in_fov if not f["saved_exact"] and f["saved_loose_only"]]

    print("\n=== geometric FOV (search records only; occlusion/rendering NOT proven) ===")
    print("frames total              : %d" % len(frames))
    print("truth matched             : %d" % sum(1 for f in frames if f.get('has_truth')))
    print("target in front of camera : %d" % len(fwd))
    print("  inside image            : %d" % len(in_fov))
    print("  outside image           : %d" % len(outside))
    print("target behind camera      : %d" % len(behind))
    if in_fov:
        ds = sorted(f["horiz_m"] for f in in_fov)
        hs = sorted(f["expected_h_px"] for f in in_fov)
        print("in-FOV horizontal distance: min %.3f  p50 %.1f  max %.1f m"
              % (ds[0], ds[len(ds)//2], ds[-1]))
        print("in-FOV expected pixel height (1.75 m ref): min %.1f p50 %.1f max %.1f px"
              % (hs[0], hs[len(hs)//2], hs[-1]))
        print("in-FOV within 22 m / 12 m : %d / %d"
              % (sum(1 for x in ds if x <= 22), sum(1 for x in ds if x <= 12)))
        print("in-FOV with EXACT same-aircraft original (<=%.2f s): %d"
              % (args.exact_tol, len(exact_hits)))
        print("in-FOV with loose-only neighbour (<=%.2f s)      : %d"
              % (args.loose_tol, len(loose_only)))
        print("\n--- the %d in-FOV frames WITH an exact original image ---" % len(exact_hits))
        for f in sorted(exact_hits, key=lambda x: x["t_image"]):
            print("  %s t=%.3f dist=%6.2f m exp_h=%5.1f px uv=(%6.1f,%5.1f)  %s"
                  % (f["uav"], f["t_image"], f["horiz_m"], f["expected_h_px"],
                     f["u"], f["v"], os.path.basename(f["saved_exact"][0])))

    if args.json_out:
        os.makedirs(os.path.dirname(args.json_out) or ".", exist_ok=True)
        with open(args.json_out, "w", encoding="utf-8") as fh:
            json.dump(dict(
                corrections="codex review 13e7a40: align by image_s; actor_truth = [t,x,y,actor_id]; "
                            "same-aircraft exact original only; search records only",
                parameters=dict(truth_tol=args.truth_tol, exact_tol=args.exact_tol,
                                loose_tol=args.loose_tol,
                                assumptions=dict(ground_z=GROUND_Z, person_m=PERSON_M,
                                                 projection_point="ground_z + person_m*0.5"),
                                fx=DEFAULT_FX, cx=DEFAULT_CX, cy=DEFAULT_CY,
                                size=[DEFAULT_W, DEFAULT_H], M_OPT2LINK=M_OPT2LINK.tolist(),
                                convention="v_opt = M_OPT2LINK.T @ R.T @ (P - cam)"),
                truth_samples=len(truth), camera_frames=len(cams),
                in_fov=len(in_fov), forward=len(fwd), behind=len(behind),
                outside=len(outside), exact_originals=len(exact_hits),
                loose_only=len(loose_only), frames=frames), fh,
                ensure_ascii=False, indent=1)
        print("\nwrote %s" % args.json_out)
    return 0


if __name__ == "__main__":
    sys.exit(main())