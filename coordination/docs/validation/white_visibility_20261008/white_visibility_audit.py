#!/usr/bin/env python3
"""White actor visibility audit for one competition round (offline, read-only).

For every search camera frame of all six aircraft, take the temporally nearest
white-actor truth sample and compute, in the optical frame that the production
source actually uses (perception_real.py):

    d_opt = M_OPT2LINK.T @ R.T @ (P_world - camera_xyz)
    forward depth = d_opt[2];  u = FX*d_opt[0]/d_opt[2] + CX ; v likewise

and report: forward/behind, inside/outside the image, horizontal distance,
expected pixel height for a 1.75 m person (1.75*FX/depth).

Geometric field of view ONLY: occlusion and rendering are NOT proven here.
No truth is fed to any control path; this script reads archives and prints.

Usage:
  python3 white_visibility_audit.py --round <round_dir> [--json-out <path>]

Round dir must contain observers/independent_visual_accuracy_white.jsonl
(rows {"kind":"actor_truth","sample":[t,x,y,z]}) and
flight/algorithm/search_camera_frames.jsonl.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

# Production constants (perception_real.py:309, and the runtime intrinsics seen
# in search_camera_frames.jsonl for this round: 640x360, fx=fy=205.46963709898583).
M_OPT2LINK = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])
DEFAULT_FX = 205.46963709898583
DEFAULT_CX, DEFAULT_CY = 320.5, 180.5
DEFAULT_W, DEFAULT_H = 640, 360
PERSON_M = 1.75


def load_jsonl(path):
    out = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def truth_samples(path):
    rows = []
    for r in load_jsonl(path):
        s = r.get("sample")
        if isinstance(s, (list, tuple)) and len(s) >= 3:
            rows.append((float(s[0]), float(s[1]), float(s[2]),
                         float(s[3]) if len(s) > 3 else 0.0))
    rows.sort(key=lambda r: r[0])
    return rows


def nearest_truth(rows, t, tol):
    """Binary-search the nearest sample within tol seconds."""
    best = None
    best_dt = None
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


def project(cam_xyz, rot9, world, fx, fy, cx, cy):
    R = np.asarray(rot9, dtype=float).reshape(3, 3)   # link->world (row-major)
    P = np.asarray(world, dtype=float)
    d = M_OPT2LINK.T @ R.T @ (P - np.asarray(cam_xyz, dtype=float))
    depth = float(d[2])
    if abs(depth) < 1e-9:
        return dict(depth=depth, u=None, v=None, forward=depth > 0)
    u = fx * float(d[0]) / depth + cx
    v = fy * float(d[1]) / depth + cy
    return dict(depth=depth, u=u, v=v, forward=depth > 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--round", required=True, help="round dir (contains observers/ and flight/)")
    ap.add_argument("--json-out", default="")
    ap.add_argument("--tol", type=float, default=0.5, help="truth match tolerance s")
    args = ap.parse_args()

    R = args.round
    truth = truth_samples(os.path.join(R, "observers",
                                       "independent_visual_accuracy_white.jsonl"))
    cams = load_jsonl(os.path.join(R, "flight", "algorithm",
                                   "search_camera_frames.jsonl"))
    print("truth samples: %d   camera frames: %d" % (len(truth), len(cams)))
    if not truth or not cams:
        print("INSUFFICIENT INPUT")
        return 2

    frames = []
    for c in cams:
        t = float(c.get("sample_s", c.get("image_s", 0.0)))
        tr = nearest_truth(truth, t, args.tol)
        fx = fy = DEFAULT_FX
        cx, cy = DEFAULT_CX, DEFAULT_CY
        W, H = DEFAULT_W, DEFAULT_H
        intr = c.get("intrinsics")
        if isinstance(intr, (list, tuple)) and len(intr) == 4:
            fx, fy, cx, cy = [float(v) for v in intr]
        size = c.get("size")
        if isinstance(size, (list, tuple)) and len(size) == 2:
            W, H = int(size[0]), int(size[1])
        rec = dict(uav=c.get("uav_id"), t_image=float(c.get("image_s", t)),
                   t_sample=t, has_truth=bool(tr))
        if tr is None:
            rec.update(in_fov=False, reason="no_truth_within_tol")
            frames.append(rec)
            continue
        tw, tx, ty, tz = tr
        # project the person's mid-height point (1.0 m above ground) and feet
        for label, z in (("foot", 0.0), ("mid", 1.0), ("top", 1.75)):
            rec[label] = project(c.get("camera_xyz"), c.get("camera_rotation"),
                                 [tx, ty, tz - 3.0 + z], fx, fy, cx, cy)
        horiz = math.hypot(tx - float(c["camera_xyz"][0]),
                           ty - float(c["camera_xyz"][1]))
        rec.update(truth_xy=[tx, ty], truth_z=tz, horiz_m=horiz)
        mid = rec["mid"]
        inside = (mid["forward"] and mid["u"] is not None
                  and 0.0 <= mid["u"] < W and 0.0 <= mid["v"] < H)
        rec["in_fov"] = bool(inside)
        rec["expected_h_px"] = (PERSON_M * fx / mid["depth"]) if mid["forward"] else None
        rec["reason"] = "in_fov" if inside else (
            "behind_camera" if not mid["forward"] else "outside_image")
        frames.append(rec)

    in_fov = [f for f in frames if f.get("in_fov")]
    behind = [f for f in frames if f.get("reason") == "behind_camera"]
    outside = [f for f in frames if f.get("reason") == "outside_image"]
    fwd = [f for f in frames if f.get("mid") and f["mid"]["forward"]]
    print("\n=== summary (geometric FOV only; occlusion/rendering NOT proven) ===")
    print("frames total:            %d" % len(frames))
    print("truth matched:           %d" % sum(1 for f in frames if f.get('has_truth')))
    print("target in front:         %d" % len(fwd))
    print("  of which in image:     %d" % len(in_fov))
    print("  of which outside img:  %d" % len(outside))
    print("target behind camera:    %d" % len(behind))
    if in_fov:
        ds = sorted(f["horiz_m"] for f in in_fov)
        hs = sorted(f["expected_h_px"] for f in in_fov)
        print("in-FOV horizontal distance m: min %.1f  p50 %.1f  max %.1f" % (
            ds[0], ds[len(ds)//2], ds[-1]))
        print("in-FOV expected pixel height (1.75 m): min %.1f  p50 %.1f  max %.1f" % (
            hs[0], hs[len(hs)//2], hs[-1]))
        print("\n--- in-FOV frames ---")
        for f in sorted(in_fov, key=lambda x: x["t_image"]):
            print("  %s t=%.3f dist=%5.1f m exp_h=%5.1f px u=%6.1f v=%6.1f depth=%6.1f" % (
                f["uav"], f["t_image"], f["horiz_m"], f["expected_h_px"],
                f["mid"]["u"], f["mid"]["v"], f["mid"]["depth"]))
    elif fwd:
        print("\n(no frame had the target inside the image; forward ones:)")
        for f in sorted(fwd, key=lambda x: x["t_image"])[:15]:
            print("  %s t=%.3f dist=%5.1f m u=%7.1f v=%7.1f" % (
                f["uav"], f["t_image"], f["horiz_m"], f["mid"]["u"], f["mid"]["v"]))

    if args.json_out:
        os.makedirs(os.path.dirname(args.json_out), exist_ok=True)
        with open(args.json_out, "w", encoding="utf-8") as fh:
            json.dump(dict(parameters=dict(tol=args.tol, person_m=PERSON_M,
                                           fx=DEFAULT_FX, cx=DEFAULT_CX, cy=DEFAULT_CY,
                                           size=[DEFAULT_W, DEFAULT_H],
                                           M_OPT2LINK=M_OPT2LINK.tolist(),
                                           convention="v_opt = M_OPT2LINK.T @ R.T @ (P_world - cam)"),
                           truth_samples=len(truth), camera_frames=len(cams),
                           in_fov=len(in_fov), forward=len(fwd), behind=len(behind),
                           outside=len(outside), frames=frames),
                      fh, ensure_ascii=False, indent=1)
        print("\nwrote %s" % args.json_out)
    return 0


if __name__ == "__main__":
    sys.exit(main())