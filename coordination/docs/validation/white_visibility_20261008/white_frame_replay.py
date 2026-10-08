#!/usr/bin/env python3
"""Replay the saved original images whose instant has the white truth inside the
geometric field of view, and check whether the projected target location
actually carries white-person-like pixels, plus what the colour/person models
emitted for that image.

Read-only observer: truth is used ONLY to compute a projected pixel location for
diagnosis. Nothing here feeds perception, reporting or control.

Usage: python3 white_frame_replay.py --round <dir> --audit-json <audit2 json>
                                      [--json-out <path>]
"""
import argparse
import json
import math
import os
import sys

import numpy as np

M_OPT2LINK = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
DEFAULT_FX = 205.46963709898583
DEFAULT_CX, DEFAULT_CY = 320.5, 180.5
PERSON_M = 1.75
PERSON_W = 0.50


def load_jsonl(p):
    return [json.loads(l) for l in open(p, encoding="utf-8") if l.strip()]


def project(cam, rot9, x, y, z, fx, fy, cx, cy):
    R = np.asarray(rot9, dtype=float).reshape(3, 3)
    d = M_OPT2LINK.T @ R.T @ (np.array([x, y, z], float) - np.asarray(cam, float))
    if abs(d[2]) < 1e-9:
        return float(d[2]), None, None
    return float(d[2]), fx * float(d[0]) / d[2] + cx, fy * float(d[1]) / d[2] + cy


def white_stats(img, u, v, h_px):
    """Bright/white pixel share in a box around (u,v) sized ~1.6x the expected box."""
    import cv2
    H, W = img.shape[:2]
    bw = max(4.0, PERSON_W * (h_px / PERSON_M))
    half_w = max(4, int(bw * 0.9))
    half_h = max(4, int(h_px * 0.8))
    x1, x2 = max(0, int(u) - half_w), min(W, int(u) + half_w + 1)
    y1, y2 = max(0, int(v) - half_h), min(H, int(v) + half_h + 1)
    if x2 <= x1 or y2 <= y1:
        return None
    patch = img[y1:y2, x1:x2]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    white = ((hsv[:, :, 1] < 70) & (hsv[:, :, 2] > 150)).mean()
    return dict(region=[int(x1), int(y1), int(x2), int(y2)],
                mean_v=float(patch.mean()),
                white_frac=float(white),
                px=int(patch.shape[0] * patch.shape[1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--round", required=True)
    ap.add_argument("--audit-json", required=True)
    ap.add_argument("--json-out", default="")
    ap.add_argument("--device", default="0")
    args = ap.parse_args()

    aud = json.load(open(args.audit_json, encoding="utf-8"))
    frames = aud["frames"]
    cams = load_jsonl(os.path.join(args.round, "flight", "algorithm",
                                   "search_camera_frames.jsonl"))
    cam_by = {}
    for c in cams:
        cam_by.setdefault((c["uav_id"], round(float(c["image_s"]), 3)), c)

    todo = [f for f in frames if f.get("in_fov") and f.get("saved_exact")]
    print("in-FOV frames with an exact original image: %d" % len(todo))

    import cv2
    from ultralytics import YOLO
    color = YOLO(os.environ.get(
        "PR_WEIGHTS",
        "/mnt/d/a/.robocup/robo_team/weights/best_yolo11n_bino_v1.pt"))
    person = YOLO(os.environ.get(
        "PR_PERSON_VERIFY_WEIGHTS",
        "/mnt/d/a/.robocup/robo_team/weights/yolo11n_person.pt"))

    rows = []
    for f in sorted(todo, key=lambda x: x["t_image"]):
        path = f["saved_exact"][0]
        img = cv2.imread(path)
        if img is None:
            rows.append(dict(uav=f["uav"], t=f["t_image"], path=path,
                             error="image_unreadable"))
            continue
        c = cam_by.get((f["uav"], round(f["t_image"], 3)))
        fx = fy = DEFAULT_FX
        cx, cy = DEFAULT_CX, DEFAULT_CY
        if c and isinstance(c.get("intrinsics"), (list, tuple)):
            fx, fy, cx, cy = [float(v) for v in c["intrinsics"]]
        tr = f["truth"]
        depth, u, v = project(c["camera_xyz"], c["camera_rotation"],
                              tr["x"], tr["y"], 0.0 + PERSON_M * 0.5,
                              fx, fy, cx, cy)
        h_px = PERSON_M * fx / depth if depth > 0 else None
        st = white_stats(img, u, v, h_px) if (depth > 0 and u is not None) else None
        # models on the whole image
        cres = color(img, conf=0.25, verbose=False, device=args.device)[0]
        cboxes = [(int(b.cls), float(b.conf), [float(x) for x in b.xyxy[0]])
                  for b in cres.boxes]
        whites = [b for b in cboxes if b[0] == 3]
        pres = person(img, classes=[0], conf=0.10, verbose=False,
                      device=args.device)[0]
        ppeople = [[float(x) for x in b.xyxy[0]] + [float(b.conf)]
                   for b in pres.boxes]
        nearest_white_d = None
        if whites and depth > 0 and u is not None:
            nearest_white_d = min(
                math.hypot((b[2][0] + b[2][2]) / 2 - u, (b[2][1] + b[2][3]) / 2 - v)
                for b in whites)
        rows.append(dict(
            uav=f["uav"], t=f["t_image"], path=os.path.basename(path),
            truth=tr, horiz_m=f["horiz_m"], depth=depth,
            proj_uv=[round(u, 1) if u is not None else None,
                     round(v, 1) if v is not None else None],
            expected_h_px=round(h_px, 1) if h_px else None,
            white_stats=st,
            colour_boxes=[[b[0], round(b[1], 3), [round(x, 1) for x in b[2]]]
                          for b in cboxes],
            white_boxes=[[round(b[1], 3), [round(x, 1) for x in b[2]]] for b in whites],
            nearest_white_box_dist_px=(round(nearest_white_d, 1)
                                       if nearest_white_d is not None else None),
            person_boxes=[[round(x, 1) for x in p[:4]] + [round(p[4], 3)]
                          for p in ppeople]))
        r = rows[-1]
        print("%s t=%.3f dist=%6.2f exp_h=%s proj=%s white_boxes=%d person_boxes=%d white_frac=%s" % (
            r["uav"], r["t"], r["horiz_m"], r["expected_h_px"], r["proj_uv"],
            len(r["white_boxes"]), len(r["person_boxes"]),
            ("%.3f" % st["white_frac"]) if st else "n/a"))

    if args.json_out:
        os.makedirs(os.path.dirname(args.json_out) or ".", exist_ok=True)
        json.dump(dict(note="truth used only to compute a diagnosis pixel location; "
                            "not a perception or control input",
                       person_m=PERSON_M, person_w=PERSON_W, rows=rows),
                  open(args.json_out, "w", encoding="utf-8"),
                  ensure_ascii=False, indent=1)
        print("wrote %s" % args.json_out)
    return 0


if __name__ == "__main__":
    sys.exit(main())