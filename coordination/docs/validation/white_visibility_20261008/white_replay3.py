#!/usr/bin/env python3
"""Replay all in-FOV frames that have an exact original image (v3, 34 frames).

Per frame this records, without changing any threshold:
- the frame identity: uav, image time, original SHA256, truth time difference
- stated assumptions and the projection point (ground z=0, 1.75 m reference)
- horizontal distance, forward depth, expected pixel height
- ALL colour model boxes (cls/conf/xyxy) and ALL person boxes (conf/xyxy)
- IoU of every colour box against every person box (association evidence)
- the jersey/torso verification result for proof classes
- white-pixel fraction in a box around the projected point
- a bounded annotated PNG: projected-truth marker vs real detections

Truth is used ONLY to place a diagnosis marker. It never feeds perception,
reporting or control. Read-only observer.

Usage: python3 white_replay3.py --round <dir> --audit-json <white_audit3 json>
                                [--json-out <path>] [--annot-dir <dir>]
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
CLASS_NAMES = {0: "red", 1: "green", 2: "blue", 3: "white", 4: "brown"}
PROOF_CLASSES = (0, 1, 2, 3)


def to_float(v):
    try:
        return float(v)
    except Exception:
        return None


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return float(inter / ua) if ua > 0 else 0.0


def project(cam, x, y, z, fx, fy, cx, cy):
    R = np.asarray(cam["camera_rotation"], float).reshape(3, 3)
    d = M_OPT2LINK.T @ R.T @ (np.array([x, y, z], float)
                              - np.asarray(cam["camera_xyz"], float))
    if abs(d[2]) < 1e-9:
        return float(d[2]), None, None
    return float(d[2]), fx * float(d[0]) / d[2] + cx, fy * float(d[1]) / d[2] + cy


def white_fraction(img, u, v, h_px):
    import cv2
    H, W = img.shape[:2]
    bw = max(4.0, PERSON_W * (h_px / PERSON_M))
    hw, hh = max(4, int(bw * 0.9)), max(4, int(h_px * 0.8))
    x1, x2 = max(0, int(u) - hw), min(W, int(u) + hw + 1)
    y1, y2 = max(0, int(v) - hh), min(H, int(v) + hh + 1)
    if x2 <= x1 or y2 <= y1:
        return None
    patch = img[y1:y2, x1:x2]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    return float(((hsv[:, :, 1] < 70) & (hsv[:, :, 2] > 150)).mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--round", required=True)
    ap.add_argument("--audit-json", required=True)
    ap.add_argument("--json-out", default="")
    ap.add_argument("--annot-dir", default="")
    ap.add_argument("--device", default="0")
    args = ap.parse_args()

    aud = json.load(open(args.audit_json, encoding="utf-8"))
    truth = [(f["truth_time"], f["truth_xy"][0], f["truth_xy"][1])
             for f in aud["frames"]
             if f.get("truth_time") is not None and f.get("truth_xy")]
    # dedupe truth by time
    truth = sorted({(round(t, 3), x, y) for t, x, y in truth})
    cams = {}
    for line in open(os.path.join(args.round, "flight", "algorithm",
                                 "search_camera_frames.jsonl"), encoding="utf-8"):
        c = json.loads(line)
        cams[(c["uav_id"], round(float(c["image_s"]), 3))] = c

    todo = [f for f in aud["frames"]
            if f.get("in_search_index") and f.get("in_fov_0875") and f.get("path")]
    print("frames to replay: %d" % len(todo))

    import cv2
    from ultralytics import YOLO
    color = YOLO(os.environ.get("PR_WEIGHTS",
                 "/mnt/d/a/.robocup/robo_team/weights/best_yolo11n_bino_v1.pt"))
    person = YOLO(os.environ.get("PR_PERSON_VERIFY_WEIGHTS",
                  "/mnt/d/a/.robocup/robo_team/weights/yolo11n_person.pt"))
    from jersey_color import torso_fractions, supported

    if args.annot_dir:
        os.makedirs(args.annot_dir, exist_ok=True)

    out_rows = []
    for f in sorted(todo, key=lambda x: (x["uav"], x["t_image"])):
        img = cv2.imread(f["path"])
        if img is None:
            out_rows.append(dict(uav=f["uav"], t_image=f["t_image"],
                                 path=f["path"], error="image_unreadable"))
            continue
        c = cams.get((f["uav"], round(f["t_image"], 3)))
        fx = fy = DEFAULT_FX
        cx, cy = DEFAULT_CX, DEFAULT_CY
        if c and isinstance(c.get("intrinsics"), (list, tuple)):
            fx, fy, cx, cy = [float(v) for v in c["intrinsics"]]
        tr = min(truth, key=lambda s: abs(s[0] - f["t_image"]))
        rec = dict(
            uav=f["uav"], t_image=f["t_image"],
            image_s=f["t_image"], sample_s=(c or {}).get("sample_s"),
            original_sha256=f.get("sha", ""), path=f["path"],
            truth_time=tr[0], truth_dt=round(f["t_image"] - tr[0], 4),
            assumptions=dict(ground_z=0.0, person_m=PERSON_M,
                             projection_point="z=0.875 (midpoint) and z=1.0"),
            id_source=f.get("id_source"))

        # colour model
        cres = color(img, conf=0.25, verbose=False, device=args.device)[0]
        cboxes = []
        for b in cres.boxes:
            xy = [float(v) for v in b.xyxy[0]]
            cboxes.append(dict(cls=int(b.cls), name=CLASS_NAMES.get(int(b.cls), "?"),
                               conf=round(float(b.conf), 4),
                               xyxy=[round(v, 1) for v in xy]))
        # person model at the production conf
        pres = person(img, classes=[0], conf=0.10, verbose=False,
                      device=args.device)[0]
        pboxes = []
        for b in pres.boxes:
            xy = [float(v) for v in b.xyxy[0]]
            pboxes.append(dict(conf=round(float(b.conf), 4),
                               xyxy=[round(v, 1) for v in xy],
                               h_px=round(xy[3] - xy[1], 1)))
        assoc = []
        for cb in cboxes:
            if cb["cls"] not in PROOF_CLASSES:
                continue
            best = 0.0
            best_i = -1
            for i, pb in enumerate(pboxes):
                v = iou(cb["xyxy"], pb["xyxy"])
                if v > best:
                    best, best_i = v, i
            fr = None
            try:
                fr = torso_fractions(img, cb["xyxy"])
            except Exception as e:
                fr = dict(error=str(e))
            assoc.append(dict(cls=cb["cls"], name=cb["name"], conf=cb["conf"],
                              best_person_iou=round(best, 4),
                              best_person_conf=(pboxes[best_i]["conf"]
                                                if best_i >= 0 else None),
                              matched=bool(best >= 0.25),
                              shirt_fractions=fr,
                              shirt_supported=(bool(supported(cb["name"], fr))
                                               if isinstance(fr, dict) and "error" not in fr
                                               else None)))
        rec["colour_boxes"] = cboxes
        rec["person_boxes"] = pboxes
        rec["association"] = assoc

        for label, z in (("0875", 0.875), ("100", 1.0)):
            depth, u, v = project(c, tr[1], tr[2], z, fx, fy, cx, cy) if c else (None, None, None)
            rec["depth_%s" % label] = round(depth, 3) if depth is not None else None
            rec["proj_uv_%s" % label] = ([round(u, 1), round(v, 1)]
                                         if u is not None else None)
            rec["expected_h_px_%s" % label] = (round(PERSON_M * fx / depth, 2)
                                               if depth and depth > 0 else None)
        rec["white_frac_at_proj"] = (white_fraction(img, rec["proj_uv_0875"][0],
                                                    rec["proj_uv_0875"][1],
                                                    rec["expected_h_px_0875"])
                                     if rec.get("proj_uv_0875") else None)
        rec["horiz_m"] = round(f.get("horiz_m", -1), 2)

        if args.annot_dir and rec.get("proj_uv_0875"):
            ann = img.copy()
            for cb in cboxes:
                x1, y1, x2, y2 = [int(v) for v in cb["xyxy"]]
                col = (0, 255, 255) if cb["cls"] != 3 else (255, 0, 255)
                cv2.rectangle(ann, (x1, y1), (x2, y2), col, 1)
                cv2.putText(ann, "%s %.2f" % (cb["name"], cb["conf"]),
                            (x1, max(10, y1 - 3)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.35, col, 1)
            for pb in pboxes:
                x1, y1, x2, y2 = [int(v) for v in pb["xyxy"]]
                cv2.rectangle(ann, (x1, y1), (x2, y2), (0, 255, 0), 1)
                cv2.putText(ann, "P%.2f" % pb["conf"], (x1, min(ann.shape[0] - 3, y2 + 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)
            u, v = rec["proj_uv_0875"]
            h = rec["expected_h_px_0875"] or 6
            cv2.drawMarker(ann, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 12, 1)
            cv2.rectangle(ann, (int(u - 4), int(v - h / 2)), (int(u + 4), int(v + h / 2)),
                          (0, 0, 255), 1)
            cv2.putText(ann, "TRUTH-PROJ %.1fm %.1fpx" % (rec["horiz_m"], h),
                        (6, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            name = "annot_%s_%s_%03d.png" % (f["uav"], str(round(f["t_image"], 1)).replace(".", "_"),
                                             int(f["t_image"]) % 1000)
            cv2.imwrite(os.path.join(args.annot_dir, name), ann)
            rec["annotated"] = name

        out_rows.append(rec)
        print("%-6s t=%.3f dist=%6.2f exp_h=%5.2f colour=%d person=%d white_frac=%s assoc=%s" % (
            rec["uav"], rec["t_image"], rec["horiz_m"],
            rec["expected_h_px_0875"] or -1, len(cboxes), len(pboxes),
            ("%.3f" % rec["white_frac_at_proj"]) if rec["white_frac_at_proj"] is not None else "n/a",
            [("%s iou=%.2f" % (a["name"], a["best_person_iou"])) for a in assoc]))

    if args.json_out:
        os.makedirs(os.path.dirname(args.json_out) or ".", exist_ok=True)
        json.dump(dict(note="truth used only for a diagnosis marker; not a perception, "
                            "reporting or control input",
                       model_sha=dict(
                           color="9d0c1fe2b6370a40eb274573de06ac4e36c81eb7a39149dd1ac4aff0ed6990c2",
                           person="0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1"),
                       rows=out_rows), open(args.json_out, "w", encoding="utf-8"),
                  ensure_ascii=False, indent=1, default=str)
        print("wrote %s" % args.json_out)
    return 0


if __name__ == "__main__":
    sys.exit(main())