#!/usr/bin/env python3
"""White visibility audit v3 - corrected input indexing (codex batch task).

Fixes the v2 defect: evidence channel ids were derived with d.split('_')[2],
which yields 'h480' for 'evidence_typhoon_h480_2', so every evidence original
was tagged '?' and silently dropped from same-aircraft matching.

Aircraft id rules (per WORKBUDDY_BATCH_TASK_20261008.md):
- Prefer the real camera binding: /uav_<N>/... in camera_binding.image_topic,
  falling back to camera_binding.image_header.frame_id.
- Cross-check against the directory model typhoon_h480_<M> -> uav_<M+1>.
- If binding and directory disagree: RECORD AND REJECT, never guess.
- Only when no binding exists at all may the directory fallback be used, and it
  is marked as such.

Truth rows: only kind=actor_truth with sample[3]==3 (other samples are not white
truth). sample format is [time, x, y, actor_id] - the last field is an ACTOR ID,
not Z. Projection assumptions (stated, not hidden): ground plane Z=0 and a
1.75 m reference person height.

Entries record: full path, file SHA256, source index, aircraft id, both times
and their difference.

Usage:
  python3 white_audit3.py --round <dir> [--json-out <path>] [--old-json <v2 json>]
"""
import argparse
import hashlib
import json
import math
import os
import re
import sys

import numpy as np

M_OPT2LINK = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
DEFAULT_FX = 205.46963709898583
DEFAULT_CX, DEFAULT_CY = 320.5, 180.5
DEFAULT_W, DEFAULT_H = 640, 360
PERSON_M = 1.75
GROUND_Z = 0.0
WHITE_ACTOR_ID = 3
EXACT_TOL = 0.01


def load_jsonl(p):
    out = []
    with open(p, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def sha256_file(path):
    h = hashlib.sha256()
    try:
        with open(path, "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                h.update(chunk)
    except OSError:
        return ""
    return h.hexdigest()


def uav_from_binding(cb):
    """Prefer the actual camera topic / frame id. None when unavailable."""
    cb = cb or {}
    for key in ("image_topic",):
        v = cb.get(key) or ""
        m = re.match(r"^/?(uav_\d+)/", v)
        if m:
            return m.group(1)
    fid = ((cb.get("image_header") or {}).get("frame_id")) or ""
    m = re.match(r"^(uav_\d+)/", fid)
    if m:
        return m.group(1)
    return None


def uav_from_dir(dirname):
    m = re.search(r"typhoon_h480_(\d+)", dirname)
    if m:
        return "uav_%d" % (int(m.group(1)) + 1)
    return None


def build_originals(round_dir):
    """Unique original-image list with aircraft id, time, path, SHA, source."""
    entries = []
    rejected = []
    fj = os.path.join(round_dir, "observers", "frames", "frames.jsonl")
    if os.path.isfile(fj):
        for r in load_jsonl(fj):
            o = r.get("observation", {})
            name = r.get("image", "")
            uav = o.get("uav_id")
            t = r.get("image_sample_s", o.get("sample_s"))
            full = os.path.join(round_dir, "observers", "frames", name)
            ok = bool(name) and os.path.isfile(full)
            entries.append(dict(uav=uav, t=float(t) if t is not None else None,
                                path=full if ok else "",
                                sha=sha256_file(full) if ok else "",
                                exists=ok, source="observers/frames/frames.jsonl",
                                id_source="index.uav_id", conflict=None,
                                png=name))
    alg = os.path.join(round_dir, "flight", "algorithm")
    if os.path.isdir(alg):
        for d in sorted(os.listdir(alg)):
            if not d.startswith("evidence_"):
                continue
            idx = os.path.join(alg, d, "index.jsonl")
            if not os.path.isfile(idx):
                continue
            dir_uav = uav_from_dir(d)
            for row in load_jsonl(idx):
                cb = row.get("camera_binding") or {}
                bind_uav = uav_from_binding(cb)
                if bind_uav and dir_uav and bind_uav != dir_uav:
                    rejected.append(dict(channel=d, png=row.get("png"),
                                         reason="BINDING_DIR_CONFLICT",
                                         binding=bind_uav, directory=dir_uav))
                    continue
                uav = bind_uav or dir_uav
                id_source = ("binding.image_topic" if bind_uav else
                             ("directory_fallback" if dir_uav else "unknown"))
                if uav is None:
                    rejected.append(dict(channel=d, png=row.get("png"),
                                         reason="NO_AIRCRAFT_ID"))
                    continue
                png = row.get("png", "")
                full = os.path.join(alg, d, png)
                ok = bool(png) and os.path.isfile(full)
                entries.append(dict(uav=uav, t=row.get("image_stamp"),
                                    path=full if ok else "", sha="",
                                    exists=ok, source="flight/algorithm/" + d,
                                    id_source=id_source, conflict=None, png=png,
                                    cls=row.get("cls"),
                                    color_conf=row.get("color_conf"),
                                    reject_reason=row.get("reject_reason"),
                                    person_iou=row.get("person_iou"),
                                    n_person_boxes=row.get("n_person_boxes"),
                                    run_id=row.get("run_id")))
    return entries, rejected


def dedupe_originals(entries):
    """Unique by (uav, rounded image time); keep all records per unique key."""
    uniq = {}
    for e in entries:
        if e["t"] is None:
            continue
        key = (e["uav"], round(float(e["t"]), 3))
        uniq.setdefault(key, []).append(e)
    return uniq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--round", required=True)
    ap.add_argument("--json-out", default="")
    ap.add_argument("--old-json", default="")
    ap.add_argument("--truth-tol", type=float, default=0.06)
    args = ap.parse_args()

    R = args.round
    truth_all = load_jsonl(os.path.join(R, "observers",
                                        "independent_visual_accuracy_white.jsonl"))
    truth = []
    dropped_kind = 0
    for r in truth_all:
        if (r.get("kind") or "") != "actor_truth":
            dropped_kind += 1
            continue
        s = r.get("sample")
        if not (isinstance(s, (list, tuple)) and len(s) >= 4):
            dropped_kind += 1
            continue
        if int(s[3]) != WHITE_ACTOR_ID:
            dropped_kind += 1
            continue
        truth.append((float(s[0]), float(s[1]), float(s[2]), int(s[3])))
    truth.sort(key=lambda x: x[0])

    cams = load_jsonl(os.path.join(R, "flight", "algorithm",
                                   "search_camera_frames.jsonl"))
    cam_uniq = {}
    for c in cams:
        cam_uniq.setdefault((c["uav_id"], round(float(c["image_s"]), 3)), c)

    entries, rejected = build_originals(R)
    uniq = dedupe_originals(entries)
    files_present = [e for e in entries if e["exists"]]

    print("=== input indices ===")
    print("truth rows read            : %d (dropped non actor_truth / non-id-%d: %d)"
          % (len(truth_all), WHITE_ACTOR_ID, dropped_kind))
    print("search camera records      : %d   unique (uav,image_s): %d   duplicates: %d"
          % (len(cams), len(cam_uniq), len(cams) - len(cam_uniq)))
    print("original files present     : %d   unique (uav,time): %d   duplicate records: %d"
          % (len(files_present), len(uniq), len(entries) - len(uniq)))
    print("rejected original records  : %d" % len(rejected))
    for r in rejected[:6]:
        print("   %s" % r)

    def nearest(rows, t, tol):
        best, bd = None, None
        lo, hi = 0, len(rows) - 1
        while lo <= hi:
            mid = (lo + hi) // 2
            dt = rows[mid][0] - t
            if bd is None or abs(dt) < bd:
                bd, best = abs(dt), rows[mid]
            if dt < 0:
                lo = mid + 1
            else:
                hi = mid - 1
        return best if (best is not None and bd <= tol) else None

    def project(cam, x, y, z, fx, fy, cx, cy):
        Rm = np.asarray(cam["camera_rotation"], float).reshape(3, 3)
        d = M_OPT2LINK.T @ Rm.T @ (np.array([x, y, z], float)
                                   - np.asarray(cam["camera_xyz"], float))
        if abs(d[2]) < 1e-9:
            return float(d[2]), None, None
        return float(d[2]), fx * float(d[0]) / d[2] + cx, fy * float(d[1]) / d[2] + cy

    rows = []
    for (uav, t_img), saved in sorted(uniq.items()):
        c = cam_uniq.get((uav, t_img))
        if c is None:
            # search record with the same key does not exist -> original belongs
            # to a non-search instant
            rows.append(dict(uav=uav, t_image=t_img, path=saved[0]["path"],
                             sha=saved[0]["sha"], id_source=saved[0]["id_source"],
                             in_search_index=False, in_fov_0875=None,
                             in_fov_100=None, reason="not_in_search_index"))
            continue
        tr = nearest(truth, t_img, args.truth_tol)
        fx, fy = DEFAULT_FX, DEFAULT_FX
        cx, cy = DEFAULT_CX, DEFAULT_CY
        intr = c.get("intrinsics")
        if isinstance(intr, (list, tuple)) and len(intr) == 4:
            fx, fy, cx, cy = [float(v) for v in intr]
        size = c.get("size") or [DEFAULT_W, DEFAULT_H]
        W, H = int(size[0]), int(size[1])
        rec = dict(uav=uav, t_image=t_img, t_sample=float(c.get("sample_s", 0.0)),
                   path=saved[0]["path"], sha=saved[0]["sha"],
                   id_source=saved[0]["id_source"], in_search_index=True,
                   truth_time=(tr[0] if tr else None),
                   truth_dt=(round(t_img - tr[0], 4) if tr else None))
        if tr is None:
            rec.update(in_fov_0875=False, in_fov_100=False, reason="no_truth_in_tol")
            rows.append(rec)
            continue
        _, tx, ty, _aid = tr
        rec["truth_xy"] = [tx, ty]
        rec["horiz_m"] = math.hypot(tx - float(c["camera_xyz"][0]),
                                    ty - float(c["camera_xyz"][1]))
        for label, z in (("0875", GROUND_Z + PERSON_M * 0.5),
                         ("100", GROUND_Z + 1.0)):
            depth, u, v = project(c, tx, ty, z, fx, fy, cx, cy)
            inside = bool(depth > 0 and u is not None
                          and 0 <= u < W and 0 <= v < H)
            rec["in_fov_%s" % label] = inside
            rec["depth_%s" % label] = round(depth, 3)
            rec["uv_%s" % label] = [round(u, 1) if u is not None else None,
                                    round(v, 1) if v is not None else None]
            rec["exp_h_px_%s" % label] = (round(PERSON_M * fx / depth, 2)
                                          if depth > 0 else None)
        rec["reason"] = ("in_fov" if (rec["in_fov_0875"] or rec["in_fov_100"])
                         else "outside_or_behind")
        rows.append(rec)

    in_search = [r for r in rows if r.get("in_search_index")]
    fov875 = [r for r in in_search if r.get("in_fov_0875")]
    fov100 = [r for r in in_search if r.get("in_fov_100")]
    both = [r for r in fov875 if r.get("in_fov_100")]
    only875 = [r for r in fov875 if not r.get("in_fov_100")]
    only100 = [r for r in fov100 if not r.get("in_fov_0875")]

    print("\n=== exact-original originals that ARE search records ===")
    print("records                    : %d" % len(in_search))
    print("in-FOV @ z=0.875 (midpoint): %d" % len(fov875))
    print("in-FOV @ z=1.00            : %d" % len(fov100))
    print("both                       : %d" % len(both))
    print("only @0.875                : %d" % len(only875))
    print("only @1.00                 : %d" % len(only100))
    print("\n--- in-FOV @0.875 list ---")
    for r in sorted(fov875, key=lambda x: (x["uav"], x["t_image"])):
        print("  %-6s t=%.3f dt=%+.4f dist=%6.2f m exp_h=%5.2f uv=(%6.1f,%5.1f) %s"
              % (r["uav"], r["t_image"], r.get("truth_dt") or 0.0, r.get("horiz_m", -1),
                 r.get("exp_h_px_0875") or -1,
                 (r["uv_0875"][0] if r.get("uv_0875") else -1),
                 (r["uv_0875"][1] if r.get("uv_0875") else -1),
                 os.path.basename(r["path"])))

    # set comparison against the previous tool's result
    if args.old_json and os.path.isfile(args.old_json):
        old = json.load(open(args.old_json, encoding="utf-8"))
        old_keys = {(f["uav"], round(float(f["t_image"]), 3))
                    for f in old.get("frames", [])
                    if f.get("in_fov") and f.get("saved_exact")}
        new_keys = {(r["uav"], round(r["t_image"], 3)) for r in fov875}
        print("\n=== set comparison: previous tool (in_fov & saved_exact) vs v3 ===")
        print("old only : %s" % sorted(old_keys - new_keys))
        print("new only : %s" % sorted(new_keys - old_keys))
        print("common   : %d" % len(old_keys & new_keys))

    if args.json_out:
        os.makedirs(os.path.dirname(args.json_out) or ".", exist_ok=True)
        json.dump(dict(
            version=3,
            fixes="evidence aircraft id via camera_binding.image_topic with directory "
                  "cross-check; actor_truth only, sample[3]==3; assumptions: ground "
                  "z=0 and 1.75 m reference height; image time = image_s",
            assumptions=dict(ground_z=GROUND_Z, person_m=PERSON_M),
            counts=dict(truth_rows=len(truth), truth_dropped=dropped_kind,
                        search_records=len(cams), search_unique=len(cam_uniq),
                        original_records=len(entries), original_files=len(files_present),
                        original_unique=len(uniq), rejected=len(rejected),
                        in_search=len(in_search), in_fov_0875=len(fov875),
                        in_fov_100=len(fov100), both=len(both),
                        only_0875=len(only875), only_100=len(only100)),
            rejected=rejected, frames=rows), open(args.json_out, "w", encoding="utf-8"),
            ensure_ascii=False, indent=1)
        print("\nwrote %s" % args.json_out)
    return 0


if __name__ == "__main__":
    sys.exit(main())