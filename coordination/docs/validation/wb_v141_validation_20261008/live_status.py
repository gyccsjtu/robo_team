#!/usr/bin/env python3
"""Live snapshot of the running validation round (read-only, cheap).

Prints one compact analysis block per call and keeps the previous snapshot in
/tmp/wb_live_state.json so it can also report the sim-time rate (RTF) between
calls and the per-stream deltas.

Run:  python3 live_status.py --root <root>
"""
import argparse
import collections
import json
import math
import os
import sys
import time

STATE = "/tmp/wb_live_state.json"


def load_jsonl(path, tail_bytes=None):
    """Parse JSONL. With tail_bytes, read only the tail (cheap for big files)."""
    rows = []
    if not os.path.isfile(path):
        return rows
    try:
        if tail_bytes:
            size = os.path.getsize(path)
            with open(path, "rb") as fh:
                if size > tail_bytes:
                    fh.seek(size - tail_bytes)
                    fh.readline()
                data = fh.read().decode("utf-8", "replace")
        else:
            data = open(path, encoding="utf-8", errors="replace").read()
        for line in data.splitlines():
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except Exception:
                    pass
    except Exception:
        pass
    return rows


def count_lines(path):
    if not os.path.isfile(path):
        return 0
    n = 0
    with open(path, "rb") as fh:
        for _ in fh:
            n += 1
    return n


def kind_counts(path):
    """Full-file kind histogram. Never use a tail window for cumulative counts
    (an earlier version did, which made 'image: 19 -> 1' look like a regression)."""
    c = collections.Counter()
    if not os.path.isfile(path):
        return c
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                c[json.loads(line).get("kind")] += 1
            except Exception:
                pass
    return c


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", required=True)
    args = ap.parse_args()
    root = args.root.rstrip("/")
    rnd = None
    if os.path.isdir(root):
        for name in sorted(os.listdir(root)):
            if name.startswith("round_"):
                rnd = os.path.join(root, name)
    if rnd is None:
        print("no round dir under %s yet" % root)
        return 1
    flight = os.path.join(rnd, "flight")
    now = time.time()

    # ---- budget + runner ---------------------------------------------------
    status = wall = None
    try:
        d = json.load(open(os.path.join(root, "budget.json"), encoding="utf-8"))
        a = d["attempts"][-1]
        status, wall = a.get("status"), a.get("wall_seconds") or 0.0
    except Exception:
        pass
    alive = False
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            cmd = open("/proc/%s/cmdline" % pid, "rb").read().decode("utf-8", "replace")
        except Exception:
            continue
        if "validate_fast_city.py" in cmd and root in cmd:
            alive = True
            break

    # ---- trajectory --------------------------------------------------------
    traj_path = os.path.join(flight, "city_trajectory.jsonl")
    traj_n = count_lines(traj_path)
    last = {}
    if traj_n:
        last = load_jsonl(traj_path, tail_bytes=4000)[-1]
    sim_s = float(last.get("sample_s") or 0.0)
    pos = last.get("positions") or {}
    zs = {k: round(float(v[2]), 2) for k, v in pos.items()}

    prev = {}
    if os.path.isfile(STATE):
        try:
            prev = json.load(open(STATE, encoding="utf-8"))
        except Exception:
            prev = {}
    dt = now - prev.get("wall", now)
    rtf = (sim_s - prev.get("sim_s", sim_s)) / dt if dt > 1 and prev else None

    # ---- judge / events ----------------------------------------------------
    ev_path = os.path.join(flight, "city_events.jsonl")
    events = load_jsonl(ev_path, tail_bytes=400000)
    kinds = collections.Counter(r.get("kind") for r in events)
    prev_left = None
    deletions = {}
    for row in events:
        if row.get("kind") != "left_actors":
            continue
        cur = set(row.get("value") or [])
        if prev_left is not None:
            for gone in sorted(prev_left - cur):
                deletions.setdefault(gone, row.get("sample_s"))
        prev_left = cur
    visual = [r for r in events if r.get("kind") == "visual"]
    vis_targets = collections.Counter((r.get("value") or {}).get("target_id") for r in visual)
    blue_vis = [r for r in visual if (r.get("value") or {}).get("target_id") == "blue"]

    # ---- contacts ----------------------------------------------------------
    contacts = load_jsonl(os.path.join(flight, "city_contacts.log"), tail_bytes=800000)
    cat = collections.Counter()
    for row in contacts:
        if row.get("kind") != "UAV_BODY_CONTACT":
            continue
        low = ("%s %s" % (row.get("collision1"), row.get("collision2"))).lower()
        if "ground_plane" in low:
            cat["ground"] += 1
        elif low.count("typhoon_h480") >= 2:
            cat["uav_uav"] += 1
        elif "actor_" in low:
            cat["actor"] += 1
        else:
            cat["other"] += 1
    hb = [r for r in contacts if r.get("kind") == "CONTACT_OBSERVER_HEARTBEAT"]

    # ---- authority ---------------------------------------------------------
    auth = load_jsonl(os.path.join(flight, "algorithm", "authority_events.jsonl"),
                      tail_bytes=400000)
    by_ev = collections.Counter(r.get("event") for r in auth)
    bridge_path = os.path.join(flight, "algorithm", "bridge_trace.jsonl")
    bridge_n = count_lines(bridge_path)

    # ---- observers --------------------------------------------------------
    obs_dir = os.path.join(rnd, "observers")
    obs = {}
    for name in ("independent_visual_accuracy", "independent_visual_accuracy_blue",
                 "independent_visual_accuracy_brown", "independent_visual_accuracy_white",
                 "independent_visual_accuracy_red"):
        p = os.path.join(obs_dir, name + ".jsonl")
        if os.path.isfile(p):
            k = kind_counts(p)
            obs[name.replace("independent_visual_accuracy", "obs") or "obs_green"] = {
                "lines": count_lines(p), "truth": k.get("actor_truth", 0),
                "image": k.get("image", 0), "official": k.get("official", 0)}
    frames_n = len(os.listdir(os.path.join(obs_dir, "frames"))) \
        if os.path.isdir(os.path.join(obs_dir, "frames")) else -1

    # ---- white evidence ----------------------------------------------------
    alg = os.path.join(flight, "algorithm")
    white_rows, near_white = 0, []
    chan_counts = {}
    for name in sorted(os.listdir(alg)) if os.path.isdir(alg) else []:
        if not name.startswith("evidence_"):
            continue
        rows = load_jsonl(os.path.join(alg, name, "index.jsonl"))
        chan_counts[name.replace("evidence_typhoon_h480_", "h")] = len(rows)
        for r in rows:
            if r.get("cls") == "white":
                white_rows += 1
                if isinstance(r.get("range_m"), (int, float)) and r["range_m"] <= 25:
                    near_white.append(dict(ch=name.replace("evidence_typhoon_h480_", "h"),
                                           range_m=r.get("range_m"),
                                           conf=r.get("color_conf"),
                                           iou=r.get("person_iou"),
                                           npb=r.get("n_person_boxes"),
                                           reason=r.get("reject_reason"),
                                           stamp=r.get("image_stamp")))
    wd = os.path.join(obs_dir, "white_dev")
    wd_rows = count_lines(os.path.join(wd, "index.jsonl")) if os.path.isdir(wd) else -1

    # ---- print -------------------------------------------------------------
    print("== LIVE %s ==" % time.strftime("%H:%M:%SZ", time.gmtime()))
    print("status=%s wall=%.0fs runner_alive=%s | sim_s=%.1f traj=%d rtf=%s"
          % (status, wall, alive, sim_s, traj_n,
             ("%.3f" % rtf) if rtf else "n/a"))
    print("z now: %s" % zs)
    print("judge: left=%s deleted=%s times=%s"
          % (sorted(prev_left or []), sorted(deletions), {str(k): round(v, 1) for k, v in deletions.items()}))
    print("visual: n=%d %s%s" % (len(visual), dict(vis_targets),
                                 ("  BLUE=%d" % len(blue_vis)) if blue_vis else "  BLUE=0"))
    print("contacts: %s  sonar_pairs=%s body_pairs=%s frames=%s last_s=%s"
          % (dict(cat), (hb[-1].get("sonar_virtual_pairs") if hb else None),
             (hb[-1].get("body_pairs") if hb else None),
             (hb[-1].get("frames") if hb else None),
             (hb[-1].get("last_sample_s") if hb else None)))
    print("authority: %s | bridge_inputs=%d" % (dict(by_ev), bridge_n))
    print("observers: %s frames=%d" % (json.dumps(obs, ensure_ascii=False), frames_n))
    print("white-evidence: white_rows=%d near<=25m=%d white_dev_rows=%s"
          % (white_rows, len(near_white), wd_rows))
    for r in near_white[-6:]:
        print("   %s range=%.2f conf=%.3f iou=%s npb=%s reason=%s stamp=%s"
              % (r["ch"], r["range_m"], r["conf"], r["iou"], r["npb"], r["reason"], r["stamp"]))
    print("channels: %s" % json.dumps(chan_counts))

    try:
        json.dump(dict(wall=now, sim_s=sim_s, status=status), open(STATE, "w"))
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
