#!/usr/bin/env python3
"""Codex review follow-ups, pass 2: aircraft travel + what targets they saw."""
import collections
import csv
import glob
import json
import math
import os

R = "/root/robocup_runs/wb_v141_validation3_20261008T104200Z/round_1_seed_8159"

print("=" * 72)
print("E) aircraft travel distance (Codex: the three zero-publish aircraft moved"
      " ~54-64 m, i.e. they were searching)")
print("=" * 72)
rows = [json.loads(l) for l in open(R + "/flight/city_trajectory.jsonl", encoding="utf-8") if l.strip()]
print("trajectory rows=%d  span %.1f -> %.1f s" % (len(rows), rows[0]["sample_s"], rows[-1]["sample_s"]))
uavs = sorted(rows[0]["positions"].keys())
dist = {u: 0.0 for u in uavs}
maxr = {u: 0.0 for u in uavs}
for i in range(len(rows) - 1):
    a, b = rows[i]["positions"], rows[i + 1]["positions"]
    for u in uavs:
        dist[u] += math.dist(a[u], b[u])
for u in uavs:
    pts = [r["positions"][u] for r in rows]
    maxr[u] = max(math.hypot(p[0], p[1]) for p in pts)
    zs = [p[2] for p in pts]
    print("  %-6s travelled %6.1f m   max_radius %5.1f m   z %.2f..%.2f"
          % (u, dist[u], maxr[u], min(zs), max(zs)))

print()
print("=" * 72)
print("F) what each aircraft actually looked at (target_id distribution)")
print("=" * 72)
for f in sorted(glob.glob(os.path.join(R, "flight", "algorithm", "perception_*.csv"))):
    name = os.path.basename(f)
    with open(f, newline="") as fh:
        rr = list(csv.DictReader(fh))
    if not rr:
        continue
    tag = name.replace("perception_typhoon_h480_", "h480_").replace(".csv", "")
    print("%s" % tag)
    for evt in ("yolo_detection", "track_reject", "track_publish"):
        sel = [r for r in rr if r.get("event") == evt]
        if not sel:
            continue
        key = "target_id" if sel[0].get("target_id") else "class_name"
        c = collections.Counter((r.get(key) or "?") for r in sel)
        print("    %-14s n=%-5d %s" % (evt, len(sel), dict(c.most_common(6))))
    # distances seen
    rng = [float(r["range_m"]) for r in rr
           if r.get("event") == "yolo_detection" and r.get("range_m")]
    if rng:
        s = sorted(rng)
        print("    detection range_m  p50=%.1f min=%.1f max=%.1f"
              % (s[len(s) // 2], s[0], s[-1]))

print()
print("=" * 72)
print("G) for the three zero-publish aircraft: was blue the ONLY thing they saw?")
print("=" * 72)
for f in sorted(glob.glob(os.path.join(R, "flight", "algorithm", "perception_*.csv"))):
    name = os.path.basename(f)
    with open(f, newline="") as fh:
        rr = list(csv.DictReader(fh))
    pub = sum(1 for r in rr if r.get("event") == "track_publish")
    if pub:
        continue
    det = [r for r in rr if r.get("event") == "yolo_detection"]
    c = collections.Counter((r.get("target_id") or "?") for r in det)
    print("  %-12s detections=%d  by target: %s" % (name, len(det), dict(c)))
