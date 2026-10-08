#!/usr/bin/env python3
"""Method self-check + open-question hunt over the closed round.

Part A: is my error measurement trustworthy?
  - truth sample interval (drives interpolation error)
  - is there a higher-rate truth source to validate against?
Part B: what else is unexplained?
  - when did track_publish actually stop (is the silence also on the
    perception side, or only at the bridge?)
  - per-aircraft last event time, to see whether all six fell silent together
"""
import collections
import csv
import glob
import json
import os
import statistics

R = "/root/robocup_runs/wb_v141_validation3_20261008T104200Z/round_1_seed_8159"


def stat(v, name):
    if not v:
        print("  %-28s (none)" % name)
        return
    s = sorted(v)
    print("  %-28s n=%-6d p50=%.3f p90=%.3f max=%.3f"
          % (name, len(s), statistics.median(s),
             s[min(len(s) - 1, int(len(s) * 0.9))], s[-1]))


print("=" * 70)
print("PART A - is the error measurement trustworthy?")
print("=" * 70)

obs = os.path.join(R, "observers", "independent_visual_accuracy.jsonl")
rows = [json.loads(l) for l in open(obs, encoding="utf-8") if l.strip()]
kinds = collections.Counter(r.get("kind") for r in rows)
print("observer file kind counts:", dict(kinds))
tr = sorted(float(r["sample"][0]) for r in rows if r.get("kind") == "actor_truth")
gaps = [tr[i + 1] - tr[i] for i in range(len(tr) - 1)]
stat(gaps, "actor_truth sample gap (s)")
print("  -> interpolation is the right call if this is ~1 s and the"
      " walker moves ~1 m/s")

# higher-rate truth sources?
print("\npossible higher-rate truth sources:")
cands = [
    "flight/actor_environment_odometry.json",
    "flight/actor_environment_wiring.json",
]
for c in cands:
    p = os.path.join(R, c)
    print("  %-46s %s" % (c, "EXISTS" if os.path.isfile(p) else "-"))
acts = sorted(glob.glob(os.path.join(R, "flight", "actor_*.log")))
print("  actor logs:", [os.path.basename(a) for a in acts][:8])

# how many truth samples do we actually have, per actor
byactor = collections.Counter()
for r in rows:
    if r.get("kind") == "actor_truth":
        s = r["sample"]
        byactor[int(s[3]) if len(s) > 3 else 0] += 1
print("  truth samples per actor:", dict(byactor))

print()
print("=" * 70)
print("PART B - unexplained: when/where did the link go quiet?")
print("=" * 70)

for f in sorted(glob.glob(os.path.join(R, "flight", "algorithm", "perception_*.csv"))):
    with open(f, newline="") as fh:
        rr = list(csv.DictReader(fh))
    if not rr:
        continue
    ev = collections.Counter(r.get("event") for r in rr)
    tcol = "ros_time" if "ros_time" in rr[0] else list(rr[0].keys())[0]
    tp = [float(r[tcol]) for r in rr if r.get("event") == "track_publish" and r.get(tcol)]
    yd = [float(r[tcol]) for r in rr if r.get("event") == "yolo_detection" and r.get(tcol)]
    last = rr[-1]
    print("%-40s" % os.path.basename(f))
    print("   events=%s" % dict(list(ev.items())[:4]))
    if tp:
        print("   track_publish: n=%-4d first=%.1f last=%.1f" % (len(tp), min(tp), max(tp)))
    else:
        print("   track_publish: 0")
    if yd:
        print("   yolo_detection: n=%-4d first=%.1f last=%.1f" % (len(yd), min(yd), max(yd)))
    print("   LAST row: event=%s  %s=%s" % (last.get("event"), tcol, last.get(tcol)))

print()
print("=" * 70)
print("PART C - bridge skip timeline (is stale growing worse over time?)")
print("=" * 70)
bt = os.path.join(R, "flight", "algorithm", "bridge_trace.jsonl")
skips = []
ups = []
with open(bt, encoding="utf-8") as fh:
    for line in fh:
        line = line.strip()
        if not line:
            continue
        r = json.loads(line)
        if r.get("kind") == "skip":
            skips.append((float(r.get("receipt_s") or 0), r.get("reason"),
                          float(r.get("age_s") or 0) if r.get("age_s") is not None else None))
        elif r.get("kind") == "upload":
            ups.append(float(r.get("receipt_s") or 0))
print("skip rows recorded: %d   upload rows: %d" % (len(skips), len(ups)))
if skips:
    reasons = collections.Counter(s[1] for s in skips)
    print("skip reasons:", dict(reasons))
    ages = [s[2] for s in skips if s[2] is not None]
    stat(ages, "skip age_s (staleness)")
    # split early vs late half
    mid = (min(s[0] for s in skips) + max(s[0] for s in skips)) / 2.0
    for label, sel in (("early half", [s for s in skips if s[0] < mid]),
                       ("late half", [s for s in skips if s[0] >= mid])):
        a = [s[2] for s in sel if s[2] is not None]
        r = collections.Counter(s[1] for s in sel)
        print("  %-10s age p50=%s  reasons=%s"
              % (label, ("%.3f" % statistics.median(a)) if a else "-", dict(r)))
