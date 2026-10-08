#!/usr/bin/env python3
"""Follow-ups from the method self-check."""
import collections
import csv
import glob
import json
import os
import statistics

R = "/root/robocup_runs/wb_v141_validation3_20261008T104200Z/round_1_seed_8159"

print("=" * 70)
print("1) upload rate over sim time - silence, or just slower?")
print("=" * 70)
ups = []
with open(os.path.join(R, "flight", "algorithm", "bridge_trace.jsonl"), encoding="utf-8") as fh:
    for line in fh:
        line = line.strip()
        if not line:
            continue
        r = json.loads(line)
        if r.get("kind") == "upload":
            ups.append((float(r["receipt_s"]), r.get("tag")))
ups.sort()
t0, t1 = ups[0][0], ups[-1][0]
print("uploads=%d  span=%.1f..%.1f (%.1f sim s)" % (len(ups), t0, t1, t1 - t0))
bucket = 60.0
buckets = collections.defaultdict(collections.Counter)
for t, tag in ups:
    b = int((t - t0) // bucket)
    buckets[b][tag] += 1
for b in sorted(buckets):
    lo = t0 + b * bucket
    tot = sum(buckets[b].values())
    bar = "#" * min(60, tot)
    print("  sim %7.1f-%7.1f  n=%-4d %-22s %s"
          % (lo, lo + bucket, tot, dict(buckets[b]), bar))

print()
print("=" * 70)
print("2) why did three aircraft never publish at all?")
print("=" * 70)
for f in sorted(glob.glob(os.path.join(R, "flight", "algorithm", "perception_*.csv"))):
    name = os.path.basename(f)
    with open(f, newline="") as fh:
        rr = list(csv.DictReader(fh))
    if not rr:
        continue
    keys = rr[0].keys()
    pub = sum(1 for r in rr if r.get("event") == "track_publish")
    rej = [r for r in rr if r.get("event") == "track_reject"]
    det = [r for r in rr if r.get("event") == "yolo_detection"]
    # the verdict counters live on yolo_detection rows as their own events
    verd = collections.Counter()
    for r in rr:
        e = r.get("event") or ""
        if e.startswith("verdict"):
            verd[e] += 1
    print("%-36s publish=%-4d reject=%-5d detect=%-4d" % (name, pub, len(rej), len(det)))
    if verd:
        print("      verdict events: %s" % dict(verd))

print()
print("keys of a track_reject row:")
with open(sorted(glob.glob(os.path.join(R, "flight", "algorithm", "perception_*.csv")))[0],
          newline="") as fh:
    for r in csv.DictReader(fh):
        if r.get("event") == "track_reject":
            print("   ", {k: v for k, v in r.items() if v not in (None, "")})
            break

print()
print("=" * 70)
print("3) stale vs not_ready: split the skip ages by reason")
print("=" * 70)
byr = collections.defaultdict(list)
with open(os.path.join(R, "flight", "algorithm", "bridge_trace.jsonl"), encoding="utf-8") as fh:
    for line in fh:
        line = line.strip()
        if not line:
            continue
        r = json.loads(line)
        if r.get("kind") == "skip":
            byr[r.get("reason")].append((float(r.get("receipt_s") or 0),
                                         r.get("age_s")))
for reason, rows in byr.items():
    ages = [a for _, a in rows if a is not None]
    if ages:
        s = sorted(ages)
        print("  %-12s n=%-4d age p50=%.3f p90=%.3f max=%.3f   (COAST_TIME=1.5)"
              % (reason, len(ages), statistics.median(s),
                 s[min(len(s) - 1, int(len(s) * .9))], s[-1]))
    else:
        print("  %-12s n=%-4d (no age field)" % (reason, len(rows)))
