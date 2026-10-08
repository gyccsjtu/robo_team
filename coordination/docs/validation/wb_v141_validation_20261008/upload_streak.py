#!/usr/bin/env python3
"""Longest consecutive upload streak vs the judge's 15 s continuous rule."""
import json
import sys

path = sys.argv[1]
ups = []
for line in open(path, encoding="utf-8", errors="replace"):
    line = line.strip()
    if not line:
        continue
    r = json.loads(line)
    if r.get("kind") == "upload":
        ups.append((float(r["receipt_s"]), r.get("tag"), r.get("requested_xy"),
                    r.get("original_s")))
print("uploads: %d" % len(ups))
if not ups:
    raise SystemExit
for t, tag, xy, orig in ups:
    print("   t=%.3f tag=%-6s xy=%s original_s=%.3f" % (t, tag, xy, orig or 0))

# streak: consecutive uploads with gap <= 1.0 s (judge DETECTION_INTERVAL)
best = cur = 1
best_start = cur_start = ups[0][0]
prev = ups[0][0]
gap_max = 0.0
for t, *_ in ups[1:]:
    d = t - prev
    gap_max = max(gap_max, d)
    if d <= 1.0:
        cur += 1
    else:
        if cur > best:
            best, best_start = cur, cur_start
        cur, cur_start = 1, t
    prev = t
if cur > best:
    best, best_start = cur, cur_start
print()
print("longest streak with gap<=1.0 s: %d message(s), from t=%.3f" % (best, best_start))
print("largest gap between uploads   : %.3f s" % gap_max)
print("judge needs DETECTION_DURATION=15 s of continuous (<=1 s interval) reports")
print()
print("=> covered %.1f s of the 15 s the judge requires (%.0f%%)"
      % (best - 1 if best > 1 else 0.0, 100.0 * max(0, best - 1) / 15.0))
