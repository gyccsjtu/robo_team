#!/usr/bin/env python3
"""Is perception's own 1 s frame gate already silently dropping frames?

perception_real.py processes a frame only when `0 <= now - frame_stamp <= 1.`
frame_probe_*.csv records every frame the camera produced, including its age, so
the drop rate caused by latency is directly countable there - and those frames
never reach the bridge, so they are invisible in the bridge trace.

Run: python3 frame_gate_check.py --round <round_dir>
"""
import argparse
import collections
import csv
import glob
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--round", required=True)
    ap.add_argument("--limit", type=float, default=1.0)
    args = ap.parse_args()
    alg = os.path.join(args.round.rstrip("/"), "flight", "algorithm")
    paths = sorted(glob.glob(os.path.join(alg, "frame_probe_typhoon_h480_*.csv")))
    print("frame_probe files: %d" % len(paths))
    if not paths:
        print("no frame probe data in this round")
        return 1

    total = over = nodata = 0
    ages = []
    per_uav = collections.Counter()
    header = None
    for p in paths:
        uav = os.path.basename(p).replace("frame_probe_", "").replace(".csv", "")
        with open(p, encoding="utf-8", errors="replace", newline="") as fh:
            reader = csv.DictReader(fh)
            header = reader.fieldnames
            for row in reader:
                total += 1
                raw = row.get("image_age_s") or row.get("age_s") or ""
                try:
                    a = float(raw)
                except ValueError:
                    nodata += 1
                    continue
                ages.append(a)
                if a > args.limit:
                    over += 1
                    per_uav[uav] += 1
    print("frame_probe columns: %s" % header)
    print("rows=%d  with age=%d  no age field=%d" % (total, len(ages), nodata))
    if not ages:
        print("age column empty; cannot judge")
        return 1
    ages.sort()
    print("age at processing: mean=%.3f p50=%.3f p90=%.3f p99=%.3f max=%.3f"
          % (sum(ages) / len(ages), ages[len(ages) // 2],
             ages[min(len(ages) - 1, int(len(ages) * .9))],
             ages[min(len(ages) - 1, int(len(ages) * .99))], ages[-1]))
    print()
    print("gate: age > %.2f s  => frame cannot be processed (perception's own limit)"
          % args.limit)
    print("  WOULD BE DROPPED: %d / %d  (%.2f%%)" % (over, len(ages),
                                                    100.0 * over / len(ages)))
    if per_uav:
        print("  per aircraft: %s" % dict(per_uav))
    return 0


if __name__ == "__main__":
    sys.exit(main())
