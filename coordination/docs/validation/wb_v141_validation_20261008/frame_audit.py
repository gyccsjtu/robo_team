#!/usr/bin/env python3
"""Frame-level audit: where the latency goes, and how blind the search is.

frame_probe_*.csv records every frame the camera produced with the inference
duration, the raw detection count and class, ROI drops and geometry verdicts.
This is the bottom of the chain, so anything wrong here propagates everywhere.

Run: python3 frame_audit.py --round <round_dir>
"""
import argparse
import collections
import csv
import glob
import os
import statistics as st
import sys


def f(row, key, default=0.0):
    try:
        return float(row.get(key) or default)
    except (TypeError, ValueError):
        return default


def pct(v, q):
    if not v:
        return float("nan")
    s = sorted(v)
    return s[min(len(s) - 1, int(len(s) * q))]


def describe(name, v, unit=""):
    if not v:
        print("  %-30s none" % name)
        return
    print("  %-30s n=%-6d mean=%8.3f p50=%8.3f p90=%8.3f p99=%8.3f max=%8.3f %s"
          % (name, len(v), sum(v) / len(v), pct(v, .5), pct(v, .9), pct(v, .99),
             max(v), unit))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--round", required=True)
    args = ap.parse_args()
    alg = os.path.join(args.round.rstrip("/"), "flight", "algorithm")
    paths = sorted(glob.glob(os.path.join(alg, "frame_probe_typhoon_h480_*.csv")))
    if not paths:
        print("no frame probe")
        return 1

    age, infer, boxes = [], [], []
    cls_count = collections.Counter()
    frames_with_det = 0
    frames_total = 0
    roi_drop = 0
    geo_pass = 0
    geo_reject = 0
    shared_true = 0
    stamps = collections.defaultdict(list)
    per_uav_det = collections.Counter()
    per_uav_total = collections.Counter()
    zero_conf = 0
    big_conf = []

    for p in paths:
        uav = os.path.basename(p).replace("frame_probe_", "").replace(".csv", "")
        with open(p, encoding="utf-8", errors="replace", newline="") as fh:
            for row in csv.DictReader(fh):
                frames_total += 1
                per_uav_total[uav] += 1
                age.append(f(row, "image_age_s"))
                infer.append(f(row, "inference_s"))
                nb = int(f(row, "raw_boxes"))
                boxes.append(nb)
                if nb > 0:
                    frames_with_det += 1
                    per_uav_det[uav] += 1
                raw = row.get("raw_cls") or ""
                for part in raw.split(",") if raw else []:
                    if ":" in part:
                        k, _, n = part.partition(":")
                        try:
                            cls_count[k] += int(n)
                        except ValueError:
                            pass
                roi_drop += int(f(row, "roi_drop"))
                geo_pass += int(f(row, "geo_pass"))
                geo_reject += int(f(row, "geo_reject"))
                if str(row.get("shared")).strip() == "True":
                    shared_true += 1
                cm = f(row, "raw_conf_max")
                if cm > 0:
                    big_conf.append(cm)
                else:
                    zero_conf += 1
                try:
                    stamps[uav].append(float(row["image_stamp"]))
                except (KeyError, ValueError):
                    pass

    print("frames total: %d across %d aircraft" % (frames_total, len(paths)))
    print()
    print("=== latency: where the time is spent ===")
    describe("image_age_s (at processing)", age, "s")
    describe("inference_s", infer, "s")
    print("  inference share of age: %.1f%%" % (100.0 * (sum(infer) / len(infer)) /
                                               (sum(age) / len(age))))
    describe("raw_conf_max (when >0)", big_conf)
    print("  frames with no detection at all: %d (%.1f%%)"
          % (frames_total - frames_with_det,
             100.0 * (frames_total - frames_with_det) / frames_total))
    print("  shared inference used on %.1f%% of frames"
          % (100.0 * shared_true / frames_total))

    print()
    print("=== per-aircraft frame rate and detection rate ===")
    for uav in sorted(per_uav_total):
        ts = sorted(stamps.get(uav) or [])
        span = (ts[-1] - ts[0]) if len(ts) > 1 else 0.0
        hz = (len(ts) - 1) / span if span > 0 else 0.0
        print("  %-22s frames=%-5d span=%7.1fs  %5.2f Hz  detections=%d (%.1f%%)"
              % (uav, per_uav_total[uav], span, hz, per_uav_det[uav],
                 100.0 * per_uav_det[uav] / max(1, per_uav_total[uav])))

    print()
    print("=== what the colour model sees (raw detections by class) ===")
    for k, v in cls_count.most_common(12):
        print("  %-8s %d" % (k, v))
    print("  ROI drops=%d   geo_pass=%d   geo_reject=%d" % (roi_drop, geo_pass, geo_reject))

    print()
    print("=== frames per detection count ===")
    for k, v in sorted(collections.Counter(boxes).items())[:8]:
        print("  %d boxes : %d frames" % (k, v))
    return 0


if __name__ == "__main__":
    sys.exit(main())
