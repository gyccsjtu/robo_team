#!/usr/bin/env python3
"""Where does the localisation latency come from?

Decomposes the staleness of the published coordinate into:
  A. fusion-window smear : the window averages observations of a MOVING actor,
                           so the fused point represents position_s, which is
                           EARLIER than the newest image (original_s)
  B. image latency       : original_s -> receipt_s (the newest original is already old)
  C. extrapolation cap   : EXTRAP_MAX_D caps how much of A+B we can compensate

Also measures cross-aircraft agreement at the same instant: if two aircraft see
the same actor at the same timestamp and disagree by ~0.2 m, the projection is
consistent and the residual error is time, not geometry.

Run: python3 latency_decomposition.py --trace <trace.jsonl>
"""
import argparse
import bisect
import json
import math
import os
import sys


def load(path):
    rows = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except Exception:
                    pass
    return rows


def stat(v, name):
    if not v:
        print("  %-34s n/a" % name)
        return
    s = sorted(v)
    print("  %-34s n=%-4d mean=%.3f p50=%.3f p90=%.3f max=%.3f"
          % (name, len(s), sum(s) / len(s), s[len(s) // 2],
             s[min(len(s) - 1, int(len(s) * .9))], s[-1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", required=True)
    ap.add_argument("--obs-window", type=float, default=0.6)
    ap.add_argument("--extrap-max-d", type=float, default=1.5)
    args = ap.parse_args()
    rows = load(args.trace)
    ups = [r for r in rows if r.get("kind") == "upload"]
    print("uploads with a window: %d" % sum(1 for r in ups if r.get("window")))

    span, img_lat, pos_lag, total_lag = [], [], [], []
    for r in ups:
        w = r.get("window") or []
        if len(w) < 1:
            continue
        ts = [float(e[0]) for e in w]
        span.append(max(ts) - min(ts))
        orig = float(r.get("original_s") or 0)
        pos = float(r.get("position_s") or 0)
        rec = float(r["receipt_s"])
        img_lat.append(rec - orig)          # newest original already this old
        pos_lag.append(orig - pos)          # fused point is behind the newest image
        total_lag.append(rec - pos)         # what extrapolation must cover

    print()
    print("=== staleness terms (seconds) ===")
    stat(span, "fusion window span")
    stat(img_lat, "image latency (orig->receipt)")
    stat(pos_lag, "fusion smear (position->orig)")
    stat(total_lag, "TOTAL (position->receipt)")

    print()
    print("=== cross-aircraft agreement at the SAME timestamp ===")
    dis = []
    for r in ups:
        w = r.get("window") or []
        groups = {}
        for e in w:
            groups.setdefault(round(float(e[0]), 3), []).append((float(e[1]), float(e[2])))
        for t, pts in groups.items():
            if len(pts) < 2:
                continue
            for i in range(len(pts)):
                for j in range(i + 1, len(pts)):
                    dis.append(math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1]))
    stat(dis, "same-instant cross-aircraft spread")
    print("  (0.1-0.3 m here means the projection geometry agrees; the residual is time)")

    if total_lag:
        print()
        mean_speed = 2.0
        print("=== implied under-compensation at ~%.1f m/s actor speed ===" % mean_speed)
        need = [mean_speed * tl for tl in total_lag]
        short = [max(0.0, n - args.extrap_max_d) for n in need]
        stat(need, "needed extrapolation distance")
        stat(short, "flat-capped shortfall (m)")
        print("  cap in use: EXTRAP_MAX_D = %.1f m; window option: OBS_WINDOW = %.1f s"
              % (args.extrap_max_d, args.obs_window))
    return 0


if __name__ == "__main__":
    sys.exit(main())
