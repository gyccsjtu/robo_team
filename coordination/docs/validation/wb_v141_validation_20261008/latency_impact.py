#!/usr/bin/env python3
"""Why image latency damages BOTH continuity and accuracy.

Continuity side (measurable here):
  ReportReadiness.observe() only counts an observation if it is still fresh at
  receipt: `0 <= now - stamp <= 1.`  With image latency mean 0.79 s (p90 0.95,
  max 1.00) a large share of observations arrive already outside that window,
  and the readiness counter resets unless consecutive observations of the same
  (target, aircraft) are <= 1 s apart. Both are the same 0.79 s being spent.

Accuracy side:
  The published point is the fused position extrapolated forward. The distance
  it must cover is speed x staleness; the cap EXTRAP_MAX_D=1.5 m truncates it.

Run: python3 latency_impact.py --trace <trace.jsonl> --speed 2.0
"""
import argparse
import collections
import json
import math
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


def pct(v, q):
    s = sorted(v)
    return s[min(len(s) - 1, int(len(s) * q))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", required=True)
    ap.add_argument("--speed", type=float, default=2.0)
    ap.add_argument("--fresh-limit", type=float, default=1.0)
    ap.add_argument("--extrap-max-d", type=float, default=1.5)
    ap.add_argument("--obs-window", type=float, default=0.6)
    args = ap.parse_args()
    rows = load(args.trace)

    inp = [r for r in rows if r.get("kind") == "input" and r.get("accepted")]
    ups = [r for r in rows if r.get("kind") == "upload"]
    print("accepted observations: %d   uploads: %d" % (len(inp), len(ups)))

    # ---------- continuity side ----------
    ages = [float(r["receipt_s"]) - float(r["observation"]["sample_s"]) for r in inp]
    print()
    print("=== continuity: is an observation still fresh when it arrives? ===")
    print("  receipt - stamp      mean=%.3f p50=%.3f p90=%.3f max=%.3f"
          % (sum(ages) / len(ages), pct(ages, .5), pct(ages, .9), max(ages)))
    dead = [a for a in ages if a > args.fresh_limit]
    print("  readiness freshness limit = %.2f s" % args.fresh_limit)
    print("  DEAD ON ARRIVAL      %d / %d  (%.1f%%)  -> cannot count toward readiness"
          % (len(dead), len(ages), 100.0 * len(dead) / len(ages)))
    margin = [args.fresh_limit - a for a in ages]
    print("  headroom left        mean=%.3f s p10=%.3f s  (how much jitter it tolerates)"
          % (sum(margin) / len(margin), pct(margin, .1)))

    # per (tag, uav) intervals: a gap > 1 s resets ReportReadiness' counter
    seq = collections.defaultdict(list)
    for r in inp:
        o = r["observation"]
        seq[(o["target_id"], o["uav_id"])].append(float(o["sample_s"]))
    gaps = []
    for key, ts in seq.items():
        ts.sort()
        for a, b in zip(ts, ts[1:]):
            gaps.append(b - a)
    if gaps:
        brk = [g for g in gaps if g > 1.0]
        print()
        print("  same-(target,aircraft) observation intervals n=%d" % len(gaps))
        print("    mean=%.3f p50=%.3f p90=%.3f max=%.3f"
              % (sum(gaps) / len(gaps), pct(gaps, .5), pct(gaps, .9), max(gaps)))
        print("    > 1 s  -> readiness count RESET : %d / %d (%.1f%%)"
              % (len(brk), len(gaps), 100.0 * len(brk) / len(gaps)))
        print("    a reset costs 3 fresh observations inside 12 m to recover")

    # ---------- accuracy side, as arithmetic on measured lateness ----------
    if ups:
        tot = []
        for r in ups:
            tot.append(float(r["receipt_s"]) - float(r.get("position_s") or r["receipt_s"]))
        print()
        print("=== accuracy: what the staleness costs at %.1f m/s ===" % args.speed)
        print("  staleness (position_s -> receipt) mean=%.3f p90=%.3f max=%.3f s"
              % (sum(tot) / len(tot), pct(tot, .9), max(tot)))
        need = [args.speed * t for t in tot]
        over = [max(0.0, n - args.extrap_max_d) for n in need]
        print("  extrapolation needed mean=%.3f m ; cap=%.1f m" % (sum(need) / len(need),
                                                                  args.extrap_max_d))
        print("  capped shortfall     mean=%.3f m (%.0f%% of uploads hit the cap)"
              % (sum(over) / len(over),
                 100.0 * sum(1 for n in need if n >= args.extrap_max_d) / len(need)))
        print()
        print("  --- projection, not a measurement: if image latency and window")
        print("      smear both halve (total staleness %.3f -> %.3f s) ---"
              % (sum(tot) / len(tot), sum(tot) / len(tot) / 2))
        half = [t / 2 for t in tot]
        need2 = [args.speed * t for t in half]
        over2 = [max(0.0, n - args.extrap_max_d) for n in need2]
        print("      needed mean=%.3f m ; still capped: %.0f%% ; shortfall mean=%.3f m"
              % (sum(need2) / len(need2),
                 100.0 * sum(1 for n in need2 if n >= args.extrap_max_d) / len(need2),
                 sum(over2) / len(over2)))
        print("      -> the cap stops binding, so EXTRAP_MAX_D would not need touching")
    return 0


if __name__ == "__main__":
    sys.exit(main())
