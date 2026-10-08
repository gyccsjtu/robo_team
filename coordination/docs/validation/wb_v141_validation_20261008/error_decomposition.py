#!/usr/bin/env python3
"""Separate detector error from latency error in what we publish.

For every upload the bridge wrote we compare three vectors against actor truth:
  asked   : requested_xy  (fused + extrapolation) at receipt time -> what the
            judge actually scores
  fused   : fused_xy at receipt time  (no extrapolation)
  at_orig : fused_xy at the observation's own original_s -> pure detector error

If at_orig is small and asked is large, the problem is time, not detection.
"""
import argparse
import bisect
import json
import math
import os
import sys


def load(path):
    out = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except Exception:
                    pass
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--round", required=True)
    ap.add_argument("--trace", required=True)
    args = ap.parse_args()
    obs = os.path.join(args.round.rstrip("/"), "observers",
                       "independent_visual_accuracy.jsonl")
    truth = sorted([(float(r["sample"][0]), float(r["sample"][1]), float(r["sample"][2]))
                    for r in load(obs) if r.get("kind") == "actor_truth"])
    ts = [t[0] for t in truth]

    def truth_at(t):
        i = bisect.bisect_left(ts, t)
        if i == 0 or i >= len(truth):
            return None
        a, b = truth[i - 1], truth[i]
        gap = b[0] - a[0]
        if not 0 < gap <= 0.2:
            return None
        f = (t - a[0]) / gap
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]))

    rows = [r for r in load(args.trace) if r.get("kind") == "upload"]
    print("uploads: %d   truth samples: %d" % (len(rows), len(truth)))
    print()
    print("%-9s %-6s %-7s %-8s %-8s %-8s %-8s %s"
          % ("t", "tag", "age_s", "asked", "fused", "at_orig", "comp_m", "note"))
    agg = dict(asked=[], fused=[], at_orig=[], comp=[])
    for r in rows:
        t = float(r["receipt_s"])
        orig = float(r.get("original_s") or 0.0)
        asked = r.get("requested_xy") or []
        fused = r.get("fused_xy") or []
        if len(asked) != 2 or len(fused) != 2:
            continue
        tr_t, tr_o = truth_at(t), truth_at(orig) if orig > 0 else None
        if not tr_t or not tr_o:
            continue
        e_asked = math.hypot(asked[0] - tr_t[0], asked[1] - tr_t[1])
        e_fused = math.hypot(fused[0] - tr_t[0], fused[1] - tr_t[1])
        e_orig = math.hypot(fused[0] - tr_o[0], fused[1] - tr_o[1])
        comp = math.hypot(asked[0] - fused[0], asked[1] - fused[1])
        needed = math.hypot(tr_t[0] - tr_o[0], tr_t[1] - tr_o[1])
        age = t - orig
        agg["asked"].append(e_asked)
        agg["fused"].append(e_fused)
        agg["at_orig"].append(e_orig)
        agg["comp"].append(comp)
        agg.setdefault("needed", []).append(needed)
        note = []
        if e_asked > 1.0:
            note.append("JUDGE WOULD RESET")
        if comp < 0.05 and age > 0.3:
            note.append("no compensation")
        print("%-9.3f %-6s %-7.3f %-8.3f %-8.3f %-8.3f %-8.3f %s"
              % (t, r.get("tag"), age, e_asked, e_fused, e_orig, comp, ", ".join(note)))

    def stat(v):
        if not v:
            return "n/a"
        s = sorted(v)
        return ("n=%d mean=%.3f p50=%.3f p90=%.3f max=%.3f frac<1m=%.1f%%"
                % (len(s), sum(s) / len(s), s[len(s) // 2],
                   s[min(len(s) - 1, int(len(s) * .9))], s[-1],
                   100.0 * sum(1 for x in s if x < 1.0) / len(s)))

    print()
    print("asked   (judge sees) : %s" % stat(agg["asked"]))
    print("fused   (no extrap)  : %s" % stat(agg["fused"]))
    print("at_orig (detector)   : %s" % stat(agg["at_orig"]))
    print("compensation size    : %s" % stat(agg["comp"]))
    print("NEEDED displacement  : %s" % stat(agg.get("needed", [])))
    nd, cp = agg.get("needed", []), agg["comp"]
    if nd:
        short = [n - c for n, c in zip(nd, cp)]
        print("under-compensation   : mean=%.3f p50=%.3f max=%.3f  (how much of the"
              " actor's motion we failed to cover)" % (
                  sum(short) / len(short), sorted(short)[len(short) // 2], max(short)))
        print("uploads where the compensation hit the EXTRAP_MAX_D=1.5 m cap: %d/%d"
              % (sum(1 for c in cp if c >= 1.499), len(cp)))
    print()
    print("mean observation age at upload: %.3f s"
          % (sum(float(r["receipt_s"]) - float(r.get("original_s") or 0)
                 for r in rows) / max(1, len(rows))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
