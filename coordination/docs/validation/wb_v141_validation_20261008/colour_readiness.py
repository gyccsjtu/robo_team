#!/usr/bin/env python3
"""Per-colour: why only green ever reaches the official channel?

For each colour, the readiness gate needs the same aircraft to observe the
target inside 12 m for >=3 frames. This compares, per colour, how often the
target was seen at all and how close it ever got, plus what the official
observer received.

Run: python3 colour_readiness.py --round <round_dir>
"""
import argparse
import glob
import json
import math
import os
import sys

COLOURS = ["green", "blue", "brown", "white", "red"]


def load(path):
    rows = []
    if not os.path.isfile(path):
        return rows
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except Exception:
                    pass
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--round", required=True)
    args = ap.parse_args()
    rnd = args.round.rstrip("/")
    obs_dir = os.path.join(rnd, "observers")

    print("%-7s %8s %8s %8s %8s %9s %10s %10s"
          % ("colour", "truth", "obs", "official", "min_dist", "p50_dist", "n<=12m", "n<=22m"))
    for c in COLOURS:
        name = ("independent_visual_accuracy" if c == "green"
                else "independent_visual_accuracy_" + c)
        rows = load(os.path.join(obs_dir, name + ".jsonl"))
        if not rows:
            print("%-7s  (no observer file)" % c)
            continue
        truth = sum(1 for r in rows if r.get("kind") == "actor_truth")
        official = sum(1 for r in rows if r.get("kind") == "official")
        imgs = [r["sample"] for r in rows if r.get("kind") == "image"
                and isinstance(r.get("sample"), dict)]
        dists = []
        for s in imgs:
            cam = s.get("camera_xyz")
            xyz = s.get("xyz")
            if isinstance(cam, (list, tuple)) and isinstance(xyz, (list, tuple)):
                dists.append(math.hypot(xyz[0] - cam[0], xyz[1] - cam[1]))
        dists.sort()
        print("%-7s %8d %8d %8d %9s %10s %10d %10d"
              % (c, truth, len(imgs), official,
                 ("%.2f" % dists[0]) if dists else "-",
                 ("%.2f" % dists[len(dists) // 2]) if dists else "-",
                 sum(1 for d in dists if d <= 12.0),
                 sum(1 for d in dists if d <= 22.0)))

    print()
    print("readiness first-time gate = 12 m (needs >=3 frames inside it from ONE aircraft)")
    print()
    print("=== official messages actually received, per colour observer ===")
    for c in COLOURS:
        name = ("independent_visual_accuracy" if c == "green"
                else "independent_visual_accuracy_" + c)
        p = os.path.join(obs_dir, name + ".jsonl")
        if not os.path.isfile(p):
            continue
        offs = [r for r in load(p) if r.get("kind") == "official"]
        if offs:
            s = offs[0]["sample"]
            print("  %-6s %d official, first t=%.2f xy=(%.2f, %.2f)"
                  % (c, len(offs), s[0], s[1], s[2]))
        else:
            print("  %-6s 0 official" % c)

    print()
    print("=== bridge-side per-tag outcome from the trace ===")
    tr = load(os.path.join(rnd, "flight", "algorithm", "bridge_trace.jsonl"))
    import collections
    up = collections.Counter(r.get("tag") for r in tr if r.get("kind") == "upload")
    sk = collections.Counter((r.get("tag"), r.get("reason")) for r in tr if r.get("kind") == "skip")
    inp = collections.Counter((r["observation"].get("target_id")) for r in tr
                              if r.get("kind") == "input")
    nc = collections.Counter((r.get("observation") or {}).get("target_id") for r in tr
                             if r.get("kind") == "navigation_candidate")
    print("  inputs        : %s" % dict(inp))
    print("  navigation_cand: %s" % dict(nc))
    print("  uploads       : %s" % dict(up))
    print("  skips         : %s" % dict(sk))
    return 0


if __name__ == "__main__":
    sys.exit(main())
