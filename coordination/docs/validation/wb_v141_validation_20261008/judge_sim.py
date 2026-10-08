#!/usr/bin/env python3
"""Replay the official judge's detection logic against a recorded round.

The judge (vendor/official_robocup/*/score_cal.py) counts a target as found
when, for one actor id, messages keep arriving with

    continuous = previous == 0.0 or now - previous <= DETECTION_INTERVAL (1.0 s)
    ok         = distance_sq < err_threshold ** 2                 (1.0 m)

and on ANY violated message it calls _reset_detection(), i.e. the 15 s window
restarts from scratch (count_flag=False).  'find actor_N' is printed only on the
FIRST ok message after a reset, so counting those prints counts resets.

This tool replays that state machine over what we actually published and
reports, per actor:
  - the longest run that would have survived (<= the 15 s the judge needs)
  - how many resets happened and WHY (error >= 1 m vs gap > 1 s)

Input: the observer's own '/actor_*_info' receipts (kind='official' = exactly
what the judge saw) plus its actor_truth stream for ground positions.

Read-only; no simulation, no ROS.
"""

import argparse
import bisect
import collections
import json
import math
import os
import sys

DETECTION_INTERVAL = 1.0     # score_cal.py
DETECTION_DURATION = 15.0    # score_cal.py
ERR_THRESHOLD = 1.0          # score_cal.py


def load_rows(path):
    rows = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except ValueError:
                continue
    return rows


def truth_by_actor(rows):
    """{"actor_id": (times, xs, ys)} sorted by time."""
    acc = collections.defaultdict(list)
    for r in rows:
        if r.get("kind") != "actor_truth":
            continue
        s = r.get("sample")
        if not isinstance(s, (list, tuple)) or len(s) < 3:
            continue
        actor = int(s[3]) if len(s) > 3 else 0
        acc[actor].append((float(s[0]), float(s[1]), float(s[2])))
    out = {}
    for actor, pts in acc.items():
        pts.sort()
        out[actor] = (([p[0] for p in pts], [p[1] for p in pts], [p[2] for p in pts]))
    return out


def truth_at(truth, t, tol=1.5):
    """Ground position at time t, LINEARLY INTERPOLATED between truth samples.

    Interpolation is required, not cosmetic: the observer samples actor truth
    about once per second and the walkers move 1-2 m in that time, so matching a
    message against the nearest stored sample injects up to ~1 m of bogus error -
    exactly the size of the judge's own threshold. Nearest-sample matching would
    have made our coordinates look far worse than they are.
    """
    times, xs, ys = truth
    if not times:
        return None
    i = bisect.bisect_left(times, t)
    if i <= 0:
        return (xs[0], ys[0]) if abs(times[0] - t) <= tol else None
    if i >= len(times):
        return (xs[-1], ys[-1]) if abs(times[-1] - t) <= tol else None
    t0, t1 = times[i - 1], times[i]
    if t1 <= t0:
        return (xs[i], ys[i]) if abs(times[i] - t) <= tol else None
    f = (t - t0) / (t1 - t0)
    if f < 0.0 or f > 1.0:              # beyond the stored pair: clip to nearest
        return (xs[i], ys[i]) if abs(times[i] - t) <= tol else None
    return (xs[i - 1] + f * (xs[i] - xs[i - 1]),
            ys[i - 1] + f * (ys[i] - ys[i - 1]))


def official_messages(rows):
    """What the judge saw: (time, x, y) of every '/actor_*_info' receipt."""
    out = []
    for r in rows:
        if r.get("kind") != "official":
            continue
        s = r.get("sample")
        if not isinstance(s, (list, tuple)) or len(s) < 3:
            continue
        out.append((float(s[0]), float(s[1]), float(s[2])))
    out.sort()
    return out


def replay(messages, truths):
    """Run score_cal's state machine, assigning each message to the actor it
    best matches (the judge does the same: it tries every actor in actor_ids)."""
    state = {a: dict(counting=False, start=None, prev=None) for a in truths}
    resets = collections.Counter()
    runs = {a: 0.0 for a in truths}
    errs = {a: [] for a in truths}
    all_errs = {a: [] for a in truths}
    hits = {a: 0 for a in truths}
    for now, x, y in messages:
        for actor, truth in truths.items():
            pos = truth_at(truth, now)
            if pos is None:
                continue
            dist = math.hypot(x - pos[0], y - pos[1])
            all_errs[actor].append(dist)
            st = state[actor]
            gap_ok = st["prev"] is None or (now - st["prev"]) <= DETECTION_INTERVAL
            err_ok = dist < ERR_THRESHOLD
            if not (gap_ok and err_ok):
                if st["counting"]:
                    resets["error_ge_1m" if not err_ok else "gap_gt_1s"] += 1
                st["counting"] = False
                st["start"] = None
                st["prev"] = now
                continue
            errs[actor].append(dist)
            hits[actor] += 1
            st["prev"] = now
            if not st["counting"]:
                st["counting"] = True
                st["start"] = now
            else:
                runs[actor] = max(runs[actor], now - st["start"])
    return runs, resets, errs, hits, all_errs


def pct(vals, p):
    if not vals:
        return None
    s = sorted(vals)
    return s[min(len(s) - 1, int(len(s) * p / 100.0))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--round", required=True)
    ap.add_argument("--observer", default=None,
                    help="observer jsonl; default tries each independent_visual_accuracy_*.jsonl")
    args = ap.parse_args()

    obs_dir = os.path.join(args.round, "observers")
    names = ([os.path.basename(args.observer)] if args.observer
             else sorted(n for n in os.listdir(obs_dir)
                         if n.startswith("independent_visual_accuracy")
                         and n.endswith(".jsonl")))
    print("round      : %s" % args.round)
    print("observer   : %s" % ", ".join(names))
    print("judge rules: interval<=%.1fs  err<%.1fm  duration>=%.1fs"
          % (DETECTION_INTERVAL, ERR_THRESHOLD, DETECTION_DURATION))
    print()

    for name in names:
        rows = load_rows(os.path.join(obs_dir, name))
        truths = truth_by_actor(rows)
        msgs = official_messages(rows)
        if not msgs or not truths:
            print("%-44s official=%-4d truth_actors=%s  (nothing to replay)"
                  % (name, len(msgs), sorted(truths)))
            continue
        runs, resets, errs, hits, all_errs = replay(msgs, truths)
        print("=== %s ===" % name)
        print("  official messages: %d   truth actors: %s" % (len(msgs), sorted(truths)))
        for actor in sorted(truths):
            r = runs[actor]
            ae = all_errs[actor]
            n = max(1, len(ae))
            print("  actor_%d: msgs=%-4d pass=%-4d (%.0f%%)  longest_run=%.2fs (need %.0fs) %s"
                  % (actor, len(ae), hits[actor], 100.0 * hits[actor] / n, r,
                     DETECTION_DURATION,
                     "WOULD DELETE" if r >= DETECTION_DURATION
                     else "short by %.2fs" % (DETECTION_DURATION - r)))
            if ae:
                print("           ALL-msg err : p50=%.2f p90=%.2f max=%.2f  <1m=%.0f%%"
                      % (pct(ae, 50), pct(ae, 90), max(ae),
                         100.0 * sum(1 for v in ae if v < ERR_THRESHOLD) / len(ae)))
        print("  resets: %s" % dict(resets))
        print()


if __name__ == "__main__":
    sys.exit(main())
