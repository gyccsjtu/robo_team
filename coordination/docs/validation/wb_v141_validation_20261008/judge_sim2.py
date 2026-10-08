#!/usr/bin/env python3
"""Faithful replay of score_cal's TWO detection callbacks (v2).

v1 (judge_sim.py) modelled one generic state machine. codex correctly pointed
out that does not match the judge: the callbacks differ in reset condition,
reset scope, and even the interval comparison operator. This version follows
the source line by line.

green/blue/brown/white -> _process_actor_detection (score_cal.py:105-151)
  prev = arrive[id]; arrive[id] = now
  continuous = prev == 0.0 or now - prev <= DETECTION_INTERVAL      (<=)
  if dist >= 1 or not continuous:  _reset_detection(id)            # clears all three
  elif not count: count=True; find_t=now; -> 'find actor_N'
  elif now - find_t >= 15: _delete_actor

red -> _process_red_detection (score_cal.py:154-225)
  for each of actor 4,5: prev=arrive[a]; arrive[a]=now
      valid = dist < 1 and now - prev < DETECTION_INTERVAL          (< not <=)
      if valid: (same find/delete logic as above)
      else:     fail += 1
  if fail == 2 and the topic's remembered candidate c != reset_value:
      count_flag[c] = False            # ONLY count_flag; find_time/arrive survive
      candidate = reset_value

Reported per target: passes, resets, longest run, whether a delete would have
happened, and the error distribution over ALL messages.
"""
import argparse
import bisect
import collections
import json
import math
import os

DETECTION_INTERVAL = 1.0
DETECTION_DURATION = 15.0
ERR_THRESHOLD = 1.0

GREEN_LIKE = {"green": 0, "blue": 1, "brown": 2, "white": 3}
RED_ACTORS = (4, 5)


def load(path):
    out = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def truths(rows):
    acc = collections.defaultdict(list)
    for r in rows:
        if r.get("kind") != "actor_truth":
            continue
        s = r["sample"]
        if len(s) < 3:
            continue
        acc[int(s[3]) if len(s) > 3 else 0].append(
            (float(s[0]), float(s[1]), float(s[2])))
    out = {}
    for a, v in acc.items():
        v.sort()
        out[a] = ([p[0] for p in v], [p[1] for p in v], [p[2] for p in v])
    return out


def truth_at(truth, when, tol=1.5):
    """Ground position at time `when`, linearly interpolated.

    NOTE the argument names matter: an earlier draft called the first parameter
    `t` and then passed it to bisect_left as if it were the query time, which
    compares a tuple against floats (and raises). Keep `truth` and `when`
    distinct.
    """
    times, xs, ys = truth
    if not times:
        return None
    i = bisect.bisect_left(times, when)
    if i <= 0:
        return (xs[0], ys[0]) if abs(times[0] - when) <= tol else None
    if i >= len(times):
        return (xs[-1], ys[-1]) if abs(times[-1] - when) <= tol else None
    t0, t1 = times[i - 1], times[i]
    if t1 <= t0:
        return None
    f = (when - t0) / (t1 - t0)
    if not 0.0 <= f <= 1.0:
        return None
    return (xs[i - 1] + f * (xs[i] - xs[i - 1]), ys[i - 1] + f * (ys[i] - ys[i - 1]))


def uploads(path, tag):
    out = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            if r.get("kind") == "upload" and r.get("tag") == tag:
                out.append((float(r["receipt_s"]), r["requested_xy"][0], r["requested_xy"][1]))
    return sorted(out)


def pct(v, p):
    if not v:
        return float("nan")
    s = sorted(v)
    return s[min(len(s) - 1, int(len(s) * p / 100.0))]


def green_like(msgs, truth):
    st = dict(count=False, find=0.0, arrive=0.0)
    resets = 0
    runs = []
    allerr = []
    deletes = []
    cur_start = None
    for now, x, y in msgs:
        pos = truth_at(truth, now)
        if pos is None:
            continue
        d = math.hypot(x - pos[0], y - pos[1])
        allerr.append(d)
        prev = st["arrive"]
        st["arrive"] = now
        cont = (prev == 0.0) or (now - prev <= DETECTION_INTERVAL)   # <=
        if d >= ERR_THRESHOLD or not cont:
            if st["count"]:
                resets += 1
                if cur_start is not None:
                    runs.append(now - cur_start)
                    cur_start = None
            st["count"] = False
            st["find"] = 0.0
            st["arrive"] = 0.0          # _reset_detection clears all three
            continue
        if not st["count"]:
            st["count"] = True
            st["find"] = now
            cur_start = now
            continue
        if now - st["find"] < DETECTION_DURATION:
            continue
        deletes.append(now)
        cur_start = None
        st["count"] = False
        st["find"] = 0.0
        st["arrive"] = 0.0
    if st["count"] and cur_start is not None:
        runs.append(msgs[-1][0] - cur_start)
    return resets, max(runs) if runs else 0.0, allerr, deletes


def red(msgs, truth4, truth5):
    st = {4: dict(count=False, find=0.0, arrive=0.0),
          5: dict(count=False, find=0.0, arrive=0.0)}
    cand = 0                      # flag_1 starts at reset_value 0
    resets = 0
    runs = {4: [], 5: []}
    cur = {4: None, 5: None}
    allerr = collections.defaultdict(list)
    deletes = []
    for now, x, y in msgs:
        fail = 0
        for aid, tr in ((4, truth4), (5, truth5)):
            pos = truth_at(tr, now)
            if pos is None:
                fail += 1
                continue
            d = math.hypot(x - pos[0], y - pos[1])
            allerr[aid].append(d)
            prev = st[aid]["arrive"]
            st[aid]["arrive"] = now
            valid = (d < ERR_THRESHOLD) and (now - prev < DETECTION_INTERVAL)   # <
            if not valid:
                fail += 1
                continue
            if not st[aid]["count"]:
                st[aid]["count"] = True
                st[aid]["find"] = now
                cur[aid] = now
                cand = aid
                continue
            if now - st[aid]["find"] < DETECTION_DURATION:
                continue
            deletes.append((aid, now))
            cur[aid] = None
            st[aid]["count"] = False
            st[aid]["find"] = 0.0
            st[aid]["arrive"] = 0.0
        if fail == len(RED_ACTORS):
            if cand != 0 and cand in RED_ACTORS:      # current_flag != reset_value
                if st[cand]["count"]:
                    resets += 1
                    if cur[cand] is not None:
                        runs[cand].append(now - cur[cand])
                        cur[cand] = None
                st[cand]["count"] = False             # ONLY count_flag
                cand = 0
    return resets, runs, allerr, deletes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--round", required=True)
    ap.add_argument("--trace", required=True)
    args = ap.parse_args()
    obs = os.path.join(args.round, "observers")

    print("faithful replay (score_cal.py:105-225)")
    print("  green-like: prev == 0.0 or gap <= 1.0 ; ANY violation -> _reset_detection")
    print("  red       : valid needs dist < 1 AND gap < 1.0 ; clears count_flag only,")
    print("              and only when BOTH red actors fail the message")
    print()

    for tag, fname in (("green", "independent_visual_accuracy.jsonl"),
                       ("brown", "independent_visual_accuracy_brown.jsonl"),
                       ("red1", "independent_visual_accuracy_red.jsonl"),
                       ("blue", "independent_visual_accuracy_blue.jsonl"),
                       ("white", "independent_visual_accuracy_white.jsonl")):
        fp = os.path.join(obs, fname)
        if not os.path.isfile(fp):
            continue
        rows = load(fp)
        tr = truths(rows)
        msgs = uploads(args.trace, tag)
        if not msgs or not tr:
            print("%-6s uploads=%-4d truth_actors=%s -> nothing" % (tag, len(msgs), sorted(tr)))
            continue
        if tag.startswith("red"):
            if 4 not in tr or 5 not in tr:
                print("%-6s needs both actor_4 and actor_5 truth; have %s" % (tag, sorted(tr)))
                continue
            resets, runs, allerr, deletes = red(msgs, tr[4], tr[5])
            print("=== %s (%d uploads) ===" % (tag, len(msgs)))
            for aid in (4, 5):
                rr = runs[aid]
                e = allerr[aid]
                print("  actor_%d: msgs=%-4d resets(cleared)=%-3d longest_run=%.2f s  delete_would_fire=%s"
                      % (aid, len(e), resets if aid == 5 else 0,
                         max(rr) if rr else 0.0,
                         "YES" if any(d[0] == aid for d in deletes) else "no"))
                if e:
                    print("           ALL-msg err p50=%.2f p90=%.2f max=%.2f  <1m=%.0f%%"
                          % (pct(e, 50), pct(e, 90), max(e),
                             100.0 * sum(1 for v in e if v < 1.0) / len(e)))
            print("  topic-level resets: %d   deletes: %s" % (resets, deletes or "none"))
        else:
            aid = GREEN_LIKE.get(tag, 0)
            if aid not in tr:
                print("%-6s truth for actor_%d missing" % (tag, aid))
                continue
            resets, longest, allerr, deletes = green_like(msgs, tr[aid])
            print("=== %s (%d uploads, actor_%d) ===" % (tag, len(msgs), aid))
            print("  resets=%-3d  longest_run=%.2f s (need 15)  delete_would_fire=%s"
                  % (resets, longest, "YES" if deletes else "no"))
            if allerr:
                print("  ALL-msg err p50=%.2f p90=%.2f max=%.2f  <1m=%.0f%%"
                      % (pct(allerr, 50), pct(allerr, 90), max(allerr),
                         100.0 * sum(1 for v in allerr if v < 1.0) / len(allerr)))
        print()


if __name__ == "__main__":
    main()
