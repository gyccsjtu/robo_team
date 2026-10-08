#!/usr/bin/env python3
"""Offline reproduction: why did yolo_target_bridge publish zero ActorInfo?

Feeds the round's real observations into the bridge's pure TargetBridgeCore and
interleaves a 10 Hz timer over the SAME timeline (inputs and ticks in
chronological order, exactly as at runtime), then evaluates _emit()'s ActorInfo
gate for every event the core returns.

An earlier version of this script fed every input first and replayed the timer
afterwards; that put the track in a post-drop state and produced a false
"tick() returns no events" conclusion. Order matters.

Run:
  python3 bridge_publish_repro.py --trace <round>/flight/algorithm/bridge_trace.jsonl
"""
import argparse
import json
import os
import sys

SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "..", "..", "..", "src",
                                      "robocup_swarm", "scripts"))
sys.path.insert(0, SCRIPTS)

import yolo_target_bridge as B  # noqa: E402
from report_readiness import ReportReadiness  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", required=True)
    ap.add_argument("--pub-hz", type=float, default=10.0)
    ap.add_argument("--approach-reporting", type=int, default=1,
                    help="value of BRIDGE_APPROACH_REPORTING. city_swarm_run.py:102 "
                         "defaults it to '1', so 1 is the realistic setting; the "
                         "first version of this script effectively assumed 0 and "
                         "therefore predicted 59 publishes for a round that sent none.")
    args = ap.parse_args()

    rows = [json.loads(l) for l in open(args.trace, encoding="utf-8") if l.strip()]
    print("trace rows: %d  kinds: %s" % (len(rows), sorted({r.get("kind") for r in rows})))

    core = B.TargetBridgeCore()
    red = B.RedObservations()
    # Mirror the runtime gate: with approach reporting ON, readiness gates every
    # publish. Computed with the production ReportReadiness class.
    approach = bool(args.approach_reporting)
    readiness = ReportReadiness()

    # Merge inputs (at receipt time) with timer ticks, chronologically.
    todo = []
    for i, r in enumerate(rows):
        todo.append((float(r["receipt_s"]), 0, i, r))
    first_receipt = min(t[0] for t in todo)
    last_receipt = max(t[0] for t in todo)
    step = 1.0 / max(1.0, args.pub_hz)
    t = first_receipt
    while t <= last_receipt + 1.0:
        todo.append((t, 1, 0, None))
        t += step
    todo.sort(key=lambda x: (x[0], x[1]))

    accepted = 0
    events = 0
    gate_pass = 0
    reasons = {}
    alive_span = []
    publishes = []

    for when, kind, idx, row in todo:
        if kind == 0:
            o = row["observation"]
            tag = o["target_id"]
            stamp = float(o["sample_s"])
            if tag in ("red1", "red2"):
                tag = red.observe(stamp, (o["xyz"][0], o["xyz"][1]))
                if tag is None:
                    continue
            ok = core.report(stamp, tag, o["xyz"][0], o["xyz"][1],
                             float(o["confidence"]), o["uav_id"],
                             o.get("observation_id"))
            accepted += 1 if ok else 0
            if approach:
                readiness.observe(o, when, ok)
            tr = core.tracks[tag]
            alive_span.append((when, tag, tr.alive))
            continue
        for ev in core.tick(when):
            events += 1
            tr = core.tracks[ev["tag"]]
            # Use the production gate function, not a private copy of it.
            tag = B.TID_TO_TAG.get(ev["tid"])
            readiness_ok = True
            if approach and tag is not None:
                readiness_ok = readiness.allowed(tag, when, tr.t_obs)
            reason = B.emit_gate_reason(tag, core.tracks, ev, tr, when,
                                        approach_reporting=approach,
                                        readiness_ok=readiness_ok)
            if reason is None:
                gate_pass += 1
                publishes.append((round(when, 1), ev["tag"], round(ev["x"], 2), round(ev["y"], 2)))
            else:
                reasons[reason] = reasons.get(reason, 0) + 1

    print("approach reporting   : %s" % ("ON (runner default)" if approach else "OFF"))
    print("accepted inputs      : %d / %d" % (accepted, len(rows)))
    ready = {k: v for k, v in readiness.sources.items() if v.get("ready")}
    print("ready approach source: %s" % (json.dumps({str(k): v for k, v in ready.items()})
                                         if ready else "{} (never ready)"))
    flips = [a for a in alive_span if a[2]]
    if flips:
        print("alive window         : first=%.3f last=%.3f  (%d input rows with alive=True)"
              % (flips[0][0], flips[-1][0], len(flips)))
    print("events from core.tick: %d" % events)
    print("gate would PASS      : %d" % gate_pass)
    print("gate failure reasons : %s" % (reasons if reasons else "none"))
    if publishes:
        print("first 5 would-publish: %s" % publishes[:5])
        print("last  would-publish  : %s" % publishes[-3:])
    print()
    if gate_pass:
        print("CONCLUSION: inputs + a 10 Hz timer DO yield publishes here.")
    elif events:
        print("CONCLUSION: events flow but every one is gate-rejected: %s" % reasons)
        if reasons.get("not_ready"):
            print("            'not_ready' = the approach-readiness gate. With")
            print("            BRIDGE_APPROACH_REPORTING=1 (runner default) a target must")
            print("            be observed inside 12 m for >=3 frames before any publish.")
    else:
        print("CONCLUSION: core.tick() never returned an event on this timeline.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
