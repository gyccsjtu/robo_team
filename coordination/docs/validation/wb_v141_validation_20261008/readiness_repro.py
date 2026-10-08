#!/usr/bin/env python3
"""Did the approach-readiness gate ever allow an official report?

Replays the round's real observations through the production ReportReadiness
class and prints, per observation, the target-to-camera distance and whether a
'ready' source ever existed. That is the condition on /actor_*_info publishing
when BRIDGE_APPROACH_REPORTING is on (default '1' in city_swarm_run.py:102).

Run: python3 readiness_repro.py --trace <round>/flight/algorithm/bridge_trace.jsonl
"""
import argparse
import json
import math
import os
import sys

SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "..", "..", "..", "src",
                                      "robocup_swarm", "scripts"))
sys.path.insert(0, SCRIPTS)
from report_readiness import ReportReadiness  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", required=True)
    args = ap.parse_args()
    rows = [json.loads(l) for l in open(args.trace, encoding="utf-8") if l.strip()]
    rows = [r for r in rows if r.get("kind") == "input"]
    print("input observations: %d" % len(rows))

    rr = ReportReadiness()
    ready_ever = False
    print()
    print("%-9s %-9s %-6s %-7s %-8s %-9s %-7s %-9s %s"
          % ("stamp", "receipt", "tag", "uav", "dist_m", "age_at_recv", "ready",
             "count", "note"))
    for r in rows:
        o = r["observation"]
        cam = o.get("camera_xyz")
        dist = math.hypot(o["xyz"][0] - cam[0], o["xyz"][1] - cam[1]) if cam else float("nan")
        age = float(r["receipt_s"]) - float(o["sample_s"])
        ready = rr.observe(o, float(r["receipt_s"]), bool(r.get("accepted")))
        ready_ever = ready_ever or bool(ready)
        note = []
        if dist > 12.0:
            note.append(">12m first-time gate")
        if age > 1.0:
            note.append("stale at receipt")
        if o.get("schema_version") != 3:
            note.append("schema!=3")
        key = (o["target_id"], o["uav_id"])
        src = rr.sources.get(key) or {}
        print("%-9.3f %-9.3f %-6s %-7s %-8.2f %-9.3f %-7s count=%-3s %s"
              % (float(o["sample_s"]), float(r["receipt_s"]), o["target_id"],
                 o["uav_id"], dist, age, ready, src.get("count"), ", ".join(note)))

    ready_sources = {k: v for k, v in rr.sources.items() if v.get("ready")}
    print()
    print("ready sources at end: %s" % (json.dumps(
        {str(k): {kk: vv for kk, vv in v.items()} for k, v in ready_sources.items()},
        ensure_ascii=False) if ready_sources else "{}"))
    print("any ready ever      : %s" % ready_ever)
    print()
    if not ready_ever:
        print("CONCLUSION: the approach gate never became ready on this round, and the")
        print("            gate defaults ON (CITY_APPROACH_REPORTING='1'), so _emit()")
        print("            skipped EVERY publish as 'not_ready' -> zero official reports")
        print("            -> the judge's 15 s timer never started -> 0/6.")
    else:
        print("CONCLUSION: readiness was reached at least once; check the allowed()")
        print("            stamp comparison and the fused-track freshness next.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
