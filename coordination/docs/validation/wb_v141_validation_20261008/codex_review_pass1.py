#!/usr/bin/env python3
"""Codex review follow-ups, pass 1: facts only, no mechanism claims."""
import collections
import json

R = "/root/robocup_runs/wb_v141_validation3_20261008T104200Z/round_1_seed_8159"
BT = R + "/flight/algorithm/bridge_trace.jsonl"

ups = collections.defaultdict(list)
inputs = []
with open(BT, encoding="utf-8") as fh:
    for line in fh:
        line = line.strip()
        if not line:
            continue
        r = json.loads(line)
        if r.get("kind") == "upload":
            ups[r["tag"]].append(float(r["receipt_s"]))
        elif r.get("kind") == "input":
            inputs.append(r)

print("=" * 72)
print("A) upload gaps > 1 s, per tag  (Codex: 0.1 s median cannot rule out breaks)")
print("=" * 72)
for tag, ts in sorted(ups.items()):
    ts.sort()
    breaks = [(ts[i], ts[i + 1], ts[i + 1] - ts[i]) for i in range(len(ts) - 1)
              if ts[i + 1] - ts[i] > 1.0]
    print("%-6s n=%-4d breaks>1s: %d" % (tag, len(ts), len(breaks)))
    for a, b, g in breaks:
        print("        gap %6.1f s   %.1f -> %.1f" % (g, a, b))

print()
print("=" * 72)
print("B) what was the bridge seeing during the gaps? (is the bridge blind too?)")
print("=" * 72)
gaps = [(2034.0, 2142.9), (2191.9, 2382.9)]
for lo, hi in gaps:
    sel = [r for r in inputs if lo <= float(r["receipt_s"]) <= hi]
    o = [r["observation"] for r in sel]
    bytag = collections.Counter(x["target_id"] for x in o)
    acc = collections.Counter((x["target_id"], r.get("accepted")) for x, r in zip(o, sel))
    alive = collections.Counter((x["target_id"], r.get("alive")) for x, r in zip(o, sel))
    print("window %.1f-%.1f  inputs=%d" % (lo, hi, len(sel)))
    print("   by tag      :", dict(bytag))
    print("   (tag,accept):", dict(acc))
    print("   (tag,alive) :", dict(alive))

print()
print("=" * 72)
print("C) all inputs over time: which tags were being fed, and when")
print("=" * 72)
bt = collections.defaultdict(list)
for r in inputs:
    bt[r["observation"]["target_id"]].append(float(r["receipt_s"]))
for tag, ts in sorted(bt.items()):
    ts.sort()
    print("  %-6s n=%-5d first=%.1f last=%.1f" % (tag, len(ts), ts[0], ts[-1]))

print()
print("=" * 72)
print("D) city_trajectory format (for aircraft travel distance)")
print("=" * 72)
with open(R + "/flight/city_trajectory.jsonl", encoding="utf-8") as fh:
    first = fh.readline()
    second = fh.readline()
print("first :", first[:260])
print("second:", second[:260])
