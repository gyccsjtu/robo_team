#!/usr/bin/env python3
"""Reproduce the on-board camera's VIEW: 2.5 m altitude, nearly level pitch -
the aircraft fly at 2-3 m looking forward, so the frustum contains a lot of
distant city. The original probe sat at 5 m pitched down 12.5 deg, framing a
nearby street, which may be why its per-frame cost looked so much cheaper.
"""
import os

SRC = "/root/render_ab/probe.world"
DST = "/root/render_ab/probe_low.world"

CX, CY, CZ = 71.0, -3.0, 2.5
PITCH, YAW = 0.06, 3.14159

s = open(SRC, encoding="utf-8").read()
# rewrite just the cam_probe pose line
old = "<pose>71.000 -3.000 5.000 0 0.21800 3.14159</pose>"
new = "<pose>%.3f %.3f %.3f 0 %.5f %.5f</pose>" % (CX, CY, CZ, PITCH, YAW)
assert s.count(old) == 1, "pose line not found (count=%d)" % s.count(old)
s = s.replace(old, new)
with open(DST, "w", encoding="utf-8") as fh:
    fh.write(s)
print("wrote %s : camera at (%.1f, %.1f, %.1f) pitch=%.3f (level, on-board-like view)"
      % (DST, CX, CY, CZ, PITCH))
