#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读官方 base.world 的建筑 AABB，打印分布 + 推荐跨城区航线。"""
import os, sys, math

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
for c in (os.path.join(os.path.dirname(HERE), '_demo2026'),
          os.path.expanduser('~/robocup_real/_demo2026'),
          os.environ.get('RADAR_DEMO_DIR', '')):
    if c and os.path.isfile(os.path.join(c, 'astar_plan.py')):
        sys.path.insert(0, c)
        break
else:
    raise SystemExit('找不到 astar_plan.py')

import astar_plan as A

WORLD = os.path.expanduser('~/XTDrone/robocup/base.world')
bs = A.buildings_from_world(WORLD)   # (name, cx, cy, hx, hy, 0.0)
print("官方世界建筑数 =", len(bs))
print()
print(" idx  名称                     中心(x,y)          半尺寸(hx,hy)     x范围              y范围")
xs, ys = [], []
for i, b in enumerate(bs):
    nm, cx, cy, hx, hy = b[0], float(b[1]), float(b[2]), float(b[3]), float(b[4])
    xs.append(cx); ys.append(cy)
    print("%4d  %-22s (%8.2f,%8.2f)  (%.1f,%.1f)  [%7.2f,%7.2f]  [%7.2f,%7.2f]"
          % (i, nm[:22], cx, cy, hx, hy, cx-hx, cx+hx, cy-hy, cy+hy))
print()
print("建筑中心范围: x ∈ [%.1f, %.1f]   y ∈ [%.1f, %.1f]"
      % (min(xs), max(xs), min(ys), max(ys)))
print("建筑包络:     x ∈ [%.1f, %.1f]   y ∈ [%.1f, %.1f]"
      % (min(float(b[1])-float(b[3]) for b in bs), max(float(b[1])+float(b[3]) for b in bs),
         min(float(b[2])-float(b[4]) for b in bs), max(float(b[2])+float(b[4]) for b in bs)))

# 找"穿过建筑密集区"的航线：对若干候选起终点，统计 A* 航点数与沿途建筑数
print()
print("=== 候选长航线（A* 航点数越多 = 穿建筑越多）===")
cands = [
    ((-60.0, 60.0), (0.0, 0.0)),
    ((-60.0, 60.0), (10.0, -25.0)),
    ((-16.5, 25.3), (10.0, -25.0)),
    ((-40.0, 40.0), (20.0, -20.0)),
    ((-60.0, 0.0), (30.0, 0.0)),
    ((-60.0, 60.0), (40.0, -40.0)),
    ((-30.0, 30.0), (30.0, -30.0)),
    ((0.0, 40.0), (0.0, -40.0)),
]
for (s, g_) in cands:
    try:
        grid, raw, wps = A.plan(bs, [], s, g_)
    except Exception as e:
        print("  %s -> %s : 规划失败 %s" % (s, g_, e)); continue
    if not wps:
        print("  %s -> %s : 无解" % (s, g_)); continue
    dist = sum(math.hypot(wps[i+1][0]-wps[i][0], wps[i+1][1]-wps[i][1])
               for i in range(len(wps)-1))
    # 沿途（航点连线 8m 内）建筑数
    near = set()
    for i in range(len(wps)-1):
        ax, ay = wps[i]; bx, by = wps[i+1]
        for j, b in enumerate(bs):
            bcx, bcy, bhx, bhy = float(b[1]), float(b[2]), float(b[3]), float(b[4])
            for t in [k/10.0 for k in range(11)]:
                px = ax + (bx-ax)*t; py = ay + (by-ay)*t
                if abs(px-bcx) < bhx+8 and abs(py-bcy) < bhy+8:
                    near.add(j); break
    print("  %-16s -> %-16s 航点%3d  直线%6.1fm  规划%6.1fm  沿线建筑 %d 栋"
          % (str(s), str(g_), len(wps), math.hypot(g_[0]-s[0], g_[1]-s[1]), dist, len(near)))
