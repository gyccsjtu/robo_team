#!/usr/bin/env python3
"""为每张随机图挑一条「起飞点干净 + A* 零穿墙 + 够长」的验证航线。

只读 black_box.txt 真值，不解析 world（避开 mesh/key/yaw 四个坑）。
"""
import os
import sys
import math

sys.path.insert(0, os.path.expanduser('~/robocup_real/_sync_in'))
import astar_plan as A                       # noqa: E402

TAKEOFF = [(0.0, -3.0), (3.0, -3.0), (0.0, 0.0), (3.0, 0.0), (0.0, 3.0), (3.0, 3.0)]
GOALS = [(110, -35), (115, 35), (-45, -45), (120, -30),
         (112, 22), (62, 44), (-42, 42), (126, -12)]


def inside(bld, p):
    for b in bld:
        if abs(p[0] - b[1]) <= b[3] and abs(p[1] - b[2]) <= b[4]:
            return b[0]
    return None


for i in (2, 3, 5, 6):
    d = os.path.expanduser('~/robocup_real/_genvm/try%d' % i)
    bb = os.path.join(d, 'black_box.txt')
    w = os.path.join(d, 'robocup.world')
    if not os.path.exists(bb):
        print('=== try%d: 无 black_box.txt，跳过' % i)
        continue
    if os.path.getsize(w) == 0:
        print('=== try%d: world 0 字节（官方 fallback），跳过' % i)
        continue
    bld = A.boxes_from_black_box(bb)
    print('=== try%d  障碍框 %d' % (i, len(bld)))
    frees = [t for t in TAKEOFF if not inside(bld, t)]
    if not frees:
        print('   ⛔ 6 个起飞点全被压（官方生成器缺陷）')
        continue
    s = frees[0]
    print('   干净起飞点: %s   -> 用 %s' % (frees, s))
    best = None
    for g in GOALS:
        nm = inside(bld, g)
        if nm:
            print('     %-12s 终点在障碍(%s)内，跳过' % (str(g), nm))
            continue
        try:
            g2, raw, p = A.plan(bld, [], s, g, 2.0)
            nb, bad = A.verify(g2, p)
            L = sum(math.dist(p[k], p[k + 1]) for k in range(len(p) - 1))
            print('     %-12s 航点%2d 长度%6.1fm 直线%6.1fm 穿墙%d'
                  % (str(g), len(p), L, math.dist(s, g), nb))
            if nb == 0 and (best is None or L > best[1]):
                best = (g, L, len(p))
        except Exception as e:
            print('     %-12s ERR %s' % (str(g), e))
    if best:
        print('   >>> 选中: %s  长 %.1fm  航点 %d' % (best[0], best[1], best[2]))
