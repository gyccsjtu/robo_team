#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""压力测试：把通道收窄 / 障碍加密 / 加大步长，找鲁棒性边界。

目的：闭环 5 场景是"理想工况"。实飞世界里建筑间距不规整，
需要在**更苛刻**的几何下仍不撞，才敢上机。
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), '_demo2026'))

import test_radar_offline as T
T.install_stubs()
import radar_avoid as R
from sim_radar_closed_loop import Box, World, simulate


def case(label, boxes, start, goal, speed=1.5, stride=4):
    w = World(boxes)
    r = simulate(w, start, goal, speed=speed, stride=stride)
    verdict = ('PASS' if (r['min_clear'] > R.CLEARANCE and r['collisions'] == 0
                          and r['ok'])
               else ('擦过' if r['min_clear'] > R.BODY_RADIUS else 'FAIL'))
    print('  %-34s 到达=%-5s 步数=%-5d 净空=%+7.3f 碰撞=%-5d %s'
          % (label, r['ok'], r['steps'], r['min_clear'], r['collisions'],
             verdict))
    return r


def main():
    print('=== 压力测试 A：通道宽度递减（飞机半径 0.45、余量要求 0.80）===')
    for half_w in (3.0, 2.5, 2.0, 1.8, 1.6, 1.4):
        boxes = [Box(10, half_w, 30, 20, 'up'),
                 Box(10, -20, 30, -half_w, 'down')]
        case('通道半宽 %.1fm（净宽 %.1fm）' % (half_w, half_w * 2),
             boxes, (0.0, 0.0), (40.0, 0.0))

    print()
    print('=== 压力测试 B：交错阵列的"通道净宽"递减 ===')
    # 几何：a/c 在上（y0=-2）、b 在下。飞机必须从 a 下方(y<-2)穿过、
    # 再从 b 上方(y>-4)穿过 ⇒ 通道 = y∈[b.y1, a.y0] = [gap-2, -2]，
    # 净宽 = gap。飞机半径 0.45、要求单侧余量 0.80 ⇒ 理论下限 2.5m。
    for gap in (4.0, 3.5, 3.0, 2.5, 2.2):
        boxes = [Box(8, -2, 16, 12, 'a'),
                 Box(20, -2 - gap - 8, 28, -2 - gap, 'b'),
                 Box(32, -2, 40, 12, 'c')]
        case('交错通道净宽 %.1fm' % gap, boxes, (0.0, 0.0), (48.0, 0.0))

    print()
    print('=== 压力测试 C：抽稀步长（算力 vs 精度）===')
    boxes = [Box(8, -2, 16, 8, 'a'), Box(20, -8, 28, 2, 'b'),
             Box(32, -2, 40, 8, 'c')]
    for st in (1, 2, 4, 8, 16):
        case('stride=%d' % st, boxes, (0.0, 0.0), (48.0, 0.0), stride=st)

    print()
    print('=== 压力测试 D：飞行速度（抗机动能力）===')
    boxes = [Box(8, -2, 16, 8, 'a'), Box(20, -8, 28, 2, 'b'),
             Box(32, -2, 40, 8, 'c')]
    for sp in (1.0, 1.5, 2.0, 3.0, 4.0):
        case('速度 %.1f m/s' % sp, boxes, (0.0, 0.0), (48.0, 0.0), speed=sp)

    print()
    print('=== 压力测试 E：起点偏置（飞机不在通道中线上）===')
    boxes = [Box(10, 3, 30, 20, 'up'), Box(10, -20, 30, -3, 'down')]
    for y0 in (0.0, 1.0, 2.0, 2.5, -2.0):
        case('起点 y=%.1f' % y0, boxes, (0.0, y0), (40.0, 0.0))

    print()
    print('=== 压力测试 F：目标在障碍背后（必须绕半圈）===')
    boxes = [Box(6, -8, 12, 8, 'wall')]
    case('目标正后方 (18,0)', boxes, (0.0, 0.0), (18.0, 0.0))
    case('目标偏右上 (18,6)', boxes, (0.0, 0.0), (18.0, 6.0))
    return 0


if __name__ == '__main__':
    sys.exit(main())
