#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定向验证：掠射长墙场景下，抽稀统计失效、全分辨率统计生效。

场景：飞机在原点，行进方向 +x，左侧 0.55m 有一条与行进方向平行的
有限长墙段（y=0.55, x∈[2,8]）。掠射 ⇒ 只有落点在墙段范围内的
少数束能命中（对应实测"128 束里仅 2~4 束"）；再将**偶数索引**的
命中束按"该束无回波"置 inf（最坏情形）——于是 GAP_STRIDE=2 抽稀后
**一束不剩**（复现"实测②"：obs 整帧为空 ⇒ 旧守门不动作）。

验证点：
  1. scan_to_obstacles(stride=GAP_STRIDE) == []   （复现旧缺陷输入）
  2. lateral_mins_fullres(...) 返回 left≈0.55     （新输入看得见墙）
  3. lateral_guard_shift(left=0.55, right=None) 把子目标向右推 ≥0.5m
"""
import math
import os
import sys

RADAR_DIR = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(
    os.path.abspath(__file__))
sys.path.insert(0, RADAR_DIR)

import test_radar_offline as _T  # noqa: E402

_T.install_stubs()                   # rospy 等不可用时用测试桩顶替
import radar_avoid as R  # noqa: E402

N = 512
WALL_Y = 0.55
WALL_X0, WALL_X1 = 2.0, 8.0          # 有限长墙段（与行进方向平行）
ang_min = -math.pi
inc = 2.0 * math.pi / N

ranges = [float('inf')] * N
hits = []
for i in range(N):
    a = ang_min + i * inc
    s = math.sin(a)
    if s <= 1e-6:
        continue                     # 不朝 +y 半平面，打不到这条墙
    x_hit = WALL_Y / math.tan(a)     # 射线与 y=WALL_Y 的交点横坐标
    if not (WALL_X0 <= x_hit <= WALL_X1):
        continue                     # 落点在墙段外 ⇒ 无回波
    r = WALL_Y / s
    if not (R.RANGE_MIN < r < R.RANGE_MAX):
        continue
    ranges[i] = r
    hits.append((i, r))

# 最坏情形：偶数索引束"无回波"（模拟掠射回波稀疏 + 抽稀错位）
ranges = [ranges[i] if i % 2 == 1 else float('inf') for i in range(N)]
kept = [(i, round(r, 3)) for i, r in hits if i % 2 == 1]
print('几何命中束:', [(i, round(r, 3)) for i, r in hits])
print('置无回波后剩余命中束（全部奇数索引）:', kept)
assert kept and all(i % 2 == 1 for i, _ in kept), '场景构造失败'

msg = type('Scan', (), {'angle_min': ang_min,
                        'angle_increment': inc,
                        'range_max': R.RANGE_MAX,
                        'ranges': ranges})()

# 1) 抽稀统计（旧守门的唯一输入源）
obs = R.scan_to_obstacles(msg, 0.0, (0.0, 0.0), (0.0, 0.0),
                          stride=R.GAP_STRIDE)
print('1) 抽稀障碍点数 stride=%d: %d  %s' % (
    R.GAP_STRIDE, len(obs),
    '⇒ 复现旧缺陷：守门无输入' if not obs else ''))

# 2) 全分辨率统计（新守门输入）
left, right = R.lateral_mins_fullres(msg, 0.0, (0.0, 0.0), (0.0, 0.0),
                                     1.0, 0.0)
print('2) 全分辨率 left_min=%s right_min=%s' % (
    None if left is None else round(left, 3),
    None if right is None else round(right, 3)))
assert obs == [], '抽稀列表应复现为空（否则场景没复现掠射）'
assert left is not None and abs(left - WALL_Y) < 0.05, \
    '全分辨率应看到墙（left≈%.2f）' % WALL_Y
assert right is None

# 3) 守门动作：子目标 (10,0)，应向右（-y）推离墙
sub_x, sub_y = R.lateral_guard_shift(
    10.0, 0.0, 0.0, 1.0, left, right,
    dilute=1.0, amt_cap=R.SAFE_GAP * 2.0)
print('3) 子目标 (10,0) -> (%.3f, %.3f)  横向位移 %.3f m（负=向右离墙）' % (
    sub_x, sub_y, sub_y))
assert sub_y <= -0.5, '守门应把子目标推离墙 ≥0.5m'

print()
print('✅ 验证通过：旧链路在此场景下守门无输入，新链路看得见墙并推开 0.5m+')
