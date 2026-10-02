#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证雷达抬高到 1.80m 后 sonar 锥体自检是否消失。
判定：<3.5m 束数 应为 0（修复前 173 束、min=1.7056）。"""
import sys, math, statistics
import rospy
from sensor_msgs.msg import LaserScan

TOPIC = sys.argv[1] if len(sys.argv) > 1 else '/typhoon_h480_0/scan'
THRESH = float(sys.argv[2]) if len(sys.argv) > 2 else 3.5
N = int(sys.argv[3]) if len(sys.argv) > 3 else 20

frames = []
def cb(msg):
    rs = [r for r in msg.ranges if r == r and 0.0 < r < 1e5]
    frames.append((list(msg.ranges), rs, msg.angle_min, msg.angle_increment))

rospy.init_node('verify_lidar_fix', anonymous=True)
sub = rospy.Subscriber(TOPIC, LaserScan, cb, queue_size=1)
t0 = rospy.Time.now().to_sec()
while len(frames) < N and not rospy.is_shutdown():
    if rospy.Time.now().to_sec() - t0 > 30:
        break
    rospy.sleep(0.2)

print("=" * 68)
print("雷达自检验证  topic=%s  采样帧=%d" % (TOPIC, len(frames)))
print("=" * 68)
if not frames:
    print("!! 没有收到任何 scan 帧"); sys.exit(2)

near_min, near_cnt_all, total = [], [], []
for raw, rs, amin, ainc in frames:
    total.append(len(rs))
    n = [r for r in rs if r < THRESH]
    near_cnt_all.append(len(n))
    if n: near_min.append(min(n))

print("总束数/帧      : %d" % (total[0] if total else -1))
print("有限回波/帧    : 中位 %.0f  范围 %d~%d" % (
    statistics.median(total), min(total), max(total)))
print("距离<%sm 束数  : 中位 %.0f  合计 %d" % (
    THRESH, statistics.median(near_cnt_all), sum(near_cnt_all)))
if near_min:
    print("其中最近距离   : min %.4f m  中位 %.4f m" % (
        min(near_min), statistics.median(near_min)))
else:
    print("其中最近距离   : 无（该距离内没有任何回波）")

print("-" * 68)
# 每帧明细（末 5 帧）
for i, (raw, rs, amin, ainc) in enumerate(frames[-5:]):
    n = [r for r in rs if r < THRESH]
    print("帧%-3d 有限=%-4d <%s=%-3d min=%-8s" % (
        i + 1, len(rs), THRESH, len(n),
        ('%.4f' % min(n)) if n else '--'))

print("=" * 68)
soft = sum(1 for c in near_cnt_all if c > 0)
print("含 <%sm 回波的帧数: %d / %d" % (THRESH, soft, len(frames)))
if soft == 0:
    print(">>> PASS  sonar 锥体自检已消除（修复前 173 束 / min 1.7056）")
else:
    print(">>> FAIL  仍有 %d 帧存在 <%.1fm 回波" % (soft, THRESH))
print("=" * 68)
