#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""探针：量 actor 的**真实**速度，验证 guide_patrol 发的 v 是否被插件尊重。

为什么要单独量
--------------
guide_patrol.py 往 /actor_N/cmd_motion 里塞的是 {x, y, v}，但 v 到底有没有
被 ros_actor_cmd_pose_plugin 采纳、有没有被限幅、是不是按**仿真时间**积分，
光看代码看不出来 —— 必须量。且本机 RTF≈0.4，用墙钟算速度会系统性偏小
（会把 1 m/s 仿真误判成 0.4 m/s），所以一律用 header.stamp（仿真时间）算。

用法：
  python3 probe_actor_speed.py <actor_id> [sec]
"""
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped

aid = sys.argv[1]
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

samples = []


def cb(m):
    samples.append((m.header.stamp.to_sec(),
                    m.pose.position.x, m.pose.position.y))


rospy.init_node("probe_actor_speed", anonymous=True)
rospy.Subscriber("/actor_%s/pose" % aid, PoseStamped, cb, queue_size=200)
t0 = time.time()
while time.time() - t0 < dur and not rospy.is_shutdown():
    rospy.sleep(0.2)

if len(samples) < 3:
    print("[spd] actor_%s 样本不足 n=%d" % (aid, len(samples)))
    raise SystemExit(1)

st = [s[0] for s in samples]
xs = [s[1] for s in samples]
ys = [s[2] for s in samples]

# 用中位瞬时速度更稳（避开折返点的掉头段）
inst = []
for i in range(1, len(samples)):
    dt = st[i] - st[i - 1]
    d = ((xs[i] - xs[i - 1]) ** 2 + (ys[i] - ys[i - 1]) ** 2) ** 0.5
    if dt > 1e-3:
        inst.append(d / dt)
inst_s = sorted(inst)
med = inst_s[len(inst_s) // 2] if inst_s else float("nan")

span_sim = st[-1] - st[0]
span_wall = dur
net = ((xs[-1] - xs[0]) ** 2 + (ys[-1] - ys[0]) ** 2) ** 0.5

print("[spd] actor_%s  n=%d" % (aid, len(samples)))
print("[spd]   仿真跨度 %.1fs  墙钟 %.1fs  => RTF=%.3f" % (span_sim, span_wall, span_sim / span_wall))
print("[spd]   瞬时速度 中位=%.3f m/s(仿真)  最大=%.3f" % (med, max(inst) if inst else float("nan")))
print("[spd]   净位移 %.2fm  起点(%.1f,%.1f) 终点(%.1f,%.1f)" % (net, xs[0], ys[0], xs[-1], ys[-1]))
