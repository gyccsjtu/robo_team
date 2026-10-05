#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把给定 actor 在两个点之间来回引导 —— 真飞验证时给相机一个"活的"目标。

为什么不用 guide_actor.py 的单点模式
------------------------------------
单点模式把 actor 引到 (x, y) 后它就走到了、然后**站住不动**。而感知的人判决
要求**净速度 >= 0.15 m/s**（这条是为了把静态误报挡掉），站住的目标会被直接
过滤 —— 看起来就像"YOLO 没检到"，其实是它按设计把静态物排除了。
所以在两点之间往复，让 actor 在整个飞行窗口里**一直在走**。

实现
----
· 订阅 /actor_<id>/pose 拿真实位置（Gazebo 对 actor 的 get_model_state 恒返 0）。
· 以 5 Hz 发 /actor_<id>/cmd_motion = {x, y, v}：当前目标点 + 速度。
· 当前位置距当前目标点 < ARRIVE_M 就切换目标点，如此往复。
· 用法前必须先 kill 掉该 actor 的 control_actor_m2 进程，否则它 10 Hz 发随机
  目标点，会把这里的指令盖掉。

用法：
  python3 guide_patrol.py <actor_id> <x1> <y1> <x2> <y2> [v] [sec]
    v   默认 1.5 m/s（必须 > 0.15，否则被当静态物滤掉）
    sec 默认 0（一直跑到 Ctrl-C）
"""
import math
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion

if len(sys.argv) < 6:
    print(__doc__)
    raise SystemExit(2)

aid = sys.argv[1]
p1 = (float(sys.argv[2]), float(sys.argv[3]))
p2 = (float(sys.argv[4]), float(sys.argv[5]))
tv = float(sys.argv[6]) if len(sys.argv) > 6 else 1.5
dur = float(sys.argv[7]) if len(sys.argv) > 7 else 0.0

ARRIVE_M = 1.5          # 到这个距离就算到了，切换目标点
cur = {"p": None}
tgt = {"p": p1}


def on_pose(m):
    cur["p"] = (m.pose.position.x, m.pose.position.y)


rospy.init_node("guide_patrol_" + aid, anonymous=True)
rospy.Subscriber("/actor_%s/pose" % aid, PoseStamped, on_pose, queue_size=1)
pub = rospy.Publisher("/actor_%s/cmd_motion" % aid, ActorMotion, queue_size=10)

rate = rospy.Rate(5)
t0 = time.time()
n = 0
switches = 0
while not rospy.is_shutdown():
    if cur["p"] is not None:
        d = math.hypot(cur["p"][0] - tgt["p"][0], cur["p"][1] - tgt["p"][1])
        if d < ARRIVE_M:
            tgt["p"] = p2 if tgt["p"] == p1 else p1
            switches += 1
            print("[patrol] actor_%s 到达，切到 (%.1f, %.1f)  累计切换=%d"
                  % (aid, tgt["p"][0], tgt["p"][1], switches), flush=True)
    m = ActorMotion()
    m.x = tgt["p"][0]
    m.y = tgt["p"][1]
    m.v = tv
    pub.publish(m)
    n += 1
    if dur > 0.0 and time.time() - t0 >= dur:
        break
    rate.sleep()

print("[patrol] actor_%s 往复 %s <-> %s  v=%.1f  发布=%d  切换=%d"
      % (aid, p1, p2, tv, n, switches), flush=True)
