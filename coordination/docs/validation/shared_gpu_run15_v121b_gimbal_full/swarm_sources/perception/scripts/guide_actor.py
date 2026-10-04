#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把指定 actor 引导到 (x, y) —— 真飞验证时保证目标进入相机视野。

为什么需要
----------
YOLO 只有在相机视野里有"够大"的目标时才可能检出。无人机在出生点 (-6, 4)
悬停 5 m 时，最近的 actor 也在 35 m 外 —— 752 px 宽的图上人只有约 5 px，
根本检不出来。那不是 YOLO 不行，是没东西可检。所以真飞验证必须先把一个
actor 送进视野，让这次实验有可证伪的对照。

做法
----
ActorMotion 消息 = {x, y, v}：目标点 + 速度。以固定频率发到
/actor_<id>/cmd_motion 即可让 actor 自己走过去。

⚠ 必须先停掉该 actor 的 control_actor_m2 进程，否则它 10 Hz 发随机目标点，
   会把我们的指令整个盖掉（现象：actor 还是乱走）。
   kill 之后世界上没人给它发指令，我们的 publisher 就是唯一来源。

⚠ v 必须 > 0.15 m/s。感知的人判决要求净速度 >= 0.15 m/s（用来把静态误报
   挡掉），站着不动的 actor 会被当静态物过滤 —— 那就白设了。

用法：
  python3 guide_actor.py <actor_id> <x> <y> [v] [sec]
    v   默认 1.5 m/s
    sec 默认 0（一直发到 Ctrl-C）
输出末尾打印实际发布条数，便于确认确实发出去了。
"""
import sys
import time

import rospy
from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion

if len(sys.argv) < 4:
    print(__doc__)
    raise SystemExit(2)

aid = sys.argv[1]
tx = float(sys.argv[2])
ty = float(sys.argv[3])
tv = float(sys.argv[4]) if len(sys.argv) > 4 else 1.5
dur = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0

rospy.init_node("guide_actor_" + aid, anonymous=True)
pub = rospy.Publisher("/actor_" + aid + "/cmd_motion", ActorMotion, queue_size=10)
rate = rospy.Rate(5)
t0 = time.time()
n = 0
while not rospy.is_shutdown():
    m = ActorMotion()
    m.x = tx
    m.y = ty
    m.v = tv
    pub.publish(m)
    n += 1
    if dur > 0.0 and time.time() - t0 >= dur:
        break
    rate.sleep()
print("[guide] actor_%s -> (%.1f, %.1f) v=%.1f  已发布 %d 条" % (aid, tx, ty, tv, n),
      flush=True)
