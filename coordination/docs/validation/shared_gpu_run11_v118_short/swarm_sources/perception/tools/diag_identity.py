#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""身份对照诊断：同一时刻并排打印「演员真值」与「感知各流上报」。

要回答的问题（brown 轮实测暴露）：
  瞬移台盯 actor_2（官方 brown，暗褐衣），画面里 13 m 处却出现被标 red 的人。
  到底是
    (X) actor_2 被模型判成了 red，还是
    (Y) 画面里那个红衣人本来就是 actor_4/5 恰好游走到旁边？
  两者用一个数就能分开：red 流上报的坐标离 actor_2 近，还是离 actor_5 近。

注意：不能靠 val 混淆矩阵回答这个问题 —— val 集是近距离清晰样本，
     而实测是 13 m 外、目标在走的分布，两者不是一回事。

输出：每秒一块
  T=xx.x 真值: a0=(..,..) a2=(..,..) ...
         green  报(  x,  y) 距a0=  1.2m  (0.1s前)
         red1   报(  x,  y) 距a2=  0.8m  (0.1s前)   <- 这一行说明 red1 抢的就是 actor_2
"""
import math
import os

import rospy
from geometry_msgs.msg import PoseStamped
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo

TOPIC = {"/actor_green_info": "green", "/actor_blue_info": "blue",
         "/actor_brown_info": "brown", "/actor_white_info": "white",
         "/actor_red1_info": "red1", "/actor_red2_info": "red2"}
DUR = float(os.environ.get("DI_DUR", "45"))
STALE = float(os.environ.get("DI_STALE", "3.0"))

truth = {i: None for i in range(6)}
seen = {}


def on_truth(i):
    def cb(m):
        truth[i] = (m.pose.position.x, m.pose.position.y)
    return cb


def on_info(name):
    def cb(m):
        seen[name] = (rospy.get_time(), m.x, m.y)
    return cb


def owner(x, y):
    best, bd = None, 1e9
    for i, p in truth.items():
        if p is None:
            continue
        d = math.hypot(p[0] - x, p[1] - y)
        if d < bd:
            best, bd = i, d
    return best, bd


def main():
    rospy.init_node("diag_identity", anonymous=True)
    for i in range(6):
        rospy.Subscriber("/actor_%d/pose" % i, PoseStamped, on_truth(i), queue_size=1)
    for t, n in TOPIC.items():
        rospy.Subscriber(t, ActorInfo, on_info(n), queue_size=1)

    rospy.sleep(2.0)                 # 血泪：init 后立刻取 get_time 会得 0
    while sum(1 for v in truth.values() if v) < 6:
        rospy.sleep(0.3)
    t0 = rospy.get_time()
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        el = rospy.get_time() - t0
        if el > DUR:
            break
        now = rospy.get_time()
        ts = " ".join("a%d=(%6.1f,%6.1f)" % (i, truth[i][0], truth[i][1])
                      for i in range(6) if truth[i])
        print("T=%5.1f 真值: %s" % (el, ts), flush=True)
        for n in ("green", "blue", "brown", "white", "red1", "red2"):
            if n not in seen:
                continue
            t, x, y = seen[n]
            age = now - t
            if age > STALE:
                continue
            o, d = owner(x, y)
            print("        %-6s 报(%7.1f,%7.1f) 距a%s=%5.1fm  (%.1fs前)"
                  % (n, x, y, o, d, age), flush=True)
        rate.sleep()
    print("[di] 诊断结束", flush=True)


if __name__ == "__main__":
    main()
