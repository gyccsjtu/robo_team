#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把感知每帧的"发布决策"原封不动落盘（jsonl），供离线归因。

为什么需要它
------------
感知已经在 `/coordination/target_report` 上发布了完整的"本帧每类选了哪个
track，这个 track 的 conf / 归一化身高 / 净速度 / 世界坐标 / hits / coast"，
但那是**瞬时的** —— 现象只在那一秒可见，事后无法复盘。
落盘之后，"brown 为什么会报出 31 m" 就从一个需要盯着屏幕猜的问题，
变成一个可以用脚本回答的数据问题。

⚠ 教训：上一轮我对 brown 病因的判断（"actor_2 被认成 red"）就是只看了几帧
得出的，随后被混淆矩阵和身份对照诊断双双否掉。**先把数据落全，再下结论。**

落盘每行（在感知原始 snap 上补两个字段）：
  {"t": 仿真时间, "stamp": 墙钟, "uav": ...,
   "truth": {"a0": [x,y], ...},              # 当时的 actor 真值
   "dets": [{...含 cls/conf/h/sp/score/tid/hits/coast...,
             "dist": {"a0": 12.3, ...}}]}    # 该坐标到 6 个 actor 真值的距离

用法：
  REC_OUT=/tmp/snap_brown.jsonl REC_DUR=90 python3 rec_snap.py
"""
import json
import math
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

OUT = os.environ.get("REC_OUT", "/tmp/snap.jsonl")
DUR = float(os.environ.get("REC_DUR", "90"))
# ⚠ 2026-09-20：调试快照已从 /coordination/target_report **拆到独立话题**。
# 那条话题上现在跑的是协同契约（每目标一条、只有 target_id/frame_id/xyz/
# confidence/observation_id 五个字段，没有 dets），形状与本脚本记录的完全不同。
TOPIC = os.environ.get("REC_TOPIC", "/perception/debug_snapshot")

truth = [None] * 6          # (x, y, t)
stat = {"n": 0, "nev": 0}


def on_truth(i):
    def cb(msg):
        truth[i] = (msg.pose.position.x, msg.pose.position.y, rospy.get_time())
    return cb


def on_snap(msg, fh):
    try:
        d = json.loads(msg.data)
    except Exception:
        return
    now = rospy.get_time()
    for det in d.get("dets", []):
        try:
            x, y = det["xyz"][0], det["xyz"][1]
        except Exception:
            continue
        det["dist"] = {("a%d" % i): (None if truth[i] is None else
                                     round(math.hypot(x - truth[i][0], y - truth[i][1]), 2))
                       for i in range(6)}
    d["t"] = round(now, 3)
    d["truth"] = {("a%d" % i): (None if truth[i] is None else
                                [round(truth[i][0], 2), round(truth[i][1], 2)])
                  for i in range(6)}
    fh.write(json.dumps(d, ensure_ascii=False) + "\n")
    fh.flush()
    stat["n"] += 1
    stat["nev"] += len(d.get("dets", []))


def main():
    rospy.init_node("rec_snap", anonymous=True)
    fh = open(OUT, "w")
    for i in range(6):
        rospy.Subscriber("/actor_%d/pose" % i, PoseStamped, on_truth(i), queue_size=1)
    rospy.Subscriber(TOPIC, String, lambda m: on_snap(m, fh), queue_size=50)
    print("[rec] 记录 %s -> %s，时长 %.0f s" % (TOPIC, OUT, DUR), flush=True)
    time.sleep(1.5)
    t0 = rospy.get_time()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        el = rospy.get_time() - t0
        if el >= DUR:
            break
        if int(el) % 20 == 0 and int(el) != getattr(main, "_last", -1):
            main._last = int(el)
            print("[rec] t=%3.0fs  帧=%d  目标条目=%d" % (el, stat["n"], stat["nev"]),
                  flush=True)
        rate.sleep()
    fh.close()
    print("[rec] 完成：%d 帧 %d 目标条目 -> %s" % (stat["n"], stat["nev"], OUT), flush=True)


if __name__ == "__main__":
    main()
