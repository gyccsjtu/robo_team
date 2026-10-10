#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""score_diag.py —— 「分数从何而来」实地取证脚本（v2 修复版）。

不修改你任何官方代码。订阅官方裁判发布的原始话题：
  /score             Int16   实时分数
  /time_usage        Int16   已用时间（秒）
  /left_actors       String  剩余 actor 清单
  /actor_<color>_info  ActorInfo  6 条颜色流

启动 10 秒后打印：
  - 初始 1 秒 / 5 秒 / 10 秒三个时刻的 /score、/time_usage、/left_actors
  - 6 条颜色流：是否到达、cls、x/y
  - 反推 sensor_cost（假设 find=tgt=track=uav_loss=0）
  - 反推 (find, target) 组合 -> score 与末帧 /score 最接近的组合
"""
import sys
import time
import json

import rospy
from std_msgs.msg import Int16, String
from robocup_swarm.msg import ActorInfo

SCORE_TOPIC = "/score"
TIME_TOPIC = "/time_usage"
LEFT_TOPIC = "/left_actors"
COLOR_TOPICS = {
    "green": "/actor_green_info",
    "blue":  "/actor_blue_info",
    "white": "/actor_white_info",
    "brown": "/actor_brown_info",
    "red1":  "/actor_red1_info",
    "red2":  "/actor_red2_info",
}

SAMPLES = [1.0, 5.0, 10.0]
score_samples = {}
time_samples = {}
left_samples = {}
first_seen = {}
last_msg = {}


def cb_score(msg):
    score_samples[rospy.get_time()] = msg.data


def cb_time(msg):
    time_samples[rospy.get_time()] = msg.data


def cb_left(msg):
    # std_msgs/String 只有一层 .data，msg.data 已经是 str
    left_samples[rospy.get_time()] = msg.data


def cb_color(name):
    def _cb(m):
        last_msg[name] = (m.cls, m.x, m.y)
        if name not in first_seen:
            first_seen[name] = (rospy.get_time(), m.cls, m.x, m.y)
    return _cb


def near(d, key):
    if not d:
        return None
    return min(d.items(), key=lambda kv: abs(kv[0] - key))[1]


def main():
    rospy.init_node("score_diag", anonymous=True)
    rospy.Subscriber(SCORE_TOPIC, Int16, cb_score)
    rospy.Subscriber(TIME_TOPIC, Int16, cb_time)
    rospy.Subscriber(LEFT_TOPIC, String, cb_left)
    for k, t in COLOR_TOPICS.items():
        rospy.Subscriber(t, ActorInfo, cb_color(k), queue_size=10)

    start = rospy.get_time()
    print("[diag v2] 已订阅 9 个话题，开始 10 秒采样…")
    print("[diag v2] 启动那一刻 score 理论值 = -sensor_cost * 0.003")
    print()

    rate = rospy.Rate(20)
    deadline = start + max(SAMPLES) + 1.0
    while rospy.get_time() < deadline and not rospy.is_shutdown():
        rate.sleep()

    print("=" * 64)
    print("【A. /score / /time_usage / /left_actors 取样】")
    for t in SAMPLES:
        s  = near(score_samples, start + t)
        tu = near(time_samples, start + t)
        lf = near(left_samples, start + t)
        print(f"  t≈{t:4.1f}s : score={s!r:<6} time_usage={tu!r:<6} left={lf!r}")

    print()
    print("【B. /score 全部记录（按时间）】")
    for t in sorted(score_samples):
        print(f"  t={t - start:6.2f}s  score={score_samples[t]}")

    print()
    print("【C. /left_actors 全部记录（按时间）】")
    for t in sorted(left_samples):
        print(f"  t={t - start:6.2f}s  left={left_samples[t]}")

    print()
    print("【D. 6 条颜色流是否真的活着】")
    for name, topic in COLOR_TOPICS.items():
        if name in first_seen:
            at, cls, x, y = first_seen[name]
            print(f"  {name:>5s} ({topic})  首达 @ {at - start:.2f}s  cls={cls!r:>10s}  x={x:.2f} y={y:.2f}")
        else:
            print(f"  {name:>5s} ({topic})  10s 内**没有收到任何消息**")

    print()
    print("【E. 反推 sensor_cost（用最早一次 score 实测值）】")
    if score_samples:
        t0, s0 = min(score_samples.items(), key=lambda kv: kv[0])
        sc = (-s0) / 0.003
        print(f"  最早 score={s0} @ t={t0 - start:.2f}s")
        print(f"  反推 sensor_cost ≈ {sc:.1f}")
        if sc > 5900:
            print(f"  ⚠ 警告：官方默认 7 个 sensor 全开 = 5900，{sc:.0f} 已超出")
            print(f"     要么 sensor_cost 公式被改了，要么 7 个 rosparam 被覆盖成了大数")
        if sc < 0:
            print(f"  ⚠ 警告：sensor_cost 是负的 → 早期 score > 0 → 已有 find/track/target 触发")

    print()
    print("【F. 反推 (find, target) 组合】")
    if score_samples:
        s_now = list(score_samples.values())[-1]
        print(f"  最新 score={s_now}")
        sc = max(0, (-(list(score_samples.values())[0])) / 0.003) if score_samples else 0
        best = []
        for f in range(0, 7):
            for tg in range(0, 7):
                guess = f * 50 + tg * 100 - sc * 0.003
                if abs(guess - s_now) < 1.0:
                    best.append((f, tg, guess))
        if best:
            for f, tg, g in best[:10]:
                print(f"    (find={f}, target={tg}, sc≈{sc:.0f}) => {g:.1f}")
        else:
            print("    在 0~6 范围内没有匹配的整数组合")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
