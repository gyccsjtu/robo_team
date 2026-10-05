#!/usr/bin/env python3
# coding: utf-8
"""测距标定：同时刻采样「感知报告坐标」与「Gazebo 真值」，求比例。

背景：官方判据是"误差<1m 连续 15s"。实测远距离偏差极大
（actor_0 真值 (26,-26)，机在 (0,-3) 真实 34.7m，感知报 21.3m）。
本脚本在若干秒内反复采样，算出 report/true 的比例与绝对误差，
用来判断：(a) 是固定比例（可标定）还是随机噪声（只能靠距离闸门规避）。
"""
import math
import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped

U = 'typhoon_h480_0'
COLOR_IDS = {'green': 0, 'blue': 1, 'brown': 2, 'white': 3, 'red1': 5, 'red2': 4}
TOPICS = ['/actor_green_info', '/actor_blue_info', '/actor_brown_info',
          '/actor_white_info', '/actor_red1_info', '/actor_red2_info']

rospy.init_node('range_calib', anonymous=True, disable_signals=True)


def snapshot():
    m = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=8)
    truth, uav = {}, None
    for n, p in zip(m.name, m.pose):
        if n == U:
            uav = (p.position.x, p.position.y, p.position.z)
        if n.startswith('actor_'):
            truth[int(n.split('_')[1])] = (p.position.x, p.position.y)
    return truth, uav


def main(duration=60.0):
    print('采样 %ds：color  上报距离  真实距离  比例  绝对误差' % duration)
    print('-' * 62)
    t0 = time.time()
    ratios = {}
    while time.time() - t0 < duration and not rospy.is_shutdown():
        truth, uav = snapshot()
        if uav is None:
            continue
        for topic, tag in zip(TOPICS, ['green', 'blue', 'brown', 'white', 'red1', 'red2']):
            try:
                msg = rospy.wait_for_message(topic, PoseStamped, timeout=0.4)
            except Exception:
                continue
            # ActorInfo 与 PoseStamped 字段布局不同，这里只取 x,y
            rx, ry = msg.pose.position.x, msg.pose.position.y
            aid = COLOR_IDS[tag]
            if aid not in truth:
                continue
            d_rep = math.hypot(rx - uav[0], ry - uav[1])
            d_true = math.hypot(truth[aid][0] - uav[0], truth[aid][1] - uav[1])
            err = math.hypot(rx - truth[aid][0], ry - truth[aid][1])
            r = d_rep / d_true if d_true > 0.01 else float('nan')
            ratios.setdefault(tag, []).append((d_true, d_rep, r, err))
            print('%-6s %8.2f %9.2f %6.3f %9.2f' % (tag, d_rep, d_true, r, err))
        time.sleep(1.0)
    print()
    print('=== 按颜色汇总 ===')
    for tag in sorted(ratios):
        v = ratios[tag]
        near = [x for x in v if x[0] <= 12.0]
        print('%s: %d 样本  距离范围 %.1f~%.1fm' % (tag, len(v),
              min(x[0] for x in v), max(x[0] for x in v)))
        if near:
            print('   近距离(<12m) %d 样本: 误差 %.2f~%.2fm  比例 %.3f~%.3f'
                  % (len(near), min(x[3] for x in near), max(x[3] for x in near),
                     min(x[2] for x in near), max(x[2] for x in near)))


if __name__ == '__main__':
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 60.0)
