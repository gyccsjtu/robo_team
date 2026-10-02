#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓实飞真实帧落盘（供离线重放定位 shift 恒 -2.20）。

合成场景已被证明复现不出饱和（19点/8m 只得 shift=-0.03），
所以必须抓**真实**的 scan + 位姿，回头离线逐级重放。

用法：
  python3 dump_scan.py --uav typhoon_h480_0 --out /tmp/scan_dump.pkl --every 10 --dur 150
"""
import argparse
import pickle
import sys
import time

import rospy
from sensor_msgs.msg import LaserScan

try:
    from gazebo_msgs.msg import ModelStates
    HAVE_GZ = True
except Exception:
    HAVE_GZ = False


class Dumper(object):
    def __init__(self, uav, out, every):
        self.uav = uav
        self.out = out
        self.every = max(1, every)
        self.frames = []
        self.n = 0
        self.pose = None          # (x, y, z, qx, qy, qz, qw)

    def on_scan(self, m):
        self.n += 1
        if self.n % self.every:
            return
        if self.pose is None:
            return
        self.frames.append({
            'n': self.n,
            't': time.time(),
            'ranges': list(m.ranges),
            'angle_min': m.angle_min,
            'angle_increment': m.angle_increment,
            'range_min': m.range_min,
            'range_max': m.range_max,
            'pose': tuple(self.pose),
        })
        if len(self.frames) % 20 == 0:
            rospy.loginfo('[dump] 已抓 %d 帧' % len(self.frames))

    def on_states(self, m):
        try:
            i = list(m.name).index(self.uav)
        except (ValueError, AttributeError):
            return
        p = m.pose[i]
        self.pose = (p.position.x, p.position.y, p.position.z,
                     p.orientation.x, p.orientation.y,
                     p.orientation.z, p.orientation.w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uav', default='typhoon_h480_0')
    ap.add_argument('--out', default='/tmp/scan_dump.pkl')
    ap.add_argument('--every', type=int, default=10)
    ap.add_argument('--dur', type=float, default=150.0)
    a = ap.parse_args()

    rospy.init_node('dump_scan', anonymous=True)
    d = Dumper(a.uav, a.out, a.every)
    rospy.Subscriber('/%s/scan' % a.uav, LaserScan, d.on_scan, queue_size=1)
    if HAVE_GZ:
        rospy.Subscriber('/gazebo/model_states', ModelStates, d.on_states,
                         queue_size=1)
    else:
        rospy.logwarn('无 gazebo_msgs，位姿将缺失')

    rospy.loginfo('[dump] 开始抓帧 -> %s（每 %d 帧存 1，最长 %.0fs）'
                  % (a.out, a.every, a.dur))
    t0 = time.time()
    while not rospy.is_shutdown() and (time.time() - t0) < a.dur:
        time.sleep(0.5)
    with open(a.out, 'wb') as f:
        pickle.dump(d.frames, f)
    rospy.loginfo('[dump] 落盘 %d 帧 -> %s' % (len(d.frames), a.out))
    if not d.frames:
        rospy.logwarn('[dump] ⚠ 一帧都没抓到！检查话题名 / 位姿是否就绪')
        sys.exit(1)


if __name__ == '__main__':
    main()
