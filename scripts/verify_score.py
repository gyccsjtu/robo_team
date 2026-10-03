#!/usr/bin/env python3
# coding: utf-8
"""
verify_score.py —— 我们自己的裁判复核器（不改官方 score_cal.py 一行）

存在的唯一理由：官方判据取真值用 /gazebo/get_model_state 服务调用，实测高负载下
可取率仅 16~80%（超时返回全零 → 判据 x²+y²!=0 失败 → actors_pos=None → 回调静默
丢弃 → 白白丢分）。本脚本判据与官方逐条对齐，但真值改从 /gazebo/model_states 话题取
（实测 100% 完整），用来回答两个问题：
  1) 我们这一场按规则到底该得多少分（find/track/target 各自进度）
  2) 官方 /score 与规则分差多少，差在哪

判据（逐条抄自 robocup/score_cal.py）：
  发现：|msg.pos - actor真实pos| < 1m  且  消息间隔 <= 1.0s  且  连续 >= 15.0s
        → 计 find(+50) 与 target(+100)
  跟踪：任一 UAV 距 actor <= 10m 连续 >= 20.0s → +80（同一目标只记一次）
  越界：UAV z > 6.0m → 官方直接判 0 分
  计分：find*50 + track*80 + target*100 - sensor_cost*3e-3 - uav_loss*100
        全部 6 个都找到（官方删光）时官方改用 2580 - 用时

本脚本**不删 actor**（删除是官方裁判的职责），只观测。
"""
import sys
import time

import rospy
from std_msgs.msg import Int16
from gazebo_msgs.msg import ModelStates
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo

ACTOR_NUM = 6
ERR_THRESHOLD = 1.0            # m
DETECTION_INTERVAL = 1.0       # s
DETECTION_DURATION = 15.0      # s
TRACKING_DURATION = 20.0       # s
TRACKING_DISTANCE = 10.0       # m
MAX_UAV_ALTITUDE = 6.0         # m
# 官方 color→actor_id 映射（red 有两个球：red1→actor_5，red2→actor_4）
COLOR_IDS = {
    'green': [0], 'blue': [1], 'brown': [2], 'white': [3],
    'red1': [5], 'red2': [4],
}
TOPIC_MAP = {
    '/actor_green_info': 'green', '/actor_blue_info': 'blue',
    '/actor_brown_info': 'brown', '/actor_white_info': 'white',
    '/actor_red1_info': 'red1', '/actor_red2_info': 'red2',
}
UAV_PREFIX = 'typhoon_h480_'


class Truth(object):
    """/gazebo/model_states 快照：actor 真值 + UAV 位置"""

    def __init__(self):
        self.lock = __import__('threading').Lock()
        self.actors = {}          # id -> (x, y, z)
        self.uavs = {}            # id -> (x, y, z)
        self.stamp = 0.0
        self.frames = 0

    def cb(self, msg):
        with self.lock:
            self.frames += 1
            # gazebo_msgs/ModelStates 无 header 字段，用本地时钟打时间戳
            self.stamp = rospy.get_time()
            for name, pose in zip(msg.name, msg.pose):
                if name.startswith('actor_'):
                    try:
                        aid = int(name.split('_')[1])
                    except ValueError:
                        continue
                    p = pose.position
                    self.actors[aid] = (p.x, p.y, p.z)
                elif name.startswith(UAV_PREFIX):
                    tail = name[len(UAV_PREFIX):]
                    if tail.isdigit():
                        p = pose.position
                        self.uavs[int(tail)] = (p.x, p.y, p.z)

    def get_actor(self, aid):
        with self.lock:
            return self.actors.get(aid)

    def get_uavs(self):
        with self.lock:
            return dict(self.uavs)

    def is_live(self, max_age=2.0):
        with self.lock:
            if self.frames == 0:
                return False
            return (rospy.get_time() - self.stamp) <= max_age


class Verifier(object):
    def __init__(self):
        self.truth = Truth()
        # 每个 actor 的发现状态机（与官方同构）
        self.count_flag = [False] * ACTOR_NUM
        self.find_time = [0.0] * ACTOR_NUM
        self.arrive_time = [0.0] * ACTOR_NUM
        self.found = set()            # 达 15s 连续 = 官方会删机计 target
        self.discover_start = set()   # 首帧误差达标 = 计 find
        self.deleted = set()          # 已被官方裁判删掉（真值消失）
        self.tracking_start = [None] * ACTOR_NUM
        self.tracked = set()
        self.max_alt = 0.0
        self.uav_seen = {}
        self.official_score = None
        self.best_err = [None] * ACTOR_NUM

    # ---------- 官方 _process_actor_detection 的等价实现 ----------
    def on_info(self, tag, msg):
        aid_list = COLOR_IDS.get(tag, [])
        if not aid_list:
            return
        if not getattr(msg, 'cls', ''):
            return
        now = rospy.get_time()
        for aid in aid_list:
            if not self.truth.is_live():
                continue
            pos = self.truth.get_actor(aid)
            prev = self.arrive_time[aid]
            self.arrive_time[aid] = now
            if pos is None:
                # 官方语义：真值取不到 → 本帧作废并重置计时
                self.count_flag[aid] = False
                self.find_time[aid] = 0.0
                continue
            d = ((msg.x - pos[0]) ** 2 + (msg.y - pos[1]) ** 2) ** 0.5
            if self.best_err[aid] is None or d < self.best_err[aid]:
                self.best_err[aid] = d
            continuous = (prev == 0.0) or (now - prev <= DETECTION_INTERVAL)
            if d >= ERR_THRESHOLD or not continuous:
                self.count_flag[aid] = False
                self.find_time[aid] = 0.0
                continue
            if not self.count_flag[aid]:
                self.count_flag[aid] = True
                self.find_time[aid] = now
                self.discover_start.add(aid)
                continue
            if now - self.find_time[aid] >= DETECTION_DURATION:
                self.found.add(aid)

    # ---------- 官方 _update_tracking 的等价实现 ----------
    def update_tracking(self, now):
        uavs = self.truth.get_uavs()
        for aid in range(ACTOR_NUM):
            pos = self.truth.get_actor(aid)
            if pos is None:
                self.tracking_start[aid] = None
                continue
            if aid in self.found:
                continue
            in_range = any(
                (u[0] - pos[0]) ** 2 + (u[1] - pos[1]) ** 2 <= TRACKING_DISTANCE ** 2
                for u in uavs.values())
            if in_range:
                if self.tracking_start[aid] is None:
                    self.tracking_start[aid] = now
                elif now - self.tracking_start[aid] >= TRACKING_DURATION:
                    if aid not in self.tracked:
                        self.tracked.add(aid)
            else:
                self.tracking_start[aid] = None

    def score(self):
        return (len(self.discover_start) * 50 + len(self.tracked) * 80
                + len(self.found) * 100)

    def report(self, elapsed):
        now = rospy.get_time()
        lines = []
        lines.append('=' * 78)
        lines.append('真值通道: /gazebo/model_states  frames=%d  live=%s'
                     % (self.truth.frames, self.truth.is_live()))
        uavs = self.truth.get_uavs()
        alt_txt = []
        for uid in sorted(uavs):
            alt_txt.append('uav%d z=%.2f' % (uid, uavs[uid][2]))
        if alt_txt:
            self.max_alt = max(self.max_alt, max(u[2] for u in uavs.values()))
        lines.append('UAV: %s   最高=%.2f%s'
                     % (' '.join(alt_txt), self.max_alt,
                        '  [!! 超 6m 官方判 0]' if self.max_alt > MAX_UAV_ALTITUDE else ''))
        lines.append('-' * 78)
        lines.append('actor  状态        连续(s)  最近间隔(s)  最小误差(m)')
        for aid in range(ACTOR_NUM):
            if aid in self.found:
                st = '已达标(官方删机)'
            elif self.count_flag[aid]:
                st = '连续播报中'
            elif aid in self.discover_start:
                st = '曾达标后中断'
            elif self.truth.get_actor(aid) is None:
                st = '真值缺失'
            else:
                st = '未发现'
            cont = (now - self.find_time[aid]) if self.count_flag[aid] else 0.0
            last = self.arrive_time[aid]
            gap = (now - last) if last else float('nan')
            be = '%.2f' % self.best_err[aid] if self.best_err[aid] is not None else '-'
            lines.append('  %d    %-12s %6.1f    %8.2f      %s'
                         % (aid, st, cont, gap, be))
        lines.append('-' * 78)
        lines.append('发现 find=%d(+%d)  找到 target=%d(+%d)  跟踪 track=%d(+%d)'
                     % (len(self.discover_start), len(self.discover_start) * 50,
                        len(self.found), len(self.found) * 100,
                        len(self.tracked), len(self.tracked) * 80))
        lines.append('规则应得分 = %d    用时 = %.1fs    官方 /score = %s'
                     % (self.score(), elapsed,
                        self.official_score if self.official_score is not None else '?'))
        if len(self.found) == ACTOR_NUM:
            lines.append('★ 6 个全找到：官方最终分 = 2580 - 用时 = %.1f' % (2580.0 - elapsed))
        lines.append('=' * 78)
        return '\n'.join(lines)


def main():
    uav_type = (sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-')
                else 'typhoon_h480')
    rospy.init_node('verify_score', anonymous=True, disable_signals=True)
    v = Verifier()
    rospy.Subscriber('/gazebo/model_states', ModelStates, v.truth.cb, queue_size=1)
    rospy.Subscriber('/score', Int16, lambda m: setattr(v, 'official_score', m.data),
                     queue_size=1)
    for topic, tag in TOPIC_MAP.items():
        rospy.Subscriber(topic, ActorInfo,
                         lambda msg, t=tag: v.on_info(t, msg), queue_size=20)

    t0 = rospy.get_time()
    rate = rospy.Rate(2)
    last_print = 0.0
    while not rospy.is_shutdown():
        v.update_tracking(rospy.get_time())
        now = rospy.get_time()
        if now - last_print >= 2.0:
            print(v.report(now - t0), flush=True)
            last_print = now
        rate.sleep()


if __name__ == '__main__':
    main()
