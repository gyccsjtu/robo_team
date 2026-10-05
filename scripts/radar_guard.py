#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""雷达避险层 —— 把飞控的"期望目标点"过一遍 2D 雷达，返回避障后的子目标。

设计约束（2026-10-03）
--------------------
1. **绝不发 setpoint**。本层只把 (gx, gy) 换成 (sx, sy)，setpoint 发布者
   永远只有 yolo_flyer 一个 —— 遵守 ~/robocup_real 的"铁律 4：同一时刻只能
   有一个 setpoint 发布者"（两个发布者会互相覆盖成 type_mask=39）。
2. **决策逻辑一行不复制**。全部委托 radar_avoid.subgoal_from_scan ——
   该函数是 ~/robo_team/radar 里声明的"避障唯一真相来源"，实飞
   (RadarPilot.step_toward) 与离线闭环仿真都调它，避免多份逻辑漂移。
3. **可无损关闭**。环境变量 RADAR_GUARD=0 或雷达无数据/超时 ⇒ 原样透传，
   行为与没装本层完全一致。

参数与被保护对象的一致性（实机核对过）
------------------------------------
  机型 SDF  : models/typhoon_h480_lidar/typhoon_h480_lidar.sdf
              <include>model://hokuyo_lidar</include> + <pose>0 0 0.080 0 0 0</pose>
  传感器    : hokuyo_lidar/model.sdf → sensor "laser" type=ray
              samples=512, min_angle=-3.14, max_angle=3.14, range 0.5~20m
              plugin libgazebo_ros_laser.so, topicName=scan
  ⇒ MOUNT_DEFAULT=(0,0,0.080)、BEAM_COUNT=512、RANGE 0.5~20 全部对上。
"""

from __future__ import division, print_function

import math
import os
import sys
import threading
import time

import rospy
from sensor_msgs.msg import LaserScan

# 雷达模块目录（VM 上是 ~/robo_team/radar；可用 RADAR_DIR 覆盖）
_RADAR_DIR = os.environ.get("RADAR_DIR", os.path.expanduser("~/robo_team/radar"))
if _RADAR_DIR not in sys.path:
    sys.path.insert(0, _RADAR_DIR)

try:
    from radar_avoid import (subgoal_from_scan, SideCommit, MOUNT_DEFAULT)
    _IMPORT_ERR = None
except Exception as _e:                                    # pragma: no cover
    subgoal_from_scan = None
    SideCommit = None
    MOUNT_DEFAULT = (0.0, 0.0, 0.080)
    _IMPORT_ERR = _e

# 雷达数据新鲜度：超过这么久没新帧就透传（不拿过期扫描当真）
SCAN_STALE_S = 1.0
# 雷达抽稀步长：512 → 256 条，抗单条跳变、省一半算力（radar_avoid 同款默认）
STRIDE = 2


class RadarGuard(object):
    """订阅一帧 2D 雷达，提供 guard() 供飞控逐帧调用。"""

    def __init__(self, scan_topic, mount=MOUNT_DEFAULT, stride=STRIDE,
                 enabled=True):
        self.enabled = bool(enabled) and (subgoal_from_scan is not None)
        self.mount = mount
        self.stride = stride
        self._msg = None
        self._t_msg = 0.0
        self._commit = SideCommit() if SideCommit is not None else None
        self._lock = threading.Lock()
        self.n_guard = 0        # 实际改写过目标的次数（诊断用）
        self.n_pass = 0         # 透传次数
        self.n_err = 0
        self.n_call = 0         # 总调用次数（用于周期打印）
        self.last_shift = 0.0
        self._sub = None
        if not self.enabled:
            rospy.logwarn("[rg] 雷达层未启用（enabled=%s import_err=%s）",
                          enabled, _IMPORT_ERR)
            return
        self._sub = rospy.Subscriber(scan_topic, LaserScan, self._cb,
                                     queue_size=1)
        rospy.loginfo("[rg] 雷达层已挂：%s（mount=%s stride=%d）",
                      scan_topic, tuple(mount), stride)

    def _cb(self, m):
        with self._lock:
            self._msg = m
            self._t_msg = rospy.get_time()

    @property
    def has_scan(self):
        with self._lock:
            return (self._msg is not None
                    and rospy.get_time() - self._t_msg <= SCAN_STALE_S)

    def guard(self, gx, gy, yaw, pos_xy):
        """给定期望目标 (gx,gy)、当前机头 yaw、当前世界位置，返回避障子目标。

        任何异常/无数据都原样返回 (gx, gy) —— 永不因为避障把飞机"卡住"。
        """
        if not self.enabled:
            self.n_pass += 1
            return gx, gy
        with self._lock:
            m = self._msg
            t = self._t_msg
        if m is None or rospy.get_time() - t > SCAN_STALE_S:
            self.n_pass += 1
            return gx, gy
        try:
            sx, sy, sh, _nr, _blk = subgoal_from_scan(
                m, float(yaw), (float(pos_xy[0]), float(pos_xy[1])),
                (float(gx), float(gy)), self.mount,
                stride=self.stride, commit=self._commit, attitude=None)
        except Exception as e:                              # pragma: no cover
            self.n_err += 1
            if self.n_err <= 5:
                rospy.logwarn("[rg] subgoal_from_scan 异常：%s", e)
            return gx, gy
        # 子目标必须在场地内、且不能是 NaN（防上游边界情况）
        if not (math.isfinite(sx) and math.isfinite(sy)):
            self.n_err += 1
            return gx, gy
        try:
            self.last_shift = float(sh)
        except Exception:
            self.last_shift = 0.0
        if abs(self.last_shift) > 1e-6:
            self.n_guard += 1
        else:
            self.n_pass += 1
        self.n_call += 1
        # 每 100 次调用打印一次（20Hz ⇒ 约 5s 一条），便于飞后核对避障是否真在介入
        if self.n_call % 100 == 0:
            rospy.loginfo("[rg] %d 次调用：改向 %d / 透传 %d / 异常 %d | "
                          "最近横向修正 %.2fm | 目标(%.1f,%.1f)→子目标(%.1f,%.1f)",
                          self.n_call, self.n_guard, self.n_pass, self.n_err,
                          self.last_shift, gx, gy, sx, sy)
        return sx, sy

    def stats(self):
        return ("guard=%d pass=%d err=%d shift=%.2f"
                % (self.n_guard, self.n_pass, self.n_err, self.last_shift))
