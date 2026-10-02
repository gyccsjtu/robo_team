#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Occupancy Grid ROS 节点 - 基于激光雷达的在线占据栅格

功能：
  - 订阅雷达 /scan 话题
  - 订阅无人机位姿 /local_pose 话题
  - 发布 1m 分辨率占据栅格
  - 发布 7m 降采样粗网格（供队友使用）

用法：
  rosrun robocup_navigation occupancy_node.py _uav_id:=1
"""

import math
import os
import sys

import rospy
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

# 导入占据栅格模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from occupancy_online import OnlineOccupancy, OccupancyPublisher


class OccupancyROSNode:
    """Occupancy Grid ROS 节点"""

    def __init__(self, uav_id=1):
        self.uav_id = uav_id
        self.ns = f"/uav{uav_id}"

        # 占据栅格实例
        self.occ = OnlineOccupancy()

        # 无人机状态
        self.latest_pose = None
        self.latest_yaw = None
        self.latest_mount = (0.0, 0.0)  # 雷达安装位置

        # ROS 订阅
        rospy.Subscriber(
            f"{self.ns}/local_pose",
            PoseStamped,
            self._pose_callback
        )
        rospy.Subscriber(
            f"{self.ns}/scan",
            LaserScan,
            self._scan_callback
        )

        # ROS 发布
        self.pub_fine = rospy.Publisher(
            f"{self.ns}/occupancy_fine",
            Int8MultiArray, queue_size=1
        )
        self.pub_coarse = rospy.Publisher(
            f"{self.ns}/occupancy_coarse",
            Int8MultiArray, queue_size=1
        )

        rospy.loginfo(f"OccupancyROSNode 启动: UAV{self.uav_id}")

    def _pose_callback(self, msg):
        """无人机位姿回调"""
        self.latest_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
        # 从四元数计算偏航角
        q = msg.pose.orientation
        self.latest_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

    def _scan_callback(self, scan_msg):
        """雷达扫描回调"""
        if self.latest_pose is None or self.latest_yaw is None:
            return

        # 更新占据栅格
        pos_enu = (self.latest_pose[0], self.latest_pose[1])
        self.occ.feed(scan_msg, self.latest_yaw, pos_enu, self.latest_mount)

        # 发布 1m 细栅格
        msg_fine = Int8MultiArray()
        msg_fine.data = list(self.occ.hits)
        self.pub_fine.publish(msg_fine)

        # 降采样到 7m 并发布
        from occupancy_online import OccupancyDownsampler
        ds = OccupancyDownsampler()
        coarse_hits, nx, ny = ds.downsample(
            self.occ.hits, self.occ.nx, self.occ.ny, coarse_cell=7.0
        )

        msg_coarse = Int8MultiArray()
        msg_coarse.data = list(coarse_hits)
        self.pub_coarse.publish(msg_coarse)


def main():
    rospy.init_node("occupancy_node", anonymous=True)

    uav_id = rospy.get_param("~uav_id", 1)

    node = OccupancyROSNode(uav_id=uav_id)

    rospy.loginfo(f"Occupancy 节点运行中: /uav{uav_id}/scan -> /uav{uav_id}/occupancy_coarse")
    rospy.spin()


if __name__ == "__main__":
    main()
