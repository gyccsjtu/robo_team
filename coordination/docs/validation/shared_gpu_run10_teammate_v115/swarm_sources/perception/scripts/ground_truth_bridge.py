#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ground_truth_bridge.py
======================
补齐 XTDrone 通信桥缺失的两类话题，使 actor 控制脚本能正常工作：

1) UAV 位姿转发：
     /typhoon_h480_N/mavros/local_position/pose (PoseStamped, MAVROS 真值)
        -> /xtdrone/typhoon_h480_N/ground_truth/odom (Odometry)
   actor 脚本订阅前者，但 XTDrone 官方 multirotor_communication.py 不发布它
   （这是官方脚本与官方 actor 脚本间的设计漏洞，本节点用 MAVROS 的
   local_position/pose 补齐。MAVROS 输出的 ENU 局部位置即作为 ground truth。）

2) Actor 位姿转发：
     /gazebo/model_states 中 actor_N 的 pose
        -> /actor_N/pose (PoseStamped)
   Gazebo 11 自带的 libgazebo_ros_actor_plugin.so 不发布 actor 位姿，
   但 model_states topic 由 gzserver 持续发布，包含所有 model 的位姿。
   官方脚本和 m2 修复都依赖 /actor_N/pose，所以这里从 model_states 拆出来。

参数: --id 0 (本节点实例的 UAV id), --actor-id 不传则与 id 相同；
      也可以 --actor-id 5 表示桥 actor_5 但 UAV id 仍然是默认映射（按需）。

设计：
  - 每个 UAV 起一个进程，分别做 UAV odom 转发（无 actor）
  - 每个 actor 起一个进程，做 actor pose 转发
  - 用 rospy.Timer 周期性从 model_states 取 actor pose（频率与 cmd_motion 一致 10Hz）

退出：SIGINT/SIGTERM；运行后输出 [bridge] PID xxx ready 字样，便于脚本判定就绪。
"""
import argparse
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from gazebo_msgs.msg import ModelStates


def make_odom_from_pose(pose_stamped: PoseStamped) -> Odometry:
    """PoseStamped (MAVROS local_position/pose) -> Odometry (xtdrone ground_truth/odom)"""
    odom = Odometry()
    odom.header = pose_stamped.header
    odom.pose.pose = pose_stamped.pose
    # twist 留零；actor 脚本里 `cmd_uav*_pose_callback` 只读 pose。
    odom.twist.twist.linear.x = 0.0
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.linear.z = 0.0
    return odom


def uav_bridge(vehicle_type: str, uav_id: int):
    """把 /<vehicle_type>_<id>/mavros/local_position/pose -> /xtdrone/<vehicle_type>_<id>/ground_truth/odom"""
    rospy.init_node(f"{vehicle_type}_{uav_id}_gt_bridge", anonymous=True)
    in_topic = f"/{vehicle_type}_{uav_id}/mavros/local_position/pose"
    out_topic = f"/xtdrone/{vehicle_type}_{uav_id}/ground_truth/odom"
    pub = rospy.Publisher(out_topic, Odometry, queue_size=10)

    rospy.loginfo(f"[gt] {vehicle_type}_{uav_id}: bridging {in_topic} -> {out_topic}")
    print(f"[gt] {vehicle_type}_{uav_id}: bridge ready", flush=True)

    def cb(msg):
        pub.publish(make_odom_from_pose(msg))

    rospy.Subscriber(in_topic, PoseStamped, cb, queue_size=1)
    rospy.spin()


class ActorPoseBridge:
    """从 /gazebo/model_states 中按名字挑出 actor_<id> 的 pose，发布到 /actor_<id>/pose"""

    def __init__(self, actor_id: int, rate_hz: float = 10.0):
        self.actor_id = actor_id
        self.target_name = f"actor_{actor_id}"
        self.pub = rospy.Publisher(f"/{self.target_name}/pose", PoseStamped, queue_size=10)
        self.latest_msg = PoseStamped()
        self.latest_msg.header.frame_id = "world"
        self.have_data = False
        rospy.loginfo(f"[act-bridge] actor_{actor_id}: subscribing /gazebo/model_states")
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate_hz), self._tick)

    def _cb(self, msg: ModelStates):
        if self.target_name not in msg.name:
            return
        idx = msg.name.index(self.target_name)
        self.latest_msg.pose = msg.pose[idx]
        self.latest_msg.header.stamp = rospy.Time.now()
        self.have_data = True

    def _tick(self, _evt):
        if not self.have_data:
            return
        self.pub.publish(self.latest_msg)


def actor_bridge(actor_id: int):
    """把 Gazebo model_states 中的 actor_<id> pose 拆出来发到 /actor_<id>/pose"""
    rospy.init_node(f"actor_{actor_id}_pose_bridge", anonymous=True)
    ActorPoseBridge(actor_id, rate_hz=10.0)
    rospy.loginfo(f"[act-bridge] actor_{actor_id}: ready")
    print(f"[act-bridge] actor_{actor_id}: bridge ready", flush=True)
    rospy.spin()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["uav", "actor"], required=True)
    parser.add_argument("--id", type=int, required=True, help="UAV / actor index 0..5")
    parser.add_argument("--vehicle-type", default="typhoon_h480")
    args = parser.parse_args()

    if args.mode == "uav":
        uav_bridge(args.vehicle_type, args.id)
    else:
        actor_bridge(args.id)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass