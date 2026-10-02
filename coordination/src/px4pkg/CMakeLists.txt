cmake_minimum_required(VERSION 3.0.2)
project(px4_shim)

# 云环境遗留的纯 shim 包（内部路径硬编码 /root/team_ws，本机不可用）。
# 已从 project(px4) 改名为 px4_shim，避免在 ROS 包解析中遮蔽真正的固件包
# /home/gycc/PX4_Firmware（包名也叫 px4，提供 launch/ 与 SITL 二进制）。
# 不编译任何代码。

find_package(catkin REQUIRED)

catkin_package()
