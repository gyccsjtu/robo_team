#!/bin/bash
# ============================================================================
# 官方 RoboCup 多旋翼集群搜索 —— 真机仿真环境变量
# 用途：在官方 base.world 里起飞 typhoon_h480（PX4 SITL + MAVROS）
# 用法：source env_robocup.sh
# ============================================================================
# 注意 ~ 下多处是 /data 的软链，实际上只有一份代码。

export ROBOCOP_ROOT="${ROBOCUP_XTDRONE:-$HOME/XTDrone/robocup}"

# ---- ROS ----
# 路径可用环境变量覆盖；缺失时**跳过而不是报错**，便于在不同机器上复用。
ROS_SETUP="${ROBOCUP_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
WS_SETUP="${ROBOCUP_WS_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
# shellcheck disable=SC1090
[ -f "$ROS_SETUP" ] && source "$ROS_SETUP"
# shellcheck disable=SC1090
[ -f "$WS_SETUP" ] && source "$WS_SETUP"

# ---- 让官方 launch 里的 $(find px4) 生效 ----
# PX4_Firmware 自带 package.xml(name=px4)，且其 launch/ 目录已被放入 XTDrone 的启动文件
export ROS_PACKAGE_PATH="$HOME/PX4_Firmware:${ROS_PACKAGE_PATH:-}"

# ---- Gazebo 模型与插件 ----
# 官方 base.world 依赖 model://walker/walk_N.dae、city_terrain、asphalt_plane 等
export GAZEBO_MODEL_PATH="$HOME/XTDrone/sitl_config/models:$HOME/PX4_Firmware/Tools/sitl_gazebo/models:${GAZEBO_MODEL_PATH:-}"
# 官方 actor 插件 + PX4 编译出的 37 个 gazebo 插件（电机模型 / mavlink 接口 / 云台 / 相机）
export GAZEBO_PLUGIN_PATH="$HOME/catkin_ws/devel/lib:$HOME/PX4_Firmware/build/px4_sitl_default/build_gazebo:${GAZEBO_PLUGIN_PATH:-}"

# ---- 已知致命坑 ----
# 置空在线模型库：否则 gzserver 会去 models.gazebosim.org 拉模型并卡死，
# 表现为世界加载不出来、插件/相机全不初始化。
export GAZEBO_MODEL_DATABASE_URI=""

# ---- 相机必需：虚拟显示 ----
# Gazebo 的 camera 传感器需要 GL 上下文才能真正渲染。无 DISPLAY 时
# 传感器会在 /gazebo/default/.../camera/image 上建出来，但
# libgazebo_ros_camera.so 不会渲染、也不会建 ROS 图像话题
# —— 表现为「无人机和相机都在，但 /cgo3_camera/image_raw 就是不存在」。
# 此前所有能出图的脚本都开了 Xvfb，这一条绝不能漏。
#
# 🔴 2026-10-01 修复：本机未安装 Xvfb（command -v 失败被 >/dev/null 吞掉），
# 却仍 export DISPLAY=:95 指向不存在的显示——gzserver spawn 带相机的飞机
# 时渲染初始化失败，直接 exit 255（全链路三次必崩的根因）。
# Xvfb 缺失时回退到启动时继承的真实 DISPLAY（都没有则试 :0）。
ROBOCUP_ORIG_DISPLAY="${DISPLAY:-}"
export DISPLAY=:95
if ! pgrep -f "Xvfb :95" >/dev/null 2>&1; then
  if command -v Xvfb >/dev/null 2>&1; then
    Xvfb :95 -screen 0 1280x1024x24 >/dev/null 2>&1 &
    sleep 2
    echo "[env] 已启动 Xvfb :95"
  elif [ -n "$ROBOCUP_ORIG_DISPLAY" ]; then
    export DISPLAY="$ROBOCUP_ORIG_DISPLAY"
    echo "[env] Xvfb 未安装，DISPLAY 回退继承值 $DISPLAY"
  else
    export DISPLAY=:0
    echo "[env] Xvfb 未安装且无继承 DISPLAY，回退 :0（桌面会话）"
  fi
fi
echo "[env] DISPLAY=$DISPLAY  Xvfb=$(pgrep -fc 'Xvfb :95')"

# 官方 aircraft 的仿真模型名
export PX4_SIM_MODEL=typhoon_h480

# 官方世界相对路径依赖（control_actor.py 用相对路径读 black_box.txt）
export ROBOCOP_ACTOR_CWD="$ROBOCOP_ROOT"
