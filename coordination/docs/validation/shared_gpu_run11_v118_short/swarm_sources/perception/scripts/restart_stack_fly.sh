#!/bin/bash
# ============================================================================
# 全栈重启（真飞验证版）
#   世界(gzserver) -> 无人机(PX4+MAVROS) -> 6 actor -> 感知
#
# 【为什么必须重启世界，而不是只重启 PX4】
#   gzserver 里残留着旧的 typhoon_h480_0 模型，新 PX4 spawn 同名模型会报
#     Spawn status: SpawnModel: Failure - entity already exists.
#   spawn 失败后 PX4 卡在
#     INFO [simulator] Waiting for simulator to accept connection on TCP port 4560
#   永远等不到 Gazebo 侧连过来 => mavlink 不广播 => MAVROS connected=False
#   => ARM/OFFBOARD 全部无效。
#   restart_uav.sh 注释里写了"必须先 delete_model"，但实现只用了
#   set_model_state（只移动、不删除），所以救不了残留；而 rosservice
#   /gazebo/delete_model 会让 gzserver 段错误(exit 139)，世界连带 actor 一起没。
#   结论：唯一稳的路是重启 gzserver（世界），让模型世界清空后重新 spawn。
#
# 【为什么不启独立的 yolov11_ros 节点】
#   perception_real.py 自带 YOLO 推理且在主循环里同步跑，发布频率 ≡ 循环频率。
#   再起一个独立 yolo_v11 节点会抢 CPU(约 0.5 核)，把感知主循环从 ~5Hz
#   拖到 0.75~1.6Hz（09-20 定位）。验收/真飞一律不要起它。
#
# 日志：/tmp/world3.log /tmp/uav3.log /tmp/actor_*.log /tmp/percep_fly.log
# 本次运行日志：/tmp/rs_out.log   标志：STACK_FLY_OK / STACK_FLY_FAIL
# ============================================================================
set +e

# ---- 路径解析（全部可用环境变量覆盖，默认按本仓库布局）----
#   ROOT      : perception/ 目录（perception_real.py 所在处）
#               下面的 cd 只是让随后的 `python3 perception_real.py` 相对调用成立
#   XTDRONE   : XTDrone 的 robocup 目录（base.world / ObstacleAvoid 等）
#   WS_SETUP  : 已编译的 catkin 工作空间 setup.bash
#               （提供 ros_actor_cmd_pose_plugin_msgs / gazebo_msgs / mavros 等）
#   ROS_SETUP : ROS 发行版 setup.bash
ROOT="${ROBOCUP_PERCEPT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
XTDRONE="${ROBOCUP_XTDRONE:-$HOME/XTDrone/robocup}"
WS_SETUP="${ROBOCUP_WS_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
ROS_SETUP="${ROBOCUP_ROS_SETUP:-/opt/ros/noetic/setup.bash}"

cd "$ROOT" || exit 1
# shellcheck disable=SC1090
[ -f "$ROS_SETUP" ] && source "$ROS_SETUP"
# shellcheck disable=SC1090
[ -f "$WS_SETUP" ] && source "$WS_SETUP"
# shellcheck disable=SC1091
[ -f "$ROOT/scripts/env_robocup.sh" ] && source "$ROOT/scripts/env_robocup.sh" >/dev/null 2>&1
export DISPLAY="${DISPLAY:-:95}"

say() { echo "[rs] $(date +%H:%M:%S) $*"; }

say "1) 停旧组件"
pkill -f gzclient
pkill -f hover_teleport
pkill -f perception_real
pkill -f control_actor_m2
pkill -f yolo_v11
pkill -f uav_only.launch
pkill -f world_only.launch
pkill -f world_min.launch
# 必须显式杀 gzserver/gzclient：只 pkill launch 文件拦不住它们，
# 残留的旧 gzserver 会与新实例抢端口(11345) => 新世界起不来。
pkill -9 -x gzserver
pkill -9 -x gzclient
pkill -9 -f 'bin/px4'
pkill -9 -f mavros_node
pkill -f multirotor_communication
sleep 8
say "   残留: gzserver=$(pgrep -c gzserver) px4=$(pgrep -fc 'bin/px4') mavros=$(pgrep -fc mavros_node)"

say "2) 起世界（roscore + gzserver，轻量 actor_min.world）"
setsid nohup roslaunch "$ROOT/launch/world_min.launch" \
    > /tmp/world3.log 2>&1 < /dev/null &
wok=0
for i in $(seq 1 30); do
    sleep 5
    if timeout 8 rosservice list 2>/dev/null | grep -q "^/gazebo/spawn_sdf_model$"; then
        wok=1
        say "   世界服务就绪（约 $((i * 5)) 秒）"
        break
    fi
    if ! pgrep -x gzserver >/dev/null 2>&1; then
        say "   ⚠ gzserver 已退出（约 $((i * 5)) 秒），见 /tmp/world3.log"
        break
    fi
done
say "   gzserver=$(pgrep -c gzserver)  world_ready=$wok"

if [ "$wok" != "1" ]; then
    say "⚠⚠ 世界未就绪 ⇒ 中止（绝不带着半死世界去 spawn 无人机，那会留下残留模型）"
    tail -12 /tmp/world3.log | sed 's/^/[rs]   /'
    echo STACK_FLY_FAIL_WORLD
    exit 1
fi
sleep 8
say "   世界内 actor 话题数=$(timeout 10 rostopic list 2>/dev/null | grep -c 'actor_')"

say "3) 起无人机（PX4 SITL + MAVROS）"
setsid nohup roslaunch "$ROOT/launch/uav_only.launch" \
    > /tmp/uav3.log 2>&1 < /dev/null &
for i in $(seq 1 24); do
    sleep 5
    OUT=$(timeout 10 rosservice call /gazebo/get_model_state \
          "{model_name: 'typhoon_h480_0', relative_entity_name: 'world'}" 2>/dev/null)
    if echo "$OUT" | grep -q 'success: True'; then
        say "   模型已生成（约 $((i * 5)) 秒）"
        echo "$OUT" | grep -A3 position | head -4 | sed 's/^/[rs]   /'
        break
    fi
done
say "   spawn 结果自检（应无 entity already exists）:"
grep -c 'entity already exists' /tmp/uav3.log 2>/dev/null | sed 's/^/[rs]   spawn冲突计数: /'

say "   等 MAVROS 连上飞控 ..."
fcu=0
for i in $(seq 1 40); do
    S=$(timeout 8 rostopic echo -n1 /typhoon_h480_0/mavros/state 2>/dev/null)
    if echo "$S" | grep -q 'connected: True'; then
        fcu=1
        echo "$S" | grep -E 'connected|armed|mode|system_status' | sed 's/^/[rs]   /'
        break
    fi
    [ $((i % 4)) -eq 0 ] && say "   等 FCU $((i * 3))s ..."
    sleep 3
done

say "4) 起 actor 控制 x6"
# 两个约束必须同时满足，缺一就静默失败：
#   · cwd 必须是 $XTDRONE —— control_actor.py 用相对路径 open("black_box.txt")
#   · import ObstacleAvoid 必须能找到 —— 该模块在 $XTDRONE 下，故显式加进 PYTHONPATH。
#     只用绝对路径启动会让 sys.path[0] 变成**脚本所在目录**，import 直接失败，
#     现象是"actor 控制进程数 = 0"。
ACTOR_PY="$XTDRONE/control_actor_m2.py"
[ -f "$ACTOR_PY" ] || ACTOR_PY="$ROOT/scripts/control_actor_m2.py"
cd "$XTDRONE" 2>/dev/null || say "   ⚠ XTDRone 目录不存在: $XTDRONE（actor 可能起不来）"
for i in 0 1 2 3 4 5; do
    setsid nohup env PYTHONPATH="$XTDRONE" python3 -u "$ACTOR_PY" $i \
        > "/tmp/actor_$i.log" 2>&1 < /dev/null &
    sleep 1
done
sleep 8
say "   actor 控制进程=$(pgrep -fc control_actor_m2)"

say "5) 起感知（自带 YOLO，不另起独立节点）"
cd "$ROOT" || exit 1
say "   独立 yolo 节点进程数（应为 0）: $(pgrep -fc yolo_v11)"
setsid nohup python3 -u perception_real.py > /tmp/percep_fly.log 2>&1 < /dev/null &
sleep 12
say "   percep=$(pgrep -fc perception_real)"

say "6) 相机话题自检"
timeout 25 rostopic list 2>/dev/null | grep -iE 'stereo_camera|cgo3' | sed 's/^/[rs]   相机: /'

if [ "$fcu" = "1" ]; then
    say "飞控栈就绪，可以起飞"; echo STACK_FLY_OK
else
    say "MAVROS 未连上飞控: fcu=$fcu"; echo STACK_FLY_FAIL
fi
say DONE
