#!/bin/bash
# 启动官方 6 个 actor 的行为 AI（control_actor.py）。
#
# 关键约束：control_actor.py 用相对路径 open("black_box.txt")，
# 必须 cd 到 ~/XTDrone/robocup 再启动，否则直接崩。
#
# 官方 control_actors.sh 用的是 `python`，Ubuntu 20.04 没有这个命令，这里改用 python3。
ROB="${ROBOCUP_XTDRONE:-$HOME/XTDrone/robocup}"
LOG=/tmp/run_actors.log
exec > "$LOG" 2>&1

ROS_SETUP="${ROBOCUP_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
WS_SETUP="${ROBOCUP_WS_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
# shellcheck disable=SC1090
[ -f "$ROS_SETUP" ] && source "$ROS_SETUP"
# shellcheck disable=SC1090
[ -f "$WS_SETUP" ] && source "$WS_SETUP"

cd "$ROB" || { echo "[act] 目录不存在: $ROB"; exit 1; }

echo "[act] $(date '+%F %T') 停止旧的 actor 进程 ..."
pkill -f "[c]ontrol_actor" 2>/dev/null
sleep 2

# cwd 必须是 $ROB：control_actor.py 用相对路径 open("black_box.txt")。
# PYTHONPATH 必须含 $ROB：import ObstacleAvoid 在这里，否则进程数为 0。
ACTOR_PY="$ROB/control_actor_m2.py"
[ -f "$ACTOR_PY" ] || ACTOR_PY="$(cd "$(dirname "$0")" && pwd)/control_actor_m2.py"
for i in 0 1 2 3 4 5; do
  # 让 nohup 出去的子进程也拿到 ROS 环境（source /opt/ros + catkin_ws），
  # 否则子进程找不到 rospy，直接 ModuleNotFoundError 退出。
  nohup env PYTHONPATH="$ROB:${PYTHONPATH:-}" \
    bash -c "source \"$ROS_SETUP\"; source \"$WS_SETUP\"; cd \"$ROB\"; exec python3 -u \"$ACTOR_PY\" \"$i\"" \
    > "/tmp/actor_$i.log" 2>&1 &
  sleep 0.6
done

sleep 8
echo "[act] 存活进程数: $(pgrep -fc '[c]ontrol_actor.py')"
for i in 0 1 2 3 4 5; do
  printf '[act] actor_%s 首行: ' "$i"
  head -3 "/tmp/actor_$i.log" 2>/dev/null | tr '\n' ' '
  echo
done
echo "[act] ACTORS_STARTED"
