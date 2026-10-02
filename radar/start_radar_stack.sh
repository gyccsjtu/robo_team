#!/bin/bash
HERE="$(cd "$(dirname "$0")" && pwd)"
LOG=/tmp/radar_stack.log
: > "$LOG"
say(){ echo "[stack] $(date '+%F %T') $*" | tee -a "$LOG"; }
SPAWN_X="${1:--6.0}"; SPAWN_Y="${2:-4.0}"

say "--- 1) 杀干净旧栈 ---"
pkill -9 -f '[g]zserver' 2>/dev/null; pkill -9 -f '[g]zclient' 2>/dev/null
pkill -9 -f '[b]in/px4' 2>/dev/null; pkill -9 -f '[m]avros_node' 2>/dev/null
pkill -9 -f '[r]osmaster' 2>/dev/null; pkill -9 -f '[r]oslaunch' 2>/dev/null
pkill -9 -f '[s]pawn_model' 2>/dev/null
pkill -9 -f '[m]ultirotor_communication' 2>/dev/null
sleep 5
source "$HERE/env_robocup.sh" >> "$LOG" 2>&1

say "--- 2) 起世界 ---"
setsid nohup roslaunch "$HERE/world_only.launch" world:="$HERE/base_fast.world" gui:=false \
    > /tmp/world_only.log 2>&1 </dev/null &
disown
gz_ok=0
for i in $(seq 1 60); do
  if pgrep -f '[g]zserver' >/dev/null 2>&1 && \
     timeout 10 rosservice list 2>/dev/null | grep -q '/gazebo/get_model_state'; then
    gz_ok=1; say "gzserver 就绪（第 $((i*3)) 秒）"; break
  fi
  sleep 3
done
[ "$gz_ok" = 1 ] || { say "gzserver 起不来"; echo RADAR_STACK_FAIL; exit 1; }
GZPID=$(pgrep -f '[g]zserver' | head -1)
[ -n "$GZPID" ] && taskset -pc 0-5 "$GZPID" >>"$LOG" 2>&1

say "--- 3) 起 PX4+MAVROS  出生点=($SPAWN_X, $SPAWN_Y) ---"
setsid nohup roslaunch "$HERE/uav_lidar_clean.launch" x:="$SPAWN_X" y:="$SPAWN_Y" \
    > /tmp/uav_lidar_clean.log 2>&1 </dev/null &
disown
mdl_ok=0
for i in $(seq 1 60); do
  OUT=$(timeout 12 rosservice call /gazebo/get_model_state \
        "{model_name: 'typhoon_h480_0', relative_entity_name: 'world'}" 2>/dev/null)
  if echo "$OUT" | grep -q 'success: True'; then
    mdl_ok=1; say "模型已生成（第 $((i*5)) 秒）"
    echo "$OUT" | grep -A3 position | head -4 | sed 's/^/[stack]   /' >> "$LOG"
    break
  fi
  sleep 5
done
[ "$mdl_ok" = 1 ] || { say "模型生成失败"; echo RADAR_STACK_FAIL; exit 1; }

fcu_ok=0
for i in $(seq 1 60); do
  S=$(timeout 8 rostopic echo -n1 /typhoon_h480_0/mavros/state 2>/dev/null)
  if echo "$S" | grep -q 'connected: True'; then fcu_ok=1; say "MAVROS connected"; break; fi
  sleep 5
done
[ "$fcu_ok" = 1 ] || { say "MAVROS 未连上"; echo RADAR_STACK_FAIL; exit 1; }
say "栈就绪  出生点=($SPAWN_X, $SPAWN_Y)"
echo RADAR_STACK_OK
