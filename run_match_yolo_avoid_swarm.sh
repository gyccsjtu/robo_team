#!/usr/bin/env bash
# ============================================================================
# RoboCup 全场比赛 - 一键 stop + start (YOLO+避障+协同+裁判)
# 把这个文件直接放在 /home/gycc/桌面/RoboCup_Team/run_match_yolo_avoid_swarm.sh
# 然后执行:
#   chmod +x /home/gycc/桌面/RoboCup_Team/run_match_yolo_avoid_swarm.sh
#   /home/gycc/桌面/RoboCup_Team/run_match_yolo_avoid_swarm.sh
# ============================================================================
set +e
set -u

# ---- 参数 ----
export GAZEBO_GUI=${GAZEBO_GUI:-true}
export START_PAUSED=${START_PAUSED:-0}
export SKIP_RADAR=${SKIP_RADAR:-0}
export RADAR_GUARD=${RADAR_GUARD:-1}
export YOLO_BRIDGE=${YOLO_BRIDGE:-1}
export D2O_ENABLE=${D2O_ENABLE:-0}
export RADAR_HANDOFF=${RADAR_HANDOFF:-stop}
export FORCE=${FORCE:-1}
export FULL_TEARDOWN=${FULL_TEARDOWN:-1}

cd /home/gycc/桌面/RoboCup_Team
echo "[run] CWD=$(pwd)"
echo "[run] 参数: GAZEBO_GUI=$GAZEBO_GUI START_PAUSED=$START_PAUSED SKIP_RADAR=$SKIP_RADAR RADAR_GUARD=$RADAR_GUARD YOLO_BRIDGE=$YOLO_BRIDGE D2O_ENABLE=$D2O_ENABLE RADAR_HANDOFF=$RADAR_HANDOFF FORCE=$FORCE FULL_TEARDOWN=$FULL_TEARDOWN"

# ---- step 1: stop ----
echo
echo "============================="
echo " STEP 1: run_match.sh stop"
echo "============================="
./run_match.sh stop
echo "[run] stop exit=$?"

sleep 3

# ---- step 1b: 兜底清理 ----
echo
echo "============================="
echo " STEP 1b: 残留兜底"
echo "============================="
for p in gzserver gzclient rosmaster roscore; do pkill -9 -x $p 2>/dev/null; done
for pat in 'bin/px4' mavros_node multirotor_communication control_actor perception_real yolo_target_bridge swarm_agent swarm_manager radar_avoid score_cal roslaunch; do
    pkill -9 -f "$pat" 2>/dev/null
done
sleep 2
if [ -f /tmp/robocup_match/components.tsv ]; then
    mv /tmp/robocup_match/components.tsv /tmp/robocup_match/components.tsv.stopped.manual_$(date +%Y%m%d_%H%M%S) 2>/dev/null
fi
echo "[run] 残留: gz=$(pgrep -fc gzserver) px4=$(pgrep -fc 'bin/px4') mav=$(pgrep -fc mavros_node) comm=$(pgrep -fc multirotor_communication) actor=$(pgrep -fc control_actor) rosmaster=$(pgrep -fc rosmaster)"

# ---- step 2: preflight ----
echo
echo "============================="
echo " STEP 2: preflight"
echo "============================="
./run_match.sh preflight
RC_PF=$?
echo "[run] preflight exit=$RC_PF"
if [ $RC_PF -ne 0 ]; then
    echo "[run] preflight 失败,补依赖后再 start"
    exit 2
fi

# ---- step 3: start ----
echo
echo "============================="
echo " STEP 3: run_match.sh start (YOLO+避障+协同+裁判)"
echo "============================="
./run_match.sh start
RC=$?
echo "[run] start exit=$RC"

if [ $RC -eq 0 ]; then
    echo
    echo "============================="
    echo " ✅ 全部就绪"
    echo "============================="
    echo
    echo "实时计分: rostopic echo /score"
    echo "健康:     ./run_match.sh status"
    echo "停止:     ./run_match.sh stop"
fi