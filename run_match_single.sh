#!/bin/bash
# ============================================================================
# VM 单机全链路得分验证编排（官方件零改动；本脚本与 robocup_single.launch 为
# 仅有的两个新增文件，其余全部原样使用队友仓/官方资源）。
#
# 步骤对齐 run_match.sh：①场景 ②通信桥+actor ③裁判 ④感知 ④b YOLO桥 ⑥协同
# 差异：单机（typhoon_h480_0）、无雷达（SKIP_RADAR 恒 1）、无 D2O。
#
# 用法:
#   ./run_match_single.sh start    # 启动全场（默认）
#   ./run_match_single.sh status   # 组件与关键话题健康
#   ./run_match_single.sh stop     # 停全场
#   ./run_match_single.sh verify   # 只挂验证器（复核官方 /score，不干预飞行）
#   ./run_match_single.sh fly      # 用已验证的飞控基准 fly_to_actor.py 单独验飞控
# 可选环境变量:
#   BRIDGE_NEW_TRACK_CONF=0.7      建轨置信度门（= 桥源码默认值；调低会放行红色场景误检）
#   BRIDGE_PUB_MAX_RANGE=12.0      播报距离闸门 m（够不着就不播，避免清零裁判 15s 计时）
#   GATE_TIMEOUT=180
#   NO_SWARM=1                    不起 swarm_agent（把控制权留给 fly 子命令）
#
# === 飞控铁律（来自 ~/robocup_real/fly_to_actor.py 头部 6 条血泪教训，10-03 亲测）===
#   1) setpoint 必须由**独立线程 20Hz 持续发布**；主线程阻塞也不得断流。
#      断流 >COM_OF_LOSS_T(1.0s) ⇒ PX4 失效保护 ⇒ 降落锁死 FLIGHT_TERMINATION，
#      必须重启 PX4。指纹：setpoint_raw/target 的 type_mask 由 0 突变为 39、pos=nan。
#   2) **分段小步飞**：每帧设定点推进 ≤0.6m。一次性发远距离目标会姿态翻转
#      （roll≈177°/pitch≈49°，相机朝天），实测把飞机顶到 z=482m（>6m 官方判 0）。
#   3) 高度硬护栏：官方 >6m 直接 score=0。本脚本限高到 2.8m（env ALT_*）。
#   4) 同一时刻只能有**一个** setpoint 发布者（多发布者会互相覆盖成 type_mask=39）。
# ============================================================================
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROBOCUP_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
WS_SETUP="${ROBOCUP_WS_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4_Firmware}"
XTDRONE_DIR="${XTDRONE_DIR:-$HOME/XTDrone}"
ROBO_DIR="$XTDRONE_DIR/robocup"
PERC_DIR="$REPO_ROOT/perception"
ENV_SCRIPT="$PERC_DIR/scripts/env_robocup.sh"
COORD_WS_SETUP="$REPO_ROOT/coordination/devel/setup.bash"
SWARM_SCRIPTS="$REPO_ROOT/coordination/src/robocup_swarm/scripts"
NAV_SRC="$REPO_ROOT/coordination/src/robocup_navigation/src"
BB2MD="$REPO_ROOT/coordination/src/robocup_training_worlds/scripts/black_box_to_metadata.py"
ROBOCUP_METADATA_FILE="$REPO_ROOT/coordination/src/robocup_training_worlds/worlds/generated/robocup_base.json"
ROBOCUP_LIVE_METADATA="$REPO_ROOT/coordination/src/robocup_training_worlds/worlds/generated/robocup_live.json"
BLACK_BOX_FILE="$HOME/XTDrone/robocup/black_box.txt"
SCENE_LAUNCH="$REPO_ROOT/launch/robocup_single.launch"
COMM_BRIDGE="$XTDRONE_DIR/communication/multirotor_communication.py"

UAV="typhoon_h480_0"
GATE_TIMEOUT="${GATE_TIMEOUT:-180}"
# ---- 建轨置信度门：必须回到 0.7（桥源码默认），不得再用 0.35 ----
# 为什么（2026-10-03 复盘，源码注释 + 实测双向印证）：
#   1) yolo_target_bridge.py:114 的默认值就是 0.7，注释写明它是为治红色误检而设：
#      「3 条 0.43~0.61 的 red1 误检（真身为 green 演员）建出鬼影轨 t4，
#        全队盘旋假目标并广播假消除」⇒ 0.7 正是这道病的药。
#   2) 感知上报的字段是 **原始 YOLO conf**（perception_real.py:1234/1356
#      `confidence=tk.conf`），不是打分 score。红色场景物（消防栓 / STOP 牌 /
#      加油站招牌 / 建筑红条）实测 conf 0.30~0.56 ⇒ 用 0.35 等于把药停了，
#      这些误检全部重新可建轨，飞机被牵去追永远消不掉的目标。
#   3) 跑单机全链路时若确实感到漏检（远处真目标 conf < 0.7），临场用
#      `BRIDGE_NEW_TRACK_CONF=0.5 ./run_match_single.sh start` 覆盖即可，
#      别把默认值再改低 —— 默认值要和比赛主链 run_match.sh 保持一致。
export BRIDGE_NEW_TRACK_CONF="${BRIDGE_NEW_TRACK_CONF:-0.7}"
export ROBOCUP_WS="$REPO_ROOT/coordination"
export SEED_TRUTH=0

RUN_DIR="/tmp/robocup_single"
REG="$RUN_DIR/components.tsv"
STAMP="$(date '+%Y%m%d_%H%M%S')"
LOGDIR="$RUN_DIR/logs_$STAMP"

if [ -t 1 ]; then C_G=$'\033[32m'; C_Y=$'\033[33m'; C_R=$'\033[31m'; C_B=$'\033[36m'; C_0=$'\033[0m'
else C_G=""; C_Y=""; C_R=""; C_B=""; C_0=""; fi
info(){ echo "${C_B}[*]${C_0} $*"; }
ok(){   echo "${C_G}[+]${C_0} $*"; }
warn(){ echo "${C_Y}[!]${C_0} $*"; }
err(){  echo "${C_R}[x]${C_0} $*" >&2; }
step(){ echo; echo "${C_B}==== $* ====${C_0}"; }

# ---------- 环境（对齐 run_match.sh ros_env，全部为路径拼接，不改任何文件） ----------
ensure_colon_path(){
    local _var="$1" _dir="$2" _cur
    eval "_cur=\${$_var:-}"
    case ":$_cur:" in
        *":$_dir:"*) : ;;
        *) export "$_var=${_cur:+$_cur:}$_dir";;
    esac
}
_x_socket_exists(){
    local full="$1" num
    [ -z "$full" ] && return 1
    num="${full##*:}"; num="${num%%.*}"
    [ -S "/tmp/.X11-unix/X$num" ]
}
pick_display(){
    if _x_socket_exists "${DISPLAY:-}"; then return 0; fi
    if [ -S /tmp/.X11-unix/X0 ]; then export DISPLAY=:0; return 0; fi
    return 1
}
ros_env(){
    set +u
    [ -f "$ROS_SETUP" ] && source "$ROS_SETUP"
    [ -f "$WS_SETUP" ]  && source "$WS_SETUP"
    [ -f "$ENV_SCRIPT" ] && source "$ENV_SCRIPT" >/dev/null 2>&1
    [ -f "$COORD_WS_SETUP" ] && source "$COORD_WS_SETUP"
    pick_display
    set -u
    ensure_colon_path ROS_PACKAGE_PATH "$PX4_ROOT"
    ensure_colon_path ROS_PACKAGE_PATH "$PX4_ROOT/Tools/sitl_gazebo"
    ensure_colon_path ROS_PACKAGE_PATH "$REPO_ROOT/launch"
    ensure_colon_path ROS_PACKAGE_PATH "$HOME/catkin_ws/src"
    ensure_colon_path PYTHONPATH "$HOME/catkin_ws/devel/lib/python3/dist-packages"
    ensure_colon_path PYTHONPATH "$SWARM_SCRIPTS"
    ensure_colon_path PYTHONPATH "$NAV_SRC"
    ensure_colon_path CMAKE_PREFIX_PATH "$HOME/catkin_ws/devel"
    ensure_colon_path GAZEBO_MODEL_PATH  "$XTDRONE_DIR/sitl_config/models"
    ensure_colon_path GAZEBO_PLUGIN_PATH "$HOME/catkin_ws/devel/lib"
    ensure_colon_path GAZEBO_PLUGIN_PATH "$PX4_ROOT/build/px4_sitl_default/build_gazebo"
}

# ---------- 进程组 ----------
start_group(){
    local name="$1" logfile="$2"; shift 2
    mkdir -p "$LOGDIR"
    setsid nohup "$@" >"$logfile" 2>&1 </dev/null &
    local pgid=$!
    printf '%s\t%s\t%s\n' "$name" "$pgid" "$logfile" >> "$REG"
    info "已启动 $name (pgid=$pgid) -> $logfile"
}
group_pgid(){ awk -F'\t' -v n="$1" '$1==n{print $2; exit}' "$REG" 2>/dev/null; }
alive_pgid(){ kill -0 -- "-$1" 2>/dev/null; }
stop_group(){
    local name="$1" pgid; pgid="$(group_pgid "$name")"
    [ -z "$pgid" ] && return 0
    if alive_pgid "$pgid"; then
        kill -TERM -"$pgid" 2>/dev/null || true
        local w=0; while [ $w -lt 6 ] && alive_pgid "$pgid"; do sleep 1; w=$((w+1)); done
        kill -KILL -"$pgid" 2>/dev/null || true
        info "已停止 $name"
    fi
}

# ---------- 等待 ----------
wait_gate(){
    local desc="$1" timeout="$2"; shift 2
    local t=0
    while ! "$@"; do
        [ $t -ge "$timeout" ] && { err "$desc —— 等待超时（${timeout}s）"; return 1; }
        sleep 2; t=$((t+2))
    done
    ok "$desc（${t}s）"
}
has_service(){ timeout 8 rosservice list 2>/dev/null | grep -q "$1"; }
mavros_connected(){
    [ "$(timeout 8 rostopic echo -n1 "/$1/mavros/state" 2>/dev/null \
        | grep -c 'connected: True')" = "1" ]
}
first_msg(){ timeout 10 rostopic echo -n1 "$1" >/dev/null 2>&1; }
count_pattern(){ pgrep -fc "$1" 2>/dev/null || true; }
topic_publisher_match(){
    timeout 8 rostopic info "$1" 2>/dev/null \
      | awk '/Publishers:/{f=1;next}/Subscribers:/{f=0}f' | grep -q "$2"
}

partial_fail(){
    err "$*"
    err "启动中断，开始清理已起组件 ..."
    while IFS=$'\t' read -r name pgid _log; do stop_group "$name"; done < "$REG"
    pre_clean
    exit 1
}

# 开跑前无条件清残留（幂等，方便反复重跑）
pre_clean(){
    for pat in '[r]oslaunch' '[r]osmaster' '[r]osout' 'gzserver' 'gzclient' \
               '[b]in/px4' '[m]avros_node' '[c]ontrol_actor' \
               '[m]ultirotor_communication' '[p]erception_real' \
               '[y]olo_target_bridge' '[s]warm_agent' '[s]warm_manager' \
               '[s]core_cal' '[s]pawn_model'; do
        pkill -9 -f "$pat" 2>/dev/null
    done
    sleep 2
    rm -f /tmp/px4_lock-* /tmp/px4_sock-* 2>/dev/null
    return 0
}

# ============================ start ============================
do_start(){
    mkdir -p "$RUN_DIR" "$LOGDIR"
    ln -sfn "$LOGDIR" "$RUN_DIR/latest"

    if [ -s "$REG" ]; then
        spgid="$(awk -F'\t' '$1=="scene"{print $2; exit}' "$REG")"
        if [ -n "$spgid" ] && kill -0 -- "-$spgid" 2>/dev/null; then
            err "已有单机场在跑。先 ./run_match_single.sh stop"; exit 1
        fi
    fi
    : > "$REG"
    pre_clean
    ros_env

    # ---------------- ① 场景 ----------------
    step "① 起场景 robocup_single.launch（1 架 + robocup.world）"
    start_group scene "$LOGDIR/01_scene.log" \
        roslaunch "$SCENE_LAUNCH" "gui:=false" "paused:=false"
    wait_gate "Gazebo 服务就绪" "$GATE_TIMEOUT" has_service '/gazebo/get_model_state' \
        || partial_fail "场景没起来，看 $LOGDIR/01_scene.log"
    wait_gate "$UAV MAVROS connected" 300 mavros_connected "$UAV" \
        || partial_fail "$UAV 飞控未连上（残留模型？pkill -9 -x gzserver 后重来）"
    # --- spawn 成功性校验（2026-10-03 加）---
    # MAVROS connected 只说明 UDP 通了，不代表 PX4 与 Gazebo 模型建立了桥。
    # gazebo_ros_api_plugin.cpp:2744 把 SDF spawn 超时写死 10s，重型世界加载
    # 期间 spawn 会失败 ⇒ EKF 收不到传感器 ⇒ local_position 在 ±150m 乱跳 ⇒
    # 飞机飞不起来也控不住（guided:False）。这种场必须立刻判失败，别白跑。
    _sp_ok=0
    for _i in $(seq 1 60); do
        if grep -aq "Successfully spawned entity" "$LOGDIR/01_scene.log" 2>/dev/null; then
            _sp_ok=1; break
        fi
        if grep -aq "spawn service timed out" "$LOGDIR/01_scene.log" 2>/dev/null; then
            break
        fi
        sleep 2
    done
    if [ "$_sp_ok" = "1" ]; then
        ok "spawn 正常（PX4↔Gazebo 桥已建立）"
    else
        partial_fail "PX4 spawn 失败/超时（gazebo_ros 10s 硬超时）⇒ EKF 无数据、飞机控不住。\
看 $LOGDIR/01_scene.log；本仓 single_vehicle_spawn_xtd_delayed.launch 已延迟 20s，\
若仍失败请把 spawn_delay 调大到 30/40"
    fi

    # ---------------- ② 通信桥 + 6 actor ----------------
    step "② 起 XTDrone 通信桥 + control_actors"
    start_group comm_0 "$LOGDIR/02_comm_0.log" \
        python3 -u "$COMM_BRIDGE" typhoon_h480 0
    sleep 3
    # 官方 control_actors.sh 里写死 `python`，非登录 shell 的 PATH 无 ~/.local/bin，
    # 这里只给子进程补 PATH，不改官方脚本。
    start_group actors "$LOGDIR/02_actors.log" bash -c \
        "export PATH=\$HOME/.local/bin:\$PATH; cd '$ROBO_DIR' && ./control_actors.sh"
    sleep 8
    nactor="$(count_pattern '[c]ontrol_actor.py')"
    if [ "$nactor" -ge 6 ]; then ok "6 个 actor 控制器在跑"
    else partial_fail "actor 控制器只有 $nactor/6，看 $LOGDIR/02_actors.log"; fi

    # ---------------- ③ 裁判 ----------------
    step "③ 起裁判 score_cal.py typhoon_h480"
    start_group judge "$LOGDIR/03_judge.log" bash -c \
        "cd '$ROBO_DIR' && python3 -u score_cal.py typhoon_h480"
    sleep 5
    if first_msg /score; then ok "裁判已在发布 /score"
    else partial_fail "裁判没起来，看 $LOGDIR/03_judge.log"; fi

    # ---------------- ④ 感知（CPU） ----------------
    step "④ 起感知 perception_real.py（CPU 推理）"
    start_group "pr_$UAV" "$LOGDIR/04_perception.log" bash -c \
        "cd '$PERC_DIR' && CUDA_VISIBLE_DEVICES='' PR_UAV='$UAV' PR_CAM_LINK='$UAV::base_link' python3 -u perception_real.py"

    # ---------------- ④b YOLO→swarm 桥 ----------------
    step "④b 起 yolo_target_bridge（建轨门 BRIDGE_NEW_TRACK_CONF=$BRIDGE_NEW_TRACK_CONF）"
    start_group yolo_bridge "$LOGDIR/04b_yolo_bridge.log" bash -c \
        "cd '$SWARM_SCRIPTS' && python3 -u yolo_target_bridge.py"
    sleep 4
    if topic_publisher_match /swarm/target_states yolo_target_bridge; then
        ok "yolo_target_bridge 已注册发布 /swarm/target_states"
    else
        partial_fail "yolo_target_bridge 未注册发布，看 $LOGDIR/04b_yolo_bridge.log"
    fi

    # ---------------- ⑥ 协同层（单机） ----------------
    step "⑥ 起协同层 swarm（manager + 1 agent）"
    check_hard(){ eval "$2" >/dev/null 2>&1 || partial_fail "$1 缺失"; }
    check_hard "官方随机化产物 black_box.txt" "[ -f '$BLACK_BOX_FILE' ]"
    if python3 "$BB2MD" --black-box "$BLACK_BOX_FILE" \
            --template "$ROBOCUP_METADATA_FILE" \
            --output "$ROBOCUP_LIVE_METADATA" >>"$LOGDIR/06_map_gen.log" 2>&1; then
        ok "动态地图已生成：black_box.txt → robocup_live.json"
        export ROBOCUP_METADATA="$ROBOCUP_LIVE_METADATA"
    else
        partial_fail "动态地图生成失败（看 $LOGDIR/06_map_gen.log）"
    fi

    rosparam set /swarm_manager/uav_ids "$UAV"
    start_group swarm_manager "$LOGDIR/06_swarm_manager.log" bash -c \
        "cd '$SWARM_SCRIPTS' && python3 -u swarm_manager.py"
    sleep 5
    alive_pgid "$(group_pgid swarm_manager)" || pgrep -f '[s]warm_manager.py' >/dev/null \
        || partial_fail "swarm_manager 启动即退出（metadata？看日志）"

    # NO_AGENT=1：不起 agent。原因（2026-10-03 实测）：agent 会自己 ARM+起飞；
    # 之后我们若把它杀掉去跑 yolo_flyer，PX4 会触发 offboard 失联保护并重置
    # 位置估计 ⇒ /mavros/local_position/pose 重新乱跳（实测 ±150m）⇒ 我们的
    # 飞行器基于它算方向就会瞎飞、冲出场地。所以要用自家飞行器时，
    # 必须从**未起飞的地面状态**接管：NO_AGENT=1 ./run_match_single.sh start
    if [ "${NO_AGENT:-0}" = "1" ]; then
        warn "NO_AGENT=1：不起 swarm_agent_0 —— 飞机交给 yolo_flyer 独占控制"
    else
    rosparam set "/swarm_agent_0/uav_id" "$UAV"
    rosparam set "/swarm_agent_0/model_name" "$UAV"
    rosparam set "/swarm_agent_0/world_offset_x" "0"
    rosparam set "/swarm_agent_0/world_offset_y" "-3"
    # 高度层压到 2.8m：官方 >6m 直接 score=0；而 3.6m 时白球 conf 0.265（阈值 0.40
    # 会漏检），3.0m 时 0.689。低空既安全又看得清。
    # agent 内置自检要求 ALT_HARD_CEIL ≥ ALT_TARGET_CAP 且 ALT_EMERG_CEIL > HARD_CEIL。
    export ALT_BASE="${ALT_BASE:-2.8}"
    export ALT_TARGET_CAP="${ALT_TARGET_CAP:-2.8}"
    export ALT_CEILING="${ALT_CEILING:-3.0}"
    export ALT_HARD_CEIL="${ALT_HARD_CEIL:-3.4}"
    export ALT_PANIC="${ALT_PANIC:-3.6}"
    export ALT_EMERG_CEIL="${ALT_EMERG_CEIL:-3.9}"
    export CLIMB_DONE_ALT="${CLIMB_DONE_ALT:-1.5}"
    start_group "swarm_agent_0" "$LOGDIR/06_swarm_agent_0.log" bash -c \
        "cd '$SWARM_SCRIPTS' && ALT_BASE=$ALT_BASE ALT_TARGET_CAP=$ALT_TARGET_CAP \
ALT_CEILING=$ALT_CEILING ALT_HARD_CEIL=$ALT_HARD_CEIL ALT_PANIC=$ALT_PANIC \
ALT_EMERG_CEIL=$ALT_EMERG_CEIL CLIMB_DONE_ALT=$CLIMB_DONE_ALT \
POS_SPV=${POS_SPV:-0} \
python3 -u swarm_agent.py __name:=swarm_agent_0"
    sleep 10
    alive_pgid "$(group_pgid swarm_agent_0)" || partial_fail "swarm_agent_0 退出，看 $LOGDIR/06_swarm_agent_0.log"
    ok "协同层 manager + agent_0 在跑"
    fi

    # 数据源审计
    if rostopic info /swarm/target_states 2>/dev/null | awk '/Publishers:/{f=1;next}/Subscribers:/{f=0}f' | grep -q yolo_target_bridge; then
        ok "/swarm/target_states 由 yolo_target_bridge 发布（YOLO 合法链路）"
    else
        warn "/swarm/target_states 数据源异常"
    fi

    echo
    echo "${C_G}==============================================================${C_0}"
    ok "单机全链路已起。日志: $LOGDIR"
    echo "  实时看分:        rostopic echo /score"
    echo "  检测上报:        rostopic echo /coordination/target_report"
    echo "  目标状态:        rostopic echo /swarm/target_states"
    echo "  官方播报(6色):   rostopic hz /actor_red_info"
    echo "  健康/停止:       $0 status | $0 stop"
    echo "${C_G}==============================================================${C_0}"
}

# ============================ status ============================
do_status(){
    [ -s "$REG" ] || { info "没有在跑的场（$REG 不存在）"; return; }
    ros_env 2>/dev/null || true
    printf '%-16s %-8s\n' "组件" "状态"
    alive=0; total=0
    while IFS=$'\t' read -r name pgid logf; do
        total=$((total+1))
        if alive_pgid "$pgid"; then s="${C_G}运行${C_0}"; alive=$((alive+1)); else s="${C_R}已退出${C_0}"; fi
        printf '%-16s %-10s %s\n' "$name" "$s"
    done < "$REG"
    info "组件存活 $alive/$total"
    timeout 5 rostopic echo -n1 "/$UAV/mavros/state" 2>/dev/null | grep -q 'connected: True' \
        && info "MAVROS connected" || warn "MAVROS 未连接"
    info "actor 控制器: $(count_pattern '[c]ontrol_actor.py')/6"
    info "感知进程: $(count_pattern '[p]erception_real')"
    sc="$(timeout 5 rostopic echo -n1 /score 2>/dev/null | grep -m1 -o '[0-9-]*' || echo '?')"
    info "当前 /score = $sc"
    n_info="$(timeout 5 rostopic list 2>/dev/null | grep -c '^/actor_.*_info')"
    info "/actor_*_info 话题数: $n_info"
}

# ============================ stop ============================
do_stop(){
    if [ ! -s "$REG" ]; then info "没有需要停止的场。"; exit 0; fi
    stop_group swarm_agent_0
    stop_group swarm_manager
    stop_group yolo_bridge
    stop_group "pr_$UAV"
    stop_group judge
    stop_group actors
    stop_group comm_0
    sleep 2
    stop_group scene
    sleep 3
    pkill -9 -f '[p]erception_real.py'      2>/dev/null || true
    pkill -9 -f '[m]ultirotor_communication.py' 2>/dev/null || true
    pkill -9 -f '[c]ontrol_actor.py'        2>/dev/null || true
    if [ "${FULL_TEARDOWN:-0}" = "1" ]; then
        pkill -9 -f '[m]avros_node' 2>/dev/null || true
        pkill -9 -f '[b]in/px4'     2>/dev/null || true
        pkill -9 -x gzserver        2>/dev/null || true
        pkill -9 -x gzclient        2>/dev/null || true
        pkill -9 -f '[r]osmaster'   2>/dev/null || true
    fi
    mv "$REG" "$REG.stopped.$STAMP" 2>/dev/null || true
    ok "已停止单机场。"
}

# ============================ verify（只观测，不干预） ============================
# 挂我们的裁判复核器：判据与官方逐条对齐，但真值走 /gazebo/model_states 话题
# （官方 get_model_state 服务在高负载下可取率仅 16~80%）。
do_verify(){
    ros_env
    [ -f "$REPO_ROOT/scripts/verify_score.py" ] || { err "缺 verify_score.py"; exit 1; }
    cd "$REPO_ROOT/scripts"
    exec python3 -u verify_score.py "${2:-typhoon_h480}"
}

# ============================ 独占 setpoint 通道（铁律 4） ============================
# 2026-10-03 事故：本脚本 do_start 起了 XTDrone 的 multirotor_communication.py，
# 而 do_yolo/do_fly 只杀了 swarm_agent/swarm_manager，**漏了这个通信节点**。
# 该节点源码 communication.py:55-61 是无条件 30Hz 广播：
#       while not rospy.is_shutdown():
#           self.target_motion_pub.publish(self.target_motion)
# 且 target_motion = PositionTarget() 初值全零、type_mask=0（位置有效）⇒
# 它从启动那一刻起就 30Hz 命令"去局部原点 (0,0,0)"。于是同一个 PX4 位置控制器
# 同时收到两条流：我们 20Hz 的目标 + 它 30Hz 的原点 ⇒ 每秒 60% 的帧是零点，
# 飞机被两个吸引子来回撕扯（实测日志位置在 (38.4,6.7)↔(63.2,10.3) 反复跳），
# 最终被拖出场地。30Hz > 20Hz，我们必输。
# 处理：自家飞行器起飞前，停掉所有竞争者（通信桥 + agent + manager + 旧 flyer）。
# 注意 pkill 模式用 [x]xx 括号写法，避免误杀当前这条命令自身。
kill_other_sp(){
    for pat in '[m]ultirotor_communication.py' '[s]warm_agent' \
               '[s]warm_manager' '[f]ly_to_actor.py' '[y]olo_flyer.py'; do
        pkill -9 -f "$pat" 2>/dev/null || true
    done
    sleep 1
}

# ============================ fly（飞控基准） ============================
# 用 ~/robocup_real/fly_to_actor.py（本机曾稳定飞行 3 小时的基准脚本）单独验飞控。
# 它自带 ARM/OFFBOARD、坐标标定、20Hz setpoint 心跳线程、分段逼近、yaw 对准。
# ⚠️ 它读 actor 真值走 /actor_N/pose（ros_actor_cmd_pose_plugin），正式比赛非法；
#    这里只作飞控基准与调试。
do_fly(){
    FLY="${FLY:-$HOME/robocup_real/fly_to_actor.py}"
    [ -f "$FLY" ] || { err "缺飞控基准脚本 $FLY"; exit 1; }
    kill_other_sp
    ros_env
    cd "$(dirname "$FLY")"
    step "飞控基准：$FLY auto ${2:-10} ${3:-2.5} --hold ${4:-40} --stay ${5:-40}"
    warn "提醒：飞机若已飞出场外（x<-55 或 x>135），本脚本飞不回来，需重启场景"
    exec python3 -u "$FLY" "${@:2}"
}

# ============================ yolo（我们自己的合法链路飞行器） ============================
# scripts/yolo_flyer.py：目标来源 = /swarm/target_states（YOLO→bridge），
# 不读任何真值话题，符合规则 §2.5.11。
# 行为：找人 → 20m 内限速 <1m/s 靠近 → 悬停播报 → 官方删机后自动转下一个。
# 用法: ./run_match_single.sh yolo [keep_m] [alt] [speed]
do_yolo(){
    FLYER="$REPO_ROOT/scripts/yolo_flyer.py"
    [ -f "$FLYER" ] || { err "缺 $FLYER"; exit 1; }
    kill_other_sp
    ros_env
    # 起飞前自检（飞行器内也会再查一遍）：确认飞机仍在官方起飞点。
    # 不满足就连飞都不飞，省得白跑一场还把飞机扔出场外。
    _lx="$(timeout 6 rostopic echo -n1 "/$UAV/mavros/local_position/pose" 2>/dev/null \
           | awk '/x:/{print $2}' | head -1)"
    _ly="$(timeout 6 rostopic echo -n1 "/$UAV/mavros/local_position/pose" 2>/dev/null \
           | awk '/y:/{print $2}' | head -2 | tail -1)"
    if [ -n "$_lx" ] && [ -n "$_ly" ]; then
        _r="$(awk -v a="$_lx" -v b="$_ly" 'BEGIN{printf "%.2f", (a*a+b*b)^0.5}')"
        info "起飞点校验：local=(${_lx},${_ly}) 水平模长=${_r}m（要求 <2m）"
        awk -v r="$_r" 'BEGIN{exit !(r>=2.0)}' && {
            err "飞机不在起飞点（离原点 ${_r}m）—— 坐标标定会整体偏移，必然出界。"
            err "请先重启场景： ./run_match_single.sh stop && NO_AGENT=1 ./run_match_single.sh start"
            exit 1
        }
    fi
    cd "$REPO_ROOT/scripts"
    step "合法链路飞行器：keep=${2:-7} alt=${3:-2.5} speed=${4:-0.9}"
    exec python3 -u yolo_flyer.py --keep="${2:-7}" --alt="${3:-2.5}" --speed="${4:-0.9}"
}

case "${1:-start}" in
    status) do_status ;;
    stop)   do_stop ;;
    start)  do_start ;;
    verify) do_verify ;;
    fly)    do_fly ;;
    yolo)   do_yolo ;;
    *) err "未知子命令: $1（start | status | stop | verify | fly | yolo）"; exit 2 ;;
esac
