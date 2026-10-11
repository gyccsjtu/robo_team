#!/bin/bash
# ============================================================================
# RoboCup 多旋翼集群搜索 —— 正式比赛一键编排脚本
#
# 用法:
#   ./run_match.sh preflight          # 只做赛前体检（不启动任何东西，安全）
#   ./run_match.sh start              # 按 ①~⑥ 顺序启动全场（默认）
#   ./run_match.sh start --gui        # 同上，强制 GAZEBO_GUI=true（复盘看 actor / 视觉）
#   ./run_match.sh start --no-gui     # 显式关闭图形界面
#   ./run_match.sh status             # 查看各组件与关键话题健康
#   ./run_match.sh stop               # 停止本次比赛的全部组件
#
# 可选环境变量（均有默认值）:
#   GAZEBO_GUI=true             Gazebo 是否开图形界面（默认打开）
#   GATE_TIMEOUT=180            单个就绪检查最长等待秒数
#   PR_UAVS="typhoon_h480_0 typhoon_h480_1 ..."  感知节点服务的飞机（默认 6 机全起）
#   COORD_WS_SETUP=<path>       协同层 catkin devel/setup.bash（未编译则第⑥步跳过）
#   YOLO_BRIDGE=1              起 yolo_target_bridge（YOLO target_report → /swarm/target_states）
#   MAX_SPEED=6.0              巡航速度上限 m/s（规则 §2.5(4)：恐怖分子 2m/s 逃逸，本机留 3x 余量）
#   ALT_HARD_CEIL=5.7         飞行高度硬护栏 m（规则 §2.5(3)：限高 6m，留 0.3m 抖动空间）
#   RADAR_WARN_R=5.5          雷达预警半径 m（规则 §2.5(7)：碰撞扣30/次，扩到车体级）
#   FRIEND_SAFE_DIST=3.5      友机避碰起点 m（6 机密集 + 碰撞扣30/次）
#   SKIP_RADAR=2 / REQUIRE_SWARM=1 / FORCE=0   正式链路要求 2D 激光与协同层
# ============================================================================

# ---- 严格模式：不使用 set -e，所有失败由检查函数显式处理 ----
set -uo pipefail

# ============================ 路径与配置 ====================================
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROBOCUP_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
WS_SETUP="${ROBOCUP_WS_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4_Firmware}"
XTDRONE_DIR="${XTDRONE_DIR:-$HOME/XTDrone}"
ROBO_DIR="$XTDRONE_DIR/robocup"

PERC_DIR="$REPO_ROOT/perception"
RADAR_DIR="$REPO_ROOT/radar"
ENV_SCRIPT="$PERC_DIR/scripts/env_robocup.sh"
WEIGHTS="$REPO_ROOT/weights/best_yolo11n_bino_v1.pt"
OFFICIAL_WORLD="$PX4_ROOT/Tools/sitl_gazebo/worlds/robocup.world"
MAP_GENERATOR="$ROBO_DIR/map_generator.py"
ACTOR_MODEL_DIR="$XTDRONE_DIR/sitl_config/models/actor"
COMM_BRIDGE="$XTDRONE_DIR/communication/multirotor_communication.py"

# 协同工作空间：默认猜测仓库内 devel；可由环境变量指定
COORD_WS_SETUP="${COORD_WS_SETUP:-$REPO_ROOT/coordination/devel/setup.bash}"
SWARM_SCRIPTS="$REPO_ROOT/coordination/src/robocup_swarm/scripts"
BRIDGE_SCRIPT="$SWARM_SCRIPTS/yolo_target_bridge.py"
READY_SCRIPT="$REPO_ROOT/tools/wait_match_ready.py"
GRAPH_AUDIT_SCRIPT="$REPO_ROOT/tools/audit_match_graph.py"
# 导航包源码目录：manager/agent 均 import robocup_navigation，必须进 PYTHONPATH
NAV_SRC="$REPO_ROOT/coordination/src/robocup_navigation/src"
# 合规地图元数据（2026-10-07 重构，规则 §2.4/§2.5 无预读随机地图）：
# robocup_base.json 只含规则定值（bounds/frame/spawn）+ 空障碍全自由栅格；
# 真实占用栅格由各 swarm_agent 用 /scan 运行时 SLAM 构建（slam_grid.py），
# 经 /swarm/occupancy_grid 汇总给 manager。不再从 black_box.txt 生成任何地图。
FORMAL_METADATA_FILE="$REPO_ROOT/coordination/src/robocup_training_worlds/worlds/generated/robocup_base.json"
ROBOCUP_METADATA_FILE="${ROBOCUP_METADATA_FILE:-$FORMAL_METADATA_FILE}"
# 官方每场随机化下发的建筑矩形真值：仅官方 control_actor.py（放 actor 的机制）使用，
# 协同层任何节点不得读取（合规硬约束）。
BB2MD="$REPO_ROOT/coordination/src/robocup_training_worlds/scripts/black_box_to_metadata.py"
# YOLO→swarm 桥开关：target_report → /swarm/target_states（规则 §2.5.11）
YOLO_BRIDGE="${YOLO_BRIDGE:-1}"
# 裁判按仿真钟控制单场时长；墙钟硬停会在低 RTF 时过早终止。
# 0=由官方裁判结束；需要本地限时调试时可显式设置 MATCH_HARD_CAP。
MATCH_HARD_CAP="${MATCH_HARD_CAP:-0}"
# 官方 robocup.launch 六机固定出生坐标；同时作为 MAVROS 局部系→世界系偏移。
# 三种本队场景 launch 必须使用相同次序，否则视觉定位与搜索坐标会错位。
TAKEOFF_X=(0 3 0 3 0 3)
TAKEOFF_Y=(-3 -3 0 0 3 3)

# 6 架飞机（官方 robocup.launch 的命名空间）
UAVS=(typhoon_h480_0 typhoon_h480_1 typhoon_h480_2
      typhoon_h480_3 typhoon_h480_4 typhoon_h480_5)
# 感知节点默认 6 机全起：每架带各自相机 YOLO，飞出后才不致盲（32核可承载）。
# 正式启动的 preflight 强制六机全起；单机试验请用独立调试入口。
read -r -a PR_UAV_ARR <<< "${PR_UAVS:-${UAVS[*]}}"

# 场景 launch：默认与 SKIP_RADAR 联动（默认值在下方 SKIP_RADAR 解析后确定）；
# 可显式覆盖为其他文件路径；预检要求其六机出生点与官方固定阵列一致。
SCENE_LAUNCH="${SCENE_LAUNCH:-}"
FORMAL_SCENE_LAUNCH="$REPO_ROOT/launch/robocup_with_laser.launch"
GAZEBO_GUI="${GAZEBO_GUI:-true}"
# 1=Gazebo 暂停启动，全部节点（judge/感知/桥/manager/agents）注册完成后统一
# unpause。复盘（2026-10-01 234150 局）：旧流程 gzserver 一起就跑，6 个 PX4
# lockstep 建立前世界以超高 RTF 空跑（~80 墙钟秒烧掉 sim 1807s），EKF 在
# 异常时序下全程反复原点 reset（2030/2038/2265/2386 多次 3~73m 跳变），
# offset 重锚定把错位坐标当"保持"，agent 误判越界、指令入障 → agent_3/5 坠地。
# 默认不暂停：原默认值=1（"6 机同步起步"）会导致启动期死锁——
#   暂停时 gazebo 不推进 sim 时间 → sim→gst→srv 启动卡在 blocked 中（X秒后 X 秒到 set -e）
#   所以默认改 paused=0 + unpause_physics 在 Gazebo 服务就绪后调、MTX50 循环。
# 高级用户要 "6 机同步起步" 可手动启 GAZEBO_START_PAUSED=1 + unpause 会同步调。
START_PAUSED="${GAZEBO_START_PAUSED:-0}"
GATE_TIMEOUT="${GATE_TIMEOUT:-180}"
# 1=由 swarm_agent 消费 /scan 并在唯一速度控制器内做雷达安全层；
# 0=启动旧版 radar_avoid 独立位置控制器（仅用于单独雷达实验）。
RADAR_GUARD="${RADAR_GUARD:-1}"
export RADAR_GUARD
# SKIP_RADAR 联动：默认 2（推荐 → 2D 激光雷达避障链路，CPU ray 安全版）。
#  SKIP_RADAR=2 → robocup_with_laser.launch（CPU ray 2D 激光，规则 §2.5(7) 真避障）
#    用新模型 typhoon_h480_laser：基于 PX4 原生 typhoon_h480（含双目相机、保留
#    感知链路），在 base_link +z 0.12m 挂 libgazebo_ros_laser（CPU 射线检测，不走
#    OGRE），每机话题 /typhoon_h480_N/scan。dwa_avoidance.py 默认 _scan_topic:=
#    /<u>/scan，run_match.sh 已按机注入，无需改代码。
#    关键修复：彻底替代 robocup_lidar_v2.launch 的 hokuyo_lidar (gpu_ray)——
#    gpu_ray 启动 OGRE 离屏渲染线程，与 robocup.world 6 个 actor 的 ActorCollisionsPlugin
#    骨骼动画在 dxSpace 上数据竞争 → libgazebo_ode 段错误；CPU ray 完全不碰 OGRE。
#  SKIP_RADAR=1 → robocup_nolidar.launch（无雷达回退，仅 swarm 自主）
#    默认协同链路；没有 /scan，dwa_avoidance 退到"无 scan 应急"分支（速度上限收紧）。
#  SKIP_RADAR=0 → robocup_lidar_v2.launch（gpu_ray 雷达，需 6 份航点）
#    旧版，保留做对照实验，官方规则下仍易触发段错误，不推荐比赛用。
SKIP_RADAR="${SKIP_RADAR:-2}"
if [ -z "$SCENE_LAUNCH" ]; then
    case "$SKIP_RADAR" in
        0) SCENE_LAUNCH="$REPO_ROOT/launch/robocup_lidar_v2.launch" ;;
        1) SCENE_LAUNCH="$REPO_ROOT/launch/robocup_nolidar.launch" ;;
        2|*) SCENE_LAUNCH="$REPO_ROOT/launch/robocup_with_laser.launch" ;;
    esac
fi
REQUIRE_SWARM="${REQUIRE_SWARM:-1}"
FORCE="${FORCE:-0}"

# ---- 官方环境前提保证（幂等；.bashrc 已持久化，但非交互 shell/子进程可能缺失）----
# actor 模型与移动插件依赖这两条路径；缺失则在启动场景前补上，不依赖调用方式。
ensure_colon_path(){  # ensure_colon_path <变量名> <目录>
    local _var="$1" _dir="$2" _cur
    eval "_cur=\${$_var:-}"
    case ":$_cur:" in
        *":$_dir:"*) : ;;
        *) export "$_var=${_cur:+$_cur:}$_dir";;
    esac
}
ensure_colon_path GAZEBO_MODEL_PATH  "$XTDRONE_DIR/sitl_config/models"
ensure_colon_path GAZEBO_PLUGIN_PATH "$HOME/catkin_ws/devel/lib"
# 2026-10-01 撞楼复盘补：typhoon_h480.sdf 的 mavlink_interface / gps 等 SITL 插件
# 在 PX4 build_gazebo 下——缺失时 spawn 飞机即 gzserver exit 255（插件 so 找不到）。
ensure_colon_path GAZEBO_PLUGIN_PATH "$PX4_ROOT/build/px4_sitl_default/build_gazebo"

# ---- 运行状态目录 ----
# 国家一等奖优化（2026-10-04 复盘）：日志默认落盘到工程目录 logs/ 子目录，
#   便于赛后复盘；/tmp 重启清空导致历史无法追溯。
#   调试 / CI 环境可用 RUN_DIR=/tmp/robocup_match 覆盖。
RUN_DIR="${RUN_DIR:-$REPO_ROOT/logs}"
MIN_FREE_DISK_MB="${MIN_FREE_DISK_MB:-2048}"
REG="$RUN_DIR/components.tsv"
STAMP="$(date '+%Y%m%d_%H%M%S')"
LOGDIR="$RUN_DIR/logs_$STAMP"

# ============================ 输出辅助 ======================================
if [ -t 1 ]; then C_G=$'\033[32m'; C_Y=$'\033[33m'; C_R=$'\033[31m'; C_B=$'\033[36m'; C_0=$'\033[0m'
else C_G=""; C_Y=""; C_R=""; C_B=""; C_0=""; fi
info(){ echo "${C_B}[*]${C_0} $*"; }
ok(){   echo "${C_G}[+]${C_0} $*"; }
warn(){ echo "${C_Y}[!]${C_0} $*"; }
err(){
    echo "${C_R}[x]${C_0} $*" >&2
    # Bash 的动态作用域让 preflight 收集自身检查失败原因；运行阶段不收集。
    if [ "${_collect_preflight_errors:-0}" = "1" ]; then
        _preflight_errors+=("$*")
    fi
    return 0
}
step(){ echo; echo "${C_B}==== $* ====${C_0}"; }

# 判断某个 X 显示号对应的 socket 是否真实存在（:0 / :95 -> X0 / X95）
_x_socket_exists(){
    local full="$1" num
    [ -z "$full" ] && return 1
    num="${full##*:}"          # 去主机/冒号前缀
    num="${num%%.*}"           # 去屏幕号(.0)
    [ -S "/tmp/.X11-unix/X$num" ]
}

# 选一个真实可用的 DISPLAY：现有 DISPLAY 有效则保留；否则回退物理 :0。
# 都没有则返回非零（裁判/相机渲染会失败，由 preflight/调用方提示）。
pick_display(){
    if _x_socket_exists "${DISPLAY:-}"; then
        return 0
    fi
    if [ -S /tmp/.X11-unix/X0 ]; then
        export DISPLAY=:0
        return 0
    fi
    return 1
}

# ============================ ROS 环境 ======================================
# 经验坑：source 第三方 setup 时临时降级 set -u，避免未绑定变量整盘退出
ros_env(){
    set +u
    # shellcheck disable=SC1090
    [ -f "$ROS_SETUP" ] && source "$ROS_SETUP"
    # shellcheck disable=SC1090
    [ -f "$WS_SETUP" ]  && source "$WS_SETUP"
    # 感知组的环境脚本：Xvfb :95 / GAZEBO_* / ROS_PACKAGE_PATH / 模型库 URI 置空
    # shellcheck disable=SC1090
    [ -f "$ENV_SCRIPT" ] && source "$ENV_SCRIPT" >/dev/null 2>&1
    # 协同层工作空间（提供 robocup_swarm 消息）
    # shellcheck disable=SC1090
    [ -f "$COORD_WS_SETUP" ] && source "$COORD_WS_SETUP"
    # 两个 catkin 工作空间并非 overlay；后 source 的 setup 会覆盖先前的
    # PYTHONPATH，导致感知节点找不到 gazebo_msgs，启动后六机全无观测。
    export PYTHONPATH="$(dirname "$COORD_WS_SETUP")/lib/python3/dist-packages:$(dirname "$WS_SETUP")/lib/python3/dist-packages:${PYTHONPATH:-}"
    # DISPLAY 校正：env 脚本为「无头服务器」默认 Xvfb :95，但若本机没装 Xvfb，
    # :95 实际不存在，裁判 score_cal.py 的 cv2.imshow 会连不上 X 而核心转储。
    # 解析顺序：当前 DISPLAY 对应的 X socket 真实存在 → 保留；否则回退物理 :0。
    pick_display
    set -u
    # 与 .bashrc 一致：同时加入 PX4_Firmware 与 sitl_gazebo（launch 内
    # $(find px4) / $(find mavlink_sitl_gazebo) 都依赖，非交互环境下也能解析）
    ensure_colon_path ROS_PACKAGE_PATH "$PX4_ROOT"
    ensure_colon_path ROS_PACKAGE_PATH "$PX4_ROOT/Tools/sitl_gazebo"
    # 仓库 launch 包（serial spawn：$(find robocup_team_launch) 依赖）
    ensure_colon_path ROS_PACKAGE_PATH "$REPO_ROOT/launch"
    # 协同层 setup.bash 会覆盖 ROS_PACKAGE_PATH（丢掉 catkin_ws），补回
    # ros_actor_cmd_pose_plugin_msgs / 雷达插件等 catkin_ws 内的包
    ensure_colon_path ROS_PACKAGE_PATH "$HOME/catkin_ws/src"
    # 同理：ENV_SCRIPT 可能覆盖 PYTHONPATH（丢掉 catkin_ws 的 Python 模块）
    # gazebo_ros Python 模块（spawn_model 依赖）在 catkin_ws/devel 下
    ensure_colon_path PYTHONPATH "$HOME/catkin_ws/devel/lib/python3/dist-packages"
    # 同理补 CMAKE_PREFIX_PATH：协同层 workspace 编译时未以 catkin_ws 为父工作空间，
    # 其 devel/setup.bash 会把 CMAKE_PREFIX_PATH 里的 catkin_ws/devel 替换为
    # coordination/devel。gzserver wrapper 用 `catkin_find libgazebo_ros_paths_plugin.so`
    # 定位插件，找不到时 -s 参数为空 → gzserver 启动即 exit 255。
    ensure_colon_path CMAKE_PREFIX_PATH "$HOME/catkin_ws/devel"
}

# ============================ 进程组管理 ====================================
# 注册: start_group <名称> <日志文件> <命令...>
start_group(){
    local name="$1" logfile="$2"; shift 2
    mkdir -p "$LOGDIR"
    setsid nohup "$@" >"$logfile" 2>&1 </dev/null &
    local pgid=$!
    printf '%s\t%s\t%s\n' "$name" "$pgid" "$logfile" >> "$REG"
    info "已启动 $name (pgid=$pgid) -> $logfile"
}

group_pgid(){ awk -F'\t' -v n="$1" '$1==n{print $2; exit}' "$REG" 2>/dev/null; }
group_log(){  awk -F'\t' -v n="$1" '$1==n{print $3; exit}' "$REG" 2>/dev/null; }

alive_pgid(){ kill -0 -- "-$1" 2>/dev/null; }
# setsid/nohup 在 roslaunch/Python 启动较慢时，短窗口内可能读到旧的进程组
# 状态；manager 已完成初始化但组检查暂时失败时，用唯一脚本名做二次确认。
swarm_manager_alive(){
    local pgid="$1"
    alive_pgid "$pgid" && return 0
    pgrep -f '[s]warm_manager.py' >/dev/null 2>&1
}

stop_group(){
    local name="$1" pgid; pgid="$(group_pgid "$name")"
    [ -z "$pgid" ] && return 0
    if alive_pgid "$pgid"; then
        kill -TERM -"$pgid" 2>/dev/null || true
        local w=0; while [ $w -lt 6 ] && alive_pgid "$pgid"; do sleep 1; w=$((w+1)); done
        kill -KILL -"$pgid" 2>/dev/null || true
        info "已停止 $name"
    fi
    # 每条命令独立容错，不做复合 pkill（历史踩坑）
}

# ============================ 就绪检查 ======================================
# 通用等待: wait_gate <描述> <超时秒> <谓词函数>
wait_gate(){
    local desc="$1" timeout="$2"; shift 2
    local started=$SECONDS deadline=$((SECONDS + timeout)) elapsed remaining
    while :; do
        remaining=$((deadline - SECONDS))
        if [ "$remaining" -le 0 ]; then
            err "$desc —— 等待超时（${timeout}s）"
            return 1
        fi
        # 谓词自身的 ROS 命令也必须受剩余预算限制，不能把一次 8~10s
        # 的 rostopic/rosservice 阻塞误算成只过了 2s。
        if "$@" "$remaining"; then
            ok "$desc（$((SECONDS - started))s）"
            return 0
        fi
        elapsed=$((SECONDS - started))
        if [ "$desc" != "Gazebo 服务就绪" ] && [ "$elapsed" -ge 2 ] && \
                ! pgrep -x gzserver >/dev/null 2>&1; then
            err "$desc —— Gazebo 已退出；检查 $LOGDIR/01_scene.log"
            return 1
        fi
        if [ "$desc" = "Gazebo 服务就绪" ] && [ "$elapsed" -ge 4 ]; then
            if grep -Eq 'Segmentation fault \(core dumped\)|process has died.*exit code 139' \
                    "$LOGDIR/01_scene.log" 2>/dev/null || ! pgrep -x gzserver >/dev/null 2>&1; then
                err "Gazebo 启动崩溃；检查 $LOGDIR/01_scene.log（不再空等 ${timeout}s）"
                return 1
            fi
        fi
        remaining=$((deadline - SECONDS))
        if [ "$remaining" -le 0 ]; then
            err "$desc —— 等待超时（${timeout}s）"
            return 1
        fi
        sleep "$((remaining < 2 ? remaining : 2))"
    done
}

has_service(){
    local limit="${2:-8}"
    [ "$limit" -gt 8 ] && limit=8
    timeout "${limit}s" rosservice list 2>/dev/null | grep -q "$1"
}
mavros_connected(){
    local limit="${2:-8}"
    [ "$limit" -gt 8 ] && limit=8
    [ "$(timeout "${limit}s" rostopic echo -n1 "/$1/mavros/state" 2>/dev/null \
        | grep -c 'connected: True')" = "1" ]
}
first_msg(){
    local limit="${2:-10}"
    [ "$limit" -gt 10 ] && limit=10
    timeout "${limit}s" rostopic echo -n1 "$1" >/dev/null 2>&1
}
count_pattern(){ pgrep -fc "$1" 2>/dev/null || true; }
# 判断话题的发布者中是否有节点名匹配片段（身份卡关，不只看有没有发布者）
topic_publisher_match(){  # topic_publisher_match <topic> <节点名片段>
    local limit="${3:-8}"
    [ "$limit" -gt 8 ] && limit=8
    timeout "${limit}s" rostopic info "$1" 2>/dev/null \
      | awk '/Publishers:/{f=1;next}/Subscribers:/{f=0}f' \
      | grep -q "$2"
}

# ============================ 赛前体检 ======================================
preflight(){
    mkdir -p "$RUN_DIR"
    local hard_miss=0 _collect_preflight_errors=1
    local -a _preflight_errors=()
    echo "${C_B}================ RoboCup 赛前体检 ================${C_0}"

    # 进程残留检查：上一局 stop 没清干净会导致新局 gzserver 被顶掉、rosmaster 冲突。
    # 复盘 2026-10-02 23:29 局：残留 gzserver 顶掉新 gzserver → "new node registered
    # with same name" → sim 冻结整场（actor 不动 + 飞机不起飞 + yolo/协同全连锁失效）。
    _stale=""
    pgrep -x gzserver       >/dev/null 2>&1 && _stale="$_stale gzserver"
    pgrep -x gzclient       >/dev/null 2>&1 && _stale="$_stale gzclient"
    pgrep -f '[r]osmaster'  >/dev/null 2>&1 && _stale="$_stale rosmaster"
    if [ -n "$_stale" ]; then
        err "检测到残留进程：$_stale —— 会顶掉新启的 gzserver 导致 sim 冻结。"
        err "  修复：./run_match.sh stop（已默认清理 gzserver/gzclient/rosmaster）；"
        err "        或手动 pkill -9 -x gzserver gzclient; pkill -9 -f rosmaster"
        hard_miss=$((hard_miss+1))
    fi

    check_hard(){   # check_hard <描述> <路径/命令>
        if eval "$2" >/dev/null 2>&1; then ok "$1"
        else err "$1 缺失"; hard_miss=$((hard_miss+1)); fi
    }
    check_soft(){
        if eval "$2" >/dev/null 2>&1; then ok "$1"
        else warn "$1 —— 缺失（相关步骤将受限，详见下文）"; fi
    }

    check_hard "ROS Noetic setup"        "[ -f '$ROS_SETUP' ]"
    check_hard "catkin 工作空间 setup"   "[ -f '$WS_SETUP' ]"
    # 正式比赛只有完整六机视觉链路才可起飞。PR_UAVS 子集是开发配置，
    # 否则后面的“六路相机/推理就绪”可能实际只检查一架。
    if [ "${#PR_UAV_ARR[@]}" -eq "${#UAVS[@]}" ] && \
            [ "${PR_UAV_ARR[*]}" = "${UAVS[*]}" ]; then
        ok "正式感知列表为官方六机全量"
    else
        err "正式启动要求 PR_UAVS 与官方六机全量一致；当前: ${PR_UAV_ARR[*]:-空}"
        hard_miss=$((hard_miss+1))
    fi
    if [ "$SCENE_LAUNCH" = "$FORMAL_SCENE_LAUNCH" ]; then
        ok "正式场景使用官方地图和六机激光 launch"
    else
        err "正式启动禁用其他场景：$SCENE_LAUNCH"
        hard_miss=$((hard_miss+1))
    fi
    if [ "$ROBOCUP_METADATA_FILE" = "$FORMAL_METADATA_FILE" ] && \
            { [ -z "${ROBOCUP_METADATA:-}" ] || [ "$ROBOCUP_METADATA" = "$FORMAL_METADATA_FILE" ]; }; then
        ok "正式导航仅使用固定空图元数据"
    else
        err "正式启动禁用外部 ROBOCUP_METADATA/ROBOCUP_METADATA_FILE 覆盖"
        hard_miss=$((hard_miss+1))
    fi
    if [ "${FLIGHT_OUTPUT:-pos}" = "pos" ] && \
            [ "${FLIGHT_X_MIN:--50.0}" = "-50.0" ] && \
            [ "${FLIGHT_X_MAX:-150.0}" = "150.0" ] && \
            [ "${FLIGHT_Y_MIN:--60.0}" = "-60.0" ] && \
            [ "${FLIGHT_Y_MAX:-60.0}" = "60.0" ] && \
            [ "${FLIGHT_INSET:-2.0}" = "2.0" ]; then
        ok "位置输出与官方人物活动范围护栏固定"
    else
        err "正式启动禁用 FLIGHT_OUTPUT 或 FLIGHT_[XY] 边界参数覆盖"
        hard_miss=$((hard_miss+1))
    fi
    if python3 -c 'import netifaces; netifaces.interfaces()' >/dev/null 2>&1; then
        ok "ROS 本机网络接口可访问"
    else
        err "ROS 本机网络接口不可访问；roslaunch 无法启动（检查容器/沙箱网络权限）"
        hard_miss=$((hard_miss+1))
    fi
    check_hard "官方地图 robocup.world"  "[ -f '$OFFICIAL_WORLD' ]"
    check_hard "XTDrone 控制脚本"        "[ -f '$ROBO_DIR/control_actors.sh' ]"
    check_hard "官方裁判 score_cal.py"   "[ -f '$ROBO_DIR/score_cal.py' ]"
    check_hard "官方 control_actor.py"   "[ -f '$ROBO_DIR/control_actor.py' ]"
    if [ -f "$ROBO_DIR/score_cal.py" ] && grep -Eq 'from[[:space:]]+ros_actor_cmd_pose_plugin_msgs\.msg[[:space:]]+import[[:space:]]+ActorInfo' "$ROBO_DIR/score_cal.py"; then
        ok "裁判 ActorInfo 类型契约：ros_actor_cmd_pose_plugin_msgs/ActorInfo"
    else
        err "裁判未使用官方 ros_actor_cmd_pose_plugin_msgs/ActorInfo；停止启动"
        hard_miss=$((hard_miss+1))
    fi
    if [ -d "$XTDRONE_DIR/.git" ]; then
        if git -C "$XTDRONE_DIR" diff --quiet origin/master -- robocup/score_cal.py robocup/control_actor.py; then
            ok "裁判与人物控制代码符合本地官方 origin/master"
        else
            err "裁判或人物控制代码偏离本地官方 origin/master；正式运行前恢复官方版本"
            hard_miss=$((hard_miss+1))
        fi
    fi
    if [ "${D2O_ENABLE:-0}" != "0" ]; then
        err "正式赛禁用 D2O_ENABLE：该开发桥不在正式启动链路中"
        hard_miss=$((hard_miss+1))
    fi
    if [ "${SIM_TARGET_NODE:-0}" != "0" ]; then
        err "正式赛禁用 SIM_TARGET_NODE：该节点从 Gazebo 真值生成目标"
        hard_miss=$((hard_miss+1))
    fi
    if [ "${SWARM_CALIB:-spawn}" = "truth" ]; then
        err "正式赛禁用 SWARM_CALIB=truth：会读取 /gazebo/model_states"
        hard_miss=$((hard_miss+1))
    fi
    if [ "$RADAR_GUARD" != "1" ]; then
        err "正式赛要求 RADAR_GUARD=1：旧雷达控制器会读取障碍真值文件"
        hard_miss=$((hard_miss+1))
    fi
    if [ "$YOLO_BRIDGE" != "1" ]; then
        err "正式赛要求 YOLO_BRIDGE=1：缺少合法目标上报与协同检测源"
        hard_miss=$((hard_miss+1))
    fi
    if [ "${PR_DIRECT_ACTOR_INFO:-0}" != "0" ]; then
        err "正式赛禁用 PR_DIRECT_ACTOR_INFO：六个感知进程会与唯一 YOLO 桥争抢裁判话题"
        hard_miss=$((hard_miss+1))
    fi
    if [ "$SKIP_RADAR" != "2" ]; then
        err "正式赛要求 SKIP_RADAR=2：六机均需 2D 激光避障"
        hard_miss=$((hard_miss+1))
    fi

    # 赛前磁盘保护：裁判和 Gazebo 日志持续写盘；磁盘满曾导致 score_cal.py 直接崩溃。
    # 只检查 RUN_DIR 所在文件系统，不清理用户文件，低于阈值直接阻止启动。
    _free_mb="$(df -Pm "$RUN_DIR" 2>/dev/null | awk 'NR==2 {print $4}')"
    if [ -n "$_free_mb" ] && [ "$_free_mb" -ge "$MIN_FREE_DISK_MB" ]; then
        ok "日志文件系统可用空间 ${_free_mb}MB（门槛 ${MIN_FREE_DISK_MB}MB）"
    else
        err "日志文件系统可用空间不足：${_free_mb:-未知}MB（门槛 ${MIN_FREE_DISK_MB}MB）"
        hard_miss=$((hard_miss+1))
    fi
    # black_box.txt / obstacle.txt 只供官方环境节点使用，参赛启动器不读取。
    check_hard "actor 模型目录"          "[ -d '$ACTOR_MODEL_DIR' ]"
    check_soft "地图生成器 map_generator.py（重新出图时才需要）" "[ -f '$MAP_GENERATOR' ]"
    check_hard "YOLO 权重 v1（双目）"    "[ -f '$WEIGHTS' ]"
    check_hard "感知主节点"              "[ -f '$PERC_DIR/perception_real.py' ]"
    # 双目相机补丁：感知订阅 /<uav>/stereo_camera/left/image_raw，原厂 typhoon_h480
    # 只有 cgo3 云台相机、无 stereo_camera → 不打补丁则话题发布者为空、整局零检测。
    # 补丁脚本只读检查，不改任何文件；NOT_INSTALLED 即硬失败，避免盲飞整场。
    STEREO_PATCH="$PERC_DIR/patch/add_stereo_camera.py"
    if [ -f "$STEREO_PATCH" ]; then
        # 从场景 launch 提取全部 sdf 机体模型，逐一检查补丁状态（PX4 侧为准）
        _models="$(grep -o '<arg name="sdf" value="[^"]*"' "$SCENE_LAUNCH" 2>/dev/null \
            | sed 's/.*value="//;s/"//' | sort -u)"
        [ -z "$_models" ] && _models="typhoon_h480"
        for _m in $_models; do
        # 注意：mawk 不支持 `$0 ~ m"re"` 直接拼接（优先级坑），正则必须在 BEGIN 里先组装
            _sp_state="$(python3 "$STEREO_PATCH" --check --model "$_m" --px4-root "$PX4_ROOT" 2>/dev/null \
                | awk -v m="$_m" 'BEGIN{re=m "\\.sdf[ \t]"} $0 ~ /\[PX4\]/{p=1} p && $0 ~ re{print $2; exit}')"
        case "$_sp_state" in
            INSTALLED*) ok "双目相机补丁已装：$_m（$_sp_state）";;
            NOT_INSTALLED|BAD_INCLUDE)
                err "双目相机补丁未装/坏：$_m=$_sp_state"
                err "  感知订阅 stereo_camera/left/image_raw，该模型无 stereo → 整局零检测。"
                err "  修复：python3 '$STEREO_PATCH' --px4-root '$PX4_ROOT' --model $_m"
                hard_miss=$((hard_miss+1));;
            MISSING)
                err "机体模型 $_m 不在 PX4 模型目录（场景 launch 引用了不存在的 sdf）"
                hard_miss=$((hard_miss+1));;
            *)
                # 体检只读，不应 FAIL_VERIFY；若出现也按硬失败处理。
                err "双目补丁状态异常（$_m: $_sp_state）——重打补丁后重试"
                hard_miss=$((hard_miss+1));;
        esac
        done
    else
        err "双目补丁脚本缺失 $STEREO_PATCH"
        hard_miss=$((hard_miss+1))
    fi
    # control_actors.sh 依赖 python 软链；必须真实指向 python3
    if command -v python >/dev/null 2>&1 && python --version 2>&1 | grep -q "Python 3"; then
        ok "python 软链 -> $(python --version 2>&1)"
    else err "python 缺失或未指向 python3（control_actors.sh 会失败）"; hard_miss=$((hard_miss+1)); fi
    # 图形显示：裁判 cv2.imshow / Gazebo 相机渲染都要连得上真实存在的 X server。
    if pick_display; then
        ok "可用图形显示 DISPLAY=$DISPLAY（裁判计分窗 / 相机渲染）"
    elif command -v Xvfb >/dev/null 2>&1; then
        ok "无物理显示但已装 Xvfb，env 脚本将自动起 :95"
    else
        err "无可用 DISPLAY 且未装 Xvfb：裁判会核心转储。本机有屏幕请确认在图形会话，或 sudo apt install xvfb"
        hard_miss=$((hard_miss+1))
    fi
    # 官方环境变量前提（.bashrc 持久化）
    case ":${GAZEBO_MODEL_PATH:-}:" in
        *":$XTDRONE_DIR/sitl_config/models:"*) ok "GAZEBO_MODEL_PATH 含 XTDrone 模型目录";;
        *) err "GAZEBO_MODEL_PATH 缺少 $XTDRONE_DIR/sitl_config/models（actor 无法加载）"
           hard_miss=$((hard_miss+1));;
    esac
    case ":${GAZEBO_PLUGIN_PATH:-}:" in
        *":$HOME/catkin_ws/devel/lib:"*) ok "GAZEBO_PLUGIN_PATH 含 catkin_ws/devel/lib（移动插件）";;
        *) err "GAZEBO_PLUGIN_PATH 缺少 $HOME/catkin_ws/devel/lib（恐怖分子不会动）"
           hard_miss=$((hard_miss+1));;
    esac
    # actor 移动插件本体：robocup.world 每个 actor 都引用 libros_actor_cmd_pose_plugin.so，
    # 缺失时 /actor_N/cmd_motion 无订阅者 → 6 个恐怖分子叠在原点不动（GUI 里"只有 1 个actor"）。
    # 2026-10-03 复盘：catkin_ws 重建时丢了该包（源码在回收站），从 Trash 恢复后重编译。
    check_hard "actor 移动插件 libros_actor_cmd_pose_plugin.so" \
        "[ -f '$HOME/catkin_ws/devel/lib/libros_actor_cmd_pose_plugin.so' ]"
    # catkin build 在本机把新库写入 build/，不会覆盖 devel/lib 的旧实体文件；
    # Gazebo 只从 devel/lib 加载。防止源码更新后仍运行旧 actor 插件。
    actor_build="$HOME/catkin_ws/build/ros_actor_cmd_pose_plugin/libros_actor_cmd_pose_plugin.so"
    actor_runtime="$HOME/catkin_ws/devel/lib/libros_actor_cmd_pose_plugin.so"
    actor_source="$HOME/catkin_ws/src/gazebo_ros_actor_cmd_plugin/src/ActorPluginRos.cpp"
    actor_header="$HOME/catkin_ws/src/gazebo_ros_actor_cmd_plugin/include/actor_plugin_ros/ActorPluginRos.hpp"
    if [ -f "$actor_build" ]; then
        if [ "$actor_source" -nt "$actor_build" ] || [ "$actor_header" -nt "$actor_build" ]; then
            err "actor 插件源码比编译产物新；先 catkin build ros_actor_cmd_pose_plugin"
            hard_miss=$((hard_miss+1))
        elif ! cmp -s "$actor_build" "$actor_runtime"; then
            err "actor 插件 devel/lib 仍是旧版；将 build/ros_actor_cmd_pose_plugin/libros_actor_cmd_pose_plugin.so 安装到 devel/lib"
            hard_miss=$((hard_miss+1))
        else
            ok "actor 插件源码、编译产物和 Gazebo 加载文件一致"
        fi
    fi
    case ":${GAZEBO_PLUGIN_PATH:-}:" in
        *":$PX4_ROOT/build/px4_sitl_default/build_gazebo:"*) ok "GAZEBO_PLUGIN_PATH 含 PX4 build_gazebo（SITL 模型插件）";;
        *) err "GAZEBO_PLUGIN_PATH 缺少 $PX4_ROOT/build/px4_sitl_default/build_gazebo（spawn 飞机即 gzserver 255 崩溃）"
           hard_miss=$((hard_miss+1));;
    esac

    check_soft "XTDrone 通信桥（actor 躲避反应依赖它）" "[ -f '$COMM_BRIDGE' ]"
    check_hard "协同层工作空间（robocup_swarm 消息）" "[ -f '$COORD_WS_SETUP' ]"
    check_hard "YOLO→swarm 桥 yolo_target_bridge.py" "[ -f '$BRIDGE_SCRIPT' ]"
    check_hard "多机消息就绪检查工具" "[ -f '$READY_SCRIPT' ]"
    check_hard "正式 ROS 图审计工具" "[ -f '$GRAPH_AUDIT_SCRIPT' ]"
    check_hard "合规空图元数据 robocup_base.json" \
        "[ -f '$ROBOCUP_METADATA_FILE' ]"
    if python3 -c "import pyquaternion" >/dev/null 2>&1; then
        ok "pyquaternion 已安装（multirotor_communication 依赖）"
    else
        err "pyquaternion 未安装：执行 python3 -m pip install --user pyquaternion"
        hard_miss=$((hard_miss+1))
    fi

    # 实际要使用的场景 launch 必须存在（默认值已按 SKIP_RADAR 选定）
    check_hard "场景 launch（$SCENE_LAUNCH）" "[ -f '$SCENE_LAUNCH' ]"
    if [ -f "$SCENE_LAUNCH" ] && [ -f "$ROBOCUP_METADATA_FILE" ]; then
        if python3 "$REPO_ROOT/tools/check_official_spawn.py" \
            --scene "$SCENE_LAUNCH" --run-script "$REPO_ROOT/run_match.sh" \
            --metadata "$ROBOCUP_METADATA_FILE"; then
            ok "官方固定出生点与坐标偏移一致"
        else
            err "出生点偏离官方固定阵列或世界坐标偏移不一致"
            hard_miss=$((hard_miss+1))
        fi
    fi

    # 正式链路只使用 CPU ray 2D 激光模型；第⑤步验证真实帧。
    LASER_PX4="$PX4_ROOT/Tools/sitl_gazebo/models/typhoon_h480_laser/typhoon_h480_laser.sdf"
    LASER_XTD="$XTDRONE_DIR/sitl_config/models/typhoon_h480_laser/typhoon_h480_laser.sdf"
    check_hard "2D 激光模型 typhoon_h480_laser（PX4 侧）" "[ -f '$LASER_PX4' ]"
    check_hard "2D 激光模型 typhoon_h480_laser（XTDrone 镜像）" "[ -f '$LASER_XTD' ]"
    # 官方规则要求相机光学规格和激光扫描规格与 XTDrone 自带传感器一致。
    if python3 "$REPO_ROOT/tools/check_official_sensor_spec.py" \
            --team "$LASER_PX4" --mirror "$LASER_XTD" \
            --hokuyo "$XTDRONE_DIR/sitl_config/models/hokuyo_lidar/model.sdf" \
            --stereo "$XTDRONE_DIR/sitl_config/models/stereo_camera/model.sdf"; then
        ok "双目与 2D 激光参数符合 XTDrone 原生传感器"
    else
        err "传感器参数与 XTDrone 原生模型不一致，正式启动中止"
        hard_miss=$((hard_miss+1))
    fi

    # 需要 ROS 环境才能验证的项
    if [ -f "$ROS_SETUP" ]; then
        ros_env
        if rospack find px4 >/dev/null 2>&1; then ok "ROS 可见 px4 包"
        else err "ROS 找不到 px4 包（检查 ROS_PACKAGE_PATH=$PX4_ROOT）"; hard_miss=$((hard_miss+1)); fi
        if rospack find ros_actor_cmd_pose_plugin_msgs >/dev/null 2>&1; then
            ok "ActorInfo 消息包就位"
        else err "ros_actor_cmd_pose_plugin_msgs 未在 catkin 工作空间中"; hard_miss=$((hard_miss+1)); fi
    fi

    echo
    echo "${C_B}---- 运行时仍会强制卡关的验证 ----${C_0}"
    info "A. /typhoon_h480_N/scan：待场景启动后并行验证六机真实帧"
    info "B. /swarm/target_states：待第④步由 yolo_target_bridge 从 YOLO 检测喂入（当前不计缺失）"
    echo "      （启动后校验发布者身份；第⑥步后自动做节点订阅合规审计）。"

    echo
    if [ "$hard_miss" -eq 0 ]; then
        ok "启动检查全部通过。"; return 0
    else
        _collect_preflight_errors=0
        err "启动检查失败 $hard_miss 项，请处理以下原因后再 start："
        local failure
        for failure in "${_preflight_errors[@]}"; do
            err "  $failure"
        done
        return 2
    fi
}

# ============================ 启动全场 ======================================
do_start(){
    local match_started_wall
    match_started_wall="$(date +%s.%N)"
    mkdir -p "$RUN_DIR" "$LOGDIR"
    ln -sfn "$LOGDIR" "$RUN_DIR/latest"

    if [ -s "$REG" ] && [ "$FORCE" != "1" ]; then
        spgid="$(awk -F'\t' '$1=="scene"{print $2; exit}' "$REG")"
        if [ -n "$spgid" ] && kill -0 -- "-$spgid" 2>/dev/null; then
            err "已有一场比赛在跑（$REG）。先 ./run_match.sh stop，或 FORCE=1 强制。"; exit 1
        fi
    fi
    preflight >"$RUN_DIR/preflight.out" 2>&1 || { cat "$RUN_DIR/preflight.out"; exit 2; }
    # A failed preflight must not erase the previous registry: stop needs its
    # process groups to clean up a half-finished or timed-out match.
    : > "$REG"
    grep -E '^\[!\]|^----|无法' "$RUN_DIR/preflight.out" || true
    ros_env
    # 正式执行的环境与 preflight 核对值保持一致，防止外部 shell 配置渗入子进程。
    export FLIGHT_OUTPUT=pos FLIGHT_X_MIN=-50.0 FLIGHT_X_MAX=150.0
    export FLIGHT_Y_MIN=-60.0 FLIGHT_Y_MAX=60.0 FLIGHT_INSET=2.0
    # ros_env 才加载 robocup_swarm 的生成消息；此处用运行时实际导入的类核对类型名和 MD5。
    if ! python3 -c 'from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo; print("%s %s" % (ActorInfo._type, ActorInfo._md5sum))' \
        >"$RUN_DIR/actorinfo_contract.txt" 2>&1; then
        err "无法导入运行时 ros_actor_cmd_pose_plugin_msgs/ActorInfo；检查 catkin 编译"
        exit 2
    fi
    info "ActorInfo 运行时契约：$(tr '\n' ' ' < "$RUN_DIR/actorinfo_contract.txt")"
    python3 "$REPO_ROOT/tools/write_run_manifest.py" \
        --root "$REPO_ROOT" --official "$ROBO_DIR" \
        --started "$match_started_wall" --output "$LOGDIR/code_manifest.json" \
        || { err "启动版本留档失败，停止启动"; exit 2; }

    # 启动中途失败：清理已起组件
    partial_fail(){
        err "$*"
        err "启动中断，开始清理已起组件 ..."
        while IFS=$'\t' read -r name pgid _log; do stop_group "$name"; done < "$REG"
        exit 1
    }

    # ---------------- ① 场景：Gazebo + 6×PX4 + MAVROS ----------------
    step "① 起场景 $SCENE_LAUNCH (gui=$GAZEBO_GUI paused=$START_PAUSED)"
    if [ ! -f "$SCENE_LAUNCH" ]; then
        partial_fail "场景 launch 文件不存在：$SCENE_LAUNCH"
    fi
    scene_cmd=(roslaunch "$SCENE_LAUNCH" "gui:=$GAZEBO_GUI" "paused:=$START_PAUSED")
    start_group scene "$LOGDIR/01_scene.log" "${scene_cmd[@]}"
    wait_gate "Gazebo 服务就绪" "$GATE_TIMEOUT" has_service '/gazebo/unpause_physics' \
        || partial_fail "场景没起来，看 $LOGDIR/01_scene.log"
    # 服务出现不代表模型就绪；立即检查进程，然后由下面六机真实 MAVROS
    # 连接检查接替固定 5 秒等待。后续仍检查相机、雷达及首次推理。
    pgrep -x gzserver >/dev/null 2>&1 || partial_fail \
        "Gazebo 在模型生成阶段退出；检查 $LOGDIR/01_scene.log"

    # ---- 若启用了 START_PAUSED=1：在等 MAVROS 之前先 unpause，否则 SITL 的 simulator start 会卡死 ----
    if [ "$START_PAUSED" = "1" ]; then
        step "Gazebo 服务就绪后统一 unpause（避免 SITL simulator start 卡死）"
        ucommon=0; out=""
        for n in $(seq 1 12); do
            out="$(rosservice call /gazebo/unpause_physics 2>&1)" || true
            if echo "$out" | grep -q "success: True"; then
                ucommon=1; break
            fi
            sleep 0.3
        done
        if [ "$ucommon" = "1" ]; then
            ok "仿真已 unpause，sim 时钟开始推进（RTF≈1）"
        else
            warn "unpause 失败（最后返回：$out）——手动: rosservice call /gazebo/unpause_physics"
        fi
    fi

    timeout 305s python3 -u "$READY_SCRIPT" --kind state --timeout 300 \
        --uavs "${UAVS[@]}" \
        || partial_fail "六机 MAVROS 未全部连接；检查上面的缺失机清单及 $LOGDIR/01_scene.log"

    # ---------------- ② XTDrone 通信桥 + 6 个恐怖分子 ----------------
    step "② 起 XTDrone 通信桥 + control_actors"
    if [ -f "$COMM_BRIDGE" ]; then
        for i in 0 1 2 3 4 5; do
            start_group "comm_$i" "$LOGDIR/02_comm_$i.log" \
                python3 -u "$COMM_BRIDGE" typhoon_h480 "$i"
            sleep 0.3
        done
        sleep 1
    else
        warn "通信桥缺失，跳过 —— actor 将只随机游走、不会躲避"
    fi

    # PYTHONUNBUFFERED=1：control_actors.sh 内 python 无 -u，块缓冲导致 02_actors.log
    # 整局空白（只能等崩溃 flush 才有内容），排查 actor 不动时两眼一抹黑。
    start_group actors "$LOGDIR/02_actors.log" bash -c "cd '$ROBO_DIR' && PYTHONUNBUFFERED=1 ./control_actors.sh"
    actors_ready(){
        [ "$(count_pattern '[c]ontrol_actor.py')" -ge 6 ]
    }
    wait_gate "6 个 actor 控制器进程" 30 actors_ready \
        || partial_fail "actor 控制器不足 6/6，看 $LOGDIR/02_actors.log"

    # ---------------- ③ 裁判计分 ----------------
    step "③ 起裁判 score_cal.py typhoon_h480"
    start_group judge "$LOGDIR/03_judge.log" bash -c \
        "cd '$ROBO_DIR' && python3 -u score_cal.py typhoon_h480"
    wait_gate "裁判发布者就绪" 30 topic_publisher_match /score score_cal \
        || partial_fail "裁判没起来，看 $LOGDIR/03_judge.log"

    # ---------------- ④ 感知（CPU 模式） ----------------
    step "④ 起感知 perception_real.py（CUDA_VISIBLE_DEVICES 置空，CPU 推理）"
    # 国家一等奖优化（2026-10-04 复盘）：
    #   PR_ACTOR_PUB_RANGE=80     边缘瞬移目标（70/72m）也接得上
    #   PR_MAX_COAST_PUB=12       外推余量加宽（远距短遮挡 6→12 帧 ≈ 1.5s）
    # ⚠ OFFICIAL_ARBITRATED/IDENTITY_GATE **不能开**：
    #   1. 当前架构走 snap → /coordination/target_report → bridge → /swarm/target_states，
    #      manager 全局派单依赖 6 架飞机的全景 detection；
    #   2. OFFICIAL_ARBITRATED=1 会让没被指派的 5 架 detection 全部被闸门剔除 → bridge
    #      收不到其他目标 → manager 失去全景派单能力；
    #   3. IDENTITY_GATE=1 要求本机 detection 与 bridge 融合坐标 ≤3m，而融合坐标来源
    #      正是本机 detection，会形成自指。远距鬼影由 bridge NEW_TRACK_CONF=0.55 拦截，
    #      这边只需要放宽距离 + 外推两项。
    # v23（2026-10-09）误检治理三连（t1 幽灵 310s 教训）：
    #  CONFIRM_HITS 2→3：2 帧(0.4s)太松，单帧误检 0.4s 就"确认"登记；
    #  ACTOR_PUB_RANGE 80→45：80m 距离门为边缘瞬移目标开的口子，实测从未接到
    #    边缘目标却放进 17~40m 远程误检（perception_real.py 注释实证 tid=6@36m）；
    #  ACTOR_CONFIRM_ONLY=1：未确认 track 一律不上报（队友实测 hits=2~3 假 track
    #    以 20~40m 误差清零裁判 15s 计时）。真目标登记延迟仅 +0.2s。
    #  ⚠ 注释必须放在 bash -c 字符串外：字符串内 \ 续行会把注释接进逻辑行，
    #    # 之后的环境变量前缀链全部被当成注释吃掉（v23 首跑 6 机 img_age=-1 全瞎的根因）。
    for u in "${PR_UAV_ARR[@]}"; do
        _spawn_id="${u##*_}"
        case "$_spawn_id" in
            0|1|2|3|4|5) ;;
            *) partial_fail "感知节点飞机名不符合官方六机编号：$u" ;;
        esac
        start_group "pr_$u" "$LOGDIR/04_perception_$u.log" bash -c \
            "cd '$PERC_DIR' && CUDA_VISIBLE_DEVICES='' \
             PR_UAV='$u' \
             PR_WORLD_OFFSET_X='${TAKEOFF_X[$_spawn_id]}' PR_WORLD_OFFSET_Y='${TAKEOFF_Y[$_spawn_id]}' \
             PR_CONFIRM_HITS=3 PR_DETECT_EVERY=2 PR_COORD_HZ=4 PR_ACTOR_PUB_RANGE=45 PR_PUB_EMA=0.5 \
             PR_ACTOR_CONFIRM_ONLY=1 PR_OFFICIAL_ARBITRATED=0 PR_IDENTITY_GATE=0 PR_DIRECT_ACTOR_INFO=0 \
             PR_RED_STICKY_R=2.5 \
             PR_MAX_COAST_PUB=18 \
             python3 -u perception_real.py"
    done

    # ---- ④b YOLO→swarm 桥：合法检测 → /swarm/target_states ----
        # Camera coordinates are measurements at image time. The bridge
        # extrapolates only by actual image age; no fixed latency override.
        start_group yolo_bridge "$LOGDIR/04b_yolo_bridge.log" bash -c \
            "cd '$SWARM_SCRIPTS' && \
             BRIDGE_DROP_TIME=4.0 BRIDGE_NEW_TRACK_CONF=0.55 BRIDGE_NEW_TRACK_FRAMES=2 \
             BRIDGE_EXTRAP_MAX_D=0.5 \
             python3 -u yolo_target_bridge.py"
        # ROS 主节点高负载时注册可晚于进程创建；按真实墙钟最多等待 35s。
        wait_gate "yolo_target_bridge 发布者注册" 35 \
            topic_publisher_match /swarm/target_states yolo_target_bridge \
            || partial_fail "yolo_target_bridge 未注册发布，看 $LOGDIR/04b_yolo_bridge.log"

    # 六路相机在一个 ROS 检查进程中同时订阅，仅记录非空帧到达；
    # 目标检测结果不能作为启动门槛，因为飞机起飞前可能还看不到人物。
    if timeout 125s python3 -u "$READY_SCRIPT" --kind camera --timeout 120 \
            --uavs "${PR_UAV_ARR[@]}"; then
        _perc_alive=1
        for u in "${PR_UAV_ARR[@]}"; do
            alive_pgid "$(group_pgid "pr_$u")" || _perc_alive=0
        done
        [ "$_perc_alive" = 1 ] || partial_fail "相机有帧但感知进程退出；查 $LOGDIR/04_perception_*.log"
        ok "感知进程全部在跑，六路相机收到真实帧"
    else
        # 感知无帧 = 整局零检测 = 必盲飞。旧逻辑只 warn，结果裁判判负后
        # 集群还在空烧 sim 时间。现改为：FORCE=0 硬中止，FORCE=1 才放行
        # （仅限已知风险下调试）。
        err "感知未全部就绪——查 $LOGDIR/04_perception_*.log"
        for u in "${PR_UAV_ARR[@]}"; do
            alive_pgid "$(group_pgid "pr_$u")" \
                || err "感知节点 $u 已退出"
            # 逐机诊断：节点活但没帧 = 话题发布者为空 = 双目补丁没打
            if [ "$START_PAUSED" != "1" ]; then
                if ! timeout 8 rostopic echo -n1 "/$u/stereo_camera/left/image_raw" \
                        >/dev/null 2>&1; then
                    _npub="$(timeout 8 rostopic info "/$u/stereo_camera/left/image_raw" 2>/dev/null \
                        | awk '/Publishers:/{f=1;next}/Subscribers:/{f=0}f' | wc -l)"
                    if [ "$_npub" = "0" ]; then
                        err "$u: stereo_camera/left/image_raw 无发布者 —— 双目补丁未打？跑 preflight 复查"
                    else
                        err "$u: 话题有发布者但 8s 内无帧 —— Gazebo 相机渲染异常（DISPLAY？）"
                    fi
                fi
            fi
        done
        if [ "$FORCE" = "1" ]; then
            warn "FORCE=1：感知未就绪仍继续（盲飞，仅用于已知风险调试）"
        else
            partial_fail "感知无帧/未就绪——补丁或相机渲染未修复前盲飞无意义，中止（FORCE=1 可强制）"
        fi
    fi
    perception_inference_ready(){
        local u
        for u in "${PR_UAV_ARR[@]}"; do
            alive_pgid "$(group_pgid "pr_$u")" || return 1
            grep -q '\[pr\] FIRST_INFERENCE ' "$LOGDIR/04_perception_$u.log" \
                2>/dev/null || return 1
        done
    }
    wait_gate "六路 YOLO 首次推理完成" 120 perception_inference_ready \
        || partial_fail "感知尚未完成首次推理；检查 $LOGDIR/04_perception_*.log"
    n_extra_yolo="$(count_pattern 'yolo_v11')"
    [ "$n_extra_yolo" = "0" ] && ok "无多余 YOLO 节点抢 CPU" \
        || warn "发现独立 yolo_v11 节点，会抢 CPU 拖垮发布频率"

    # ---------------- ⑤ 2D 激光雷达传感器 / 避障 ----------------
    step "⑤ 验证六机 2D 激光雷达真实帧"
    timeout 35s python3 -u "$READY_SCRIPT" --kind scan --timeout 30 \
        --uavs "${UAVS[@]}" \
        || partial_fail "2D 激光传感器有缺帧机；检查上面的清单和雷达模型"
    ok "六路 /scan 已验证；由 swarm_agent 内置安全层消费，不启动第二个控制器"

    # ---------------- ⑥ 协同层（身份映射到 typhoon 命名空间） ----------------
    step "⑥ 起协同层 swarm（manager + 6 agent，映射到 typhoon_h480_0..5）"
    if [ ! -f "$COORD_WS_SETUP" ]; then
        msg="协同工作空间未编译（找不到 $COORD_WS_SETUP）"
        [ "$REQUIRE_SWARM" = "1" ] && partial_fail "$msg" || warn "$msg —— 跳过协同层"
        echo
        info "①~⑤ 已在运行。构建协同工作空间后可单独补启动（见排查指南）。"
    else
        ok "协同 agent 消费 /scan，保持单一飞控指令源"

        export ROBOCUP_WS="$REPO_ROOT/coordination"
        export PYTHONPATH="$SWARM_SCRIPTS:$NAV_SRC:${PYTHONPATH:-}"
        # 合规地图（2026-10-07 重构，规则 §2.4/§2.5 无预读随机地图）：
        # 不再从 black_box.txt 生成任何地图。metadata = 静态合规空图
        # （robocup_base.json，障碍恒空），占用栅格由各 swarm_agent 用
        # /scan 运行时 SLAM 构建（slam_grid.py → /swarm/occupancy_grid）。
        export ROBOCUP_METADATA="$FORMAL_METADATA_FILE"
        # 比赛段硬检：metadata 含障碍即疑似预读真值，判失败拒绝起飞
        if python3 "$BB2MD" --verify "$ROBOCUP_METADATA" >>"$LOGDIR/06_map_gen.log" 2>&1; then
            ok "合规空图校验通过（0 障碍，运行时雷达 SLAM 建图）：$ROBOCUP_METADATA"
        else
            partial_fail "metadata 非合规（疑似含预读障碍），看 $LOGDIR/06_map_gen.log —— 拒绝带违规地图起飞"
        fi
        # 协同层不得真值播种：目标只能由 YOLO 链路（经桥）获知（规则 §2.5.11）
        export SEED_TRUTH=0
        export VISUAL_DISPATCH=1       # 仅使用本队 YOLO 新鲜候选提前派机，不播种裁判真值

        # ---- 协同层调度参数（系统内部参数，不改比赛硬规则） ----
        # 比赛硬约束：6 目标 / 600 仿真秒 / 确认15s / 误差1m / 瞬移30s / 不订阅 model_states
        # 以下 9 项全是 swarm_manager 内部调度节奏：备份机上限、确认卡多久加派、备份机多远才接棒、
        # 拍卖周期、热目标留存时间、派遣/跟踪半径余量。改这些不会触犯任何比赛条款。
        # 国家一等奖优化（2026-10-04 复盘）：首见距离余量 + 跟踪半径余量同时放宽，
        #   让"飞机距离目标 25m 已看见但没派"这类浪费窗口消失；备份机接棒半径提到 60m
        #   让边缘瞬移（70/72m）也能在 3~5s 内被接上。
        export BACKUP_MAX=2             # v23（2026-10-09）：3→2 回滚——v22b 实证 t1 单帧误检
                                        # （真值 32m 外）stall=2s 就吸走 4 机（h480_1/3/4/5）20s，
                                        # 搜索瘫痪 310s。少一架 backup 少被误检吸走一架。
        export SEARCH_RESERVE_MAX=2    # 未发现目标仍存在时，至少留 1~2 架机继续搜索
        export SEARCH_HARD_HOME_ZONE=1 # 起飞初期先搜本责任区，避免六机反复回西侧
        export SEARCH_HOME_HARD_S=120.0 # 初期分区展开后允许跨区支援
        export SEARCH_Y_EDGE_DEFER_M=7.0 # 南北外排相机可由内侧一排看见，先推进到中央目标带
        export BACKUP_AFTER=5.0         # v23：2.0→5.0 恢复 2026-10-05 修复（run_match.sh 旧注释
                                        # "默认3"已过时，代码默认 2026-10-05 起就是 5.0，此处 2.0
                                        # 把它覆盖回退了）。stall=2s 是正常确认波动，5s 才是真卡住。
        export BACKUP_MAX_DIST=20.0     # 20:21局：50m外备份占住东侧唯一近机，白/蓝目标无人可派
        export DISPATCH_BACKUP_DIST=60.0 # 默认40 → 60：主派机 >60m 即派接棒机
        export ALLOC_PERIOD=1.0         # 默认1.5 → 1.0：拍卖更密集，6 架少空转
        export HOT_TARGET_TTL=10.0      # 默认8 → 10：热目标留住更久，飞过别处也知道
        export HOT_TARGET_ENABLE=0     # 已有目标由主追/近机接棒处理，搜索机继续找剩余目标
        export DISPATCH_MARGIN=0.0      # 默认2 → 0：首见即派，不再等最近机贴到 18m 内
        export DETECT_RADIUS_MARGIN=3.0 # 默认5 → 3（2026-10-05）：配合 DETECT_RADIUS 20→10，派遣上限回到 13m
        # v24（2026-10-09）：搜索巡航提速 + 覆盖推进 —— 4/6 actor 因搜索格从未
        # 推进到其活动区而全程零观测（5min 只覆盖地图 17%）。恒速 3.0m/s 巡航 +
        # 2m 减速带；出生区锁定 30s 后立即散开；已发现目标派机距离放宽到 60m
        # （原 13m 存在「飞机不派过去就永远够不到 13m」的鸡生蛋死结）。
        export SEARCH_CRUISE_SPEED=3.0
        export SEARCH_DECEL_M=2.0
        export SEARCH_ARRIVE_TOL=2.0  # 与 manager 的 4m 验收半径衔接，避免在 1m 多的误差处悬停
        export SEARCH_SWEEP_DEG=20.0  # 开阔搜索小幅扫视；候选/跟踪/近障时退出
        export SEARCH_SWEEP_PERIOD_S=12.0
        export ORBIT_ALT_MAX=2.6       # 近于 5.5m 保持低视点，避免脚部出框
        export ORBIT_FAR_ALT_MAX=3.5   # 7.5m 外保持较高视线，避免 8m 跟踪时降至 2.2m 失画面
        export ORBIT_FAR_ALT_START=5.5
        export ORBIT_FAR_ALT_FULL=7.5
        export W_FLIGHT=0.16         # 航程梯度足以压过相邻格的分散奖励，减少 30–41m 折返
        export LEASE_STALL_S=20.0    # 靠墙停滞早于 90s 总租约回收
        export LEASE_STALL_PROGRESS_M=2.0
        export EARLY_PHASE_SEC=30.0
        export DISPATCH_FAR_LIMIT=0.0  # 全场可派主追；距离计入成本，接近有时限
        export LEASE_DURATION=22.0      # 默认20 → 22：飞行+扫描+衔接余量
        export DWELL_LOOKAHEAD=1        # 默认1（开）：搜索飞行+到点 dwell 机头持续对准下一引导点（2026-10-05 修单相机视野丢点）

        # 国家一等奖 v2（2026-10-05）——「5 分钟内消灭全部目标」×「无预读随机地图」
        # 0 行代码改动，仅环境变量；不触犯任何比赛硬规则
        # (1) 真正遵守「无预读随机地图」：EDGE / VIS 都会从 ROBOCUP_METADATA 算临楼度
        #     / LOS 预计算, 真比赛拿不到 metadata 必须关掉。
        export EDGE_ENABLE=0
        export VIS_ENABLE=0
        # (2) 租约速度对齐 agent MAX_SPEED=5.0：原 3.0 把租约算长 67%, 接力损耗大
        export LEASE_CRUISE_SPEED=5.0
        # (3) CooperativeTracker GAP_TOL 1.0s→2.5s：YOLO 帧丢 / 摆头时 confirm_since
        #     不被频繁清零 → 15s 凑不满的最关键瓶颈。需 cooperative_tracker.py 支持
        #     env 读（见补丁 1）。
        export GAP_TOL=2.5
        # (4) hot target 吸引飞机上限=2 架：原 60m 半径 4.0 分让 6 架全被吸到一处,
        #     搜索/接力队形坍塌（实测多次）。
        export HOT_TARGET_MAX_UAV=2
        # (5) 全队都在追踪时, 也允许 reopen covered cells（避免『所有格子都 COVERED
        #     + 都在追新目标 → 没有空闲机 → 不重开 → 接力回来全队没格可拍』）。
        export REOPEN_WHEN_TRACKING=1
        # (6) covered 30s 自动 reopen（原 60s 太长, 5 分钟场地覆盖率吃紧）
        export REV_COVER_AGE=30.0
        # (7) 必须官方剩余清单为空才发 MISSION_FINISHED：tracker 偶发空 → 飞机
        #     全停摆, 真比赛必扣分（_finish_cb 会让 agent 退出搜索循环）。
        export REQUIRE_LEFT_FOR_FINISH=1
        # ---- 2026-10-05 比赛规则硬约束：撞墙/越界停止行为修复 ----
        # (8) OOB_RECOVER_SPEED 3.0→5.0：旧值在 6m/s 越界下净速度仍向外漂移 3m/s，
        #     实测 uav_2 在 2s 内 world_xy 漂到 199m（地图 +99m 外），EKF 崩坏后撞墙
        #     强制坠地恢复。改为 5.0 m/s → 净速度 ≥ -1 m/s，1s 收回 1m，5s 收回 5m。
        export OOB_RECOVER_SPEED=5.0
        # (9) OOB_BYPASS_ACC_LIM=1：撞墙越界时跳过水平加速度限幅（0.125 m/s/帧爬升率），
        #     让回收指令当帧生效。否则反向指令被延迟 5-10 帧（0.25-0.5s）才有效果，
        #     期间飞机仍以原速度向外冲 1.5-3m。1=开（默认），0=关（A/B 对照）。
        export OOB_BYPASS_ACC_LIM=1
        # (10) MAP_GUARD_SOFT 2.0→6.0 + MAP_GUARD_MARGIN 1.0→2.0：总软减速带从 3m 扩到
        #      8m。本机巡航 6 m/s 下，0.05s 一帧 0.3m，3m 软带只能覆盖 10 帧 = 0.5s，
        #      飞机在进入软带到硬停之间的刹车距离不足会冲出硬边界。8m 软带提供 1.3s
        #      缓冲，允许 MAX_ACC=2.5 m/s² 把横向速度从 6 m/s 降到 4 m/s 进入硬边界。
        export MAP_GUARD_SOFT=6.0
        export MAP_GUARD_MARGIN=2.0
        # (11) RADAR_WARN_R 4.0→5.5：规则 §2.5(7) 碰撞扣 30/次，雷达预警半径扩到 5.5m
        #      （车体级别障碍），留 1.5m 减速带宽。
        export RADAR_WARN_R=5.5
        # (12) EKF_JUMP_MIN_M 仍 3.0，但 swarm_agent.py 内已把单帧阈值收紧到
        #      MAX_SPEED*dt*1.2（≈ 7.2m/s），并新增 1s 滑动累积窗口兜底慢速漂移。
        export EKF_JUMP_MIN_M=3.0

        # ---- ⑤ 单机 DWA 避障（规则 §2.5(7) 碰撞扣30/次） ----
        # 2026-10-05 国家一等奖修复：DWA 6 个影子 601 次 set_mode 失败后退场，
        # 全场 0 个避障节点生效；swarm_agent.py 已自带 _grid_guard/_map_guard/
        # _radar_guard 三层边界守卫（OOB_RECOVER_SPEED=5.0 / MAP_GUARD_SOFT=6.0
        # / RADAR_WARN_R=5.5），覆盖规则 §2.5(7) 的安全要求 ⇒ 直接关闭启动。
        if [ "${ENABLE_AVOID:-0}" = "1" ]; then
            warn "ENABLE_AVOID=1 已弃用（DWA 启动块 10-03 后 601 次失败），忽略。设 0 关闭提示。"
        else
            ok "DWA 避障已关闭：全队仅靠 swarm_agent 雷达安全层（_grid_guard/_map_guard/_radar_guard）"
        fi

        # 集中式管理器
        UAV_CSV="$(IFS=,; echo "${UAVS[*]}")"
        rosparam set /swarm_manager/uav_ids "$UAV_CSV"
        start_group swarm_manager "$LOGDIR/06_swarm_manager.log" bash -c \
            "cd '$SWARM_SCRIPTS' && python3 -u swarm_manager.py"
        manager_ready(){
            local limit="${1:-5}" pgid
            [ "$limit" -gt 5 ] && limit=5
            pgid="$(group_pgid swarm_manager)"
            swarm_manager_alive "$pgid" || return 1
            timeout "${limit}s" rostopic info /swarm/assignment 2>/dev/null \
                | awk '/Publishers:/{f=1;next}/Subscribers:/{f=0}f' \
                | grep -q swarm_manager
        }
        wait_gate "swarm_manager 任务发布者就绪" 30 manager_ready \
            || partial_fail "swarm_manager 未就绪（metadata？看 $LOGDIR/06_swarm_manager.log）"

        # 6 个 agent：__name 对齐私有参数命名空间（等价 swarm.launch 的逐节点编排）
        i=0
        for u in "${UAVS[@]}"; do
            rosparam set "/swarm_agent_$i/uav_id" "$u"
            rosparam set "/swarm_agent_$i/model_name" "$u"
            # MAVROS 局部系→世界系 offset = 该机起飞世界坐标（对齐 launch）
            rosparam set "/swarm_agent_$i/world_offset_x" "${TAKEOFF_X[$i]}"
            rosparam set "/swarm_agent_$i/world_offset_y" "${TAKEOFF_Y[$i]}"
            start_group "swarm_agent_$i" "$LOGDIR/06_swarm_agent_$i.log" bash -c "
                cd '$SWARM_SCRIPTS' && python3 -u swarm_agent.py __name:=swarm_agent_$i"
            sleep 0.3
            i=$((i+1))
        done

        agents_ready(){
            local limit="${1:-5}" pgid nodes i
            [ "$limit" -gt 5 ] && limit=5
            for i in 0 1 2 3 4 5; do
                pgid="$(group_pgid "swarm_agent_$i")"
                alive_pgid "$pgid" || return 1
            done
            nodes="$(timeout "${limit}s" rosnode list 2>/dev/null)" || return 1
            for i in 0 1 2 3 4 5; do
                printf '%s\n' "$nodes" | grep -qx "/swarm_agent_$i" || return 1
            done
        }
        wait_gate "协同层 6 agent ROS 节点注册" 45 agents_ready \
            || partial_fail "agent 不足 6/6，看 $LOGDIR/06_swarm_agent_*.log"

    fi

    # ---- 旧的"启时统一暂停、最后统一 unpause"已上移到 Gazebo 服务就绪之后
    # （不放在这里的原因：MAVROS 等 SITL heartbeat 但 SITL 在 paused 状态下不会推 sensor —— 启期死锁。）

    # ROS master 两次有界查询一次核验六路裁判链和所有节点订阅。
    # 不订阅尚未发现目标的 ActorInfo 消息，也不读取裁判反馈内容。
    if timeout 12s python3 -u "$GRAPH_AUDIT_SCRIPT" --timeout 8 \
            >"$LOGDIR/07_graph_audit.log" 2>&1; then
        ok "正式 ROS 图审计通过：六路 ActorInfo 与全部节点数据源合规"
    else
        cat "$LOGDIR/07_graph_audit.log" >&2
        partial_fail "正式 ROS 图审计失败；检查 $LOGDIR/07_graph_audit.log"
    fi
    info "实际播报数量请在运行中用 status 查看；启动时目标未发现，不等待六路消息"
    for _component in judge actors yolo_bridge swarm_manager \
            swarm_agent_0 swarm_agent_1 swarm_agent_2 \
            swarm_agent_3 swarm_agent_4 swarm_agent_5; do
        alive_pgid "$(group_pgid "$_component")" \
            || partial_fail "启动验收时 $_component 已退出；检查 $LOGDIR 日志"
    done
    [ "$(count_pattern '[c]ontrol_actor.py')" -ge 6 ] \
        || partial_fail "启动验收时 actor 控制器不足六个；检查 $LOGDIR/02_actors.log"

    # ============================ 完成横幅 ==================================
    echo
    echo "${C_G}==============================================================${C_0}"
    ok "比赛栈编排完成。日志目录: $LOGDIR （也可在 /tmp/robocup_match/latest）"
    echo
    echo "  健康总览:    $0 status"
    echo "  停全场:      $0 stop"
    echo "${C_G}==============================================================${C_0}"

    # ---- 比赛墙钟 hard cap：MATCH_HARD_CAP 秒后自动调 stop_group 全杀 ----
    # 关键：必须放独立 session（setsid -f），主控 Ctrl-C 不会向子 shell 的 sleep 传 SIGINT，
    # 否则 5min 未到时主控若 Ctrl-C，watcher 跟着死 → 6 机残跑 + 拿不到 [FINAL]。
    if [ "${MATCH_HARD_CAP:-0}" -gt 0 ]; then
        setsid -f bash -c '
            trap "" INT TERM HUP
            sleep "$1"
            echo "[$(date +%H:%M:%S)] [hard_cap] 比赛已运行 ${1}s，触发自动 stop"
            # 把全部业务节点 TERM，3s 后再 KILL（与原 stop_group 路径一致）
            REG="$2"
            if [ -s "$REG" ]; then
                while IFS=$'\''\t'\'' read -r name pgid logf; do
                    if [ -n "$pgid" ] && kill -0 -- -"$pgid" 2>/dev/null; then
                        kill -TERM -"$pgid" 2>/dev/null || true
                    fi
                done < "$REG"
                sleep 1
                while IFS=$'\''\t'\'' read -r name pgid logf; do
                    if [ -n "$pgid" ] && kill -0 -- -"$pgid" 2>/dev/null; then
                        kill -KILL -"$pgid" 2>/dev/null || true
                    fi
                done < "$REG"
            fi
            pkill -9 -f '\''[r]adar_avoid.py'\'' 2>/dev/null || true
            pkill -9 -f '\''[p]erception_real.py'\'' 2>/dev/null || true
            pkill -9 -f '\''[m]ultirotor_communication.py'\'' 2>/dev/null || true
            pkill -9 -f '\''[c]ontrol_actor.py'\'' 2>/dev/null || true
            pkill -9 -f '\''[s]warm_agent.py'\'' 2>/dev/null || true
            pkill -9 -f '\''[s]warm_manager.py'\'' 2>/dev/null || true
        ' _ "$MATCH_HARD_CAP" "$REG"
        ok "hard_cap=${MATCH_HARD_CAP}s 已启动独立 session watcher（主控 Ctrl-C 不影响）"
    fi

}

# ============================ 状态查询 ======================================
do_status(){
    [ -s "$REG" ] || { info "没有在跑的比赛（$REG 不存在）"; return; }
    ros_env 2>/dev/null || true

    printf '%-16s %-8s %s\n' "组件" "状态" "日志"
    alive=0; total=0
    while IFS=$'\t' read -r name pgid logf; do
        total=$((total+1))
        if alive_pgid "$pgid"; then s="${C_G}运行${C_0}"; alive=$((alive+1)); else s="${C_R}已退出${C_0}"; fi
        printf '%-16s %-10s %s\n' "$name" "$s" "$logf"
    done < "$REG"
    echo
    info "组件存活 $alive/$total"

    # 六机状态并行订阅；逐机 rostopic echo 最坏要空等 6×5 秒。
    _state_probe="$(timeout 8s python3 -u "$READY_SCRIPT" --kind state --timeout 6 \
        --uavs "${UAVS[@]}" 2>&1 || true)"
    if printf '%s\n' "$_state_probe" | grep -q '^\[ready\] PASS 6/6'; then
        info "MAVROS 已连接: 6/6"
    else
        _state_count="$(printf '%s\n' "$_state_probe" \
            | sed -n 's/^\[ready\] FAIL \([0-6]\)\/6.*/\1/p' | head -1)"
        warn "MAVROS 已连接: ${_state_count:-未知}/6"
        printf '%s\n' "$_state_probe" | grep '^\[ready\] MISSING' || true
    fi
    info "actor 控制器: $(count_pattern '[c]ontrol_actor.py')/6"
    info "感知进程: $(count_pattern '[p]erception_real')"
    info "雷达进程: $(count_pattern '[r]adar_avoid')"
    info "协同 agent: $(count_pattern '[s]warm_agent.py')"
    if timeout 5 rostopic info /swarm/target_states 2>/dev/null | grep -q ' \* '; then
        info "/swarm/target_states 有发布者"
    else
        warn "/swarm/target_states 无发布者（agent 无法检测目标）"
    fi
    # 运行中查看己方桥实际调用官方发布接口的次数；只读己方日志，
    # 不在启动期逐路订阅六个尚无消息的裁判输入话题。
    _bridge_log="$(group_log yolo_bridge)"
    if [ -f "$_bridge_log" ]; then
        _report_counts=""
        for _tag in green blue brown white red1 red2; do
            _count="$(grep -c "\[REPORT_POINT\] tag=$_tag " "$_bridge_log" 2>/dev/null || true)"
            _report_counts="${_report_counts}${_report_counts:+ }${_tag}=${_count:-0}"
        done
        info "本场上报日志累计: $_report_counts"
    fi
}

# ============================ 停止 ==========================================
do_stop(){
    # A failed start in older versions could truncate components.tsv before
    # preflight rejected stale Gazebo/ROS processes.  Clean named residuals
    # even when there are no registered process groups.
    if [ ! -s "$REG" ]; then info "无已登记组件，仍检查并清理残留进程。"; fi
    # 先停业务节点（后起的先停），最后停场景
    stop_group d2o
    for i in 5 4 3 2 1 0; do stop_group "swarm_agent_$i"; done
    stop_group swarm_manager
    stop_group yolo_bridge
    for u in "${UAVS[@]}"; do stop_group "radar_$u"; done
    for u in "${PR_UAV_ARR[@]}"; do stop_group "pr_$u"; done
    stop_group judge
    stop_group actors
    for i in 5 4 3 2 1 0; do stop_group "comm_$i"; done
    sleep 2
    stop_group scene
    sleep 3

    # 残留兜底：每条单独容错
    pkill -9 -f '[r]adar_avoid.py'          2>/dev/null || true
    pkill -9 -f '[p]erception_real.py'      2>/dev/null || true
    pkill -9 -f '[m]ultirotor_communication.py' 2>/dev/null || true
    pkill -9 -f '[c]ontrol_actor.py'        2>/dev/null || true
    # When a stop command is run from an isolated PID namespace, it can
    # remove the shared registry without seeing the host processes.  The
    # host-side stop must still clear every business node by name.
    pkill -9 -f '[s]warm_agent.py'         2>/dev/null || true
    pkill -9 -f '[s]warm_manager.py'       2>/dev/null || true
    pkill -9 -f '[y]olo_target_bridge.py'  2>/dev/null || true
    pkill -9 -f '[d]etection_to_official.py' 2>/dev/null || true
    pkill -9 -f '[s]core_cal.py'           2>/dev/null || true
    # 底层进程默认全清（原仅在 FULL_TEARDOWN=1 才清）：
    #   复盘 2026-10-02 23:29 局：gzserver 启动 2 分钟后被"new node registered with
    #   same name"顶掉 → sim 冻结 → control_actor 调 get_model_state 失败 + Python3
    #   print bug 崩溃 → 6 个恐怖分子全不动 → PX4 EKF 卡死 → 无人机不起飞。
    #   根因是上一局 stop 默认不清 gzserver/gzclient/rosmaster，残留进程在新局 start
    #   时顶掉新 gzserver。改为默认清，FULL_TEARDOWN 标记保留但不再作开关。
    pkill -9 -f '[m]avros_node' 2>/dev/null || true
    pkill -9 -f '[b]in/px4'     2>/dev/null || true
    pkill -9 -x gzserver        2>/dev/null || true
    pkill -9 -x gzclient        2>/dev/null || true
    pkill -9 -f '[r]osmaster'   2>/dev/null || true
    mv "$REG" "$REG.stopped.$STAMP" 2>/dev/null || true
    ok "已停止全场比赛。"
}

# ============================ 入口 ==========================================
# CLI 参数解析：--gui / --no-gui 控制 GAZEBO_GUI；env 仍可覆盖（优先级 CLI > env > 默认）
_subcmd="${1:-start}"
shift 2>/dev/null || true
while [ $# -gt 0 ]; do
    case "$1" in
        --gui)    export GAZEBO_GUI=true ;;
        --no-gui) export GAZEBO_GUI=false ;;
        -h|--help)
            sed -n '5,11p' "$0"; exit 0 ;;
        *) err "未知参数: $1（仅支持 --gui/--no-gui/-h）"; exit 2 ;;
    esac
    shift
done

case "$_subcmd" in
    preflight) preflight ;;
    start)    do_start ;;
    status)   do_status ;;
    stop)     do_stop ;;
    *) err "未知子命令: $_subcmd（支持 preflight | start | status | stop）"; exit 2 ;;
esac
