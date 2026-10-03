#!/bin/bash
# ============================================================================
# RoboCup 多旋翼集群搜索 —— 全场比赛一键编排脚本（整合层，不含任何业务逻辑改动）
#
# 用法:
#   ./run_match.sh preflight   # 只做赛前体检（不启动任何东西，安全）
#   ./run_match.sh start       # 按 ①~⑥ 顺序启动全场（默认）
#   ./run_match.sh status      # 查看各组件与关键话题健康
#   ./run_match.sh stop        # 停止本次比赛的全部组件
#
# 可选环境变量（均有默认值）:
#   GAZEBO_GUI=false            Gazebo 是否开图形界面
#   GATE_TIMEOUT=180            单个就绪检查最长等待秒数
#   PR_UAVS="typhoon_h480_0 typhoon_h480_1 ..."  感知节点服务的飞机（默认 6 机全起）
#   RADAR_WP_DIR=<repo>/radar/waypoints   每机航点目录，文件名 typhoon_h480_N.txt
#   RADAR_SPEED=1.5             雷达巡航速度 m/s（高度固定 5.5，不得改）
#   RADAR_HANDOFF=stop          起协同层前 stop=停雷达避撞双控制 | keep=保留
#   COORD_WS_SETUP=<path>       协同层 catkin devel/setup.bash（未编译则第⑥步跳过）
#   YOLO_BRIDGE=1              1=起 yolo_target_bridge（YOLO target_report → /swarm/target_states）
#   D2O_ENABLE=0               1=起 detection_to_official 桥（开发联调，坐标取真值）
#                              正式比赛须保持 0（规则 §2.5.11 监控订阅）
#   SKIP_RADAR=1 / REQUIRE_SWARM=0 / FORCE=0   0=恢复雷达带飞链路（需 6 份航点）
# ============================================================================

# ---- 严格模式：不使用 set -e，所有失败由检查函数显式处理 ----
set -uo pipefail

# ============================ 路径与配置 ====================================
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOCUP_RUN_ID="${ROBOCUP_RUN_ID:-$(python3 -c 'import uuid; print(uuid.uuid4())')}"
ROS_SETUP="${ROBOCUP_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
WS_SETUP="${ROBOCUP_WS_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4_Firmware}"
XTDRONE_DIR="${XTDRONE_DIR:-$HOME/XTDrone}"
ROBO_DIR="$XTDRONE_DIR/robocup"

PERC_DIR="$REPO_ROOT/perception"
RADAR_DIR="$REPO_ROOT/radar"
ENV_SCRIPT="$PERC_DIR/scripts/env_robocup.sh"
WEIGHTS="$REPO_ROOT/weights/best_yolo11n_bino_v1.pt"
# 障碍/目标真值（官方资源，均在 ~/XTDrone/robocup，不在整合包内）
BLACK_BOX="$ROBO_DIR/black_box.txt"
OBSTACLE_TXT="$ROBO_DIR/obstacle.txt"
OFFICIAL_WORLD="$PX4_ROOT/Tools/sitl_gazebo/worlds/robocup.world"
MAP_GENERATOR="$ROBO_DIR/map_generator.py"
ACTOR_MODEL_DIR="$XTDRONE_DIR/sitl_config/models/actor"
COMM_BRIDGE="$XTDRONE_DIR/communication/multirotor_communication.py"

# 协同工作空间：默认猜测仓库内 devel；可由环境变量指定
COORD_WS_SETUP="${COORD_WS_SETUP:-$REPO_ROOT/coordination/devel/setup.bash}"
RADAR_WP_DIR="${RADAR_WP_DIR:-$RADAR_DIR/waypoints}"
SWARM_SCRIPTS="$REPO_ROOT/coordination/src/robocup_swarm/scripts"
BRIDGE_SCRIPT="$SWARM_SCRIPTS/yolo_target_bridge.py"
# 导航包源码目录：manager/agent 均 import robocup_navigation，必须进 PYTHONPATH
NAV_SRC="$REPO_ROOT/coordination/src/robocup_navigation/src"
# 正确地图元数据（官方 black_box.txt 生成的 base 图，bounds -55..135 / -65..65）。
# 注意：manager/agent 代码内默认值误指向 training_city_full_s7.json（另一张训练图），必须覆盖。
ROBOCUP_METADATA_FILE="${ROBOCUP_METADATA_FILE:-$REPO_ROOT/coordination/src/robocup_training_worlds/worlds/generated/robocup_base.json}"
# 官方每场随机化（规则 §2.4/§2.5）后下发的建筑矩形真值：map_generator.py 输出，
# control_actor.py 靠它放 actor。比赛段用它动态生成 robocup_live.json，否则
# 静态 robocup_base.json 一旦主办方重跑随机化就整体过时（2026-10-01 撞楼复盘：
# 旧快照与随机化后 black_box.txt 的 43 个矩形只重合 10 个）。
BLACK_BOX_FILE="${BLACK_BOX_FILE:-$HOME/XTDrone/robocup/black_box.txt}"
BB2MD="$REPO_ROOT/coordination/src/robocup_training_worlds/scripts/black_box_to_metadata.py"
ROBOCUP_LIVE_METADATA="$REPO_ROOT/coordination/src/robocup_training_worlds/worlds/generated/robocup_live.json"
# YOLO→swarm 桥开关：target_report → /swarm/target_states，替换真值订阅（规则 §2.5.11）
YOLO_BRIDGE="${YOLO_BRIDGE:-1}"
# 起飞世界坐标（对齐 launch/robocup_lidar.launch 六机 x/y），即各机 MAVROS→世界 offset
TAKEOFF_X=(0 3 0 3 0 3)
TAKEOFF_Y=(-3 -3 0 0 3 3)

# 6 架飞机（官方 robocup.launch 的命名空间）
UAVS=(typhoon_h480_0 typhoon_h480_1 typhoon_h480_2
      typhoon_h480_3 typhoon_h480_4 typhoon_h480_5)
# Identity must be identical before perception, bridge and manager start.
export SWARM_UAV_IDS="$(IFS=,; echo "${UAVS[*]}")"
# 感知节点默认 6 机全起：每架带各自相机 YOLO，飞出后才不致盲（32核可承载）。
# 只调试单机时可用 PR_UAVS="typhoon_h480_0" 覆盖。
read -r -a PR_UAV_ARR <<< "${PR_UAVS:-${UAVS[*]}}"
PR_SENSOR_PROFILE="${PR_SENSOR_PROFILE:-stereo}"
case "$PR_SENSOR_PROFILE" in
    cgo3) PR_IMAGE_SUFFIX="cgo3_camera/image_raw"; PR_LINK_SUFFIX="cgo3_camera_link"; PR_SENSOR_OFFSET="0,0,-0.162" ;;
    stereo) PR_IMAGE_SUFFIX="stereo_camera/left/image_raw"; PR_LINK_SUFFIX="base_link"; PR_SENSOR_OFFSET="0.12,0.06,-0.05" ;;
    *) echo "Unsupported PR_SENSOR_PROFILE: $PR_SENSOR_PROFILE" >&2; exit 2 ;;
esac

# 场景 launch：默认与 SKIP_RADAR 联动（默认值在下方 SKIP_RADAR 解析后确定）；
# 可显式覆盖，如 SCENE_LAUNCH="px4 robocup.launch"
SCENE_LAUNCH="${SCENE_LAUNCH:-}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
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
RADAR_SPEED="${RADAR_SPEED:-1.5}"
RADAR_ALT="5.5"
RADAR_HANDOFF="${RADAR_HANDOFF:-stop}"
# 1=由 swarm_agent 消费 /scan 并在唯一速度控制器内做雷达安全层；
# 0=启动旧版 radar_avoid 独立位置控制器（仅用于单独雷达实验）。
# 默认跳过雷达：协同层（YOLO 桥 + swarm）为主链路，agent 自主起飞
SKIP_RADAR="${SKIP_RADAR:-1}"
if [ "$SKIP_RADAR" = "1" ]; then
    RADAR_GUARD="${RADAR_GUARD:-0}"
else
    RADAR_GUARD="${RADAR_GUARD:-1}"
fi
export RADAR_GUARD
# 场景 launch 默认值（按 SKIP_RADAR 联动）：
#  SKIP_RADAR=1 → robocup_nolidar.launch（无 gpu_ray）
#    根因：robocup.world 自带 6 个挂 ActorCollisionsPlugin 的 actor（共约 126 个随骨骼动画
#    移动的碰撞体）。飞机上的 gpu_ray 在 gzserver 中初始化 OGRE 离屏渲染、且每帧遍历 actor
#    骨骼，与物理线程的 actor 碰撞位姿同步发生数据竞争，损坏 ODE 空间(dxSpace)结构 →
#    libgazebo_ode 段错误。时序敏感（spawn 插入时或 sim 16~30s）；gdb ptrace 会改变竞争
#    时序，表现为 gzserver 卡死而非崩溃。去掉 gpu_ray 后不再初始化渲染引擎，竞争消失。
#  SKIP_RADAR=0 → robocup_lidar.launch（gpu_ray 雷达链路，需 6 份航点）
if [ -z "$SCENE_LAUNCH" ]; then
    if [ "$SKIP_RADAR" = "1" ]; then
        SCENE_LAUNCH="$REPO_ROOT/launch/robocup_nolidar.launch"
    else
        SCENE_LAUNCH="$REPO_ROOT/launch/robocup_lidar.launch"
    fi
fi
REQUIRE_SWARM="${REQUIRE_SWARM:-0}"
# detection_to_official 桥（/swarm/detection → 官方 /actor_<color>_info）。
# 该节点坐标取自 /swarm/target_states，属真值链路：默认关闭。
# 正式比赛规则 §2.5.11 监控节点订阅；感知已有 YOLO 直发官方话题，无需此桥。
# 仅在旧开发链路联调时显式 D2O_ENABLE=1。
D2O_ENABLE="${D2O_ENABLE:-0}"
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
RUN_DIR="/tmp/robocup_match"
REG="$RUN_DIR/components.tsv"
STAMP="$(date '+%Y%m%d_%H%M%S')"
LOGDIR="$RUN_DIR/logs_$STAMP"

# ============================ 输出辅助 ======================================
if [ -t 1 ]; then C_G=$'\033[32m'; C_Y=$'\033[33m'; C_R=$'\033[31m'; C_B=$'\033[36m'; C_0=$'\033[0m'
else C_G=""; C_Y=""; C_R=""; C_B=""; C_0=""; fi
info(){ echo "${C_B}[*]${C_0} $*"; }
ok(){   echo "${C_G}[+]${C_0} $*"; }
warn(){ echo "${C_Y}[!]${C_0} $*"; }
err(){  echo "${C_R}[x]${C_0} $*" >&2; }
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
# 判断话题的发布者中是否有节点名匹配片段（身份卡关，不只看有没有发布者）
topic_publisher_match(){  # topic_publisher_match <topic> <节点名片段>
    timeout 8 rostopic info "$1" 2>/dev/null \
      | awk '/Publishers:/{f=1;next}/Subscribers:/{f=0}f' \
      | grep -q "$2"
}

# ============================ 赛前体检 ======================================
preflight(){
    mkdir -p "$RUN_DIR"
    local hard_miss=0
    echo "${C_B}================ RoboCup 赛前体检 ================${C_0}"

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
    check_hard "PX4 场景 robocup.launch" "[ -f '$PX4_ROOT/launch/robocup.launch' ]"
    check_hard "官方地图 robocup.world"  "[ -f '$OFFICIAL_WORLD' ]"
    check_hard "XTDrone 控制脚本"        "[ -f '$ROBO_DIR/control_actors.sh' ]"
    check_hard "官方裁判 score_cal.py"   "[ -f '$ROBO_DIR/score_cal.py' ]"
    check_hard "官方 control_actor.py"   "[ -f '$ROBO_DIR/control_actor.py' ]"
    check_hard "障碍真值 black_box.txt"  "[ -f '$ROBO_DIR/black_box.txt' ]"
    check_hard "障碍真值 obstacle.txt"   "[ -f '$OBSTACLE_TXT' ]"
    check_hard "actor 模型目录"          "[ -d '$ACTOR_MODEL_DIR' ]"
    check_soft "地图生成器 map_generator.py（重新出图时才需要）" "[ -f '$MAP_GENERATOR' ]"
    check_hard "YOLO 权重 v1（双目）"    "[ -f '$WEIGHTS' ]"
    check_hard "感知主节点"              "[ -f '$PERC_DIR/perception_real.py' ]"
    check_hard "雷达避障主程序"          "[ -f '$RADAR_DIR/radar_avoid.py' ]"
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
    case ":${GAZEBO_PLUGIN_PATH:-}:" in
        *":$PX4_ROOT/build/px4_sitl_default/build_gazebo:"*) ok "GAZEBO_PLUGIN_PATH 含 PX4 build_gazebo（SITL 模型插件）";;
        *) err "GAZEBO_PLUGIN_PATH 缺少 $PX4_ROOT/build/px4_sitl_default/build_gazebo（spawn 飞机即 gzserver 255 崩溃）"
           hard_miss=$((hard_miss+1));;
    esac

    check_soft "XTDrone 通信桥（actor 躲避反应依赖它）" "[ -f '$COMM_BRIDGE' ]"
    check_soft "协同层工作空间（robocup_swarm 消息）"   "[ -f '$COORD_WS_SETUP' ]"
    check_soft "detection_to_official 桥脚本（D2O_ENABLE）" \
        "[ -f '$REPO_ROOT/coordination/src/robocup_swarm/scripts/detection_to_official.py' ]"
    check_soft "YOLO→swarm 桥 yolo_target_bridge.py（YOLO_BRIDGE，协同层合法数据源）" \
        "[ -f '$BRIDGE_SCRIPT' ]"
    check_hard "正确地图元数据 robocup_base.json（官方 base 图）" \
        "[ -f '$ROBOCUP_METADATA_FILE' ]"
    if python3 -c "import pyquaternion" >/dev/null 2>&1; then
        ok "pyquaternion 已安装（multirotor_communication 依赖）"
    else
        err "pyquaternion 未安装：执行 python3 -m pip install --user pyquaternion"
        hard_miss=$((hard_miss+1))
    fi

    # 实际要使用的场景 launch 必须存在（默认值已按 SKIP_RADAR 选定）
    check_hard "场景 launch（$SCENE_LAUNCH）" "[ -f '$SCENE_LAUNCH' ]"

    # 雷达模型（gpu_ray）：仅 SKIP_RADAR=0 时加载、需要卡关两处副本一致；
    # SKIP_RADAR=1 不加载 gpu_ray（规避 actor 碰撞竞争段错误），只做软提示。
    LIDAR_PX4="$PX4_ROOT/Tools/sitl_gazebo/models/typhoon_h480_lidar/typhoon_h480_lidar.sdf"
    LIDAR_XTD="$XTDRONE_DIR/sitl_config/models/typhoon_h480_lidar/typhoon_h480_lidar.sdf"
    if [ "$SKIP_RADAR" = "1" ]; then
        info "SKIP_RADAR=1：不加载 gpu_ray（默认协同链路），跳过雷达模型卡关"
        check_soft "雷达模型文件（SKIP_RADAR=0 时才需要）" "[ -f '$LIDAR_PX4' ] && [ -f '$LIDAR_XTD' ]"
    else
        check_hard "雷达模型（PX4 侧 typhoon_h480_lidar）"   "[ -f '$LIDAR_PX4' ]"
        check_hard "雷达模型（XTDrone 侧 typhoon_h480_lidar）" "[ -f '$LIDAR_XTD' ]"
        if [ -f "$LIDAR_PX4" ] && [ -f "$LIDAR_XTD" ]; then
            _a="$(md5sum "$LIDAR_PX4" | cut -d' ' -f1)"
            _b="$(md5sum "$LIDAR_XTD" | cut -d' ' -f1)"
            if [ "$_a" = "$_b" ]; then ok "雷达模型两处 md5 一致 ($_a)"
            else err "雷达模型两处 md5 不一致（重新同步）"; hard_miss=$((hard_miss+1)); fi
        fi
    fi

    # 航点文件清点
    local nwp=0
    for u in "${UAVS[@]}"; do [ -f "$RADAR_WP_DIR/$u.txt" ] && nwp=$((nwp+1)); done
    if [ "$nwp" -gt 0 ]; then ok "雷达航点文件 $nwp/6（$RADAR_WP_DIR）"
    elif [ "$SKIP_RADAR" = "1" ]; then
        info "雷达航点文件 0/6 —— 默认 SKIP_RADAR=1，不影响（协同链路自主起飞）"
    else
        warn "雷达航点文件 0/6 —— 第⑤步将无法启动（每行 'x y' 世界坐标）"
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
    if [ "$SKIP_RADAR" = "1" ]; then
        info "A. 默认 SKIP_RADAR=1：不做 /scan 卡关，协同层自主起飞"
    else
        warn "A. /typhoon_h480_N/scan：雷达模式下场景启动后逐机检查必须有帧"
        echo "      （话题存在不算数；hz 若为 0，检查模型是否被 Gazebo 正常加载）。"
    fi
    warn "B. /swarm/target_states：第④步由 yolo_target_bridge 从 YOLO 检测喂入"
    echo "      （启动后校验发布者身份；第⑥步后自动做节点订阅合规审计）。"

    echo
    if [ "$hard_miss" -eq 0 ]; then
        ok "硬依赖全部就位。"; return 0
    else
        err "硬依赖缺失 $hard_miss 项，补齐后再 start。"; return 2
    fi
}

# ============================ 启动全场 ======================================
do_start(){
    mkdir -p "$RUN_DIR" "$LOGDIR"
    ln -sfn "$LOGDIR" "$RUN_DIR/latest"

    if [ -s "$REG" ] && [ "$FORCE" != "1" ]; then
        spgid="$(awk -F'\t' '$1=="scene"{print $2; exit}' "$REG")"
        if [ -n "$spgid" ] && kill -0 -- "-$spgid" 2>/dev/null; then
            err "已有一场比赛在跑（$REG）。先 ./run_match.sh stop，或 FORCE=1 强制。"; exit 1
        fi
    fi
    : > "$REG"

    preflight >/tmp/robocup_match/preflight.out 2>&1 || { cat /tmp/robocup_match/preflight.out; exit 2; }
    cat /tmp/robocup_match/preflight.out | grep -E '^\[!\]|^----|无法' || true
    ros_env

    # 启动中途失败：清理已起组件
    partial_fail(){
        err "$*"
        err "启动中断，开始清理已起组件 ..."
        while IFS=$'\t' read -r name pgid _log; do stop_group "$name"; done < "$REG"
        exit 1
    }

    # ---------------- ① 场景：Gazebo + 6×PX4 + MAVROS ----------------
    step "① 起场景 $SCENE_LAUNCH (gui=$GAZEBO_GUI paused=$START_PAUSED)"
    if [ -f "$SCENE_LAUNCH" ]; then
        scene_cmd=(roslaunch "$SCENE_LAUNCH" "gui:=$GAZEBO_GUI" "paused:=$START_PAUSED")
    else
        warn "场景 launch 文件不存在（$SCENE_LAUNCH），回退官方 px4 robocup.launch（无雷达）"
        scene_cmd=(roslaunch px4 robocup.launch "gui:=$GAZEBO_GUI" "paused:=$START_PAUSED")
    fi
    start_group scene "$LOGDIR/01_scene.log" "${scene_cmd[@]}"
    wait_gate "Gazebo 服务就绪" "$GATE_TIMEOUT" has_service '/gazebo/get_model_state' \
        || partial_fail "场景没起来，看 $LOGDIR/01_scene.log"

    # ---- 若启用了 START_PAUSED=1：在等 MAVROS 之前先 unpause，否则 SITL 的 simulator start 会卡死 ----
    if [ "$START_PAUSED" = "1" ]; then
        step "Gazebo 服务就绪后统一 unpause（避免 SITL simulator start 卡死）"
        ucommon=0; out=""
        for n in $(seq 1 12); do
            out="$(rosservice call /gazebo/unpause_physics 2>&1)" || true
            if echo "$out" | grep -q "success: True"; then
                ucommon=1; break
            fi
            sleep 1
        done
        if [ "$ucommon" = "1" ]; then
            ok "仿真已 unpause，sim 时钟开始推进（RTF≈1）"
        else
            warn "unpause 失败（最后返回：$out）——手动: rosservice call /gazebo/unpause_physics"
        fi
    fi

    for u in "${UAVS[@]}"; do
        wait_gate "$u MAVROS connected" 300 mavros_connected "$u" \
            || partial_fail "$u 飞控未连上（残留模型？需 pkill -9 -x gzserver 后重来）"
        sleep 3  # 每架飞机间隔3秒，避免 Segmentation fault
    done

    # ---------------- ② XTDrone 通信桥 + 6 个恐怖分子 ----------------
    step "② 起 XTDrone 通信桥 + control_actors"
    if [ -f "$COMM_BRIDGE" ]; then
        for i in 0 1 2 3 4 5; do
            start_group "comm_$i" "$LOGDIR/02_comm_$i.log" \
                python3 -u "$COMM_BRIDGE" typhoon_h480 "$i"
            sleep 1
        done
        sleep 3
    else
        warn "通信桥缺失，跳过 —— actor 将只随机游走、不会躲避"
    fi

    start_group actors "$LOGDIR/02_actors.log" bash -c "cd '$ROBO_DIR' && ./control_actors.sh"
    sleep 8
    nactor="$(count_pattern '[c]ontrol_actor.py')"
    if [ "$nactor" -ge 6 ]; then ok "6 个 actor 控制器在跑"
    else partial_fail "actor 控制器只有 $nactor/6，看 $LOGDIR/02_actors.log"; fi

    # ---------------- ③ 裁判计分 ----------------
    step "③ 起裁判 score_cal.py typhoon_h480"
    start_group judge "$LOGDIR/03_judge.log" bash -c \
        "cd '$ROBO_DIR' && python3 -u score_cal.py typhoon_h480"
    sleep 5
    if first_msg /score; then ok "裁判已在发布 /score"
    else partial_fail "裁判没起来，看 $LOGDIR/03_judge.log"; fi

    # ---------------- ④ 感知（CPU 模式） ----------------
    step "④ 起感知 perception_real.py（CUDA_VISIBLE_DEVICES 置空，CPU 推理）"
    for u in "${PR_UAV_ARR[@]}"; do
        start_group "pr_$u" "$LOGDIR/04_perception_$u.log" bash -c \
            'cd "$1" && CUDA_VISIBLE_DEVICES="" PR_UAV="$2" PR_CAM_LINK="$2::$3" PR_CAM_OFF_BL="$4" PR_CAM_TOPIC="/$2/$5" exec "${VISION_PYTHON:-python3}" -u perception_real.py' \
            perception "$PERC_DIR" "$u" "$PR_LINK_SUFFIX" "$PR_SENSOR_OFFSET" "$PR_IMAGE_SUFFIX"
    done

    # ---- ④b YOLO→swarm 桥：合法检测 → /swarm/target_states ----
    if [ "$YOLO_BRIDGE" = "1" ]; then
        start_group yolo_bridge "$LOGDIR/04b_yolo_bridge.log" bash -c \
            "cd '$SWARM_SCRIPTS' && python3 -u yolo_target_bridge.py"
        sleep 4
        if topic_publisher_match /swarm/target_states yolo_target_bridge; then
            ok "yolo_target_bridge 已注册发布 /swarm/target_states"
        else
            partial_fail "yolo_target_bridge 未注册发布，看 $LOGDIR/04b_yolo_bridge.log"
        fi
    else
        warn "YOLO_BRIDGE=0：/swarm/target_states 无合法数据源，协同层将无法工作"
    fi

    # 感知就绪判据：节点存活 + 相机有帧。
    # 注意不能等 /coordination/target_report（检测结果）——那要等飞机起飞并看到
    # 人才会发，而集群 agent 要在本门放行后才启动，形成"没起飞→看不到人→门不放行→
    # 不起飞"的死锁（旧逻辑还会因谓词自带 10s 超时把墙钟等待放大到 ~24 分钟）。
    perc_ready() {
        for u in "${PR_UAV_ARR[@]}"; do
            alive_pgid "$(group_pgid "pr_$u")" || return 1
            # 暂停启动时世界未步进、相机无帧——帧验证推迟到 unpause 之后；
            # 此处只判节点存活，避免 120s 空等误报。
            if [ "$START_PAUSED" != "1" ]; then
                timeout 8 rostopic echo -n1 "/$u/$PR_IMAGE_SUFFIX" \
                    >/dev/null 2>&1 || return 1
            fi
        done
    }
    if wait_gate "感知节点存活（暂停态，帧待 unpause）" 120 perc_ready; then
        ok "感知节点全部在跑（相机帧与检测结果待 unpause 后产出）"
    else
        warn "感知未全部就绪——查 $LOGDIR/04_perception_*.log"
        for u in "${PR_UAV_ARR[@]}"; do
            alive_pgid "$(group_pgid "pr_$u")" \
                || partial_fail "感知节点 $u 已退出"
        done
    fi
    n_extra_yolo="$(count_pattern 'yolo_v11')"
    [ "$n_extra_yolo" = "0" ] && ok "无多余 YOLO 节点抢 CPU" \
        || warn "发现独立 yolo_v11 节点，会抢 CPU 拖垮发布频率"

    # ---------------- ⑤ 雷达传感器 / 避障 ----------------
    step "⑤ 验证雷达传感器（复用运行中的场景，不重启世界）"
    RADAR_UAVS=()
    if [ "$SKIP_RADAR" = "1" ]; then
        warn "SKIP_RADAR=1，跳过雷达避障"
    else
        for u in "${UAVS[@]}"; do
            wp="$RADAR_WP_DIR/$u.txt"
            [ -f "$wp" ] || { warn "$u 无航点文件 $wp，跳过该机雷达"; continue; }
            # 运行时硬卡关：雷达话题必须真有帧（话题存在不算数）。
            # 暂停启动时世界未步进、/scan 无帧——帧验证推迟到 unpause 后
            # 由 agent 雷达安全层实际消费保证，此处按航点文件先登记。
            if [ "$START_PAUSED" = "1" ]; then
                RADAR_UAVS+=("$u")
            elif wait_gate "$u 雷达有帧 (/scan)" 30 first_msg "/$u/scan"; then
                RADAR_UAVS+=("$u")
            else
                echo "${C_R}========================================================${C_0}"
                err "$u 没有 /scan —— 飞机上没有雷达传感器，避障必然静默失效。"
                err "处置（雷达组 RADAR_INTEGRATION.md §4）："
                echo "  1) 取得 typhoon_h480_lidar 模型并放到两处且内容一致："
                echo "       $PX4_ROOT/Tools/sitl_gazebo/models/typhoon_h480_lidar/"
                echo "       $XTDRONE_DIR/sitl_config/models/typhoon_h480_lidar/"
                echo "  2) 场景需用 sdf=typhoon_h480_lidar 起机（或将 sensor 并入 typhoon_h480.sdf）"
                echo "  3) 重新 start；本场已起的①~④仍在运行不受影响"
                echo "${C_R}========================================================${C_0}"
                partial_fail "雷达传感器缺失，停止编排（未启动协同层）"
            fi
        done

        [ ${#RADAR_UAVS[@]} -eq 0 ] && partial_fail "没有任何可用的雷达航点/传感器"

        if [ "$RADAR_GUARD" = "1" ]; then
            ok "6 路 /scan 已验证；由 swarm_agent 内置雷达安全层消费，不启动第二个控制器"
        else
            for u in "${RADAR_UAVS[@]}"; do
                wp="$RADAR_WP_DIR/$u.txt"
                start_group "radar_$u" "$LOGDIR/05_radar_$u.log" bash -c "
                    cd '$RADAR_DIR' && python3 -u radar_avoid.py \
                        --uav '$u' --ns '/$u' --scan-topic '/$u/scan' \
                        --wp '$wp' --black-box '$BLACK_BOX' \
                        --obstacle-txt '$OBSTACLE_TXT' \
                        --alt $RADAR_ALT --speed $RADAR_SPEED --verbose"
            done
            # 检查节点自身就绪日志
            radar_ready=1
            for u in "${RADAR_UAVS[@]}"; do
                lf="$(group_log "radar_$u")"
                t=0
                until grep -q '\[radar\] 就绪' "$lf" 2>/dev/null; do
                    sleep 2; t=$((t+2))
                    rpgid="$(group_pgid "radar_$u")"
                    if ! alive_pgid "$rpgid"; then
                        partial_fail "radar_$u 退出，看 $lf"
                    fi
                    [ $t -ge 120 ] && { warn "radar_$u 120s 未见就绪日志"; radar_ready=0; break; }
                done
            done
            [ "$radar_ready" = "1" ] && ok "独立雷达控制器全部就绪（RADAR_GUARD=0 实验模式）"
        fi
    fi

    # ---------------- ⑥ 协同层（身份映射到 typhoon 命名空间） ----------------
    step "⑥ 起协同层 swarm（manager + 6 agent，映射到 typhoon_h480_0..5）"
    if [ ! -f "$COORD_WS_SETUP" ]; then
        msg="协同工作空间未编译（找不到 $COORD_WS_SETUP）"
        [ "$REQUIRE_SWARM" = "1" ] && partial_fail "$msg" || warn "$msg —— 跳过协同层"
        echo
        info "①~⑤ 已在运行。构建协同工作空间后可单独补启动（见排查指南）。"
    else
        # 控制律交接：radar_avoid 发 setpoint_position，swarm_agent 发 setpoint_velocity，
        # 同机同时存在必抖动失控 —— 默认先停雷达
        if [ "$RADAR_GUARD" = "1" ] && [ "$SKIP_RADAR" != "1" ]; then
            ok "协同 agent 将直接消费 /scan，保持单一速度控制器"
        elif [ "$RADAR_HANDOFF" = "stop" ] && [ "$SKIP_RADAR" != "1" ]; then
            for u in "${RADAR_UAVS[@]:-}"; do stop_group "radar_$u"; done
            ok "雷达已交接控制权（注意 setpoint 流会中断数秒）"
        else
            warn "RADAR_HANDOFF=keep：雷达与协同同时控制同一批飞机，存在抖动风险"
        fi

        export ROBOCUP_WS="$REPO_ROOT/coordination"
        export PYTHONPATH="$SWARM_SCRIPTS:$NAV_SRC:${PYTHONPATH:-}"
        # 动态地图（2026-10-01 撞楼复盘）：官方每场随机化建筑位置，静态
        # robocup_base.json 会过时。这里从本场 black_box.txt 现场生成
        # robocup_live.json——任意随机图都能对上 Gazebo 世界。
        # ROBOCUP_METADATA 已被外部显式指定时尊重之（调试用途）。
        if [ -z "${ROBOCUP_METADATA:-}" ]; then
            check_hard "官方随机化产物 black_box.txt 存在" "[ -f '$BLACK_BOX_FILE' ]"
            if python3 "$BB2MD" --black-box "$BLACK_BOX_FILE" \
                    --template "$ROBOCUP_METADATA_FILE" \
                    --output "$ROBOCUP_LIVE_METADATA" >>"$LOGDIR/06_map_gen.log" 2>&1; then
                ok "动态地图已生成：black_box.txt → robocup_live.json（摘要见 $LOGDIR/06_map_gen.log）"
                export ROBOCUP_METADATA="$ROBOCUP_LIVE_METADATA"
            else
                partial_fail "动态地图生成失败（看 $LOGDIR/06_map_gen.log），回退静态 robocup_base.json——若主办方重跑过随机化，静态图已过时！"
                export ROBOCUP_METADATA="$ROBOCUP_METADATA_FILE"
            fi
        else
            warn "ROBOCUP_METADATA 已外部指定（$ROBOCUP_METADATA），跳过动态生成"
        fi
        # 协同层不得真值播种：目标只能由 YOLO 链路（经桥）获知（规则 §2.5.11）
        export SEED_TRUTH=0

        # 集中式管理器
        UAV_CSV="$(IFS=,; echo "${UAVS[*]}")"
        export SWARM_UAV_IDS="$UAV_CSV"
        rosparam set /swarm_manager/uav_ids "$UAV_CSV"
        start_group swarm_manager "$LOGDIR/06_swarm_manager.log" bash -c \
            "cd '$SWARM_SCRIPTS' && python3 -u swarm_manager.py"
        sleep 5
        pgid="$(group_pgid swarm_manager)"
        swarm_manager_alive "$pgid" || partial_fail "swarm_manager 启动即退出（metadata？看日志）"

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
            sleep 2
            i=$((i+1))
        done

        sleep 15
        dead=0
        for i in 0 1 2 3 4 5; do
            pgid="$(group_pgid "swarm_agent_$i")"
            alive_pgid "$pgid" || { err "swarm_agent_$i 退出"; dead=$((dead+1)); }
        done
        [ "$dead" = 0 ] && ok "协同层 6 agent + manager 全部在跑" \
            || partial_fail "$dead 个 agent 退出，看 $LOGDIR/06_swarm_agent_*.log"

        # ---- 数据源审计：/swarm/target_states 必须由 YOLO 桥发布 ----
        pub_lines="$(rostopic info /swarm/target_states 2>/dev/null \
                     | awk '/Publishers:/{f=1;next}/Subscribers:/{f=0}f')"
        if echo "$pub_lines" | grep -q yolo_target_bridge; then
            ok "/swarm/target_states 由 yolo_target_bridge 发布（YOLO 合法链路）"
        else
            warn "/swarm/target_states 无 yolo_target_bridge（协同检测链路不通）"
        fi
        if echo "$pub_lines" | grep -Eq 'target_sim|official_target'; then
            err "/swarm/target_states 检测到真值发布者（target_sim/official_target）—— 正式比赛违规"
        fi

        # ---- 订阅合规审计：己方节点实际订阅中不得出现 Gazebo 真值话题 ----
        info "订阅审计：yolo_bridge / manager / agent 不得订阅 Gazebo 真值话题"
        audit_fail=0
        while read -r node; do
            [ -z "$node" ] && continue
            if timeout 8 rosnode info "$node" 2>/dev/null \
                 | sed -n '/Subscriptions:/,/Services:/p' \
                 | grep -Eq '/gazebo/(model|link)_states'; then
                err "$node 订阅了 Gazebo 真值话题（model_states/link_states）"
                audit_fail=1
            fi
        done < <(rosnode list 2>/dev/null | grep -E 'yolo_target_bridge|swarm_manager|swarm_agent')
        [ "$audit_fail" = 0 ] && ok "订阅审计通过：无 Gazebo 真值订阅"

        # detection_to_official 桥：/swarm/detection → 官方颜色话题（ActorInfo）
        # 仅在 swarm 层存活时启动；坐标取自真值，属开发联调链路（见 D2O_ENABLE 注释）
        if [ "$D2O_ENABLE" = "1" ]; then
            warn "启动 detection_to_official 桥（开发联调用，坐标取自真值；"
            echo "      正式比赛前须 D2O_ENABLE=0，规则 §2.5.11）"
            start_group d2o "$LOGDIR/06_d2o.log" bash -c \
                "cd '$SWARM_SCRIPTS' && python3 -u detection_to_official.py"
            sleep 4
            pgid="$(group_pgid d2o)"
            if alive_pgid "$pgid"; then
                if grep -q "detection_to_official" "$LOGDIR/06_d2o.log" 2>/dev/null; then
                    ok "detection_to_official 在跑（日志 06_d2o.log；无真值桥时静默无输出）"
                else
                    warn "detection_to_official 进程在但日志未见初始化行，查 06_d2o.log"
                fi
            else
                warn "detection_to_official 启动即退出，查 $LOGDIR/06_d2o.log（不影响其他组件）"
            fi
        else
            info "D2O_ENABLE=0：不启 detection_to_official（感知节点仍直发官方话题）"
        fi
    fi

    # ---- 旧的"启时统一暂停、最后统一 unpause"已上移到 Gazebo 服务就绪之后
    # （不放在这里的原因：MAVROS 等 SITL heartbeat 但 SITL 在 paused 状态下不会推 sensor —— 启期死锁。）

    # ============================ 完成横幅 ==================================
    echo
    echo "${C_G}==============================================================${C_0}"
    ok "比赛栈编排完成。日志目录: $LOGDIR （也可在 /tmp/robocup_match/latest）"
    echo
    echo "  实时看分:    rostopic echo /score"
    echo "  健康总览:    $0 status"
    echo "  停全场:      $0 stop"
    echo "${C_G}==============================================================${C_0}"
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

    # 关键话题
    nfc=0
    for u in "${UAVS[@]}"; do
        timeout 5 rostopic echo -n1 "/$u/mavros/state" 2>/dev/null \
            | grep -q 'connected: True' && nfc=$((nfc+1))
    done
    info "MAVROS 已连接: $nfc/6"
    info "actor 控制器: $(count_pattern '[c]ontrol_actor.py')/6"
    info "感知进程: $(count_pattern '[p]erception_real')"
    info "雷达进程: $(count_pattern '[r]adar_avoid')"
    info "协同 agent: $(count_pattern '[s]warm_agent.py')"
    sc="$(timeout 5 rostopic echo -n1 /score 2>/dev/null | grep -m1 -o '[0-9-]*' || echo '?')"
    info "当前 /score = $sc"
    if timeout 5 rostopic info /swarm/target_states 2>/dev/null | grep -q ' \* '; then
        info "/swarm/target_states 有发布者"
    else
        warn "/swarm/target_states 无发布者（agent 无法检测目标）"
    fi
}

# ============================ 停止 ==========================================
do_stop(){
    if [ ! -s "$REG" ]; then info "没有需要停止的比赛。"; exit 0; fi
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

    # Only registered process groups belong to this run. Never kill by name.
    mv "$REG" "$REG.stopped.$STAMP" 2>/dev/null || true
    ok "已停止全场比赛。"
}

# ============================ 入口 ==========================================
case "${1:-start}" in
    preflight) preflight ;;
    start)    do_start ;;
    status)   do_status ;;
    stop)     do_stop ;;
    *) err "未知子命令: $1（支持 preflight | start | status | stop）"; exit 2 ;;
esac
