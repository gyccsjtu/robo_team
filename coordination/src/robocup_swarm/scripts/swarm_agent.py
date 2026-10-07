#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集群搜索单机节点（每架无人机跑一个实例，自建实现）。

当前实际接入：
  1. 从逐机MAVROS读取自身位姿/实测速率，以显式出生偏移转换到地图ENU；
     只有显式开发标定模式才读取一次自身Gazebo模型位姿；
  2. 发布 /swarm/uav_status（UavStatus 自建消息）给集中式管理器；
  3. 复验schema2任务授权和schema1路线授权，在线雷达地图规划/最终保护，
     唯一发布setpoint_raw/local；旧SearchAssignment只是内部意图表示。

坐标系说明（关键，避免多机坐标系踩坑）：
  世界/地图系与 gazebo 世界都是 ENU（x 东 y 北 z 上），各机本地 ENU 系只是
  原点不同、轴向平行。因此「世界坐标差」算出的速度向量可直接作为
  raw/local速度字段下发；停止XY使用逐机局部位置环保持。

只依赖标准库 + rospy + 标准消息，无 ROS 自定义依赖之外的第三方库。
"""

import os
import re
import threading
import time
import rospy
import math
import traceback
import json
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State, PositionTarget
from position_brake import PositionBrake
from bounded_escape import RestEvidence
from pose_quality import PoseQuality
from pose_rate import PoseRate, motion_evidence
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from std_msgs.msg import String, Float32

from robocup_swarm.msg import UavStatus, SearchAssignment, TargetState, TargetDetection
from robocup_navigation.astar import load_metadata, GridMap, plan
from swarm_task import LineOfSight, DETECT_RADIUS, CoverageGrid, GRID_SIZE_M
from csv_logger import logger
from radar_velocity_guard import guard_velocity
from collections import deque
from task_authority import TaskGate
from visual_observation import VisualEvidence, TAG_TO_TID
from target_motion import TargetMotion, current_target_point, handoff_reacquisition_point
from route_reservation import RouteGate, exclude_peers, valid_points
from route_endpoint import connect_exact_goal
from orbit_geometry import orbit_goal
from white_reacquisition import WhiteReacquisition
from tracking_navigation import TrackingNavigation, StoppedObservation
from companion_tracking import companion_guidance
from publisher_authority import PublisherAuthority
from fcu_configuration import configure as configure_fcu_parameters
from fleet_motion_guard import MotionCache, protect as protect_fleet_motion
from radar_observed_map import ObservedMap
from nav_msgs.msg import OccupancyGrid
from online_radar_planner import OnlinePlanner, fixture_seed
from search_completion import parse_actor_list
from red_observations import actor_slot_remaining
from search_observation import FrameCache, ObservationLedger, SearchSweep, visible_samples, proven_points, MIN_FRAMES

# 覆盖栅格参数（与 manager 一致）
MAP_X_MIN, MAP_X_MAX = -100.0, 100.0
MAP_Y_MIN, MAP_Y_MAX = -50.0, 50.0

# ============================ 参数 ============================
# 工作空间根目录：可用环境变量 ROBOCUP_WS 覆盖（云端部署/换用户时无需改代码）
WS_ROOT = os.environ.get("ROBOCUP_WS", "/home/ros/team_ws/robocup")
SEARCH_ALTITUDE = 4.5     # 搜索高度 m（最高限制）

# === 官方裁判合规护栏（score_cal.py: z > 6.0 -> score=0 并立即终止任务）===
# 2026-10-01 修复坠毁循环：原 ALT_PANIC=5.0 / ALT_EMERG_CEIL=5.0 太接近最高
# 巡航高度 4.6m，正常 0.5~1m 超调就触发 EMERG vz=-3.0 暴力下降 → 坠地 →
# 恢复爬升 → 再超调的死循环（agent_2 实测 5.28→-5.28→2.74→5.28 循环）。
# 改为递进式层级：4.5(目标上限) → 5.5(硬顶-1.0) → 5.7(快降-1.5) → 5.9(紧急-2.0) → 6.0(官方红线)。
ALT_HARD_CEIL     = float(os.environ.get('ALT_HARD_CEIL', '5.5'))
ALT_PANIC         = float(os.environ.get('ALT_PANIC', '5.7'))       # 强制快降阈值 m
ALT_PANIC_DESCENT = float(os.environ.get('ALT_PANIC_DESCENT', '-1.5'))  # 快降速度 m/s
ALT_PANIC_HSCALE  = float(os.environ.get('ALT_PANIC_HSCALE', '0.3'))    # 此时水平速度系数
MAX_ACC           = float(os.environ.get('MAX_ACC', '2.5'))         # 水平加速度限幅 m/s^2
ALT_HARD_DESCENT  = -1.0   # 强制下降速度 m/s（原 -0.6 下降太慢）
# === 6m 红线二次保险 ===
ALT_EMERG_CEIL     = float(os.environ.get('ALT_EMERG_CEIL', '5.9'))      # 二次保险阈值 m（距 6m 留 0.1m）
ALT_EMERG_DESCENT  = float(os.environ.get('ALT_EMERG_DESCENT', '-2.0'))  # 强制快降 m/s（原 -3.0 太暴导致坠地）
ALT_EMERG_HSCALE   = float(os.environ.get('ALT_EMERG_HSCALE', '0.35'))   # 水平速度系数
ALT_TARGET_CAP     = float(os.environ.get('ALT_TARGET_CAP', '4.5'))      # 目标高度上限（巡航尖峰的源头）

# 2026-09-29：护栏常量一致性自检。历史上 ALT_HARD_CEIL(4.2) < ALT_TARGET_CAP(4.5)
# 静默存在了很久，只在极限环出现后才被反推出来。这里启动即告警。
def _check_alt_consistency():
    if ALT_HARD_CEIL < ALT_TARGET_CAP:
        rospy.logerr('[ALT_CFG] 配置漂移：ALT_HARD_CEIL(%.2f) < ALT_TARGET_CAP(%.2f) '
                     '-> 目标高于硬顶，将出现爬升/压回极限环！'
                     % (ALT_HARD_CEIL, ALT_TARGET_CAP))
    if ALT_EMERG_CEIL <= ALT_HARD_CEIL:
        rospy.logwarn('[ALT_CFG] ALT_EMERG_CEIL(%.2f) <= ALT_HARD_CEIL(%.2f)：'
                      '二次保险先于硬顶触发，等价于把硬顶提到 %.2f'
                      % (ALT_EMERG_CEIL, ALT_HARD_CEIL, ALT_EMERG_CEIL))
    rospy.loginfo('[ALT_CFG] HARD_CEIL=%.2f EMERG_CEIL=%.2f TARGET_CAP=%.2f PANIC=%.2f'
                  % (ALT_HARD_CEIL, ALT_EMERG_CEIL, ALT_TARGET_CAP, ALT_PANIC))


# === 追踪移动目标时的 A* 重规划节流 ===
TRACK_REPLAN_MOVE = 3.0    # 目标移动超过此距离才重规划 (m)
TRACK_REPLAN_SEC  = 2.0    # 距上次规划超过此时间才重规划 (s)
# A* 失败时不能直飞：当前地图含建筑，直飞会把一次规划失败升级为撞楼。
# 需要恢复旧的实验行为时仍可显式设置 PLAN_FALLBACK=1。
PLAN_FALLBACK = int(os.environ.get("PLAN_FALLBACK", "0"))
MAX_SPEED       = float(os.environ.get('SWARM_MAX_SPEED', '5.0'))
POS_KP          = 0.8     # 位置 P 控制增益
ARRIVE_TOL      = float(os.environ.get('SWARM_ARRIVE_TOL', '0.8'))
PUB_RATE        = 10.0    # 状态发布频率 Hz
CTRL_RATE       = 20.0    # 控制频率 Hz
DETECT_RATE     = 5.0     # 目标检测发布频率 Hz（规则3 几何判定）
# 幽灵目标闸门：/swarm/target_states 停发（桥接 DROP_TIME）超过此时长后，
# 停止对该目标的几何检测上报。与 manager TRUTH_TTL 对齐——否则本字典里的
# 过期坐标会被几何检测持续上报，manager _detection_cb 当新检测反复派机
# （2026-10-01 22:41 局实测：t3 幽灵吸住 4 架 3 分钟，确认进度反复归零）。
TARGET_TTL      = float(os.environ.get("TARGET_TTL", "6.0"))

# 2D 雷达安全层：由 swarm_agent 唯一发布 MAVROS 速度设定点，避免与
# radar_avoid 的 setpoint_position 双控制。雷达只在近障时修正当前速度。
RADAR_GUARD     = int(os.environ.get('RADAR_GUARD', '1'))
RADAR_FRESH_S   = float(os.environ.get('RADAR_FRESH_S', '0.5'))
RADAR_WARN_R    = float(os.environ.get('RADAR_WARN_R', '4.0'))
RADAR_STOP_R    = float(os.environ.get('RADAR_STOP_R', '1.2'))
RADAR_LATENCY_S = float(os.environ.get('RADAR_LATENCY_S', '0.5'))
RADAR_BRAKE_MPS2 = float(os.environ.get('RADAR_BRAKE_MPS2', '0.5'))
MAP_GUARD_MARGIN = float(os.environ.get('MAP_GUARD_MARGIN', '1.0'))
MAP_GUARD_SOFT   = float(os.environ.get('MAP_GUARD_SOFT', '2.0'))   # 硬边界外的软减速带宽 m
# 越界主动回收：旧逻辑越界后 A* 起点在外拒绝规划、栅格守卫把界外当墙、
# 地图守卫只锁朝外分量不主动内拉 → 飞机瘫痪在界外永远回不来（2026-10-01
# 实测 uav_2 飘到 (201,-178)、uav_3 到 (-84,141) 并死锁）。
OOB_RECOVER_SPEED = float(os.environ.get('OOB_RECOVER_SPEED', '3.0'))
OOB_RECOVER_INSET = float(os.environ.get('OOB_RECOVER_INSET', '2.0'))  # 回到界内多少 m
# 无激光（SKIP_RADAR=1）时雷达保护失效，用 A* 栅格在执行层做反应式近障兜底：
# 沿速度方向前瞻，撞障就转向/刹停。防 A* 失败续发、追逃跑演员贴墙/冲边界撞墙。
GRID_GUARD      = int(os.environ.get('GRID_GUARD', '1'))
GRID_GUARD_STEP = float(os.environ.get('GRID_GUARD_STEP', '0.5'))   # 前瞻采样步长 m

# ---- 自适应高度 ----
ALT_OPEN_SPACE   = 5.5     # 开阔区域高度 m
ALT_BUILDING     = 4.5      # 建筑附近高度 m
ALT_TRACKING    = 4.0      # 追踪目标高度 m
ALT_TAKEOFF     = 2.0      # 起飞/降落高度 m
# 2026-10-01 修复「趴地起飞」：解锁后水平速度锁零，先垂直爬到此高度再允许
# 水平机动。实测旧逻辑地面上就发大 vx/vy，速度环大倾角、升力垂直分量不足，
# typhoon 贴地滑行/翻机（uav0 滑出 47m 高度仅 1m，其余全程蹭地）。
CLIMB_DONE_ALT  = float(os.environ.get('CLIMB_DONE_ALT', '2.5'))
ALT_BASE        = float(os.environ.get('ALT_BASE', '2.8'))    # 最低巡航层
ALT_STEP        = float(os.environ.get('ALT_STEP', '0.6'))    # 层间距
ALT_NLAYER      = int(os.environ.get('ALT_NLAYER', '4'))     # 层数
ALT_CEILING     = float(os.environ.get('ALT_CEILING', '4.6')) # 硬顶：官方对 >6m 重罚，这里留 1.4m 余量
VERT_SEP        = float(os.environ.get('VERT_SEP', '0.8'))   # 判定「不在同一层」的竖直间隔
MIN_CRUISE_ALT  = 2.0                                        # 追踪降高后的下限
ALT_P           = float(os.environ.get('ALT_P', '1.0'))      # 高度 P 控制增益（原 0.5 太小，贴近目标时爬升乏力）
ALT_VZ_MIN      = float(os.environ.get('ALT_VZ_MIN', '0.2')) # 高度误差存在时的最小升降速度 m/s
CLIMB_NO_AVOID  = float(os.environ.get('CLIMB_NO_AVOID', '1.0'))
SAFE_3D         = float(os.environ.get('SAFE_3D', '4.0'))
# 2026-09-27 复标：3.0 时实测最近 0.70~0.85m（一次碰撞 −30 分）。
# 控制周期 20Hz、友机位置来自 10Hz 广播，5m/s 下 0.2s 滞后就是 1m ——
# 3m 门限根本来不及反应，抬到 4.0 才有足够的提前量。
SLOWDOWN_DIST   = float(os.environ.get('SLOWDOWN_DIST', '14.0'))

# === 2026-09-29 P1-2：三处高度护栏原来静默覆写 cmd.twist.linear.z ===
# 出事只能靠抓包反推。这里统一打点：同 tag 首次必打，之后每 5s 一条并带累计次数。
_ALT_GUARD_STATE = {}


def _alt_guard_hit(tag, z, ceil, vz, vx, vy):
    import time as _time
    _now = _time.time()
    _n = _ALT_GUARD_STATE.get(tag + '_n', 0) + 1
    _ALT_GUARD_STATE[tag + '_n'] = _n
    if _now - _ALT_GUARD_STATE.get(tag, 0.0) >= 5.0 or _n == 1:
        _ALT_GUARD_STATE[tag] = _now
        try:
            rospy.logwarn('[ALT_GUARD] %s z=%.2f > ceil=%.2f -> vz=%.2f '
                          '(vx=%.2f vy=%.2f) hits=%d',
                          tag, z, ceil, vz, vx, vy, _n)
        except Exception:
            pass
# === 2026-09-29 P0-1/P0-2/P1-3 新增 ===
# ORCA 可解性硬顶：sep_vel 必须严格小于 MAX_SPEED，否则半平面 v·n >= sep_vel
# 在 |v| <= MAX_SPEED 下数学上无解（v·n <= |v|），退化成不避让。
# 0.8 -> sep_vel <= 4.0，全向采样相邻夹角 22.5°，最坏 cos(11.25°)=0.98，
# |v|=5.0 时 v·n 可达 4.9 > 4.0，恒有解。
SEP_CAP_FRAC    = float(os.environ.get('SEP_CAP_FRAC', '0.8'))
# 远场（d3 > SAFE_3D）分离要求的除数。原来是硬编码 2.0 -> sep_vel 仅 0~3.0，
# d3=8m 时只有 1.0 m/s，表现为「只降速不避让」（P0-2 真空带）。
# 1.0 -> 远场 sep_vel = 10-d3（d3=8 时 2.0，翻倍）。设 2.0 可完全回退旧行为做 A/B 对照。
SEP_FAR_DIV     = float(os.environ.get('SEP_FAR_DIV', '1.0'))
# 目标高度与硬顶的安全间隔：目标必须低于硬顶，否则反复触发硬顶下降形成极限环
ALT_HARD_MARGIN = float(os.environ.get('ALT_HARD_MARGIN', '0.2'))
# 坠地检测（P1-3）：不依赖新订阅，用「高度持续低于目标 + 几乎不动」判定
CRASH_DETECT    = int(os.environ.get('CRASH_DETECT', '1'))
CRASH_DROP_M    = float(os.environ.get('CRASH_DROP_M', '1.5'))   # 低于目标多少米算掉高
CRASH_V_EPS     = float(os.environ.get('CRASH_V_EPS', '0.3'))    # 水平速度低于此值算不动
CRASH_HOLD_S    = float(os.environ.get('CRASH_HOLD_S', '3.0'))   # 持续多久才判定
CRASH_RECOVER_VZ = float(os.environ.get('CRASH_RECOVER_VZ', '1.5'))  # 坠机恢复爬升速度 m/s
# 2026-09-27：原来是 FRIEND_SAFE_DIST*3 = 30m。6 架同场几乎恒有友机落在
# 30m 内 → 全程只有 3.0 m/s，搜索吞吐被腰斩。收到 14m，其余时间跑满。
SLOW_SPEED      = float(os.environ.get('SLOW_SPEED', '3.0'))
SEP_GAIN        = float(os.environ.get('SEP_GAIN', '1.0'))     # sep_vel 除数，越小排斥越强
AVOID_ANGLES    = int(os.environ.get('AVOID_ANGLES', '16'))    # ORCA 候选方向数（全向）
DANGER_3D       = float(os.environ.get('DANGER_3D', '3.0'))    # 低于此间距直接纯排斥
# 2026-09-29：2.5 -> 3.0。sep_vel 被可解性削顶后近距排斥变弱，
# 纯排斥兜底线相应上移，覆盖原来「ORCA 无解 + 兜底未触发」的死亡带。
# 三维安全距离。垂直分离本来就该算进「够不够远」里，所以不再二值判断同层与否。
# 起飞爬升期豁免水平避让的高度余量：z < ALT_TAKEOFF + CLIMB_NO_AVOID 时不避让。
# 不豁免的话，起飞区 6 架间距只有 8m，全部落进 ORCA 互斥范围 → 谁也爬不起来。
ALT_NARROW      = 3.5       # 窄通道高度 m
BUILDING_DIST    = 5.0      # 建筑判定距离 m（小于此值视为建筑附近）

# ---- 目标盘旋确认 ----
# 2026-09-29 调优：ORBIT_RADIUS 从 5.0 → 8.0，匹配 actor 的 uav_safety_radius=7.0m
# （control_actor.py:53）—— 半径 < 7m 时 actor 会主动推 UAV，根本稳不住。
# 8m 仍小于DETECT_RADIUS=20m；在线盘旋速度由已提交路线和追踪速度配置控制。
ORBIT_RADIUS    = 8.0     # 盘旋半径 m
ORBIT_PLAN_CHORD = float(os.environ.get('SWARM_ORBIT_PLAN_CHORD_M', '0'))
if not math.isfinite(ORBIT_PLAN_CHORD) or not 0. <= ORBIT_PLAN_CHORD <= 2.*ORBIT_RADIUS:
    raise ValueError('Invalid orbit planning chord')
# Legacy non-online orbit angular rate; not an official UAV speed restriction.
# The competition rule describes 1m/s walking and 2m/s fleeing targets.
# Online tracking follows a committed radar route using the chase cap below.
ORBIT_SPEED     = 0.12     # 盘旋角速度 rad/s（r=8m 时线速度 0.96 m/s）
# Keep the existing environment name for launch compatibility. All fresh target
# guidance may chase at this speed, without waiting for a FLEE classification.
# Search is not slowed by stale or unrelated target coordinates.
FLEE_CHASE_SPEED  = float(os.environ.get('FLEE_CHASE_SPEED', '2.2'))
FLEE_STATE_FRESH  = float(os.environ.get('FLEE_STATE_FRESH', '3.0'))
# ---- 偏航对准（2026-10-01：修「盘旋时目标甩出视场」）----
# 双目是水平朝前安装的（HFOV=90°，±45°），而 agent 历史上只发线速度、不发偏航，
# PX4 保持机头朝向不变。圆形盘旋时目标相对机头的方位连续转 360°，只有约 1/4 时间
# 在视场内 → track 反复断、actor_info 断帧、15s 连续确认永远累不满。
# 追踪/盘旋时用偏航角速度 P 控制让机头始终指向目标。
YAW_KP          = float(os.environ.get("YAW_KP", "2.5"))      # 偏航环增益（1/s）
YAW_RATE_MAX    = float(os.environ.get("YAW_RATE_MAX", "1.5"))# 偏航角速度限幅 rad/s
CONFIRM_TIME    = 15.0     # 连续确认时间才消除（规则5）

# ---- 盘旋放弃 / 防扎堆（2026-09-27：修「飞机被已消除目标占死 571s」）----
TARGET_STALE    = float(os.environ.get("TARGET_STALE", "6.0"))    # 给 bridge/备份机留接力窗口 s
# Guidance may coast between valid 2 Hz camera messages. Evidence acceptance
# stays at 1.0 s in VisualEvidence; coasting never creates an observation.
TRACK_GUIDANCE_TTL = 1.5
ORBIT_GIVEUP    = float(os.environ.get("ORBIT_GIVEUP", "30.0"))   # 满 15s 后官方这么久还没消除 → 放弃 s
GIVEUP_COOLDOWN = float(os.environ.get("GIVEUP_COOLDOWN", "45.0"))# 放弃后这段时间内不再自动盘旋该目标 s
CLAIM_ENABLE    = int(os.environ.get("CLAIM_ENABLE", "1"))        # 防扎堆：别机正在确认的目标不再抢
CLAIM_FRESH     = float(os.environ.get("CLAIM_FRESH", "3.0"))     # 认领消息的新鲜期 s

# ---- 确认失败退避（2026-09-28：修「单机被抖动目标锁死 600s」）----
BACKOFF_ENABLE    = int(os.environ.get("BACKOFF_ENABLE", "1"))          # 0=关闭（A/B 对照）
CONFIRM_RESET_MAX = int(os.environ.get("CONFIRM_RESET_MAX", "3"))       # 官方重置这么多次就退避
BACKOFF_COOLDOWN  = float(os.environ.get("BACKOFF_COOLDOWN", "60.0"))   # 退避时长 s
RESET_DECAY       = float(os.environ.get("RESET_DECAY", "120.0"))       # 距上次重置这么久就清零计数 s

# ---- 友机避碰 ----
FRIEND_SAFE_DIST = 10.0  # 增加到10m    # 友机安全距离 m（小于此值开始排斥）
FRIEND_K        = 2.0  # 增加排斥增益      # 排斥增益

# ---- A* 避障飞行 ----
METADATA_PATH   = os.environ.get(
    "ROBOCUP_METADATA",
    os.path.join(WS_ROOT, "src/robocup_training_worlds/worlds/generated/robocup_base.json"))
INFLATE_M       = 1.5     # A* 障碍膨胀半径 m（覆盖桨尖0.37 + 建筑偏大0.45 + 切角/超调0.68）
LOOKAHEAD       = float(os.environ.get('SWARM_LOOKAHEAD_M', '3.0'))
if not math.isfinite(LOOKAHEAD) or LOOKAHEAD <= 0.:
    raise ValueError('Invalid path lookahead')
STABLE_NEEDED   = 40      # EKF 稳定判定：连续多少次 20Hz 采样高度达标（40 = 2s）
# EKF 局部原点跳变判定阈值 m：单帧 local_xy 位移超过此值（且超过物理速度上限）
# 判定为 EKF 原点重置而非真实运动，触发 offset 重锚定保持 world_xy 连续。
EKF_JUMP_MIN_M  = float(os.environ.get('EKF_JUMP_MIN_M', '3.0'))


def _param_float(name):
    """读 ROS 参数为 float；未设置/非法返回 None（不抛异常）。"""
    try:
        v = rospy.get_param(name, None)
        return None if v is None else float(v)
    except (TypeError, ValueError, rospy.ROSException):
        return None


def inflate_grid(grid, inflation_m):
    """对障碍物向外膨胀 inflation_m（圆盘结构元素），返回新 GridMap。

    复用团队单机避障的同名工具逻辑：A* 若直接在原始栅格规划会贴墙，
    膨胀后路径与墙面保持安全距离。
    """
    r = int(math.ceil(inflation_m / grid.resolution))
    if r <= 0:
        return grid
    w, h = grid.width, grid.height
    occ = [(ix, iy) for iy in range(h) for ix in range(w)
           if not grid.is_free((ix, iy))]
    cells = bytearray(grid.cells)
    r2 = r * r
    for cx, cy in occ:
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r2:
                    continue
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    cells[ny * w + nx] = 1
    return GridMap(w, h, grid.resolution, grid.origin, bytes(cells), grid.frame_id)


def smooth_path(points, samples_per_segment=6):
    """在线性安全段内加密 A* 路径，不跨越拐角切入障碍物。

    Catmull-Rom 会在两个安全栅格点之间产生曲线外插，可能穿过 A* 已经
    膨胀掉的墙角；这里保留原折线几何，只增加跟踪采样点。
    """
    pts = list(points)
    if len(pts) < 2:
        return pts
    out = [pts[0]]
    n = max(1, int(samples_per_segment))
    for p0, p1 in zip(pts, pts[1:]):
        for j in range(1, n + 1):
            t = j / float(n)
            out.append((p0[0] + (p1[0] - p0[0]) * t,
                        p0[1] + (p1[1] - p0[1]) * t))
    return out


class SwarmAgent(object):
    def __init__(self, uav_id, model_name):
        self.uav_id = uav_id
        self.model_name = model_name
        self.mavros_ns = rospy.get_param('~mavros_namespace', '/%s/mavros' % uav_id).rstrip('/')

        # ---- 高度层分配（基于 ID，6m 以内）----
        # uav_1/2 -> 5.0m, uav_3/4 -> 5.5m, uav_5/6 -> 6.0m（层间距 0.5m）
        # 机号取最后一个下划线段：兼容 uav_1..6 与 typhoon_h480_0..5 两种命名
        uav_num = int(uav_id.split('_')[-1]) if '_' in uav_id else 1
        self.altitude_layer = ALT_BASE + ((uav_num - 1) % ALT_NLAYER) * ALT_STEP
        # 原来是 3 档 0.5m 循环，6 架里 4 架同高；改成 4 档 0.6m，配合下面 0.8m 的同层阈值
        # 保证「同层的才会互相水平避让，不同层的不会误跳过」。

        # ---- 状态 ----
        self.local_xy = None        # (x, y) MAVROS 局部坐标（轻量、高频）
        self.offset = None          # 局部->世界 的恒定平移 (dx, dy)，启动时标定一次
        self._offset_param = None   # 注入的起飞点世界坐标（EKF 重锚定基准）
        self._anchor_done = False   # 起飞锚定完成后启用定位突跳隔离
        self._local_prev_t = 0.0    # 上一帧 local_position 的 ROS 时间（跳变检测）
        self.local_z = None         # 高度（来自 local_position）
        self.yaw = 0.0              # 机体 yaw（ENU 弧度，供雷达 body->world）
        self._orientation_xyzw = (0.,0.,0.,1.)
        self._scan = None            # 最近一帧 2D LaserScan
        self._scan_t = 0.0           # 最近雷达帧的 ROS 时间
        self._scan_window = deque(maxlen=6)
        self._velocity_sample = None  # (vx, vy, sample_s), ENU measured motion
        self._velocity_z = 0.
        self._pose_rate_enabled = os.environ.get('SWARM_POSE_RATE_GUARD', '1') == '1'
        self._current_scan_corridor = os.environ.get('SWARM_CURRENT_SCAN_CORRIDOR','0') == '1'
        self._pose_rate = PoseRate()
        self._pose_sample_s = 0.
        self._pose_quality = PoseQuality(MAX_SPEED, EKF_JUMP_MIN_M)
        self._authority_lock = threading.RLock()
        self._gate = TaskGate(os.environ.get('ROBOCUP_RUN_ID', ''), uav_id)
        self._visual_evidence = VisualEvidence(self._gate.run_id,
            os.environ.get('SWARM_UAV_IDS', 'uav_1,uav_2,uav_3,uav_4,uav_5,uav_6').split(','))
        self._route_gate = RouteGate(self._gate.run_id, uav_id)
        self._route_pending = None
        self._route_pending_s = 0.
        self._escape_enabled = os.environ.get('SWARM_BOUNDED_ESCAPE','0') == '1'
        self._escape_rest = RestEvidence()
        self._escape_ready = False
        self._escape_active = None
        self._escape_pending = None
        self._escape_used = set()
        self._computed_escape = None
        self._orbit_plan_chord_m = ORBIT_PLAN_CHORD
        self._route_offer_id = 0
        self._peer_routes = None
        self._peer_routes_seq = 0
        self._peer_routes_s = None
        self._ack_seq = 0
        self._stopped_since = None
        self.state = State()
        self.assignment = None      # SearchAssignment 当前任务
        self._v124_behavior = os.environ.get('SWARM_BEHAVIOR_BASELINE','') == 'c77063e'
        self._handoff_yaw_reacquire = os.environ.get('SWARM_HANDOFF_REACQUIRE','0') == '1'
        self._handoff_reacquire_window = None
        self._search_enabled = os.environ.get('SWARM_SEARCH_OBSERVATION', '0') == '1'
        self._search_frames = FrameCache(self._gate.run_id, [uav_id])
        self._search_ledger = ObservationLedger()
        self._search_sweep = None
        self._search_remaining = {}
        self._search_granted_s = 0.
        self._search_feedback_seq = 0
        self._search_feedback_s = -1.

        # ---- A* 避障 ----
        self.md, _ = load_metadata(METADATA_PATH)
        self.grid = inflate_grid(GridMap.from_metadata(self.md), INFLATE_M)
        self._online_map = ObservedMap(self.grid.width, self.grid.height, self.grid.resolution, self.grid.origin)
        self._online_map_offset = None
        self._online_map_epoch_s = None
        self._online_map_lock = threading.RLock()
        self._online_planner = None
        self._online_safe_grid = None
        self._online_safe_epoch = None
        self._online_safe_s = None
        self._last_online_request_s = 0.
        if os.environ.get('ONLINE_RADAR_PLANNING', '0') == '1':
            seed = fixture_seed(os.environ['RADAR_START_CLEARANCE_FILE'], os.environ['ROBOCUP_RUN_ID'], uav_id)
            self._online_planner = OnlinePlanner(seed)
            self.grid = GridMap(self.grid.width, self.grid.height, self.grid.resolution,
                                self.grid.origin, bytes([1])*(self.grid.width*self.grid.height), 'map')
        self._map_pub = None
        self.path = []              # 当前全局路径（世界坐标航点列表）
        self.path_target = None     # 当前路径终点（格中心），用于判断是否需重规划
        self._last_plan_t = 0.0

        # ---- 异步规划器（2026-10-01）----
        # 旧实现把全图 A*（实测 27~74ms，栅格 380x260）塞进 20Hz 控制循环，单次
        # 规划就阻塞 1.5~3 个控制周期；追踪任务 5Hz 重发时循环被拖到 20Hz 以下，
        # PX4 判 offboard setpoint 丢失 → failsafe（实测 32 次 "no RC and no offboard"）。
        # 改为后台守护线程：主循环只投递请求、立即继续发速度；规划好后原子提交路径。
        self._plan_lock = threading.Lock()
        self._plan_pending = None    # 最新待规划目标 (x, y)（旧请求被新请求覆盖合并）
        self._plan_ticket = 0
        self._plan_inflight = None
        self._plan_event = threading.Event()
        self._plan_fail_t = 0.0      # 上次规划失败时刻（失败后 2s 冷却，避免刷屏）
        self._takeoff_done = False   # 是否已完成起飞垂直爬升
        self._last_flight_v = None   # 上次下发的水平速度 (vx, vy)，规划未就绪时续命
        self._last_orca_v = (0.0, 0.0)
        self._last_csv_t = 0.0
        self._csv = logger("control_%s" % uav_id, [
            "ros_time", "event", "target_id", "target_x", "target_y",
            "target_z", "uav_x", "uav_y", "uav_z", "distance_m",
            "capture_threshold_m", "is_captured", "orca_vx", "orca_vy",
            "orca_vz", "cmd_vx", "cmd_vy", "cmd_vz", "source"])
        self._look_at = None         # 机头需持续对准的世界点 (x,y)；None=不控偏航

        # ---- 覆盖栅格（与 manager 一致，10m 格）----
        self.cov_grid = CoverageGrid(MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX, GRID_SIZE_M)
        if self._search_enabled:
            b = self.md['bounds']
            self.cov_grid = CoverageGrid(b['x_min'], b['x_max'], b['y_min'], b['y_max'], GRID_SIZE_M)

        # ---- 目标检测（规则3 几何判定：距离 + 视线遮挡）----
        # 用**未膨胀**的原始栅格做 LOS 判定（膨胀是给飞行留裕度的，
        # 判定遮挡要用真实建筑轮廓，否则会把建筑边缘 0.5m 内误判为遮挡）。
        raw_grid = GridMap.from_metadata(self.md)
        self.los = LineOfSight(
            lambda ix, iy: (not raw_grid.in_bounds((ix, iy)))
                           or (not raw_grid.is_free((ix, iy))),
            cell_size=raw_grid.resolution, origin=raw_grid.origin)
        if self._online_planner is not None:
            self.los = LineOfSight(lambda ix, iy: not self.grid.is_free((ix, iy)),
                                   cell_size=self.grid.resolution, origin=self.grid.origin)
        self.targets = {}           # target_id -> (x, y, vx, vy)，来自 /swarm/target_states
        self._target_state = {}    # target_id -> (state, t)，目标运动状态（1=FLEE）
        self._visual_motion = TargetMotion()
        self._detect_log_t = {}     # tid -> 上次 [ALGO] detect 日志时刻（按 target 分别节流）
        self._last_detect_t = 0.0

        # ---- 目标盘旋确认 ----
        self._orbit_target = None    # 当前盘旋目标 ID
        self._last_track_goal = None # 追踪时上次 A* 的目标点（用于节流）
        self._last_track_plan_t = 0.0  # 追踪时上次 A* 规划时刻
        self._plan_fallback_n = 0  # A* 失败退化直飞的次数
        self._orbit_center = None    # 盘旋中心 (x, y)
        self._confirm_start = 0.0   # 连续确认开始时间
        self._tracking_retry_pending = None  # One fresh-evidence retry per new target grant.
        self._last_confirm_t = 0.0   # 上次确认时间
        self._target_to_orbit = None  # 待盘旋目标位置 (x, y)
        self._white_reacquire = WhiteReacquisition()
        navigation_s = float(os.environ.get('SWARM_TRACK_NAVIGATION_S', '0'))
        if not math.isfinite(navigation_s) or navigation_s < 0:
            raise ValueError('Invalid tracking navigation window')
        self._tracking_navigation = TrackingNavigation(navigation_s) if navigation_s > 0 else None
        self._tracking_navigation_phase = None
        self._stopped_observation = StoppedObservation(self.uav_id, navigation_s) if navigation_s > 0 else None
        self._companion_tracking_enabled = os.environ.get('SWARM_COMPANION_TRACKING','0') == '1'
        self._white_reacquire_enabled = os.environ.get('SWARM_WHITE_REACQUIRE', '0') == '1'
        self._white_reacquire_phase = None
        self._t_seen = {}          # tid -> 最后一次收到位置的时刻（判定目标是否已消失）
        self._giveup_until = {}    # tid -> 该时刻前不再自动盘旋（放弃过 / 已消除）
        self._claims = {}          # tid -> (uav_id, 时刻) 别机正在确认的目标
        self._left_seen = False    # 是否已收到过有效的 /left_actors
        self._last_claim_t = 0.0   # 认领广播节流
        self._find_t = {}           # actor 下标 -> 官方 /find_actor_N 最近的时间戳
        self._reset_n = {}          # actor 下标 -> 已观测到的官方重置次数
        self._reset_t = {}          # actor 下标 -> 最近一次重置的时刻
        for _i in range(6):
            rospy.Subscriber('/find_actor_%d' % _i, Float32,
                             self._find_cb, callback_args=_i, queue_size=5)

        # ---- 友机避碰 ----
        self._friend_positions = {}  # 其他无人机位置 {uav_id: (x, y, z)}
        fleet = os.environ.get('SWARM_UAV_IDS', 'uav_1,uav_2,uav_3,uav_4,uav_5,uav_6').split(',')
        self._motion_cache = MotionCache(os.environ.get('ROBOCUP_RUN_ID', ''), fleet)
        if self.uav_id not in fleet:
            raise ValueError('Agent missing from SWARM_UAV_IDS')
        self._motion_seq = 0

        # ---- MAVROS 服务 ----
        self.arm_srv = rospy.ServiceProxy(self.mavros_ns + '/cmd/arming', CommandBool)
        self.mode_srv = rospy.ServiceProxy(self.mavros_ns + '/set_mode', SetMode)
        self.param_srv = rospy.ServiceProxy(self.mavros_ns + '/param/set', ParamSet)

        # ---- 订阅 ----
        # 注意：**不订阅 /gazebo/model_states**。它是 250Hz × 687 模型 × 6.4MB/s 的
        # 巨型消息，每个 agent 都要用 Python 反序列化，实测每机吃掉 ~37% CPU
        # （比 PX4 还高），6 机时是灾难。改为只订阅轻量的 MAVROS local_position，
        # 启动时用一次 model_states 标定「局部->世界」的恒定平移即可。
        rospy.Subscriber(self.mavros_ns + '/state', State, self._state_cb)
        # 官方剩余 actor 清单：权威的「谁还在场上」，用来剔掉已被删除的目标
        rospy.Subscriber("/left_actors", String, self._left_actors_cb, queue_size=5)
        # 别机认领广播：防止多架机扎堆确认同一个目标
        rospy.Subscriber("/swarm/orbit_claim", String, self._claim_cb, queue_size=20)
        self._claim_pub = rospy.Publisher("/swarm/orbit_claim", String, queue_size=10)
        rospy.Subscriber(self.mavros_ns + '/local_position/pose', PoseStamped, self._local_cb)
        rospy.Subscriber(self.mavros_ns + '/local_position/velocity_local',
                         TwistStamped, self._velocity_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~scan_topic', '/%s/scan' % uav_id), LaserScan, self._scan_cb,
                 queue_size=1)
        self._authority_ack_pub = rospy.Publisher('/swarm/authority_ack', String, queue_size=10)
        self._route_offer_pub = rospy.Publisher('/swarm/route_offer', String, queue_size=10)
        rospy.Subscriber('/swarm/route_grant', String, self._route_grant_cb, queue_size=100)
        rospy.Subscriber('/swarm/route_reservations', String, self._route_snapshot_cb, queue_size=1)
        rospy.Subscriber('/swarm/authorized_assignment', String, self._authorized_cb, queue_size=100)
        if os.environ.get('SEED_TRUTH', '0') == '1':
            rospy.Subscriber('/swarm/target_states', TargetState, self._target_cb)
        else:
            rospy.Subscriber('/swarm/confirmed_visual_observation', String, self._confirmed_visual_cb, queue_size=100)
        rospy.Subscriber("/swarm/finish", String, self._finish_cb)
        rospy.Subscriber("/swarm/uav_status", UavStatus, self._friend_status_cb)

        # ---- 发布 ----
        self.status_pub = rospy.Publisher("/swarm/uav_status", UavStatus, queue_size=5)
        self._motion_pub = rospy.Publisher('/swarm/motion_state', String, queue_size=10)
        self._map_pub = rospy.Publisher('/' + self.uav_id + '/radar_observed_map', OccupancyGrid, queue_size=1)
        rospy.Subscriber('/swarm/motion_state', String, self._motion_cb, queue_size=100)
        self.detect_pub = rospy.Publisher("/swarm/detection", TargetDetection, queue_size=10)
        self.vel_pub = rospy.Publisher(self.mavros_ns + '/setpoint_raw/local',
                                       PositionTarget, queue_size=5)
        self._position_brake = PositionBrake()
        self._xyz_stop_requested = False
        self._final_stop_reason = 'REQUESTED_STOP'
        self._navigation_log = None
        self._navigation_reason = None
        self._blocked_plan = None
        self._blocked_feedback_seq = 0
        self._navigation_pub = rospy.Publisher('/swarm/navigation_feedback', String, queue_size=5)
        self._search_feedback_pub = rospy.Publisher('/swarm/search_feedback', String, queue_size=20)
        if self._search_enabled:
            rospy.Subscriber('/swarm/processed_camera_frame', String, self._search_frame_cb, queue_size=100)
            rospy.Subscriber('/swarm/search_feedback_ack', String, self._search_feedback_ack_cb, queue_size=100)

        self.ctrl_rate = rospy.Rate(CTRL_RATE)
        self.pub_rate = rospy.Rate(PUB_RATE)
        self._last_status_t = rospy.Time.now()
        self._mission_finished = False  # 任务完成标志
        self._landing = False  # 降落标志

        # 自适应速度状态
        self._last_heading = None  # 上次朝向
        self._last_speed = 0.0    # 上次速度大小

        # 启动后台规划守护线程（只是阻塞等事件，早启动无害）
        self._plan_thread = threading.Thread(target=self._planner_loop, name="astar_worker")
        self._plan_thread.daemon = True
        self._plan_thread.start()

    @property
    def world_xy(self):
        """世界坐标 = 局部坐标 + 标定偏移（偏移未标定好前返回 None）。"""
        if (self.local_xy is None or self.offset is None
                or getattr(self, '_pose_quality', None) is not None
                and self._pose_quality.fault is not None):
            return None
        return (self.local_xy[0] + self.offset[0], self.local_xy[1] + self.offset[1])

    def _calibrate_offset(self):
        """确定局部→世界的恒定平移（世界系 = MAVROS 局部系 + offset）。

        优先级：
          1. 私有参数 ~world_offset_x / ~world_offset_y（合规路径）：
             世界系与各机 MAVROS 局部系轴向平行、仅原点不同；PX4 SITL 各机
             局部系原点即起飞点，故 offset 就是起飞世界坐标，由启动编排
             （run_match.sh）按机号从 launch 注入。
          2. SWARM_CALIB=truth（仅开发期）：用一次 /gazebo/model_states 实测；
             该路径会订阅真值话题，正式比赛禁用（规则 §2.5.11）。
          3. 二者皆无 → 失败退出，绝不带 None offset 起飞。
        """
        ox = _param_float("~world_offset_x")
        oy = _param_float("~world_offset_y")
        if ox is not None and oy is not None:
            self.offset = (ox, oy)
            self._offset_param = (ox, oy)   # 锚定基准：起飞点世界坐标
            rospy.loginfo("[%s] 坐标系标定（参数）：offset=(%.2f, %.2f)",
                          self.uav_id, ox, oy)
            if os.environ.get("SWARM_CALIB") == "truth":
                measured = self._calibrate_with_model_states()
                if measured is not None:
                    rospy.loginfo("[%s] 真值比对：参数(%.2f,%.2f) 实测(%.2f,%.2f) "
                                  "偏差=(%.2f,%.2f)", self.uav_id, ox, oy,
                                  measured[0], measured[1],
                                  measured[0] - ox, measured[1] - oy)
            return True

        if os.environ.get("SWARM_CALIB") == "truth":
            measured = self._calibrate_with_model_states()
            if measured is None:
                return False
            self.offset = measured
            rospy.logwarn("[%s] 未提供 offset 参数，真值标定仅可用于开发期",
                          self.uav_id)
            return True

        rospy.logerr(
            "[%s] 标定失败：未注入 ~world_offset_x/y 参数。正式链路请由启动脚本"
            "按起飞点注入（开发期可 SWARM_CALIB=truth 临时实测）", self.uav_id)
        return False

    def _calibrate_with_model_states(self):
        """用一次 /gazebo/model_states 实测 offset，返回 (dx,dy) 或 None。

        世界系与各机 MAVROS 局部系轴向平行、仅原点不同，故偏移是常量
        （实测两机分别恒定，误差 <0.01m）。只在显式开启时调用。
        """
        try:
            msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=20.0)
        except rospy.ROSException as exc:
            rospy.logerr("[%s] 真值标定失败：取不到 model_states（%s）", self.uav_id, exc)
            return None
        try:
            i = msg.name.index(self.model_name)
        except ValueError:
            rospy.logerr("[%s] 真值标定失败：model_states 里没有 %s", self.uav_id, self.model_name)
            return None
        # 等 local_xy 就绪
        for _ in range(200):
            if self.local_xy is not None:
                break
            self.ctrl_rate.sleep()
        if self.local_xy is None:
            rospy.logerr("[%s] 真值标定失败：local_position 无数据", self.uav_id)
            return None
        p = msg.pose[i].position
        offset = (p.x - self.local_xy[0], p.y - self.local_xy[1])
        rospy.loginfo("[%s] 真值标定完成：offset=(%.2f, %.2f)", self.uav_id,
                      offset[0], offset[1])
        return offset

    # ---------------- 回调 ----------------
    def _state_cb(self, msg):
        self.state = msg

    def _local_cb(self, msg):
        stamp = msg.header.stamp.to_sec()
        new_xy = (msg.pose.position.x, msg.pose.position.y)
        if not self._pose_quality.accept((*new_xy, msg.pose.position.z), stamp,
                                         enforce=self._anchor_done):
            if self._pose_quality.fault:
                rospy.logerr_throttle(2., '[%s] ESTIMATOR_QUARANTINE %s; no reanchor',
                                      self.uav_id, self._pose_quality.fault)
            return
        self._pose_sample_s = stamp
        self.local_z = msg.pose.position.z
        self.local_xy = new_xy
        self._local_prev_t = stamp
        q = msg.pose.orientation
        self._orientation_xyzw = (q.x,q.y,q.z,q.w)
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose_rate.add((*new_xy, self.local_z), stamp, self.offset)

    def _scan_cb(self, msg):
        self._scan = msg
        self._scan_t = msg.header.stamp.to_sec()
        quality = getattr(self, '_pose_quality', None)
        if quality is not None and not quality.usable(rospy.Time.now().to_sec()):
            return
        observed = getattr(self, '_online_map', None)
        if observed is None or self._map_pub is None:
            return
        position = self.world_xy
        local_updates = getattr(self,'_current_scan_corridor',False)
        period = .2 if local_updates else .5
        last_full = getattr(self,'_online_full_scan_s',None)
        full_due = local_updates and (last_full is None or self._scan_t-last_full >= .5
                                      or self._online_map_offset != self.offset)
        if (position is None or self._local_prev_t is None
                or (observed.last_scan_s is not None and self._scan_t-observed.last_scan_s < period
                    and not full_due)):
            return
        now = rospy.Time.now().to_sec()
        with self._online_map_lock:
            observed = self._online_map
            # Reset, feed and freeze one epoch atomically against planner snapshots.
            if self._online_map_offset != self.offset:
                observed = self._online_map = ObservedMap(self.grid.width, self.grid.height, self.grid.resolution, self.grid.origin)
                self._online_map_offset = self.offset
                self._online_map_epoch_s = self._scan_t
                self._online_full_scan_s = None
            last_full = getattr(self,'_online_full_scan_s',None)
            full = not local_updates or last_full is None or self._scan_t-last_full >= .5
            maximum_distance = None
            if not full:
                evidence = self._measured_motion(now)
                if evidence is None:
                    return
                speed = max([MAX_SPEED]+[math.hypot(*v) for v in evidence['velocity_candidates']])
                maximum_distance = speed*.5+speed*speed+1.2+observed.resolution
            if not observed.feed(msg.ranges, position, self.yaw, msg.angle_min, msg.angle_increment,
                                 msg.range_min, msg.range_max, self._scan_t, self._local_prev_t, now,
                                 maximum_distance=maximum_distance):
                return
            self._scan_window.append(dict(sample_s=self._scan_t,position_xy=position,
                pose_s=self._local_prev_t,
                local_z=self.local_z,yaw=self.yaw,orientation_xyzw=self._orientation_xyzw,angle_min=msg.angle_min,
                angle_increment=msg.angle_increment,range_min=msg.range_min,range_max=msg.range_max,
                ranges=[float(v) if math.isfinite(v) else None for v in msg.ranges],
                nonfinite_ranges=[[i, 'nan' if math.isnan(v) else 'positive_infinity' if v > 0 else 'negative_infinity']
                                  for i, v in enumerate(msg.ranges) if not math.isfinite(v)]))
            planner = getattr(self, '_online_planner', None)
            if planner is not None:
                planner.carry_body_proof(observed, position, now, self._online_map_epoch_s)
            if not full:
                return  # Near-field evidence is current; whole-map publication stays decimated.
            self._online_full_scan_s = self._scan_t
            map_epoch, values, version = self._online_map_epoch_s, observed.snapshot(now), observed.version
        output = OccupancyGrid()
        output.header.stamp = msg.header.stamp
        output.header.seq = version
        output.header.frame_id = 'map'
        output.info.width, output.info.height = observed.width, observed.height
        output.info.resolution = observed.resolution
        output.info.map_load_time = rospy.Time.from_sec(map_epoch or self._scan_t)
        output.info.origin.position.x, output.info.origin.position.y = observed.origin
        output.info.origin.orientation.w = 1.
        output.data = values
        self._map_pub.publish(output)

    def _velocity_cb(self, msg):
        self._velocity_sample = (msg.twist.linear.x, msg.twist.linear.y,
                                 msg.header.stamp.to_sec())
        self._velocity_z = msg.twist.linear.z
        self._publish_motion()

    def _motion_cb(self, message):
        try:
            with self._authority_lock:
                self._motion_cache.receive(json.loads(message.data), rospy.Time.now().to_sec())
        except (ValueError, TypeError):
            return

    def _measured_motion(self, now):
        quality = getattr(self, '_pose_quality', None)
        if quality is not None and not quality.usable(now):
            return None
        enabled = getattr(self, '_pose_rate_enabled', False)
        rate = getattr(self, '_pose_rate', None)
        estimate = rate.estimate(now, self.offset) if enabled and rate is not None else None
        return motion_evidence(self._velocity_sample, getattr(self, '_velocity_z', 0.),
                               estimate, now, require_pose=enabled)

    def _publish_motion(self):
        quality = getattr(self, '_pose_quality', None)
        if quality is not None and not quality.usable(rospy.Time.now().to_sec()):
            return
        if (getattr(self, '_motion_pub', None) is None or self.world_xy is None
                or self._velocity_sample is None or self._local_prev_t is None):
            return
        evidence = self._measured_motion(rospy.Time.now().to_sec())
        if evidence is None:
            return
        with self._authority_lock:
            self._motion_seq += 1
            vx, vy = evidence['selected_xy']
            message = dict(schema_version=1, run_id=self._motion_cache.run_id,
                uav_id=self.uav_id, seq=self._motion_seq,
                sample_s=min(evidence['sample_s'], self._local_prev_t), frame='world_enu_xy',
                position_xy=list(self.world_xy), velocity_xy=[vx, vy])
            self._motion_cache.receive(message, rospy.Time.now().to_sec())
            self._motion_pub.publish(String(data=json.dumps(message)))

    def _friend_guard_velocity(self, vx, vy):
        result = protect_fleet_motion((vx, vy), self.uav_id, self._motion_cache,
                                      rospy.Time.now().to_sec(),
                                      separation=float(os.environ.get('SWARM_FLEET_SEPARATION_M', '4.5')))
        if result['reason'] != 'CLEAR':
            self._final_stop_reason = 'FLEET_'+result['reason']
            rospy.logwarn_throttle(2, '[%s] fleet_guard=%s', self.uav_id, result['reason'])
        return result['velocity_xy']

    def _authorized_cb(self, msg):
        try:
            message = json.loads(msg.data)
            with self._authority_lock:
                previous_generation = self._gate.generation
                if not self._gate.receive(message, rospy.Time.now().to_sec()):
                    return
                if self._gate.stopping:
                    if getattr(self,'_escape_enabled',False):
                        self._cancel_escape('AUTHORITY_STOP')
                    if getattr(self, '_white_reacquire', None) is not None:
                        self._white_reacquire.cancel()
                    if getattr(self, '_search_enabled', False):
                        self._pause_search(rospy.Time.now().to_sec())
                    self._tracking_retry_pending = None
                    self._handoff_reacquire_window = None
                    self._route_gate.clear()
                    self._route_pending = None
                    self._last_flight_v = (0., 0.)
                    self.path, self.path_target = [], None
                    return
                if previous_generation != self._gate.generation:
                    if getattr(self,'_escape_enabled',False):
                        self._cancel_escape('GENERATION_CHANGED')
                    if getattr(self, '_white_reacquire', None) is not None:
                        self._white_reacquire.reset(self._gate.generation, rospy.Time.now().to_sec())
                    if getattr(self, '_search_enabled', False):
                        self._pause_search(rospy.Time.now().to_sec())
                    self._route_gate.clear()
                    self._route_pending = None
                    self._stopped_since = None
                    self.path, self.path_target = [], None
                    self._last_flight_v = (0., 0.)
                    self._orbit_target = self._orbit_center = self._target_to_orbit = None
                    self._look_at = None
                    self._tracking_retry_pending = ((self._gate.task['target_id'], self._gate.generation)
                        if self._gate.task['task_type'] == 1 else None)
                    granted_s = rospy.Time.now().to_sec()
                    self._handoff_reacquire_window = ((self._gate.task['target_id'],
                        self._gate.generation, granted_s, granted_s+5.)
                        if self._gate.task['task_type'] == 1 else None)
                    with self._plan_lock:
                        self._plan_ticket += 1
                        self._plan_pending = None
                assignment = SearchAssignment()
                assignment.uav_id = self.uav_id
                for name, value in self._gate.task.items():
                    setattr(assignment, name, value)
                self._assign_cb(assignment)
                if (getattr(self, '_search_enabled', False) and previous_generation != self._gate.generation
                        and self._gate.task['task_type'] == 0):
                    self._search_granted_s = granted_s
                    key = (assignment.cell_ix, assignment.cell_iy)
                    checkpoint = self._search_remaining.pop(key, None)
                    remaining = checkpoint[1] if checkpoint and 0 <= granted_s-checkpoint[0] < 30. else ()
                    if remaining and (self._online_safe_grid is None or self._online_safe_s is None
                            or not 0 <= granted_s-self._online_safe_s <= 1.5
                            or self.world_xy is None or not self._online_planner.connector_clear(
                                self._online_safe_grid, self.world_xy, remaining[0])):
                        remaining = ((assignment.target_x, assignment.target_y),)+tuple(
                            p for p in remaining if p != (assignment.target_x, assignment.target_y))
                    self._search_sweep = SearchSweep(self._gate.generation, key,
                        (assignment.target_x, assignment.target_y), granted_s, remaining)
                self._resume_tracking_attempt()
        except (ValueError, TypeError, KeyError) as exc:
            rospy.logwarn_throttle(2., '[%s] AUTHORITY_MESSAGE_REJECTED %s', self.uav_id, exc)

    def _pause_search(self, now):
        sweep = getattr(self, '_search_sweep', None)
        if sweep is not None and not sweep.finished:
            self._search_remaining[sweep.key] = (now, sweep.remaining())
        self._search_sweep = None

    def _search_frame_cb(self, msg):
        try:
            frame = json.loads(msg.data)
            with self._authority_lock:
                now = rospy.Time.now().to_sec()
                active = dict(task=self._gate.task, generation=self._gate.generation,
                    started_s=self._search_granted_s, stopping=self._gate.stopping,
                    expires_s=self._gate.expires_s) if self._gate.task is not None else None
                if not self._search_frames.receive(frame, active, now):
                    return
                sweep = self._search_sweep
                xy = self.world_xy
                if (sweep is None or sweep.phase != 'OBSERVE' or xy is None
                        or not 0 <= now-self._pose_sample_s <= .5
                        or math.dist(xy, frame['camera_xyz'][:2]) > 1.5):
                    return
                with self._online_map_lock:
                    visible = visible_samples(frame, self.cov_grid, self._online_map, now)
                sweep.add_frame(frame, visible, now)
        except (ValueError, TypeError, KeyError):
            return

    def _choose_search_view(self, sweep, now):
        grid = self._online_safe_grid
        if (grid is None or self._online_safe_s is None or not 0 <= now-self._online_safe_s <= 1.5
                or self._online_safe_epoch != self._online_map_epoch_s):
            return None
        cx, cy = sweep.origin
        candidates = [(cx+dx, cy+dy) for dx, dy in
                      ((-5., 0.), (5., 0.), (0., -5.), (0., 5.), (-4., -4.), (-4., 4.), (4., -4.), (4., 4.))]
        if len(sweep.views) > sweep.view_idx+1:
            candidates.insert(0, sweep.views[sweep.view_idx+1])
        usable = []
        with self._online_map_lock:
            for p in candidates:
                if (self.cov_grid.world_to_cell(*p) is None or math.dist(p, sweep.origin) > 8.
                        or any(math.dist(p, used) < 2. for used in sweep.views[:sweep.view_idx+1])
                        or not self._online_planner.connector_clear(grid, self.world_xy, p)):
                    continue
                distance = math.dist(p, self.world_xy)
                # Query the existing current-scan footprint checker over the
                # entire candidate connector. This velocity is never published.
                speed = (-RADAR_BRAKE_MPS2*RADAR_LATENCY_S+math.sqrt(
                    (RADAR_BRAKE_MPS2*RADAR_LATENCY_S)**2+2*RADAR_BRAKE_MPS2*distance))
                query = tuple((p[i]-self.world_xy[i])*speed/max(distance, 1e-9) for i in range(2))
                if self._online_planner.local_command_clear(self._online_map, self.world_xy, query, now,
                        self._online_map_epoch_s, latency=RADAR_LATENCY_S, brake=RADAR_BRAKE_MPS2):
                    usable.append(p)
        return min(usable, key=lambda p: (math.dist(p, self.world_xy), p)) if usable else None

    def _publish_search_result(self, now):
        sweep = self._search_sweep
        if sweep.report is None or sweep.report_ack or now-self._search_feedback_s < .5:
            return
        self._search_feedback_seq += 1
        message = dict(sweep.report, seq=self._search_feedback_seq, sample_s=now,
                       position_xy=list(self.world_xy))
        sweep.sent_sequences.add(self._search_feedback_seq)
        self._search_feedback_pub.publish(String(data=json.dumps(message, allow_nan=False)))
        self._search_feedback_s = now

    def _search_feedback_ack_cb(self, msg):
        try:
            message = json.loads(msg.data)
            with self._authority_lock:
                sweep = self._search_sweep
                now = rospy.Time.now().to_sec()
                if (sweep is None or set(message) != {'schema_version', 'run_id', 'uav_id', 'generation', 'seq', 'view_idx', 'accepted'}
                        or type(message['schema_version']) is not int or message['schema_version'] != 1
                        or message['run_id'] != self._gate.run_id or message['uav_id'] != self.uav_id
                        or message['accepted'] is not True or type(message['seq']) is not int
                        or message['seq'] not in sweep.sent_sequences
                        or type(message['generation']) is not int or message['generation'] != sweep.generation
                        or type(message['view_idx']) is not int or message['view_idx'] != sweep.view_idx
                        or not self._gate.can_move(now)):
                    return
                sweep.report_ack = True
                if not sweep.finished and sweep.pending_view is not None:
                    sweep.next_view(sweep.pending_view, now)
                    self.path, self.path_target = [], None
                    self._route_pending = None
                    self._route_gate.clear()
                    self._last_flight_v = (0., 0.)
                    with self._plan_lock:
                        self._plan_ticket += 1
                        self._plan_pending = None
        except (ValueError, TypeError, KeyError):
            return

    def _finish_search_view(self, sweep, now, reason=None):
        frames = sweep.frames if sweep.phase == 'OBSERVE' else []
        fresh = frames and 0 <= now-frames[-1]['image_s'] <= 1.
        if reason is None:
            reason = ('NO_FRESH_FRAMES' if len(frames) < MIN_FRAMES or not fresh else
                      'OBSERVED' if any(p['visible'] for p in frames) else 'VIEW_OCCLUDED')
        if reason == 'NO_FRESH_FRAMES':
            frames = []  # Old image timestamps never regain freshness by retransmission.
        new_probes = sum(1 for identity in proven_points(frames)
                         if identity not in self._search_ledger.points
                         or now-self._search_ledger.points[identity] >= 30.)
        useful = new_probes >= 5  # One cell's worth of new sampled view, including neighbours.
        self._search_ledger.record(frames)
        complete = self._search_ledger.complete(sweep.key, now)
        next_view = (self._choose_search_view(sweep, now)
                     if reason != 'NO_FRESH_FRAMES' and reason not in
                     ('NO_ROUTE_COMMIT', 'NO_ACTUAL_PROGRESS', 'EXECUTION_GATE_BLOCKED')
                     and not (complete or useful) and sweep.view_idx < 2 else None)
        finished = next_view is None
        if (finished and not (complete or useful) and sweep.view_idx < 2 and reason in ('OBSERVED', 'VIEW_OCCLUDED')):
            reason = 'NO_CONNECTED_VIEW'
        sweep.report = dict(schema_version=1, run_id=self._gate.run_id, uav_id=self.uav_id,
            generation=sweep.generation, cell=list(sweep.key), view_idx=sweep.view_idx,
            view_xy=list(sweep.goal), phase='NEXT_VIEW' if complete or useful or next_view is not None else 'REVIEW_PENDING',
            outcome=reason, finished=finished, window_start_s=sweep.window_s if sweep.window_s is not None else now,
            blocked_since_s=sweep.progress_s if sweep.progress_s is not None else now, frames=frames)
        self._search_feedback_s = -1.
        sweep.report_started_s = now
        self._publish_search_result(now)
        rospy.loginfo('[%s] SEARCH_VIEW_RESULT gen=%s view=%s outcome=%s frames=%s complete=%s new_probes=%s finished=%s',
                      self.uav_id, sweep.generation, sweep.view_idx, reason, len(frames), complete, new_probes, finished)
        if finished:
            sweep.finished = True
            sweep.phase = sweep.report['phase']
        else:
            sweep.pending_view, sweep.phase = next_view, 'NEXT_VIEW'

    def _control_search_view(self, now):
        sweep = self._search_sweep
        if sweep is None:
            self._look_at = None
            self._send_vel(0., 0.)
            return True
        evidence = self._measured_motion(now)
        fresh = (evidence is not None and 0 <= now-evidence['sample_s'] <= .5
                 and 0 <= now-self._pose_sample_s <= .5)
        speed = evidence['speed_mps'] if fresh else float('inf')
        if sweep.finished or sweep.phase == 'NEXT_VIEW':
            self._look_at = None
            self._send_vel(0., 0.)
            if fresh and speed <= .15:
                self._publish_search_result(now)
                # Lost feedback/receipt must not become a new infinite hover.
                # This existing advisory channel requests STOP, never releases a lock.
                if not sweep.report_ack and sweep.report_started_s is not None and now-sweep.report_started_s >= 6.:
                    self._blocked_feedback_seq += 1
                    self._navigation_pub.publish(String(data=json.dumps(dict(schema_version=1,
                        run_id=self._gate.run_id, uav_id=self.uav_id, generation=sweep.generation,
                        seq=self._blocked_feedback_seq, sample_s=now, blocked_since_s=sweep.report_started_s,
                        position_xy=list(self.world_xy), reason='NO_REACHABLE_PROGRESS'), allow_nan=False)))
                    rospy.logwarn_throttle(5., '[%s] SEARCH_FEEDBACK_TIMEOUT -> advisory STOP', self.uav_id)
            return True
        if sweep.phase == 'OBSERVE':
            if math.dist(self.world_xy, sweep.goal) > 1.:
                sweep.phase, sweep.window_s, sweep.frames = 'GO_TO_VIEW', None, []
                sweep.progress_s, sweep.progress_xy = now, self.world_xy
                return False
            # Refresh the existing planning snapshot for conditional next views.
            if self._online_planner is not None and now-self._last_online_request_s >= 1.:
                self._request_plan(sweep.goal)
                self._last_online_request_s = now
            yaw = sweep.yaw(now)
            self._look_at = (self.world_xy[0]+10.*math.cos(yaw), self.world_xy[1]+10.*math.sin(yaw))
            self._send_vel(0., 0.)  # Existing XYZ position hold, including altitude.
            if fresh and speed <= .25 and sweep.view_ready(now):
                self._finish_search_view(sweep, now)
            return True
        if math.dist(self.world_xy, sweep.goal) < ARRIVE_TOL:
            yaw = (math.atan2(sweep.origin[1]-self.world_xy[1], sweep.origin[0]-self.world_xy[0])
                   if sweep.view_idx else self.yaw)
            sweep.start_observing(now, self.world_xy, speed, yaw)
            self._look_at = None
            self._send_vel(0., 0.)
            return True
        record = self._route_gate.record
        has_route = record is not None and record['generation'] == sweep.generation and now < record['expires_s']
        reason = (sweep.blocked_reason(now, self.world_xy, speed, has_route, self._final_stop_reason)
                  if fresh else None)
        if reason:
            self._look_at = None
            self._send_vel(0., 0.)
            self._finish_search_view(sweep, now, reason)
            return True
        return False

    def _publish_authority_state(self):
        now = rospy.Time.now().to_sec()
        if getattr(self,'_escape_enabled',False):
            self._update_escape_rest(now)
        quality = getattr(self, '_pose_quality', None)
        if quality is not None and not quality.usable(now):
            self._stopped_since = None
            return
        evidence = self._measured_motion(now)
        xyz = self.world_xy
        if (xyz is None or self.local_z is None or evidence is None
                or not 0 <= now - self._pose_sample_s <= .5
                or not 0 <= now - evidence['sample_s'] <= .5):
            self._stopped_since = None
            return
        speed = evidence['speed_mps']
        if not all(math.isfinite(v) for v in (*xyz, self.local_z, speed)):
            self._stopped_since = None
            return
        if self._gate.stopping and speed <= .15:
            if self._stopped_since is None or now < self._stopped_since:
                self._stopped_since = now
        else:
            self._stopped_since = None
        duration = now - self._stopped_since if self._stopped_since is not None else 0.
        self._ack_seq += 1
        message = dict(schema_version=2, run_id=self._gate.run_id, uav_id=self.uav_id,
                       seq=self._ack_seq, generation=self._gate.generation,
                       sample_s=min(self._pose_sample_s, evidence['sample_s']),
                       xyz=[xyz[0], xyz[1], self.local_z], speed_mps=speed,
                       stopped_s=duration, status='STOPPED' if duration >= 1. else 'STATE')
        self._authority_ack_pub.publish(String(data=json.dumps(message, allow_nan=False)))

    def _route_grant_cb(self, msg):
        try:
            message = json.loads(msg.data)
            with self._authority_lock:
                if not self._gate.can_move(rospy.Time.now().to_sec()):
                    return
                pending = self._route_pending
                expected = pending[0] if pending else self._route_offer_id
                renewing_current = bool(pending and message.get('offer_id') == self._route_offer_id
                                        and message.get('points') == self.path)
                if renewing_current:
                    expected = self._route_offer_id
                if not self._route_gate.receive(message, rospy.Time.now().to_sec(), self._gate.generation, expected):
                    return
                if pending and not renewing_current:
                    ticket, path, target, generation = pending
                    if ticket != self._plan_ticket or generation != self._gate.generation or message['points'] != path:
                        self._route_gate.clear()
                        return
                    escape = getattr(self,'_escape_pending',None)
                    if escape is not None and escape['ticket'] == ticket:
                        now = rospy.Time.now().to_sec()
                        if (escape['generation'] != generation
                                or escape['offset'] != self.offset
                                or escape['epoch'] != self._online_map_epoch_s
                                or not 0 <= now-escape['scan_s'] <= 1.5
                                or generation in self._escape_used
                                or not self._update_escape_rest(now)
                                or not self._escape_path_valid([list(self.world_xy)]+path,escape)):
                            self._route_gate.clear()
                            self._route_pending = self._escape_pending = None
                            rospy.loginfo('[%s] BOUNDED_ESCAPE_GRANT_REJECTED gen=%s offer=%s',
                                          self.uav_id,generation,ticket)
                            return
                        escape['started_s'] = now
                        escape['local_z'] = self.local_z
                        escape['points'] = path
                        escape['travel_m'] = 0.
                        escape['last_pose_xy'] = self.world_xy
                        escape['pose_s'] = self._pose_sample_s
                        self._escape_active = escape
                        self._escape_used.add(generation)
                        self._escape_pending = None
                        rospy.loginfo('[%s] BOUNDED_ESCAPE_COMMITTED gen=%s offer=%s',
                                      self.uav_id,generation,ticket)
                    elif getattr(self,'_escape_active',None) is not None:
                        self._escape_active = None
                    self.path, self.path_target = path, target
                    self._route_offer_id = ticket
                    self._route_pending = None
                    self._escape_pending = None
                    rospy.loginfo('[%s] ROUTE_COMMITTED offer=%s points=%s', self.uav_id, ticket, len(path))
        except (ValueError, TypeError, KeyError):
            return

    def _route_snapshot_cb(self, msg):
        try:
            snapshot = json.loads(msg.data)
            if (set(snapshot) != {'schema_version', 'run_id', 'seq', 'reservations'}
                    or type(snapshot['schema_version']) is not int or snapshot['schema_version'] != 1
                    or snapshot['run_id'] != self._gate.run_id
                    or type(snapshot['seq']) is not int or snapshot['seq'] <= self._peer_routes_seq
                    or not isinstance(snapshot['reservations'], dict)):
                return
            for uid, record in snapshot['reservations'].items():
                if (uid not in self._motion_cache.fleet or set(record) != {'generation', 'segments'}
                        or type(record['generation']) is not int or record['generation'] < 1
                        or not isinstance(record['segments'], list) or len(record['segments']) > 65536
                        or not all(valid_points(pair) and len(pair) == 2 for pair in record['segments'])):
                    return
            with self._authority_lock:
                if snapshot['seq'] <= self._peer_routes_seq:
                    return
                received_s = rospy.Time.now().to_sec()
                if not self._route_gate.receive_reservations(snapshot, received_s, self._gate.generation):
                    return
                self._peer_routes_seq, self._peer_routes = snapshot['seq'], snapshot['reservations']
                self._peer_routes_s = received_s
        except (ValueError, TypeError, KeyError):
            return

    def _assign_cb(self, msg):
        if msg.uav_id != self.uav_id:
            return
        self.assignment = msg
        if getattr(self, '_online_planner', None) is not None and msg.task_type in (2, 3):
            self._landing = False
            self._look_at = None
            rospy.logwarn('[%s] VERTICAL_ROUTE_EVIDENCE_MISSING -> hold', self.uav_id)
            return
        # 处理特殊任务类型
        if msg.task_type == 1:  # 目标确认/追踪
            tid = getattr(msg, 'target_id', '')
            # 2026-10-01：manager 对同一目标会以 ~5Hz 重发追踪指派
            # （_update_tracker_position 每次收到检测都发）。旧代码对每条都
            # 清空 _orbit_target、重设目标点 → 正在进行的 15s 盘旋确认被反复
            # 重置、且触发 A* 重规划风暴。同目标只静默刷新位置，换目标才重置。
            if tid and tid == getattr(self, '_track_assigned_id', None):
                # 冷却期内：同一目标的重发不再重新武装 _target_to_orbit。
                # 否则 manager 5Hz 重发会立刻覆盖 _abort_orbit 的清场，
                # _control 又进入对已丢失目标的盘旋/追踪，与放弃分支形成 20Hz 死循环。
                if rospy.Time.now().to_sec() < self._giveup_until.get(tid, 0.0):
                    return
                self._target_to_orbit = (msg.target_x, msg.target_y)
                return
            self._track_assigned_id = tid if tid else None
            rospy.loginfo("[%s] 收到目标追踪任务 %s @ (%.1f, %.1f)",
                          self.uav_id, tid if tid else '?',
                          msg.target_x, msg.target_y)
            self._orbit_target = None  # 新目标：让 _control 飞向目标点后自动盘旋
            self._target_to_orbit = (msg.target_x, msg.target_y)
        elif msg.task_type == 2:  # RTL 返航
            self._track_assigned_id = None
            rospy.loginfo("[%s] 收到 RTL 返航指令", self.uav_id)
            self.mode_srv.call(0, "RTL")  # 切换到 RTL 模式
        elif msg.task_type == 3:  # 降落
            self._track_assigned_id = None
            rospy.loginfo("[%s] 收到降落指令", self.uav_id)
            self._landing = True  # 进入降落模式
        else:
            # 搜索任务（task_type=0）：此前的追踪指派已结束，清除去重标记，
            # 否则同一目标以后再次派给本机会被误判为重复而忽略。
            self._track_assigned_id = None

    def _resume_tracking_attempt(self):
        """A new grant may retry once, after fresh accepted camera evidence."""
        if getattr(self,'_v124_behavior',False):
            return False
        pending = getattr(self, '_tracking_retry_pending', None)
        if pending is None:
            return False
        tid, generation = pending
        now = rospy.Time.now().to_sec()
        if (generation != self._gate.generation or not self._gate.can_move(now)
                or self._gate.task is None or self._gate.task['task_type'] != 1
                or self._gate.task['target_id'] != tid or tid not in ('t0','t1','t2','t3','t4','t5')
                or not math.isfinite(self._giveup_until.get(tid, 0.))):
            return False
        point = current_target_point(self.targets, self._t_seen, tid, now)
        if point is None:
            return False
        self._tracking_retry_pending = None
        self._giveup_until.pop(tid, None)
        self._reset_n[int(tid[1:])] = 0
        self._target_to_orbit = point
        rospy.loginfo('[%s] TRACKING_ATTEMPT_READY target=%s generation=%s camera_s=%.3f',
                      self.uav_id, tid, generation, self._t_seen[tid])
        return True

    def _confirmed_visual_cb(self, msg):
        try:
            with self._authority_lock:
                observation = self._visual_evidence.receive(json.loads(msg.data), rospy.Time.now().to_sec())
                if observation is None:
                    return
                tid = TAG_TO_TID[observation['target_id']]
                stopped_view = getattr(self, '_stopped_observation', None)
                if stopped_view is not None:
                    stopped_view.observe(observation['uav_id'], tid, observation['sample_s'],
                        observation['xyz'][:2], rospy.Time.now().to_sec())
                if (getattr(self, '_white_reacquire_enabled', False)
                        and observation['target_id'] == 'white' and observation['uav_id'] == self.uav_id
                        and self._gate.can_move(rospy.Time.now().to_sec())
                        and self._gate.task['task_type'] == 1 and self._gate.task['target_id'] == tid):
                    self._white_reacquire.observe(self._gate.generation, observation['sample_s'],
                        observation['xyz'][:2], rospy.Time.now().to_sec())
                if observation['sample_s'] < self._t_seen.get(tid, -1.):
                    return
                target = TargetState()
                target.header.stamp = rospy.Time.from_sec(observation['sample_s'])
                target.target_id = tid
                target.x, target.y, _ = observation['xyz']
                # Motion is fitted per camera; original image time remains unchanged.
                motion = self._visual_motion.observe(tid,observation['uav_id'],
                    observation['sample_s'],target.x,target.y)
                if motion is None:
                    return
                target.vx,target.vy,fleeing = motion
                target.state = 1 if fleeing else 0
                self._target_cb(target)
        except (ValueError, TypeError, KeyError):
            return

    def _target_cb(self, msg):
        """Cache confirmed camera coordinates and update this assigned target."""
        if msg.eliminated:
            stopped_view = getattr(self, '_stopped_observation', None)
            if stopped_view is not None:
                stopped_view.images.pop(msg.target_id, None)
            if msg.target_id == 't3' and getattr(self, '_white_reacquire', None) is not None:
                self._white_reacquire.cancel()
            self.targets.pop(msg.target_id, None)
            self._target_state.pop(msg.target_id, None)
            # BUGFIX: 目标消除后必须清除盘旋状态，否则飞机会卡在盘旋不动
            if self._orbit_target == msg.target_id:
                rospy.loginfo("[%s] 目标 %s 已消除，清除盘旋状态", self.uav_id, msg.target_id)
                self._orbit_target = None
                self._orbit_center = None
                self._confirm_start = 0.0
            # 同步解除追踪指派去重：该目标已不存在
            if getattr(self, '_track_assigned_id', None) == msg.target_id:
                self._track_assigned_id = None
                self._target_to_orbit = None
            return
        sample_s = msg.header.stamp.to_sec()
        if not 0 <= rospy.Time.now().to_sec()-sample_s <= TARGET_TTL:
            return
        self.targets[msg.target_id] = (msg.x, msg.y, msg.vx, msg.vy)
        self._target_state[msg.target_id] = (
            int(msg.state), sample_s)
        self._t_seen[msg.target_id] = sample_s
        assignment = getattr(self, 'assignment', None)
        if (getattr(assignment, 'task_type', None) == 1
                and getattr(assignment, 'target_id', None) == msg.target_id):
            # Assignment supplies the initial rendezvous, not a permanent point:
            # keep the approach aimed at fresh observations of the locked target.
            self._target_to_orbit = (msg.x, msg.y)

    def _friend_status_cb(self, msg):
        """接收友机位置和高度，用于避碰"""
        if msg.uav_id != self.uav_id:
            self._friend_positions[msg.uav_id] = (msg.x, msg.y, msg.z)

    def _finish_cb(self, msg):
        """收到任务完成广播后退出搜索循环。"""
        if msg.data == "MISSION_FINISHED":
            rospy.loginfo("[%s] 收到任务完成广播", self.uav_id)
            self._mission_finished = True
            # 兜底：如果还没进入降落模式，则进入
            if not self._landing and self.assignment is not None and getattr(self.assignment, 'task_type', 0) == 3:
                self._landing = True

    # ---------------- 目标检测（规则3：几何判定） ----------------
    def _detect_targets(self):
        """对每个已知目标做「距离 + 视线遮挡」判定，命中则发布 TargetDetection。

        规则3 的几何判定：水平距离 < DETECT_RADIUS **且** 中间无建筑遮挡。
        注意只用水平距离 —— 无人机在 6m 高度、目标在地面，垂直差恒定，
        水平距才是决定「能否看到」的量（与比赛判定的平面几何一致）。
        """
        if os.environ.get('SEED_TRUTH', '0') != '1':
            return  # Actual camera evidence is published once by the YOLO bridge.
        wx = self.world_xy
        if wx is None:
            return
        now = rospy.Time.now().to_sec()
        if now - self._last_detect_t < 1.0 / DETECT_RATE:
            return
        self._last_detect_t = now

        for tid, (tx, ty, _vx, _vy) in list(self.targets.items()):
            # 幽灵目标闸门：桥接停发超过 TARGET_TTL 的目标不再几何上报。
            # 只跳过不 pop —— 盘旋放弃逻辑（_t_seen gap）依赖条目仍在。
            if now - self._t_seen.get(tid, 0.0) > TARGET_TTL:
                continue
            d = math.hypot(tx - wx[0], ty - wx[1])
            _los_ok = self.los.visible(wx[0], wx[1], tx, ty)
            _captured = d < DETECT_RADIUS and _los_ok
            # === 算法层日志：几何检测的内部距离/阈值/捕获判定 ===
            # 排查「画面看着近但算法不消除」：必须看代码算的 d，不是肉眼。
            # 注意 rospy.loginfo_throttle 按调用点节流，循环里每秒只会打出
            # 字典第一个目标（实测 agent_5 永远只显示 t4），改为按 target
            # 分别节流，保证每个已知目标都有判定日志。
            if now - self._detect_log_t.get(tid, 0.0) >= 1.0:
                self._detect_log_t[tid] = now
                rospy.loginfo(
                    "[ALGO] detect uav=%s pos=(%.2f,%.2f) target=%s pos=(%.2f,%.2f) "
                    "dist=%.3fm | detect_radius=%.1fm | los=%s | is_captured=%s | ts=%.3f",
                    self.uav_id, wx[0], wx[1], tid, tx, ty, d,
                    DETECT_RADIUS, _los_ok, _captured, now)
            if d > DETECT_RADIUS:
                continue
            if not _los_ok:
                continue    # 隔着建筑，不算看到
            m = TargetDetection()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "map"
            m.uav_id = self.uav_id
            m.target_id = tid
            m.x, m.y = tx, ty
            m.confidence = 1.0
            m.source = 0     # 0=几何判定
            self.detect_pub.publish(m)

    # ---------------- 参数与模式 ----------------
    def _set_param(self, param_id, value):
        from mavros_msgs.msg import ParamValue
        try:
            return self.param_srv(param_id, ParamValue(integer=value, real=0.0)).success
        except Exception as exc:
            rospy.logwarn_throttle(10, "[%s] 设置 %s 异常: %s", self.uav_id, param_id, exc)
            return False

    def _configure_fcu(self):
        """SITL 无遥控器会触发 RC 失联 failsafe，拒绝解锁/进 OFFBOARD。"""
        from mavros_msgs.srv import ParamPull, ParamGet
        pull_srv = rospy.ServiceProxy(self.mavros_ns + '/param/pull', ParamPull)
        get_srv = rospy.ServiceProxy(self.mavros_ns + '/param/get', ParamGet)
        rospy.wait_for_service(self.mavros_ns + '/param/pull', timeout=30)
        def pull():
            response = pull_srv(True)
            return response.success and response.param_received > 0
        def get(name):
            response = get_srv(name)
            return response.value.integer if response.success else None
        configure_fcu_parameters(pull, self._set_param, get,
                                 {'NAV_RCL_ACT': 0, 'COM_RCL_EXCEPT': 4, 'NAV_DLL_ACT': 0})
        if os.environ.get('SWARM_EXPECT_SINGLE_EKF')=='1':
            expected={'EKF2_MULTI_IMU':0,'EKF2_MULTI_MAG':0,'SENS_IMU_MODE':1,'SENS_MAG_MODE':1}
            actual={name:get(name) for name in expected}
            if actual!=expected:
                raise RuntimeError('SINGLE_EKF_BOOTSTRAP_READBACK_FAILED:'+str(actual))
            rospy.loginfo('[%s] Single-EKF startup parameters read back and verified',self.uav_id)
        rospy.loginfo('[%s] FCU autonomous-flight parameters read back and verified', self.uav_id)

    def _arm_and_offboard(self):
        """预热 setpoint → 切 OFFBOARD → 解锁。与单机避障已验证的时序一致。

        多机 SITL 注意：第二架及以后的机 EKF 收敛更慢，local_position 可能
        出现 -12m 之类的瞬时坏值，直接切 OFFBOARD 会失败。故先等 EKF 收敛
        （local_z 就绪且 |z| 合理），OFFBOARD 切换失败则重试而非直接放弃。

        关键：预热阶段必须发「纯零速」(vz=0)，而非 _send_vel 的竖直爬升速度。
        单机脚本 dwa_avoidance 预热发 _send_vel(0,0,0)，若此处带 vz=0.5*(6-z)
        的爬升速度，EKF 未收敛时 PX4 会拒绝 OFFBOARD。
        """
        # 等 EKF 大致稳定：要求「连续 STABLE_NEEDED 次采样」高度在出生点附近。
        # 单次采样不够 —— 多机 SITL 下 EKF 位置先收敛、速度/姿态后稳定，瞬时达标
        # 就切 OFFBOARD 会被 PX4 拒。阈值放宽到 1.5m（地面上有噪声抖动），真正
        # 是否就绪交给 PX4 preflight 判定，靠后续多次重试兜底。

        # 重启协同层时飞机已在空中：跳过等待直接进 OFFBOARD
        if self.state.armed and self.state.mode == "OFFBOARD" and self.local_z is not None and self.local_z > 1.0:
            rospy.loginfo("[%s] 已在空中 OFFBOARD（z=%.2f）→ 跳过起飞等待",
                          self.uav_id, self.local_z)
            # 已离地，无法用「起飞点=当前位置」重锚定，沿用参数 offset；
            # 后续突跳必须隔离，不能凭相邻坐标自行推定世界偏移。
            self._anchor_done = True
            return True

        stable = 0
        while not rospy.is_shutdown():
            if self.local_z is not None and abs(self.local_z) < 1.5:
                stable += 1
                if stable >= STABLE_NEEDED:
                    break
            else:
                if stable > 0:
                    rospy.logwarn_throttle(5, "[%s] EKF 抖动（z=%.2f），重新计数",
                                           self.uav_id,
                                           self.local_z if self.local_z is not None else -999)
                stable = 0
            self._send_vel(0.0, 0.0, vz=0.0)
            self.ctrl_rate.sleep()
        rospy.loginfo("[%s] EKF 已稳定（z=%.2f，连续 %d 次采样达标）", self.uav_id,
                      self.local_z if self.local_z is not None else -999, stable)

        # ---- EKF 稳定时刻重锚定 offset（方案 B，2026-10-02）----
        # 「offset=起飞点世界坐标」隐含假设 EKF 局部原点恰在起飞点（local_xy≈0）。
        # 多机 SITL 下 EKF 在预热/解锁阶段仍可能重置局部原点（实测 uav_2 的
        # world_xy 跳到 (203,-148) 越界锁死）。此刻飞机物理上就在起飞点 →
        # 用稳定时刻的 local_xy 反推 offset，强制 world_xy=起飞点。
        if self._offset_param is not None and self.local_xy is not None:
            drift = math.hypot(self.local_xy[0], self.local_xy[1])
            if drift > 0.5:
                rospy.logwarn("[%s] EKF 稳定时 local_xy=(%.2f,%.2f) 偏离原点 %.2fm，"
                              "offset 重锚定 (%.2f,%.2f)→(%.2f,%.2f)",
                              self.uav_id, self.local_xy[0], self.local_xy[1], drift,
                              self.offset[0], self.offset[1],
                              self._offset_param[0] - self.local_xy[0],
                              self._offset_param[1] - self.local_xy[1])
            self.offset = (self._offset_param[0] - self.local_xy[0],
                           self._offset_param[1] - self.local_xy[1])
        self._anchor_done = True   # 此后 _local_cb 启用定位突跳隔离

        # 预热：持续发纯零速，让 OFFBOARD setpoint 生效（vz=0 而非爬升）
        for _ in range(120):
            self._send_vel(0.0, 0.0, vz=0.0)
            self.ctrl_rate.sleep()

        # 切 OFFBOARD：**持续重试直到成功**（多机 SITL 下 EKF 速度/姿态估计就绪
        # 时间不定，固定次数会误判放弃，导致必须重启整个 SITL。永不放弃 + 每次
        # 重试间隔继续发零速预热，EKF 稳了自然就进得去。）
        attempt = 0
        while not rospy.is_shutdown() and self.state.mode != "OFFBOARD":
            attempt += 1
            if not self.mode_srv(0, "OFFBOARD").mode_sent:
                rospy.logwarn_throttle(10, "[%s] 切换 OFFBOARD 请求失败（第 %d 次）",
                                       self.uav_id, attempt)
            for _ in range(50):
                if self.state.mode == "OFFBOARD":
                    break
                self._send_vel(0.0, 0.0, vz=0.0)
                self.ctrl_rate.sleep()
            if self.state.mode == "OFFBOARD":
                break
            rospy.logwarn_throttle(10, "[%s] OFFBOARD 未生效（第 %d 次，当前 %s），等待 EKF 稳定后重试",
                                   self.uav_id, attempt, self.state.mode)
            for _ in range(40):
                if self.state.mode == "OFFBOARD":
                    break
                self._send_vel(0.0, 0.0, vz=0.0)
                self.ctrl_rate.sleep()
        if rospy.is_shutdown():
            return False
        rospy.loginfo("[%s] 已进入 OFFBOARD（重试 %d 次）", self.uav_id, attempt)

        # 解锁：同样**持续重试直到成功**。PX4 preflight 会因 EKF 速度估计未稳 /
        # Roll failure 拒解锁，等 EKF 稳了自然会成功。
        attempt = 0
        while not rospy.is_shutdown() and not self.state.armed:
            attempt += 1
            if not self.arm_srv(True).success:
                rospy.logwarn_throttle(10, "[%s] 解锁请求失败（第 %d 次），等待 EKF 稳定后重试",
                                       self.uav_id, attempt)
            for _ in range(50):
                if self.state.armed:
                    break
                self._send_vel(0.0, 0.0, vz=0.0)
                self.ctrl_rate.sleep()
        if rospy.is_shutdown():
            return False
        rospy.loginfo("[%s] OFFBOARD + 解锁完成（解锁重试 %d 次）", self.uav_id, attempt)
        return True

    # ---------------- 控制 ----------------
    def _compute_desired_altitude(self):
        """根据当前状态自适应计算目标高度的偏移量。

        基础高度由 altitude_layer 决定（5.0/5.5/6.0m），
        这里返回相对于基础高度的偏移量（不超过 6m 上限）。
        策略：
        - 追踪目标时：-1.0m（更好地观察）
        - 建筑附近：-0.5m（保持距离）
        - 开阔区域：0m（常规搜索）
        """
        # A horizontal scan does not certify a lower flight plane. Keep the
        # configured search height in online mode, including while tracking.
        if getattr(self, '_online_planner', None) is not None:
            return 0.0
        wx, wy = self.world_xy
        if wx is None:
            return 0.0

        # 检查是否在追踪目标
        if self._orbit_target is not None:
            return -1.0

        # 检查是否在建筑附近（使用未膨胀的原始栅格）
        if hasattr(self, 'los') and self.los:
            # 简单判断：当前位置是否靠近障碍
            # 用 A* 膨胀后的栅格判断
            cell = self.grid.world_to_cell((wx, wy))
            if cell and not self.grid.is_free(cell):
                # 已经在障碍附近，降低高度
                return -0.5

        # 检查与最近建筑的距离
        # 简化：用 coverage grid 中心判断
        if hasattr(self, 'cov_grid') and self.cov_grid:
            cov_cell = self.cov_grid.world_to_cell(wx, wy)
            if cov_cell:
                cx = self.cov_grid.x_min + (cov_cell[0] + 0.5) * self.cov_grid.cell_m
                cy = self.cov_grid.y_min + (cov_cell[1] + 0.5) * self.cov_grid.cell_m
                # 检查这个位置是否在建筑附近
                # 简化：使用 A* 栅格判断
                astar_cell = self.grid.world_to_cell((wx, wy))
                if astar_cell and not self.grid.is_free(astar_cell):
                    return -0.5

        # 开阔区域
        return 0.0

    def _radar_guard_velocity(self, vx, vy):
        """Final planar constraint; cannot add unverified lateral motion."""
        if not RADAR_GUARD:
            return vx, vy
        now = rospy.Time.now().to_sec()
        scan = self._scan
        evidence = self._measured_motion(now)
        pose_age = now - self._local_prev_t
        if (scan is None or evidence is None or not 0 <= pose_age <= RADAR_FRESH_S
                or not 0 <= now - evidence['sample_s'] <= RADAR_FRESH_S):
            self._final_stop_reason = 'RADAR_INPUT_MISSING_OR_STALE'
            rospy.logwarn_throttle(2., '[%s] RADAR_INPUT_MISSING_OR_STALE -> horizontal stop',
                                   self.uav_id)
            return 0., 0.
        for measured in evidence['velocity_candidates']:
            result = guard_velocity(
                (vx, vy), measured, self.yaw, scan.ranges,
                scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
                scan.header.stamp.to_sec(), now, max_age=RADAR_FRESH_S,
                radius=RADAR_STOP_R, latency=RADAR_LATENCY_S, brake_accel=RADAR_BRAKE_MPS2)
            if result['reason'] not in ('CLEAR', 'REQUESTED_STOP'):
                self._final_stop_reason = 'RADAR_'+result['reason']
                rospy.logwarn_throttle(2., '[%s] radar_guard=%s clearance=%s',
                                       self.uav_id, result['reason'], result['clearance_m'])
            vx, vy = result['velocity_xy']
        return vx, vy

    def _update_escape_rest(self, now):
        task = self._gate.task
        quality = getattr(self,'_pose_quality',None)
        last_command = getattr(self,'_last_flight_v',None)
        eligible = (self._gate.can_move(now) and task is not None and task['task_type'] == 0
            and getattr(self,'_takeoff_done',False) and not self._landing
            and getattr(self,'_pose_rate_enabled',False)
            and quality is not None and quality.usable(now)
            and self.world_xy is not None and self.local_z is not None
            and math.isfinite(self.local_z) and self.local_z <= ALT_HARD_CEIL
            and self.offset is not None and 0 <= now-self._pose_sample_s <= .5
            and last_command is not None and math.hypot(*last_command) < 1e-9)
        key = (self._gate.run_id,self._gate.generation,self.offset,self._online_map_epoch_s)
        self._escape_ready = self._escape_rest.update(
            self._measured_motion(now) if eligible else None,now,key if eligible else None)
        return self._escape_ready

    def _cancel_escape(self, reason):
        if getattr(self,'_escape_active',None) is not None:
            rospy.loginfo('[%s] BOUNDED_ESCAPE_END reason=%s generation=%s',
                          self.uav_id,reason,self._escape_active['generation'])
            self.path, self.path_target = [], None
            self._last_flight_v = self._last_cmd_v = (0.,0.)
        self._escape_active = self._escape_pending = None
        self._escape_altitude_request = None
        self._escape_ready = False
        self._escape_rest.reset()

    def _current_escape(self, now):
        escape = getattr(self,'_escape_active',None)
        if escape is None:
            return None
        record = self._route_gate.record
        quality = getattr(self,'_pose_quality',None)
        if (not self._gate.can_move(now) or self._gate.task is None
                or self._gate.task['task_type'] != 0
                or escape['generation'] != self._gate.generation
                or escape['offset'] != self.offset or escape['epoch'] != self._online_map_epoch_s
                or not 0 <= now-escape['started_s'] <= 15.
                or not getattr(self,'_takeoff_done',False) or self._landing
                or quality is None or not quality.usable(now)
                or self.local_z is None or not math.isfinite(self.local_z)
                or self.world_xy is None or self._measured_motion(now) is None
                or not 0 <= now-self._pose_sample_s <= .5
                or record is None or record['generation'] != escape['generation']
                or record['offer_id'] != escape['ticket'] or record['points'] != escape['points']
                or now < self._route_gate.last_s or now >= record['expires_s']):
            self._cancel_escape('AUTHORITY_ROUTE_POSE_OR_TIMEOUT')
            return None
        if self._pose_sample_s < escape['pose_s']:
            self._cancel_escape('POSE_TIME_BACKWARDS')
            return None
        if self._pose_sample_s > escape['pose_s']:
            escape['travel_m'] += math.dist(self.world_xy,escape['last_pose_xy'])
            escape['last_pose_xy'],escape['pose_s'] = self.world_xy,self._pose_sample_s
        if escape['travel_m'] > 2.:
            self._cancel_escape('MEASURED_EXIT_DISTANCE_EXCEEDED')
            return None
        return escape

    def _escape_path_valid(self, path, proposal):
        if sum(math.dist(a,b) for a,b in zip(path,path[1:])) > 2.+1e-9:
            return False
        for before,after in zip(path,path[1:]):
            step = (after[0]-before[0],after[1]-before[1])
            if (not OnlinePlanner.connector_clear(proposal['grid'],before,after)
                    or any(sum((before[i]-hit[i])*step[i] for i in (0,1)) < -1e-9
                           for hit in proposal['blocking_hits'])):
                return False
        return True

    def _escape_limit_velocity(self, vx, vy, now):
        was_active = getattr(self,'_escape_active',None) is not None
        escape = self._current_escape(now)
        if escape is None:
            if was_active:
                self._final_stop_reason = 'ESCAPE_AUTHORITY_INVALIDATED'
                return 0.,0.
            return vx,vy
        if any((self.world_xy[0]-hit[0])*vx+(self.world_xy[1]-hit[1])*vy < -1e-9
               for hit in escape['blocking_hits']):
            self._final_stop_reason = 'ESCAPE_DIRECTION_TOWARD_HIT'
            return 0.,0.
        speed = math.hypot(vx,vy)
        scale = min(1.,.3/max(speed,1e-9))
        return vx*scale,vy*scale

    def _control_escape(self, now):
        escape = self._current_escape(now)
        if escape is None:
            return False
        with self._online_map_lock:
            complete = (math.dist(self.world_xy,escape['points'][-1]) <= .15
                and self._online_planner.local_command_clear(self._online_map,self.world_xy,
                    (0.,0.),now,self._online_map_epoch_s))
        if complete:
            self._cancel_escape('NORMAL_CLEARANCE_REJOINED')
            self._send_vel(0.,0.)
            return True
        goal = self._pick_local_goal()
        self._look_at = goal
        if goal is None:
            self._send_vel(0.,0.)
        else:
            vx,vy = (POS_KP*(goal[i]-self.world_xy[i]) for i in (0,1))
            vx,vy = self._escape_limit_velocity(vx,vy,now)
            self._send_vel(*self._apply_friend_avoidance(vx,vy))
        return True

    def _body_proof_diagnostic(self):
        def snapshot(planner):
            if planner is None:
                return None
            with planner._body_lock:
                return dict(radius_m=planner.radius,seed_xy=list(planner.seed_xy),
                    epoch=planner.epoch,scan_s=planner._body_sample_s,
                    indices=sorted(planner.body_proof))
        escape = getattr(self,'_escape_active',None)
        return dict(map_epoch_s=getattr(self,'_online_map_epoch_s',None),
            normal=snapshot(getattr(self,'_online_planner',None)),
            escape=snapshot(escape['planner']) if escape is not None else None)

    def _online_velocity_clear(self, vx, vy):
        planner = getattr(self, '_online_planner', None)
        if planner is None or abs(vx)+abs(vy) < 1e-9:
            return True
        now = rospy.Time.now().to_sec()
        escape = self._current_escape(now) if getattr(self,'_escape_enabled',False) else None
        if escape is not None:
            planner = escape['planner']
        position = self.world_xy
        evidence = self._measured_motion(now)
        if position is None or evidence is None:
            return False
        with self._online_map_lock:
            return all(planner.local_command_clear(self._online_map,position,velocity,now,
                       self._online_map_epoch_s) for velocity in [(vx,vy)]+evidence['velocity_candidates'])

    def _route_velocity_clear(self, vx, vy, now):
        evidence = self._measured_motion(now)
        return evidence is not None and all(self._route_gate.command_clear(
            self.world_xy, (vx, vy), measured, now, self._gate.generation)
            for measured in evidence['velocity_candidates'])

    def _route_guard_velocity(self, vx, vy, now, stage='final'):
        gate = getattr(self, '_route_gate', None)
        if gate is None or abs(vx)+abs(vy) < 1e-9:
            return vx, vy
        evidence = self._measured_motion(now)
        if evidence is None:
            if not isinstance(getattr(self, '_route_velocity_evidence', None), dict):
                self._route_velocity_evidence = {}
            self._route_velocity_evidence[stage] = dict(reason='ROUTE_MOTION_EVIDENCE_MISSING')
            self._final_stop_reason = 'ROUTE_MOTION_EVIDENCE_MISSING'
            return 0., 0.
        decision = gate.limited_command(self.world_xy, (vx, vy), evidence['velocity_candidates'],
                                        now, self._gate.generation)
        if not isinstance(getattr(self, '_route_velocity_evidence', None), dict):
            self._route_velocity_evidence = {}
        self._route_velocity_evidence[stage] = decision
        if decision['scale'] == 0.:
            self._final_stop_reason = decision['reason']
        return decision['velocity_xy']

    def _grid_blocked(self, wx, wy):
        """世界点在栅格上是否不可通行（障碍或越界；越界也视为墙，防冲出地图）。"""
        c = self.grid.world_to_cell((wx, wy))
        return c is None or not self.grid.is_free(c)

    def _grid_guard_velocity(self, vx, vy):
        """无激光时用 A* 栅格做执行层近障约束（20Hz，纯查表）。

        沿速度方向按 GRID_GUARD_STEP 逐点前瞻到刹停距离；任一点被占，就在
        当前航向左右各 15° 起步搜索自由扇区，把速度转向该方向；搜不到则刹停。
        雷达在线时雷达先修，本方法再兜底，二者不冲突。
        """
        if not GRID_GUARD:
            return vx, vy
        wx = self.world_xy
        if wx is None:
            return vx, vy
        planner = getattr(self, '_online_planner', None)
        if planner is not None:
            # The online grid already includes obstacle and peer envelopes.
            # Legacy cone steering can leave the committed route and get stopped
            # by the route gate. Slow along the requested direction instead.
            for scale in (1., .8, .6, .4, .2):
                candidate = (vx*scale, vy*scale)
                if self._online_velocity_clear(*candidate):
                    return candidate
            self._final_stop_reason = 'ONLINE_SPACE_UNKNOWN'
            return 0., 0.
        spd = math.hypot(vx, vy)
        if spd < 0.05:
            return vx, vy
        hd = math.atan2(vy, vx)
        # 刹停距离 v²/(2a) + 裕度；至少看 0.8m。裕度加大防建筑偏大/超调。
        look = max(0.8, spd * spd / (2.0 * MAX_ACC) + 1.5)
        # 锥形前瞻：正前方 + 左右各 15°/30°，避免楼角贴着直航线擦过去
        # （UAV3 在 (-23.5,36.5) 坠毁就是楼角从侧前方切入，单线采样漏检）。
        cone_angles = [0.0, math.radians(15), math.radians(-15),
                       math.radians(30), math.radians(-30)]
        d0 = GRID_GUARD_STEP
        n_pts = max(1, int(math.ceil(look / d0)))
        blocked = False
        for a in cone_angles:
            ca = hd + a
            for k in range(1, n_pts + 1):
                d = min(look, k * d0)
                if self._grid_blocked(wx[0] + d * math.cos(ca),
                                      wx[1] + d * math.sin(ca)):
                    blocked = True
                    break
            if blocked:
                break
        if not blocked:
            return vx, vy
        # 左右搜索自由航向（15° 步进到 120°；同侧从小到大，取偏转最小的）
        best = None
        for deg in range(15, 121, 15):
            hit = False
            for s in (1.0, -1.0):
                a = hd + s * math.radians(deg)
                clear = True
                for k in range(1, n_pts + 1):
                    d = min(look, k * d0)
                    if self._grid_blocked(wx[0] + d * math.cos(a),
                                          wx[1] + d * math.sin(a)):
                        clear = False
                        break
                if clear:
                    best = a
                    hit = True
                    break
            if hit:
                break
        if best is None:
            rospy.logwarn_throttle(2.0, '[%s] 栅格近障：前方堵死 → 刹停', self.uav_id)
            return 0.0, 0.0
        # 近障转向时强制减速：避免高速转弯离心/超调擦墙
        safe_spd = min(spd, 1.5)
        rospy.logwarn_throttle(2.0, '[%s] 栅格近障：前方被占 → 转向 %.0f° 减速 %.1f→%.1f m/s',
                               self.uav_id, math.degrees(best), spd, safe_spd)
        return safe_spd * math.cos(best), safe_spd * math.sin(best)

    def _map_guard_velocity(self, vx, vy):
        """地图边界护栏：禁止速度把飞机继续带出 A* 栅格。

        越界时锁死朝外分量；边界 MAP_GUARD_MARGIN 内禁止朝外；再往外扩
        MAP_GUARD_SOFT 一段做线性减速，避免高速贴边转弯时惯性冲出栅格
        （UAV3 坠毁后位置 0.5s 内跳 20m，就是撞楼+边界外失控的组合）。
        """
        wxy = self.world_xy
        if wxy is None:
            return vx, vy
        xmin, ymin = self.grid.origin
        xmax = xmin + self.grid.width * self.grid.resolution
        ymax = ymin + self.grid.height * self.grid.resolution
        x, y = wxy
        if x < xmin or x > xmax or y < ymin or y > ymax:
            rospy.logerr_throttle(2.0,
                                  '[%s] 世界坐标越界 (%.1f,%.1f)，限制速度回收',
                                  self.uav_id, x, y)
            return (max(0.0, vx) if x < xmin else min(0.0, vx)
                    if x > xmax else vx,
                    max(0.0, vy) if y < ymin else min(0.0, vy)
                    if y > ymax else vy)
        # 软减速带：进入 MAP_GUARD_MARGIN+MAP_GUARD_SOFT 带后把朝外分量压到 ≤0.5m/s
        soft = MAP_GUARD_MARGIN + MAP_GUARD_SOFT
        if x <= xmin + soft and vx < 0:
            vx = min(vx, 0.5)
        elif x >= xmax - soft and vx > 0:
            vx = min(vx, 0.5)
        if y <= ymin + soft and vy < 0:
            vy = min(vy, 0.5)
        elif y >= ymax - soft and vy > 0:
            vy = min(vy, 0.5)
        if x <= xmin + MAP_GUARD_MARGIN:
            vx = max(0.0, vx)
        elif x >= xmax - MAP_GUARD_MARGIN:
            vx = min(0.0, vx)
        if y <= ymin + MAP_GUARD_MARGIN:
            vy = max(0.0, vy)
        elif y >= ymax - MAP_GUARD_MARGIN:
            vy = min(0.0, vy)
        return vx, vy

    def _bounds_recovery_velocity(self):
        """越界主动回收速度。未越界返回 None；越界返回朝界内的限速速度 (vx, vy)。

        界外没有障碍物，因此该速度必须绕过雷达/栅格/地图守卫直接下发。
        目标点 = 把当前位置夹到「边界内缩 OOB_RECOVER_INSET」处。
        """
        wxy = self.world_xy
        if wxy is None:
            return None
        xmin, ymin = self.grid.origin
        xmax = xmin + self.grid.width * self.grid.resolution
        ymax = ymin + self.grid.height * self.grid.resolution
        x, y = wxy
        if xmin <= x <= xmax and ymin <= y <= ymax:
            return None
        inset = min(OOB_RECOVER_INSET,
                    (xmax - xmin) * 0.25, (ymax - ymin) * 0.25)
        tx = min(max(x, xmin + inset), xmax - inset)
        ty = min(max(y, ymin + inset), ymax - inset)
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return 0.0, 0.0
        spd = min(OOB_RECOVER_SPEED, d)      # 接近目标点时减速
        return spd * dx / d, spd * dy / d

    def _send_vel(self, vx, vy, vz=None):
        """下发 ENU 水平速度 + 自适应高度。

        vz=None 时按当前高度竖直 P 控制爬升到自适应高度；预热/悬停阶段显式传
        vz=0.0 发纯零速，避免 EKF 未收敛时带爬升速度导致 OFFBOARD 被拒。
        """
        self._final_stop_reason = 'REQUESTED_STOP'
        self._route_velocity_evidence = {}
        requested_xy = (vx, vy)
        self._escape_altitude_request = None
        if getattr(self,'_escape_enabled',False):
            vx,vy = self._escape_limit_velocity(vx,vy,rospy.Time.now().to_sec())
        quality = getattr(self, '_pose_quality', None)
        if quality is not None and not quality.usable(rospy.Time.now().to_sec()):
            self._last_cmd_v = (0., 0.)
            self._last_flight_v = (0., 0.)
            self._publish_command(TwistStamped())
            return
        # 最高优先级：越界主动回收，绕过其他守卫（界外无障碍）。
        rec = self._bounds_recovery_velocity()
        if rec is not None:
            vx, vy = rec
            rospy.logerr_throttle(2.0,
                '[%s] 越界主动回收 → (%.1f,%.1f) 当前(%.1f,%.1f)',
                self.uav_id, vx, vy,
                self.world_xy[0], self.world_xy[1])
        else:
            # 无激光时栅格兜底（雷达在线也再过一道，双保险）
            vx, vy = self._grid_guard_velocity(vx, vy)
            vx, vy = self._map_guard_velocity(vx, vy)

        # Budget speed against the committed route before accelerating into a
        # corner or endpoint. Final gates below still check the actual command.
        vx, vy = self._route_guard_velocity(vx, vy, rospy.Time.now().to_sec(),stage='before_acceleration')

        # === 2026-09-27：水平加速度限幅 ===
        # 避让增益提高后，ORCA 输出可能在相邻帧跳到近乎反向（22:29 轮事故：
        # iris_3 因此把高度超调从 0.2m 放大到 1.5m，冲过官方 6m 红线，score=0）。
        # 这里把「指令加速度」钉死，PX4 位置环才有能力跟踪，机身不再大幅倾斜。
        if not hasattr(self, "_last_cmd_v"):
            self._last_cmd_v = None
        # A requested/guarded stop must reach the position brake this tick.
        # Slewing zero from the previous velocity delayed XYZ hold by 0.316s
        # in the actual white backoff window and kept altitude control active.
        # Nonzero motion still uses the existing acceleration limit.
        if abs(vx)+abs(vy) < 1e-9:
            vx, vy = 0., 0.
        elif self._last_cmd_v is not None:
            _lvx, _lvy = self._last_cmd_v
            _maxdv = MAX_ACC / CTRL_RATE
            _dvx, _dvy = vx - _lvx, vy - _lvy
            _dvn = math.hypot(_dvx, _dvy)
            if _dvn > _maxdv and _dvn > 1e-9:
                vx = _lvx + _dvx * _maxdv / _dvn
                vy = _lvy + _dvy * _maxdv / _dvn
        self._last_cmd_v = (vx, vy)

        # === 高度最后防线（加速度限幅之后再裁，避免被限幅抵消）===
        _zn = self.local_z
        if _zn is not None and _zn > ALT_PANIC:
            vx = vx * ALT_PANIC_HSCALE
            vy = vy * ALT_PANIC_HSCALE

        # 计算目标高度 = ID对应基础高度 + 场景偏移（不超过 6m 上限）
        base_alt = self.altitude_layer
        alt_offset = self._compute_desired_altitude()  # 返回偏移量
        # === 2026-09-29 P1-1：目标高度必须低于硬顶，否则极限环 ===
        # ALT_HARD_CEIL 默认 4.2，而 ALT_TARGET_CAP=4.5 / 顶层目标 4.6，
        # 目标高于硬顶 -> 爬上去被压回 -> 再爬 -> uav_4 高度极限环。
        _alt_cap = min(ALT_TARGET_CAP, ALT_HARD_CEIL - ALT_HARD_MARGIN)
        if _alt_cap < MIN_CRUISE_ALT:
            _alt_cap = MIN_CRUISE_ALT
        target_alt = max(MIN_CRUISE_ALT, min(_alt_cap, base_alt + alt_offset))
        # 上限 4.5（远低于官方 6m），下限 MIN_CRUISE_ALT 防止追踪降高过低

        cmd = TwistStamped()
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        # 纯速度模式：vz 语义是「保持当前高度」，这里给竖直 P 控制爬升到自适应高度
        if vz is not None:
            cmd.twist.linear.z = vz
        elif self.local_z is not None:
            # 自适应高度控制
            alt_error = target_alt - self.local_z
            cmd.twist.linear.z = ALT_P * alt_error  # P 控制
            # 给小误差一个最小升降速度，避免贴近目标高度时 P 输出趋近于 0
            # 导致长期悬停在偏低高度（UAV1/UAV5 实测 z=2.42/1.95 < 目标）。
            if alt_error > 0.05:
                cmd.twist.linear.z = max(cmd.twist.linear.z, ALT_VZ_MIN)
            elif alt_error < -0.05:
                cmd.twist.linear.z = min(cmd.twist.linear.z, -ALT_VZ_MIN)
            # 限制升降速度
            if cmd.twist.linear.z > 1.0:
                cmd.twist.linear.z = 1.0
            if cmd.twist.linear.z < -1.0:
                cmd.twist.linear.z = -1.0
        # === 高度硬护栏 ===
        # 官方 score_cal.py 判定 z > 6.0 直接 score=0 并 _finish() 终止任务，
        # 是一票否决。不能只依赖 P 控制不超调，这里无条件兜底强制下降。
        if self.local_z is not None and self.local_z > ALT_HARD_CEIL:
            cmd.twist.linear.z = ALT_HARD_DESCENT
            _alt_guard_hit('HARD', self.local_z, ALT_HARD_CEIL, ALT_HARD_DESCENT,
                           cmd.twist.linear.x, cmd.twist.linear.y)
        if self.local_z is not None and self.local_z > ALT_PANIC:
            cmd.twist.linear.z = ALT_PANIC_DESCENT
            _alt_guard_hit('PANIC', self.local_z, ALT_PANIC, ALT_PANIC_DESCENT,
                           cmd.twist.linear.x, cmd.twist.linear.y)
        # === 6m 红线二次保险：无条件覆盖，放在 publish 前最后一步 ===
        if self.local_z is not None and self.local_z > ALT_EMERG_CEIL:
            cmd.twist.linear.z = ALT_EMERG_DESCENT
            _alt_guard_hit('EMERG', self.local_z, ALT_EMERG_CEIL, ALT_EMERG_DESCENT,
                           cmd.twist.linear.x, cmd.twist.linear.y)
            cmd.twist.linear.x = cmd.twist.linear.x * ALT_EMERG_HSCALE
            cmd.twist.linear.y = cmd.twist.linear.y * ALT_EMERG_HSCALE
        if getattr(self,'_escape_enabled',False):
            escape = self._current_escape(rospy.Time.now().to_sec())
            if (escape is not None and self.local_z is not None and self.local_z <= ALT_HARD_CEIL
                    and (vz is None or abs(vz) < 1e-9)):
                cmd.twist.linear.z = 0.
                self._escape_altitude_request = escape['local_z']
        # Final horizontal authority: check the command after all direction/
        # acceleration changes, including bounds recovery. Braking must not be
        # undone by the ordinary acceleration limiter on this or the next tick.
        vx, vy = self._radar_guard_velocity(cmd.twist.linear.x, cmd.twist.linear.y)
        if not self._gate.can_move(rospy.Time.now().to_sec()):
            vx, vy = 0., 0.
            self._final_stop_reason = 'AUTHORITY_STOP'
        vx, vy = self._friend_guard_velocity(vx, vy)
        if getattr(self,'_escape_enabled',False):
            vx,vy = self._escape_limit_velocity(vx,vy,rospy.Time.now().to_sec())
        route_gate = getattr(self, '_route_gate', None)
        if route_gate is not None and abs(vx)+abs(vy) > 0:
            vx, vy = self._route_guard_velocity(vx, vy, rospy.Time.now().to_sec())
            if abs(vx)+abs(vy) > 0 and not self._route_velocity_clear(vx, vy, rospy.Time.now().to_sec()):
                rospy.logwarn_throttle(2, '[%s] ROUTE_ENVELOPE_OR_GRANT_UNKNOWN -> horizontal stop', self.uav_id)
                vx, vy = 0., 0.
                self._final_stop_reason = 'ROUTE_ENVELOPE_OR_GRANT_UNKNOWN'
        if getattr(self, '_online_planner', None) is not None and (abs(vx)+abs(vy) > 0):
            if not self._online_velocity_clear(vx,vy):
                rospy.logwarn_throttle(2, '[%s] ONLINE_SPACE_UNKNOWN -> horizontal stop', self.uav_id)
                vx, vy = 0., 0.
                self._final_stop_reason = 'ONLINE_SPACE_UNKNOWN'
        cmd.twist.linear.x, cmd.twist.linear.y = vx, vy
        self._xyz_stop_requested = (abs(vx)+abs(vy) < 1e-9
            and getattr(self,'_takeoff_done',False) and not self._landing
            and self.local_z is not None and self.local_z <= ALT_HARD_CEIL
            and (vz is None or abs(vz) < 1e-9))
        if self._xyz_stop_requested:
            cmd.twist.linear.z = 0.
        if abs(vx)+abs(vy) > 1e-9:
            self._final_stop_reason = 'MOVING'
        if hasattr(self,'_navigation_reason') and (self._navigation_reason != self._final_stop_reason
                or rospy.Time.now().to_sec()-getattr(self,'_navigation_latest_s',-1.) >= .5):
            changed = self._navigation_reason != self._final_stop_reason
            self._navigation_reason = self._final_stop_reason
            if self._navigation_log is None:
                directory = os.environ.get('ROBOCUP_LOG_DIR',os.path.expanduser('~/robocup_logs'))
                os.makedirs(directory,exist_ok=True)
                self._navigation_log = open(os.path.join(directory,'navigation_%s.jsonl' % self.uav_id),'a',encoding='utf-8')
            snapshot = json.dumps(dict(schema_version=1,run_id=self._gate.run_id,
                uav_id=self.uav_id,sample_s=rospy.Time.now().to_sec(),generation=self._gate.generation,
                reason=self._final_stop_reason,command_xyz=[vx,vy,cmd.twist.linear.z],
                requested_xy=requested_xy,route_velocity_evidence=self._route_velocity_evidence,
                route_state=(dict(offer_id=route_gate.record['offer_id'],
                    generation=route_gate.record['generation'],expires_s=route_gate.record['expires_s'],
                    points=route_gate.record['points']) if route_gate is not None and route_gate.record else None),
                route_check=(route_gate.last_check if route_gate is not None else None),
                world_xy=self.world_xy,local_z=self.local_z,
                measured=[v if math.isfinite(v) else None for v in self._velocity_sample] if self._velocity_sample else None,
                motion_evidence=self._measured_motion(rospy.Time.now().to_sec()),
                online_body_proof=self._body_proof_diagnostic(),
                bounded_escape=(dict(generation=self._escape_active['generation'],
                    offer_id=self._escape_active['ticket'],started_s=self._escape_active['started_s'],
                    scan_s=self._escape_active['scan_s'],radius_m=.9,speed_limit_mps=.3,
                    travel_m=self._escape_active['travel_m'],local_z=self._escape_active['local_z'])
                    if getattr(self,'_escape_active',None) is not None else None),
                hold_xyz=self._xyz_stop_requested,scan_window=list(self._scan_window)),allow_nan=False)+'\n'
            if changed:
                self._navigation_log.write(snapshot)
                self._navigation_log.flush()
            # A bounded rolling window also covers contact during an unchanged
            # command. The watchdog preserves this file before owned cleanup.
            latest_path = self._navigation_log.name.replace('.jsonl','_latest.json')
            with open(latest_path+'.tmp','w',encoding='utf-8') as latest_file:
                latest_file.write(snapshot)
            os.replace(latest_path+'.tmp',latest_path)
            self._navigation_latest_s = rospy.Time.now().to_sec()
        self._last_cmd_v = (vx, vy)
        _now = rospy.Time.now().to_sec()
        if _now - self._last_csv_t >= 0.2:
            _target = self.assignment
            _wxy = self.world_xy
            _dist = None
            if _target is not None and _wxy is not None:
                _dist = math.sqrt((_target.target_x - _wxy[0]) ** 2 +
                                  (_target.target_y - _wxy[1]) ** 2 +
                                  (self.local_z or 0.0) ** 2)
            self._csv.write(
                ros_time=_now, event="velocity", target_id=(
                    getattr(_target, "target_id", "") if _target is not None else ""),
                target_x=_target.target_x if _target is not None else None,
                target_y=_target.target_y if _target is not None else None,
                target_z=0.0 if _target is not None else None,
                uav_x=_wxy[0] if _wxy is not None else None,
                uav_y=_wxy[1] if _wxy is not None else None,
                uav_z=self.local_z, distance_m=_dist,
                capture_threshold_m=DETECT_RADIUS,
                is_captured=(_dist is not None and _dist <= DETECT_RADIUS),
                orca_vx=self._last_orca_v[0], orca_vy=self._last_orca_v[1],
                cmd_vx=cmd.twist.linear.x, cmd_vy=cmd.twist.linear.y,
                cmd_vz=cmd.twist.linear.z, source="swarm_agent")
            self._last_csv_t = _now
        # === 2026-09-29 P1-3：坠地检测 ===
        # 原来没有任何姿态/坠机检测，坠机后 landed_state 仍报 IN_AIR，系统完全不知情。
        # 这里不新增订阅，用「实际高度持续远低于目标 + 几乎无水平运动」判定。
        # 起飞前 local_z 通常约为 0，而 target_alt 在 2.8~4.5m；若不要求
        # 已完成垂直爬升，初始悬停会被误报为坠机。
        if CRASH_DETECT and self._takeoff_done and self.local_z is not None:
            try:
                _drop = target_alt - self.local_z
            except Exception:
                _drop = 0.0
            if _drop > CRASH_DROP_M and math.hypot(vx, vy) < CRASH_V_EPS:
                self._crash_cnt = getattr(self, '_crash_cnt', 0) + 1
            else:
                self._crash_cnt = 0
            if (self._crash_cnt >= int(CRASH_HOLD_S * CTRL_RATE)
                    and not getattr(self, '_crash_flagged', False)):
                self._crash_flagged = True
                try:
                    rospy.logerr('[CRASH] %s 疑似坠地 z=%.2f target=%.2f v=%.2f '
                                 '(landed_state 仍可能报 IN_AIR) → 强制爬升恢复',
                                 getattr(self, 'uav_id', getattr(self, 'ns', '?')),
                                 self.local_z, target_alt, math.hypot(vx, vy))
                except Exception:
                    pass
            # 坠毁恢复：一旦 flag，持续以 CRASH_RECOVER_VZ 爬升，直到高度回到
            # target_alt 的 80% 以上再清 flag，避免坠地后继续下发水平速度撞楼。
            if getattr(self, '_crash_flagged', False):
                cmd.twist.linear.z = CRASH_RECOVER_VZ
                cmd.twist.linear.x = 0.0
                cmd.twist.linear.y = 0.0
                if self.local_z >= target_alt * 0.8:
                    self._crash_flagged = False
                    self._crash_cnt = 0
                    rospy.loginfo('[CRASH] %s 高度恢复到 %.2f，清除坠机标志',
                                  getattr(self, 'uav_id', '?'), self.local_z)
        # === 偏航对准（追踪/盘旋）===
        # 机头持续指向 _look_at，保证水平双目在圆形轨迹中不把目标甩出 ±45° 视场。
        # 非追踪阶段 _look_at=None，angular.z 保持 0（由 PX4 自管偏航）。
        _wxy = self.world_xy
        if self._look_at is not None and _wxy is not None:
            _bear = math.atan2(self._look_at[1] - _wxy[1],
                               self._look_at[0] - _wxy[0])
            _yerr = (_bear - self.yaw + math.pi) % (2.0 * math.pi) - math.pi
            _yr = YAW_KP * _yerr
            if _yr > YAW_RATE_MAX:
                _yr = YAW_RATE_MAX
            elif _yr < -YAW_RATE_MAX:
                _yr = -YAW_RATE_MAX
            cmd.twist.angular.z = _yr
        # 记录最终下发的水平速度（供路径未就绪时继续发，保证 offboard 不断流）
        self._last_flight_v = (vx, vy)
        self._publish_command(cmd)

    def _publish_command(self, cmd):
        quality = getattr(self, '_pose_quality', None)
        if quality is not None and not quality.usable(rospy.Time.now().to_sec()):
            # Never hold an old local coordinate after the estimator changed
            # frame. Keep the one publisher alive with a zero velocity request;
            # this is not evidence that the aircraft has physically stopped.
            self._position_brake.anchor = None
            self._position_brake.transform = None
            cmd = TwistStamped()
        position = self.local_xy if (self._pose_sample_s is not None
            and (quality is None or quality.usable(rospy.Time.now().to_sec()))
            and 0 <= rospy.Time.now().to_sec()-self._pose_sample_s <= .5) else None
        fields = self._position_brake.encode((cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z),
                                             cmd.twist.angular.z, position, self.offset,
            hold_altitude=(getattr(self,'_escape_altitude_request',None)
                if getattr(self,'_escape_altitude_request',None) is not None else self.local_z)
                if (getattr(self,'_xyz_stop_requested',False)
                and position is not None) else None,
            moving_altitude=getattr(self,'_escape_altitude_request',None) if position is not None else None)
        message = PositionTarget()
        message.header.stamp = rospy.Time.now()
        message.coordinate_frame, message.type_mask = fields['coordinate_frame'], fields['type_mask']
        message.position.x, message.position.y = fields['position_xy']
        message.position.z = fields['position_z']
        message.velocity.x, message.velocity.y, message.velocity.z = fields['velocity_xyz']
        message.yaw_rate = fields['yaw_rate']
        self.vel_pub.publish(message)

    def _need_replan_track(self, goal):
        """追踪移动目标时的 A* 重规划节流。

        原实现每个控制循环都重跑全图 A*（目标一移动就整条 replan），实测把控制
        频率从 20Hz 拖到 5.5Hz，且日志刷屏（单架曾累计 10020 次规划）。
        这里改成：无路径 / 目标移动超阈值 / 距上次规划超时，三者满足其一才重规划。
        """
        if not self.path:
            return True
        if self._last_track_goal is None:
            return True
        moved = math.hypot(goal[0] - self._last_track_goal[0],
                           goal[1] - self._last_track_goal[1])
        if moved >= TRACK_REPLAN_MOVE:
            return True
        if (rospy.Time.now().to_sec() - self._last_track_plan_t) >= TRACK_REPLAN_SEC:
            return True
        return False

    def _compute_plan(self, goal_xy):
        """从当前世界坐标 A* 规划到 goal_xy。纯计算：返回 (path, goal, ok)，
        不修改 self.path / self.path_target（由后台线程调用，提交交给主循环视角）。"""
        self._computed_escape = None
        if self.world_xy is None:
            return [], goal_xy, False
        if self._online_planner is not None:
            if self._offset_param is None or math.dist(self._offset_param, self._online_planner.seed_xy) > .05:
                raise ValueError('Startup clearance does not match the injected coordinate transform')
            with self._online_map_lock:
                current = self._online_map
                frozen = ObservedMap(current.width, current.height, current.resolution, current.origin)
                frozen.cells, frozen.observed_s = current.cells[:], current.observed_s[:]
                frozen.last_scan_s, frozen.version = current.last_scan_s, current.version
                epoch = self._online_map_epoch_s
            now = rospy.Time.now().to_sec()
            grid = self._online_planner.grid(frozen, self.world_xy, now, epoch)
            with self._authority_lock:
                peer_now = rospy.Time.now().to_sec()
                if self._peer_routes is None or self._peer_routes_s is None or not 0 <= peer_now-self._peer_routes_s <= 1.5:
                    return [], goal_xy, False
                motion = MotionCache(self._motion_cache.run_id, self._motion_cache.fleet)
                motion.samples = dict(self._motion_cache.samples)
                peers = self._peer_routes
            try:
                grid = exclude_peers(grid, self.uav_id, motion, peers, peer_now)
            except ValueError:
                rospy.logwarn_throttle(2, '[%s] ONLINE_PLAN waiting for fresh fleet evidence', self.uav_id)
                return [], goal_xy, False
            result = self._online_planner.route(grid, self.world_xy, goal_xy, frozen, now)
            if result['reason'] == 'START_CLEARANCE_UNKNOWN' and getattr(self,'_escape_enabled',False):
                with self._authority_lock:
                    escape_generation = self._gate.generation
                    eligible = (self._update_escape_rest(rospy.Time.now().to_sec())
                        and escape_generation not in self._escape_used
                        and self._escape_active is None)
                    start = self.world_xy
                    altitude, offset = self.local_z, self.offset
                if eligible:
                    proposal = self._online_planner.exit_candidate(frozen,start,now,epoch,
                        lambda candidate: exclude_peers(candidate,self.uav_id,motion,peers,peer_now))
                    if proposal is not None:
                        proposal.update(generation=escape_generation,local_z=altitude,offset=offset)
                        self._computed_escape = proposal
                        result = dict(ok=True,reason='BOUNDED_ESCAPE_PROPOSAL',points=proposal['points'])
            self.grid = self._online_safe_grid = grid
            self._online_safe_s, self._online_safe_epoch = frozen.last_scan_s, epoch
            rospy.loginfo('[%s] ONLINE_PLAN %s version=%s', self.uav_id, result['reason'], frozen.version)
            self._report_blocked_plan(result['reason'],rospy.Time.now().to_sec())
            return list(result['points']), goal_xy, result['ok']
        route = plan(self.grid, self.world_xy, goal_xy, connectivity=8)
        if not route.success and route.reason == "START_OCCUPIED":
            # 起点落在膨胀障碍内（刚起飞/贴墙）→ 找最近自由栅格重试
            nearest = self._nearest_free_cell()
            if nearest is not None:
                route = plan(self.grid, nearest, goal_xy, connectivity=8)
        if not route.success and route.reason == "GOAL_OCCUPIED":
            # 目标观测可能落在建筑边缘或膨胀区；停在目标附近的自由点，
            # 不要把一个不可站立的观测点变成整条追踪路径的失败。
            nearest_goal = self._nearest_free_cell(goal_xy)
            if nearest_goal is not None:
                route = plan(self.grid, self.world_xy, nearest_goal, connectivity=8)
        if not route.success and PLAN_FALLBACK:
            # 2026-09-28：START_OUT_OF_BOUNDS 会让飞机永久停摆。实测 uav_1 被推到
            # y≈69（A* 栅格 y_max=65 之外）后每 2s 规划失败一次、速度指令全程 0，
            # 整轮只飞 10m；6 架里 4 架这样趴着 → 全机均速 0.47 m/s、扫描率只有
            # 理论值的 1/8。越界/无解也要动起来：先把起点拉回最近自由栅格重试。
            nearest = self._nearest_free_cell(goal_xy)
            if nearest is not None:
                route = plan(self.grid, nearest, goal_xy, connectivity=8)
        if not route.success and PLAN_FALLBACK and self.world_xy is not None:
            # 还不行就退化成直飞航点串。可能穿楼，但原地不动是 100% 无收益，
            # 而且穿楼前还过一道建筑避障 + 高度护栏。
            x0, y0 = self.world_xy
            dx, dy = goal_xy[0] - x0, goal_xy[1] - y0
            d = math.hypot(dx, dy)
            n = max(2, int(d / 3.0))
            path = [(x0 + dx * (i + 1) / float(n),
                     y0 + dy * (i + 1) / float(n)) for i in range(n)]
            self._plan_fallback_n += 1
            rospy.logwarn_throttle(10, "[%s] A* 失败(%s) → 直飞 (%.1f,%.1f)，累计 %d 次",
                                   self.uav_id, route.reason, goal_xy[0], goal_xy[1],
                                   self._plan_fallback_n)
            return path, goal_xy, True
        if not route.success:
            rospy.logwarn("[%s] A* 规划到 (%.1f,%.1f) 失败: %s",
                          self.uav_id, goal_xy[0], goal_xy[1], route.reason)
            return [], goal_xy, False
        path = connect_exact_goal(smooth_path(route.points, samples_per_segment=6), goal_xy, self.grid)
        rospy.loginfo("[%s] A* 规划：%d 航点 -> %d 平滑点",
                      self.uav_id, len(route.points), len(path))
        return path, goal_xy, True

    def _report_blocked_plan(self, reason, now):
        position, evidence = self.world_xy, self._measured_motion(now)
        if (reason not in ('START_CLEARANCE_UNKNOWN','NO_REACHABLE_PROGRESS','TARGET_VISUAL_LOST')
                or position is None or evidence is None or evidence['speed_mps'] > .15
                or not self._gate.can_move(now)):
            self._blocked_plan = None
            return
        old = self._blocked_plan
        if old is None or old[2] != self._gate.generation or math.dist(old[1],position) > .5:
            self._blocked_plan = (now,position,self._gate.generation)
            return
        if now-old[0] >= 6.:
            self._blocked_feedback_seq += 1
            self._navigation_pub.publish(String(data=json.dumps(dict(schema_version=1,
                run_id=self._gate.run_id,uav_id=self.uav_id,generation=self._gate.generation,
                seq=self._blocked_feedback_seq,sample_s=now,blocked_since_s=old[0],
                position_xy=list(position),reason=reason),allow_nan=False)))

    def _fly_companion(self, now, target_id):
        record = self.targets.get(target_id)
        stamp = self._t_seen.get(target_id)
        if record is None or stamp is None or not self._gate.can_move(now):
            self._send_vel(0.,0.)
            return
        guidance = companion_guidance(self.world_xy,record[:2],record[2:4],now-stamp,
                                      maximum_speed=min(MAX_SPEED,FLEE_CHASE_SPEED))
        if guidance is None:
            self._send_vel(0.,0.)
            return
        self._look_at = guidance['look_at']
        if guidance['speed_mps'] < .05:
            self._send_vel(0.,0.)
            return
        goal = guidance['goal']
        if self._need_replan_track(goal) and now-self._plan_fail_t >= 2.:
            self._request_plan(goal)
        local_goal = self._pick_local_goal()
        if local_goal is None:
            self._send_vel(0.,0.)
            return
        vx,vy = POS_KP*(local_goal[0]-self.world_xy[0]),POS_KP*(local_goal[1]-self.world_xy[1])
        speed = math.hypot(vx,vy)
        cap = guidance['speed_mps']
        if speed > cap:
            vx,vy = vx*cap/speed,vy*cap/speed
        self._send_vel(*self._apply_friend_avoidance(vx,vy))

    def _control_tracking_navigation(self, now, target_id, in_cooldown):
        navigation = getattr(self, '_tracking_navigation', None)
        if navigation is None or in_cooldown:
            return False
        record = self.targets.get(target_id)
        point = record[:2] if record is not None else None
        white = getattr(self, '_white_reacquire', None)
        allow_scan = not (target_id == TAG_TO_TID['white'] and white is not None and white.used)
        phase, destination = navigation.step(self._gate.generation, target_id, now,
            self.world_xy, point, self._t_seen.get(target_id),
            eligible=self._gate.can_move(now), allow_scan=allow_scan)
        if phase != self._tracking_navigation_phase:
            self._blocked_plan = None
            rospy.loginfo('[%s] TRACK_NAVIGATION %s target=%s generation=%s original_s=%s',
                self.uav_id, phase, target_id, self._gate.generation, self._t_seen.get(target_id))
            self._tracking_navigation_phase = phase
        if phase == 'CURRENT':
            return False
        self._look_at = point
        if phase == 'APPROACH':
            # An old observation is only a bounded navigation hint. Every
            # command still passes actual route/map/lidar/fleet gates.
            if self._need_replan_track(destination) and now-self._plan_fail_t >= 2.:
                self._request_plan(destination)
            # A previously committed route may still aim at the person's old
            # position. Do not use it to pass through the observation standoff.
            committed_target = getattr(self, 'path_target', None)
            if committed_target is None or math.dist(committed_target, destination) > .75:
                self._send_vel(0., 0.)
                return True
            local_goal = self._pick_local_goal()
            if local_goal is None:
                self._send_vel(0., 0.)
                return True
            vx = POS_KP*(local_goal[0]-self.world_xy[0])
            vy = POS_KP*(local_goal[1]-self.world_xy[1])
            speed = math.hypot(vx, vy)
            cap = min(MAX_SPEED, FLEE_CHASE_SPEED)
            if speed > cap:
                vx, vy = vx*cap/speed, vy*cap/speed
            self._send_vel(*self._apply_friend_avoidance(vx, vy))
            return True
        if phase == 'REACQUIRE':
            self._look_at = destination
            if target_id == TAG_TO_TID['white'] and white is not None:
                white.used = True  # One opportunity shared with white's existing scan.
        self._send_vel(0., 0.)
        if phase == 'FAILED':
            self._report_blocked_plan('TARGET_VISUAL_LOST', now)
        return True

    def _control_white_reacquire(self, now, target_id, in_cooldown):
        eligible = (target_id == TAG_TO_TID['white'] and not in_cooldown
                    and self._gate.can_move(now))
        state, point = self._white_reacquire.step(self._gate.generation, now, self.world_xy, eligible)
        if state != self._white_reacquire_phase:
            if state == 'FAILED':
                self._blocked_plan = None
            rospy.loginfo('[%s] WHITE_REACQUIRE %s generation=%s original_s=%s',
                self.uav_id, state, self._gate.generation, self._white_reacquire.sample_s)
            self._white_reacquire_phase = state
        if state == 'RESUMED':
            self._target_to_orbit = point
            self._blocked_plan = None
        if state in ('SCANNING', 'FAILED'):
            self._look_at = point
            self._send_vel(0., 0.)
            if state == 'FAILED':
                self._report_blocked_plan('TARGET_VISUAL_LOST', now)
            return True
        return False

    # ---------------- 异步规划接口 ----------------
    def _request_plan(self, goal_xy):
        """异步投递；追踪同代次先完成在途规划，下一次再取最新相机目标。"""
        with self._authority_lock, self._plan_lock:
            if (getattr(self,'_escape_enabled',False)
                    and self._current_escape(rospy.Time.now().to_sec()) is not None):
                return
            candidate = (float(goal_xy[0]), float(goal_xy[1]))
            generation = self._gate.generation
            tracking = self._gate.task is not None and self._gate.task['task_type'] == 1
            pending = self._route_pending
            if (pending is not None and (tracking or pending[2] == candidate) and pending[3] == generation
                    and 0 <= rospy.Time.now().to_sec()-self._route_pending_s < 1.):
                return
            if any(request is not None and request[1] == generation
                   and (tracking or request[0] == candidate)
                   for request in (self._plan_pending, self._plan_inflight)):
                return
            self._plan_ticket += 1
            self._plan_pending = (candidate, generation, self._plan_ticket)
            if tracking:
                # Coalesced camera updates did not start a new calculation;
                # only an enqueued goal may advance the replan throttle.
                self._last_track_goal = candidate
                self._last_track_plan_t = rospy.Time.now().to_sec()
        self._plan_event.set()

    def _planner_loop(self):
        """后台守护线程：取最新请求 → 规划 → 原子提交 self.path；失败记录冷却时刻。"""
        while not rospy.is_shutdown():
            self._plan_event.wait()
            with self._plan_lock:
                request = self._plan_pending
                if request is None:
                    self._plan_event.clear()
                    continue
                self._plan_pending = None
                self._plan_event.clear()
                self._plan_inflight = request
            goal, generation, ticket = request
            started_s = rospy.Time.now().to_sec()
            try:
                path, target, ok = self._compute_plan(goal)
            except Exception as exc:
                rospy.logerr("[%s] 规划线程异常: %s\n%s",
                             self.uav_id, exc, traceback.format_exc())
                ok = False
            # Preserve the same lock order as authority callbacks/control. There
            # must be no idle gap between inflight and the route offer: a new
            # camera point in that gap would cancel the completed computation.
            with self._authority_lock, self._plan_lock:
                if self._plan_inflight == request:
                    self._plan_inflight = None
                if (generation != self._gate.generation or ticket != self._plan_ticket
                        or not self._gate.can_move(rospy.Time.now().to_sec())):
                    reason = ('GENERATION_CHANGED' if generation != self._gate.generation else
                              'SUPERSEDED_TICKET' if ticket != self._plan_ticket else
                              'AUTHORITY_STOPPED_OR_EXPIRED')
                    rospy.loginfo('[%s] PLAN_RESULT_DISCARDED generation=%s ticket=%s reason=%s',
                                  self.uav_id, generation, ticket, reason)
                    continue
                if ok:
                    if self.world_xy is None:
                        continue
                    path = [list(self.world_xy)] + [list(p) for p in path]
                    proposal = getattr(self,'_computed_escape',None)
                    if proposal is not None:
                        if (proposal['generation'] != generation
                                or not self._update_escape_rest(rospy.Time.now().to_sec())
                                or not self._escape_path_valid(path,proposal)):
                            self._plan_fail_t = rospy.Time.now().to_sec()
                            continue
                        proposal['ticket'] = ticket
                    self._escape_pending = proposal
                    self._route_pending = (ticket, path, target, generation)
                    self._route_pending_s = rospy.Time.now().to_sec()
                    rospy.loginfo('[%s] PLAN_OFFERED generation=%s ticket=%s elapsed_s=%.3f',
                                  self.uav_id, generation, ticket, self._route_pending_s-started_s)
                    self._route_offer_pub.publish(String(data=json.dumps(dict(schema_version=1,
                        run_id=self._gate.run_id, uav_id=self.uav_id, generation=generation,
                        offer_id=ticket, points=path), allow_nan=False)))
                else:
                    _wxy = self.world_xy
                    _out = (_wxy is not None and self.grid.world_to_cell(_wxy) is None)
                    if not self.path or _out:
                        self.path = []
                        self._last_flight_v = (0.0, 0.0)
                    self._plan_fail_t = rospy.Time.now().to_sec()

    def _stream_last_v(self):
        """规划未就绪时的兜底：继续发上一次水平速度（无则悬停）。
        PX4 要求 offboard setpoint 以约 20Hz 持续到达，中断 >COM_OF_LOSS_T（默认~1s）
        即判 offboard 丢失 → failsafe。任何路径空档都必须这样把指令流续上。"""
        if self._last_flight_v is None:
            self._send_vel(0.0, 0.0)
        else:
            self._send_vel(self._last_flight_v[0], self._last_flight_v[1])

    def _nearest_free_cell(self, center_xy=None):
        """在给定点附近找最近自由栅格，避免全地图 O(w*h) 扫描。"""
        center_xy = center_xy if center_xy is not None else self.world_xy
        if center_xy is None:
            return None
        cx, cy = center_xy
        # 以当前坐标为圆心，半径 5m 内找自由栅格（膨胀后建筑间距 > 5m）
        radius_cells = int(math.ceil(5.0 / self.grid.resolution))
        center_cell = self.grid.world_to_cell((cx, cy))
        if center_cell is None:
            return None
        best, best_d = None, float("inf")
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                cell = (center_cell[0] + dx, center_cell[1] + dy)
                if not self.grid.is_free(cell):
                    continue
                wx, wy = self.grid.cell_to_world(cell)
                d = math.hypot(wx - cx, wy - cy)
                if d < best_d:
                    best_d = d
                    best = (wx, wy)
        return best

    def _pick_local_goal(self):
        """沿全局路径从最近点往前取 LOOKAHEAD 距离的引导目标。"""
        if not self.path or self.world_xy is None:
            return None
        escape = self._current_escape(rospy.Time.now().to_sec()) if getattr(self,'_escape_enabled',False) else None
        if not self.path:
            return None
        safe_grid = escape['grid'] if escape is not None else getattr(self,'_online_safe_grid',None)
        route_gate = getattr(self, '_route_gate', None)
        connector_guard = (lambda start, end: route_gate.connector_clear(start, end,
            rospy.Time.now().to_sec(), self._gate.generation)) if route_gate is not None else None
        cx, cy = self.world_xy
        best = 0
        best_dist = float("inf")
        for i, (px, py) in enumerate(self.path):
            d = math.hypot(px - cx, py - cy)
            if d < best_dist:
                best_dist = d
                best = i
        acc = 0.0
        for i in range(best, len(self.path) - 1):
            x0, y0 = self.path[i]
            x1, y1 = self.path[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if acc + seg >= LOOKAHEAD:
                frac = (LOOKAHEAD - acc) / max(seg, 1e-6)
                candidate = (x0 + (x1 - x0) * frac, y0 + (y1 - y0) * frac)
                planner = getattr(self, '_online_planner', None)
                if planner is not None:
                    return planner.visible_goal(safe_grid,self.world_xy,
                                                self.path[best:i+1]+[candidate],connector_guard)
                return candidate
            acc += seg
        planner = getattr(self, '_online_planner', None)
        if planner is not None:
            return planner.visible_goal(safe_grid,self.world_xy,self.path[best:],connector_guard)
        return self.path[-1]

    def _control(self):
        if (getattr(self, '_online_planner', None) is not None
                and self._gate.task is not None and self._gate.task['task_type'] in (2, 3)):
            self._look_at = None
            self._send_vel(0., 0.)
            return
        if not self._gate.can_move(rospy.Time.now().to_sec()):
            if getattr(self, '_white_reacquire', None) is not None:
                self._white_reacquire.cancel()
            view = getattr(self, '_stopped_observation', None)
            quality = getattr(self, '_pose_quality', None)
            now = rospy.Time.now().to_sec()
            blocked = [tid for tid, until in getattr(self, '_giveup_until', {}).items() if not math.isfinite(until)]
            self._look_at = (view.look(now, self._gate.stopping, self._gate.generation, blocked,
                eligible=self.world_xy is not None and getattr(self, '_takeoff_done', False)
                and not getattr(self, '_landing', False)
                and (quality is None or quality.usable(now))) if view is not None else None)
            if view is not None:
                selected = view.selected if self._look_at is not None else None
                state = (self._gate.generation, selected)
                if state != getattr(self, '_stopped_observation_phase', None):
                    self._stopped_observation_phase = state
                    rospy.loginfo('[%s] STOP_OBSERVATION target=%s generation=%s original_s=%s',
                        self.uav_id, selected, self._gate.generation,
                        view.images[selected][0] if selected is not None else None)
            self._send_vel(0., 0.)
            return
        """有任务 → A* 绕障飞向格中心；无任务 → 原地悬停。"""
        # === 降落模式 ===
        if self._landing:
            self._look_at = None
            if self.local_z is not None and self.local_z < 0.3:
                # 触地，解锁
                rospy.loginfo("[%s] 已触地，解锁", self.uav_id)
                try:
                    self.arm_srv(False)
                except Exception as e:
                    rospy.logwarn("[%s] 解锁失败: %s", self.uav_id, e)
                self._landing = False
                return
            else:
                # 持续发 vz = -1.0 下降，不依赖模式切换
                self._send_vel(0.0, 0.0, vz=-1.0)
                return

        # === 起飞垂直爬升门（2026-10-01 修复趴地起飞）===
        # 未爬到 CLIMB_DONE_ALT 之前，无论派到什么任务都只准垂直爬升，水平速度锁零。
        # 重启协同时若已在此高度之上（_arm_and_offboard 早退路径），首轮即放行。
        if not self._takeoff_done:
            if self.local_z is not None and self.local_z >= CLIMB_DONE_ALT:
                self._takeoff_done = True
                rospy.loginfo("[%s] 已爬升到 %.1fm，开始水平机动",
                              self.uav_id, CLIMB_DONE_ALT)
            else:
                self._send_vel(0.0, 0.0)
                return

        if self.world_xy is None:
            self._send_vel(0.0, 0.0)
            return

        if self.assignment is None:
            self.path = []
            self.path_target = None
            self._orbit_target = None
            self._target_to_orbit = None
            self._look_at = None
            self._send_vel(0.0, 0.0)
            return

        if (getattr(self,'_escape_enabled',False)
                and self._control_escape(rospy.Time.now().to_sec())):
            return

        # === 任务类型处理 ===
        task_type = getattr(self.assignment, 'task_type', 0)

        # 目标追踪任务（task_type=1）：飞向目标位置
        if task_type == 1:
            self._resume_tracking_attempt()
            # 冷却闸门：目标在 giveup 冷却内，或已 stale（无位置更新），
            # 安全悬停等待 manager 改派，绝不重入盘旋或朝过期点飞。
            _track_id = getattr(self.assignment, 'target_id', None)
            _in_cooldown = (_track_id and
                            rospy.Time.now().to_sec() < self._giveup_until.get(_track_id, 0.0))
            if (getattr(self, '_white_reacquire_enabled', False)
                    and self._control_white_reacquire(rospy.Time.now().to_sec(), _track_id, _in_cooldown)):
                return
            if (getattr(self, '_tracking_navigation', None) is not None
                    and self._control_tracking_navigation(rospy.Time.now().to_sec(), _track_id, _in_cooldown)):
                return
            if _in_cooldown or self._orbit_stale() or self._target_to_orbit is None:
                # Backoff stops translation, not observation. Never fall through
                # to search with a retained orbit route after _abort_orbit.
                self._look_at = (current_target_point(self.targets, self._t_seen,
                    _track_id, rospy.Time.now().to_sec())
                    if math.isfinite(self._giveup_until.get(_track_id, 0.)) else None)
                # A stopped handoff can outlast the 1s image gate. A new,
                # still-pending attempt may aim at its last actual camera point
                # for up to 5s to reacquire; this cannot clear cooldown, move,
                # publish an observation or refresh its acquisition timestamp.
                if (self._look_at is None and
                        getattr(self, '_tracking_retry_pending', None) ==
                        (_track_id, self._gate.generation) and
                        math.isfinite(self._giveup_until.get(_track_id, 0.))):
                    self._look_at = current_target_point(self.targets, self._t_seen,
                        _track_id, rospy.Time.now().to_sec(), maximum_age=5.)
                if getattr(self,'_v124_behavior',False):
                    self._look_at = None
                    if getattr(self, '_handoff_yaw_reacquire', False):
                        now = rospy.Time.now().to_sec()
                        self._look_at = handoff_reacquisition_point(self.targets, self._t_seen,
                            _track_id, self._gate.generation,
                            getattr(self, '_handoff_reacquire_window', None), now,
                            self._gate.can_move(now), self._giveup_until.get(_track_id, 0.))
                self._send_vel(0.0, 0.0)
                return
            # Fresh guidance has resumed: this grant's one yaw opportunity ends.
            self._handoff_reacquire_window = None
            if getattr(self,'_companion_tracking_enabled',False):
                self._fly_companion(rospy.Time.now().to_sec(),_track_id)
                return
            tx, ty = self._target_to_orbit
            self._look_at = (tx, ty)       # 接近阶段机头就对准目标
            dist = math.hypot(tx - self.world_xy[0], ty - self.world_xy[1])
            if dist < ORBIT_RADIUS:
                # 到达目标附近，开始盘旋
                self._orbit_center = (tx, ty)
                # 存真实 target_id
                target_id = getattr(self.assignment, 'target_id', None)
                if target_id in ('t0','t1','t2','t3','t4','t5') and self._orbit_target != target_id:
                    # Each target retry gets a new confirmation attempt;
                    # historical resets must not end a new task after 1 frame.
                    if not getattr(self,'_v124_behavior',False):
                        self._reset_n[int(target_id[1:])] = 0
                self._orbit_target = target_id if target_id else "tracking"
                self._confirm_start = rospy.Time.now().to_sec()
                self._last_confirm_t = self._confirm_start
                # 不要清除 _target_to_orbit，_fly_orbit 需要用
                self._fly_orbit()
                return
            else:
                # 飞向目标（移动目标：A* 重规划节流）。异步投递，绝不阻塞指令流。
                if self._need_replan_track((tx, ty)):
                    if rospy.Time.now().to_sec() - self._plan_fail_t >= 2.0:
                        self._request_plan((tx, ty))
                local_goal = self._pick_local_goal()
                if local_goal is None:
                    # 路径尚未提交：续发上一次速度，等规划线程把路径发下来
                    self._stream_last_v()
                    return
                err_x = local_goal[0] - self.world_xy[0]
                err_y = local_goal[1] - self.world_xy[1]
                vx = POS_KP * err_x
                vy = POS_KP * err_y
                spd = math.hypot(vx, vy)
                # Reporting can trigger 2m/s flight before motion classification
                # catches up. Radar, reservation and braking still gate the command.
                cap = min(MAX_SPEED, FLEE_CHASE_SPEED)
                if spd > cap:
                    vx *= cap / spd
                    vy *= cap / spd
                vx, vy = self._apply_friend_avoidance(vx, vy)
                self._send_vel(vx, vy)
                return

        # === 搜索任务（task_type=0）===
        # 检查是否在盘旋，以及是否能看到目标
        self._update_orbit()
        if self._orbit_target is not None:
            # 正在盘旋确认，执行盘旋飞行
            self._fly_orbit()
            return
        self._look_at = None   # 未在盘旋/追踪，交回 PX4 自管偏航

        if getattr(self, '_search_enabled', False):
            if self._control_search_view(rospy.Time.now().to_sec()):
                return
            goal = self._search_sweep.goal
        else:
            goal = (self.assignment.target_x, self.assignment.target_y)
        if self._online_planner is not None and rospy.Time.now().to_sec()-self._last_online_request_s >= 1.:
            self._request_plan(goal)
            self._last_online_request_s = rospy.Time.now().to_sec()
        dist = math.hypot(goal[0] - self.world_xy[0], goal[1] - self.world_xy[1])

        if dist < ARRIVE_TOL:
            # 已到达格中心，检查是否有可盘旋的目标
            self.path = []
            self._check_start_orbit()
            if self._orbit_target is not None:
                self._fly_orbit()
            else:
                self._send_vel(0.0, 0.0)
            return

        # 目标变了 → 异步投递规划（非阻塞；失败后按 _plan_fail_t 冷却 2s 再试）
        if self.path_target != goal:
            if rospy.Time.now().to_sec() - self._plan_fail_t >= 2.0:
                self._request_plan(goal)

        # 沿当前路径继续飞：规划窗口内沿用旧路径（27~74ms，位移可忽略），提交后
        # 自动切到新目标；路径为空（首次起飞/刚换格）时续发上一次速度，指令不断。
        local_goal = self._pick_local_goal()
        if local_goal is None:
            self._stream_last_v()
            return
        self._look_at = local_goal
        err_x = local_goal[0] - self.world_xy[0]
        err_y = local_goal[1] - self.world_xy[1]
        vx = POS_KP * err_x
        vy = POS_KP * err_y
        spd = math.hypot(vx, vy)
        if spd > MAX_SPEED:
            vx *= MAX_SPEED / spd
            vy *= MAX_SPEED / spd
        # === 友机避碰 ===
        vx, vy = self._apply_friend_avoidance(vx, vy)

        self._send_vel(vx, vy)

    def _nearest_target_dist(self):
        """当前已知目标中距本机的最小水平距离；无目标/无定位时返回 None。"""
        wx = self.world_xy
        if wx is None or not self.targets:
            return None
        best = None
        for tid, (tx, ty, _, _) in self.targets.items():
            d = math.hypot(tx - wx[0], ty - wx[1])
            if best is None or d < best[1]:
                best = (tid, d)
        return best

    def _target_fleeing(self, tid):
        """目标是否处于 FLEE（state=1）且状态信息新鲜。"""
        if not tid:
            return False
        ent = self._target_state.get(str(tid))
        if ent is None:
            return False
        state, t = ent
        return state == 1 and \
            0 <= (rospy.Time.now().to_sec() - t) <= FLEE_STATE_FRESH

    # ---- 已消除目标清理 / 盘旋放弃 / 防扎堆（2026-09-27 新增）----
    def _left_actors_cb(self, msg):
        """官方 /left_actors：不在清单里的 actor = 已被官方确认并删除。

        这是「飞机被已消除目标占死」的权威兜底。official_target_bridge 只转发
        model_states，actor 被 del_model 后它就不再发该目标，而它发的
        TargetState.eliminated 恒为 False —— 于是 agent 的 self.targets 永不清理，
        _update_orbit 的「目标已消失」分支永远不触发，飞机对着一个已经不存在的
        actor 盘旋到超时（实测 uav_5/uav_6 对 t1 盘旋 571.3s，4/6 架机全程空转）。
        """
        parsed = parse_actor_list(str(msg.data))
        if parsed is None or (not parsed and not self._left_seen):
            return
        ids = set(parsed)
        self._left_seen = True
        for tid in list(self.targets.keys()):
            m = re.match(r'^t(\d+)$', str(tid))
            if m is None:
                continue
            # Red task IDs are geometric camera slots, not official actor IDs.
            # Neither slot is retired while either official red actor remains.
            if not actor_slot_remaining(int(m.group(1)), ids):
                self.targets.pop(tid, None)
                self._t_seen.pop(tid, None)
                self._giveup_until[tid] = float('inf')   # 已消除，永不追
                if self._orbit_target == tid:
                    self._abort_orbit('%s 已被官方消除，回归搜索' % tid)

    def _abort_orbit(self, reason, clear_path=True):
        """退出盘旋；目标授权期间停控，等待新证据或管理器交接。

        清空 _target_to_orbit 防止执行旧的追踪点。目标任务仍进入自己的
        停控处理，不能因为追踪点为空就续发搜索或旧盘旋路线。

        clear_path=False（stale/重置退避场景）：保留 self.path，避免
        _need_replan_track 因“空路径”恒为真，在冷却期每帧重跑 A* 形成
        规划风暴（目标点没动，结果完全相同，纯浪费）。
        """
        tid = self._orbit_target
        self._orbit_target = None
        self._orbit_center = None
        self._confirm_start = 0.0
        self._target_to_orbit = None
        if clear_path:
            self.path = []
            self.path_target = None
        self._look_at = None
        rospy.logwarn('[%s] 放弃盘旋 %s：%s', self.uav_id, tid, reason)

    def _claim_cb(self, msg):
        """别机广播「我正在确认 tid」。"""
        try:
            uid, tid = str(msg.data).split(':', 1)
        except ValueError:
            return
        if uid != self.uav_id:
            self._claims[tid] = (uid, rospy.Time.now().to_sec())

    def _find_cb(self, msg, actor_idx):
        """官方 /find_actor_N：时间戳每变一次 = 官方刚刚重置了一次 15s 确认。

        这是团队侧唯一能观测到「确认又断了」的权威信号。官方 _reset_detection()
        把 count_flag 清零、find_time 归零，下一次有效上报又会
        count_flag False->True 并重新 publish 一个新的时间戳。
        """
        t = float(msg.data)
        old = self._find_t.get(actor_idx)
        if old is not None and abs(t - old) < 1e-6:
            return
        self._find_t[actor_idx] = t
        if old is None:
            return                          # 首次发现，不算重置
        self._reset_n[actor_idx] = self._reset_n.get(actor_idx, 0) + 1
        self._reset_t[actor_idx] = rospy.Time.now().to_sec()
        tid = 't%d' % actor_idx
        rospy.logwarn('[%s] 官方重置了 %s 的确认（第 %d 次）',
                      self.uav_id, tid, self._reset_n[actor_idx])
        if self._orbit_target != tid:
            return
        # 镜像官方窗口：把自己的确认计时也归零，ORBIT_GIVEUP 的语义才成立
        self._confirm_start = rospy.Time.now().to_sec()
        self._last_confirm_t = self._confirm_start
        if BACKOFF_ENABLE and self._reset_n[actor_idx] >= CONFIRM_RESET_MAX:
            self._giveup_until[tid] = rospy.Time.now().to_sec() + BACKOFF_COOLDOWN
            self._abort_orbit('官方已重置 %d 次确认，退避 %.0fs 去搜别处'
                              % (self._reset_n[actor_idx], BACKOFF_COOLDOWN),
                              clear_path=False)

    def _publish_claim(self):
        """每 0.5s 广播一次自己正在确认的目标，供别机避让（防多机扎堆）。"""
        if not CLAIM_ENABLE or self._orbit_target is None:
            return
        now = rospy.Time.now().to_sec()
        if now - self._last_claim_t < 0.5:
            return
        self._last_claim_t = now
        try:
            self._claim_pub.publish('%s:%s' % (self.uav_id, self._orbit_target))
        except Exception:
            pass

    def _claimed_by_other(self, tid):
        """返回正在确认 tid 的别机 id（无人认领或已过期则返回 None）。"""
        if not CLAIM_ENABLE:
            return None
        ent = self._claims.get(tid)
        if ent is None:
            return None
        uid, t = ent
        if rospy.Time.now().to_sec() - t > CLAIM_FRESH:
            return None
        return uid

    def _orbit_stale(self):
        """Bound guidance coasting without refreshing the original image time."""
        tid = self._orbit_target or getattr(self.assignment, 'target_id', None)
        if tid is None:
            return False
        t_seen = self._t_seen.get(tid, 0.0)
        if not t_seen:
            return True
        age = rospy.Time.now().to_sec() - t_seen
        return not 0 <= age <= TRACK_GUIDANCE_TTL

    def _update_orbit(self):
        """更新盘旋状态：检查目标是否可见，更新确认时间"""
        if self._orbit_target is None:
            return

        wx = self.world_xy
        if wx is None:
            return

        tid = self._orbit_target
        if tid not in self.targets:
            # 目标已消失，停止盘旋
            self._orbit_target = None
            self._orbit_center = None
            return

        tx, ty, _, _ = self.targets[tid]
        dist = math.hypot(tx - wx[0], ty - wx[1])

        now = rospy.Time.now().to_sec()
        # 检查是否能看到目标（距离 + LOS）
        if dist < DETECT_RADIUS and self.los.visible(wx[0], wx[1], tx, ty):
            if self._confirm_start == 0.0:
                self._confirm_start = now
            self._last_confirm_t = now
        else:
            # 看不到目标，重置确认
            self._confirm_start = 0.0

    def _check_start_orbit(self):
        """检查是否需要开始盘旋（发现可确认的目标）"""
        # A search assignment is not a target lock. Discovery is reported to
        # manager; only a current target grant permits autonomous confirmation.
        if (not self._gate.can_move(rospy.Time.now().to_sec()) or not self._gate.task
                or self._gate.task['task_type'] != 1):
            return
        wx = self.world_xy
        if wx is None:
            return

        for tid, (tx, ty, _, _) in self.targets.items():
            if tid != self._gate.task['target_id']:
                continue
            dist = math.hypot(tx - wx[0], ty - wx[1])
            # 在检测范围内且 LOS 可见
            if dist >= DETECT_RADIUS or not self.los.visible(wx[0], wx[1], tx, ty):
                continue
            # 放弃冷却期内不再自动追（刚放弃过的目标，别立刻又贴回去）
            if rospy.Time.now().to_sec() < self._giveup_until.get(tid, 0.0):
                continue
            # 距上次官方重置已久 → 清零计数（误差是时变的，回来值得再试一次）
            _m = re.match(r'^t(\d+)$', str(tid))
            if _m is not None:
                _i = int(_m.group(1))
                if rospy.Time.now().to_sec() - self._reset_t.get(_i, 0.0) > RESET_DECAY:
                    self._reset_n[_i] = 0
            # 别机已经在确认这个目标 → 不扎堆，继续搜自己的格
            _other = self._claimed_by_other(tid)
            if _other is not None:
                continue
            # 开始盘旋确认
            self._orbit_target = tid
            self._orbit_center = (tx, ty)
            self._confirm_start = rospy.Time.now().to_sec()
            self._last_confirm_t = self._confirm_start
            rospy.loginfo("[%s] 开始盘旋确认目标 %s", self.uav_id, tid)
            break

    def _fly_orbit(self):
        """执行盘旋飞行：绕目标做圆周运动"""
        if self._orbit_target is None:
            self._send_vel(0.0, 0.0)
            return

        wx, wy = self.world_xy
        if wx is None:
            return

        # 目标长时间没有位置更新才判定已删除；短时遮挡要给 bridge/备份机接力窗口。
        if self._orbit_stale():
            tid0 = self._orbit_target
            gap = rospy.Time.now().to_sec() - self._t_seen.get(tid0, 0.0)
            self.targets.pop(tid0, None)
            self._giveup_until[tid0] = rospy.Time.now().to_sec() + GIVEUP_COOLDOWN
            # 保留路径：目标点未变，重算结果完全相同，纯浪费 CPU。
            self._abort_orbit('已 %.1fs 无位置更新，暂退避等待接力' % gap,
                              clear_path=False)
            return

        # 用最新目标位置更新盘旋中心
        if self._orbit_target == "tracking" and self._target_to_orbit is not None:
            tx, ty = self._target_to_orbit
        elif self._orbit_target in self.targets:
            # 从已知目标获取最新位置
            tx, ty, _, _ = self.targets[self._orbit_target]
        else:
            # 无目标位置，原地悬停
            self._send_vel(0.0, 0.0)
            return

        self._orbit_center = (tx, ty)
        self._look_at = (tx, ty)       # 盘旋全程机头持续指向目标中心
        now = rospy.Time.now().to_sec()

        # Every orbit leg follows the same observed-space planner and serial
        # route authority as search. A raw circle velocity has no reservation.
        if self._online_planner is not None:
            previous = getattr(self, '_reserved_orbit_goal', None)
            changed = previous is None or math.dist(previous[0], (tx, ty)) > .5
            arrived = previous is not None and math.dist(previous[1], (wx, wy)) < .5
            if changed or arrived:
                chord = getattr(self, '_orbit_plan_chord_m', 0.)
                if chord > 0.:
                    motion = self._measured_motion(now)
                    candidates = motion['velocity_candidates'] if motion is not None else ()
                    target_velocity = self.targets.get(self._orbit_target, (tx, ty, 0., 0.))[2:4]
                    point = orbit_goal((wx, wy), (tx, ty), ORBIT_RADIUS, chord,
                                       candidates, target_velocity)
                else:
                    angle = math.atan2(wy-ty, wx-tx) + .2
                    point = (tx+ORBIT_RADIUS*math.cos(angle), ty+ORBIT_RADIUS*math.sin(angle))
                self._reserved_orbit_goal = ((tx, ty), point)
            point = self._reserved_orbit_goal[1]
            if now-getattr(self, '_last_orbit_request_s', -100.) >= 1.:
                self._request_plan(point)
                self._last_orbit_request_s = now
            # A moving target changes the next requested endpoint before its
            # replacement route is granted. Keep following the committed route
            # in this task generation; _send_vel still enforces its live grant.
            # STOP and generation changes clear path_target in _authorized_cb.
            local_goal = self._pick_local_goal() if self.path_target is not None else None
            if local_goal is None:
                self._send_vel(0., 0.)
                return
            vx, vy = POS_KP*(local_goal[0]-wx), POS_KP*(local_goal[1]-wy)
            speed = math.hypot(vx, vy)
            cap = min(MAX_SPEED, FLEE_CHASE_SPEED)
            if speed > cap:
                vx, vy = vx*cap/speed, vy*cap/speed
            vx, vy = self._apply_friend_avoidance(vx, vy)
            self._send_vel(vx, vy)
            # Camera observations and the official judge establish confirmation;
            # orbit duration alone never establishes a 15-second capture.
            self._publish_claim()
            return

        # 计算当前相对于目标的角度
        angle = math.atan2(wy - ty, wx - tx)

        # 更新角度（顺时针盘旋）
        dt = 1.0 / CTRL_RATE
        angle += ORBIT_SPEED * dt
        if angle > math.pi:
            angle -= 2 * math.pi

        # 目标位置
        target_x = tx + ORBIT_RADIUS * math.cos(angle)
        target_y = ty + ORBIT_RADIUS * math.sin(angle)

        # P 控制飞向盘旋点
        err_x = target_x - wx
        err_y = target_y - wy
        vx = POS_KP * err_x
        vy = POS_KP * err_y
        spd = math.hypot(vx, vy)
        if spd > MAX_SPEED:
            vx *= MAX_SPEED / spd
            vy *= MAX_SPEED / spd

        # 友机避碰
        vx, vy = self._apply_friend_avoidance(vx, vy)

        self._send_vel(vx, vy)

        # 检查确认时间
        confirm_duration = now - self._confirm_start
        self._publish_claim()

        if confirm_duration >= CONFIRM_TIME:
            rospy.loginfo("[%s] 目标 %s 已连续确认 %.1fs，可消除", self.uav_id, self._orbit_target, confirm_duration)
            # 团队侧已满足 15s，官方却迟迟没消除 = 官方链路断了（坐标过期 / 遮挡 / 它在逃跑）。
            # 继续死等只会把飞机锁死（实测 571s），不如放弃、让 manager 重新派活。
            if confirm_duration >= ORBIT_GIVEUP:
                self._giveup_until[self._orbit_target] = (
                    rospy.Time.now().to_sec() + GIVEUP_COOLDOWN)
                self._abort_orbit('确认 %.0fs 官方仍未消除，放弃（%.0fs 内不再追）'
                                  % (confirm_duration, GIVEUP_COOLDOWN),
                                  clear_path=False)

    def _adaptive_speed(self, vx, vy):
        """根据场景自适应速度：搜索快、确认慢、避障稳"""
        # 基础速度
        base_speed = MAX_SPEED

        # 1. Do not cap tracking below a 2m/s target while waiting for FLEE state.
        if self._orbit_target is not None:
            base_speed = min(MAX_SPEED, FLEE_CHASE_SPEED)
        # 2. ORCA 激活时（友机近）降速
        elif self._friend_positions:
            # 检查是否有近距友机
            wx, wy = self.world_xy
            for fid, (fx, fy, fz) in self._friend_positions.items():
                dist = math.hypot(fx - wx, fy - wy)
                if dist < SLOWDOWN_DIST:
                    base_speed = SLOW_SPEED
                    break
        # 3. 大幅转向时降速（防止超调）
        if vx != 0 or vy != 0:
            current_heading = math.atan2(vy, vx)
            if self._last_heading is not None:
                heading_diff = abs(current_heading - self._last_heading)
                # 处理角度跳变（-pi 到 pi）
                if heading_diff > math.pi:
                    heading_diff = 2 * math.pi - heading_diff
                if heading_diff > 0.5:  # > 30度
                    base_speed = min(base_speed, 3.0)
            self._last_heading = current_heading

        # 应用自适应速度
        current_speed = math.hypot(vx, vy)
        if current_speed > base_speed:
            scale = base_speed / current_speed if current_speed > 0 else 1
            vx *= scale
            vy *= scale

        return vx, vy

    def _apply_friend_avoidance(self, vx, vy):
        """ORCA 避碰：修复无解兜底、对称死锁、速度衰减问题"""
        wx, wy = self.world_xy
        if wx is None:
            self._last_orca_v = (vx, vy)
            return vx, vy
        _vx_in, _vy_in = vx, vy   # 记录 ORCA 前的期望速度

        # 先应用自适应速度
        vx, vy = self._adaptive_speed(vx, vy)

        # 起飞/爬升阶段豁免水平避让：起飞区 6 架间距只有 8m，全部落在 ORCA
        # 互斥范围内会直接死锁（实测 ORCA 无解 110 次、6 架全程趴地起不来）。
        if self.local_z is not None and self.local_z < ALT_TAKEOFF + CLIMB_NO_AVOID:
            self._last_orca_v = (vx, vy)
            return vx, vy

        # ====== BUG 1: 无解时没兜底 - 改为强制使用排斥 ======
        # 先检查是否有近距友机需要避让
        has_conflict = False
        for fid, (fx, fy, fz) in self._friend_positions.items():
            dist = math.hypot(fx - wx, fy - wy)
            if 0.5 < dist < FRIEND_SAFE_DIST:
                has_conflict = True
                break

        if not has_conflict:
            self._last_orca_v = (vx, vy)
            return vx, vy  # 无冲突直接返回

        # 当前速度
        v_current = math.hypot(vx, vy)
        # ====== BUG 2: 速度衰减后无法恢复 - 始终保持最小速度 ======
        MIN_SPEED = MAX_SPEED * 0.3  # 最小巡航速度 0.9 m/s
        if v_current < MIN_SPEED:
            v_current = MIN_SPEED

        # ORCA 半平面集合
        orca_halfplanes = []

        for fid, (fx, fy, fz) in self._friend_positions.items():
            dist = math.hypot(fx - wx, fy - wy)
            # ====== 高度分层：用真实高度判断 ======
            # 本机真实高度
            my_alt = self.local_z if self.local_z is not None else self.altitude_layer
            # 友机高度（从状态消息获取真实高度）
            friend_alt = fz if fz is not None else self.altitude_layer
            # 用三维距离判断够不够远：垂直 separation 与水平距离同等计数。
            # 旧写法只用二值的「是否同层」，层间距 0.5m 时名义分层却仍能让两机
            # 水平贴到 0.22m（实测 iris_2×iris_6）；反过来像 VERT_SEP=0.8 那样
            # 让相邻层也避让，又会避让密度爆炸、起飞区直接互斥死锁。
            dz = abs(my_alt - friend_alt)
            d3 = math.hypot(dist, dz)
            if d3 > SAFE_3D:
                continue

            # ====== BUG 3: 对称死锁 - 用 ID 做 tie-break ======
            # 奇数 ID 往左，偶数 ID 往右
            uav_num = int(self.uav_id.split('_')[-1]) if '_' in self.uav_id else 0
            bias_dir = 1 if uav_num % 2 == 1 else -1  # 1=右, -1=左

            if dist < FRIEND_SAFE_DIST * 3:
                px, py = fx - wx, fy - wy
                if dist > 0.01:
                    nx, ny = px / dist, py / dist
                    # 加 bias 让对称情况不反复横跳
                    nx += bias_dir * 0.3
                    ny += bias_dir * 0.3
                    # 归一化
                    n_len = math.hypot(nx, ny)
                    if n_len > 0.01:
                        nx, ny = nx / n_len, ny / n_len

                    # 2026-09-27：用三维距离（分层高度也算进去）+ 近距加大增益。
                    # 原来的水平距离 + 固定 /2.0，同层贴近时才够强，来不及分离。
                    # === 2026-09-29 P0-1/P0-2 ===
                    # 原实现：(a) d3 跨过 SAFE_3D 时分母 2.0->1.0，sep_vel 由 3.0 突跳到 6.0；
                    #        sep_vel=6.0 > MAX_SPEED=5.0 时半平面 v·n>=6.0 无解 -> 不避让（坠机起因）
                    #        (b) d3 in (4,10] 时 sep_vel 仅 0~3.0，太弱 -> 只降速不转向（真空带）
                    # 修：分母在 [DANGER_3D, SAFE_3D] 上连续过渡到远场值 SEP_FAR_DIV，
                    #     再对 sep_vel 做可解性削顶。SEP_FAR_DIV=2.0 即回退旧远场行为。
                    _frac = max(0.0, min(1.0,
                               (d3 - DANGER_3D) / max(1e-6, SAFE_3D - DANGER_3D)))
                    _div = SEP_GAIN + (SEP_FAR_DIV - SEP_GAIN) * _frac
                    sep_vel = max(0.0, (FRIEND_SAFE_DIST - d3)) / _div
                    # 可解性硬顶：绝不越过 MAX_SPEED * SEP_CAP_FRAC
                    sep_vel = min(sep_vel, MAX_SPEED * SEP_CAP_FRAC)
                    orca_halfplanes.append((nx, ny, sep_vel))

        # 最后防线：已经贴到 DANGER_3D 以内，任何 ORCA 解都来不及 → 纯排斥
        _min_d3 = None
        for fid, (fx, fy, fz) in self._friend_positions.items():
            _dz = abs((self.local_z if self.local_z is not None else self.altitude_layer)
                      - (fz if fz is not None else self.altitude_layer))
            _d3 = math.hypot(math.hypot(fx - wx, fy - wy), _dz)
            if _min_d3 is None or _d3 < _min_d3:
                _min_d3 = _d3
        if _min_d3 is not None and _min_d3 < DANGER_3D:
            _rx, _ry = 0.0, 0.0
            for fid, (fx, fy, fz) in self._friend_positions.items():
                _dd = math.hypot(fx - wx, fy - wy)
                if 0.01 < _dd < DANGER_3D * 2.0:
                    _wg = (DANGER_3D * 2.0 - _dd) / _dd
                    _rx += (wx - fx) * _wg
                    _ry += (wy - fy) * _wg
            _rl = math.hypot(_rx, _ry)
            if _rl > 1e-6:
                _sp = MAX_SPEED * 0.6
                self._last_orca_v = (_rx / _rl * _sp, _ry / _rl * _sp)
                return self._last_orca_v

        # 搜索安全速度（全向采样）
        if orca_halfplanes:
            v_angle = math.atan2(vy, vx)
            best_vx, best_vy = vx, vy
            best_score = -float('inf')

            # 2026-09-27：候选改为全向。原来只在 [v_angle±1.5rad] 采样，
            # 相向接近时必须大转向才能满足半平面 → 无解 → 退化到强制排斥，
            # 实测贴到 0.70m。全向采样保证只要几何上存在安全方向就找得到。
            _nang = max(8, AVOID_ANGLES)
            _angles = [v_angle]   # 先放当前朝向，评分相同时保持惯性
            for _k in range(_nang):
                _angles.append(v_angle + 2.0 * math.pi * _k / _nang)
            for angle in _angles:
                for speed in [MIN_SPEED, MAX_SPEED * 0.5, MAX_SPEED * 0.7, MAX_SPEED]:
                    cand_vx = speed * math.cos(angle)
                    cand_vy = speed * math.sin(angle)
                    valid = True
                    for nx, ny, min_vel in orca_halfplanes:
                        vel_along_n = cand_vx * nx + cand_vy * ny
                        if vel_along_n < min_vel - 0.1:
                            valid = False
                            break
                    if valid:
                        score = -abs(speed - v_current)  # 优先保持原速度
                        if score > best_score:
                            best_score = score
                            best_vx, best_vy = cand_vx, cand_vy

            if best_score == -float('inf'):
                # ====== BUG 1 修复: 无解时强制排斥 ======
                rospy.logwarn("[%s] ORCA 无解，强制排斥", self.uav_id)
                for fid, (fx, fy, fz) in self._friend_positions.items():
                    dist = math.hypot(fx - wx, fy - wy)
                    if 0.5 < dist < FRIEND_SAFE_DIST:
                        force = FRIEND_K * (FRIEND_SAFE_DIST - dist) / dist
                        best_vx += (wx - fx) * force
                        best_vy += (wy - fy) * force
            vx, vy = best_vx, best_vy

        # 限幅到 MAX_SPEED（防止 ORCA 排斥力过大导致撞墙）
        spd = math.hypot(vx, vy)
        if spd > MAX_SPEED:
            vx = vx * MAX_SPEED / spd
            vy = vy * MAX_SPEED / spd

        # === 算法层日志：ORCA 避障输入/输出速度 + 最近友机距离 ===
        rospy.loginfo_throttle(
            0.5,
            "[ALGO] ORCA uav=%s in=(%.2f,%.2f) out=(%.2f,%.2f) "
            "n_halfplanes=%d min_friend_d3=%.2fm | safe3d=%.1f danger3d=%.1f",
            self.uav_id, _vx_in, _vy_in, vx, vy,
            len(orca_halfplanes),
            _min_d3 if _min_d3 is not None else -1.0,
            SAFE_3D, DANGER_3D)
        self._last_orca_v = (vx, vy)
        return vx, vy

    # ---------------- 状态上报 ----------------
    def _publish_status(self):
        quality = getattr(self, '_pose_quality', None)
        if quality is not None and not quality.usable(rospy.Time.now().to_sec()):
            return
        if self.world_xy is None:
            return

        st = UavStatus()
        st.header.stamp = rospy.Time.now()
        st.uav_id = self.uav_id
        st.x = self.world_xy[0]
        st.y = self.world_xy[1]
        st.z = self.local_z if self.local_z is not None else 0.0
        st.connected = self.state.connected

        # 用实际位置计算所在格子（而非被分配的格子）- 使用 10m 覆盖栅格
        # 覆盖判定由 manager 端用感知半径批量处理，agent 只需上报位置
        actual_cell = self.cov_grid.world_to_cell(self.world_xy[0], self.world_xy[1])
        if actual_cell is not None:
            st.cell_ix = actual_cell[0]
            st.cell_iy = actual_cell[1]
            # 只要有有效位置就报 confidence=1.0，manager 会用 20m 半径批量覆盖
            st.confidence = 1.0
        else:
            st.cell_ix = -1
            st.cell_iy = -1
            st.confidence = 0.0

        # 仍然记录被分配的格子（用于调试对比）
        if self.assignment is not None:
            st.assigned_cell_ix = self.assignment.cell_ix
            st.assigned_cell_iy = self.assignment.cell_iy
            st.assigned_target_x = self.assignment.target_x
            st.assigned_target_y = self.assignment.target_y
        else:
            st.assigned_cell_ix = -1
            st.assigned_cell_iy = -1
            st.assigned_target_x = 0.0
            st.assigned_target_y = 0.0

        self.status_pub.publish(st)

    # ---------------- 主循环 ----------------
    def run(self):
        # 等待连接
        while not rospy.is_shutdown() and not self.state.connected:
            self.ctrl_rate.sleep()
        rospy.loginfo("[%s] MAVROS 已连接", self.uav_id)
        # 等局部位置就绪
        while not rospy.is_shutdown() and self.local_xy is None:
            self.ctrl_rate.sleep()
        # 标定「局部->世界」的恒定平移（只取一次 model_states，之后不再订阅）
        if not self._calibrate_offset():
            raise SystemExit(1)

        self._configure_fcu()
        if not self._arm_and_offboard():
            raise SystemExit(1)

        rospy.loginfo("[%s] 开始协同搜索（高度层 %.1f m），等待管理器分配任务", self.uav_id, self.altitude_layer)
        while not rospy.is_shutdown():
            try:
                with self._authority_lock:
                    if self._mission_finished and not self._landing:
                        self._gate.stopping = True
                        self._send_vel(0., 0.)
                    else:
                        self._control()
                    self._publish_authority_state()
                self._detect_targets()      # 规则3：几何判定，命中即上报管理器
                if (rospy.Time.now() - self._last_status_t).to_sec() >= 1.0 / PUB_RATE:
                    self._publish_status()
                    self._last_status_t = rospy.Time.now()
            except Exception as e:
                rospy.logerr("[%s] 主循环异常: %s\n%s", self.uav_id, e, traceback.format_exc())
                with self._authority_lock:
                    self._gate.stopping = True
                    self._send_vel(0., 0.)
            self.ctrl_rate.sleep()


if __name__ == "__main__":
    import sys
    # 从 sys.argv 获取 __name:= 重映射的节点名，确保参数命名空间一致
    node_name = "swarm_agent"
    for arg in sys.argv:
        if arg.startswith("__name:="):
            node_name = arg.split(":=")[1]
            break
    rospy.init_node(node_name, anonymous=False)
    uav_id = rospy.get_param("~uav_id", "uav_1")
    model_name = rospy.get_param("~model_name", "iris_1")
    rospy.loginfo("swarm_agent 启动: uav_id=%s model=%s node=%s", uav_id, model_name, node_name)
    with PublisherAuthority(model_name):
        try:
            SwarmAgent(uav_id, model_name).run()
        except rospy.ROSInterruptException:
            pass
