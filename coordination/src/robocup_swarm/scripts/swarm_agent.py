#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集群搜索单机节点（每架无人机跑一个实例，自建实现）。

职责（步骤 1：两机共享状态）：
  1. 从 MAVROS 本地位姿加固定起飞点偏移计算世界坐标，
     从 /uav_N/mavros/local_position/pose 读高度与连接状态；
  2. 发布 /swarm/uav_status（UavStatus 自建消息）给集中式管理器；
  3. 订阅 /swarm/assignment（SearchAssignment），过滤出指派给自己的搜索格，
     用 ENU 速度控制飞向格中心。

坐标系说明（关键，避免多机坐标系踩坑）：
  世界/地图系与 gazebo 世界都是 ENU（x 东 y 北 z 上），各机本地 ENU 系只是
  原点不同、轴向平行。因此「世界坐标差」算出的速度向量可直接作为
  setpoint_velocity/cmd_vel 下发，无需逐机做 TF 变换。

只依赖标准库 + rospy + 标准消息，无 ROS 自定义依赖之外的第三方库。
"""

import os
import re
import json
import threading
from functools import wraps
import rospy
import math
import traceback
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robocup_swarm.msg import UavStatus, SearchAssignment, TargetState, TargetDetection
from robocup_navigation.astar import load_metadata, GridMap, plan
from slam_grid import SlamGrid
from flight_execution import bounded_step
from motion_safety import braking_cap, boundary_constraints, scan_constraints, HeightClearanceHold
from swarm_task import (LineOfSight, DETECT_RADIUS, CoverageGrid, GRID_SIZE_M,
                        coverage_bounds, ACTOR_SEARCH_BOUNDS)
from search_observation import (cell_points, in_view, observable_points,
                                next_heading, ground_distance_band)
from edge_search_review import EdgeReview
from csv_logger import logger
from visual_evidence import parse_visual_evidence
from tracking_view import (parse_tracking_view, update_altitude_hold, view_needs_space,
                           bind_red_view, bind_red_instance_view,
                           retain_pixel_view, pixel_recovery_view,
                           update_pixel_altitude_hold, ReportViewWindow,
                           VIEW_MIN_DOWN_SIN)
from fast_tracking_view import fast_yaw_enabled, validate_fast_view
from gimbal_attention import validate_handoff, selected_uavs
from search_view_hint import CameraOrigins, SearchViewInbox, failed_view
from search_segment_agent import SearchSegmentAgent
from search_segments import clear_line
from camera_attention import (approach_image, candidate_velocity, candidate_decision,
                              candidate_reposition, attention_expired,
                              localized_follow_decision)
CANDIDATE_IMAGE_MAX_TILT = math.radians(max(0., min(18.,
    float(os.environ.get('CANDIDATE_IMAGE_MAX_TILT_DEG', '8')))))
# 2026-10-03：飞控参数读回校验（取自队友 codex/radar-coordination-20261003 分支）
from fcu_configuration import configure as configure_fcu_parameters


def serialized_visual_task(method):
    """Keep camera identity changes and task callbacks in one transaction."""
    @wraps(method)
    def call(self, *args, **kwargs):
        lock = self.__dict__.setdefault('_visual_task_lock', threading.RLock())
        with lock:
            return method(self, *args, **kwargs)
    return call

# 覆盖栅格参数（与 manager 一致）
MAP_X_MIN, MAP_X_MAX = -100.0, 100.0
MAP_Y_MIN, MAP_Y_MAX = -50.0, 50.0

# Search uses the public actor box. Safe flight viewpoints and sensor-map
# padding are independent; neither creates extra routine search jobs.
FLIGHT_X_MIN = max(-50.0, float(os.environ.get('FLIGHT_X_MIN', '-50.0')))
FLIGHT_X_MAX = min(150.0, float(os.environ.get('FLIGHT_X_MAX', '150.0')))
FLIGHT_Y_MIN = max(-60.0, float(os.environ.get('FLIGHT_Y_MIN', '-60.0')))
FLIGHT_Y_MAX = min(60.0, float(os.environ.get('FLIGHT_Y_MAX', '60.0')))
FLIGHT_INSET = float(os.environ.get('FLIGHT_INSET', '2.0'))
MOTION_SAFETY = os.environ.get('MOTION_SAFETY','0') == '1'
# Trial bounds, kept separate from the command slew limit. The recorded 13
# sustained stops averaged 0.93--2.92 m/s^2; 0.8 is a conservative trial value,
# not a certified lower bound on physical braking in all scenes.
BRAKE_DECEL = float(os.environ.get('BRAKE_DECEL','0.8'))
BRAKE_DELAY = float(os.environ.get('BRAKE_DELAY','0.25'))


def flight_bounds():
    return FLIGHT_X_MIN, FLIGHT_X_MAX, FLIGHT_Y_MIN, FLIGHT_Y_MAX


def search_bounds(map_bounds=None):
    return coverage_bounds(ACTOR_SEARCH_BOUNDS if map_bounds is None else map_bounds,
                           flight_bounds())


def flight_waypoint(x, y, inset=FLIGHT_INSET):
    xmin, xmax, ymin, ymax = flight_bounds()
    inset = max(0.0, min(inset, (xmax - xmin) / 4.0,
                         (ymax - ymin) / 4.0))
    return (min(max(x, xmin + inset), xmax - inset),
            min(max(y, ymin + inset), ymax - inset))


def estimated_world_hard_bounds(grid, margin_fraction=0.25):
    """Accept legal map-edge flight; reject only positions well beyond it.

    The old fixed +/-100 m defaults made x>150 look corrupt even though the
    match metadata permits x up to 155 m.  Recovery from the actual boundary
    is handled separately by _bounds_recovery_velocity().
    """
    xmin, ymin = grid.origin
    xmax = xmin + grid.width * grid.resolution
    ymax = ymin + grid.height * grid.resolution
    mx = (xmax - xmin) * margin_fraction
    my = (ymax - ymin) * margin_fraction
    return xmin - mx, xmax + mx, ymin - my, ymax + my

# ============================ 参数 ============================
# 工作空间根目录：可用环境变量 ROBOCUP_WS 覆盖（云端部署/换用户时无需改代码）
WS_ROOT = os.environ.get("ROBOCUP_WS", "/home/ros/team_ws/robocup")
SEARCH_ALTITUDE = 4.5     # 搜索高度 m（最高限制）

# === 官方裁判合规护栏（score_cal.py: z > 6.0 -> score=0 并立即终止任务）===
# 2026-10-01 修复坠毁循环：原 ALT_PANIC=5.0 / ALT_EMERG_CEIL=5.0 太接近最高
# 巡航高度 4.6m，正常 0.5~1m 超调就触发 EMERG vz=-3.0 暴力下降 → 坠地 →
# 恢复爬升 → 再超调的死循环（agent_2 实测 5.28→-5.28→2.74→5.28 循环）。
# 改为递进式层级：4.5(目标上限) → 5.5(硬顶-1.0) → 5.7(快降-1.5) → 5.9(紧急-2.0) → 6.0(官方红线)。
ALT_HARD_CEIL     = float(os.environ.get('ALT_HARD_CEIL', '5.2'))   # 规则 §2.5(3)：飞行高度 ≤6m。2026-10-06 七轮复盘：judge 判 Gazebo 真值而护栏用 EKF 估计，多机 SITL 下 z 估计正偏可达 ~1m（t=1887 估计 11.66 真值正常；t=2005 估计 7.13 真值>6 已终止），阈值整体下移留足估计偏差缓冲
ALT_PANIC         = float(os.environ.get('ALT_PANIC', '5.5'))        # 强制快降阈值 m（估计偏差缓冲后距 6m 约 1m）
ALT_PANIC_DESCENT = float(os.environ.get('ALT_PANIC_DESCENT', '-1.5'))  # 快降速度 m/s
ALT_PANIC_HSCALE  = float(os.environ.get('ALT_PANIC_HSCALE', '0.3'))    # 此时水平速度系数
MAX_ACC           = float(os.environ.get('MAX_ACC', '2.5'))         # 水平加速度限幅 m/s^2
ALT_HARD_DESCENT  = -1.0   # 强制下降速度 m/s（原 -0.6 下降太慢）
# === 6m 红线二次保险 ===
ALT_EMERG_CEIL     = float(os.environ.get('ALT_EMERG_CEIL', '5.7'))      # 二次保险（估计 5.7 时真值即使 +0.5m 偏差仍 <6.2，配合 HARD 5.2 已先触发）
ALT_EMERG_DESCENT  = float(os.environ.get('ALT_EMERG_DESCENT', '-2.0'))  # 强制快降 m/s（原 -3.0 太暴导致坠地）
ALT_EMERG_HSCALE   = float(os.environ.get('ALT_EMERG_HSCALE', '0.35'))   # 水平速度系数
ALT_TARGET_CAP     = float(os.environ.get('ALT_TARGET_CAP', '4.2'))      # 目标高度上限（4.5→4.2：距 6m 红线 1.8m，弹飞/估计偏差缓冲）

# === v14 Fix C/D：坠毁恢复爬升上限 + P 控制爬升卡死闩锁（2026-10-07）===
# v14 (logs_20261007_123525) agent_4 真值 z 在 t=438s 越过 6m 红线 score=0。
# 根因：EKF 雪崩熔断后静默 return → offboard stream 断 → PX4 failsafe HOLD
# → EKF2 z 漂移 236s 缓慢爬升。另两条爬升路径同样无 EKF 上限：
#   1) 坠毁恢复 CRASH_RECOVER_VZ=1.5 m/s 恒爬升（_send_vel 开头分支 + crash 段）
#   2) P 控制 alt_error 恒正（z 估计卡低）时 0.6 m/s 持续爬升
ALT_RECOVER_CEIL    = float(os.environ.get('ALT_RECOVER_CEIL', '4.5'))      # 坠毁恢复爬升的 EKF 高度上限（到顶悬停，防无限爬升冲红线）
CRASH_RECOVER_MAX_S = float(os.environ.get('CRASH_RECOVER_MAX_S', '3.0'))   # 坠毁恢复爬升最长时长（EKF z 卡低时 local_z 判据失效，用时间兜底强制停止；3s × 1.5 = 4.5m 真值上限 < 6m）
CLIMB_LATCH_S       = float(os.environ.get('CLIMB_LATCH_S', '2.5'))         # P 控制连续爬升判卡死的时长窗
CLIMB_LATCH_DZ      = float(os.environ.get('CLIMB_LATCH_DZ', '0.3'))        # 判卡死的最小高度位移（窗口内动了 < 此值 → 估计卡死）

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
# ---- 国家一等奖修复（2026-10-04）：追逃自适应节流 ----
# 真实仿真（logs_20261004_0837 局）实锤：t1 确认两次死于「全部观察机同时丢失
# 目标 >1s」（resets 1→3 而 rejects 恒 1，即 not-covered 分支）。根因：actor
# 2 m/s 逃跑时，旧节流（3.0m/2.0s）让路径目标滞后真位最多 4m，UAV 贴着 20m
# 探测门边缘飞，一个滞后就把几何门顶穿 → 全队同时丢 → 确认清零 → 30s 瞬移。
# 修复：目标处于 FLEE 时收紧到 1.5m/0.8s（A* 最坏 1.25Hz，控制频率仍 >15Hz），
# 路径目标滞后压到 ≤1.2m，UAV 稳定咬在 20m 门内。非逃跑目标维持旧节流省算力。
TRACK_REPLAN_MOVE_FLEE = 1.5   # 逃跑目标：移动 1.5m 即重规划 (m)
TRACK_REPLAN_SEC_FLEE  = 0.8   # 逃跑目标：0.8s 必重规划 (s)
# A* 失败时不能直飞：当前地图含建筑，直飞会把一次规划失败升级为撞楼。
# 需要恢复旧的实验行为时仍可显式设置 PLAN_FALLBACK=1。
PLAN_FALLBACK = int(os.environ.get("PLAN_FALLBACK", "0"))
MAX_SPEED       = float(os.environ.get('MAX_SPEED', '6.0'))   # 巡航速度上限 m/s（2026-规则 §2.5(4)：恐怖分子感知后 2m/s 逃逸，本机必须 ≥6 才能跟住；6m/s 留 2x 余量）
POS_KP          = 0.8     # 位置 P 控制增益
ARRIVE_TOL      = 0.8     # 到达格中心判定半径 m（< 此值视为已到，开始原地搜索）
SEARCH_ARRIVE_TOL = float(os.environ.get('SEARCH_ARRIVE_TOL', '2.0'))
SEARCH_SWEEP_DEG = float(os.environ.get('SEARCH_SWEEP_DEG', '20.0'))
SEARCH_SWEEP_PERIOD_S = float(os.environ.get('SEARCH_SWEEP_PERIOD_S', '12.0'))
EDGE_SEARCH_REVIEW = os.environ.get('EDGE_SEARCH_REVIEW', '0') == '1'
SEARCH_HEADING_RATE = math.radians(25.0)
CAMERA_CUE_HOLD_S = 2.0
CAMERA_HANDOFF_HOLD_S = 4.0
CAMERA_CUE_RETRY_S = 2.0
PIXEL_VIEW_RECOVERY = os.environ.get('PIXEL_VIEW_RECOVERY', '0') == '1'
PIXEL_RECOVERY_APPROACH = os.environ.get('PIXEL_RECOVERY_APPROACH', '0') == '1'
# Control-only candidate; the validated baseline remains the default until
# paired GUI trials establish whether it improves moving cropped views.
COUPLED_VIEW_GEOMETRY = os.environ.get('COUPLED_VIEW_GEOMETRY', '0') == '1'
SEARCH_SCAN_YAW_TOL = math.radians(float(os.environ.get('SEARCH_SCAN_YAW_TOL_DEG', '12')))
SEARCH_SCAN_HOLD_S = float(os.environ.get('SEARCH_SCAN_HOLD_S', '0.25'))
SEARCH_SCAN_TIMEOUT_S = float(os.environ.get('SEARCH_SCAN_TIMEOUT_S', '4.0'))
# A south-patrol aircraft may arrive facing along the route. At the normal
# 25 deg/s search-heading rate a 90-degree turn alone takes 3.6 seconds.
SEARCH_DIRECTED_LOOK_TIMEOUT_S = float(os.environ.get(
    'SEARCH_DIRECTED_LOOK_TIMEOUT_S', '8.0'))
PUB_RATE        = 10.0    # 状态发布频率 Hz
CTRL_RATE       = 20.0    # 控制频率 Hz
DETECT_RATE     = float(os.environ.get('DETECT_RATE', '3.0'))   # 规则 §2.5(10)：连续 15s 正确广播 → 3Hz × 15s = 45 个采样，远高于裁判认定的连续窗口
# 幽灵目标闸门：/swarm/target_states 停发（桥接 DROP_TIME）超过此时长后，
# 停止对该目标的几何检测上报。与 manager TRUTH_TTL 对齐——否则本字典里的
# 过期坐标会被几何检测持续上报，manager _detection_cb 当新检测反复派机
# （2026-10-01 22:41 局实测：t3 幽灵吸住 4 架 3 分钟，确认进度反复归零）。
# 国家一等奖优化（2026-10-03）：与 yolo_target_bridge DROP_TIME=8s 对齐，
# 防止 bridge 已停发但 agent 仍上报过期坐标的窗口期。
TARGET_TTL      = float(os.environ.get("TARGET_TTL", "2.5"))

# 2D 雷达安全层：由 swarm_agent 唯一发布 MAVROS 速度设定点，避免与
# radar_avoid 的 setpoint_position 双控制。雷达只在近障时修正当前速度。
RADAR_GUARD     = int(os.environ.get('RADAR_GUARD', '1'))
RADAR_FRESH_S   = float(os.environ.get('RADAR_FRESH_S', '0.5'))
RADAR_WARN_R    = float(os.environ.get('RADAR_WARN_R', '5.5'))   # 规则 §2.5(7)：碰撞扣30/次，雷达预警半径扩大到 5.5m（车体级别障碍），留 1.5m 减速带宽
RADAR_STOP_R    = float(os.environ.get('RADAR_STOP_R', '1.6'))   # 硬停距 ≥ 1.6m，确保横向漂移不会擦肩
RADAR_SIDE_PANIC_R = float(os.environ.get('RADAR_SIDE_PANIC_R', '1.6'))
RADAR_SIDE_PANIC_SPEED = float(os.environ.get('RADAR_SIDE_PANIC_SPEED', '0.5'))
# === 2026-10-03 雷达「必须能刹住」硬约束 + 贴墙后退（防撞楼）===
# 实测事故（02:24 场）：飞机在 world (2.87,-8.78) 顶住 house_2_126 的北立面
# （该楼 x[-2.5,8.5] y[-18.5,-9.5]，北面 y=-9.5，机身 y=-8.8 ⇒ 净空 0.7m），
# 雷达 front 长期饱和在量程下限 0.50m；随后 EKF 崩坏、OFFBOARD 失效保护落地。
# 旧逻辑只在 front < RADAR_STOP_R 时把「前向分量」压零，但：
#   a) 侧向逃逸分量仍可达 1.5 m/s，没有任何"必须能在接触前停住"的约束；
#   b) 飞机一旦进了 A* 膨胀带（INFLATE_M=1.5）内侧，网格守卫只看前向 ±30° 锥，
#      贴墙平行滑动时完全看不见侧面的墙，于是"合法"地一路蹭过去。
# 这里加两条：① 合速度按刹停距离 v <= sqrt(2*a*(d-安全间隙)) 硬夹；
#             ② front 近到 RADAR_BACKOFF_R 以内时主动沿机体 -x 后退离开。
RADAR_SAFE_GAP    = float(os.environ.get('RADAR_SAFE_GAP', '0.8'))    # 期望最小净空 m
RADAR_BACKOFF_R   = float(os.environ.get('RADAR_BACKOFF_R', '1.8'))   # 触发后退的 front m（> RADAR_STOP_R=1.6，确保 front ∈ (1.6, 1.8] 时仍可后退，否则 v_cap=0 死锁）
RADAR_BACKOFF_SPD = float(os.environ.get('RADAR_BACKOFF_SPD', '1.5')) # 后退速度 m/s（2026-10-05 撞墙修复：0.6→1.5）
RADAR_BACKOFF_MAX = float(os.environ.get('RADAR_BACKOFF_MAX', '2.5')) # 后退速度上限 m/s（防过冲）
# === 2026-10-05 国家一等奖修复：雷达 v_cap=0 死锁 + spike 滤波 ===
# 实测（logs_20261005_183958/06_swarm_agent_0.log）：
#   - front=1.45 < RADAR_STOP_R=1.6 → 触发「单面近障」分支
#   - d_min=0.70 < RADAR_SAFE_GAP=0.8 → v_cap = sqrt(2*2.5*max(0, -0.1)) = 0
#   - 任何水平速度 (speed>1e-9) 都被夹到 0
#   - RADAR_BACKOFF_R=1.2 < front=1.45 → 后退不触发
#   - 结果：vout=(0,0) 持续 ~50ms，飞机 0 速度死锁"不动"
# 修复：
#   1. v_cap 有最低保底 0.4 m/s（绝不输出 0 速度，否则雷达一帧噪声就永久卡死）
#   2. EMA 滤波防 spike：front/left/right 用滑动平均，单帧随机跳变不再触发死锁
RADAR_VCAP_FLOOR  = float(os.environ.get('RADAR_VCAP_FLOOR', '0.4'))   # v_cap 最低保底 m/s（防 0 死锁）
RADAR_EMA_ALPHA   = float(os.environ.get('RADAR_EMA_ALPHA', '0.6'))    # 雷达扇区 EMA 平滑系数（0=全记忆，1=不滤波）
# v11 复盘（2026-10-07）：自回波地板。SDF 雷达已抬到 z=0.38m（盒顶 0.335m 上方），
# 但实测仍有 0.31~0.34m 恒定回波（疑似云台/盒顶边缘残余部件，方向随部件转动变化）。
# 低于此距离的回波一律视为机身自身而非真实障碍（真实障碍 <0.5m 时碰撞已不可避免，
# 由 A* 栅格层 + 提前减速兜底）。同时用于 SLAM 建图前的射线预过滤。
# v12 复盘（logs_20261007_120121）：地板 0.40 时 left=0.40 出现 177 次——v11 在开阔
# 起飞区（无真实障碍）就有 right=0.49/0.50 读数，证明回波带尾部延伸到 ~0.50m，
# 0.40 地板只滤掉主体、0.40~0.45 尾部残余仍触发虚假近障/三面堵死。抬到 0.50
# 与 SLAM mark_scan 的 min_range 对齐（单一地板）。
RADAR_SELF_ECHO_M = float(os.environ.get('RADAR_SELF_ECHO_M', '0.50'))
# B1 近障困局脱离：前方与两侧持续收窄时，限时沿来路反向退出，避免在建筑巷道内原地抖动。
RADAR_JAM_FRONT_R = float(os.environ.get('RADAR_JAM_FRONT_R', '2.5'))
RADAR_JAM_SIDE_R = float(os.environ.get('RADAR_JAM_SIDE_R', '4.0'))
RADAR_JAM_TRIGGER_S = float(os.environ.get('RADAR_JAM_TRIGGER_S', '4.0'))
RADAR_JAM_ESCAPE_S = float(os.environ.get('RADAR_JAM_ESCAPE_S', '3.0'))
RADAR_JAM_ESCAPE_SPD = float(os.environ.get('RADAR_JAM_ESCAPE_SPD', '1.0'))
RADAR_JAM_CLEAR_R = float(os.environ.get('RADAR_JAM_CLEAR_R', '4.0'))
# R1（2026-10-09）守卫交替振荡死锁检测：雷达/栅格守卫每帧反复改写方向
# 互相抵消 → 原地打转（v26 agent_3 90s、agent_0 80s）。检测到守卫持续
# 交替反向时，强制沿目标航向直行 OSC_BREAKOUT_S，期间守卫只限幅不改向。
OSC_WATCH_S = float(os.environ.get('OSC_WATCH_S', '3.0'))      # 观察窗口 s
OSC_SWITCH_N = int(os.environ.get('OSC_SWITCH_N', '4'))        # 窗口内反向切换 ≥N 次判死锁
OSC_BREAKOUT_S = float(os.environ.get('OSC_BREAKOUT_S', '2.0'))# 强制脱困直行时长 s
OSC_BREAKOUT_SPD = float(os.environ.get('OSC_BREAKOUT_SPD', '1.2'))  # 直行速度 m/s
# R2（2026-10-09）世界坐标可信度：EKF 慢漂移钳制/病态重锚定后，world 与世界
# 物理位置脱节，禁止这段时间内做"到圈即贴脸"决策（v26 距真身 48m 假到达）。
WORLD_TRUST_HOLD_S = float(os.environ.get('WORLD_TRUST_HOLD_S', '3.0'))
# 正常飞行的 EKF 速度尖峰常只需截断几厘米；这种微调不应连续刷新
# 3 秒不可信窗口，使目标近旁的控制循环一直悬停。
WORLD_TRUST_CLAMP_EXCESS_M = float(os.environ.get('WORLD_TRUST_CLAMP_EXCESS_M', '0.15'))
# === 2026-10-05 比赛规则硬约束：扩大软减速带 ===
# 旧默认 MAP_GUARD_SOFT=2.0 + MAP_GUARD_MARGIN=1.0 = 总 3m 软带。本机巡航 6 m/s
# 下，0.05s 一帧移动 0.3m，3m 软带只能覆盖 10 帧 = 0.5s，飞机在进入软带到硬停之间
# 的「刹车距离」不足，会冲出硬边界（uav_2 实测 199m）。新默认 SOFT=6.0 + MARGIN=2.0
# = 总 8m 软带：在 6 m/s 下提供 1.3s 缓冲，足以让 _send_vel 的加速度限幅
# (MAX_ACC=2.5 m/s²) 把横向速度从 6 m/s 刹到 0.5 m/s（数学上需 6/2.5=2.4s，
# 8m 仍偏紧但允许减速到 4 m/s 进入硬边界，回收模式可挽回）。
MAP_GUARD_MARGIN = float(os.environ.get('MAP_GUARD_MARGIN', '2.0'))
MAP_GUARD_SOFT   = float(os.environ.get('MAP_GUARD_SOFT', '6.0'))   # 硬边界外的软减速带宽 m
# 越界主动回收：旧逻辑越界后 A* 起点在外拒绝规划、栅格守卫把界外当墙、
# 地图守卫只锁朝外分量不主动内拉 → 飞机瘫痪在界外永远回不来（2026-10-01
# 实测 uav_2 飘到 (201,-178)、uav_3 到 (-84,141) 并死锁）。
OOB_RECOVER_SPEED = float(os.environ.get('OOB_RECOVER_SPEED', '5.0'))
# === 2026-10-05 比赛规则硬约束：OOB 回收速度对齐 MAX_SPEED ===
# 旧默认 3.0 m/s：无人机以 6 m/s 越界时，3 m/s 反向回收净速度仍有 3 m/s 向外
# （回收永远追不上漂移），实测 uav_2 在 2s 内 world_xy 漂到 199m（地图 +99m 外），
# 撞墙后 z=-8.06 强制坠地恢复。改为 5.0 m/s → 净速度 ≥ -2 m/s（向内），1s 内可
# 收回 2m，5s 内收回 10m。仍低于 MAX_ACC=2.5 加速度上限 → 不致触发 PX4 限制。
OOB_RECOVER_INSET = float(os.environ.get('OOB_RECOVER_INSET', '3.0'))  # 回到界内多少 m
OOB_BYPASS_ACC_LIM = os.environ.get('OOB_BYPASS_ACC_LIM', '1') not in ('0', 'false', 'False', '')
# OOB 回收时跳过水平加速度限幅：撞墙越界时必须立即反向，0.125 m/s 每帧的爬升率
# （MAX_ACC/CTRL_RATE=2.5/20）会让回收指令被延迟 5-10 帧生效（0.25-0.5s），期间
# 飞机继续以原速度向外冲。关闭限幅确保「回收 = 当帧满速反向」。
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
# 2026-10-07 v12 复盘: 爬升期显式垂直速度. 爬升闸门原走 _send_vel(0,0)（vz=None
# → 自适应目标高度 P 控制），而 _compute_desired_altitude 在自身格被灯柱膨胀区
# (INFLATE_M=2.5) 覆盖时返回 -0.5 → target_alt = 层高2.8-0.5 = 2.3 < 2.5，
# P 控制收敛在 2.3m，闸门永不开（实测 agent1/5 全场悬停 z≈2.28-2.37 零任务，
# CSV cmd_vz≈±0.01 实锤）。改显式 vz 后爬升与自适应目标解耦，过闸后仍由
# 自适应高度接管（贴楼降高语义保留）。
CLIMB_VZ        = float(os.environ.get('CLIMB_VZ', '0.5'))
ALT_BASE        = float(os.environ.get('ALT_BASE', '2.8'))    # 最低巡航层
ALT_STEP        = float(os.environ.get('ALT_STEP', '0.6'))    # 层间距
ALT_NLAYER      = int(os.environ.get('ALT_NLAYER', '4'))     # 层数
ALT_CEILING     = float(os.environ.get('ALT_CEILING', '4.6')) # 硬顶：官方对 >6m 重罚，这里留 1.4m 余量
VERT_SEP        = float(os.environ.get('VERT_SEP', '0.8'))   # 判定「不在同一层」的竖直间隔
MIN_CRUISE_ALT  = 2.0                                        # 追踪降高后的下限
ALT_P           = float(os.environ.get('ALT_P', '1.0'))      # 高度 P 控制增益（原 0.5 太小，贴近目标时爬升乏力）
ALT_VZ_MIN      = float(os.environ.get('ALT_VZ_MIN', '0.2')) # 高度误差存在时的最小升降速度 m/s
CLIMB_NO_AVOID  = float(os.environ.get('CLIMB_NO_AVOID', '3.0'))  # 起飞爬升期豁免 ORCA/grid 避让的高度余量 m（z < ALT_TAKEOFF+CLIMB_NO_AVOID 时不避让）
# 原值 1.0 只豁免到 z=3m；6 机在 8m 间距内爬升，3m 以上就全部落进 ORCA 互斥圈互相排斥压速，形成原地振荡。
# 改为 3.0，豁免到 z=5m（接近 ALT_CEILING=4.6m），起飞阶段全程不避让。
# Grid guard 同样需要豁免：z<ALT_SAFE_ALT 时完全绕过，ALT_SAFE_ALT~ALT_CEILING 间线性衰减强度。
SAFE_ALT        = ALT_TAKEOFF + CLIMB_NO_AVOID  # 安全爬升高度上限（=5.0m）
SAFE_3D         = float(os.environ.get('SAFE_3D', '5.0'))   # 规则 §2.5(7)：6机密集，友机3D排斥起点抬到 5.0m（避免 0.7m 擦肩）
# 2026-09-27 复标：3.0 时实测最近 0.70~0.85m（一次碰撞 −30 分）。
# 控制周期 20Hz、友机位置来自 10Hz 广播，5m/s 下 0.2s 滞后就是 1m ——
# 3m 门限根本来不及反应，抬到 4.0 才有足够的提前量。
SLOWDOWN_DIST   = float(os.environ.get('SLOWDOWN_DIST', '14.0'))

# === 2026-09-29 P1-2：三处高度护栏原来静默覆写 cmd.twist.linear.z ===
# 出事只能靠抓包反推。这里统一打点：同 tag 首次必打，之后每 5s 一条并带累计次数。
_ALT_GUARD_STATE = {}


def _alt_guard_hit(tag, z, ceil, vz, vx, vy):
    # 2026-10-05 修复:墙钟→仿真钟。
    # 原 time.time() 在 RTF=0.15 仿真下,墙钟 5s ≈ 仿真 0.75s,
    # 节流窗口设的 5s 实际过密,日志刷屏,排查时看不到节奏。
    # 改成仿真钟后,日志密度与业务秒数一致,节流策略才符合最初设计意图。
    import rospy as _rospy
    _now = _rospy.Time.now().to_sec()
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
# === 2026-10-05 国家一等奖修：EST_GUARD 雪崩熔断（EKF 永久崩坏保护）===
# 实测（logs_20261005_194720）：PX4 EKF 在 Gazebo 多机物理重叠时会持续雪崩，
# 报告 50-60 m/s 幻影速度。EST_GUARD 闸门判定不可信 → 悬停等待，累计很快破千拍
# → 整局停滞。引入熔断：累计悬停 N 拍后判定 EKF 永久不可信，直接切入
# AUTO.RTL（Return-To-Launch，强制回 home），等下次启动重新跑。
# 之前用 AUTO.LAND，但实测（logs_20261005_200254）切 LAND 后飞机仍卡在某位姿
# 持续触发 EKF 雪崩，5/6 架"看似悬停实则卡住"，只 1 架正常真飞。RTL 让 PX4
# 不依赖本地估计直接飞回 home，更可靠。
EST_FROZEN_N        = int(os.environ.get('EST_FROZEN_N', '50'))    # 累计 50 拍 ≈ 2.5s@20Hz 后 LAND
EST_FROZEN_HOLD_S   = float(os.environ.get('EST_FROZEN_HOLD_S', '0.5'))  # 熔断后 RTL 持续多久才允许重试起飞
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

# ---- 目标定距观察 ----
# 国家一等奖标准修复（2026-10-04）：
#   旧 ORBIT_RADIUS=5.0 + ORBIT_SPEED=0.19 → 线速度 = 0.19×5 = 0.95 m/s，**刚好压在 1.0 m/s
#   触发线上**。一旦 EKF 抖动或位置控制超调，飞机瞬时速度就会突破 1.0 m/s → actor 触发逃跑。
# 官方 control_actor.py 在无人机距演员 <7m 时会持续把演员推走。
# 下载目录的六目标开发场景快照采用 8m 观察圈；留 1m 缓冲，同时保持
# 在本队 9.5m 的近距裁判播报门内。
ORBIT_RADIUS    = 8.0     # 观察距离 m（> actor uav_push_radius=7.0）
# 近距观察半径：09:15 本轮 red1 在 2.5~4.5m 时框中心落到 480px
# 图像的 y=466~477，随后失去真实观测、官方坐标误差持续扩大。8m 使人
# 保持在画面内部且位于 7m 驱离区之外，仍低于 9.5m 播报进入距离门。
ORBIT_RADIUS_CLOSE = float(os.environ.get('ORBIT_RADIUS_CLOSE', '8.0'))
# 近处压低以免脚部出画面；8m 观察圈则保留搜索高度附近的视线。
# 21:03 局绿色在 8m 处由 3.4m 降至约 2.2m 后连续丢失真实视觉，
# 地图几何 LOS 却一直为 True，裁判确认被反复重置。
ORBIT_ALT_MAX      = float(os.environ.get('ORBIT_ALT_MAX', '2.6'))
ORBIT_FAR_ALT_MAX  = float(os.environ.get('ORBIT_FAR_ALT_MAX', '3.5'))
ORBIT_FAR_ALT_START = float(os.environ.get('ORBIT_FAR_ALT_START', '5.5'))
ORBIT_FAR_ALT_FULL = float(os.environ.get('ORBIT_FAR_ALT_FULL', '7.5'))


def observation_altitude_cap(range_m):
    """Smoothly raise a distant observer without cropping a nearby actor."""
    if range_m is None or not math.isfinite(range_m):
        return ORBIT_ALT_MAX
    span = max(ORBIT_FAR_ALT_FULL - ORBIT_FAR_ALT_START, 0.1)
    frac = min(1.0, max(0.0, (range_m - ORBIT_FAR_ALT_START) / span))
    return ORBIT_ALT_MAX + frac * (ORBIT_FAR_ALT_MAX - ORBIT_ALT_MAX)
# 平稳近距速度保持 0.95m/s；距离持续拉大时短时使用追逃速度。
CLOSE_TRANSIT_SPEED = float(os.environ.get('CLOSE_TRANSIT_SPEED', '0.95'))
EARLY_VIEW_PURSUIT = os.environ.get('EARLY_VIEW_PURSUIT', '0') == '1'
CLOSE_BACKOFF_SPEED = float(os.environ.get('CLOSE_BACKOFF_SPEED', '1.8'))
CLOSE_RECEDING_MIN_DIST = 6.0
CLOSE_RECEDING_RATE = 0.45
CLOSE_TARGET_AWAY_RATE = 0.25
CLOSE_RECEDING_SAMPLE_S = 0.7
CLOSE_CHASE_HOLD_S = 2.0
# A search aircraft can enter the observation ring at 3 m/s.  The ordinary
# 2.5 m/s² command slew then takes almost a second to reach the close cap,
# carrying it inside the actor's 7 m push radius before the slow orbit starts.
# Only increase the slew rate while *reducing* an existing command in close
# mode.  Acceleration toward the actor keeps the normal bound.
CLOSE_BRAKE_ACC = float(os.environ.get('CLOSE_BRAKE_ACC', '8.0'))
VIEW_MAX_ACC = 1.2
VIEW_MOTION_RANGE = 12.0
VIEW_BRAKE_MARGIN = .25


def horizontal_slew_limit(vx, vy, last_v, close_mode=False, visual_mode=False):
    """Limit command changes while allowing prompt braking at ring entry."""
    lvx, lvy = last_v
    max_acc = MAX_ACC
    old_speed_sq = lvx * lvx + lvy * lvy
    if visual_mode:
        max_acc = min(MAX_ACC, VIEW_MAX_ACC)
    elif (close_mode and old_speed_sq > CLOSE_TRANSIT_SPEED ** 2 and
            vx * vx + vy * vy < old_speed_sq and
            vx * lvx + vy * lvy < old_speed_sq):
        max_acc = max(MAX_ACC, CLOSE_BRAKE_ACC)
    max_dv = max_acc / CTRL_RATE
    dvx, dvy = vx - lvx, vy - lvy
    dv = math.hypot(dvx, dvy)
    if dv > max_dv and dv > 1e-9:
        return lvx + dvx * max_dv / dv, lvy + dvy * max_dv / dv
    return vx, vy


def view_approach_velocity(vx, vy, wx, wy, tx, ty, tvx, tvy, radius=8.0):
    """Brake relative inward motion before the ring; retain target translation.

    This preference is applied BEFORE collision avoidance. Zero requests stay
    zero, and the helper cannot increase the existing command's speed budget.
    """
    speed = math.hypot(vx, vy)
    distance = math.hypot(tx-wx, ty-wy)
    if speed < .05 or distance < 1e-6:
        return vx, vy
    ux, uy = (tx-wx)/distance, (ty-wy)/distance
    relative_cap = math.sqrt(2.*VIEW_MAX_ACC*max(
        0., distance-radius-VIEW_BRAKE_MARGIN))
    excess = (vx-tvx)*ux+(vy-tvy)*uy-relative_cap
    if excess <= 0.:
        return vx, vy
    vx, vy = vx-excess*ux, vy-excess*uy
    result_speed = math.hypot(vx, vy)
    if result_speed > speed:
        vx, vy = vx*speed/result_speed, vy*speed/result_speed
    return vx, vy


def view_braking_distances(last_v, wx, wy, tx, ty, tvx, tvy, radius=8.0):
    """Return available/required radial room for the softer visual brake.

    Use the preceding final command, never the newly capped preference: a
    late image must not hide momentum already commanded toward the target.
    Reserve one control interval in addition to continuous stopping distance.
    """
    distance = math.hypot(tx-wx, ty-wy)
    room = max(0., distance-radius-VIEW_BRAKE_MARGIN)
    if distance < 1e-6:
        return room, float('inf')
    closing = max(0., ((last_v[0]-tvx)*(tx-wx)+
                       (last_v[1]-tvy)*(ty-wy))/distance)
    need = closing*closing/(2.*min(MAX_ACC, VIEW_MAX_ACC))+closing/CTRL_RATE
    return room, need


def observation_ring_goal(wx, wy, tx, ty, radius, bounds, margin=2.5):
    """Choose an in-bounds observation point when the radial point is outside.

    A target at the north edge can have a UAV just north of it.  Preserving
    that radial angle asks the aircraft to orbit outside the search map, where
    the map guard cancels its velocity and the camera soon loses the target.
    Prefer a reachable point along the side of the ring; penalize paths that
    cut through the actor's 7 m push circle.
    """
    xmin, xmax, ymin, ymax = bounds
    xmin += margin
    xmax -= margin
    ymin += margin
    ymax -= margin
    angle = math.atan2(wy - ty, wx - tx)
    preferred = (tx + radius * math.cos(angle),
                 ty + radius * math.sin(angle))
    if xmin <= preferred[0] <= xmax and ymin <= preferred[1] <= ymax:
        return preferred
    best = None
    for i in range(32):
        a = 2.0 * math.pi * i / 32.0
        gx, gy = tx + radius * math.cos(a), ty + radius * math.sin(a)
        if not (xmin <= gx <= xmax and ymin <= gy <= ymax):
            continue
        dx, dy = gx - wx, gy - wy
        length_sq = dx * dx + dy * dy
        fraction = (max(0.0, min(1.0,
                    ((tx - wx) * dx + (ty - wy) * dy) / length_sq))
                    if length_sq > 1e-9 else 0.0)
        closest = math.hypot(wx + fraction * dx - tx,
                             wy + fraction * dy - ty)
        cut_penalty = 100.0 if closest < min(7.0, math.hypot(wx - tx, wy - ty)) - 0.1 else 0.0
        cost = math.sqrt(length_sq) + cut_penalty
        if best is None or cost < best[0]:
            best = (cost, gx, gy)
    # The actor can be exactly on a corner: the requested radius may have no
    # legal point.  Still choose the closest interior view, never the old
    # out-of-bounds preferred point.
    if best is not None:
        return best[1], best[2]
    return (min(max(preferred[0], xmin), xmax),
            min(max(preferred[1], ymin), ymax))


def close_follow_velocity(err_x, err_y, target_vx, target_vy, observation_age,
                          speed_cap=CLOSE_TRANSIT_SPEED):
    """Follow the moving observation point with the selected pursuit cap."""
    vx, vy = POS_KP * err_x, POS_KP * err_y
    if (0.0 <= observation_age <= TARGET_TTL and
            math.isfinite(target_vx) and math.isfinite(target_vy)):
        target_speed = math.hypot(target_vx, target_vy)
        feedforward_cap = 2.5 if speed_cap > CLOSE_TRANSIT_SPEED else 1.2
        if target_speed > feedforward_cap:
            target_vx *= feedforward_cap / target_speed
            target_vy *= feedforward_cap / target_speed
        vx += target_vx
        vy += target_vy
    speed = math.hypot(vx, vy)
    if speed > speed_cap:
        vx *= speed_cap / speed
        vy *= speed_cap / speed
    return vx, vy


def close_target_is_receding(dist, old_dist, elapsed, observation_age,
                             target_away_rate):
    """Detect a widening camera range from fresh observations, not track velocity.

    The vision velocity is heavily smoothed and was only 0.2 m/s while the
    red actor moved about 2 m/s in the 13:52 match.
    """
    return (dist >= CLOSE_RECEDING_MIN_DIST and
            CLOSE_RECEDING_SAMPLE_S <= elapsed <= 2.0 and
            0.0 <= observation_age <= 1.0 and
            target_away_rate >= CLOSE_TARGET_AWAY_RATE and
            (dist - old_dist) / elapsed >= CLOSE_RECEDING_RATE)


def ring_standoff_velocity(vx, vy, wx, wy, tx, ty, radius,
                           target_vx=0.0, target_vy=0.0,
                           speed_cap=CLOSE_TRANSIT_SPEED):
    """Give a too-close camera a definite outward command, even with feedforward."""
    dx, dy = wx - tx, wy - ty
    dist = math.hypot(dx, dy)
    if dist < 1e-6 or dist >= radius - 0.5:
        return vx, vy
    ux, uy = dx / dist, dy / dist
    # When a walking actor is already moving away, it will restore the ring
    # separation itself.  Forcing the aircraft to retreat at 6-7 m lost the
    # camera view in the 12:24 run, despite an otherwise valid 0.95 m/s
    # follow command.  Keep the backoff when truly inside the 5.5 m buffer.
    receding = -(target_vx * ux + target_vy * uy)
    if dist > 5.5 and receding >= 0.4:
        return vx, vy
    outward = vx * ux + vy * uy
    if outward < 0.35:
        vx += (0.35 - outward) * ux
        vy += (0.35 - outward) * uy
    speed = math.hypot(vx, vy)
    if speed > speed_cap:
        vx *= speed_cap / speed
        vy *= speed_cap / speed
    return vx, vy


def urgent_close_backoff_velocity(vx, vy, wx, wy, tx, ty, tvx, tvy):
    """Retreat when a walking actor is about to enter the camera's blind spot.

    A 0.95 m/s cap cannot preserve standoff from an actor walking toward the
    aircraft at about 1 m/s.  Keep the quiet cap while there is room, then
    spend the actor's one-time escape response if the alternative is losing
    the observation entirely.
    """
    dx, dy = wx - tx, wy - ty
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return vx, vy, False
    ux, uy = dx / dist, dy / dist
    approaching = tvx * ux + tvy * uy
    if dist < 6.5 and approaching >= 0.4:
        speed = min(CLOSE_BACKOFF_SPEED, max(1.2, approaching + 0.45))
        return speed * ux, speed * uy, True
    if dist < 5.5:
        outward = vx * ux + vy * uy
        if outward < 0.85:
            vx += (0.85 - outward) * ux
            vy += (0.85 - outward) * uy
        return vx, vy, False
    return vx, vy, False
# 2026-10-06 国家一等奖: 贴脸锁定时长, 比官方 15s + 余量.
# 期间即使 manager 接着确认其他 tid, 本机贴脸也不退出.
CONFIRM_LOCK_DURATION = 30.0
# === 2026-10-03 队友补（单机主链实测）：播报距离闸门 ===
# 够不着的距离（m）。官方判据是"误差<1m 连续 15s"，实测单目测距 7m 内误差
# 0.26~0.69m、15.98m 时误差 1.1m 已越界 ⇒ 超过此距离不判"播报被拒"而放弃目标。
CLOSE_ENOUGH_M  = float(os.environ.get("CLOSE_ENOUGH_M", "11.0"))
# SPOOK_DIST 从 15m 提到 22m：比逃跑触发边界 20m 还远，让 actor 永远看不到
# 任何 > 1.0 m/s 的 UAV 冲进来。
SPOOK_DIST      = 22.0    # 进入此距离就压速（官方逃跑判定边界 20m，我们提前 2m 保险）
# 国家一等奖标准修复（2026-10-04）：SPOOK_SPEED=0.5 已 OK，但 0.5 m/s 在 22m 外接近 actor
# 时，6 机密集区域容易触发 ORCA 把速度拉得更慢（MIN_SPEED=0.9）。SPOOK_SPEED 改为 0.85
# 既不触犯 1.0 m/s 阈值（15% 余量）、又能用 SPOOK_DIST=22m 让 ORCA 互斥开始前就到位。
# 仍然配合 SPOOK_DIST=22m 形成「飞机不冲到 20m 内」的硬护栏，actor 不会逃跑。
SPOOK_SPEED     = 0.85    # 接近阶段最大线速度 m/s（离 1.0 m/s 阈值 15% 余量，ORCA 起步前稳进）
# v24 提速（2026-10-09）：搜索巡航恒速 + 近引导点减速带距离。
# 依据 v23 重定标：官方逃跑为「一次性闩锁」且未武装（未播报）目标不会逃，
# 搜索段可放心全速巡航；武装目标 22m 内由 _transit_cap 压速兜底。
# 原 P 控制搜索在 LOOKAHEAD=3m 时理论 2.4m/s、实际仅 ~1.7m/s（拥堵/近障），
# 5min 只覆盖地图 17%（x∈[-44.5,43], y∈[-54.5,-9]），4/6 actor 的活动区
# 从未被搜索格覆盖 → 全程零观测零派机。恒速 3.0m/s + 2m 减速带（近引导点
# 切回 P 控制自然减速，防冲过格中心）。
SEARCH_CRUISE_SPEED = float(os.environ.get('SEARCH_CRUISE_SPEED', '3.0'))
SEARCH_START_DISTANCE_M = max(0.0, float(os.environ.get('SEARCH_START_DISTANCE_M', '10.0')))
SEARCH_START_SPEED = max(0.0, float(os.environ.get('SEARCH_START_SPEED', '1.5')))
SEARCH_DECEL_M      = float(os.environ.get('SEARCH_DECEL_M', '2.0'))
# v23 重定标（2026-10-09，规则 PDF 深挖 + 官方 master e9e4ef8 代码核对）：
# 官方 control_actor.py 逃跑机制实为「一次性闩锁」——
#   1) 触发是瞬时判定：reported（官方已收到有效目标报告）且任意 UAV
#      <20m 且地速>1.0m/s → 立即逃跑（无「持续 2s」判定，v21 注释有误）；
#   2) escape_triggered 置位后全场无任何复位点 → 每个 actor 全场至多逃一次，
#      跑完路线后回 1m/s 随机走且永远不会再逃；
#   3) 逃跑速度 escape_speed=2.0，路线=距最近 UAV 最远的地图角（可达 100m+）。
# 因此 v21「0.95 追逃等官方复位」模型作废（0.95 追 2.0 必丢 YOLO 12m 接触，
# 正是 v20/v21 confirming lost 的根因）。新策略：
#   - 真在逃（观测速度≥ESCAPE_RUN_SPEED）→ 全速咬住，14m 内切牧羊 pace；
#   - 逃跑跑完=「已花掉」→ 解除一切压速护栏，全速贴脸消除；
#   - 正式节点不读取裁判触发状态，所有已见目标按可能武装控速；
#   - 从本队视觉速度判断逃跑并咬住目标。
FLEE_CHASE_SPEED  = float(os.environ.get('FLEE_CHASE_SPEED', '6.0'))
# 国家一等奖标准修复（2026-10-04）：bridge 已把 state 语义对齐「被观测即逃跑」，
# FLEE_STATE_FRESH 不应只 3s（3s 后就回到 SPOOK_SPEED=0.5 慢速又被甩开）。
# 规则 §2.5(4)：30s 未消除才瞬移；这之前 actor 一直在 2 m/s 跑。给到 25s 留 5s 余量。
FLEE_STATE_FRESH  = float(os.environ.get('FLEE_STATE_FRESH', '25.0'))
# === v23（2026-10-09）一次性逃跑生命周期参数（配套上方重定标） ===
ESCAPE_RUN_SPEED      = float(os.environ.get('ESCAPE_RUN_SPEED', '1.7'))    # 观测速度≥此值视为逃跑跑中（官方逃 2.0 / 随机走 1.0）
WALK_SPEED_MAX        = float(os.environ.get('WALK_SPEED_MAX', '1.2'))      # 观测速度≤此值视为行走
ESCAPE_SPENT_QUIET_S  = float(os.environ.get('ESCAPE_SPENT_QUIET_S', '8.0'))  # 逃跑跑过后持续行走此时长 → 判定「已花掉」
UNARMED_BRAKE_DIST    = float(os.environ.get('UNARMED_BRAKE_DIST', '13.0'))   # 未武装目标全速接近的减速点（播报闸门 11m + 2m 余量）
FLEE_SHEPHERD_DIST    = float(os.environ.get('FLEE_SHEPHERD_DIST', '14.0'))   # 追逃时距目标小于此值切牧羊 pace（不冲过目标）
FLEE_SHEPHERD_SPEED   = float(os.environ.get('FLEE_SHEPHERD_SPEED', '2.6'))   # 牧羊 pace：略快于逃跑 2.0，保持 YOLO 视场
SPEND_ENABLE          = int(os.environ.get('SPEND_ENABLE', '1'))              # 官方反复重置确认后主动花掉一次性逃跑（0=回退旧退避行为）
# === 2026-10-03 队友补（单机主链实测）：输出通道 / 位置设定点 / 估计可信性闸门 ===
# 最终下发给 PX4 的通道：pos=位置设定点（默认，实测唯一能起飞且跟踪正常的通道）；
# vel=旧的 setpoint_velocity 通道（本机实测垂向跟踪只有 20%，留作对照）。
FLIGHT_OUTPUT   = os.environ.get("FLIGHT_OUTPUT", "pos")
# 位置通道下发 sp = 当前位置 + v * POS_SP_LEAD；PX4 位置环（MPC_XY_P=0.95）给出的
# 速度设定点 ≈ 0.95 * POS_SP_LEAD * v。取 POS_SP_LEAD ≈ 1/MPC_XY_P，使实际速度 ≈ 指令速度。
# POS_SP_STEP_MAX 同步放开：取 MAX_SPEED*POS_SP_LEAD，否则高速段又被夹回衰减。
POS_SP_LEAD     = float(os.environ.get("POS_SP_LEAD", "1.0"))       # 位置设定点前瞻 s
POS_SP_STEP_MAX = float(os.environ.get("POS_SP_STEP_MAX",
                                       str(MAX_SPEED * 1.0)))       # 单帧位置增量上限 m
POS_SPV         = int(os.environ.get("POS_SPV", "0"))                # 1=打印位置设定点诊断
# 位置估计可信性闸门（P0，防"追幻影"）：近 EST_WIN_S 秒平均位移超过物理可能
# （MAX_SPEED*EST_V_RATIO）即判估计不可信 → 悬停等待 EKF 收敛，不追垃圾坐标。
EST_GUARD   = int(os.environ.get('EST_GUARD', '1'))       # 0=关闭（A/B 对照）
EST_V_RATIO = float(os.environ.get('EST_V_RATIO', '2.0'))  # 允许速度 / MAX_SPEED
EST_WIN_S   = float(os.environ.get('EST_WIN_S', '0.3'))    # 判定窗口 s
# 跳变后允许采纳的 offset 修正量上限 m：合理的 EKF 原点重置只平移几米，
# 实测崩坏时曾要求修正 32.56m —— 那种量级必须拒绝（否则守卫全在错误坐标上）。
EKF_REANCHOR_MAX_M = float(os.environ.get('EKF_REANCHOR_MAX_M', '5.0'))
# ---- 偏航对准（2026-10-01：修「盘旋时目标甩出视场」）----
# 双目是水平朝前安装的（HFOV=90°，±45°），而 agent 历史上只发线速度、不发偏航，
# PX4 保持机头朝向不变。圆形盘旋时目标相对机头的方位连续转 360°，只有约 1/4 时间
# 在视场内 → track 反复断、actor_info 断帧、15s 连续确认永远累不满。
# 追踪/盘旋时用偏航角速度 P 控制让机头始终指向目标。
YAW_KP          = float(os.environ.get("YAW_KP", "2.5"))      # 偏航环增益（1/s）
YAW_RATE_MAX    = float(os.environ.get("YAW_RATE_MAX", "1.5"))# 偏航角速度限幅 rad/s
CONFIRM_TIME    = 15.0     # 连续确认时间才消除（规则5）
# 国家一等奖标准：LOS 短暂丢失（actor 绕到建筑背后 1-2s）不重置
# 累计确认进度。修复后进度条不会被短时遮挡清零，5 分钟内可多完成
# 多次 15s 计数 → 直接对应奖级任务的「连续 15s」判定。
CONFIRM_LOS_GRACE = float(os.environ.get("CONFIRM_LOS_GRACE", "2.0"))

# ---- 盘旋放弃 / 防扎堆（2026-09-27：修「飞机被已消除目标占死 571s」）----
TARGET_STALE    = float(os.environ.get("TARGET_STALE", "6.0"))    # 给 bridge/备份机留接力窗口 s
# === v16：到圈后 stale 宽限搜索窗口 ===
# v15 实测（agent_0 追 t1）：接近阶段 _orbit_target=None 使 _orbit_stale() 形同
# 虚设，朝冻结位置飞 56s；到圈瞬间 gap=56.6s>TARGET_STALE 立即放弃——白飞。
# actor 静止时冻结位置即真实位置，到圈后应盘旋改变视角给 YOLO 重捕获窗口。
STALE_SEARCH_GRACE = float(os.environ.get("STALE_SEARCH_GRACE", "12.0"))
# 接近阶段 stale 拦截的距离门槛：远于此值才因 stale 悬停，已在途中放行
STALE_ABORT_DIST_M = float(os.environ.get("STALE_ABORT_DIST_M", "40.0"))
# 国家一等奖优化：满 15s 后官方这么久还没消除 → 放弃 s（原 30s）
ORBIT_GIVEUP    = float(os.environ.get("ORBIT_GIVEUP", "20.0"))   # 15+20=35s 就放弃
# 国家一等奖优化：放弃后这段时间内不再自动盘旋该目标 s（原 45s）
GIVEUP_COOLDOWN = float(os.environ.get("GIVEUP_COOLDOWN", "30.0"))# 缩短冷却，尽快投入新目标
CLAIM_ENABLE    = int(os.environ.get("CLAIM_ENABLE", "1"))        # 防扎堆：别机正在确认的目标不再抢
CLAIM_FRESH     = float(os.environ.get("CLAIM_FRESH", "1.5"))     # 认领消息的新鲜期 s（缩短到 1.5，让接力响应更快）

# ---- 确认失败退避（2026-09-28：修「单机被抖动目标锁死 600s」）----
BACKOFF_ENABLE    = int(os.environ.get("BACKOFF_ENABLE", "1"))          # 0=关闭（A/B 对照）
# 国家一等奖优化：官方重置这么多次就退避（原 3）
CONFIRM_RESET_MAX = int(os.environ.get("CONFIRM_RESET_MAX", "2"))       # 2次就退避
BACKOFF_COOLDOWN  = float(os.environ.get("BACKOFF_COOLDOWN", "45.0"))   # 退避时长 s（原 60s）
RESET_DECAY       = float(os.environ.get("RESET_DECAY", "90.0"))        # 距上次重置这么久就清零计数 s（原 120s）

# ---- 友机避碰 ----
FRIEND_SAFE_DIST = 3.5  # 规则 §2.5(7)：6机密集 + 碰撞扣30/次；友机起点抬到 3.5m（避让柔和，避免起飞区互斥死锁）
FRIEND_K        = 2.0  # 增加排斥增益      # 排斥增益

# ---- A* 避障飞行 ----
METADATA_PATH   = os.environ.get(
    "ROBOCUP_METADATA",
    os.path.join(WS_ROOT, "src/robocup_training_worlds/worlds/generated/robocup_base.json"))
INFLATE_M       = float(os.environ.get('INFLATE_M', '2.5'))  # A* 障碍膨胀半径 m（2026-10-06 灯杆撞杆修复：2.0 仍实测贴到 front=0.31m；2.5 = 灯杆半径 0.61m + 桨尖 0.37m + 切角超调 0.68m + 雷达误差 0.20m + 0.64m 缓冲）

# ---- 合规 SLAM 实时建图（2026-10-07 重构，规则 §2.4/§2.5 无预读）----
# metadata 障碍恒为空；占用栅格由本机 /scan 射线投射实时构建（slam_grid.py），
# 经 /swarm/occupancy_grid 汇总给 manager。以下为建图节流/量程/深度参数。
SLAM_HZ         = float(os.environ.get('SLAM_HZ', '5.0'))          # 建图处理频率 Hz（雷达 40Hz 全处理太耗 CPU）
SLAM_PERIOD     = 1.0 / max(0.1, SLAM_HZ)
SLAM_MAX_RANGE  = float(os.environ.get('SLAM_MAX_RANGE', '12.0'))  # 建图最大量程 m（远端回波噪声大；> RADAR_WARN_R 5.5 留减速余量）
SLAM_DEPTH_CELLS = int(os.environ.get('SLAM_DEPTH_CELLS', '4'))    # 命中点沿射线再填充 cell 数（0.5m/cell → 2m 墙厚，防 A* 从背面穿墙）
SLAM_REPLAN_SEC = float(os.environ.get('SLAM_REPLAN_SEC', '2.0'))  # 发现新障碍后触发重规划的最小间隔 s
SLAM_PUB_SEC    = float(os.environ.get('SLAM_PUB_SEC', '2.0'))     # 占用栅格发布间隔 s（RLE JSON，manager 合并用）
# v11 复盘（2026-10-07）：机体倾斜超过此角度时丢弃整帧雷达（2D 平面假设失效）。
# 爬升/加减速段俯仰 5~15°，前向射线随俯仰下压打地（z<2m 时打地点落在 8~12m 量程内），
# 把地面标成占用墙 → 起飞航迹前方一条假墙，A* 全程绕行/无解。
SLAM_TILT_RAD   = math.radians(float(os.environ.get('SLAM_TILT_DEG', '5.0')))
LOOKAHEAD       = 3.0     # 路径跟踪前瞻距离 m（> 刹停距离 v²/2a=2.25m）
# === 国家一等奖（2026-10-05）：搜索阶段机头持续对准「下一引导点」 ===
# 背景：本队 YOLO 使用官方前视双目左目，HFOV=90°（±45°）。
#       若到点 dwell 期机头"随便指" = 相机视野对到哪算哪，跟队友多点方案脱节。
# 行为：
#   - 飞行段：复用 _pick_local_goal() 的 LOOKAHEAD 点作为 _look_at（机头预摆）
#   - 到点 dwell 段：冻结到点前最后一个 local_goal（=下一个原本要飞到的搜索点）
# 关闭（=0）：保持原行为，到点交回 PX4 自管偏航。
DWELL_LOOKAHEAD = os.environ.get("DWELL_LOOKAHEAD", "1") not in ("0", "false", "False", "")
STABLE_NEEDED   = 40      # EKF 稳定判定：连续多少次 20Hz 采样高度达标（40 = 2s）
OFFBOARD_PRESTREAM_S = 2.0  # 与定位稳定检查重叠，按仿真时间累计连续零速输出
OFFBOARD_PRESTREAM_MAX_GAP_S = 0.25  # 输出间隔过大时重新预热
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


def remaining_path_blocked(grid, path, position):
    """Whether the planned route ahead now crosses an inflated occupied cell.

    Laser rays change cells all over the local map.  Replanning every time any
    cell changes causes six A* workers to repeat whole-map searches even when
    their current routes remain clear.  Only points from the closest route
    point onward matter; the already traversed prefix can be blocked safely.
    """
    if not path or position is None:
        return False
    px, py = position
    first = min(range(len(path)),
                key=lambda i: (path[i][0] - px) ** 2 + (path[i][1] - py) ** 2)
    own_cell = grid.world_to_cell(position)
    # A side wall can put the aircraft itself inside the 2.5 m inflation
    # margin while its raw cell is free.  Repeatedly declaring the first
    # metre of that same safe route invalid creates a NO_PATH/replan loop.
    # Only this local inflated pocket is exempt; raw hits and every farther
    # route point still invalidate the route.  The laser guard protects the
    # actual near-field flight direction independently.
    local_inflation = (getattr(grid, '_inflate_r', 0) * grid.resolution
                       if own_cell is not None and
                       not grid.is_free(own_cell) and
                       hasattr(grid, 'is_free_raw') and
                       grid.is_free_raw(own_cell) else 0.0)
    last_cell = None
    for waypoint in path[first:]:
        cell = grid.world_to_cell(waypoint)
        if cell is None:
            return True
        if cell != last_cell:
            if not grid.is_free(cell):
                local_margin = (local_inflation > 0.0 and
                                (waypoint[0] - px) ** 2 +
                                (waypoint[1] - py) ** 2 <=
                                local_inflation ** 2 and
                                grid.is_free_raw(cell))
                if not local_margin:
                    return True
            last_cell = cell
    return False


def cap_inward_close_speed(vx, vy, wx, wy, tx, ty,
                           stop_radius=5.5, slow_radius=7.0):
    """Slow a command toward a person before it enters the camera blind zone.

    Scaling keeps the direction already vetted by radar, map, and ORCA.  A
    forced outward command here could instead fly into a wall or another UAV.
    Outward and tangential commands are unchanged.
    """
    dx, dy = tx - wx, ty - wy
    dist = math.hypot(dx, dy)
    if dist >= slow_radius:
        return vx, vy
    if dist < 1e-6:
        return 0.0, 0.0
    inward = (vx * dx + vy * dy) / dist
    if inward <= 0.0:
        return vx, vy
    allowed = CLOSE_TRANSIT_SPEED * max(
        0.0, min(1.0, (dist - stop_radius) /
                 max(1e-6, slow_radius - stop_radius)))
    if inward <= allowed:
        return vx, vy
    scale = allowed / inward
    return vx * scale, vy * scale


class SwarmAgent(SearchSegmentAgent):
    _segment_flight_waypoint = staticmethod(flight_waypoint)

    def __init__(self, uav_id, model_name):
        self.uav_id = uav_id
        rospy.loginfo('[FINAL_SAFETY_CONFIG] uav=%s enabled=%s decel=%.2f delay=%.2f lidar_margin=%.2f',
                      uav_id,MOTION_SAFETY,BRAKE_DECEL,BRAKE_DELAY,RADAR_STOP_R)
        rospy.loginfo('[TRACKING_CONFIG] uav=%s early_view_pursuit=%s coupled_view_geometry=%s',
                      uav_id, EARLY_VIEW_PURSUIT, COUPLED_VIEW_GEOMETRY)
        self.model_name = model_name

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
        self._offset_param = None   # 官方固定出生点；运行中绝不按 EKF 漂移修改
        self._anchor_done = False   # 起飞前的稳定门；完成后监测局部位姿异常
        self._localization_fault = False  # 粘滞故障，须重启并重新建立合法坐标系
        self._local_prev_t = 0.0    # 上一帧 local_position 的 ROS 时间（跳变检测）
        self.local_z = None         # 高度（来自 local_position）
        # v14 Fix D：P 控制连续爬升卡死闩锁状态（EKF z 卡低防真值冲红线）
        self._climb_t0 = None        # 连续爬升计时窗起点
        self._climb_z0 = None        # 计时窗起点高度
        self._climb_latch = False    # 卡死闩锁（触发后 vz≤0 直到恢复）
        self._climb_latch_z = None   # 闩锁时刻高度（恢复变化检测基准）
        # v14 Fix C：坠毁恢复爬升起始时间（时间上限兜底 EKF z 卡低）
        self._crash_t0 = None        # crash_flagged 置位的 ROS 时间
        self._offboard_rearm_done = False  # v14 Fix E：熔断后 OFFBOARD 重臂一次性
        self.yaw = 0.0              # 机体 yaw（ENU 弧度，供雷达 body->world）
        self.pitch = 0.0            # 机体俯仰（弧度，SLAM 倾斜帧丢弃用）
        self.roll = 0.0             # 机体横滚（弧度，SLAM 倾斜帧丢弃用）
        self._scan = None            # 最近一帧 2D LaserScan
        self._scan_t = 0.0           # 最近雷达帧的 ROS 时间
        self.state = State()
        self.assignment = None      # SearchAssignment 当前任务
        self._fast_yaw_enabled = fast_yaw_enabled(os.environ, uav_id)
        self._fast_tracking_image = None
        self._fast_yaw_csv = (logger('fast_yaw_%s' % uav_id, [
            'ros_time', 'event', 'target_id', 'reason', 'yaw_source',
            'sample_s', 'seed_s', 'image_age', 'seed_age', 'quality',
            'real_sample_s', 'position_sample_s', 'bearing', 'real_bearing',
            'yaw', 'yaw_rate', 'candidate_id', 'world_candidate_id'])
            if self._fast_yaw_enabled else None)
        self._team_tracks = {}      # target_id -> assigned aircraft (main/backup)
        self._team_visual = {}      # (source, target) -> recent accepted image streak
        self._camera_cue = None
        self._camera_cue_retry_after = {}
        # Auxiliary rays have their own finite lease. A colour match cannot
        # turn this camera's instance into a fixed-camera person identity.
        self._gimbal_cue = None
        self._gimbal_handoff_enabled = (
            os.environ.get('GIMBAL_AUX_MODE', 'off') == 'handoff' and
            self.uav_id in selected_uavs(os.environ.get('GIMBAL_AUX_UAVS', '2,5')))
        self._search_scan_key = None
        self._search_scan_yaw0 = None
        self._search_scan_phase = 0
        self._search_scan_settle_t = None
        self._search_scan_started_t = None
        self._search_scan_done = False
        self._search_scan_failed = False
        self._search_scan_seen = set()
        self._search_scan_first = {}
        self._search_scan_directed_first = None
        self._search_scan_directed_last = None
        self._search_scan_directed_done = False
        self._search_last_frame = 0.0

        # ---- A* 避障（合规重构 2026-10-07：雷达 SLAM 实时建图）----
        # metadata 只提供规则定值（bounds/frame/spawn），障碍恒为空；
        # 占用栅格由本机 /scan 射线投射实时构建（扫到→mark occupied），
        # 不预读任何建筑真值（规则 §2.4/§2.5 无预读随机地图）。
        # 未知区域视为可通行：A* 先直飞，RADAR_GUARD 近障兜底，
        # slam 发现新障碍自动触发重规划（见 _slam_update）。
        self.md, _ = load_metadata(METADATA_PATH)
        _b = self.md.get("bounds", {}) or {}
        _res = (self.md.get("grid", {}) or {}).get("resolution_m", 0.5)
        self.slam = SlamGrid(
            _b.get("x_min", MAP_X_MIN), _b.get("y_min", MAP_Y_MIN),
            _b.get("x_max", MAP_X_MAX), _b.get("y_max", MAP_Y_MAX),
            _res, INFLATE_M)
        self.grid = self.slam          # is_free 已含膨胀，plan()/grid_guard 直接用
        self._slam_last_t = 0.0        # 上次建图处理的 ROS 时间
        self._slam_replan_t = 0.0      # 上次因新障碍触发重规划的时刻
        self._slam_pub_t = 0.0         # 上次发布占用栅格的时刻
        self._slam_dirty = False       # 自上次发布以来栅格有更新
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
        self._plan_epoch = 0         # 换任务后拒绝旧规划线程迟到的路径
        self._plan_event = threading.Event()
        self._plan_fail_t = 0.0      # 上次规划失败时刻（失败后 2s 冷却，避免刷屏）
        self._takeoff_done = False   # 是否已完成起飞垂直爬升
        self._search_start_progress = None
        self._last_flight_v = None   # 上次下发的水平速度 (vx, vy)，规划未就绪时续命
        self._last_orca_v = (0.0, 0.0)
        self._last_csv_t = 0.0
        self._csv = logger("control_%s" % uav_id, [
            "ros_time", "event", "target_id", "target_x", "target_y",
            "target_z", "uav_x", "uav_y", "uav_z", "distance_m",
            "capture_threshold_m", "is_captured", "orca_vx", "orca_vy",
            "orca_vz", "cmd_vx", "cmd_vy", "cmd_vz", "source",
            "visual_slew", "horizontal_stop", "view_sample_s",
            "view_needs_space", "view_range_floor", "view_brake_room",
            "view_brake_need", "view_brake_fallback"])
        self._look_at = None         # 机头需持续对准的世界点 (x,y)；None=不控偏航
        # 2026-10-05 国家一等奖：缓存到点前的最后一个 local_goal，dwell 期冻结视野
        self._last_local_goal = None

        # ---- 覆盖栅格（与 manager 一致，10m 格）----
        # 覆盖格与 manager 使用同一份 metadata 边界；固定 ±100 的旧边界
        # 会让 x>100 的扫描状态 confidence 恒为 0，边缘格永远无法完成。
        _coverage_bounds=search_bounds((
            _b.get("x_min", ACTOR_SEARCH_BOUNDS[0]), _b.get("x_max", ACTOR_SEARCH_BOUNDS[1]),
            _b.get("y_min", ACTOR_SEARCH_BOUNDS[2]), _b.get("y_max", ACTOR_SEARCH_BOUNDS[3])))
        self.cov_grid = CoverageGrid(*_coverage_bounds,cell_m=GRID_SIZE_M)

        # ---- 目标检测（规则3 几何判定：距离 + 视线遮挡）----
        # 用**未膨胀**的原始 SLAM 栅格做 LOS 判定（膨胀是给飞行留裕度的，
        # 判定遮挡要用真实建筑轮廓，否则会把建筑边缘 0.5m 内误判为遮挡）。
        # 合规重构：LOS 遮挡随雷达建图动态增长 —— 未扫到的区域视为无遮挡
        # （未知≠遮挡），扫到的墙体才判定遮挡。
        self.los = LineOfSight(
            lambda ix, iy: not self.slam.is_free_raw((ix, iy)),
            cell_size=self.slam.resolution, origin=self.slam.origin)
        self.targets = {}           # target_id -> (x, y, vx, vy)，来自 /swarm/target_states
        self._target_state = {}    # target_id -> (state, t)，目标运动状态（1=FLEE）
        self._detect_log_t = {}     # tid -> 上次 [ALGO] detect 日志时刻（按 target 分别节流）
        self._last_detect_t = 0.0
        # ---- v23：一次性逃跑生命周期（官方 escape_triggered 闩锁模型的团队侧镜像）----
        self._flee_run_t = {}       # tid -> 最近一次观测到逃跑跑速（≥ESCAPE_RUN_SPEED）的时刻
        self._walk_since = {}       # tid -> 逃跑跑过后持续行走的起始时刻
        self._escape_spent = set()  # 已花掉逃跑的目标（全场永不再逃 → 解除压速护栏）
        self._spend_mode = set()    # 指令态：主动全速触发并花掉该目标的一次性逃跑

        # ---- 目标盘旋确认 ----
        self._orbit_target = None    # 当前盘旋目标 ID
        # 团队图像连续只用于提示保持观察；不能证明定位精度或消除。
        # 收到 manager 的 /swarm/visual_streak 且 tid 匹配自己 _orbit_target 时,
        # _confirm_close=True; 目标切换或视觉过期时清回 False.
        self._confirm_close = False
        self._confirm_close_tid = None
        self._confirm_close_t0 = 0.0
        self._confirm_close_until = 0.0
        self._close_range_ref = None
        self._close_chase_tid = None
        self._close_chase_until = 0.0
        self._last_track_goal = None # 追踪时上次 A* 的目标点（用于节流）
        self._last_track_plan_t = 0.0  # 追踪时上次 A* 规划时刻
        self._plan_fallback_n = 0  # A* 失败退化直飞的次数
        self._orbit_center = None    # 盘旋中心 (x, y)
        self._visual_observations = {}
        self._eligible_observations = {}
        self._last_eligible_image = {}
        self._reobserve_view = None
        self._reobserve_forced = None
        self._reobserve_next = 0.0
        self._orbit_entered_t = 0.0
        self._confirm_start = 0.0   # 本机真实图像连续观测开始时间
        self._last_confirm_t = 0.0   # 上次确认时间
        self._los_lost_t = 0.0      # LOS 宽容窗口起算时刻（2026-10-09 修复：
                                    # 之前在 __init__ 缺失，agent 首次进入
                                    # _update_orbit 的不可见分支即 AttributeError
                                    # 崩溃，实测 agent_1 连续崩 2389 次、
                                    # agent_0 崩 8 次，直接瘫痪 2 机）
        self._target_to_orbit = None  # 待盘旋目标位置 (x, y)
        self._t_seen = {}          # tid -> 最后一次收到位置的时刻（判定目标是否已消失）
        self._giveup_until = {}    # tid -> 该时刻前不再自动盘旋（放弃过 / 已消除）
        self._stale_search_until = {}  # v16: tid -> 到圈后 stale 宽限搜索截止时刻
        self._alt_sag = 0.0        # v16: Fix D 自适应降层量（EKF z 病态时保底可用高度）
        self._claims = {}          # tid -> (uav_id, 时刻) 别机正在确认的目标
        self._last_claim_t = 0.0   # 认领广播节流

        # ---- 友机避碰 ----
        self._friend_positions = {}  # 其他无人机位置 {uav_id: (x, y, z)}
        self._friend_seen = {}

        # ---- MAVROS 服务 ----
        self.arm_srv = rospy.ServiceProxy("/%s/mavros/cmd/arming" % uav_id, CommandBool)
        self.mode_srv = rospy.ServiceProxy("/%s/mavros/set_mode" % uav_id, SetMode)
        self.param_srv = rospy.ServiceProxy("/%s/mavros/param/set" % uav_id, ParamSet)

        # ---- 订阅 ----
        # 正式节点只用 MAVROS 本地位置和固定官方起飞点参数；不读取仿真真值。
        rospy.Subscriber("/%s/mavros/state" % uav_id, State, self._state_cb)
        # 别机认领广播：防止多架机扎堆确认同一个目标
        rospy.Subscriber("/swarm/orbit_claim", String, self._claim_cb, queue_size=20)
        self._claim_pub = rospy.Publisher("/swarm/orbit_claim", String, queue_size=10)
        rospy.Subscriber("/%s/mavros/local_position/pose" % uav_id, PoseStamped, self._local_cb)
        if MOTION_SAFETY:
            self._directed_view_settle_enabled = True
            self._directed_view_settle_timeout = SEARCH_DIRECTED_LOOK_TIMEOUT_S
            self._directed_view_settle_yaw_tol = SEARCH_SCAN_YAW_TOL
            rospy.Subscriber("/%s/mavros/local_position/velocity_local" % uav_id,
                             TwistStamped,self._measured_velocity_cb,queue_size=1)
        rospy.Subscriber("/%s/scan" % uav_id, LaserScan, self._scan_cb,
                 queue_size=1)
        rospy.Subscriber("/swarm/assignment", SearchAssignment, self._assign_cb)
        rospy.Subscriber('/swarm/search_view_hint', String,
                         self._search_view_hint_cb, queue_size=6)
        rospy.Subscriber("/swarm/target_states", TargetState, self._target_cb)
        rospy.Subscriber("/swarm/discovery_states", TargetState, self._discovery_state_cb)
        rospy.Subscriber("/swarm/visual_source", String, self._visual_source_cb, queue_size=30)
        rospy.Subscriber("/swarm/visual_candidate", String, self._visual_candidate_cb, queue_size=6)
        rospy.Subscriber("/swarm/tracking_view", String, self._tracking_view_cb, queue_size=6)
        if self._gimbal_handoff_enabled:
            self._gimbal_feedback_pub = rospy.Publisher(
                '/swarm/gimbal_feedback', String, queue_size=6)
            rospy.Subscriber('/swarm/gimbal_candidate', String,
                             self._gimbal_candidate_cb, queue_size=3)
        rospy.loginfo('[GIMBAL_CONFIG] uav=%s handoff=%s; auxiliary rays only, '
                      'fixed camera retains person identity and reporting',
                      self.uav_id, self._gimbal_handoff_enabled)
        if self._fast_yaw_enabled:
            rospy.Subscriber('/perception/fast_tracking_view', String,
                             self._fast_tracking_view_cb, queue_size=6)
        rospy.Subscriber("/swarm/processed_camera_frame", String,
                         self._processed_camera_frame_cb, queue_size=6)
        # 2026-10-06 国家一等奖: 团队侧已确认广播
        rospy.Subscriber("/swarm/visual_streak", String, self._confirmed_cb, queue_size=10)
        rospy.Subscriber("/swarm/reobserve", String, self._reobserve_cb, queue_size=6)
        rospy.Subscriber("/swarm/uav_status", UavStatus, self._friend_status_cb)

        # ---- 发布 ----
        self.status_pub = rospy.Publisher("/swarm/uav_status", UavStatus, queue_size=5)
        self.detect_pub = rospy.Publisher("/swarm/detection", TargetDetection, queue_size=10)
        self._candidate_missed_pub = rospy.Publisher(
            "/swarm/visual_candidate_missed", String, queue_size=10)
        # 合规 SLAM 栅格共享：manager 合并多机占用并集（RLE JSON）
        self.occ_pub = rospy.Publisher("/swarm/occupancy_grid", String, queue_size=1)
        self.vel_pub = rospy.Publisher("/%s/mavros/setpoint_velocity/cmd_vel" % uav_id,
                                       TwistStamped, queue_size=5)
        # 2026-10-03：位置设定点输出通道。
        # 实测（本 VM，PX4 v1.13.2 + typhoon_h480 + robocup.world）：
        #   纯速度设定点 setpoint_velocity/cmd_vel 的跟踪能力只有指令的 ~20%
        #   （指令 vz=+1.0，真实爬升 ~0.16 m/s；且在地面上 vz>0 根本起不来，
        #     垂向平衡点卡在 z≈0.8m，飞机趴在地上 forever）；
        #   位置设定点 setpoint_position/local 正常：3s 爬到 2.3m 并稳住。
        # 因此默认走位置模式：把 _send_vel 算出的速度积分成位置设定点下发，
        # 水平逻辑/避障/限幅全部保留不变；FLIGHT_OUTPUT=vel 可切回旧行为。
        self.pos_pub = rospy.Publisher("/%s/mavros/setpoint_position/local" % uav_id,
                                       PoseStamped, queue_size=5)
        self._pos_sp = None          # 本地 ENU 位置设定点 [x, y, z]
        self._pos_sp_t = 0.0

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
        """Read-only conversion using the official fixed spawn offset.

        A MAVROS EKF reset has no independent world anchor.  Altering this
        offset made the agent and camera disagree, then turned a bad EKF pose
        into a fictitious boundary excursion.  Quarantine that sortie instead.
        """
        if (self.local_xy is None or self.offset is None or
                getattr(self, '_localization_fault', False)):
            return None
        wx = self.local_xy[0] + self.offset[0]
        wy = self.local_xy[1] + self.offset[1]
        _grid = getattr(self, 'grid', None)
        if _grid is not None:
            bounds = estimated_world_hard_bounds(_grid)
        else:
            bounds = (MAP_X_MIN * 1.5, MAP_X_MAX * 1.5,
                      MAP_Y_MIN * 1.5, MAP_Y_MAX * 1.5)
        xmin, xmax, ymin, ymax = bounds
        if not (math.isfinite(wx) and math.isfinite(wy) and
                xmin <= wx <= xmax and ymin <= wy <= ymax):
            self._mark_localization_fault('fixed-frame pose outside plausible map')
            return None
        self._last_good_world = (wx, wy)
        return (wx, wy)

    def _world_trusted(self):
        """位姿失信后保持禁止世界任务；短时观察窗口也不得做贴近判定。"""
        return (not getattr(self, '_localization_fault', False) and
                rospy.Time.now().to_sec() > getattr(self, "_world_trust_until", 0.0))

    def _mark_localization_fault(self, reason):
        """Latch bad MAVROS positioning; never invent a replacement origin."""
        if getattr(self, '_localization_fault', False):
            return
        self._localization_fault = True
        self._last_cmd_v = (0.0, 0.0)
        self._last_flight_v = (0.0, 0.0)
        rospy.logerr('[%s] MAVROS 定位失信：%s；固定 offset=(%.2f,%.2f)，停止世界任务并安全降落',
                     self.uav_id, reason, self.offset[0], self.offset[1])

    def _calibrate_offset(self):
        """确定局部→世界的恒定平移（世界系 = MAVROS 局部系 + offset）。

        世界系与各机 MAVROS 局部系轴向平行、仅原点不同；偏移由
        run_match.sh 根据官方固定起飞点注入。缺参即失败，不用仿真真值补偿。
        """
        if os.environ.get("SWARM_CALIB") == "truth":
            rospy.logerr("[%s] 禁止 SWARM_CALIB=truth：正式节点不读取仿真真值",
                         self.uav_id)
            return False
        ox = _param_float("~world_offset_x")
        oy = _param_float("~world_offset_y")
        if ox is not None and oy is not None:
            self.offset = (ox, oy)
            self._offset_param = (ox, oy)   # 锚定基准：起飞点世界坐标
            rospy.loginfo("[%s] 坐标系标定（参数）：offset=(%.2f, %.2f)",
                          self.uav_id, ox, oy)
            return True

        rospy.logerr(
            "[%s] 标定失败：未注入 ~world_offset_x/y 参数。正式链路请由启动脚本"
            "按起飞点注入", self.uav_id)
        return False

    # ---------------- 回调 ----------------
    def _state_cb(self, msg):
        previous = getattr(self, 'state', None)
        fields = ('connected', 'armed', 'mode')
        if previous is None or any(getattr(previous, k, None) != getattr(msg, k, None)
                                   for k in fields):
            rospy.loginfo('[FCU_STATE] uav=%s connected=%s armed=%s mode=%s z=%s',
                          self.uav_id, msg.connected, msg.armed, msg.mode,
                          getattr(self, 'local_z', None))
        self.state = msg

    def _local_cb(self, msg):
        if MOTION_SAFETY:
            sample = msg.header.stamp.to_sec()
            now = rospy.Time.now().to_sec()
            q = msg.pose.orientation
            p = msg.pose.position
            if (not 0. <= now-sample <= .5 or
                    sample <= getattr(self,'_pose_sample_s',-1e18) or
                    not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) or
                    abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .02):
                return  # receipt time cannot turn a delayed pose into fresh data
            self._pose_sample_s = sample
        self.local_z = msg.pose.position.z
        if not getattr(getattr(self, 'state', None), 'armed', True):
            stamp = msg.header.stamp.to_sec()
            q = msg.pose.orientation
            tilt = math.acos(max(-1., min(1., 1.-2.*(q.x*q.x+q.y*q.y))))
            samples = getattr(self, '_takeoff_ground_samples', [])
            if not samples or stamp > samples[-1][0]:
                samples = [p for p in samples if stamp-p[0] <= 2.]
                samples.append((stamp, self.local_z, tilt))
                self._takeoff_ground_samples = samples
        new_xy = (msg.pose.position.x, msg.pose.position.y)
        # Freshness above uses receipt time; displacement and velocity must
        # use the sampled pose time. Queued callbacks can arrive together.
        now = sample if MOTION_SAFETY else rospy.Time.now().to_sec()
        # The camera and the agent use the same official spawn offset. A pose
        # jump has no legal independent world reference, so do not rewrite that
        # offset to make an impossible MAVROS trajectory look continuous.
        if (self._anchor_done and self.local_xy is not None and
                not getattr(self, '_localization_fault', False)):
            dt = now - self._local_prev_t
            if 0.0 < dt < 1.0:
                step = math.hypot(new_xy[0] - self.local_xy[0],
                                  new_xy[1] - self.local_xy[1])
                window = getattr(self, '_ekf_window', None)
                if window is None:
                    window = []
                window.append((new_xy, now))
                while len(window) > 2 and now - window[0][1] > 1.0:
                    window.pop(0)
                self._ekf_window = window
                reason = None
                single_limit = max(EKF_JUMP_MIN_M, MAX_SPEED * dt * 1.2)
                if step > single_limit:
                    reason = '单帧位移 %.2fm / %.3fs 超过 %.2fm' % (
                        step, dt, single_limit)
                elif len(window) >= 3:
                    origin_xy, origin_t = window[0]
                    window_dt = now - origin_t
                    if window_dt > 0.10:
                        window_dist = math.hypot(new_xy[0] - origin_xy[0],
                                                 new_xy[1] - origin_xy[1])
                        if window_dist > MAX_SPEED * window_dt * 1.2:
                            reason = '累计位移 %.2fm / %.2fs 超过 %.1fm/s' % (
                                window_dist, window_dt, MAX_SPEED * 1.2)
                # Command-relative residual catches slower EKF drift before
                # it reaches a plausible but wrong position at the boundary.
                cmd = getattr(self, '_last_cmd_v', None)
                cmd_speed = math.hypot(*cmd) if cmd is not None else 0.0
                peak = getattr(self, '_world_cmd_peak', 0.0)
                peak_t = getattr(self, '_world_cmd_peak_t', now)
                peak = max(cmd_speed, peak - 1.5 * max(0.0, now - peak_t))
                self._world_cmd_peak = peak
                self._world_cmd_peak_t = now
                allowed = (float(os.environ.get('WORLD_VEL_CLAMP_BASE', '1.2'))
                           + 1.3 * peak) * dt
                if step - allowed >= WORLD_TRUST_CLAMP_EXCESS_M:
                    reason = '位姿位移超出指令物理上限 %.2fm（%.2fm / %.3fs）' % (
                        step - allowed, step, dt)
                if reason is not None:
                    self._mark_localization_fault(reason)
            elif dt >= 1.0:
                # Callback gap is not a measurable velocity; the next fresh
                # sequence starts a new window, while stale-pose gating holds.
                self._ekf_window = [(new_xy, now)]
        elif not getattr(self, '_anchor_done', False):
            self._ekf_window = [(new_xy, now)]
        # 维护位置估计轨迹（EST_GUARD 闸门的数据源；必须在 anchor 闸门之外每帧调用，
        # 否则 _est_hist 恒为空 → _est_trustworthy 恒 False → 飞机永远悬停不走）
        self._est_track(new_xy, now)
        self.local_xy = new_xy
        self._local_prev_t = now
        q = msg.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # v11 复盘（2026-10-07）：俯仰/横滚供 SLAM 倾斜帧丢弃（SLAM_TILT_RAD）
        self.pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        self.roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                               1.0 - 2.0 * (q.x * q.x + q.y * q.y))

    def _measured_velocity_cb(self,msg):
        sample=msg.header.stamp.to_sec()
        velocity=(msg.twist.linear.x,msg.twist.linear.y)
        old=getattr(self,'_measured_velocity',None)
        now=rospy.Time.now().to_sec()
        if (0. <= now-sample <= .5 and all(math.isfinite(v) for v in velocity) and
                (old is None or sample>old[0])):
            self._measured_velocity=(sample,velocity)

    def _est_track(self, xy, now):
        """维护最近 EST_WIN_S 秒的位置轨迹（判位置估计是否可信）。"""
        tr = getattr(self, '_est_hist', None)
        if tr is None:
            tr = self._est_hist = []
        tr.append((now, xy[0], xy[1]))
        while len(tr) > 2 and now - tr[0][0] > max(EST_WIN_S, 0.05):
            tr.pop(0)

    def _est_recent_speed(self):
        """近 EST_WIN_S 秒的平均水平速度估计；数据不足返回 None。"""
        tr = getattr(self, '_est_hist', None)
        if not tr or len(tr) < 2:
            return None
        (t0, x0, y0), (t1, x1, y1) = tr[0], tr[-1]
        dt = t1 - t0
        if dt <= 1e-6:
            return None
        return math.hypot(x1 - x0, y1 - y0) / dt

    def _est_trustworthy(self):
        """位置估计是否可信（EST_GUARD）。

        实测（2026-10-03 两场）：飞机贴楼后 PX4 位置估计崩坏 —— MAVROS 自己
        报出 19.64 m/s 的水平速度（物理上限 5 m/s），local/world 在 100m 量级
        上振荡，而真值静止在楼边 (6.89,-9.32,0.34)。此时任何基于 world_xy 的
        决策（越界回收 / 栅格避障 / A* 起点）都是垃圾：`越界主动回收 → (0,3)
        当前(63.2,-66.1)` 就是在追幻影。判据只用"近窗口内位移是否物理可能"，
        不依赖任何真值，合规。
        """
        if getattr(self, '_localization_fault', False):
            return False
        if not EST_GUARD:
            return True
        if not getattr(self, '_anchor_done', False):
            return True          # 起飞前不拦（预热阶段 local_xy 可能还没就绪）
        if self.local_xy is None:
            return False
        tr = getattr(self, '_est_hist', None)
        if not tr or len(tr) < 2:
            return False
        if rospy.Time.now().to_sec() - tr[-1][0] > 1.0:
            return False         # 位置流断了 1s 以上，同样不可信
        v = self._est_recent_speed()
        if v is None:
            return False
        return v <= MAX_SPEED * EST_V_RATIO

    def _scan_cb(self, msg):
        if MOTION_SAFETY:
            sample=msg.header.stamp.to_sec()
            if (not 0. <= rospy.Time.now().to_sec()-sample <= RADAR_FRESH_S or
                    sample <= getattr(self,'_scan_t',-1e18)):
                return
        self._scan = msg
        stamped = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
        self._scan_t = stamped if stamped > 0.0 else rospy.Time.now().to_sec()
        self._slam_update()

    def _slam_update(self):
        """合规 SLAM：把最新雷达帧射线投射进占用栅格（节流）。

        扫到有效回波 → mark occupied（slam_grid.SlamGrid，含增量膨胀 +
        墙体深度填充）。发现新障碍且当前有路径时触发 A* 重规划（节流，
        且不覆盖规划线程里更新的追踪请求）。栅格有更新时按 SLAM_PUB_SEC
        节流发布 RLE 快照给 manager 合并。
        """
        if (self._scan is None or self.world_xy is None or
                getattr(self, '_localization_fault', False)):
            return
        now = self._scan_t
        if now - self._slam_last_t < SLAM_PERIOD:
            return
        self._slam_last_t = now
        # v11 复盘（2026-10-07）：机体倾斜时 2D 雷达平面不再水平，前向射线随
        # 俯仰下压打地（z<2m 时打地点落在量程内），把地面标成占用墙 → 起飞
        # 航迹前方一条假墙，A* 全程绕行/无解。倾斜超阈值丢弃整帧。
        if abs(getattr(self, 'pitch', 0.0)) > SLAM_TILT_RAD or \
                abs(getattr(self, 'roll', 0.0)) > SLAM_TILT_RAD:
            return
        try:
            marked = self.slam.mark_scan(self.world_xy, self.yaw, self._scan,
                                         SLAM_MAX_RANGE, SLAM_DEPTH_CELLS,
                                         sample_s=now)
        except Exception as exc:
            rospy.logerr_throttle(5.0, '[%s] 激光建图失败: %s', self.uav_id, exc)
            return
        if marked > 0:
            self._slam_dirty = True
        # 只有当前路径前方被新栅格挡住时才失效重规划。雷达空闲射线清除
        # 动态占用或地图其它区域更新，不能让六机每两秒重复全图 A*。
        if marked > 0 and self.path and self.path_target is not None \
                and now - self._slam_replan_t >= SLAM_REPLAN_SEC \
                and remaining_path_blocked(self.grid, self.path, self.world_xy):
            with self._plan_lock:
                pending = self._plan_pending is not None
            if not pending:
                self._slam_replan_t = now
                self._request_plan(self.path_target)
        # 发布栅格快照（节流）
        if now - self._slam_pub_t >= SLAM_PUB_SEC:
            # Publish evidence heartbeat even when occupancy bits are unchanged.
            # A final dirty frame must not wait forever for another changed cell.
            import uuid
            if not hasattr(self, '_slam_epoch'):
                self._slam_epoch = uuid.uuid4().hex
                self._slam_sequence = 0
            try:
                self.occ_pub.publish(String(data=json.dumps({
                    "uav_id": self.uav_id,
                    "width": self.slam.width,
                    "height": self.slam.height,
                    "resolution": self.slam.resolution,
                    "origin": [self.slam.origin[0], self.slam.origin[1]],
                    "rle": self.slam.to_rle(),
                    "schema": 2, "epoch": self._slam_epoch,
                    "sequence": self._slam_sequence+1, "sample_s": now,
                    "evidence": self.slam.evidence_snapshot(now),
                })))
                self._slam_sequence += 1
                self._slam_pub_t = now
                self._slam_dirty = False
            except Exception as exc:
                self._slam_dirty = True
                rospy.logwarn_throttle(5., '[%s] map publish failed: %s', self.uav_id, exc)

    def _discard_old_plan(self):
        """Stop the previous task now; a late planner result must not revive it."""
        with self._plan_lock:
            self._plan_epoch += 1
            self._plan_pending = None
            self.path = []
            self.path_target = None
            self._plan_fail_t = 0.0
        self._last_local_goal = None
        self._last_flight_v = (0.0, 0.0)

    def _stage_interrupted_camera_cue(self, now):
        """Keep one exact-image direction while search cancellation precedes pursuit."""
        cue = getattr(self, '_camera_cue', None)
        if (getattr(self, '_interrupted_camera_cue', None) is not None or cue is None or
                getattr(getattr(self, 'assignment', None), 'task_type', None) not in (None, 0) or
                getattr(self, '_landing', False) or not getattr(self, '_takeoff_done', False) or
                not self._world_trusted() or self.world_xy is None or
                not getattr(getattr(self, 'state', None), 'armed', False) or
                getattr(getattr(self, 'state', None), 'mode', '') != 'OFFBOARD' or
                cue.get('xy') is not None or not cue.get('interrupt_revisit_ok', False)):
            return
        try:
            if attention_expired(cue, now) or failed_view(cue, self.uav_id, now) is None:
                return
            saved = dict(cue, view_origin=dict(cue['view_origin']))
        except (KeyError, TypeError, ValueError, OverflowError):
            return
        # Single bounded slot; no movement, new person position or renewed deadline.
        self._interrupted_camera_cue = dict(cue=saved, ready_s=now+.25)

    @serialized_visual_task
    def _finish_interrupted_camera_cue(self, assignment=None):
        pending = getattr(self, '_interrupted_camera_cue', None)
        if pending is None:
            return
        now = rospy.Time.now().to_sec()
        wait_age = now-(pending['ready_s']-.25)
        if not math.isfinite(wait_age) or wait_age < 0.:
            self.__dict__.pop('_interrupted_camera_cue', None)
            return  # a ROS clock reset cannot keep the slot occupied indefinitely
        if assignment is None and wait_age < .25:
            return
        # Consume once before validating/publishing, including cancellation-only
        # control sweeps. Assignment and control callbacks share this RLock.
        pending = self.__dict__.pop('_interrupted_camera_cue', None)
        cue = pending['cue']
        if (getattr(assignment, 'task_type', 0) not in (0, 1) or
                getattr(self, '_landing', False) or not getattr(self, '_takeoff_done', False) or
                not self._world_trusted() or self.world_xy is None or
                not getattr(getattr(self, 'state', None), 'armed', False) or
                getattr(getattr(self, 'state', None), 'mode', '') != 'OFFBOARD'):
            return
        try:
            if attention_expired(cue, now) or cue.get('xy') is not None:
                return
            paired = failed_view(cue, self.uav_id, now)
            if paired is None:
                return  # never fall back to receipt-time position or a renewed image age
            tid = cue.get('target_id')
            if cue['color'] != 'red' and tid:
                if self._stable_other_owner(tid, now, bearing=cue['bearing']):
                    return
                own = getattr(self, '_visual_observations', {}).get(tid, {})
                if (own.get('eligible', False) and own.get('source_uav') == self.uav_id and
                        cue['sample_s'] <= own.get('sample_s', -1e18) <= now and
                        now-own['sample_s'] <= 1.0):
                    dx, dy = own['x']-paired['x'], own['y']-paired['y']
                    angle = math.atan2(math.sin(math.atan2(dy, dx)-cue['bearing']),
                                       math.cos(math.atan2(dy, dx)-cue['bearing']))
                    if math.hypot(dx, dy) <= 20. and abs(angle) <= math.radians(15.):
                        return  # fresh, spatially matching localization already reached the manager
            # An unlocalized red cue has no global t4/t5 instance binding;
            # a same-colour assignment alone must not erase the other red person.
            publisher = getattr(self, '_candidate_missed_pub', None)
            if publisher is not None:
                publisher.publish(String(data=json.dumps(paired)))
                rospy.loginfo('[INTERRUPTED_VIEW_REVISIT] uav=%s candidate=%s image=%.3f; '
                              'camera origin/direction only, no person coordinate',
                              self.uav_id, cue['candidate_id'], cue['sample_s'])
        except (KeyError, TypeError, ValueError, AttributeError, OverflowError):
            return

    @serialized_visual_task
    def _assign_cb(self, msg):
        if not self._accept_segment_message(msg):
            return
        # Every agent sees the shared assignment topic. Remember ownership so
        # another aircraft's active target does not repeatedly stop searchers
        # for image-only cues of the same blue/white/green/brown person.
        roles = getattr(self, '_team_tracks', None)
        if roles is None:
            roles = self._team_tracks = {}
        tid = getattr(msg, 'target_id', '')
        if msg.task_type == 1 and tid:
            # Each aircraft has one task. A handoff or reassignment can arrive
            # before the cancellation of its old target; do not retain a ghost
            # owner that suppresses another searcher's camera cue.
            for key in list(roles):
                if key != tid:
                    roles[key].discard(msg.uav_id)
                    if not roles[key]:
                        roles.pop(key, None)
            roles.setdefault(tid, set()).add(msg.uav_id)
        elif msg.task_type in (0, 255):
            for key in list(roles):
                if msg.task_type == 255 and tid and key != tid:
                    continue
                roles[key].discard(msg.uav_id)
                if not roles[key]:
                    roles.pop(key, None)
        if msg.uav_id != self.uav_id:
            return
        if msg.task_type == 255:
            old_tid = getattr(self, '_track_assigned_id', None)
            if old_tid and msg.target_id and msg.target_id != old_tid:
                return
            self._stage_interrupted_camera_cue(rospy.Time.now().to_sec())
            self.__dict__.setdefault('_search_view_inbox', SearchViewInbox()).assign(
                msg, rospy.Time.now().to_sec())
            self._discard_old_plan()
            self.assignment = None
            self._tracking_image = None
            self._fast_tracking_image = None
            self._tracking_altitude = None
            self._observation_detour = None
            self._tracking_blocked_since = None
            self._search_scan_key = None
            self._search_scan_done = False
            self._search_scan_failed = False
            self._search_scan_seen = set()
            self._search_scan_first = {}
            self._search_scan_directed_first = None
            self._search_scan_directed_last = None
            self._search_scan_directed_done = False
            self._camera_cue = None
            self._track_assigned_id = None
            self._target_to_orbit = None
            self._orbit_target = None
            self._orbit_center = None
            self._orbit_prev = None
            self._confirm_start = 0.0
            self._last_confirm_t = 0.0
            self._confirm_close = False
            self._confirm_close_tid = None
            self._confirm_close_until = 0.0
            self._look_at = None
            self._last_local_goal = None
            return
        if msg.task_type == 1 and tid != getattr(self, '_track_assigned_id', None):
            self._stage_interrupted_camera_cue(rospy.Time.now().to_sec())
        self._finish_interrupted_camera_cue(msg)
        self.__dict__.setdefault('_search_view_inbox', SearchViewInbox()).assign(
            msg, rospy.Time.now().to_sec())
        if msg.task_type == 0:
            scan_key = (msg.cell_ix, msg.cell_iy,
                        round(msg.target_x, 2), round(msg.target_y, 2))
            if scan_key != self._search_scan_key:
                self._discard_old_plan()
                # A new search waypoint cannot cancel a bounded look at a
                # person. Keep the latest assignment for resumption afterwards.
                self._search_scan_key = scan_key
                self._search_scan_yaw0 = None
                self._search_scan_phase = 0
                self._search_scan_settle_t = None
                self._search_scan_started_t = None
                self._search_scan_done = False
                self._search_scan_failed = False
                self._search_scan_seen = set()
                self._search_scan_first = {}
                self._search_scan_directed_first = None
                self._search_scan_directed_last = None
                self._search_scan_directed_done = False
        else:
            self._search_scan_key = None
            self._search_scan_done = False
            self._search_scan_failed = False
        # Remove the search path BEFORE exposing the new pursuit assignment to
        # the control loop. A planner result already in flight is invalidated
        # by the epoch, and _stream_last_v can only repeat zero while replanning.
        same_track = (tid and tid == getattr(self, '_track_assigned_id', None))
        if msg.task_type == 1 and not same_track:
            self._discard_old_plan()
            self._camera_cue = None
        if msg.task_type != 1 or not same_track:
            self._tracking_image = None
            self._fast_tracking_image = None
            self._tracking_altitude = None
            self._observation_detour = None
            self._observation_detour_next = 0.0
            self._tracking_blocked_since = None
        self.assignment = msg
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
                sample = self._t_seen.get(tid, 0.0)
                if (self._target_to_orbit is None and sample and
                        rospy.Time.now().to_sec() - sample >=
                        TARGET_STALE + STALE_SEARCH_GRACE):
                    return  # 等新的真实图像，不沿 manager 的旧坐标复飞
                if self._target_to_orbit is None:
                    self._discard_old_plan()
                self._target_to_orbit = (msg.target_x, msg.target_y)
                return
            self._track_assigned_id = tid if tid else None
            rospy.loginfo("[%s] 收到目标追踪任务 %s @ (%.1f, %.1f)",
                          self.uav_id, tid if tid else '?',
                          msg.target_x, msg.target_y)
            self._orbit_target = None  # 新目标：让 _control 飞向目标点后自动盘旋
            self._target_to_orbit = (msg.target_x, msg.target_y)
            # 2026-10-06 国家一等奖: 切目标时清贴脸标志, 防止上一个目标的
            # _confirm_close 状态被新目标继承导致 yolo 误差超标.
            if self._confirm_close:
                self._confirm_close = False
                self._confirm_close_tid = None
                self._orbit_prev = None
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
            if msg.task_type == 0:
                self._target_to_orbit = None
                self._orbit_target = None
                self._orbit_center = None
                self._confirm_start = self._last_confirm_t = 0.0
                self._confirm_close = False
                self._confirm_close_tid = None
                self._close_range_ref = None

    @serialized_visual_task
    def _target_cb(self, msg):
        """缓存本队视觉桥的目标观测；不采信其他节点宣称的消除状态。"""
        if msg.eliminated:
            # 裁判反馈不进入正式节点，单条团队消息不能证明官方已删除。
            # 视觉断流由时间戳与 manager 的租约超时释放。
            return
        now = rospy.Time.now().to_sec()
        # header.stamp 来自最后一次真实图像。bridge 的保活帧仍会到达，
        # 但不能重置目标新鲜度；否则 8s 内的冻结位置每帧都变成“新观测”。
        sample = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
        if sample < getattr(self, '_hypothesis_switch_s', {}).get(msg.target_id, -1e18):
            return
        if not math.isfinite(sample) or sample <= 0.0 or sample > now:
            return
        previous = self._t_seen.get(msg.target_id, 0.0)
        if sample < previous - 0.01:
            return  # 倒序相机帧不能把追踪点拉回旧位置
        if sample > previous + 1e-6:
            getattr(self, '_stale_search_until', {}).pop(msg.target_id, None)
        self.targets[msg.target_id] = (msg.x, msg.y, msg.vx, msg.vy)
        self._target_state[msg.target_id] = (int(msg.state), sample)
        self._t_seen[msg.target_id] = max(previous, sample)
        if not hasattr(self, '_report_state_sample'):
            self._report_state_sample = {}
        self._report_state_sample[msg.target_id] = sample
        self._escape_bookkeeping(msg.target_id)   # v23：观测速度 → 逃跑生命周期

    @serialized_visual_task
    def _discovery_state_cb(self, msg):
        """Manager-selected exploratory position; never visual confirmation."""
        now = rospy.Time.now().to_sec()
        sample = msg.header.stamp.to_sec()
        tid = str(msg.target_id)
        if sample < getattr(self, '_hypothesis_switch_s', {}).get(tid, -1e18):
            return
        if (tid not in ('t0', 't1', 't2', 't3') or msg.eliminated or
                not all(math.isfinite(v) for v in (sample, msg.x, msg.y)) or
                not 0. <= now-sample <= 1. or
                sample <= self._t_seen.get(tid, -1e18)):
            return
        report_sample = getattr(self, '_report_state_sample', {}).get(tid, -1e18)
        if 0. <= now-report_sample <= .5:
            return  # preserve fresh bridge velocity instead of replacing it by zero
        old = self.targets.get(tid)
        if (getattr(getattr(self, 'assignment', None), 'target_id', None) == tid and
                old is not None and now-self._t_seen.get(tid, 0.) <= 1. and
                math.hypot(old[0]-msg.x, old[1]-msg.y) > 3.):
            return
        # This target-state cache steers approach only. Do not fill own-camera
        # evidence, velocity, visual-streak counters, or report eligibility.
        self.targets[tid] = (msg.x, msg.y, 0., 0.)
        self._t_seen[tid] = sample

    def _friend_status_cb(self, msg):
        """接收友机位置和高度，用于避碰"""
        if msg.uav_id != self.uav_id:
            self._friend_positions[msg.uav_id] = (msg.x, msg.y, msg.z)
            self._friend_seen[msg.uav_id] = rospy.Time.now().to_sec()

    def _confirmed_cb(self, msg):
        """React to a team visual streak by stabilizing an assigned nearby view.

        This message is a flight-mode hint, not confirmation of accuracy or
        elimination. The assigned-target and physical approach checks remain.
        """
        try:
            data = msg.data
            if not data or not data.startswith("tid:"):
                return
            tid = data[4:].strip()
        except Exception:
            return
        if not tid:
            return
        # 团队确认是目标级广播，不能把“收到广播”当作本机到圈。
        # 上轮 agent_5 距 t2 43m 仍因被指派而进入 0.95m/s 贴脸模式，
        # 此后数十秒追不上目标。只有本机正在执行这个目标、视觉仍新鲜、
        # 实际已在盘旋圈且 LOS 可见，才允许切低速。
        is_my_target = (
            str(getattr(self, '_track_assigned_id', None)) == tid or
            str(getattr(self, '_orbit_target', None)) == tid)
        target = self.targets.get(tid)
        wx = self.world_xy
        now = rospy.Time.now().to_sec()
        near_and_visible = (target is not None and wx is not None and
                            now - self._t_seen.get(tid, 0.0) <= TARGET_TTL and
                            math.hypot(target[0] - wx[0], target[1] - wx[1]) <= ORBIT_RADIUS and
                            self.los.visible(wx[0], wx[1], target[0], target[1]))
        if is_my_target and near_and_visible and self._world_trusted():
            # 2026-10-07 五分钟冲刺: manager 团队确认作为兜底入口 (到圈即贴脸后
            # 此处幂等跳过; 若 agent 侧未触发——如目标中途换机接力——由此补上)
            self._enter_close_mode(tid)
        else:
            # 不归本机: 若本机处于贴脸锁定期内, 不切走 (避免 manager 4s 间隔
            # 同时确认多目标导致贴脸反复重置). 仅打印 debug.
            if self._confirm_close and self._confirm_close_tid != tid:
                rospy.loginfo(
                    "[%s] 忽略 confirm(tid=%s): 本机贴脸锁定 %s 至 %.1fs",
                    self.uav_id, tid, self._confirm_close_tid,
                    self._confirm_close_until - rospy.Time.now().to_sec(),
                )

    def _stable_other_owner(self, tid, now, xy=None, bearing=None):
        """Suppress duplicate attention only for a spatially matching person.

        A colour slot is not an identity: a peer watching a blue distractor
        on the other side of the map must not suppress a new blue candidate.
        """
        for uid in tuple(getattr(self, '_team_tracks', {}).get(tid, set())):
            if uid == self.uav_id:
                continue
            evidence = getattr(self, '_team_visual', {}).get((uid, tid))
            if (evidence is not None and evidence.get('hits', 0) >= 3 and
                    evidence['sample_s'] - evidence['since_s'] >= 0.4 - 1e-6 and
                    0.0 <= now - evidence['sample_s'] <= 1.0):
                ex, ey = evidence.get('x'), evidence.get('y')
                if ex is None or ey is None:
                    continue
                if xy is not None:
                    if math.hypot(ex-xy[0], ey-xy[1]) <= 3.0:
                        return True
                elif bearing is not None and self.world_xy is not None:
                    wx, wy = self.world_xy
                    # A bearing alone is ambiguous at long range. Only a
                    # nearby peer observation on the same ray can take over.
                    expected = math.atan2(ey-wy, ex-wx)
                    error = abs(math.atan2(math.sin(expected-bearing),
                                           math.cos(expected-bearing)))
                    if math.hypot(ex-wx, ey-wy) <= 20.0 and error <= math.radians(15.):
                        return True
        return False

    @serialized_visual_task
    def _cancel_gimbal_cue(self, result, now):
        cue = getattr(self, '_gimbal_cue', None)
        if cue is None:
            return
        self._gimbal_cue = None
        retry = {cid: until for cid, until in
                 getattr(self, '_gimbal_retry_after', {}).items() if until > now}
        retry[cue['candidate_id']] = now + 12.
        self._gimbal_retry_after = dict(sorted(retry.items(),
                                             key=lambda item: item[1])[-64:])
        publisher = getattr(self, '_gimbal_feedback_pub', None)
        if publisher is not None:
            publisher.publish(String(data=json.dumps(dict(
                source_uav=self.uav_id, candidate_id=cue['candidate_id'],
                sample_s=now, result=result))))
        rospy.loginfo('[GIMBAL_HANDOFF_END] uav=%s cid=%s reason=%s; '
                      'no cross-camera identity or report credit',
                      self.uav_id, cue['candidate_id'], result)

    @serialized_visual_task
    def _gimbal_candidate_cb(self, msg):
        """Receive measured image rays, never person positions or assignments."""
        if not getattr(self, '_gimbal_handoff_enabled', False):
            return
        try:
            now = rospy.Time.now().to_sec()
            data = validate_handoff(json.loads(msg.data), self.uav_id, now)
            if (getattr(getattr(self, 'assignment', None), 'task_type', None) != 0 or
                    self.world_xy is None or not self._world_trusted() or
                    getattr(self, '_landing', False) or
                    not getattr(self, '_takeoff_done', False) or
                    getattr(self, '_camera_cue', None) is not None or
                    getattr(self, '_orbit_target', None) is not None or
                    getattr(self, '_target_to_orbit', None) is not None):
                return
            cue = getattr(self, '_gimbal_cue', None)
            if cue is not None:
                if (cue['candidate_id'] != data['candidate_id'] or
                        now >= cue['until'] or
                        data['sample_s'] <= cue['sample_s']):
                    return
                # New images can update the ray, but cannot extend the lease.
                cue.update(bearing=data['bearing'], sample_s=data['sample_s'])
                return
            if now < getattr(self, '_gimbal_retry_after', {}).get(
                    data['candidate_id'], -1e18):
                return
            self._gimbal_cue = dict(data, started=now,
                until=min(now + 6., data['until_s']), last_hold_s=now)
            rospy.loginfo('[GIMBAL_ATTENTION] uav=%s cid=%s image=%.3f '
                          'until=%.3f; stop search and turn through final safety exit',
                          self.uav_id, data['candidate_id'], data['sample_s'],
                          self._gimbal_cue['until'])
        except (ValueError, TypeError, KeyError, OverflowError):
            return

    @serialized_visual_task
    def _hold_gimbal_candidate(self):
        cue = getattr(self, '_gimbal_cue', None)
        if cue is None:
            return False
        now = rospy.Time.now().to_sec()
        if (getattr(getattr(self, 'assignment', None), 'task_type', None) != 0 or
                getattr(self, '_camera_cue', None) is not None or
                getattr(self, '_orbit_target', None) is not None or
                getattr(self, '_target_to_orbit', None) is not None or
                getattr(self, '_landing', False) or not self._world_trusted() or
                getattr(getattr(self, 'state', None), 'mode', '') != 'OFFBOARD'):
            self._cancel_gimbal_cue('preempted', now)
            return False
        if now >= cue['until'] or not 0. <= now-cue['sample_s'] <= .5:
            self._cancel_gimbal_cue('expired', now)
            return False
        if self.world_xy is None:
            self._cancel_gimbal_cue('preempted', now)
            return False
        wx, wy = self.world_xy
        self._look_at = (wx + 10.*math.cos(cue['bearing']),
                         wy + 10.*math.sin(cue['bearing']))
        last = cue['last_hold_s']
        if getattr(self, '_search_scan_started_t', None) is not None:
            self._search_scan_started_t += max(0., min(now-last, .25))
        cue['last_hold_s'] = now
        # No cloud range estimate: only braking and measured-ray yaw. The
        # fixed camera independently establishes identity and localization.
        self._send_vel(0., 0.)
        return True

    def _attention_attempt(self, color, bearing, xy, now):
        """Bound retries by observed place/view, independently of detector IDs.

        This stores camera-derived evidence only. It is a short retry budget,
        never a static-person blacklist. A newly localized pixel cue can still
        promote immediately; moving to a different view also gets a new look.
        """
        wx = self.world_xy
        attempts = [a for a in getattr(self, '_camera_attention_attempts', [])
                    if 0.0 <= now-a['started'] <= 60.0]
        self._camera_attention_attempts = attempts[-32:]
        for attempt in reversed(self._camera_attention_attempts):
            if attempt['color'] != color:
                continue
            old_xy = attempt.get('xy')
            if xy is not None and old_xy is not None:
                same = math.hypot(xy[0]-old_xy[0], xy[1]-old_xy[1]) <= 3.0
            else:
                angle = abs(math.atan2(math.sin(bearing-attempt['bearing']),
                                       math.cos(bearing-attempt['bearing'])))
                same = (wx is not None and
                        math.hypot(wx[0]-attempt['origin'][0],
                                   wx[1]-attempt['origin'][1]) <= 3.0 and
                        angle <= math.radians(15.))
            if same:
                # Reliable localization advances the candidate once, even
                # when its pixel-only look has just exhausted its budget.
                if xy is not None and old_xy is None:
                    attempt['retry_at'] = now
                return attempt
        attempt = dict(color=color, bearing=bearing, xy=xy,
                       origin=wx, started=now, count=0, retry_at=now)
        self._camera_attention_attempts.append(attempt)
        return attempt

    def _offer_camera_attention(self, cid, color, bearing, sample, now,
                                target_id=None, xy=None, same_sample_pixel_id=None):
        """Keep a bounded attention lease, independently of search waypoints.

        New frames update where to look, never its absolute deadline. A
        localized track may promote a pixel cue once while dispatch catches up.
        """
        assignment = getattr(self, 'assignment', None)
        if assignment is not None and assignment.task_type != 0:
            return  # tracking/reacquisition/landing always has priority
        if target_id and self._stable_other_owner(target_id, now, xy, bearing):
            return
        cue = getattr(self, '_camera_cue', None)
        if cue is not None and attention_expired(cue, now):
            return  # control consumes expiry and records a missed-view revisit
        if cue is not None and now < cue['until']:
            if xy is not None and cue.get('xy') is not None:
                dt = max(0.0, min(1.0, sample-cue.get('sample_s', sample)))
                if math.hypot(xy[0]-cue['xy'][0], xy[1]-cue['xy'][1]) > 3.0+2.2*dt:
                    return  # reusing a colour slot cannot pull this view across the map
            same = (cue.get('candidate_id') == cid or
                    (xy is not None and cue.get('xy') is not None and
                     target_id is not None and cue.get('target_id') == target_id))
            # Color alone cannot associate the two red people. Only promote
            # a nearby bearing; a different candidate cannot steal attention.
            if (not same and xy is not None and cue.get('xy') is None and
                    cue.get('color') == color):
                error = (bearing - cue['bearing'] + math.pi) % (2*math.pi) - math.pi
                same = abs(error) <= math.radians(15.)
            # Perception emits the pixel cue before its localized view of the
            # SAME image. Only its explicit same-frame pixel binding permits
            # this one promotion; it never counts another image or new lease.
            same_frame_promotion = (xy is not None and cue.get('xy') is None and
                same_sample_pixel_id is not None and
                same_sample_pixel_id == cue.get('candidate_id') and
                sample == cue.get('sample_s'))
            if same_frame_promotion:
                same = True
            if (not same or sample < cue.get('sample_s', -1e18) or
                    (sample == cue.get('sample_s') and not same_frame_promotion)):
                return
            # A six-second pixel approach can outlive the four-second
            # localized hold. Do not replace its valid remaining budget with
            # an already-expired deadline when localization arrives late.
            if (xy is not None and cue.get('xy') is None and
                    now >= cue['started'] + CAMERA_HANDOFF_HOLD_S):
                return
            cue.update(bearing=bearing, sample_s=sample)
            if xy is not None:
                cue.update(xy=xy, target_id=target_id, candidate_id=cid,
                           until=cue['started'] + CAMERA_HANDOFF_HOLD_S)
                self._camera_cue_retry_after[cid] = cue['until'] + CAMERA_CUE_RETRY_S
                attempt = cue.get('attempt')
                if attempt is not None:
                    attempt.update(xy=xy, bearing=bearing,
                        retry_at=max(attempt['retry_at'],
                                     cue['until'] + CAMERA_CUE_RETRY_S))
            return
        retry = getattr(self, '_camera_cue_retry_after', {})
        if now < retry.get(cid, -1e18):
            return
        attempt = self._attention_attempt(color, bearing, xy, now)
        if now < attempt['retry_at']:
            return
        hold = CAMERA_HANDOFF_HOLD_S if xy is not None else CAMERA_CUE_HOLD_S
        attempt.update(started=now, bearing=bearing, xy=xy, origin=self.world_xy,
                       count=attempt['count']+1)
        cooldown = min(12.0, CAMERA_CUE_RETRY_S * 2**min(attempt['count']-1, 3))
        attempt['retry_at'] = now + hold + cooldown
        self._camera_cue_retry_after = {key: value for key, value in retry.items()
                                       if value > now}
        self._camera_cue_retry_after[cid] = now + hold + CAMERA_CUE_RETRY_S
        self._camera_cue = dict(candidate_id=cid, color=color, bearing=bearing,
                                sample_s=sample, started=now, until=now+hold,
                                target_id=target_id, xy=xy, attempt=attempt)
        rospy.loginfo('[%s] 人物优先补看 %s，最多 %.1fs；搜索暂挂，等待正式派遣',
                      self.uav_id, cid, hold)

    @serialized_visual_task
    def _visual_candidate_cb(self, msg):
        """Bounded own-camera attention; candidates grant no task or confirmation."""
        try:
            data = json.loads(msg.data)
            now = rospy.Time.now().to_sec()
            assignment = getattr(self, 'assignment', None)
            if (getattr(assignment, 'task_type', None) == 1 and
                    data.get('purpose') == 'camera_reacquire'):
                previous_view = getattr(self, '_tracking_image', None)
                binding = getattr(self, '_camera_instance', {}).get(assignment.target_id)
                if (previous_view is not None and binding is not None and
                        0. <= now-binding['sample_s'] <= 1. and
                        previous_view.get('world_candidate_id') is not None and
                        previous_view['world_candidate_id'] != binding['candidate_id']):
                    self._tracking_image = None
                    return
                view = (pixel_recovery_view(previous_view, data, self.uav_id,
                                            assignment.target_id, now)
                        if PIXEL_VIEW_RECOVERY else None)
                if view is None:
                    view = retain_pixel_view(previous_view, data,
                                             self.uav_id, assignment.target_id, now)
                if view is not None:
                    if view.get('image_recovery') and self.local_z is not None:
                        if view.get('recovery_origin') is None:
                            view['recovery_origin'] = self.world_xy
                        self._tracking_altitude = update_pixel_altitude_hold(
                            getattr(self, '_tracking_altitude', None), view, self.local_z,
                            max(MIN_CRUISE_ALT, 2.6),
                            min(ALT_TARGET_CAP, ALT_HARD_CEIL-ALT_HARD_MARGIN))
                        view['altitude_goal'] = self._tracking_altitude['goal']
                        rospy.loginfo_throttle(1., '[PIXEL_GEOMETRY_RECOVERY] uav=%s '
                            'tid=%s image=%.3f anchor=%.3f until=%.3f '
                            'foot_down_sin=%.3f alt_goal=%.3f approach=%s; no report',
                            self.uav_id, assignment.target_id, view['sample_s'],
                            view['position_sample_s'], view['recovery_until'],
                            view['foot_down_sin'], view['altitude_goal'],
                            PIXEL_RECOVERY_APPROACH)
                    self._tracking_image = view
                    rospy.loginfo_throttle(2., '[PIXEL_VIEW] uav=%s tid=%s image=%.3f '
                        'position_age=%.2f; yaw only, no coordinate refresh',
                        self.uav_id, assignment.target_id, view['sample_s'],
                        now-view['position_sample_s'])
                return
            if ((assignment is not None and assignment.task_type != 0) or
                    data.get('purpose') != 'camera_reacquire' or
                    data.get('source_uav') != self.uav_id or
                    data.get('real_detection') is not True):
                return
            sample, bearing, confidence = (float(data[k]) for k in
                ('sample_s', 'bearing', 'confidence'))
            cid = str(data['candidate_id'])
            # Three associated image samples can justify a bounded look for
            # every clothing colour. Coordinate/report gates remain separate.
            active = getattr(self, '_camera_cue', None)
            updating = (active is not None and active.get('candidate_id') == cid and
                        now < active['until'])
            cue_conf = 0.45 if updating else 0.55
            if (not all(math.isfinite(v) for v in (sample, bearing, confidence)) or
                    not -0.05 <= now - sample <= 0.5 or
                    not cue_conf <= confidence <= 1.0 or
                    int(data['hits']) < 3):
                return
            if self.world_xy is None:
                return
            target_for_color = {'green': 't0', 'blue': 't1',
                                'brown': 't2', 'white': 't3'}.get(cid.split(':', 1)[0])
            self._offer_camera_attention(cid, cid.split(':', 1)[0], bearing,
                                         sample, now, target_for_color)
            cue = getattr(self, '_camera_cue', None)
            if (cue is not None and cue['candidate_id'] == cid and
                    cue['sample_s'] == sample and now < cue['until']):
                self.__dict__.setdefault('_camera_frame_origins', CameraOrigins()).attach(cue)
                # This marker only permits a later direction revisit. Existing
                # attention/approach and official localization gates are unchanged.
                cue['interrupt_revisit_ok'] = (data.get('appearance_status') == 'actor' and
                                               data.get('association_unique') is True)
                # Invalid/clipped new frames clear permission to move at once.
                cue['image'] = approach_image(data, CANDIDATE_IMAGE_MAX_TILT)
                if (cue['image'] is not None and confidence >= .65 and
                        cue.get('xy') is None and not cue.get('approach_started')):
                    cue['approach_started'] = True
                    cue['approach_origin'] = self.world_xy
                    cue['until'] = cue['started'] + 6.
                    if cue.get('attempt') is not None:
                        cue['attempt']['retry_at'] = max(
                            cue['attempt']['retry_at'], cue['until']+CAMERA_CUE_RETRY_S)
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError):
            return

    def _offer_localized_search_view(self, raw, now):
        """Keep a new own-camera person visible while bridge identity matures.

        This is a four-second observation intent, not a target observation,
        assignment or report. The bridge still requires its six independent
        images before replacing a distant colour hypothesis. Red camera slots
        have no global identity here and deliberately retain their old path.
        """
        tid = raw.get('target_id')
        cue = getattr(self, '_camera_cue', None)
        sample = raw.get('sample_s')
        if (os.environ.get('LOCALIZED_SEARCH_FOLLOW', '0') == '1' and
                cue is not None and cue.get('localized_search_view') and
                raw.get('source_uav') == self.uav_id and
                cue.get('candidate_id') == 'target:%s:%s' % (tid, raw.get('world_candidate_id')) and
                isinstance(sample, (int, float)) and math.isfinite(sample) and
                sample > cue['sample_s'] and 0. <= now-sample <= .5 and
                not (raw.get('geometry_ok') is True and raw.get('report_ok') is True and
                     raw.get('person_verdict_ok') is True and
                     raw.get('appearance_status') == 'actor')):
            cue['follow_samples'] = []
            cue['image'] = None
        if (getattr(getattr(self, 'assignment', None), 'task_type', None) != 0 or
                tid not in ('t0', 't1', 't2', 't3') or
                raw.get('geometry_ok') is not True or raw.get('report_ok') is not True or
                raw.get('person_verdict_ok') is not True or raw.get('person_verdict_reason') != '' or
                raw.get('appearance_status') != 'actor' or
                getattr(self, '_landing', False) or
                getattr(self, '_track_assigned_id', None) is not None or
                getattr(self, '_target_to_orbit', None) is not None or
                getattr(self, '_orbit_target', None) is not None or
                getattr(getattr(self, 'state', None), 'mode', '') in ('AUTO.LAND', 'LAND', 'AUTO.RTL') or
                self.world_xy is None or self.local_z is None or not self._world_trusted()):
            return
        instance = raw.get('world_candidate_id')
        if (not isinstance(instance, str) or not instance or len(instance) > 128 or
                any(c.isspace() or c == '\0' for c in instance)):
            return
        view = parse_tracking_view(raw, self.uav_id, tid, now)
        if (float(raw['confidence']) < .55 or view['track_id'] < 0 or
                view['sample_s'] < getattr(self, '_hypothesis_switch_s', {}).get(tid, -1e18)):
            return
        color = {'t0': 'green', 't1': 'blue', 't2': 'brown', 't3': 'white'}[tid]
        pixel_id = raw.get('candidate_id')
        if pixel_id is not None and (not isinstance(pixel_id, str) or
                                    not pixel_id.startswith(color+':')):
            return
        sample = view['sample_s']
        samples = self.__dict__.setdefault('_localized_search_view_samples', {})
        if sample <= samples.get(tid, -1e18):
            return  # repeated or delayed camera output cannot restart a hold
        samples[tid] = sample
        cid = 'target:' + tid + ':' + instance
        cue = getattr(self, '_camera_cue', None)
        if cue is not None and cue.get('xy') is not None and cue['candidate_id'] != cid:
            return  # another local world instance cannot steal this active view
        self._offer_camera_attention(cid, color, view['bearing'], sample, now,
            target_id=tid, xy=(view['x'], view['y']), same_sample_pixel_id=pixel_id)
        cue = getattr(self, '_camera_cue', None)
        if cue is None or cue['candidate_id'] != cid or cue['sample_s'] != sample:
            return
        cue['localized_search_view'] = True
        history = cue.setdefault('follow_samples', [])
        if history and (sample-history[-1][0] > .5 or
                math.hypot(view['x']-history[-1][1], view['y']-history[-1][2]) >
                2.5*(sample-history[-1][0])+.2):
            history.clear()
        history.append((sample, view['x'], view['y']))
        cue['follow_samples'] = [item for item in history if sample-item[0] <= .6]
        cue.setdefault('follow_origin', self.world_xy)
        # Coherent new own-camera positions can sustain the existing finite
        # attention while dispatch catches up. They confer no report credit.
        cue['image'] = dict(box=view['box'], width=view['width'],
                            height=view['height'], camera_tilt=view['camera_tilt'])
        self.__dict__.setdefault('_camera_frame_origins', CameraOrigins()).attach(cue)
        rospy.loginfo_throttle(2., '[LOCALIZED_SEARCH_HOLD] uav=%s tid=%s candidate=%s '
            'image=%.3f until=%.3f; camera intent only, no observation/report credit',
            self.uav_id, tid, instance, sample, cue['until'])

    @serialized_visual_task
    def _tracking_view_cb(self, msg):
        """Image control feedback never creates a detection or confirmation."""
        assignment = getattr(self, 'assignment', None)
        if getattr(assignment, 'task_type', None) == 0:
            try:
                self._offer_localized_search_view(json.loads(msg.data),
                                                   rospy.Time.now().to_sec())
            except (ValueError, TypeError, KeyError, AttributeError, OverflowError):
                pass
            return
        if (getattr(assignment, 'task_type', None) != 1 or
                self.world_xy is None or self.local_z is None):
            return
        try:
            now = rospy.Time.now().to_sec()
            tid = assignment.target_id
            raw = json.loads(msg.data)
            local_tid = raw.get('target_id')
            if tid in ('t4', 't5') and local_tid in ('t4', 't5'):
                view = parse_tracking_view(raw, self.uav_id, local_tid, now)
                key = (local_tid, view['sample_s'])
                with self.__dict__.setdefault('_red_view_lock', threading.RLock()):
                    binding = getattr(self, '_red_view_bindings', {}).get(key)
                    continued = (bind_red_instance_view(
                        view, raw.get('world_candidate_id'),
                        getattr(self, '_camera_instance', {}).get(tid), tid, now)
                        if binding is None else None)
                    if binding is None and continued is None:
                        # Camera feedback is published before the bridge handles
                        # that image. Preserve this order without guessing identity.
                        pending = {k: v for k, v in
                                   getattr(self, '_pending_red_views', {}).items()
                                   if 0.0 <= now-k[1] <= .5}
                        pending[key] = raw
                        self._pending_red_views = dict(
                            sorted(pending.items(), key=lambda item: item[0][1])[-16:])
                        return
                view = continued if continued is not None else bind_red_view(view, binding, tid)
                if view is None:
                    return
            else:
                view = parse_tracking_view(raw, self.uav_id, tid, now)
            instance = raw.get('world_candidate_id')
            binding = getattr(self, '_camera_instance', {}).get(tid)
            if (instance is not None and binding is not None and
                    0. <= now-binding['sample_s'] <= 1. and
                    instance != binding['candidate_id']):
                return  # another nearby same-color object cannot steer this view
            view['world_candidate_id'] = instance
            if view['sample_s'] < getattr(self, '_hypothesis_switch_s', {}).get(tid, -1e18):
                return
            previous = getattr(self, '_tracking_image', None)
            if (previous is not None and previous['target_id'] == tid and
                    view['sample_s'] < previous['sample_s']):
                return
            target = self.targets.get(tid)
            if target is None:
                return
            error = math.hypot(view['x']-target[0], view['y']-target[1])
            if error > 3.0:
                return  # same-colour false candidate must not steer this assignment
            if (previous is not None and previous['target_id'] == tid and
                    view['sample_s'] == previous['sample_s'] and
                    error >= previous.get('association_error', float('inf')) and
                    not previous.get('pixel_only', False)):
                return
            view['association_error'] = error
            wx, wy = self.world_xy
            rng = math.hypot(target[0]-wx, target[1]-wy)
            if not math.isfinite(rng) or not math.isfinite(self.local_z):
                return
            previous_altitude = getattr(self, '_tracking_altitude', None)
            if previous_altitude is not None:
                previous_altitude = {key: value for key, value in previous_altitude.items()
                                     if not key.startswith('recovery_')}
            self._tracking_altitude = update_altitude_hold(
                previous_altitude, view,
                self.local_z, rng, max(MIN_CRUISE_ALT, 2.6),
                min(ALT_TARGET_CAP, ALT_HARD_CEIL-ALT_HARD_MARGIN),
                preferred_range=(ORBIT_RADIUS_CLOSE if COUPLED_VIEW_GEOMETRY and
                    view.get('geometry_ok') and view.get('person_verdict_ok') and
                    not view.get('report_ok') else None))
            view['altitude_goal'] = self._tracking_altitude['goal']
            view['needs_space'] = self._tracking_altitude.get('needs_space', False)
            view['range_floor'] = self._tracking_altitude.get('range_floor', 0.)
            self._tracking_image = view
            quality=self.__dict__.setdefault('_report_view_window',ReportViewWindow())
            quality.observe(view)
            rospy.loginfo_throttle(2.0,
                '[TRACK_VIEW] uav=%s tid=%s image=%.3f centre_v=%.1f bottom=%.1f '
                'alt_goal=%.2f range=%.2f local_tid=%s needs_space=%s range_floor=%.2f coupled=%s',
                self.uav_id, tid, view['sample_s'],
                (view['box'][1]+view['box'][3])/2., view['box'][3],
                view['altitude_goal'], rng, view.get('local_target_id', tid),
                view['needs_space'], view['range_floor'],
                self._tracking_altitude.get('coupled_geometry', False))
        except (ValueError, TypeError, KeyError, OverflowError):
            return

    def _remember_red_view_binding(self, local_tid, data, now):
        """Join our original image to its global identity across ROS callbacks."""
        if (data['source_uav'] != self.uav_id or
                data['target_id'] not in ('t4', 't5') or
                local_tid not in ('t4', 't5') or
                not 0.0 <= now-data['sample_s'] <= .5):
            return
        key = (local_tid, data['sample_s'])
        with self.__dict__.setdefault('_red_view_lock', threading.RLock()):
            bindings = {k: v for k, v in getattr(self, '_red_view_bindings', {}).items()
                        if 0.0 <= now-k[1] <= .5}
            bindings[key] = dict(data, local_target_id=local_tid)
            self._red_view_bindings = dict(
                sorted(bindings.items(), key=lambda item: item[0][1])[-16:])
            pending = {k: v for k, v in getattr(self, '_pending_red_views', {}).items()
                       if 0.0 <= now-k[1] <= .5}
            raw = pending.pop(key, None)
            self._pending_red_views = pending
        if raw is not None:
            self._tracking_view_cb(String(data=json.dumps(raw)))

    def _fresh_tracking_view(self, now):
        view = getattr(self, '_tracking_image', None)
        assignment = getattr(self, 'assignment', None)
        if (view is not None and getattr(assignment, 'task_type', None) == 1 and
                assignment.target_id == view['target_id'] and
                0.0 <= now-view['sample_s'] <= .5):
            return view
        return None

    def _hold_pixel_recovery(self, now):
        """Bound image-only height recovery suspends stale-coordinate motion.

        There is no inferred actor depth or moving waypoint here. The same
        final flight output preserves stop, laser, height and boundary guards.
        """
        view = self._fresh_tracking_view(now)
        if (not PIXEL_VIEW_RECOVERY or view is None or
                not view.get('image_recovery') or now >= view['recovery_until']):
            return False
        world = self.world_xy
        if world is None:
            return False
        wx, wy = world
        self._look_at = (wx+10.*math.cos(view['bearing']),
                         wy+10.*math.sin(view['bearing']))
        vx, vy, reason = 0., 0., 'height_only'
        if PIXEL_RECOVERY_APPROACH:
            cue=dict(sample_s=view['sample_s'],bearing=view['bearing'],xy=None,
                     started=view['position_sample_s'],
                     approach_origin=view.get('recovery_origin'),
                     image=approach_image(view, CANDIDATE_IMAGE_MAX_TILT))
            if 0. < view['foot_down_sin'] < VIEW_MIN_DOWN_SIN:
                vx,vy,reason=candidate_decision(cue,now,self.yaw,world,
                    getattr(self,'_scan',None),getattr(self,'_scan_t',-1e18))
            else:
                reason='not_shallow'
            if math.hypot(vx,vy) > 0. and getattr(self,'_friend_positions',{}):
                vx,vy=self._apply_friend_avoidance(vx,vy)
            rospy.loginfo_throttle(1., '[PIXEL_RECOVERY_MOTION] uav=%s tid=%s '
                'request=(%.2f,%.2f) remaining=%.2f reason=%s; no world waypoint',
                self.uav_id,view['target_id'],vx,vy,max(0.,view['recovery_until']-now),reason)
        self._send_vel(vx, vy)
        return True

    @serialized_visual_task
    def _fast_tracking_view_cb(self, msg):
        """Store only explicitly bound image feedback; never touch real evidence."""
        if not getattr(self, '_fast_yaw_enabled', False):
            return
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or data.get('source_uav') != self.uav_id:
                return
            now = rospy.Time.now().to_sec()
            assignment = getattr(self, 'assignment', None)
            if getattr(assignment, 'task_type', None) != 1:
                return
            real = self._fresh_tracking_view(now)
            try:
                view = validate_fast_view(data, self.uav_id, assignment.target_id,
                                          real, now,
                                          getattr(self, '_fast_tracking_image', None))
            except ValueError as exc:
                self._write_fast_yaw(now, 'reject', str(exc), 'real', data, real)
                return
            self._fast_tracking_image = view
            self._write_fast_yaw(now, 'accept', '', 'fast', view, real)
        except (ValueError, KeyError, TypeError, OverflowError):
            return

    def _fresh_fast_tracking_view(self, now):
        if not getattr(self, '_fast_yaw_enabled', False):
            return None
        assignment = getattr(self, 'assignment', None)
        view = getattr(self, '_fast_tracking_image', None)
        if getattr(assignment, 'task_type', None) != 1 or view is None:
            return None
        real = self._fresh_tracking_view(now)
        tid = assignment.target_id
        binding = getattr(self, '_camera_instance', {}).get(tid)
        if (binding is not None and 0. <= now-binding['sample_s'] <= 1. and
                binding.get('candidate_id') != view.get('bound_world_candidate_id')):
            return None
        if view['seed_s'] < getattr(self, '_hypothesis_switch_s', {}).get(tid, -1e18):
            return None
        try:
            return validate_fast_view(view, self.uav_id, tid, real, now)
        except ValueError:
            return None

    def _write_fast_yaw(self, now, event, reason, source, fast=None, real=None,
                        bearing=None, yaw_rate=None):
        log = getattr(self, '_fast_yaw_csv', None)
        if log is None:
            return
        fast, real = fast or {}, real or {}
        # Diagnostics must not make a malformed experimental message interrupt
        # the established flight stream.
        def age(key):
            try:
                return now-float(fast[key])
            except (KeyError, TypeError, ValueError, OverflowError):
                return None
        try:
            log.write(ros_time=now, event=event, reason=reason, yaw_source=source,
                      target_id=getattr(getattr(self, 'assignment', None), 'target_id', ''),
                      sample_s=fast.get('sample_s'), seed_s=fast.get('seed_s'),
                      image_age=age('sample_s'), seed_age=age('seed_s'),
                      quality=fast.get('quality'), real_sample_s=real.get('sample_s'),
                      position_sample_s=real.get('position_sample_s'),
                      bearing=bearing if bearing is not None else fast.get('bearing'),
                      real_bearing=real.get('bearing'), yaw=getattr(self, 'yaw', None),
                      yaw_rate=yaw_rate, candidate_id=fast.get('seed_candidate_id'),
                      world_candidate_id=real.get('world_candidate_id'))
        except (OSError, ValueError):
            self._fast_yaw_csv = None
            rospy.logwarn_throttle(10., '[FAST_YAW_LOG] cannot write diagnostic CSV')

    def _view_motion_context(self, now):
        """Own, fresh localized image supports near-range control, never credit."""
        view = self._fresh_tracking_view(now)
        own = self.world_xy
        altitude = getattr(self, 'local_z', None)
        if (view is None or own is None or altitude is None or
                view.get('pixel_only') or not view.get('geometry_ok') or
                getattr(self, '_landing', False) or
                not 0. <= now-view.get('position_sample_s', view['sample_s']) <= .5):
            return None
        tid = view['target_id']
        target = getattr(self, 'targets', {}).get(tid)
        if (target is None or
                not 0. <= now-getattr(self, '_t_seen', {}).get(tid, -1e18) <= 1.):
            return None
        distance = math.hypot(target[0]-own[0], target[1]-own[1])
        if not all(math.isfinite(v) for v in (*target, altitude, distance)) or not 0. < distance <= VIEW_MOTION_RANGE:
            return None
        tvx, tvy = target[2:4]
        speed = math.hypot(tvx, tvy)
        if speed > 2.5:
            tvx, tvy = tvx*2.5/speed, tvy*2.5/speed
        radius = ORBIT_RADIUS_CLOSE
        if view.get('needs_space'):
            radius = max(radius, min(VIEW_MOTION_RANGE, view.get('range_floor', radius)))
        return dict(own=own, target=target[:2], velocity=(tvx, tvy),
                    radius=radius, distance=distance)

    def _view_follow_velocity(self, vx, vy):
        context = self._view_motion_context(rospy.Time.now().to_sec())
        if context is None:
            return vx, vy
        return view_approach_velocity(vx, vy, *context['own'],
                                      *context['target'], *context['velocity'],
                                      radius=context['radius'])

    def _view_friend_conflict(self, now):
        own = self.world_xy
        if own is None:
            return False
        for fid, (fx, fy, fz) in getattr(self, '_friend_positions', {}).items():
            if now-getattr(self, '_friend_seen', {}).get(fid, -1e18) > 1.:
                continue
            friend_alt = self.local_z if fz is None else fz
            if math.hypot(math.hypot(fx-own[0], fy-own[1]), friend_alt-self.local_z) < SAFE_3D:
                return True
        return False

    @serialized_visual_task
    def _processed_camera_frame_cb(self, msg):
        """Credit cell subpoints using the manager's actual camera footprint."""
        try:
            data = json.loads(msg.data)
            assignment = getattr(self, 'assignment', None)
            if (data.get('source_uav') != self.uav_id or
                    data.get('processed') is not True):
                return
            sample, x, y, yaw, altitude, down_pitch = (float(data[k]) for k in
                ('sample_s', 'x', 'y', 'yaw', 'z', 'down_pitch'))
            now = rospy.Time.now().to_sec()
            if (not all(math.isfinite(v) for v in (sample, x, y, yaw, altitude, down_pitch)) or
                    altitude <= 0.0 or
                    not -0.05 <= now - sample <= 0.5):
                return
            if not getattr(getattr(self, 'state', None), 'armed', True):
                first, last = getattr(self, '_takeoff_camera_frames', (sample, sample))
                if sample > last:
                    first = first if sample-last <= .5 else sample
                    self._takeoff_camera_frames = (first, sample)
                elif not hasattr(self, '_takeoff_camera_frames'):
                    self._takeoff_camera_frames = (sample, sample)
            origins = self.__dict__.setdefault('_camera_frame_origins', CameraOrigins())
            origins.remember(sample, x, y, now)
            cue = getattr(self, '_camera_cue', None)
            if cue is not None and cue.get('sample_s') == sample:
                origins.attach(cue)
            if sample > getattr(self, '_search_view_image_s', -1e18):
                self._search_view_image_s = sample
                self._search_view_image_yaw = yaw
            if (assignment is None or assignment.task_type != 0 or
                    getattr(self, '_search_scan_key', None) is None or
                    sample <= getattr(self, '_search_last_frame', 0.0)):
                return
            self._search_last_frame = sample
            self._search_camera_pose = (sample, x, y, yaw, altitude, down_pitch)
            if math.hypot(x-assignment.target_x, y-assignment.target_y) > 2.5:
                return
            points = self._search_observable_points(x, y, altitude, down_pitch)
            seen = getattr(self, '_search_scan_seen', None)
            if seen is None:
                seen = self._search_scan_seen = set()
            first = getattr(self, '_search_scan_first', None)
            if first is None:
                first = self._search_scan_first = {}
            for point, xy in points.items():
                if point in seen or not in_view(x, y, yaw, *xy, altitude, down_pitch):
                    continue
                prior = first.get(point)
                if prior is None or sample-prior[1] > .7:
                    first[point] = (sample, sample)
                    continue
                first[point] = (prior[0], sample)
                if sample-prior[0] >= SEARCH_SCAN_HOLD_S:
                    seen.add(point)
        except (ValueError, KeyError, TypeError, AttributeError):
            return

    @serialized_visual_task
    def _search_view_hint_cb(self, msg):
        """Queue an assignment-bound old camera direction, never a detection."""
        try:
            state = self.__dict__.setdefault('_search_view_inbox', SearchViewInbox())
            if state.offer(json.loads(msg.data), self.uav_id, rospy.Time.now().to_sec()):
                if state.active is not None:
                    self._search_scan_done = False
                    self._search_scan_failed = False
        except (ValueError, TypeError, AttributeError):
            return

    def _hold_search_view_hint(self):
        assignment = getattr(self, 'assignment', None)
        if assignment is None or assignment.task_type != 0:
            return False
        state = getattr(self, '_search_view_inbox', None)
        if state is None:
            return False
        now = rospy.Time.now().to_sec()
        hint = state.step(now, self.world_xy,
                          getattr(self, '_search_view_image_s', -1e18),
                          self.yaw, getattr(self, '_search_view_image_yaw', float('nan')),
                          self._world_trusted())
        if hint is None:
            if state.last_result is not None:
                rospy.loginfo('[SEARCH_VIEW_HINT_END] uav=%s result=%s; no coverage credit',
                              self.uav_id, state.last_result['reason'])
                state.last_result = None
            return False
        # A hint is not a new camera cue. Hold only yaw/XY through normal
        # flight safety, ahead of grid scanning; no target or coverage writes.
        wx, wy = self.world_xy
        self._look_at = (wx+10.*math.cos(hint['bearing']), wy+10.*math.sin(hint['bearing']))
        self._search_heading = (now, self.yaw)
        self._send_vel(0.0, 0.0)
        rospy.loginfo_throttle(1., '[SEARCH_VIEW_HINT] uav=%s candidate=%s '
            'image=%.3f assignment=%.3f phase=%s; bounded yaw only, no observation credit',
            self.uav_id, hint['candidate_id'], hint['image_sample_s'], hint['assignment_s'],
            hint['phase'])
        return True

    def _search_observable_points(self, x, y, altitude, down_pitch):
        key = self._search_scan_key[:2]
        cell = self.cov_grid.cell(key)
        if cell is None:
            return {}
        samples = cell_points(self.cov_grid, cell)
        segment = getattr(self, '_search_segment', None)
        if segment is not None and segment.get('purpose') == 'building_review':
            index = segment['index']
            if not 0 <= index < len(segment['look_points']):
                return {}  # images can arrive after DONE, before its cancellation ACK
            look = segment['look_points'][index]
            raw = getattr(self, 'slam', None)
            if raw is None or not raw.is_free_raw(raw.world_to_cell(look)):
                return {}
            return {i: point for i, point in observable_points(samples, x, y,
                altitude, down_pitch, self.los.visible).items()
                if math.dist(point, look) <= 1e-6}
        return observable_points(samples, x, y,
                                 altitude, down_pitch, self.los.visible)

    def _hold_camera_candidate(self):
        cue = getattr(self, '_camera_cue', None)
        if cue is None:
            return False
        if self.assignment is not None and self.assignment.task_type != 0:
            self._camera_cue = None
            return False
        now = rospy.Time.now().to_sec()
        if self._stable_other_owner(cue.get('target_id'), now,
                                    cue.get('xy'), cue.get('bearing')):
            # A verified handoff is not a failed look. If the peer later
            # loses this person, local attention may resume immediately.
            if cue.get('attempt') is not None:
                cue['attempt'].update(retry_at=now, count=0)
            getattr(self, '_camera_cue_retry_after', {}).pop(
                cue.get('candidate_id'), None)
            self._camera_cue = None
            return False
        if attention_expired(cue, now):
            self._camera_cue = None
            # Only a real own-camera localization resolves a pixel cue. If
            # the 2 s look ended without one, remember this CAMERA POSITION
            # for later review; no actor position is inferred or reported.
            color_ids = {'green': ('t0',), 'blue': ('t1',),
                         'brown': ('t2',), 'white': ('t3',),
                         'red': ('t4', 't5')}
            observations = getattr(self, '_visual_observations', {})
            found = any(
                observations.get(tid, {}).get('eligible', False) and
                observations.get(tid, {}).get('sample_s', 0.0) >=
                cue.get('started', now)
                for tid in color_ids.get(cue.get('color'), ()))
            wx = self.world_xy
            if not found and wx is not None and self._world_trusted():
                publisher = getattr(self, '_candidate_missed_pub', None)
                if publisher is not None:
                    paired = failed_view(cue, self.uav_id, now)
                    publisher.publish(String(data=json.dumps(paired or {
                        'x': wx[0], 'y': wx[1], 'yaw': self.yaw,
                        'source_uav': self.uav_id, 'sample_s': now,
                    })))
            return False
        wx = self.world_xy
        if wx is None:
            return False
        # Pause scan time only while attention is actually controlling this
        # aircraft. A waypoint change can reset that scan during the hold.
        last = cue.get('last_hold_s', now)
        if getattr(self, '_search_scan_started_t', None) is not None:
            self._search_scan_started_t += max(0.0, min(now-last, 0.25))
        cue['last_hold_s'] = now
        self._look_at = cue.get('xy') or (
            wx[0] + 10.0 * math.cos(cue['bearing']),
            wx[1] + 10.0 * math.sin(cue['bearing']))
        vx, vy, reason = (candidate_decision(cue, now, self.yaw, wx,
                                    getattr(self, '_scan', None),
                                    getattr(self, '_scan_t', -1e18))
                  if cue.get('image') is not None and self._world_trusted()
                  else (0., 0., 'untrusted_pose_or_image'))
        if (os.environ.get('LOCALIZED_SEARCH_FOLLOW', '0') == '1' and
                cue.get('localized_search_view') and self._world_trusted()):
            vx, vy, reason = localized_follow_decision(cue, now, self.yaw, wx,
                getattr(self, '_scan', None), getattr(self, '_scan_t', -1e18),
                ORBIT_RADIUS_CLOSE)
        if reason == 'obstacle':
            vx, vy, reason = candidate_reposition(
                cue, now, self.yaw, wx, getattr(self, '_scan', None),
                getattr(self, '_scan_t', -1e18), getattr(self, 'grid', None))
        if getattr(self, '_friend_positions', {}):
            vx, vy = self._apply_friend_avoidance(vx, vy)
        rospy.loginfo_throttle(2., '[CANDIDATE_APPROACH] uav=%s candidate=%s '
                              'requested=(%.2f,%.2f) remaining=%.2f reason=%s',
                              self.uav_id, cue['candidate_id'], vx, vy,
                              max(0., cue['until']-now), reason)
        self._send_vel(vx, vy)
        return True

    @serialized_visual_task
    def _apply_visual_hypothesis(self, data):
        """Forget the old person's control history when a colour changes site.

        This is camera evidence from the bridge, never an elimination verdict.
        A peer can relocate the team position but cannot become our own image.
        """
        event = data.get('hypothesis_switch')
        if event is None:
            return True
        tid, stamp = data['target_id'], event['switch_s']
        stamps = self.__dict__.setdefault('_hypothesis_switch_s', {})
        previous = stamps.get(tid, -1e18)
        if stamp < previous:
            return False
        if stamp == previous:
            return True
        stamps[tid] = stamp
        for name in ('_visual_observations', '_eligible_observations',
                     '_last_eligible_image',
                     '_flee_run_t', '_walk_since', '_stale_search_until',
                     '_giveup_until'):
            getattr(self, name, {}).pop(tid, None)
        report_samples = getattr(self, '_report_state_sample', {})
        if report_samples.get(tid, -1e18) < stamp:
            report_samples.pop(tid, None)
        history = getattr(self, '_team_visual', {})
        for key in list(history):
            if key[1] == tid:
                history.pop(key, None)
        for name in ('_escape_spent', '_spend_mode'):
            getattr(self, name, set()).discard(tid)
        # Both topic delivery orders are valid. Never roll a newer target
        # state back merely because its switch metadata arrived afterwards.
        if data['sample_s'] >= getattr(self, '_t_seen', {}).get(tid, -1e18):
            self.__dict__.setdefault('targets', {})[tid] = (data['x'], data['y'], 0., 0.)
            self.__dict__.setdefault('_t_seen', {})[tid] = data['sample_s']
            self.__dict__.setdefault('_target_state', {})[tid] = (0, data['sample_s'])
        assignment = getattr(self, 'assignment', None)
        if getattr(assignment, 'task_type', None) == 1 and assignment.target_id == tid:
            self._discard_old_plan()
            self._tracking_image = self._tracking_altitude = None
            self._observation_detour = self._tracking_blocked_since = None
            self._reobserve_view = self._reobserve_forced = None
            self._reobserve_next = self._observation_detour_next = 0.
            self._orbit_target = self._orbit_center = self._orbit_prev = None
            self._confirm_start = self._last_confirm_t = 0.
            self._confirm_close = False
            self._confirm_close_tid = self._close_range_ref = None
            self._confirm_close_until = self._close_chase_until = 0.
            self._close_chase_tid = None
            target = self.targets[tid]
            assignment.target_x, assignment.target_y = flight_waypoint(*target[:2])
            self._target_to_orbit = target[:2]
            self._look_at = target[:2]
            rospy.loginfo('[HYPOTHESIS_REBIND] uav=%s tid=%s image=%.3f; '
                          'discard old path/view, retain assignment pending dispatch',
                          self.uav_id, tid, stamp)
        return True

    @serialized_visual_task
    def _visual_source_cb(self, msg):
        """Own images establish local vision; peer evidence only verifies owners."""
        try:
            now = rospy.Time.now().to_sec()
            allowed = {self.uav_id}
            allowed.update(getattr(self, '_friend_positions', {}))
            for owners in list(getattr(self, '_team_tracks', {}).values()):
                allowed.update(owners)
            data = parse_visual_evidence(msg.data, now, allowed)
            tid = data['target_id']
            uid = data['source_uav']
            if (not self._apply_visual_hypothesis(data) or data['sample_s'] <
                    getattr(self, '_hypothesis_switch_s', {}).get(tid, -1e18)):
                return
            if uid == self.uav_id and data['eligible'] and data.get('candidate_id'):
                bindings = self.__dict__.setdefault('_camera_instance', {})
                prior = bindings.get(tid)
                if prior is None or data['sample_s'] > prior['sample_s']:
                    bindings[tid] = dict(candidate_id=data['candidate_id'], sample_s=data['sample_s'])
                    view = getattr(self, '_tracking_image', None)
                    changed = prior is not None and prior['candidate_id'] != data['candidate_id']
                    wrong_view = (view is not None and view.get('world_candidate_id') is not None and
                                  view['world_candidate_id'] != data['candidate_id'])
                    if ((changed or wrong_view) and
                            getattr(getattr(self, 'assignment', None), 'target_id', None) == tid):
                        self._tracking_image = None
                        self._tracking_altitude = None
                        self._camera_cue = None
                        self._look_at = (data['x'], data['y'])
            if uid == self.uav_id and tid in ('t4', 't5'):
                self._remember_red_view_binding(
                    json.loads(msg.data).get('local_target_id'), data, now)
            history = getattr(self, '_team_visual', None)
            if history is None:
                history = self._team_visual = {}
            key = (uid, tid)
            previous = history.get(key)
            if previous is not None and data['sample_s'] <= previous['sample_s']:
                return
            if data['eligible']:
                continuous = (previous is not None and
                              data['sample_s'] - previous['sample_s'] <= 1.0 and
                              (not previous.get('candidate_id') or not data.get('candidate_id')
                               or previous['candidate_id'] == data['candidate_id']))
                history[key] = dict(data,
                    hits=previous['hits'] + 1 if continuous else 1,
                    since_s=previous['since_s'] if continuous else data['sample_s'])
            if uid != self.uav_id:
                return  # no peer sample can be relabelled as our detection
            old = self._visual_observations.get(tid)
            if old is not None and data['sample_s'] <= old['sample_s']:
                return
            self._visual_observations[tid] = data
            if data['eligible']:
                if not hasattr(self, '_last_eligible_image'):
                    self._last_eligible_image = {}
                self._last_eligible_image[tid] = data['sample_s']
                if not hasattr(self, '_eligible_observations'):
                    self._eligible_observations = {}
                self._eligible_observations[tid] = data
                evidence = history[key]
                wx = self.world_xy
                if (wx is not None and evidence['hits'] >= 3 and
                        data['sample_s'] - evidence['since_s'] >= 0.4 - 1e-6):
                    color = {'t0': 'green', 't1': 'blue', 't2': 'brown',
                             't3': 'white', 't4': 'red', 't5': 'red'}[tid]
                    self._offer_camera_attention('target:' + tid + ':' + data.get('candidate_id', 'legacy'), color,
                        math.atan2(data['y'] - wx[1], data['x'] - wx[0]),
                        data['sample_s'], now, tid, (data['x'], data['y']))
            detection = TargetDetection()
            detection.header.stamp = rospy.Time.from_sec(data['sample_s'])
            detection.header.frame_id = 'world_enu'
            detection.uav_id, detection.target_id = self.uav_id, tid
            detection.x, detection.y = data['x'], data['y']
            detection.confidence, detection.source = data['confidence'], 1
            self.detect_pub.publish(detection)
        except (ValueError, KeyError, TypeError, AttributeError):
            return

    def _detect_targets(self):
        """Geometry diagnoses reachability; it does not publish detections."""
        wx = self.world_xy
        if wx is None:
            return
        now = rospy.Time.now().to_sec()
        if now-self._last_detect_t < 1.0/DETECT_RATE:
            return
        self._last_detect_t = now
        for tid, (tx, ty, _vx, _vy) in list(self.targets.items()):
            if now-self._t_seen.get(tid, 0.0) > TARGET_TTL:
                continue
            if now-self._detect_log_t.get(tid, 0.0) < 1.0:
                continue
            self._detect_log_t[tid] = now
            distance = math.hypot(tx-wx[0], ty-wx[1])
            evidence = self._visual_observations.get(tid)
            seen = evidence is not None and 0.0 <= now-evidence['sample_s'] <= 1.0
            rospy.loginfo('[GEOMETRY] uav=%s target=%s dist=%.3fm los=%s '
                          'own_camera_seen=%s official_success=unknown ts=%.3f',
                          self.uav_id, tid, distance,
                          self.los.visible(wx[0], wx[1], tx, ty), seen, now)

    # ---------------- 参数与模式 ----------------
    def _set_param(self, param_id, value):
        from mavros_msgs.msg import ParamValue
        try:
            return self.param_srv(param_id, ParamValue(integer=value, real=0.0)).success
        except Exception as exc:
            rospy.logwarn_throttle(10, "[%s] 设置 %s 异常: %s", self.uav_id, param_id, exc)
            return False

    def _configure_fcu(self):
        """SITL 无遥控器会触发 RC 失联 failsafe，拒绝解锁/进 OFFBOARD。

        2026-10-03 改（取自队友 codex 分支的 fcu_configuration）：
        原实现只 set、不看返回值 —— MAVROS 参数表尚未就绪时 `param/set` 会被
        **静默拒绝**，飞机照样往下走 arm / 进 OFFBOARD，最后在 RC 失联 failsafe
        上被拦下，表象是"解锁失败"，根因却在几百行之前的参数设置处。
        现在 pull → set → get 读回验证，三轮仍不一致就抛异常终止启动
        （宁可起不来，也不要带着未生效的参数上天）。
        """
        from mavros_msgs.srv import ParamPull, ParamGet
        _pull_ns = "/%s/mavros/param/pull" % self.uav_id
        _get_ns = "/%s/mavros/param/get" % self.uav_id
        try:
            rospy.wait_for_service(_pull_ns, timeout=30)
        except rospy.ROSException as exc:
            raise RuntimeError("FCU_PARAM_PULL_UNAVAILABLE: %s" % exc)
        pull_srv = rospy.ServiceProxy(_pull_ns, ParamPull)
        get_srv = rospy.ServiceProxy(_get_ns, ParamGet)

        def _pull():
            resp = pull_srv(True)
            return resp.success and resp.param_received > 0

        def _get(name):
            resp = get_srv(name)
            return resp.value.integer if resp.success else None

        # PX4 also checks the Hold exception during AUTO.TAKEOFF. The logged
        # grounded recovery switched commander main_state to takeoff, but
        # without bit 1 its missing RC/datalink enabled failsafe and left the
        # actual navigation state in OFFBOARD. Cover only the two autonomous
        # modes we use; OFFBOARD signal-loss and positioning guards stay intact.
        rc_exceptions = (1 << 1) | (1 << 2)  # Hold + Offboard, never Mission
        configure_fcu_parameters(_pull, self._set_param, _get,
                                 {"NAV_RCL_ACT": 0, "COM_RCL_EXCEPT": rc_exceptions})
        rospy.loginfo("[%s] 飞控参数已读回验证：NAV_RCL_ACT=0 COM_RCL_EXCEPT=6 (Hold+Offboard)",
                      self.uav_id)

    def _camera_takeoff_ready(self, now):
        """Live processed images prove the ground datum still works now.

        A historical FIRST_INFERENCE/HEIGHT_DATUM log is insufficient: the
        datum can become invalid again before arming and then freeze empty.
        This checks the existing legal image channel, without recalibrating
        an airborne camera or relaxing pose/geometry thresholds.
        """
        if os.environ.get('TAKEOFF_REQUIRE_CAMERA', '0') != '1':
            return True
        first, last = getattr(self, '_takeoff_camera_frames', (0., 0.))
        samples = getattr(self, '_takeoff_ground_samples', [])
        stable_ground = (len(samples) >= 15 and
                         0. <= now-samples[-1][0] <= .2 and
                         samples[-1][0]-samples[0][0] >= 1.9 and
                         max(p[1] for p in samples)-min(p[1] for p in samples) <= .04 and
                         max(abs(p[2]) for p in samples) <= math.radians(3.))
        return stable_ground and 0. <= now-last <= .3 and last-first >= 1.

    def _commit_camera_datum(self):
        """Close the image-ready -> arming race with the owning perception node."""
        if os.environ.get('TAKEOFF_REQUIRE_CAMERA', '0') != '1':
            return True
        try:
            response = rospy.ServiceProxy('/%s/perception/prepare_takeoff' % self.uav_id,
                                          Trigger)()
            if response.success:
                return True
            rospy.logwarn_throttle(3., '[TAKEOFF_DATUM_WAIT] %s: %s',
                                   self.uav_id, response.message)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(3., '[TAKEOFF_DATUM_WAIT] %s: %s', self.uav_id, exc)
        return False

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
            rospy.logerr("[%s] 已在空中但没有合法的出生点局部位姿锚定；拒绝用固定出生偏移接管",
                         self.uav_id)
            return False

        stable = 0
        stream_start = last_setpoint = None
        while not rospy.is_shutdown():
            if getattr(self, '_localization_fault', False):
                rospy.logerr('[%s] 起飞预热期间定位失信，终止起飞', self.uav_id)
                return False
            # EKF 等待同时完成预热；不能把断流或时钟回退前的时间算进去。
            now = rospy.Time.now().to_sec()
            if (last_setpoint is None or now < last_setpoint or
                    now - last_setpoint > OFFBOARD_PRESTREAM_MAX_GAP_S):
                stream_start = now
                stable = 0
            self._send_vel(0.0, 0.0, vz=0.0)
            last_setpoint = now
            if self.local_z is not None and abs(self.local_z) < 1.5:
                stable += 1
                if (stable >= STABLE_NEEDED and
                        now - stream_start >= OFFBOARD_PRESTREAM_S and
                        self._camera_takeoff_ready(now)):
                    break
            else:
                if stable > 0:
                    rospy.logwarn_throttle(5, "[%s] EKF 抖动（z=%.2f），重新计数",
                                           self.uav_id,
                                           self.local_z if self.local_z is not None else -999)
                stable = 0
            self.ctrl_rate.sleep()
        if rospy.is_shutdown():
            return False
        rospy.loginfo("[%s] EKF 已稳定（z=%.2f，连续 %d 次采样达标）", self.uav_id,
                      self.local_z if self.local_z is not None else -999, stable)
        rospy.loginfo('[%s] 零速预热完成（连续 %.2fs，与 EKF 检查并行）',
                      self.uav_id, now - stream_start)

        # Official launch coordinates are fixed. Both perception and agent
        # must use the same local-to-world translation throughout the sortie.
        # A nonzero local origin cannot be corrected independently in flight.
        if (self.local_xy is None or
                math.hypot(self.local_xy[0], self.local_xy[1]) > 1.0):
            rospy.logerr("[%s] 出生点 MAVROS local_xy=%s 偏离固定原点，拒绝起飞",
                         self.uav_id, self.local_xy)
            return False
        self._anchor_done = True   # 此后 _local_cb 监测异常位姿，偏移保持不变

        # 切 OFFBOARD：**持续重试直到成功**（多机 SITL 下 EKF 速度/姿态估计就绪
        # 时间不定，固定次数会误判放弃，导致必须重启整个 SITL。永不放弃 + 每次
        # 重试间隔继续发零速预热，EKF 稳了自然就进得去。）
        attempt = 0
        while not rospy.is_shutdown() and self.state.mode != "OFFBOARD":
            if getattr(self, '_localization_fault', False):
                rospy.logerr('[%s] OFFBOARD 等待期间定位失信，终止起飞', self.uav_id)
                return False
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
            if not self._camera_takeoff_ready(rospy.Time.now().to_sec()):
                rospy.logwarn_throttle(3., '[TAKEOFF_CAMERA_WAIT] uav=%s '
                                       'waiting for current calibrated image sequence', self.uav_id)
                self._send_vel(0.0, 0.0, vz=0.0)
                self.ctrl_rate.sleep()
                continue
            if not self._commit_camera_datum():
                self._send_vel(0.0, 0.0, vz=0.0)
                self.ctrl_rate.sleep()
                continue
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
        - 追踪目标时：+1.0m（2026-10-06 八轮复盘：旧值 -1.0 写于层高 5.0/5.5/6.0
          时代，现层高 2.8 起步，追踪时 2.8-1.0=1.8m 与 actor 1.7m 几乎平视，
          视线接近水平 -> t=-pz/v_world[2] 地面交点几何爆炸（实测偏差 ±7-10m）。
          改为 +1.0 升到 ~3.8-4.0m 俯视观测，8m 观测距俯角 >=26°，几何稳定）
        - 建筑附近：-0.5m（保持距离）
        - 开阔区域：0m（常规搜索）
        """
        # === v18 修复：先判 None 再 unpack ===
        # v17 实测（logs_20261007_215530）：EKF 雪崩期 world_xy 返回 None，
        # 旧代码 `wx, wy = self.world_xy` 先 unpack 后判 None → TypeError
        # 主循环反复崩溃（agent_0 复飞爬升被打断 → 加剧熔断-复飞循环）。
        _w = self.world_xy
        if _w is None:
            return 0.0
        wx, wy = _w

        # 检查是否在追踪目标
        # 近处保持低视点以免脚出画面；8m 观察圈不再强制降到
        # 2.6m（叠加 EKF 高度降层后实测约 2.2m）。相机是否真的看到
        # 人只能由视觉链确认，地图 LOS 不能替代图像。
        assignment = getattr(self, 'assignment', None)
        tracking_tid = (assignment.target_id
                        if getattr(assignment, 'task_type', None) == 1
                        else self._orbit_target)
        hold = getattr(self, '_tracking_altitude', None)
        if hold is not None and hold['target_id'] == tracking_tid:
            # Leaving the orbit changes horizontal pursuit, not the established
            # image-height goal. Falling back to the search layer here lowers
            # the camera precisely when a receding target needs a steeper ray.
            # Retain the original two-second budget; no image/position refresh.
            now = rospy.Time.now().to_sec()
            age = now-hold['sample_s']
            within_recovery = now < hold.get('recovery_until', float('inf'))
            goal = hold['goal'] if 0. <= age <= 2. and within_recovery else self.local_z
            # _send_vel subtracts the old layer discount. This goal is already
            # anchored to measured altitude and must not be discounted twice.
            return goal-self.altitude_layer + getattr(self, '_alt_sag', 0.0)
        if self._orbit_target is not None:
            target = self.targets.get(self._orbit_target)
            rng = (math.hypot(target[0] - wx, target[1] - wy)
                   if target is not None else None)
            return min(1.0, observation_altitude_cap(rng) - self.altitude_layer)

        # 检查是否在建筑附近（使用未膨胀的原始栅格）
        if hasattr(self, 'los') and self.los:
            # 简单判断：当前位置是否靠近障碍
            # 用 A* 膨胀后的栅格判断
            cell = self.grid.world_to_cell((wx, wy))
            if cell and not self.grid.is_free(cell):
                if MOTION_SAFETY:
                    # A horizontal inflated cell cannot prove free space
                    # below us. Hold this layer while leaving horizontally.
                    return self.local_z-self.altitude_layer+getattr(self, '_alt_sag', 0.)
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
                    if MOTION_SAFETY:
                        return self.local_z-self.altitude_layer+getattr(self, '_alt_sag', 0.)
                    return -0.5

        # 开阔区域
        return 0.0

    def _radar_guard_velocity(self, vx, vy):
        """用 2D 雷达给当前 ENU 速度加一层近障安全约束。

        雷达角度在机体系内，0 弧度为机头方向。这里只改水平速度，
        不接管 OFFBOARD，也不另发 MAVROS 设定点。
        """
        # R1（2026-10-09）：近障强介入标记。雷达进入近障逻辑（front < WARN 或
        # 三面堵死/后退）时置 True，_send_vel 据此让栅格守卫放弃方向改写，
        # 根治「雷达转向→栅格覆盖反向→50ms 互相对消原地打转」（v26 撞墙主因）。
        # 所有非近障提前返回在此统一先置 False（下方近障 return 前再置 True）。
        self._radar_strict = False
        if not RADAR_GUARD or self._scan is None:
            return vx, vy
        now = rospy.Time.now().to_sec()
        if not 0. <= now - self._scan_t <= RADAR_FRESH_S:
            return 0.0, 0.0
        speed = math.hypot(vx, vy)
        if speed < 0.05:
            return vx, vy

        scan = self._scan
        front, left, right = scan.range_max, scan.range_max, scan.range_max
        for i, raw in enumerate(scan.ranges):
            r = float(raw)
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            if r < RADAR_SELF_ECHO_M:
                continue  # 自回波地板：机身残余部件回波视为无障碍（EMA 前过滤）
            angle = scan.angle_min + i * scan.angle_increment
            deg = math.degrees(angle)
            if -30.0 <= deg <= 30.0:
                front = min(front, r)
            elif 30.0 < deg <= 100.0:
                left = min(left, r)
            elif -100.0 <= deg < -30.0:
                right = min(right, r)

        # ---- 国家一等奖修复（2026-10-05）：EMA 滤波防 spike ----
        # 单帧 2D 激光的某次随机回波（桨叶/雨滴/反射）会把 front 拉到 0.3m，
        # 直接触发 v_cap=0 死锁。用 EMA 把单帧噪声平均掉：
        #   smoothed = α * raw + (1-α) * prev
        # α=0.6 表示新数据占 60% 权重（响应快但仍吸收部分噪声）。
        _prev = getattr(self, '_radar_prev', None)
        if _prev is None:
            _ema_f, _ema_l, _ema_r = front, left, right
        else:
            _ema_f = RADAR_EMA_ALPHA * front + (1 - RADAR_EMA_ALPHA) * _prev[0]
            _ema_l = RADAR_EMA_ALPHA * left  + (1 - RADAR_EMA_ALPHA) * _prev[1]
            _ema_r = RADAR_EMA_ALPHA * right + (1 - RADAR_EMA_ALPHA) * _prev[2]
        self._radar_prev = (_ema_f, _ema_l, _ema_r)
        front, left, right = _ema_f, _ema_l, _ema_r

        # ---- v11 复盘（2026-10-07）自扫残留过滤（全局，EMA 后立即生效）----
        # SDF 已把雷达抬到 z=0.38m（盒顶 0.335m 上方），但 v11 实测仍有
        # 0.31~0.34m 恒定回波（front=0.32 出现 39 次）：雷达平面仍扫到
        # 机身碰撞盒顶边缘/起落架连接件。特征：某扇区读数恒定贴 0.3x 且
        # 不随真实世界变化（v11 中同一读数持续 400s）。
        # 处理：三扇区中凡 <0.40m 的读数直接抬为量程上限（视为无回波），
        # 不再参与 v_cap 刹停 / 三面堵死 / 扇区统计。真实障碍在 0.40m 内
        # 出现且恰好恒定贴 0.3x 的概率可忽略；且 A* 栅格层仍兜底防撞。
        _SELF_ECHO = RADAR_SELF_ECHO_M
        if front < _SELF_ECHO:
            front = scan.range_max
        if left < _SELF_ECHO:
            left = scan.range_max
        if right < _SELF_ECHO:
            right = scan.range_max

        # 前视目标时，盘旋和侧移的运动方向经常与机头相差 90°。旧分支只看
        # 机头 front，然后无条件把输入速度重建为「前进+横移」：即使输入是
        # 安全侧移，也会被推向机头前方的墙。非前向运动按真实运动方向激光
        # 净空限速；盲区或堵死时原地停止，等待航向/路径重规划。
        body_angle = (math.atan2(vy, vx) - self.yaw + math.pi) % (2.0 * math.pi) - math.pi
        if abs(body_angle) > math.radians(35.0):
            guarded = self._final_scan_cap(vx, vy)
            if math.hypot(*guarded) < 0.05:
                self._radar_strict = True
                self._radar_strict_d = 0.0
            return guarded

        # 最近净空（三扇区最小值）→ 硬刹停速度上限 v <= sqrt(2*a*(d-安全间隙))
        d_min = min(front, left, right)
        self._radar_strict_d = d_min   # R1：供栅格守卫互斥分支读取
        v_cap = math.sqrt(max(0.0, 2.0 * MAX_ACC *
                              max(0.0, d_min - RADAR_SAFE_GAP)))
        # ---- 国家一等奖修复（2026-10-05）：v_cap 最低保底 ----
        # v_cap=0 → 后续 gspd > v_cap 必然把水平速度夹成 0 → 飞机 0 速度死锁。
        # 任何情景下保留 RADAR_VCAP_FLOOR 的最低刹停速度，配合 RADAR_BACKOFF_R=1.8
        # 触发后退，保证飞机有"离开"动作而不会卡死在建筑边上。
        v_cap = max(v_cap, RADAR_VCAP_FLOOR)
        if front >= RADAR_WARN_R:
            if speed > v_cap and speed > 1e-9:
                return vx * (v_cap / speed), vy * (v_cap / speed)
            return vx, vy

        # 优先选择净空更大的侧面；正前方两侧都未知时固定向左，避免左右抖动。
        # 左右净空差很小时保持上次绕行侧，避免测量噪声每帧反转。
        side = getattr(self, '_radar_side', 1.0 if left >= right else -1.0)
        if left - right > 0.7:
            side = 1.0
        elif right - left > 0.7:
            side = -1.0
        self._radar_side = side

        # ---- 国家一等奖修复（2026-10-04）：三面近障兜底后退 ----
        # 真实仿真日志（logs_20261004_013325）实锤：UAV_0 以 front=0.31 / left=0.30 /
        # right=0.32 三面全堵的姿态，被旧逻辑输出 vout=(0.07,1.59)「向左横移」，
        # 直接顶进 0.30m 外的左墙，位置 40s+ 纹丝不动（DWA 死锁）。
        # 根因：旧逻辑只比较 left/right 谁大，没有检查「两侧是否都堵死」。
        # 修复：两侧净空都低于 SIDE_BLOCK_R 时，唯一可行方向是后退——沿 -vin
        # （远离障碍）方向退，退到 front 恢复到 RADAR_WARN_R 之前一直保持后退。
        SIDE_BLOCK_R = 0.6   # 侧向净空低于此值视为侧墙堵死（m）
        # ---- v10 复盘（2026-10-07）自扫伪影过滤 ----
        # 旧 SDF 中 laser_2d（绝对 z=0.32m）落在机身碰撞盒 0.67x0.67x0.15
        # @ z 0.185~0.335m 内部，射线打自家盒壁 → 三面恒 ≈0.31m 且几乎相等，
        # v10 实测六机因此被三面堵死锁死在起飞线 300s（x≈-47 净位移≈0）。
        # 真实障碍三面等距（极差<0.10m 且均<0.45m）概率极低；SDF 已根治
        # （雷达抬到 z=0.38m），此处为纵深防御：识别到自扫特征直接放行，
        # 信任 A*（SLAM 栅格 unknown=free 已确认路径）。
        _fmax, _fmin = max(front, left, right), min(front, left, right)
        if _fmax < 0.45 and (_fmax - _fmin) < 0.10:
            return vx, vy
        if left < SIDE_BLOCK_R and right < SIDE_BLOCK_R:
            # ---- 国家一等奖修复 v2（2026-10-04）：扫描引导后退 ----
            # v1 用 -vin 方向后退，但真实仿真实锤：UAV_0 卡在建筑夹缝（死胡同）时
            # -vin 恰好指向夹缝后墙，指令 2.4 m/s 实际位移 0.09 m/s，仍卡死。
            # v2：在雷达覆盖范围（±100°，机尾 160° 为盲区不可选）内按 30° 扇区
            # （50% 重叠）统计各扇区最小净空，朝净空最大的扇区中心后退——
            # 这是从夹缝里出来的唯一几何可行方向，与 vin 无关。
            SECTOR = 30.0
            best_deg, best_r = 180.0, -1.0
            d0 = -100.0
            while d0 + SECTOR <= 100.0 + 1e-6:
                sec_min = scan.range_max
                for i, raw in enumerate(scan.ranges):
                    r = float(raw)
                    if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                        continue
                    if r < RADAR_SELF_ECHO_M:
                        continue  # 自回波地板：防单一扇区被机身回波污染成 0.3m
                    deg = math.degrees(scan.angle_min + i * scan.angle_increment)
                    if d0 <= deg < d0 + SECTOR and r < sec_min:
                        sec_min = r
                if sec_min > best_r:
                    best_r = sec_min
                    best_deg = d0 + SECTOR * 0.5
                d0 += SECTOR * 0.5
            if best_deg > 100.0:      # 全盲（无有效回波）：保底正后方不可选，退向左前
                best_deg, best_r = -60.0, 0.0
            # === 国家一等奖修复 v4（2026-10-06）：撞杆 + 盘旋不动根治 ===
            # 实测（logs_20261005_215441）：6 个 UAV 在 lamp_post 边缘三面堵死
            # front=0.31~0.32m，旧逻辑「沿 best_deg 方向 retreat=1.5 m/s 后退」
            # 导致飞机原地横跳 60+ 次（每次净位移 0.015m，净移动 0.9m/40s）。
            # 根因：当 best_r≥1.0m 时本就是「净空足够」场景，不应后退，应推进！
            # 修复策略（按 best_r 分级，决定是「推进 / 横滑 / 后退」）：
            #   1. best_r≥1.5m：净空充足，沿扇区推进 0.8 m/s（脱离死循环）
            #   2. 1.0≤best_r<1.5：中等净空，沿扇区横滑 0.5 m/s（脱离贴墙）
            #   3. 0.5≤best_r<1.0：净空紧张，沿扇区横滑 0.3 m/s
            #   4. best_r<0.5：贴墙（实测 logs_20261006_004620：整场 best_r 始终 0.35~0.38m
            #      6 架全卡在 lamp_post 边缘，旧逻辑用 1.5 m/s 撞墙，无效)
            #      v5 修复：低速横滑 0.4 m/s 慢慢蹭出（不后退，避免扇区内净空被自挤压）
            #   5. 全程设 _radar_retreating=True 旁路加速度限幅（保留旧设计）
            if best_r >= 1.5:
                advance = min(RADAR_BACKOFF_MAX, speed, 0.5)
                advance_mode = '推进'
            elif best_r >= 1.0:
                advance = min(speed, 0.5)
                advance_mode = '横滑'
            elif best_r >= 0.5:
                advance = min(speed, 0.3)
                advance_mode = '慢滑'
            else:
                advance = 0.0   # 没有可验证净空时停车，不以“脱困”为由蹭墙
                advance_mode = '贴墙横滑'
            # B1 近障困局脱离（2026-10-09）：三面收窄持续超过阈值时，限时沿来路反向退出。
            # 反向后净空不再被机身自挤压（前进时雷达扫到建筑巷道内壁，扇区净空被压缩），
            # 用离线干净的「反向后退」打破原地横跳/抖动循环；退出到净空或超时后回到旧逻辑。
            if advance < RADAR_BACKOFF_SPD * 0.8 and \
               front < RADAR_JAM_FRONT_R and left < RADAR_JAM_SIDE_R and right < RADAR_JAM_SIDE_R:
                _jam_t0 = getattr(self, "_jam_escape_t0", None)
                _jam_until = getattr(self, "_jam_escape_until", 0.0)
                _now = now.to_sec() if hasattr(now, "to_sec") else now
                if _jam_t0 is None:
                    self._jam_escape_t0 = _now
                    _jam_t0 = _now
                if _now - _jam_t0 >= RADAR_JAM_TRIGGER_S:
                    # 已持续触发时长：进入限时反向退出
                    if _now >= _jam_until:
                        self._jam_escape_until = _now + RADAR_JAM_ESCAPE_S
                    if _now < self._jam_escape_until:
                        # 反向 = 沿机头反向（yaw 方向 180°），低速后退，避免撞墙
                        escape_speed = min(speed, RADAR_JAM_ESCAPE_SPD, 0.5)
                        _bx = escape_speed * math.cos(self.yaw + math.pi)
                        _by = escape_speed * math.sin(self.yaw + math.pi)
                        guarded = (_bx, _by)
                        self._radar_retreating = False
                        self._radar_strict = True
                        rospy.logwarn_throttle(2.0,
                                               '[%s] 近障困局 %.0fs → 反向退出 %.0fs vout=(%.2f,%.2f)',
                                               self.uav_id, _now - _jam_t0,
                                               RADAR_JAM_ESCAPE_S, guarded[0], guarded[1])
                        return guarded
                # 未触发或已退出：回落旧分级逻辑
            else:
                self._jam_escape_t0 = None
            ang = math.radians(best_deg)                          # 机体系
            body_x = advance * math.cos(ang)
            body_y = advance * math.sin(ang)
            cy, sy = math.cos(self.yaw), math.sin(self.yaw)
            guarded = (cy * body_x - sy * body_y,
                       sy * body_x + cy * body_y)
            self._radar_retreating = False  # 退出也受加速度约束，停车仍可立即生效
            self._radar_strict = True                              # R1：栅格守卫不得再改写方向
            rospy.logwarn_throttle(2.0,
                                   '[%s] 2D雷达三面堵死 front=%.2f left=%.2f right=%.2f '
                                   '→ 扇区%.0f°净空%.2fm %s vout=(%.2f,%.2f) %s速度=%.2f',
                                   self.uav_id, front, left, right,
                                   best_deg, best_r,
                                   advance_mode,
                                   guarded[0], guarded[1], advance_mode, advance)
            return guarded

        # === v16 修复：前向硬上限 0.35 → 0.65（近墙线性段保留）===
        # v15 实测（logs_20261007_140942）：城区 front 3~4m 常态化，守卫持续介入，
        # vout 前向分量均值仅 0.16 m/s（中位 0.47），13m 直线飞 210s。
        # 0.35 上限把远端预警区（front>3.9m）也压死；线性段在 front<3.9m 时
        # 主导（(front-1.6)/3.9 < 0.65），近墙减速不受影响，仅放宽远端。
        # v24 提速（2026-10-09）：去掉无近障时的 0.65 硬上限 —— front≥WARN 时
        # 恒速巡航应保持 SEARCH_CRUISE_SPEED=3.0（0.65 上限会把 3.0 压到 1.95，
        # 抵消 v24 搜索提速，5min 覆盖仍锁死 17%）。front<WARN 的线性减速段
        # 原样保留（front=3m→1.8m/s、front=1.6m→0，刹停距离仍 < LOOKAHEAD=3m）。
        forward = max(0.0, min(speed,
                                speed * (front - RADAR_STOP_R) /
                                max(RADAR_WARN_R - RADAR_STOP_R, 1e-6)))
        # 侧向逃逸速度随净空收紧（旧值可达 1.5 m/s，在 0.5m 净空下横滑 = 刮擦）
        lateral = min(speed, 1.2 * (RADAR_WARN_R - front) /
                       max(RADAR_WARN_R - RADAR_STOP_R, 1e-6), v_cap)
        body_x, body_y = forward, side * lateral
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        guarded = (cy * body_x - sy * body_y,
                   sy * body_x + cy * body_y)
        # 先按「刹得住」夹合速度（含前向与侧向逃逸）
        gspd = math.hypot(guarded[0], guarded[1])
        if gspd > v_cap and gspd > 1e-9:
            k = v_cap / gspd
            guarded = (guarded[0] * k, guarded[1] * k)
        # 再叠加贴墙后退：front 已近到雷达量程下限时，唯一安全的动作是沿机体 -x
        # 退出（远离障碍，故不受刹停约束；否则 v_cap=0 会把后退也一起夹成 0，
        # 飞机就永远顶死在墙上 —— 这正是 02:24 场的实况）。
        if front < RADAR_BACKOFF_R and RADAR_BACKOFF_SPD > 0.0:
            backoff = min(speed, RADAR_BACKOFF_SPD, 0.5)
            guarded = (guarded[0] - cy * backoff,
                       guarded[1] - sy * backoff)
        # UAV2: vin=(-.12,-.04) became a 1.5 m/s escape beside .50 m
        # returns. A safety constraint may redirect or reduce a command, but
        # must not amplify a slow command from tracking or inter-UAV avoidance.
        output_cap = min(speed, 0.5) if d_min < RADAR_BACKOFF_R else speed
        gspd = math.hypot(*guarded)
        if gspd > output_cap and gspd > 1e-9:
            guarded = (guarded[0]*output_cap/gspd, guarded[1]*output_cap/gspd)
        self._radar_strict = True   # R1：近障线性段强介入，栅格守卫不得改写
        rospy.logwarn_throttle(2.0,
                               '[%s] 2D雷达近障 front=%.2fm left=%.2f right=%.2f '
                               '-> vin=(%.2f,%.2f) vout=(%.2f,%.2f)',
                               self.uav_id, front, left, right,
                               vx, vy, guarded[0], guarded[1])
        return guarded

    def _final_scan_cap(self, vx, vy):
        """在所有转向与限幅之后，对最终运动方向再查一次原始激光。"""
        speed = math.hypot(vx, vy)
        if not RADAR_GUARD or speed < 0.05:
            return vx, vy
        now = rospy.Time.now().to_sec()
        scan = self._scan
        if scan is None or not 0. <= now - self._scan_t <= RADAR_FRESH_S:
            return 0.0, 0.0
        body_angle = (math.atan2(vy, vx) - self.yaw + math.pi) % (2 * math.pi) - math.pi
        nearest = None
        nearest_any = None
        nearest_any_angle = None
        for i, raw in enumerate(scan.ranges):
            beam = scan.angle_min + i * scan.angle_increment
            delta = (beam - body_angle + math.pi) % (2 * math.pi) - math.pi
            r = float(raw)
            if math.isnan(r) or r == -math.inf:
                continue
            # A valid close return is an obstacle at the execution boundary.
            # The mapping self-echo floor must not erase a real post/wall as
            # it approaches from 0.60 m to less than 0.50 m.
            if math.isfinite(r) and r < scan.range_min:
                continue
            if not math.isfinite(r) or r > scan.range_max:
                r = scan.range_max
            # 机尾 ±100..135° 的 0.50m 固定回波来自机身/桨架：六机在
            # 完全不同位置均反复出现，但 front/left/right 净空仍为数米。
            # 它不应限制朝前飞行；若真正朝该角度移动，下方 ±20° 的
            # 运动方向刹停检查仍会读取原始射线并停车。
            if abs(beam) <= math.radians(100.0) and (nearest_any is None or r < nearest_any):
                nearest_any, nearest_any_angle = r, beam
            if abs(delta) > math.radians(20.0):
                continue
            nearest = r if nearest is None else min(nearest, r)
        # 任一方向已贴近障碍时，禁止继续向它移动；平行滑行或后退也须慢速。
        # 上轮侧向回波曾低至 0.58m，单纯检查运动方向 ±20° 无法约束擦墙速度。
        if nearest_any is not None and nearest_any < RADAR_SIDE_PANIC_R:
            toward = vx * math.cos(self.yaw + nearest_any_angle) + \
                     vy * math.sin(self.yaw + nearest_any_angle)
            if toward > 0.05:
                rospy.logwarn_throttle(2.0,
                    '[%s] 末级雷达侧障停车 d=%.2fm toward=%.2fm/s',
                    getattr(self, 'uav_id', '?'), nearest_any, toward)
                return 0.0, 0.0
            if speed > RADAR_SIDE_PANIC_SPEED:
                k = RADAR_SIDE_PANIC_SPEED / speed
                vx, vy = vx * k, vy * k
                speed = RADAR_SIDE_PANIC_SPEED
                rospy.logwarn_throttle(2.0,
                    '[%s] 末级雷达侧障限速 d=%.2fm cap=%.2fm/s',
                    getattr(self, 'uav_id', '?'), nearest_any, speed)
        if nearest is None or nearest <= RADAR_STOP_R:
            return 0.0, 0.0
        cap = min(MAX_SPEED, math.sqrt(max(0.0, 2.0 * MAX_ACC *
                                          (nearest - RADAR_STOP_R))))
        if speed > cap:
            return vx * cap / speed, vy * cap / speed
        return vx, vy

    def _guard_vertical_clearance(self, vz):
        """Hold routine height changes beside obstacles seen by the 2D lidar.

        The horizontal scan cannot establish clearance below an aircraft.
        UAV4's horizontal stop therefore did not justify a -1 m/s descent
        beside a scaffold. Takeoff, landing and the hard altitude protections
        retain their own authority.
        """
        if ((abs(vz) < .05 and not MOTION_SAFETY) or not RADAR_GUARD or
                not getattr(self, '_takeoff_done', False) or
                getattr(self, '_landing', False)):
            return vz
        now = rospy.Time.now().to_sec()
        scan = getattr(self, '_scan', None)
        if scan is None or not 0. <= now-getattr(self, '_scan_t', -1e18) <= RADAR_FRESH_S:
            rospy.logwarn_throttle(2., '[VERTICAL_CLEARANCE] uav=%s stale scan; hold routine height',
                                   self.uav_id)
            return 0.
        nearest = scan.range_max
        observed = False
        for i, raw in enumerate(scan.ranges):
            angle = scan.angle_min+i*scan.angle_increment
            # Fixed rear airframe echoes are outside this observed sector.
            if abs(angle) > math.radians(100.):
                continue
            r = float(raw)
            if r == float('inf'):
                observed = True  # valid no-return beam, unlike NaN
            elif math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                observed = True
                nearest = min(nearest, r)
        if not observed:
            rospy.logwarn_throttle(2., '[VERTICAL_CLEARANCE] uav=%s invalid scan; hold routine height',
                                   self.uav_id)
            return 0.
        if MOTION_SAFETY:
            xy = self.world_xy
            if xy is None:
                return 0.
            grid = getattr(self, 'grid', None)
            cell = grid.world_to_cell(xy) if grid is not None else None
            occupied = grid is not None and (cell is None or not grid.is_free(cell))
            hold = getattr(self, '_height_clearance_hold', None)
            if hold is None:
                hold = self._height_clearance_hold = HeightClearanceHold()
            # The release band is one map cell beyond the old near threshold.
            # A drop in scan height must not make a rail disappear and restart
            # vertical motion at the same XY. Repeated output calls cannot
            # substitute for fresh scans or actual horizontal displacement.
            clear_range = RADAR_BACKOFF_R+max(.25, getattr(grid, 'resolution', .5))
            if hold.update(self._scan_t, xy, occupied or nearest < RADAR_BACKOFF_R,
                           not occupied and nearest >= clear_range, RADAR_STOP_R):
                if abs(vz) >= .05:
                    rospy.logwarn_throttle(2., '[VERTICAL_CLEARANCE_HOLD] uav=%s '
                        'near=%.2f occupied=%s; require horizontal exit and fresh clear scans',
                        self.uav_id, nearest, occupied)
                return 0.
        if nearest < RADAR_BACKOFF_R:
            rospy.logwarn_throttle(2., '[VERTICAL_CLEARANCE] uav=%s near=%.2fm vz=%.2f -> 0; '
                                   'move to clear observation space before changing height',
                                   self.uav_id, nearest, vz)
            return 0.
        return vz

    def _occupied_cell_exit(self, vx, vy):
        """Leave an inflated cell slowly, through a lidar-checked raw-free route."""
        speed = math.hypot(vx, vy)
        wx = self.world_xy
        if speed < .05 or wx is None or not RADAR_GUARD:
            return 0., 0.
        if not hasattr(self.grid, 'is_free_raw'):
            return 0., 0.
        own = self.grid.world_to_cell(wx)
        heading = math.atan2(vy, vx)
        exit_speed = min(.5, speed)
        candidates = []
        for degrees in (0, -30, 30, -60, 60, -90, 90, -120, 120, 180):
            angle = heading+math.radians(degrees)
            candidate = (exit_speed*math.cos(angle), exit_speed*math.sin(angle))
            checked = self._final_scan_cap(*candidate)
            if math.hypot(*checked) < exit_speed*.98:
                continue
            exit_distance = None
            safe = True
            for i in range(1, 13):
                d = i*.25
                point = (wx[0]+d*math.cos(angle), wx[1]+d*math.sin(angle))
                cell = self.grid.world_to_cell(point)
                if (cell is None or (cell != own and not self.grid.is_free_raw(cell)) or
                        (exit_distance is not None and not self.grid.is_free(cell))):
                    safe = False
                    break
                if exit_distance is None and self.grid.is_free(cell):
                    exit_distance = d
                if exit_distance is not None and d >= exit_distance+.5:
                    break
            if safe and exit_distance is not None:
                candidates.append((abs(degrees), exit_distance, candidate))
        if not candidates:
            rospy.logwarn_throttle(2., '[GRID_EXIT_BLOCKED] uav=%s occupied cell; no safe exit',
                                   self.uav_id)
            return 0., 0.
        chosen = min(candidates, key=lambda item:item[:2])[2]
        rospy.logwarn_throttle(2., '[GRID_EXIT] uav=%s occupied cell; slow checked exit=(%.2f,%.2f)',
                               self.uav_id, *chosen)
        return chosen

    def _grid_blocked(self, wx, wy):
        """世界点在栅格上是否不可通行（障碍或越界；越界也视为墙，防冲出地图）。"""
        c = self.grid.world_to_cell((wx, wy))
        return c is None or not self.grid.is_free(c)

    def _grid_guard_velocity(self, vx, vy):
        """无激光时用 A* 栅格做执行层近障约束（20Hz，纯查表）。

        沿速度方向按 GRID_GUARD_STEP 逐点前瞻到刹停距离；任一点被占，就在
        当前航向左右各 15° 起步搜索自由扇区，把速度转向该方向；搜不到则刹停。
        雷达在线时雷达先修，本方法再兜底，二者不冲突。

        高度豁免（2026-10-03）：起飞爬升期 z < SAFE_ALT 时不启用栅格近障，
        否则 6 机在起飞区（z≈2-4m）会因为栅格里的建筑/边界反复刹停-转向振荡。
        z 在 SAFE_ALT ~ ALT_CEILING 之间时线性衰减 guard 强度，防止高空贴墙。
        国家一等奖修复（2026-10-06）：原完全豁免会让 UAV0 起飞撞 lamp_post_191
        （距起飞点 (0,-3) 仅 3.08m）；改成「仍走栅格但缩前瞻」，爬升期也保护。
        """
        if not GRID_GUARD:
            return vx, vy
        wx = self.world_xy
        if wx is None:
            return vx, vy
        own_cell = self.grid.world_to_cell(wx)
        if own_cell is not None and not self.grid.is_free(own_cell):
            return self._occupied_cell_exit(vx, vy)
        # R1（2026-10-09）：雷达近障强介入时，栅格守卫只做速度限幅、不再改写
        # 方向。v26 实证（agent_3）：雷达守卫把速度转向右侧避开前障 → 栅格守卫
        # 随即判定"前方被占→转向 -170°"，两守卫每 50ms 互相覆盖 → 原地打转
        # 90s+（agent_0 在 (-37,-19) 困 80s）。此时雷达层数据更原始、更真，
        # 栅格（SLAM 膨胀+滞后）让位。
        if getattr(self, "_radar_strict", False):
            # 仅按"最近净空"夹合速度上限（v_cap 同款物理），不碰方向
            d_min = getattr(self, "_radar_strict_d", None)
            if d_min is not None and d_min < RADAR_WARN_R:
                _sp = math.hypot(vx, vy)
                if _sp > 1e-9:
                    _cap = math.sqrt(max(0.0, 2.0 * MAX_ACC *
                                         max(0.0, d_min - RADAR_SAFE_GAP)))
                    _cap = max(_cap, RADAR_VCAP_FLOOR)
                    if _sp > _cap:
                        k = _cap / _sp
                        vx, vy = vx * k, vy * k
            return vx, vy
        # 自身格可能处于障碍膨胀区。只允许低速、雷达验证的原始自由路径退出；
        # 无安全退出路径就停车，不能因为起点被占用而豁免整条运动指令。
        spd = math.hypot(vx, vy)
        if spd < 0.05:
            return vx, vy
        # 高度豁免：起飞爬升期（z < SAFE_ALT）仍走栅格，但缩前瞻距离
        z_now = self.local_z
        climb_scale = 1.0
        if z_now is not None and z_now < SAFE_ALT:
            # 爬升期：look 缩到 0.5m（防止 1.0m 外建筑/灯杆被忽略）
            # 同时爬升期一般 VIN 速度本就低（≤0.5m/s），刹停距离 ~0.05m 已足够
            climb_scale = 0.4   # 缩 60% 前瞻，但保留近障保护
        # 半豁免区（SAFE_ALT ~ ALT_CEILING）：衰减前瞻距离，减少高空误判
        guard_scale = 1.0
        if z_now is not None and ALT_CEILING > SAFE_ALT:
            guard_scale = max(0.0, min(1.0, (ALT_CEILING - z_now) / max(1e-6, ALT_CEILING - SAFE_ALT)))
        hd = math.atan2(vy, vx)
        # 刹停距离 v²/(2a) + 裕度；至少看 0.8m。裕度加大防建筑偏大/超调。
        # 半豁免区衰减前瞻：guard_scale<1 时缩短探测距离，允许飞机更快穿过近障区
        look = max(0.8, spd * spd / (2.0 * MAX_ACC) + 1.5) * guard_scale * climb_scale
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
        # 左右搜索自由航向（15° 步进到 180°；同侧从小到大，取偏转最小的）
        # 2026-10-07 v12: 120°→180°。速度控制是全向的，倒退与前进同样安全
        # （真实近障由雷达守卫 v_cap 兜底）。实测 agent0/2/3/4 卡建筑线时
        # 前向±120°全堵但正后方（180°）开阔，旧上限导致只能刹停无法脱困，
        # 六机各刹停 175 次/场。
        best = None
        for deg in range(15, 181, 15):
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

    def _oscillation_breakout(self, vx, vy, guard_changed=False):
        """R1（2026-10-09）：守卫交替振荡脱困。

        雷达/栅格守卫在"近障->转向"路径上互相覆盖（雷达转向右侧、栅格判前方占
        再转回左侧），每 50ms 方向翻转一次，位置原地打转（v26 撞墙主因）。检测
        速度方向在窗口内频繁大角度翻转 → 强制沿最近一个目标航向直行，期间守卫
        只做速度限幅（由 _grid_guard_velocity 的互斥分支承载）。
        """
        # A range controller legitimately reverses while holding a person in
        # view. The 08:05 run classified those reversals as obstacle deadlocks
        # and forced 1.2 m/s motion for two seconds, repeatedly cropping green.
        # Tracking has a separate safe observation-point recovery planner.
        task = getattr(getattr(self, 'assignment', None), 'task_type', None)
        if (task == 1 or getattr(self, '_orbit_target', None) is not None or
                getattr(self, '_camera_cue', None) is not None or
                not guard_changed or math.hypot(vx, vy) < .05):
            self._osc_prev = None
            self._osc_flicks = 0
            self._osc_win_t0 = self._osc_break_until = 0.0
            return vx, vy
        _prev = getattr(self, "_osc_prev", None)
        _flicks = getattr(self, "_osc_flicks", 0)
        _win_t0 = getattr(self, "_osc_win_t0", 0.0)
        _break_until = getattr(self, "_osc_break_until", 0.0)
        _now = rospy.Time.now().to_sec()

        # 脱困期内：输出固定直行方向（沿脱困时刻锁定航向），不给守卫改写机会。
        # 但雷达近障强介入（_radar_strict=True）时让位给雷达输出——严禁脱困直行
        # 顶着真实障碍飞（近障≥脱困，振荡是慢病，撞墙是急症）。
        if _now < _break_until:
            if getattr(self, "_radar_strict", False):
                return vx, vy
            _hd = getattr(self, "_osc_break_hd", 0.0)
            _sp = min(math.hypot(vx, vy),
                      getattr(self, "_osc_break_sp", OSC_BREAKOUT_SPD))
            return _sp * math.cos(_hd), _sp * math.sin(_hd)

        spd = math.hypot(vx, vy)
        if spd < 0.05:
            return vx, vy

        hd = math.atan2(vy, vx)
        if _prev is not None:
            _dh = abs(hd - _prev)
            while _dh > math.pi:
                _dh = abs(2 * math.pi - _dh)
            if _dh > math.radians(120.0):
                _flicks += 1
                if _win_t0 == 0.0:
                    _win_t0 = _now
        self._osc_prev = hd
        self._osc_flicks = _flicks
        self._osc_win_t0 = _win_t0

        if _win_t0 == 0.0:
            return vx, vy
        _win_span = _now - _win_t0
        # 窗口内持续大幅翻转 → 判死锁，触发脱困
        if _win_span >= OSC_WATCH_S and _flicks >= OSC_SWITCH_N:
            # 锁定当前航向直行（脱困期保持恒定），重置检测状态
            self._osc_break_hd = hd
            self._osc_break_sp = min(spd, OSC_BREAKOUT_SPD)
            self._osc_break_until = _now + OSC_BREAKOUT_S
            self._osc_prev = None
            self._osc_flicks = 0
            self._osc_win_t0 = 0.0
            rospy.logwarn('[%s] 守卫交替振荡 %.1fs/%d次 → 强制脱困直行 %.0fs',
                          self.uav_id, _win_span, _flicks, OSC_BREAKOUT_S)
            return self._osc_break_sp * math.cos(hd), self._osc_break_sp * math.sin(hd)
        # 窗口横跨过长但翻转数不足（偶发转向/正常朝向调整）→ 重置窗口，
        # 避免"一次翻转 + 很久以后"误触发脱困。真振荡会持续翻转、窗口重开。
        if _win_span > max(OSC_WATCH_S, 3.0) * 2.0 and _flicks < OSC_SWITCH_N:
            self._osc_prev = None
            self._osc_flicks = 0
            self._osc_win_t0 = 0.0
        return vx, vy

    def _map_guard_velocity(self, vx, vy):
        """飞行边界护栏；搜索栅格外圈只用于观察边缘目标。

        越界时锁死朝外分量；边界 MAP_GUARD_MARGIN 内禁止朝外；再往外扩
        MAP_GUARD_SOFT 一段做线性减速，避免高速贴边转弯时惯性冲出栅格
        （UAV3 坠毁后位置 0.5s 内跳 20m，就是撞楼+边界外失控的组合）。
        """
        wxy = self.world_xy
        if wxy is None:
            return vx, vy
        xmin, xmax, ymin, ymax = flight_bounds()
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
            vx = max(vx, -0.5)
        elif x >= xmax - soft and vx > 0:
            vx = min(vx, 0.5)
        if y <= ymin + soft and vy < 0:
            vy = max(vy, -0.5)
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

        === 2026-10-05 比赛规则硬约束：分级回收速度 ===
        旧逻辑：固定 OOB_RECOVER_SPEED=3 m/s + 接近目标减速。
        比赛硬约束要求：「不能撞墙后停止行为」→ 必须能从越界状态可靠收回。
        分级回收：超出越多 → 回收越快（MAX_SPEED 上限），但接近 inset 时仍按
        旧逻辑减速（防回弹穿边）。
        """
        wxy = self.world_xy
        if wxy is None:
            return None
        xmin, xmax, ymin, ymax = flight_bounds()
        x, y = wxy
        if xmin <= x <= xmax and ymin <= y <= ymax:
            return None
        inset = min(max(OOB_RECOVER_INSET, FLIGHT_INSET),
                    (xmax - xmin) * 0.25, (ymax - ymin) * 0.25)
        tx = min(max(x, xmin + inset), xmax - inset)
        ty = min(max(y, ymin + inset), ymax - inset)
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return 0.0, 0.0
        # 分级速度：d>1m → MAX_SPEED 全力回收；d≤1m → 线性减速到 OOB_RECOVER_SPEED。
        # 不超过 OOB_RECOVER_SPEED + 2.0 m/s（防过冲）。
        if d > 1.0:
            spd = MAX_SPEED
        else:
            spd = min(OOB_RECOVER_SPEED, d) + 0.5 * d * (MAX_SPEED - OOB_RECOVER_SPEED)
        spd = min(spd, OOB_RECOVER_SPEED + 2.0)
        return spd * dx / d, spd * dy / d

    def _close_range_final_guard(self, vx, vy):
        """Keep a vetted final command from carrying the camera through a person.

        Emergency radar retreat and close friend separation have higher
        priority.  In the ordinary case this only scales an already checked
        vector; it never invents an outward escape direction through a wall.
        """
        assignment = getattr(self, 'assignment', None)
        tid = getattr(assignment, 'target_id', None)
        if (getattr(assignment, 'task_type', None) != 1 or
                tid != getattr(self, '_orbit_target', None) or
                getattr(self, '_radar_strict', False) or
                getattr(self, '_radar_retreating', False)):
            return vx, vy
        own = self.world_xy
        target = getattr(self, 'targets', {}).get(tid)
        if own is None or target is None:
            return vx, vy
        now = rospy.Time.now().to_sec()
        seen = getattr(self, '_t_seen', {}).get(tid, 0.0)
        if not 0.0 <= now - seen <= TARGET_TTL:
            return vx, vy
        my_alt = (self.local_z if self.local_z is not None
                  else self.altitude_layer)
        for fid, (fx, fy, fz) in getattr(self, '_friend_positions', {}).items():
            if now - getattr(self, '_friend_seen', {}).get(fid, -1e18) > 1.0:
                continue
            friend_alt = my_alt if fz is None else fz
            if math.hypot(math.hypot(fx - own[0], fy - own[1]),
                          friend_alt - my_alt) < SAFE_3D:
                return vx, vy
        limited = cap_inward_close_speed(vx, vy, own[0], own[1],
                                         target[0], target[1])
        if limited != (vx, vy):
            rospy.logwarn_throttle(
                2.0, '[%s] 末级近距径向限速 tid=%s dist=%.2fm '
                'vin=(%.2f,%.2f) vout=(%.2f,%.2f)',
                self.uav_id, tid,
                math.hypot(target[0] - own[0], target[1] - own[1]),
                vx, vy, limited[0], limited[1])
        return limited

    def _send_vel(self, vx, vy, vz=None):
        """下发 ENU 水平速度 + 自适应高度。

        vz=None 时按当前高度竖直 P 控制爬升到自适应高度；预热/悬停阶段显式传
        vz=0.0 发纯零速，避免 EKF 未收敛时带爬升速度导致 OFFBOARD 被拒。
        """
        requested_speed = math.hypot(vx, vy)
        if getattr(self, '_localization_fault', False):
            # Neither crash recovery nor takeoff may override a lost position
            # frame. Keep zero horizontal output until AUTO.LAND takes over.
            vx = vy = 0.0
            vz = None
            self._crash_flagged = False
        # === 国家一等奖修（2026-10-05）：CRASH 优先级提到 EST_GUARD 之前 ===
        # 原顺序：先 EST_GUARD 判定不可信→悬停等待→CRASH 强制爬升永远发不出去。
        # 撞墙恢复是「物理救命」级，必须先发，绕开 EST_GUARD 闸门。
        if vz is None and CRASH_DETECT and self._takeoff_done and getattr(self, '_crash_flagged', False) \
                and self.local_z is not None:
            try:
                target_alt = self.altitude_layer or ALT_BASE
            except Exception:
                target_alt = ALT_BASE
            # === v14 Fix C：恢复爬升必须有高度/时间双上限 ===
            # CRASH_RECOVER_VZ=1.5 m/s 无限爬升会冲破 6m 红线（EKF z 卡低时
            # local_z 判据失效，用时间兜底强制停止）。到顶/超时即悬停。
            _vz = CRASH_RECOVER_VZ
            _now_s = rospy.Time.now().to_sec()
            if self.local_z >= ALT_RECOVER_CEIL:
                _vz = 0.0
            elif self._crash_t0 is not None and _now_s - self._crash_t0 >= CRASH_RECOVER_MAX_S:
                _vz = 0.0
            if _vz == 0.0:
                # 本分支在 _send_vel 的最前面直接 return；若只把 vz 置零，
                # 下方通常负责清标志的代码永远到不了，飞机会永久原地等待。
                self._crash_flagged = False
                self._crash_cnt = 0
                self._crash_t0 = None
                rospy.logwarn('[CRASH] %s 恢复达到高度/时间上限，清除恢复标志',
                              self.uav_id)
            self._flight_output(0.,0.,_vz,target_alt=min(target_alt,ALT_RECOVER_CEIL))
            return

        # === 国家一等奖修（2026-10-05）：EST_GUARD 雪崩熔断 ===
        # 累计悬停 EST_FROZEN_N 拍仍未恢复 → 判定 EKF 永久崩坏 → 强制 LAND。
        # 不能再让飞机"看似悬停实则撞墙"持续到比赛结束。
        # === v18 修复：熔断触地复飞 ===
        # v17 实测（logs_20261007_213201）：agent_0/1/2 熔断 LAND 落地后
        # _ekf_frozen 永不清除 → 5/6 架趴地到比赛结束（雷达打地面假近障 +
        # 相机看地面 → 0 检测 0 播报 → 无法消除 actor）。EKF 雪崩是阵发的：
        # 触地 + EKF 恢复可信 → 清熔断状态重新起飞。复飞走「起飞垂直爬升门」；
        # 若 EKF 再次雪崩会再次熔断，安全闭环。注意：恢复检查必须在外层
        # 「不可信」分支之前——恢复后 trustworthy=True 永远进不了该分支。
        _recover_ready = (getattr(self, '_ekf_frozen', False)
                          and not getattr(self, '_crash_flagged', False)
                          and self._est_trustworthy()
                          and self.local_z is not None
                          and self.local_z < MIN_CRUISE_ALT - 0.3)
        if _recover_ready:
            _now_s = rospy.Time.now().to_sec()
            if getattr(self, '_ekf_recover_since', None) is None:
                self._ekf_recover_since = _now_s
            _recover_ready = (_now_s - self._ekf_recover_since >= 2.0)
        else:
            self._ekf_recover_since = None
        if _recover_ready:
            rospy.logwarn('[%s] 熔断后低空稳定且 EKF 恢复可信 → 清熔断状态复飞',
                          self.uav_id)
            self._ekf_frozen = False
            self._ekf_recover_since = None
            self._est_hold_n = 0
            self._offboard_rearm_done = False
            self._takeoff_done = False   # 重新走垂直爬升门
            self._climb_latch = False
            self._climb_t0 = None
            try:
                if not self.state.armed:
                    self.arm_srv(True)
                if self.state.mode != 'OFFBOARD':
                    self.mode_srv(custom_mode='OFFBOARD')
            except Exception:
                pass
            self._send_vel(0.0, 0.0, vz=CLIMB_VZ)
            return
        if vz is None and not self._est_trustworthy():
            self._est_hold_n = getattr(self, '_est_hold_n', 0) + 1
            _cap = min(ALT_TARGET_CAP, ALT_HARD_CEIL - ALT_HARD_MARGIN)
            _base = self.altitude_layer or ALT_BASE
            _alt = max(MIN_CRUISE_ALT, min(_cap, _base))
            if self._est_hold_n >= EST_FROZEN_N and not getattr(self, '_ekf_frozen', False):
                # 熔断：累计悬停超过 N 拍 → 切 LAND 让 PX4 接管 → 安全落地
                # v14 Fix F：AUTO.RTL 不可用 —— PX4 默认 RTL_RETURN_ALT=60m，
                # 若模式切换被接受会爬到 60m 必冲破 6m 红线 score=0；改 AUTO.LAND
                # 原地降落是唯一安全的 PX4 接管路径。
                self._ekf_frozen = True
                try:
                    rospy.logerr('[%s] EKF 雪崩熔断：累计 %d 拍位置不可信 → 强制 LAND',
                                 self.uav_id, self._est_hold_n)
                    self.mode_srv(custom_mode='AUTO.LAND')
                except Exception as e:
                    rospy.logwarn('[%s] 切 LAND 失败: %s', self.uav_id, e)
                self._last_cmd_v = (0.0, 0.0)
                self._last_flight_v = (0.0, 0.0)
                return
            if getattr(self, '_ekf_frozen', False):
                # v14 Fix E：已熔断不能静默 return —— offboard stream 停止触发
                # PX4 failsafe HOLD，EKF2 z 漂移在 HOLD 下持续爬升（v14 agent_4
                # 实测 236s 漂过 6m 红线）。继续发零速保持 stream 活跃；若模式
                # 已被 failsafe 切走，一次性尝试切回 OFFBOARD 让零速悬停指令生效
                # （LAND 已成功的场合 PX4 在 AUTO 模式下忽略 offboard，无害）。
                self._flight_output(0.,0.,0.,target_alt=_alt)
                if not self._offboard_rearm_done:
                    _cur = getattr(self.state, 'mode', None)
                    if _cur not in ('OFFBOARD', 'AUTO.LAND'):
                        try:
                            self.mode_srv(custom_mode='OFFBOARD')
                            rospy.logwarn('[%s] 熔断后切回 OFFBOARD 零速悬停（替代 failsafe HOLD）',
                                          self.uav_id)
                        except Exception:
                            pass
                    self._offboard_rearm_done = True
                return
            rospy.logwarn_throttle(2.0,
                '[%s] 位置估计不可信（近 %.2fs 平均 %.1f m/s > %.1f m/s）→ 悬停'
                '等待 EKF 收敛（累计 %d 拍 / 熔断 %d 拍）', self.uav_id, EST_WIN_S,
                (self._est_recent_speed() or -1.0), MAX_SPEED * EST_V_RATIO,
                self._est_hold_n, EST_FROZEN_N)
            self._last_cmd_v = (0.0, 0.0)
            self._last_flight_v = (0.0, 0.0)
            self._flight_output(0.,0.,0.,target_alt=_alt)
            return

        # === 位置估计可信性闸门（EST_GUARD，2026-10-03，最高优先级）===
        # 只在自动飞行路径生效（vz is None；预热/降落都显式传 vz，不受影响）。
        # 估计不可信时绝不做任何基于 world_xy 的决策 —— 实测「越界主动回收 →
        # (0,3) 当前(63.2,-66.1)」是在追一个 100m 外的幻影，越追越远。
        # （上方已替换为本修复的雪崩熔断版；此注释保留说明历史脉络）

        # 最高优先级：越界主动回收，绕过其他守卫（界外无障碍）。
        rec = self._bounds_recovery_velocity()
        visual_slew = False
        view_brake_room = view_brake_need = None
        view_brake_fallback = False
        if rec is not None:
            vx, vy = rec
            rospy.logerr_throttle(2.0,
                '[%s] 越界主动回收 → (%.1f,%.1f) 当前(%.1f,%.1f)',
                self.uav_id, vx, vy,
                self.world_xy[0], self.world_xy[1])
            # === 2026-10-05 比赛规则硬约束：回收时跳过加速度限幅 ===
            # 撞墙瞬间必须立刻反向，不能让 MAX_ACC/CTRL_RATE=0.125 m/s 每帧的
            # 爬升率拖慢反向。直接更新 _last_cmd_v 为目标速度，下一帧限幅从
            # 此值起算（视觉上等效「瞬时反向」，PX4 位置环仍受其内部 acc 约束）。
            if OOB_BYPASS_ACC_LIM:
                self._last_cmd_v = (vx, vy)
        else:
            # 雷达先修正水平速度，再经过统一的加速度限幅和高度护栏。
            before_guard = (vx, vy)
            self._radar_retreating = False  # 重置标记
            self._radar_strict = False      # R1：每帧重置（雷达守卫内按需置 True）
            vx, vy = self._radar_guard_velocity(vx, vy)
            # 无激光时栅格兜底（雷达在线也再过一道，双保险）
            # R1：雷达强介入时 _grid_guard_velocity 内部已自动让位只夹速度
            vx, vy = self._grid_guard_velocity(vx, vy)
            vx, vy = self._map_guard_velocity(vx, vy)
            # R1 振荡死锁检测：守卫交替反向（雷达/栅格对阵）持续太久 → 强制
            # 沿当前自由扇区直行 2s，期间两守卫只做速度限幅（v26 agent_3 例证）
            before_speed = math.hypot(*before_guard)
            after_speed = math.hypot(vx, vy)
            guard_changed = (before_speed >= .3 and after_speed >= .05 and
                (vx*before_guard[0]+vy*before_guard[1]) <
                math.cos(math.radians(45.))*before_speed*after_speed)
            vx, vy = self._oscillation_breakout(vx, vy, guard_changed=guard_changed)
            # Safety interventions retain their existing response. The softer
            # visual envelope is only for an ordinary, clear-space command.
            now_view = rospy.Time.now().to_sec()
            view_context = self._view_motion_context(now_view) if vz is None else None
            visual_slew = (vz is None and
                view_context is not None and
                not self._radar_strict and not self._radar_retreating and
                math.hypot(vx-before_guard[0], vy-before_guard[1]) < 1e-8 and
                not self._view_friend_conflict(now_view))
            last_v = getattr(self, '_last_cmd_v', None)
            if visual_slew and last_v is not None:
                view_brake_room, view_brake_need = view_braking_distances(
                    last_v, *view_context['own'], *view_context['target'],
                    *view_context['velocity'], radius=view_context['radius'])
                # Fresh imagery can arrive at 6 m/s only 12 m from an actor.
                # When 1.2 m/s² cannot stop within the remaining ring margin,
                # preserve the existing ordinary/close emergency brake.
                view_brake_fallback = view_brake_need > view_brake_room
                visual_slew = not view_brake_fallback

        guard_stop = math.hypot(vx, vy) < 0.05

        # === 2026-09-27：水平加速度限幅 ===
        # 避让增益提高后，ORCA 输出可能在相邻帧跳到近乎反向（22:29 轮事故：
        # iris_3 因此把高度超调从 0.2m 放大到 1.5m，冲过官方 6m 红线，score=0）。
        # 这里把「指令加速度」钉死，PX4 位置环才有能力跟踪，机身不再大幅倾斜。
        # === 2026-10-05 撞墙修复：雷达三面堵死后退旁路加速度限幅 ===
        # _radar_guard 在三面堵死时设 self._radar_retreating=True，否则限幅
        # (MAX_ACC=2.5 m/s² / CTRL_RATE=20Hz = 0.125 m/s 每帧) 会把 1.5 m/s
        # 后退速度夹成几帧才能爬到目标速度，飞机在 0.3m 墙里推不开。
        if not hasattr(self, "_last_cmd_v"):
            self._last_cmd_v = None
        visual_slew_applied = False
        if not guard_stop and self._last_cmd_v is not None and not getattr(self, "_radar_retreating", False):
            vx, vy = horizontal_slew_limit(
                vx, vy, self._last_cmd_v,
                close_mode=bool(getattr(self, '_confirm_close', False)),
                visual_mode=visual_slew)
            visual_slew_applied = visual_slew
        if guard_stop:
            vx, vy = 0.0, 0.0
        # Breakout and slew limiting can reintroduce an outward command after
        # the first guard.  Recheck the exact velocity that will be published.
        vx, vy = self._map_guard_velocity(vx, vy)
        if rec is None and vz is None:
            # `_fly_orbit` protects the 8 m observation ring, but radar/grid
            # detours, breakout and slew limiting may override its direction.
            # Scale the *final* inward velocity before publishing.  Emergency
            # radar and ORCA proximity still retain precedence in the helper.
            vx, vy = self._close_range_final_guard(vx, vy)
        vx, vy = self._final_motion_gate(vx, vy)
        self._last_cmd_v = (vx, vy)
        # A safe stop is correct, but the view planner must know that its
        # observation point cannot be maintained. Never override this stop.
        now_control = rospy.Time.now().to_sec()
        obstructed = (getattr(self, '_orbit_target', None) is not None and
                      requested_speed > .3 and math.hypot(vx, vy) < .2)
        if obstructed:
            if getattr(self, '_tracking_blocked_since', None) is None:
                self._tracking_blocked_since = now_control
        else:
            self._tracking_blocked_since = None

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
        # === v16 Fix D 缓解：自适应降层 ===
        # EKF z 估计病态时 P 控制持续爬升 → 闩锁反复触发（v15 agent_5 每 7~14s
        # 一次）。触发时永久降层 0.5m（下限 3.5m），让 alt_error 收敛、闩锁可
        # 解锁，飞机在稍低但可用的层高继续任务（对 2D 雷达/俯视感知影响小）。
        target_alt = max(MIN_CRUISE_ALT,
                         min(_alt_cap, base_alt + alt_offset - self._alt_sag))
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
            # 限制升降速度（2026-10-06：爬升 1.0→0.6，异常爬升/弹飞时护栏来得及；
            # 下降保持 -1.0 不变，保应急下降能力）
            if cmd.twist.linear.z > 0.6:
                cmd.twist.linear.z = 0.6
            if cmd.twist.linear.z < -1.0:
                cmd.twist.linear.z = -1.0
        # Routine view/obstacle height changes cannot continue after a near
        # horizontal obstacle stop. Emergency ceiling descent is applied next.
        if vz is None:
            cmd.twist.linear.z = self._guard_vertical_clearance(cmd.twist.linear.z)
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
        _now = rospy.Time.now().to_sec()
        if _now - self._last_csv_t >= 0.2:
            _target = self.assignment
            _wxy = self.world_xy
            _view_log = self._fresh_tracking_view(_now) or {}
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
                is_captured=False,  # Geometry cannot establish judge success.
                orca_vx=self._last_orca_v[0], orca_vy=self._last_orca_v[1],
                cmd_vx=cmd.twist.linear.x, cmd_vy=cmd.twist.linear.y,
                cmd_vz=cmd.twist.linear.z, source="swarm_agent",
                visual_slew=visual_slew_applied,
                horizontal_stop=math.hypot(cmd.twist.linear.x, cmd.twist.linear.y) < .05,
                view_sample_s=_view_log.get('sample_s'),
                view_needs_space=_view_log.get('needs_space'),
                view_range_floor=_view_log.get('range_floor'),
                view_brake_room=view_brake_room,
                view_brake_need=view_brake_need,
                view_brake_fallback=view_brake_fallback)
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
                # v14 Fix C：记录 flag 置位的 ROS 时间，供时间上限兜底
                self._crash_t0 = rospy.Time.now().to_sec()
                try:
                    rospy.logerr('[CRASH] %s 疑似坠地 z=%.2f target=%.2f v=%.2f '
                                 '(landed_state 仍可能报 IN_AIR) → 强制爬升恢复 '
                                 '(上限 EKF z=%.2fm 或 %.1fs 后悬停)',
                                 getattr(self, 'uav_id', getattr(self, 'ns', '?')),
                                 self.local_z, target_alt, math.hypot(vx, vy),
                                 ALT_RECOVER_CEIL, CRASH_RECOVER_MAX_S)
                except Exception:
                    pass
            # 坠毁恢复：一旦 flag，持续以 CRASH_RECOVER_VZ 爬升，直到高度回到
            # target_alt 的 80% 以上再清 flag，避免坠地后继续下发水平速度撞楼。
            # v14 Fix C：增加 EKF 高度 + 时间双上限 —— CRASH_RECOVER_VZ=1.5 m/s
            # 恒爬升在 EKF z 卡低时会冲破 6m 红线（local_z 永远 < target_alt*0.8
            # 导致 flag 永不清除）。双上限任意触发即 vz=0 + 清 flag，恢复正常控制。
            if getattr(self, '_crash_flagged', False):
                _now_s = rospy.Time.now().to_sec()
                _timed_out = self._crash_t0 is not None and \
                             _now_s - self._crash_t0 >= CRASH_RECOVER_MAX_S
                _hit_ceil = self.local_z >= ALT_RECOVER_CEIL
                if _hit_ceil or _timed_out:
                    cmd.twist.linear.z = 0.0
                    self._crash_flagged = False
                    self._crash_cnt = 0
                    self._crash_t0 = None
                    rospy.logwarn('[CRASH] %s 恢复爬升达上限（z=%.2f >= ceil=%.2f 或 '
                                  '已爬 %.1fs）→ 强制清 flag 悬停，避免真值冲 6m 红线',
                                  getattr(self, 'uav_id', '?'), self.local_z,
                                  ALT_RECOVER_CEIL, CRASH_RECOVER_MAX_S)
                else:
                    cmd.twist.linear.z = CRASH_RECOVER_VZ
                cmd.twist.linear.x = 0.0
                cmd.twist.linear.y = 0.0
                if self.local_z >= target_alt * 0.8:
                    self._crash_flagged = False
                    self._crash_cnt = 0
                    self._crash_t0 = None
                    rospy.loginfo('[CRASH] %s 高度恢复到 %.2f，清除坠机标志',
                                  getattr(self, 'uav_id', '?'), self.local_z)
        # === 偏航对准（追踪/盘旋）===
        # 机头持续指向 _look_at，保证水平双目在圆形轨迹中不把目标甩出 ±45° 视场。
        # 非追踪阶段 _look_at=None，angular.z 保持 0（由 PX4 自管偏航）。
        _wxy = self.world_xy
        if self._look_at is not None and _wxy is not None:
            _bear = math.atan2(self._look_at[1] - _wxy[1],
                               self._look_at[0] - _wxy[0])
            _yaw_now = rospy.Time.now().to_sec()
            view = self._fresh_tracking_view(_yaw_now)
            _yaw_source = 'position'
            if view is not None:
                _bear = view['bearing']  # measured optical ray, not estimated depth
                _yaw_source = 'real_image'
            _fast_view = self._fresh_fast_tracking_view(_yaw_now)
            if _fast_view is not None:
                _bear = _fast_view['bearing']
                _yaw_source = 'fast_image'
            _yerr = (_bear - self.yaw + math.pi) % (2.0 * math.pi) - math.pi
            _yr = YAW_KP * _yerr
            if _yr > YAW_RATE_MAX:
                _yr = YAW_RATE_MAX
            elif _yr < -YAW_RATE_MAX:
                _yr = -YAW_RATE_MAX
            cmd.twist.angular.z = _yr
            self._write_fast_yaw(_yaw_now, 'output', '', _yaw_source,
                                _fast_view, view, _bear, _yr)
        # 记录最终下发的水平速度（供路径未就绪时继续发，保证 offboard 不断流）
        self._last_flight_v = (vx, vy)
        self._flight_output(cmd.twist.linear.x,cmd.twist.linear.y,
                            cmd.twist.linear.z,cmd.twist.angular.z,target_alt,
                            monitor_climb=(vz is None and not getattr(self,"_crash_flagged",False)))

    def _output_motion_safety(self,vx,vy,vz,yaw_rate):
        """One execution check for ordinary commands and early recovery exits."""
        if not MOTION_SAFETY:
            return vx,vy,vz,yaw_rate
        now=rospy.Time.now().to_sec()
        original=(vx,vy,vz,yaw_rate)
        reason=''
        pose_ok=(not getattr(self,'_localization_fault',False) and
                 0. <= now-getattr(self,'_pose_sample_s',-1e18) <= .3)
        if not pose_ok or not all(math.isfinite(v) for v in original):
            vx=vy=vz=yaw_rate=0.
            reason='untrusted_pose_or_command'
        else:
            measured=getattr(self,'_measured_velocity',None)
            if measured is None or not 0. <= now-measured[0] <= .3:
                vx=vy=0.
                reason='stale_measured_velocity'
            elif math.hypot(vx,vy) > 1e-9:
                own=self.world_xy
                scan=getattr(self,'_scan',None)
                scan_age=now-getattr(self,'_scan_t',-1e18)
                if own is None or scan is None or not 0. <= scan_age <= RADAR_FRESH_S:
                    vx=vy=0.
                    reason='stale_scan_or_frame'
                else:
                    delay=BRAKE_DELAY+max(scan_age,now-measured[0])
                    (vx,vy),reason=braking_cap((vx,vy),measured[1],
                        scan_constraints(scan,self.yaw,(vx,vy)),RADAR_STOP_R,delay,BRAKE_DECEL)
                    (vx,vy),edge_reason=braking_cap((vx,vy),measured[1],
                        boundary_constraints(own,flight_bounds(),FLIGHT_INSET),
                        0.,delay,BRAKE_DECEL)
                    reason=edge_reason or reason
                    vx,vy=self._final_motion_gate(vx,vy)
            # Crash recovery is not permission to climb beside a scaffold.
            # Hard ceiling descent and landing retain their explicit priority.
            if not getattr(self,'_landing',False) and self.local_z <= ALT_HARD_CEIL:
                old_vz=vz
                vz=self._guard_vertical_clearance(vz)
                if vz != old_vz:
                    reason=reason or 'vertical_clearance'
        if original != (vx,vy,vz,yaw_rate):
            rospy.logwarn_throttle(1.,'[FINAL_SAFETY] uav=%s reason=%s intent=%s output=%s',
                self.uav_id,reason or 'final_obstacle_or_friend',original,(vx,vy,vz,yaw_rate))
        self._last_cmd_v=self._last_flight_v=(vx,vy)
        return vx,vy,vz,yaw_rate

    def _guard_executed_climb(self, vz, target_alt, enabled):
        """Watch the command that survives safety/encoding, never a vetoed intent.

        Intentional obstacle/pose/ceiling holds reset the progress window. The
        existing estimator-stall latch still stops an actual sustained climb;
        this gate can only remove upward motion, never request new motion.
        """
        z=self.local_z
        if (not enabled or target_alt is None or z is None or
                getattr(self,'_landing',False) or not getattr(self,'_takeoff_done',False)):
            self._climb_t0=None
            return vz
        now=rospy.Time.now().to_sec()
        alt_error=target_alt-z
        if getattr(self,'_climb_latch',False):
            latched_z=getattr(self,'_climb_latch_z',None)
            if (alt_error < .3 or z < 1.5 or
                    (latched_z is not None and abs(z-latched_z) >= CLIMB_LATCH_DZ)):
                self._climb_latch=False
                self._climb_t0=self._climb_latch_z=None
            else:
                return min(vz,0.)
        if vz <= .2+1e-6 or z <= 1.5:
            self._climb_t0=None
        elif getattr(self,'_climb_t0',None) is None or now < self._climb_t0:
            self._climb_t0,self._climb_z0=now,z
        elif now-self._climb_t0 >= CLIMB_LATCH_S:
            if abs(z-self._climb_z0) < CLIMB_LATCH_DZ:
                self._climb_latch=True
                self._climb_latch_z=z
                rospy.logerr('[CLIMB_STALL] uav=%s executed_vz=%.2f elapsed=%.2f dz=%.2f',
                    self.uav_id,vz,now-self._climb_t0,abs(z-self._climb_z0))
                if self._alt_sag < 1. and target_alt-.5 >= 3.5:
                    self._alt_sag+=.5
                return min(vz,0.)
            self._climb_t0,self._climb_z0=now,z
        return vz

    def _flight_output(self,vx,vy,vz,yaw_rate=0.,target_alt=None,monitor_climb=False):
        """Sole flight-output entry; stale positions use zero velocity holding."""
        now=rospy.Time.now().to_sec()
        pose_ok=(not MOTION_SAFETY or (not getattr(self,'_localization_fault',False) and
                 0. <= now-getattr(self,'_pose_sample_s',-1e18) <= .3))
        if (FLIGHT_OUTPUT == 'pos' and pose_ok):
            self._publish_pos_output(vx,vy,target_alt,vz,yaw_rate,monitor_climb)
            return
        self._publish_velocity_output(vx,vy,vz,yaw_rate,target_alt,monitor_climb)

    def _publish_velocity_output(self,vx,vy,vz,yaw_rate,target_alt=None,monitor_climb=False):
        vx,vy,vz,yaw_rate=self._output_motion_safety(vx,vy,vz,yaw_rate)
        vz=self._guard_executed_climb(vz,target_alt,monitor_climb)
        cmd=TwistStamped();cmd.header.stamp=rospy.Time.now();cmd.header.frame_id='world'
        cmd.twist.linear.x,cmd.twist.linear.y,cmd.twist.linear.z=vx,vy,vz
        cmd.twist.angular.z=yaw_rate
        self.vel_pub.publish(cmd)

    def _final_motion_gate(self, vx, vy):
        """Validate the direction actually sent, after all direction changes.

        This gate only stops/scales. It cannot generate a new avoidance turn.
        """
        vx, vy = self._final_scan_cap(vx, vy)
        if math.hypot(vx, vy) < 1e-9:
            return 0.0, 0.0
        own = self.world_xy
        if own is None:
            return 0.0, 0.0
        now = rospy.Time.now().to_sec()
        own_z = self.local_z
        for uid, (x, y, z) in getattr(self, '_friend_positions', {}).items():
            if not 0. <= now-getattr(self, '_friend_seen', {}).get(uid, -1e18) <= 1.:
                continue
            dx, dy = x-own[0], y-own[1]
            dz = 0. if z is None or own_z is None else z-own_z
            if math.hypot(math.hypot(dx, dy), dz) < SAFE_3D:
                if math.hypot(dx, dy) < .01 or vx*dx+vy*dy > 1e-8:
                    return 0.0, 0.0
        grid = getattr(self, 'grid', None)
        if grid is not None:
            start = grid.world_to_cell(own)
            escaped = grid.is_free(start)
            length = min(POS_SP_STEP_MAX, math.hypot(vx, vy)*POS_SP_LEAD)
            steps = max(1, int(math.ceil(length/max(.05, grid.resolution*.5))))
            for i in range(1, steps+1):
                dt = min(POS_SP_LEAD, length/math.hypot(vx, vy))*i/steps
                cell = grid.world_to_cell((own[0]+vx*dt, own[1]+vy*dt))
                if cell is None:
                    return 0.0, 0.0
                # Permit leaving the initial inflation footprint. A newly
                # entered measured obstacle or a later inflated cell blocks.
                raw_free = getattr(grid, 'is_free_raw', grid.is_free)
                if (escaped and not grid.is_free(cell)) or (cell != start and not raw_free(cell)):
                    return 0.0, 0.0
                escaped = escaped or grid.is_free(cell)
        return vx, vy

    def _publish_pos_output(self, vx, vy, target_alt, vz=None, yaw_rate=0.0, monitor_climb=False):
        """速度 → 位置设定点。

        速度指令本身已经过加速度限幅 / 雷达 / 栅格 / 地图边界四层守卫，这里只做
        「积分成位置」这一个动作，逻辑与限幅全部保留：

          sp = 当前位置 + v * POS_SP_LEAD

        每帧都以**实际 localize 位置**为基准重算（不累积上次结果），因此不存在
        积分漂移；单帧增量再夹在 POS_SP_STEP_MAX 内，防止守卫给出的速度很大时
        一步跳太远。高度直接用 target_alt（官方 >6m 判 0，这里目标 2.8m）。
        """
        lxy = self.local_xy
        if lxy is None or self.local_z is None:
            # 位置还没就绪 → 退回速度通道保持 offboard 流不断
            self._publish_velocity_output(vx,vy,0. if vz is None else vz,yaw_rate,target_alt,monitor_climb)
            return
        step_x = vx * POS_SP_LEAD
        step_y = vy * POS_SP_LEAD
        step = math.hypot(step_x, step_y)
        if step > POS_SP_STEP_MAX and step > 1e-9:
            k = POS_SP_STEP_MAX / step
            step_x *= k
            step_y *= k
        off = getattr(self, 'offset', None)
        if off is not None:
            step_x, step_y = bounded_step(
                (lxy[0]+off[0], lxy[1]+off[1]), (step_x, step_y), flight_bounds(),
                FLIGHT_INSET if getattr(self, '_takeoff_done', False) else 0.)
        # Validate the actual displacement, then never change its direction.
        vx, vy = self._final_motion_gate(step_x/POS_SP_LEAD, step_y/POS_SP_LEAD)
        vx,vy,checked_vz,yaw_rate=self._output_motion_safety(vx,vy,
            0. if vz is None else vz,yaw_rate)
        if MOTION_SAFETY:
            vz=checked_vz
        step_x, step_y = vx*POS_SP_LEAD, vy*POS_SP_LEAD
        self._last_cmd_v = self._last_flight_v = (vx, vy)
        sp = PoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = 'map'
        sp.pose.position.x = lxy[0] + step_x
        sp.pose.position.y = lxy[1] + step_y
        # 位置模式也必须执行最终安全竖直速度，尤其是悬停、硬顶下降和坠机恢复。
        if self._landing:
            z = max(0.0, self.local_z - 0.6)
        elif vz is not None:
            z = self.local_z + vz * POS_SP_LEAD
        elif target_alt is not None:
            z = target_alt
        else:
            z = self.local_z
        z=max(0.0,min(z,ALT_RECOVER_CEIL))
        executed_vz=self._guard_executed_climb((z-self.local_z)/POS_SP_LEAD,
                                               target_alt,monitor_climb)
        sp.pose.position.z=self.local_z+executed_vz*POS_SP_LEAD
        yaw = getattr(self, 'yaw', 0.0) or 0.0
        yaw += max(-YAW_RATE_MAX, min(YAW_RATE_MAX, yaw_rate)) * POS_SP_LEAD
        sp.pose.orientation.z = math.sin(yaw * 0.5)
        sp.pose.orientation.w = math.cos(yaw * 0.5)
        self._pos_sp = (sp.pose.position.x, sp.pose.position.y, sp.pose.position.z)
        self._pos_sp_t = rospy.Time.now().to_sec()
        self.pos_pub.publish(sp)
        # === 诊断埋点（POS_SPV 开启时打印，验证方向与限幅是否符合预期） ===
        if POS_SPV:
            rospy.loginfo_throttle(
                1.0,
                "[%s][SPV] v=(%.2f,%.2f) cur_local=(%.2f,%.2f) cur_world=%s "
                "sp_local=(%.2f,%.2f,%.2f)",
                self.uav_id, vx, vy, lxy[0], lxy[1],
                ("%.2f,%.2f" % self.world_xy) if self.world_xy else "None",
                sp.pose.position.x, sp.pose.position.y, sp.pose.position.z)

    def _need_replan_track(self, goal, tid=None):
        """追踪移动目标时的 A* 重规划节流。

        原实现每个控制循环都重跑全图 A*（目标一移动就整条 replan），实测把控制
        频率从 20Hz 拖到 5.5Hz，且日志刷屏（单架曾累计 10020 次规划）。
        这里改成：无路径 / 目标移动超阈值 / 距上次规划超时，三者满足其一才重规划。
        2026-10-04：目标处于 FLEE（2 m/s 逃跑）时改用收紧节流，防止路径目标
        滞后 4m 把 20m 探测门顶穿（真实仿真 t1 两次确认清零的根因）。
        """
        if not self.path:
            return True
        if self._last_track_goal is None:
            return True
        flee = False
        if tid is not None:
            flee = self._target_fleeing(tid)
        mv = TRACK_REPLAN_MOVE_FLEE if flee else TRACK_REPLAN_MOVE
        sec = TRACK_REPLAN_SEC_FLEE if flee else TRACK_REPLAN_SEC
        moved = math.hypot(goal[0] - self._last_track_goal[0],
                           goal[1] - self._last_track_goal[1])
        if moved >= mv:
            return True
        if (rospy.Time.now().to_sec() - self._last_track_plan_t) >= sec:
            return True
        return False

    def _compute_plan(self, goal_xy):
        """从当前世界坐标 A* 规划到 goal_xy。纯计算：返回 (path, goal, ok)，
        不修改 self.path / self.path_target（由后台线程调用，提交交给主循环视角）。"""
        if self.world_xy is None:
            return [], goal_xy, False
        route = plan(self.grid, self.world_xy, goal_xy, connectivity=8)
        free_start = None
        if not route.success and route.reason == "START_OCCUPIED":
            # 起点落在膨胀障碍内（刚起飞/贴墙/SLAM 自回波污染残留）→ 找最近自由格重试
            nearest = self._nearest_free_cell()
            if nearest is None:
                # v11 复盘（2026-10-07）：自回波污染盘（0.32m 命中 + 2m 深度填充
                # + 2.5m 膨胀 ≈4.8m 死区）叠加真实障碍时，8m 内可能找不到自由格
                # → 规划永久失败。放宽到 20m 确保能跳出污染区。
                nearest = self._nearest_free_cell(radius_m=20.0)
            if nearest is not None:
                free_start = nearest
                route = plan(self.grid, nearest, goal_xy, connectivity=8)
                if not route.success:
                    # 2026-10-05 国家一等奖修复：第二次仍 START_OCCUPIED 时再次放宽
                    # 起点自由格（12m 半径）— 实测 actor 后续位置恰好覆盖 5m 内的
                    # 所有自由栅格，让 A* 永远起点失败；12m 已能跳出 actor 半径
                    # （actor 半径 0.4m + 膨胀 1.5m ≈ 2m），确保能找到无覆盖起点。
                    nearest2 = self._nearest_free_cell(radius_m=12.0)
                    if nearest2 is None:
                        nearest2 = self._nearest_free_cell(radius_m=20.0)
                    if nearest2 is not None:
                        free_start = nearest2
                        route = plan(self.grid, nearest2, goal_xy, connectivity=8)
        if not route.success and route.reason == "GOAL_OCCUPIED":
            # 目标观测可能落在建筑边缘或膨胀区；停在目标附近的自由点，
            # 不要把一个不可站立的观测点变成整条追踪路径的失败。
            nearest_goal = self._nearest_free_cell(goal_xy)
            if nearest_goal is not None:
                # v12 复盘（logs_20261007_120121）：起点可能仍在膨胀区内（贴墙
                # 飞行常态），从被占起点重规划必然再次 START_OCCUPIED 并覆盖
                # 级联结果——agent_2 以 START_OCCUPIED 记录 30 次实为
                # GOAL_OCCUPIED。改用级联已找到的自由起点（若有）。
                start_xy = free_start if free_start is not None else self.world_xy
                route = plan(self.grid, start_xy, nearest_goal, connectivity=8)
                if not route.success:
                    # 2026-10-05：第二次 GOAL_OCCUPIED 时同步把终点拉到 12m 半径
                    # 自由格再试，避开「actor 在建筑贴边而终点卡死」的常见失败。
                    nearest_goal2 = self._nearest_free_cell(goal_xy, radius_m=12.0)
                    if nearest_goal2 is not None:
                        route = plan(self.grid, start_xy, nearest_goal2, connectivity=8)
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
        path = smooth_path(route.points, samples_per_segment=6)
        rospy.loginfo("[%s] A* 规划：%d 航点 -> %d 平滑点",
                      self.uav_id, len(route.points), len(path))
        return path, goal_xy, True

    # ---------------- 异步规划接口 ----------------
    @serialized_visual_task
    def _request_plan(self, goal_xy):
        """投递一个规划请求并立即返回（不阻塞控制循环）。多次请求只保留最新目标。"""
        with self._plan_lock:
            goal = flight_waypoint(float(goal_xy[0]), float(goal_xy[1]))
            identity = self._segment_identity()
            request = (self._plan_epoch,goal,identity)
            if getattr(self,'_plan_inflight',None) == request:
                return
            self._plan_pending = goal
            self._plan_pending_epoch = self._plan_epoch
            self._plan_pending_identity = identity
            task = getattr(self,'_search_segment',None)
            self._plan_pending_points = (task['points'][task['index']:] if task is not None else None)
        self._plan_event.set()

    def _planner_loop(self):
        """后台守护线程：取最新请求 → 规划 → 原子提交 self.path；失败记录冷却时刻。"""
        while not rospy.is_shutdown():
            self._plan_event.wait()
            with self._plan_lock:
                goal = self._plan_pending
                if goal is None:
                    self._plan_event.clear()
                    continue
                epoch = getattr(self,'_plan_pending_epoch',self._plan_epoch)
                identity = getattr(self,'_plan_pending_identity',None)
                points = getattr(self,'_plan_pending_points',None)
                self._plan_inflight = (epoch,goal,identity)
                self._plan_pending = None
                self._plan_event.clear()
            try:
                blocked, blocked_reason = 0, ''
                if identity is not None:
                    path,target,ok,blocked,blocked_reason = self._compute_segment_plan(points)
                else:
                    path,target,ok = self._compute_plan(goal)
            except Exception as exc:
                rospy.logerr("[%s] 规划线程异常: %s\n%s",
                             self.uav_id, exc, traceback.format_exc())
                ok = False
                blocked, blocked_reason = 1, 'PLANNER_EXCEPTION'
            with self._plan_lock:
                self._plan_inflight = None
                if epoch != self._plan_epoch or identity != self._segment_identity():
                    # A new task arrived while A* computed; never commit its
                    # previous route after the search/pursuit switch.
                    continue
                if identity is not None:
                    self._search_segment['blocked'] = identity[2]+blocked if blocked else 0
                    self._search_segment['reason'] = blocked_reason
                if ok:
                    self.path = path
                    self.path_target = target
                    if identity is not None:
                        self._segment_path_key = identity[:2]
                        self._segment_path_sequence = getattr(self,'_segment_path_sequence',0)+1
                else:
                    # Keep an already safe route for the current task.  A
                    # failed replan must not leave a newly blocked route
                    # active; nor may its last nonzero command be streamed.
                    _wxy = self.world_xy
                    _out = (_wxy is not None and
                            self.grid.world_to_cell(_wxy) is None)
                    _blocked = remaining_path_blocked(
                        self.grid, self.path, _wxy)
                    if not self.path or _out or _blocked:
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

    def _nearest_free_cell(self, center_xy=None, radius_m=8.0):
        """在给定点附近找最近自由栅格，避免全地图 O(w*h) 扫描。

        2026-10-05 国家一等奖修复：搜索半径 5.0m → 8.0m。
        原 5m 在密集建筑群（如 6 座建筑横向排列、相距 5m）时，找不到自由栅格；
        8m 既能覆盖「建筑间距 7m」场景，又仍远小于搜索格 7m 不破坏多机分散逻辑。
        """
        center_xy = center_xy if center_xy is not None else self.world_xy
        if center_xy is None:
            return None
        cx, cy = center_xy
        radius_cells = int(math.ceil(radius_m / self.grid.resolution))
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
                if flight_waypoint(wx, wy) != (wx, wy):
                    continue
                d = math.hypot(wx - cx, wy - cy)
                if d < best_d:
                    best_d = d
                    best = (wx, wy)
        return best

    def _pick_local_goal(self, route=None):
        """沿全局路径从最近点往前取 LOOKAHEAD 距离的引导目标。"""
        route = self.path if route is None else route
        if not route or self.world_xy is None:
            return None
        cx, cy = self.world_xy
        best = 0
        best_dist = float("inf")
        for i, (px, py) in enumerate(route):
            d = math.hypot(px - cx, py - cy)
            if d < best_dist:
                best_dist = d
                best = i
        acc = 0.0
        for i in range(best, len(route) - 1):
            x0, y0 = route[i]
            x1, y1 = route[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if acc + seg >= LOOKAHEAD:
                frac = (LOOKAHEAD - acc) / max(seg, 1e-6)
                return (x0 + (x1 - x0) * frac, y0 + (y1 - y0) * frac)
            acc += seg
        return route[-1]

    def _search_sweep_bearing(self, bearing, distance):
        """A slow search-only look around the route, with no motion changes."""
        now = rospy.Time.now().to_sec()
        previous = getattr(self, '_search_sweep_state', None)
        dt = now - previous[0] if previous is not None else 0.0
        turn = abs(math.atan2(math.sin(bearing - previous[1]),
                              math.cos(bearing - previous[1]))) if previous else 0.0
        error = abs(math.atan2(math.sin(bearing - self.yaw),
                               math.cos(bearing - self.yaw)))
        clearance = getattr(self, '_radar_prev', None)
        enabled = (SEARCH_SWEEP_DEG > 0.0 and SEARCH_SWEEP_PERIOD_S > 0.0 and
                   getattr(getattr(self, 'assignment', None), 'task_type', None) == 0 and
                   getattr(self, '_camera_cue', None) is None and
                   getattr(self, '_orbit_target', None) is None and
                   not getattr(self, '_radar_strict', False) and
                   distance > SEARCH_DECEL_M and error < math.radians(35.) and
                   turn < math.radians(5.) and 0.0 <= dt <= 0.5 and
                   0.0 <= now - getattr(self, '_scan_t', -1e18) <= RADAR_FRESH_S and
                   clearance is not None and min(clearance) >= 8.0)
        phase = (previous[2] + dt if enabled and previous else 0.0)
        self._search_sweep_state = (now, bearing, phase)
        if not enabled:
            return bearing
        return bearing + math.radians(min(25.0, SEARCH_SWEEP_DEG)) * math.sin(
            2.0 * math.pi * phase / SEARCH_SWEEP_PERIOD_S)

    def _set_search_heading(self, desired):
        """Slew search-only heading across replans; attention bypasses this."""
        now = rospy.Time.now().to_sec()
        previous = getattr(self, '_search_heading', None)
        if previous is None or not 0.0 <= now - previous[0] <= 0.5:
            heading = getattr(self, 'yaw', 0.0)
            dt = 1.0 / CTRL_RATE
        else:
            heading = previous[1]
            dt = now - previous[0]
        error = (desired - heading + math.pi) % (2.0 * math.pi) - math.pi
        limit = SEARCH_HEADING_RATE * dt
        heading += max(-limit, min(limit, error))
        self._search_heading = (now, heading)
        wx, wy = self.world_xy
        self._look_at = (wx + 10.0 * math.cos(heading),
                         wy + 10.0 * math.sin(heading))

    def _hold_edge_review(self):
        """Finite inward/corner look after a search visit, below human attention."""
        assignment = getattr(self, 'assignment', None)
        review = getattr(self, '_edge_review', None)
        if (not EDGE_SEARCH_REVIEW or
                getattr(assignment, 'task_type', None) != 0 or
                getattr(self, '_camera_cue', None) is not None or
                getattr(self, '_orbit_target', None) is not None):
            if review is not None:
                review.cancel('person_or_task')
            return False
        now = rospy.Time.now().to_sec()
        own = self.world_xy
        clearance = getattr(self, '_radar_prev', None)
        if (own is None or not self._world_trusted() or
                not 0. <= now-getattr(self, '_pose_sample_s', -1e18) <= .3 or
                getattr(self, '_radar_strict', False) or
                not 0. <= now-getattr(self, '_scan_t', -1e18) <= .3 or
                clearance is None or min(clearance) < RADAR_STOP_R or
                not self.grid.is_free(self.grid.world_to_cell(own))):
            if review is not None:
                review.cancel('pose_or_clearance_unavailable')
            return False
        if review is None:
            review = self._edge_review = EdgeReview((self.cov_grid.x_min,
                self.cov_grid.x_max, self.cov_grid.y_min, self.cov_grid.y_max),
                dwell=SEARCH_SCAN_HOLD_S)
        key = (getattr(assignment, 'search_task_id', ''),
               getattr(assignment, 'search_revision', 0), self._search_scan_key)
        heading = review.step(key, now, own,
            getattr(self, '_search_camera_pose', None), self.cov_grid, self.los.visible)
        if review.last_result is not None:
            rospy.loginfo('[EDGE_SEARCH_REVIEW_END] uav=%s result=%s elapsed=%.2f; '
                          'coverage uses processed frames', self.uav_id,
                          review.last_result['reason'], now-review.last_result['started'])
            review.last_result = None
        if heading is None:
            return False
        if math.isfinite(heading):
            self._set_search_heading(heading)
        rospy.loginfo_throttle(1., '[EDGE_SEARCH_REVIEW] uav=%s cell=%s '
                              'looking inward/along edge; bounded yaw, no target credit',
                              self.uav_id, self._search_scan_key)
        self._send_vel(0., 0.)
        return True

    def _search_scan_step(self):
        """Look at a segment's explicit region before scanning missing subpoints.

        A transit waypoint directly over its cell cannot image that cell with
        a forward camera. Ordinary search releases such a visit; directed
        patrol waits for processed images at the commanded bearing. Neither a
        visit nor this bounded look grants coverage credit.
        """
        assignment = getattr(self, 'assignment', None)
        if (assignment is None or getattr(assignment, 'task_type', None) != 0 or
                self._search_scan_key is None):
            self._send_vel(0.0, 0.0)
            return
        now = rospy.Time.now().to_sec()
        segment = getattr(self, '_search_segment', None)
        look = None
        if (segment is not None and segment.get('purpose') in ('south_patrol', 'building_review') and
                segment['index'] < len(segment.get('look_points', ()))):
            look = segment['look_points'][segment['index']]
        if self._search_scan_yaw0 is None:
            self._search_scan_yaw0 = self.yaw
            self._search_scan_started_t = now
            self._search_scan_phase = None
            self._search_scan_settle_t = None
            self._search_scan_directed_first = None
            self._search_scan_directed_last = None
            self._search_scan_directed_done = False
        if self._search_scan_done or self._search_scan_failed:
            if self._search_scan_done and self._hold_edge_review():
                return
            self._send_vel(0.0, 0.0)
            return
        timeout = (max(SEARCH_SCAN_TIMEOUT_S, SEARCH_DIRECTED_LOOK_TIMEOUT_S)
                   if look is not None else SEARCH_SCAN_TIMEOUT_S)
        if now - self._search_scan_started_t > timeout:
            self._search_scan_failed = True
            rospy.logwarn('[%s] 搜索视角补看超时，保留未覆盖部分: %s',
                          self.uav_id, self._search_scan_key)
            self._send_vel(0.0, 0.0)
            return
        if look is not None and (not self._world_trusted() or
                not 0. <= now-getattr(self, '_pose_sample_s', -1e18) <= .3 or
                self.world_xy is None):
            self._send_vel(0.0, 0.0)
            return
        if look is not None and not self._search_scan_directed_done:
            # The route heading is not an observation heading. Command the
            # reserved cell's centre through the sole search-yaw controller;
            # the final flight safety chain also owns any local correction.
            wx, wy = self.world_xy
            self._set_search_heading(math.atan2(look[1]-wy, look[0]-wx))
        camera = getattr(self, '_search_camera_pose', None)
        if camera is None or not 0.0 <= now-camera[0] <= .5:
            self._send_vel(0.0, 0.0)
            return
        sample, x, y, yaw, altitude, down_pitch = camera
        if math.hypot(x-self.world_xy[0], y-self.world_xy[1]) > 1.0:
            self._send_vel(0.0, 0.0)
            return
        if look is not None and not self._search_scan_directed_done:
            # A commanded bearing or a repeated old image is not proof that
            # the fixed camera has looked into this region. Require two
            # distinct processed frames spanning the normal dwell interval.
            bearing = math.atan2(look[1]-y, look[0]-x)
            yaw_error = math.atan2(math.sin(bearing-yaw), math.cos(bearing-yaw))
            if (sample < self._search_scan_started_t or
                    abs(yaw_error) > SEARCH_SCAN_YAW_TOL):
                self._search_scan_directed_first = None
                self._search_scan_directed_last = None
                self._send_vel(0.0, 0.0)
                return
            if not in_view(x, y, yaw, *look, altitude, down_pitch):
                self._search_scan_directed_first = None
                self._search_scan_directed_last = None
                # Move only for a neutral-pose distance error. Transient pitch
                # can make the actual band empty after arrival; stopping there
                # cannot recover a camera that has drifted too near the look.
                # A neutral-valid distance instead waits for actual new frames
                # to settle. Actual in_view/dwell alone still grants credit.
                vx, vy = self._directed_view_range_velocity(camera, look,
                    segment['points'][segment['index']], SEARCH_ARRIVE_TOL, POS_KP)
                self._send_vel(vx, vy)
                return
            first = self._search_scan_directed_first
            last = self._search_scan_directed_last
            if first is None or last is None or sample < last or sample-last > .7:
                self._search_scan_directed_first = sample
                self._search_scan_directed_last = sample
                self._send_vel(0.0, 0.0)
                return
            self._search_scan_directed_last = sample
            if sample-first < SEARCH_SCAN_HOLD_S:
                self._send_vel(0.0, 0.0)
                return
            self._search_scan_directed_done = True
        points = self._search_observable_points(x, y, altitude, down_pitch)
        if segment is not None and segment.get('purpose') == 'building_review' and not points:
            # A newly blocked/missing ground ray cannot be treated as an
            # empty successfully observed region. Keep the same 8 s budget.
            self._send_vel(0.0, 0.0)
            return
        missing = {i: p for i, p in points.items()
                   if i not in getattr(self, '_search_scan_seen', set())}
        if not missing:
            self._search_scan_done = True
            rospy.loginfo('[SEARCH_VISIT] uav=%s cell=%s imaged=%d observable=%d; '
                          'coverage credited by processed images only',
                          self.uav_id, self._search_scan_key,
                          len(getattr(self, '_search_scan_seen', set())), len(points))
            if self._hold_edge_review():
                return
            self._send_vel(0.0, 0.0)
            return
        desired = next_heading(missing, x, y, self.yaw)
        self._set_search_heading(desired)
        self._send_vel(0.0, 0.0)

    def _search_start_cap(self, vx, vy):
        """Give the first search metres more image time, before safety filters.

        Count measured travel between consecutive search control cycles, not
        commanded distance or distance to a waypoint. Candidate attention and
        tracking bypass this cap; returning to search retains the same budget.
        Long gaps, clock resets and pose jumps cannot consume the budget.
        """
        if SEARCH_START_DISTANCE_M <= 0.0:
            return vx, vy
        progress = getattr(self, '_search_start_progress', None)
        if progress is not None and progress['metres'] >= SEARCH_START_DISTANCE_M:
            return vx, vy
        now = rospy.Time.now().to_sec()
        xy = self.world_xy
        if progress is None:
            progress = self._search_start_progress = {'metres': 0.0, 'last': None}
            rospy.loginfo('[%s] SEARCH_START first %.1fm cap %.2fm/s',
                          self.uav_id, SEARCH_START_DISTANCE_M, SEARCH_START_SPEED)
        if xy is not None and all(math.isfinite(v) for v in (now, *xy)):
            last = progress['last']
            if last is not None:
                dt = now - last[0]
                step = math.hypot(xy[0] - last[1], xy[1] - last[2])
                if 0.0 < dt <= 0.5 and step <= MAX_SPEED * dt + 0.05:
                    progress['metres'] += step
            progress['last'] = (now, *xy)
        else:
            progress['last'] = None
        if progress['metres'] >= SEARCH_START_DISTANCE_M:
            rospy.loginfo('[%s] SEARCH_START complete travelled=%.2fm; cruise=%.2fm/s',
                          self.uav_id, progress['metres'], SEARCH_CRUISE_SPEED)
            return vx, vy
        speed = math.hypot(vx, vy)
        if speed > SEARCH_START_SPEED:
            scale = SEARCH_START_SPEED / speed
            return vx * scale, vy * scale
        return vx, vy

    @serialized_visual_task
    def _control(self):
        """有任务 → A* 绕障飞向格中心；无任务 → 原地悬停。"""
        self._finish_interrupted_camera_cue()
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
            self._check_climb_progress()
            if self._recover_stalled_takeoff():
                return
            if self.local_z is not None and self.local_z >= CLIMB_DONE_ALT:
                self._takeoff_done = True
                self._search_start_progress = None
                self._climb_progress = None
                rospy.loginfo("[%s] 已爬升到 %.1fm，开始水平机动",
                              self.uav_id, CLIMB_DONE_ALT)
            else:
                # 2026-10-07 v12: 显式 vz 爬升（见 CLIMB_VZ 注释）——
                # 自适应目标高度可能被"贴楼降高 -0.5m"拖到 2.3m < 闸门 2.5m
                self._send_vel(0.0, 0.0, vz=CLIMB_VZ)
                return

        if self.world_xy is None:
            # A lost world position cannot be repaired from the same drifting
            # local pose. _send_vel keeps a safe stream and enters AUTO.LAND.
            self._send_vel(0.0, 0.0)
            return

        # A real camera cue interrupts travel and scan immediately, even
        # while the manager is changing the search waypoint or assigning work.
        if self._hold_camera_candidate():
            self._cancel_gimbal_cue('preempted', rospy.Time.now().to_sec())
            return
        if self._hold_gimbal_candidate():
            return

        if self.assignment is None:
            self.path = []
            self.path_target = None
            self._orbit_target = None
            self._target_to_orbit = None
            # === 国家一等奖（2026-10-05）：无任务悬停也保留机头朝向 ===
            # 兜底 _last_local_goal（若存在），避免 dwell 期机头"随便指"。
            if DWELL_LOOKAHEAD and getattr(self, '_last_local_goal', None) is not None:
                self._look_at = self._last_local_goal
            else:
                self._look_at = None
            self._send_vel(0.0, 0.0)
            return

        # === 任务类型处理 ===
        task_type = getattr(self.assignment, 'task_type', 0)

        # 目标追踪任务（task_type=1）：飞向目标位置
        if task_type == 1 and self._target_to_orbit is not None:
            # 冷却闸门：目标在 giveup 冷却内，或已 stale（无位置更新），
            # 安全悬停等待 manager 改派，绝不重入盘旋或朝过期点飞。
            # === v16 修复：旧代码 _orbit_stale() 检查 _orbit_target，但接近阶段
            # 它是 None → 恒 False，stale 拦截形同虚设（v15 agent_0 朝冻结位置
            # 飞 56s 后到圈才被放弃）。改用 assignment.target_id 判 stale；
            # 且已在途中（< STALE_ABORT_DIST_M）时放行继续接近——actor 静止时
            # 冻结位置即真实位置，到圈后由 _fly_orbit 宽限搜索兜底。
            _track_id = getattr(self.assignment, 'target_id', None)
            _now_s = rospy.Time.now().to_sec()
            _in_cooldown = (_track_id and
                            _now_s < self._giveup_until.get(_track_id, 0.0))
            _stale_far = False
            if _track_id and not _in_cooldown:
                _t_seen = self._t_seen.get(_track_id, 0.0)
                if _t_seen and _now_s - _t_seen > TARGET_STALE:
                    if _now_s - _t_seen >= TARGET_STALE + STALE_SEARCH_GRACE:
                        # 接近途中也给冻结坐标绝对期限。旧逻辑只按 40m
                        # 距离判定：35m 内可能一直沿过期点飞到建筑后面。
                        tx0, ty0 = self._target_to_orbit
                        self._discard_old_plan()
                        self._target_to_orbit = None
                        self._orbit_target = None
                        self._confirm_close = False
                        self._confirm_close_tid = None
                        self._look_at = (tx0, ty0)
                        rospy.logwarn('[%s] %s 接近中已 %.1fs 无真实位置，'
                                      '停车等待新图像或管理器改派',
                                      self.uav_id, _track_id, _now_s - _t_seen)
                        self._send_vel(0.0, 0.0)
                        return
                    tx0, ty0 = self._target_to_orbit
                    _d0 = math.hypot(tx0 - self.world_xy[0],
                                     ty0 - self.world_xy[1])
                    _stale_far = _d0 > STALE_ABORT_DIST_M
            if _in_cooldown or _stale_far:
                self._look_at = None
                self._send_vel(0.0, 0.0)
                return
            # Assignment coordinates may be the manager's future intercept,
            # several metres ahead of the camera-derived target position.
            # Using that future point for range/yaw broke an 8m observation
            # ring whenever it crossed 9.5m, then A* rushed toward the actor.
            # Keep the intercept exclusively for far-route planning; use the
            # same visual position as _fly_orbit for mode changes and bearing.
            approach_goal = self._target_to_orbit
            visual_target = self.targets.get(_track_id)
            tx, ty = (visual_target[:2] if visual_target is not None
                      else approach_goal)
            self._look_at = (tx, ty)
            dist = math.hypot(tx - self.world_xy[0], ty - self.world_xy[1])
            view_motion = self._view_motion_context(_now_s)
            ring_radius = (view_motion['radius'] if view_motion else ORBIT_RADIUS)
            capture_radius = ring_radius+(.5 if view_motion else 0.)
            # Keep the close controller across small range-estimate changes.
            # The old exact 8 m switch ran A* pursuit whenever a walking
            # target moved just outside the ring, then rushed inside 6 m and
            # cropped the person out of the camera in the 10:54 match.
            _retain_close = (self._confirm_close and
                             self._confirm_close_tid == _track_id and
                             self._orbit_target == _track_id and
                             dist <= ring_radius + 1.5)
            if (self._confirm_close and self._confirm_close_tid == _track_id and
                    dist > ring_radius + 1.5 and
                    _now_s - self._t_seen.get(_track_id, 0.0) <= TARGET_TTL):
                self._confirm_close = False
                self._confirm_close_tid = None
                self._orbit_prev = None
                self._orbit_target = None
                self._confirm_start = 0.0
            # R2（2026-10-09）：world 不可信时禁止"到圈即贴脸"，改为悬停等待
            # 世界坐标自愈，防止用被污染的 world 判定假到达（v26 距真身 48m 贴脸）
            if dist < capture_radius and not self._world_trusted():
                rospy.logwarn_throttle(2.0,
                    '[%s] 到圈判定 dist=%.1fm 但 world 不可信（钳制/重锚定后 %.0fs 内）'
                    '→ 悬停等自愈，不切贴脸', self.uav_id, dist,
                    WORLD_TRUST_HOLD_S)
                self._send_vel(0.0, 0.0)
                return
            if dist < capture_radius or _retain_close:
                # 到达目标附近，开始盘旋
                # === v16.1 修复：到圈分支每周期重入 ===
                # v16 实测（agent_4）：宽限期内 dist 仍 < ORBIT_RADIUS，本分支
                # 每控制周期重入 → pop 宽限记录 → _fly_orbit 每周期重打宽限日志
                # （50ms 一条，2154 次）。加守卫：仅目标切换时才重新初始化。
                # === v18 修复：v16.1 重排时丢失 target_id 赋值 → NameError ===
                # v17 实测（logs_20261007_213201）：agent_3/4 到圈贴脸即崩
                # （7420/6860 次 NameError），贴脸跟随全失效 → actor_0/3 无法消除。
                target_id = getattr(self.assignment, 'target_id', None)
                _want_target = target_id if target_id else "tracking"
                if self._orbit_target != _want_target:
                    self._orbit_center = (tx, ty)
                    # 存真实 target_id
                    self._orbit_target = _want_target
                    self._stale_search_until.pop(self._orbit_target, None)  # v16: 全新宽限窗口
                    self._orbit_entered_t = rospy.Time.now().to_sec()
                    self._confirm_start = self._last_confirm_t = 0.0
                    # 2026-10-07 五分钟冲刺: 到圈即贴脸, 不等 manager 团队确认 15s
                    if target_id:
                        self._enter_close_mode(target_id)
                # 不要清除 _target_to_orbit，_fly_orbit 需要用
                self._fly_orbit()
                return
            else:
                # 飞向目标（移动目标：A* 重规划节流）。异步投递，绝不阻塞指令流。
                # 2026-10-04：传入 tid，追逃（FLEE）时自动收紧重规划节流。
                if self._need_replan_track(approach_goal, tid=_track_id):
                    if rospy.Time.now().to_sec() - self._plan_fail_t >= 2.0:
                        self._request_plan(approach_goal)
                        self._last_track_goal = approach_goal
                        self._last_track_plan_t = rospy.Time.now().to_sec()
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
                # v23：按官方一次性逃跑闩锁模型取上限（真逃→全速咬住/牧羊；
                # 已花掉/spend→全速；未武装 13m 外→全速；已武装未花掉→22m 内压速）。
                cap = self._approach_cap(_track_id, dist)
                if spd > cap:
                    vx *= cap / spd
                    vy *= cap / spd
                vx, vy = self._apply_friend_avoidance(vx, vy)
                self._send_vel(vx, vy)
                return

        # === 搜索任务（task_type=0）===
        # _abort_orbit clears the pursuit pointer before the manager releases
        # its assignment.  A task_type=1 with no pointer used to fall through
        # here, fly toward the actor as a search waypoint and scan four
        # directions with key=None.  Hold position and keep the target bearing
        # until a refreshed pursuit or a real search assignment arrives.
        if task_type != 0:
            self._look_at = ((self.assignment.target_x, self.assignment.target_y)
                             if task_type == 1 else None)
            self._send_vel(0.0, 0.0)
            return
        # 检查是否在盘旋，以及是否能看到目标
        self._update_orbit()
        if self._orbit_target is not None:
            # 正在盘旋确认，执行盘旋飞行
            self._fly_orbit()
            return
        if self._hold_search_view_hint():
            return
        if self._segment_control(SEARCH_CRUISE_SPEED, MAX_SPEED, POS_KP,
                                 SEARCH_DECEL_M, SEARCH_ARRIVE_TOL):
            return
        # === 2026-10-05 国家一等奖：到点 dwell 期机头冻结到「下一个原本要飞到的搜索点」 ===
        # 原行为：_look_at = None → 交回 PX4 自管偏航 → 前视 90° 相机对到哪算哪。
        # 修复：DWELL_LOOKAHEAD=1 时复用 _pick_local_goal() 的 LOOKAHEAD 点，path
        #       清空时回退到 _last_local_goal（=到点前的下一个引导点）。
        if not DWELL_LOOKAHEAD:
            self._look_at = None

        # 搜索机遵守管理器的责任格。原先这里让所有距离任意视觉候选
        # 50m 内的搜索机私自改道，单个蓝色假目标曾吸走大半机队，
        # 导致真正未发现的目标活动区无人搜索。近距真实目标仍由
        # _update_orbit 接手；远距候选由 manager 按任务类型派追踪机。
        goal = flight_waypoint(self.assignment.target_x,
                               self.assignment.target_y)
        dist = math.hypot(goal[0] - self.world_xy[0], goal[1] - self.world_xy[1])

        if dist < SEARCH_ARRIVE_TOL:
            # 前视相机只覆盖一个扇区。抵达格心并不等于看过周围；原来 manager
            # 立即把半径 10m 的圆判成已搜索，蓝/白目标因此整局漏检。
            self.path = []
            # 搜索机不可仅凭共享坐标/几何 LOS 自行抢目标。真实检测由 manager
            # 立即改派 task_type=1；只有图像候选时只做有界补看。
            self._search_scan_step()
            return

        # 目标变了 → 异步投递规划（非阻塞；失败后按 _plan_fail_t 冷却 2s 再试）
        if self.path_target != goal:
            if rospy.Time.now().to_sec() - self._plan_fail_t >= 2.0:
                self._request_plan(goal)

        # 沿当前路径继续飞：规划窗口内沿用旧路径（27~74ms，位移可忽略），提交后
        # 自动切到新目标；路径为空（首次起飞/刚换格）时续发上一次速度，指令不断。
        local_goal = self._pick_local_goal()
        if local_goal is None:
            # === 国家一等奖（2026-10-05）：路径未就绪但 DWELL_LOOKAHEAD 开 ===
            # 缓存到点前的最后一个 local_goal，避免到点瞬间视野突跳。
            if DWELL_LOOKAHEAD and getattr(self, '_last_local_goal', None) is not None:
                self._look_at = self._last_local_goal
            self._stream_last_v()
            return
        # === 国家一等奖（2026-10-05）：飞行段机头持续对准下一引导点 ===
        # YOLO 使用的官方左目相机前视 90°；机头预摆可让到点瞬间就处于「即将
        # 进入下一个搜索格」的方向，与队友多点方案脱节问题修复。
        # 到点 dwell 段 path 会被 _check_start_orbit 清空 → _pick_local_goal
        # 返回 None → 上一句 self._look_at = self._last_local_goal 兜底冻结。
        if DWELL_LOOKAHEAD:
            # Stable heading by default. An optional experimental sweep is
            # still rate limited and never runs while attending to a person.
            _bearing = math.atan2(local_goal[1] - self.world_xy[1],
                                  local_goal[0] - self.world_xy[0])
            _bearing = self._search_sweep_bearing(_bearing, math.hypot(
                local_goal[0] - self.world_xy[0], local_goal[1] - self.world_xy[1]))
            self._set_search_heading(_bearing)
            self._last_local_goal = local_goal
        err_x = local_goal[0] - self.world_xy[0]
        err_y = local_goal[1] - self.world_xy[1]
        # v24 提速（2026-10-09）：搜索巡航改为恒速 —— 原 P 控制在 LOOKAHEAD=3m
        # 时理论 2.4m/s、实际受拥堵/近障降到 ~1.7m/s，5min 只覆盖地图 17%，
        # 4/6 actor 的活动区从未被搜索格覆盖 → 全程零观测。恒速 3.0m/s +
        # SEARCH_DECEL_M 减速带（近引导点切回 P 控制防冲过格中心）。
        _err = math.hypot(err_x, err_y)
        if _err > SEARCH_DECEL_M:
            vx = err_x / _err * SEARCH_CRUISE_SPEED
            vy = err_y / _err * SEARCH_CRUISE_SPEED
        else:
            vx = POS_KP * err_x
            vy = POS_KP * err_y
        spd = math.hypot(vx, vy)
        if spd > MAX_SPEED:
            vx *= MAX_SPEED / spd
            vy *= MAX_SPEED / spd
        # v23：对 22m 内所有「可被惊吓」目标（已武装/未花掉/未在逃）取保守压速；
        # 未武装 13m 外、已花掉、在逃目标均不设限（详见 _transit_cap 注释）。
        # 旧逻辑只看最近目标，会漏掉 22m 内的其他武装目标（路过惊逃）。
        _cap = self._transit_cap()
        if spd > _cap:
            vx *= _cap / spd
            vy *= _cap / spd

        vx, vy = self._search_start_cap(vx, vy)

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
            if rospy.Time.now().to_sec() - self._t_seen.get(tid, 0.0) > TARGET_TTL:
                continue
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
            (rospy.Time.now().to_sec() - t) <= FLEE_STATE_FRESH

    # ---- v23：一次性逃跑生命周期（官方闩锁模型的团队侧推断） ----
    def _target_speed(self, tid):
        """目标最近一次广播的观测速度（m/s）；未知返回 None。"""
        ent = self.targets.get(tid)
        if ent is None:
            return None
        return math.hypot(ent[2], ent[3])

    def _is_armed(self, tid):
        """无法合法观察裁判的触发闩锁，按可能已触发保守控速。"""
        return True

    def _escape_bookkeeping(self, tid):
        """基于观测速度推断「一次性逃跑」生命周期（每帧 /swarm/target_states 调用）。

        - 观测速度 ≥ ESCAPE_RUN_SPEED → 逃跑跑中（官方 escape_speed=2.0，随机走 1.0）；
        - 逃跑跑过后持续 ≤ WALK_SPEED_MAX 走 ESCAPE_SPENT_QUIET_S → 判定「已花掉」：
          官方 escape_triggered 闩锁永不复位，该目标全场永不再逃 → 解除压速护栏。
        误判自愈：若把行走噪声误判为逃跑跑，随后全速接近会真正触发（一次性）逃跑，
        进入追逃-跑完-已花掉闭环，仅损失一次 15s streak，无系统性风险。"""
        ent = self.targets.get(tid)
        if ent is None:
            return
        spd = math.hypot(ent[2], ent[3])
        now = rospy.Time.now().to_sec()
        if spd >= ESCAPE_RUN_SPEED:
            self._flee_run_t[tid] = now
            self._walk_since.pop(tid, None)
            return
        if tid not in self._flee_run_t or spd > WALK_SPEED_MAX:
            return
        t0 = self._walk_since.setdefault(tid, now)
        if now - t0 >= ESCAPE_SPENT_QUIET_S and tid not in self._escape_spent:
            self._escape_spent.add(tid)
            self._spend_mode.discard(tid)
            rospy.loginfo('[%s] %s 一次性逃跑已花掉（跑后持续行走 %.0fs）'
                          '→ 解除压速护栏，全速贴脸',
                          self.uav_id, tid, now - t0)

    def _approach_cap(self, tid, dist):
        """v23：追踪/接近指定目标的速度上限（官方一次性逃跑闩锁模型）。

        优先级：
        1. 已花掉 → 全速（闩锁已消耗完，永不再逃）；
        2. 在逃（当前跑速）或逃跑跑过（_flee_run_t 非空=闩锁已触发，含跑动中途
           障碍减速的瞬时低谷）→ 全速咬住；14m 内切牧羊 pace
          （0.95 追 2.0 必丢 YOLO 接触）；
        3. spend_mode（主动花掉）→ 全速（故意触发）；
        4. 未武装且距离>UNARMED_BRAKE_DIST → 全速（reported=False 官方不会逃；
           13m 处减速是为播报闸门 11m 处武装瞬间机速已 <1.0，不惊逃）；
        5. 其余（已武装未花掉）→ 22m 内 SPOOK_SPEED 压速保护 15s streak。"""
        if tid in self._escape_spent:
            return MAX_SPEED
        spd_t = self._target_speed(tid)
        if ((spd_t is not None and spd_t >= ESCAPE_RUN_SPEED)
                or tid in self._flee_run_t):
            return MAX_SPEED if dist > FLEE_SHEPHERD_DIST else FLEE_SHEPHERD_SPEED
        if tid in self._spend_mode:
            return MAX_SPEED if dist > FLEE_SHEPHERD_DIST else FLEE_SHEPHERD_SPEED
        # 正常行走约 1m/s；目标沿远离本机方向走时，0.85m/s 温顺接近
        # 在数学上永远到不了 6m 观测圈。只在新鲜观测证明其远离且当前
        # 距离仍超过 8m 时，主动花掉官方一次性逃跑闩锁并加速接近。
        # 待目标真正逃跑后，以上面的逃跑分支按 2.6~6m/s 接力追踪。
        ent = self.targets.get(tid)
        wx = self.world_xy
        if (ent is not None and wx is not None and dist > 8.0 and
                rospy.Time.now().to_sec() - self._t_seen.get(tid, 0.0) <= TARGET_TTL):
            radial = ((ent[0] - wx[0]) * ent[2] +
                      (ent[1] - wx[1]) * ent[3]) / max(dist, 1e-6)
            if spd_t is not None and spd_t >= 0.7 and radial >= 0.35:
                self._spend_mode.add(tid)
                rospy.loginfo('[%s] %s 目标远离 rad=%.2fm/s 且慢速无法追近，'
                              '启动一次性追逃', self.uav_id, tid, radial)
                return MAX_SPEED if dist > FLEE_SHEPHERD_DIST else FLEE_SHEPHERD_SPEED
        if not self._is_armed(tid) and dist > UNARMED_BRAKE_DIST:
            return MAX_SPEED
        return SPOOK_SPEED if dist < SPOOK_DIST else MAX_SPEED

    def _transit_cap(self):
        """v23：搜索/巡航分支的压速——对 SPOOK_DIST 内所有「可被惊吓」目标取保守上限。

        可被惊吓 = 距离≤SPOOK_DIST 且（已武装 或 距离≤UNARMED_BRAKE_DIST）
                   且 闩锁未触发（未观测到逃跑跑速、未花掉、非 spend_mode）。
        未武装目标在 13m 内同样压速：一播报就武装，若此刻机速>1.0 会立即触发逃跑。
        观测到过逃跑跑速（_flee_run_t 非空）= 闩锁已触发 = 永不可再惊吓（含跑动
        中途障碍减速的瞬时低谷，不因此重新压速追丢）。
        无可惊吓目标 → 不压速（旧逻辑只看最近目标，会漏掉 22m 内的其他武装目标）。"""
        wx = self.world_xy
        if wx is None or not self.targets:
            return MAX_SPEED
        now = rospy.Time.now().to_sec()
        for tid, (tx, ty, vx, vy) in self.targets.items():
            # 225123 east transit stayed at .85 m/s beside the green
            # coordinate after its last image (1981.928) had expired.
            # Cached/predicted positions cannot keep slowing unrelated
            # searches. Fresh people retain the existing escape protection.
            sample = self._t_seen.get(tid, 0.)
            if not (math.isfinite(sample) and sample > 0. and
                    0. <= now-sample <= TARGET_TTL):
                continue
            d = math.hypot(tx - wx[0], ty - wx[1])
            if d > SPOOK_DIST:
                continue
            if (tid in self._flee_run_t or tid in self._escape_spent
                    or tid in self._spend_mode):
                continue
            if not self._is_armed(tid) and d > UNARMED_BRAKE_DIST:
                continue
            return SPOOK_SPEED
        return MAX_SPEED

    # ---- 视觉过期时放弃盘旋 / 防扎堆 ----
    def _abort_orbit(self, reason, clear_path=True):
        """退出盘旋并回到搜索分支。

        关键：必须清空 _target_to_orbit，否则 _control 里
        `if task_type == 1 and self._target_to_orbit is not None` 仍然成立，
        飞机又会飞回那个已经不存在的目标点。

        clear_path=False（stale/重置退避场景）：保留 self.path，避免
        _need_replan_track 因“空路径”恒为真，在冷却期每帧重跑 A* 形成
        规划风暴（目标点没动，结果完全相同，纯浪费）。

        国家一等奖标准改进（2026-10-03）：同时清空 _los_lost_t 与
        本机发布的 claim 缓存 _claims[self._orbit_target]，避免
        自己广播的 claim 在放弃后 3s 内被别机判定为「被本机认领」
        而错失接力（实测放弃 → 别机仍被挡 3s 的协同层错误）。
        """
        tid = self._orbit_target
        self._orbit_target = None
        self._tracking_image = None
        self._tracking_altitude = None
        self._observation_detour = None
        self._observation_detour_next = 0.0
        self._tracking_blocked_since = None
        if tid is not None:
            getattr(self, '_stale_search_until', {}).pop(tid, None)
        self._orbit_center = None
        self._confirm_start = self._last_confirm_t = 0.0
        self._orbit_entered_t = 0.0
        self._los_lost_t = 0.0
        self._target_to_orbit = None
        # 2026-10-06 国家一等奖: 退出盘旋时清贴脸标志, 防下一目标继承.
        if self._confirm_close:
            self._confirm_close = False
            self._confirm_close_tid = None
            self._orbit_prev = None
        if clear_path:
            self.path = []
            self.path_target = None
        self._look_at = None
        # 国家一等奖：主动撤销自己之前的 claim，避免别机在 CLAIM_FRESH
        # 窗口内被错判「本机还在追」而不来接力
        if tid is not None and CLAIM_ENABLE:
            self._claims.pop(tid, None)
        rospy.logwarn('[%s] 放弃盘旋 %s：%s', self.uav_id, tid, reason)

    def _claim_cb(self, msg):
        """别机广播「我正在确认 tid」。"""
        try:
            uid, tid = str(msg.data).split(':', 1)
        except ValueError:
            return
        if uid != self.uav_id:
            self._claims[tid] = (uid, rospy.Time.now().to_sec())

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
        """当前盘旋目标是否已经没有位置更新（多半已被官方删除）。"""
        tid = self._orbit_target
        if tid is None:
            return False
        t_seen = self._t_seen.get(tid, 0.0)
        if not t_seen:
            return False
        return (rospy.Time.now().to_sec() - t_seen) > TARGET_STALE

    def _update_orbit(self):
        """Advance local visual streak from eligible own-camera timestamps only."""
        if self._orbit_target is None:
            return
        now = rospy.Time.now().to_sec()
        evidence = self._visual_observations.get(self._orbit_target)
        accepted = getattr(self, '_eligible_observations', {}).get(self._orbit_target)
        if accepted is not None and (evidence is None or not evidence.get('eligible') or
                                    accepted['sample_s'] > evidence['sample_s']):
            evidence = accepted
        if (evidence is None or not evidence['eligible'] or
                not 0.0 <= now-evidence['sample_s'] <= 1.0):
            self._confirm_start = self._last_confirm_t = 0.0
            return
        sample = evidence['sample_s']
        if sample <= self._last_confirm_t:
            return
        if self._confirm_start == 0.0 or sample-self._last_confirm_t > 1.0:
            self._confirm_start = sample
        self._last_confirm_t = sample

    def _enter_close_mode(self, tid):
        """Enter the stable near-range flight mode; this proves no recognition.

        Keep the existing 0.95m/s transition cap. Camera samples establish
        visual continuity, and official feedback alone establishes elimination.
        """
        if self._confirm_close and self._confirm_close_tid == tid:
            return
        self._confirm_close = True
        self._confirm_close_tid = tid
        self._confirm_close_t0 = rospy.Time.now().to_sec()
        self._confirm_close_until = self._confirm_close_t0 + CONFIRM_LOCK_DURATION
        self._close_range_ref = None
        self._orbit_prev = None
        rospy.loginfo(
            "[%s] 到达观察圈 → 定点稳视角 (tid=%s) R=%.1fm "
            "(速度cap=%.2fm/s), 锁定 %.0fs",
            self.uav_id, tid, ORBIT_RADIUS_CLOSE, CLOSE_TRANSIT_SPEED,
            CONFIRM_LOCK_DURATION,
        )

    def _check_start_orbit(self):
        """仅为本机明确获派的目标进入观察，不能从搜索中自行抢人。"""
        assignment = getattr(self, 'assignment', None)
        assigned_tid = (getattr(assignment, 'target_id', None)
                        if getattr(assignment, 'task_type', None) == 1 else None)
        if not assigned_tid:
            return
        # R2（2026-10-09）：world 不可信（EKF 钳制/重锚定后）不开始新盘旋——
        # 检测距离判据基于被污染的 world_xy 会假到达/追错点
        if not self._world_trusted():
            return
        wx = self.world_xy
        if wx is None:
            return

        for tid, (tx, ty, _, _) in self.targets.items():
            if tid != assigned_tid:
                continue
            if rospy.Time.now().to_sec() - self._t_seen.get(tid, 0.0) > TARGET_TTL:
                continue
            dist = math.hypot(tx - wx[0], ty - wx[1])
            # 在检测范围内且 LOS 可见
            if dist >= DETECT_RADIUS or not self.los.visible(wx[0], wx[1], tx, ty):
                continue
            # 放弃冷却期内不再自动追（刚放弃过的目标，别立刻又贴回去）
            if rospy.Time.now().to_sec() < self._giveup_until.get(tid, 0.0):
                continue
            # manager 可以显式给同目标指定备机；旧 claim 不得推翻任务。
            # 开始盘旋确认
            self._orbit_target = tid
            self._orbit_center = (tx, ty)
            self._stale_search_until.pop(tid, None)  # v16: 全新宽限窗口
            self._orbit_entered_t = rospy.Time.now().to_sec()
            self._confirm_start = self._last_confirm_t = 0.0
            # 检测半径是 10m，稳定观察圈是 8m。10m 处就限 0.95m/s
            # 会使尚在逃跑的 actor 以更快速度甩开本机。
            if dist <= ORBIT_RADIUS:
                self._enter_close_mode(tid)
            rospy.loginfo("[%s] 开始盘旋确认目标 %s", self.uav_id, tid)
            break

    def _close_follow_cap(self, tid, observation_age):
        """Respect pursuit decisions at ring entry without accelerating blind.

        These are camera-derived control states, not knowledge of an official
        escape latch. Quiet observation retains its original speed limit.
        """
        pursuing = (tid in getattr(self, '_escape_spent', set()) or
                    tid in getattr(self, '_spend_mode', set()) or
                    tid in getattr(self, '_flee_run_t', {}))
        if pursuing and 0.0 <= observation_age <= 1.0:
            return min(FLEE_SHEPHERD_SPEED, MAX_SPEED)
        return CLOSE_TRANSIT_SPEED

    def _reobserve_cb(self, msg):
        """A task-scoped request to change view; never grants detection credit."""
        try:
            data = json.loads(msg.data)
            now = rospy.Time.now().to_sec()
            assignment = getattr(self, 'assignment', None)
            sample = float(data['sample_s'])
            if (assignment is None or assignment.task_type != 1 or
                    data.get('uav_id') != self.uav_id or
                    data.get('target_id') != assignment.target_id or
                    not math.isfinite(sample) or not 0.0 <= now-sample <= 1.0 or
                    sample <= getattr(self, '_reobserve_request_sample', -1e18)):
                return
            self._reobserve_request_sample = sample
            self._reobserve_forced = (assignment.target_id, now+6.0)
        except (ValueError, KeyError, TypeError, AttributeError):
            return

    def _reobserve_goal(self, tx, ty, now):
        """Try one bounded safe view after loss or sustained poor report quality.

        Camera evidence owns this watchdog. Team coordinates only locate the
        observation ring; they do not establish that this camera sees a person.
        """
        tid = self._orbit_target
        request = getattr(self, '_reobserve_forced', None)
        forced = request is not None and request[0] == tid and now < request[1]
        quality=getattr(self,'_report_view_window',None)
        degraded=quality is not None and quality.degraded(tid,now)
        active = getattr(self, '_reobserve_view', None)
        recovering = (active is not None and active['tid'] == tid and
                      now < active['until'])
        # A narrow walking person alternates good and rejected geometry. One
        # good frame used to cancel a safe side move before it improved the
        # view, while the ten-second retry cooldown remained in force.
        hold_recovery = recovering and not (quality is not None and quality.recovered(tid, now))
        image = self._fresh_tracking_view(now)
        if (not forced and not degraded and not hold_recovery and image is not None and image.get('geometry_ok') and
                image.get('report_ok', image.get('geometry_ok')) and
                image.get('person_verdict_ok', True) and
                not view_needs_space(image)):
            # A pixel-only view cannot veto a request to recover geometry.
            self._reobserve_view = None
            return None
        last = getattr(self, '_last_eligible_image', {}).get(tid, -1e18)
        evidence = getattr(self, '_visual_observations', {}).get(tid)
        if evidence is not None and evidence.get('eligible'):
            last = max(last, evidence['sample_s'])
        forced = getattr(self, '_reobserve_forced', None)
        forced = forced is not None and forced[0] == tid and now < forced[1]
        if 0.0 <= now - last <= 3.0 and not forced and not degraded and not hold_recovery:
            self._reobserve_view = None
            return None
        entered = getattr(self, '_orbit_entered_t', now)
        if now - entered < 3.0 or not 0.0 <= now - self._t_seen.get(tid, 0.0) <= TARGET_TTL:
            return None
        wx, wy = self.world_xy
        if not 6.5 <= math.hypot(wx - tx, wy - ty) <= 10.0:
            return None  # first regain range; do not sweep through the close blind zone
        view = getattr(self, '_reobserve_view', None)
        if view is not None and view['tid'] == tid:
            if now < view['until'] and math.hypot(tx-view['tx'], ty-view['ty']) <= 2.0:
                goal = view['goal']
                if self._safe_reobserve_segment((wx, wy), goal):
                    return goal
            self._reobserve_view = None
        if now < getattr(self, '_reobserve_next', 0.0):
            return None
        self._reobserve_next = now + 10.0
        angle = math.atan2(wy - ty, wx - tx)
        sign = getattr(self, '_reobserve_sign', 1)
        self._reobserve_sign = -sign
        for offset in (sign * math.pi/6.0, -sign * math.pi/6.0):
            goal = (tx + ORBIT_RADIUS_CLOSE * math.cos(angle + offset),
                    ty + ORBIT_RADIUS_CLOSE * math.sin(angle + offset))
            if (not self._safe_reobserve_segment((wx, wy), goal) or
                    not self.los.visible(goal[0], goal[1], tx, ty)):
                continue
            self._reobserve_view = dict(tid=tid, goal=goal, tx=tx, ty=ty, until=now+6.0)
            rospy.logwarn('[VIEW_REOBSERVE] uav=%s tid=%s goal=(%.2f,%.2f) '
                          'last_good_image=%.3f sparse_qualified_view=%s; 保持朝向、限时侧移补看',
                          self.uav_id, tid, goal[0], goal[1], last,degraded)
            return goal
        return None

    def _safe_reobserve_segment(self, start, goal):
        """Check the whole short move against inflated lidar grid and bounds."""
        xmin, xmax, ymin, ymax = flight_bounds()
        n = max(1, int(math.ceil(math.hypot(goal[0]-start[0], goal[1]-start[1]) / 0.2)))
        for i in range(n+1):
            x = start[0] + (goal[0]-start[0]) * i/n
            y = start[1] + (goal[1]-start[1]) * i/n
            if not (xmin+MAP_GUARD_MARGIN <= x <= xmax-MAP_GUARD_MARGIN and
                    ymin+MAP_GUARD_MARGIN <= y <= ymax-MAP_GUARD_MARGIN):
                return False
            cell = self.grid.world_to_cell((x, y))
            if cell is None or not self.grid.is_free(cell):
                return False
        return True

    def _obstructed_view_goal(self, tx, ty, tvx, tvy, now, desired_velocity=None):
        """Choose a short safe side move BEFORE obstacle stopping loses the view.

        Candidate routes use only the team's lidar map. Every output still
        passes the ordinary final radar/altitude/boundary safety constraints.
        """
        if (self.world_xy is None or
                not 0.0 <= now-getattr(self, '_scan_t', -1e18) <= RADAR_FRESH_S):
            return None
        wx, wy = self.world_xy
        mvx, mvy = desired_velocity if desired_velocity is not None else (tvx, tvy)
        motion_speed = math.hypot(mvx, mvy)
        # A wall elsewhere in the scan must not make a well-held view orbit.
        # Look ahead only along the intended motion, including backward flight.
        near_obstacle = False
        if motion_speed > .2:
            probe = (wx+3.5*mvx/motion_speed, wy+3.5*mvy/motion_speed)
            near_obstacle = not self._safe_reobserve_segment((wx, wy), probe)
        blocked_since = getattr(self, '_tracking_blocked_since', None)
        blocked = blocked_since is not None and now-blocked_since >= .4
        distance = math.hypot(wx-tx, wy-ty)
        if not 2.0 <= distance <= 12.0:
            return None
        cached = getattr(self, '_observation_detour', None)
        if cached is not None and cached['tid'] == self._orbit_target and now < cached['until']:
            goal = (tx+ORBIT_RADIUS_CLOSE*math.cos(cached['angle']),
                    ty+ORBIT_RADIUS_CLOSE*math.sin(cached['angle']))
            if (math.hypot(goal[0]-wx, goal[1]-wy) <= 6.0 and
                    self._safe_reobserve_segment((wx, wy), goal) and
                    self.los.visible(goal[0], goal[1], tx, ty)):
                return goal
            self._observation_detour = None
        if not (near_obstacle or blocked):
            return None
        if now < getattr(self, '_observation_detour_next', 0.):
            return None
        self._observation_detour_next = now+.5
        angle = math.atan2(wy-ty, wx-tx)
        speed = math.hypot(tvx, tvy)
        candidates = []
        for degrees in (-20, 20, -40, 40, -60, 60, -90, 90):
            a = angle+math.radians(degrees)
            goal = (tx+ORBIT_RADIUS_CLOSE*math.cos(a), ty+ORBIT_RADIUS_CLOSE*math.sin(a))
            travel = math.hypot(goal[0]-wx, goal[1]-wy)
            if travel > 6.0 or not self._safe_reobserve_segment((wx, wy), goal):
                continue
            if not self.los.visible(goal[0], goal[1], tx, ty):
                continue
            # Prefer side/rear views when equally reachable; never force a
            # circuit through buildings just to get behind a walking actor.
            ahead = max(0., (math.cos(a)*tvx+math.sin(a)*tvy)/max(speed, .1))
            candidates.append((travel+2.*ahead, a, goal))
        if not candidates:
            rospy.logwarn_throttle(2., '[VIEW_BLOCKED] uav=%s tid=%s range=%.2f; '
                                  'no safe short observation move',
                                  self.uav_id, self._orbit_target, distance)
            return None
        _, angle, goal = min(candidates)
        self._observation_detour = dict(tid=self._orbit_target, angle=angle, until=now+3.)
        rospy.logwarn_throttle(2., '[VIEW_DETOUR] uav=%s tid=%s goal=(%.2f,%.2f) '
                              'obstacle=%s stopped=%s image_margin=%s',
                              self.uav_id, self._orbit_target, *goal,
                              near_obstacle, blocked, False)
        return goal

    def _fly_orbit(self):
        """Keep a safe, inward observation point and face the assigned actor."""
        if self._orbit_target is None:
            self._send_vel(0.0, 0.0)
            return
        assignment = getattr(self, 'assignment', None)
        expected_tid = getattr(assignment, 'target_id', None) or 'tracking'
        if (assignment is None or getattr(assignment, 'task_type', None) != 1 or
                expected_tid != self._orbit_target):
            self._abort_orbit('管理器追踪指派已结束')
            self._send_vel(0.0, 0.0)
            return

        _w = self.world_xy
        if _w is None:
            return
        wx, wy = _w

        # R2（2026-10-09）：贴脸期间 world 突然不可信（EKF 钳制/重锚定）→ 悬停
        # 等坐标自愈，不朝错误位置继续贴脸（v26 贴脸飞偏 48m 的直接诱因）
        if not self._world_trusted():
            self._send_vel(0.0, 0.0)
            return

        if self._hold_pixel_recovery(rospy.Time.now().to_sec()):
            return

        # 短时遮挡时允许有界重获。截止点从最后真实样本计算，而非从
        # "终于飞到冻结位置"的时刻计算；否则远途路程可再白耗一轮宽限。
        if self._orbit_stale():
            tid0 = self._orbit_target
            _now_s = rospy.Time.now().to_sec()
            gap = _now_s - self._t_seen.get(tid0, 0.0)
            _deadline = (self._t_seen.get(tid0, 0.0) + TARGET_STALE +
                         STALE_SEARCH_GRACE)
            if _now_s >= _deadline:
                self.targets.pop(tid0, None)
                self._giveup_until[tid0] = _now_s + GIVEUP_COOLDOWN
                self._stale_search_until.pop(tid0, None)
                self._abort_orbit('已 %.1fs 无位置更新，暂退避等待接力' % gap,
                                  clear_path=False)
                return
            if tid0 not in self._stale_search_until:
                self._stale_search_until[tid0] = _deadline
                rospy.logwarn('[%s] 目标 %s 已 %.1fs 无更新 → 剩余 %.1fs '
                              '有界补看；不续算冻结坐标',
                              self.uav_id, tid0, gap, _deadline - _now_s)

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
        orbit_dist = math.hypot(tx - wx, ty - wy)
        obs_age = now - self._t_seen.get(self._orbit_target, 0.0)
        view_motion = self._view_motion_context(now)
        ring_radius = view_motion['radius'] if view_motion else ORBIT_RADIUS_CLOSE
        if self._confirm_close:
            ref = getattr(self, '_close_range_ref', None)
            if ref is None or ref[0] != self._orbit_target or now - ref[1] > 2.0:
                self._close_range_ref = (self._orbit_target, now, orbit_dist,
                                         tx, ty, wx, wy)
            elif now - ref[1] >= CLOSE_RECEDING_SAMPLE_S:
                old_ux = (ref[3] - ref[5]) / max(ref[2], 1e-6)
                old_uy = (ref[4] - ref[6]) / max(ref[2], 1e-6)
                away_rate = ((tx - ref[3]) * old_ux +
                             (ty - ref[4]) * old_uy) / (now - ref[1])
                # Optional experiment: keep an already localized full-body
                # view before it opens beyond the observation ring. Never
                # accelerate toward a cropped/close person using team-only
                # coordinates. The preference still passes all final guards.
                _run_seen = self._flee_run_t.get(self._orbit_target)
                image = self._fresh_tracking_view(now)
                early = (EARLY_VIEW_PURSUIT and view_motion is not None and
                         image.get('report_ok') and not view_needs_space(image) and
                         image['sample_s'] >= ref[1] + .5 and
                         orbit_dist >= max(7.5, ring_radius - .5))
                if (((_run_seen is not None and now - _run_seen <= 3.0) or early) and
                        self._orbit_target not in self._escape_spent and
                        close_target_is_receding(
                        orbit_dist, ref[2], now - ref[1], obs_age,
                        away_rate)):
                    self._close_chase_tid = self._orbit_target
                    self._close_chase_until = now + CLOSE_CHASE_HOLD_S
                    self._spend_mode.add(self._orbit_target)
                    rospy.logwarn_throttle(2.0,
                        '[%s] %s 近距目标远离 %.2fm/s（距离 %.1fm）→ 短时追赶 early_view=%s',
                        self.uav_id, self._orbit_target,
                        (orbit_dist - ref[2]) / (now - ref[1]), orbit_dist, early)
                self._close_range_ref = (self._orbit_target, now, orbit_dist,
                                         tx, ty, wx, wy)
        # The close cap is below a walking actor's nominal 1m/s speed.  If
        # that actor walks away, a 30s close-mode latch makes reacquisition
        # mathematically impossible.  Let the normal one-shot escape policy
        # decide whether faster pursuit is warranted once contact opens up.
        if (self._confirm_close and orbit_dist > ring_radius + 1.5 and
                now - self._t_seen.get(self._orbit_target, 0.0) <= TARGET_TTL):
            self._confirm_close = False
            self._confirm_close_tid = None
            self._orbit_prev = None
            rospy.loginfo('[%s] %s 距离 %.1fm 已离开近距圈，切受控重捕获',
                          self.uav_id, self._orbit_target, orbit_dist)
        # 从 10m 检测圈开始追的飞机，接近到 8m 后才能切低速。
        # 团队广播可能先于到圈抵达，所以在实际位置上重复检查一次。
        if (not self._confirm_close and self._orbit_target in self.targets and
                now - self._t_seen.get(self._orbit_target, 0.0) <= TARGET_TTL and
                orbit_dist <= ring_radius+(.5 if view_motion else 0.) and
                self.los.visible(wx, wy, tx, ty)):
            self._enter_close_mode(self._orbit_target)

        # Bridge 的速度按真实图像时间估计。旧代码每 50ms 对重复发布的
        # 相同坐标再作差分，目标下一帧一跳就产生虚假的 2.5m/s 脉冲。
        # 仅用新鲜速度作一个控制周期的小预测；过期则停在有界补看点。
        tvx, tvy = self.targets.get(self._orbit_target, (0, 0, 0, 0))[2:4]
        if (not 0.0 <= obs_age <= TARGET_TTL or
                not math.isfinite(tvx) or not math.isfinite(tvy)):
            tvx = tvy = 0.0
        else:
            tv_speed = math.hypot(tvx, tvy)
            if tv_speed > 2.5:
                tvx *= 2.5 / tv_speed
                tvy *= 2.5 / tv_speed
        dt = 1.0 / CTRL_RATE
        tx_c = tx + tvx * dt
        ty_c = ty + tvy * dt
        angle = math.atan2(wy - ty_c, wx - tx_c)

        xmin, xmax, ymin, ymax = flight_bounds()
        _ring_bounds = (xmin, xmax, ymin, ymax)

        # 维持 8m 观察距离；静止人前无需持续绕圈，边界一律选场内侧
        # 的观察点。移动人的平移由后面的新鲜速度前馈承担。
        inward_view = False
        if self._confirm_close:
            _radius = ring_radius
            # 这里的 angle 只决定观察方向；移动目标的平移由速度前馈负责。
            angle = math.atan2(wy - ty_c, wx - tx_c)
            target_x = tx_c + _radius * math.cos(angle)
            target_y = ty_c + _radius * math.sin(angle)
            if not (xmin + MAP_GUARD_MARGIN <= target_x <= xmax - MAP_GUARD_MARGIN and
                    ymin + MAP_GUARD_MARGIN <= target_y <= ymax - MAP_GUARD_MARGIN):
                inward_view = True
                target_x, target_y = observation_ring_goal(
                    wx, wy, tx_c, ty_c, _radius,
                    _ring_bounds, MAP_GUARD_MARGIN + 0.5)
            # 平滑近距目标点，防止切换时位置指令突跳。
            target_x = wx + 0.3 * (target_x - wx)
            target_y = wy + 0.3 * (target_y - wy)
        else:
            _radius = ring_radius if view_motion else ORBIT_RADIUS
            target_x = tx_c + _radius * math.cos(angle)
            target_y = ty_c + _radius * math.sin(angle)
            if not (xmin + MAP_GUARD_MARGIN <= target_x <= xmax - MAP_GUARD_MARGIN and
                    ymin + MAP_GUARD_MARGIN <= target_y <= ymax - MAP_GUARD_MARGIN):
                target_x, target_y = observation_ring_goal(
                    wx, wy, tx_c, ty_c, _radius,
                    _ring_bounds, MAP_GUARD_MARGIN + 0.5)

        detour = self._obstructed_view_goal(
            tx, ty, tvx, tvy, now,
            desired_velocity=(POS_KP*(target_x-wx)+tvx, POS_KP*(target_y-wy)+tvy))
        if detour is not None:
            target_x, target_y = detour
            inward_view = True  # radial correction must not cancel the safe side move
        else:
            reobserve = self._reobserve_goal(tx, ty, now)
            if reobserve is not None:
                target_x, target_y = reobserve
                inward_view = True

        # P 控制飞向盘旋点
        err_x = target_x - wx
        err_y = target_y - wy
        vx = POS_KP * err_x
        vy = POS_KP * err_y
        # Carry the existing pursuit decision through every close-mode cap.
        # Previously _approach_cap allowed pursuit, but these three stages
        # clamped back to .95 m/s as soon as the UAV entered the ring.
        follow_cap = self._close_follow_cap(self._orbit_target, obs_age)
        _urgent_backoff = False
        if self._confirm_close:
            _tvx, _tvy = tvx, tvy
            vx, vy = close_follow_velocity(
                err_x, err_y, _tvx, _tvy,
                now - self._t_seen.get(self._orbit_target, 0.0),
                speed_cap=follow_cap)
            if not inward_view:
                # At a map edge the radial "back off" direction can point
                # outside the arena and fight the inward observation goal.
                vx, vy = ring_standoff_velocity(
                    vx, vy, wx, wy, tx, ty, _radius,
                    _tvx, _tvy, speed_cap=follow_cap)
                vx, vy, _urgent_backoff = urgent_close_backoff_velocity(
                    vx, vy, wx, wy, tx, ty, _tvx, _tvy)
        else:
            # 8m 外圈也维持固定观察方位，不为产生角速度而绕人。
            # 对移动目标用最近真实图像轨迹作前馈；随后仍统一限速、
            # 避友机并经过 _send_vel 的雷达/地图/加速度安全链。
            vx += tvx
            vy += tvy
        spd = math.hypot(vx, vy)
        _v_cap = (CLOSE_BACKOFF_SPEED if _urgent_backoff else
                  follow_cap if self._confirm_close else
                  self._approach_cap(self._orbit_target, orbit_dist))
        if spd > _v_cap:
            vx *= _v_cap / spd
            vy *= _v_cap / spd

        # 友机避碰
        vx, vy = self._apply_friend_avoidance(vx, vy)

        self._send_vel(vx, vy)

        self._update_orbit()
        confirm_duration = (self._last_confirm_t-self._confirm_start
                            if self._confirm_start > 0.0 else 0.0)
        # Reacquisition watchdog has a separate residence clock. A zero visual
        # timer must neither imply huge confirmation time nor lock the UAV forever.
        if getattr(self, '_orbit_entered_t', 0.0) <= 0.0:
            self._orbit_entered_t = now
        evidence = self._visual_observations.get(self._orbit_target)
        own_seen = evidence is not None and 0.0 <= now-evidence['sample_s'] <= 1.0
        image = getattr(self, '_tracking_image', None)
        if image is not None and image['target_id'] == self._orbit_target:
            own_seen = own_seen or 0. <= now-image['sample_s'] <= .5
        # The loss budget starts at the last real OWN image, not at ring
        # entry. Team updates, predictions and replayed samples cannot renew
        # it. A 1.01 s camera gap after 20 s of healthy tracking is recoverable.
        last_own_image = self._orbit_entered_t
        if evidence is not None and evidence['sample_s'] <= now:
            last_own_image = max(last_own_image, evidence['sample_s'])
        if (image is not None and image['target_id'] == self._orbit_target and
                image['sample_s'] <= now):
            last_own_image = max(last_own_image, image['sample_s'])
        lost_for = now-last_own_image
        if lost_for >= ORBIT_GIVEUP and not own_seen:
            self._giveup_until[self._orbit_target] = now+GIVEUP_COOLDOWN
            self._abort_orbit('重捕获超时，无本机真实图像，释放任务', clear_path=False)
            return
        # A residence timer must not interrupt a still-visible assigned actor.
        # Lost-image recovery and task cancellation retain their own deadlines.
        if self._confirm_close and now >= self._confirm_close_until:
            if own_seen:
                self._confirm_close_until = now + CONFIRM_LOCK_DURATION
            else:
                self._confirm_close = False
                self._confirm_close_tid = None
                self._orbit_prev = None
                rospy.loginfo("[%s] 观察锁定到期且无本机新图像，转限时重获",
                              self.uav_id)
        self._publish_claim()

        if confirm_duration >= CONFIRM_TIME:
            rospy.loginfo_throttle(
                5.0, "[%s] 目标 %s 本机真实图像连续观测 %.1fs，继续跟踪上报；官方消除未知",
                self.uav_id, self._orbit_target, confirm_duration)
    def _adaptive_speed(self, vx, vy):
        """根据场景自适应速度：搜索快、确认慢、避障稳"""
        # 基础速度
        base_speed = MAX_SPEED

        # 1. 追踪目标时降速（确认需要稳定，但需要追住 2m/s 逃跑的 actor）
        # 2026-10-03 改进 3：1.5→3.5。
        # 旧值 1.5 完全追不上 actor 的 2m/s 逃逸 → 飞机被甩在后面、
        # 视距越来越远 → 单点上报频率不足 → 确认期反复 reset 进度卡 0%。
        # 3.5 是 actor 2m/s × 1.75 倍 + DWA 安全留量，足以咬住又不会破坏 SP。
        if self._orbit_target is not None:
            base_speed = 3.5
        # 2. ORCA 激活时（友机近）降速
        elif self._friend_positions:
            # 检查是否有近距友机
            _w = self.world_xy
            wx, wy = _w if _w is not None else (0.0, 0.0)
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
        # Both routed approach and orbit pass here. Form the visual preference
        # before ORCA/radar/map safety may replace or stop it.
        vx, vy = self._view_follow_velocity(vx, vy)
        _w = self.world_xy
        if _w is None:
            self._last_orca_v = (vx, vy)
            return vx, vy
        wx, wy = _w
        _vx_in, _vy_in = vx, vy   # 记录 ORCA 前的期望速度
        now = rospy.Time.now().to_sec()
        for fid in list(self._friend_positions):
            if now - self._friend_seen.get(fid, -1e18) > 1.0:
                self._friend_positions.pop(fid, None)
                self._friend_seen.pop(fid, None)

        # 先应用自适应速度
        vx, vy = self._adaptive_speed(vx, vy)

        # 起飞/爬升阶段豁免水平避让：起飞区 6 架间距只有 8m，全部落在 ORCA
        # 互斥范围内会直接死锁（实测 ORCA 无解 110 次、6 架全程趴地起不来）。
        if self.local_z is not None and self.local_z < ALT_TAKEOFF \
                and math.hypot(vx, vy) < 0.05:
            self._last_orca_v = (vx, vy)
            return vx, vy

        # ====== BUG 1: 无解时没兜底 - 改为强制使用排斥 ======
        # 先检查是否有近距友机需要避让
        has_conflict = False
        for fid, (fx, fy, fz) in self._friend_positions.items():
            dist = math.hypot(fx - wx, fy - wy)
            friend_alt = fz if fz is not None else self.altitude_layer
            my_alt = self.local_z if self.local_z is not None else self.altitude_layer
            if math.hypot(dist, my_alt - friend_alt) < SAFE_3D:
                if dist < .01:
                    self._last_orca_v = (0.0, 0.0)
                    return self._last_orca_v
                has_conflict = True
                break

        if not has_conflict:
            self._last_orca_v = (vx, vy)
            return vx, vy  # 无冲突直接返回

        # 保留期望速度作为避碰优化目标。不能为候选评分强制抬高速度：
        # 悬停时也可能需要友机避让，但不得无故变成全速机动。
        desired_vx, desired_vy = vx, vy
        v_current = math.hypot(vx, vy)
        MIN_SPEED = MAX_SPEED * 0.3

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

            if dist < FRIEND_SAFE_DIST * 3:
                px, py = fx - wx, fy - wy
                if dist > 0.01:
                    # 半平面必须沿真实径向远离友机。旧版给 n 的 x/y 各
                    # 加固定偏置后，某些侧向候选虽满足半平面却仍朝友机前进。
                    nx, ny = -px / dist, -py / dist

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

            def consider(cand_vx, cand_vy):
                nonlocal best_vx, best_vy, best_score
                speed = math.hypot(cand_vx, cand_vy)
                if speed > MAX_SPEED + 1e-6:
                    return
                for nx, ny, min_vel in orca_halfplanes:
                    if cand_vx * nx + cand_vy * ny < min_vel - 1e-6:
                        return
                # 最接近上层轨迹的安全速度优先；同等偏差时低速更安全。
                score = -((cand_vx - desired_vx) ** 2
                          + (cand_vy - desired_vy) ** 2) - 1e-4 * speed ** 2
                if score > best_score:
                    best_score = score
                    best_vx, best_vy = cand_vx, cand_vy

            consider(desired_vx, desired_vy)
            # 单个约束时这是最近的可行速度；多个约束时仍由 consider
            # 校验全部半平面，再与全向采样结果比较。
            for nx, ny, min_vel in orca_halfplanes:
                deficit = min_vel - (desired_vx * nx + desired_vy * ny)
                if deficit > 0.0:
                    consider(desired_vx + deficit * nx,
                             desired_vy + deficit * ny)

            # 2026-09-27：候选改为全向。原来只在 [v_angle±1.5rad] 采样，
            # 相向接近时必须大转向才能满足半平面 → 无解 → 退化到强制排斥，
            # 实测贴到 0.70m。全向采样保证只要几何上存在安全方向就找得到。
            _nang = max(8, AVOID_ANGLES)
            _angles = [v_angle]
            for _k in range(_nang):
                _angles.append(v_angle + 2.0 * math.pi * _k / _nang)
            speeds = [0.0, min(v_current, MAX_SPEED),
                      min(0.5, MAX_SPEED), MIN_SPEED,
                      MAX_SPEED * 0.5, MAX_SPEED * 0.7, MAX_SPEED]
            for angle in _angles:
                for speed in speeds:
                    consider(speed * math.cos(angle), speed * math.sin(angle))

            if best_score == -float('inf'):
                # 对置友机可能让半平面相互矛盾。旧版从原追踪速度叠加
                # 排斥力，两侧力抵消时会继续撞向一架友机。无解则停车，
                # 等下一帧友机位置变化后重新选择方向。
                rospy.logwarn("[%s] ORCA 多机约束无解，水平停车", self.uav_id)
                best_vx, best_vy = 0.0, 0.0
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
    def _recover_stalled_takeoff(self):
        """One bounded standard-mode recovery of the OFFBOARD takeoff deadlock.

        PX4 can leave landed=false while its takeoff state remains READY and
        thus ignores OFFBOARD climb setpoints. AUTO.TAKEOFF uses the ordinary
        navigator takeoff request. Never force arm/disarm, change land-detector
        thresholds or use this recovery after flight/positioning failure.
        """
        now = rospy.Time.now().to_sec()
        recovery = getattr(self, '_takeoff_recovery', None)
        if recovery is None:
            baseline = getattr(self, '_climb_progress', None)
            if (baseline is None or now-baseline[0] < 10. or
                    getattr(self, '_takeoff_recovery_attempted', False)):
                return False
            z, xy = self.local_z, self.local_xy
            speed = self._est_recent_speed()
            roll = getattr(self, 'roll', float('inf'))
            pitch = getattr(self, 'pitch', float('inf'))
            if (getattr(self, '_takeoff_done', False) or
                    getattr(self, '_landing', False) or
                    getattr(self, '_localization_fault', False) or
                    getattr(self, '_ekf_frozen', False) or
                    not getattr(self, '_anchor_done', False) or
                    not self.state.connected or not self.state.armed or
                    self.state.mode != 'OFFBOARD' or
                    z is None or not math.isfinite(z) or not -.3 <= z <= .3 or
                    getattr(self, '_takeoff_max_z', z) > .3 or
                    abs(z-baseline[1]) > .1 or xy is None or
                    not all(math.isfinite(v) for v in xy) or
                    math.hypot(*xy) > .5 or speed is None or speed > .15 or
                    not math.isfinite(speed) or
                    not 0. <= now-getattr(self, '_local_prev_t', -1e18) <= .3 or
                    not math.isfinite(roll) or not math.isfinite(pitch) or
                    abs(roll) > math.radians(10.) or abs(pitch) > math.radians(10.)):
                return False
            self._takeoff_recovery_attempted = True
            # AUTO.TAKEOFF must not use an unchecked default height. Read back
            # PX4's normal parameter without modifying it or its safety gates.
            try:
                from mavros_msgs.srv import ParamGet
                response = rospy.ServiceProxy('/%s/mavros/param/get' % self.uav_id,
                                               ParamGet)('MIS_TAKEOFF_ALT')
                height = response.value.real
                if (not response.success or not math.isfinite(height) or
                        height < 2.0 or height+max(0., z) >
                        min(3.5, ALT_HARD_CEIL-ALT_HARD_MARGIN)):
                    raise ValueError('unsafe or unavailable MIS_TAKEOFF_ALT=%s' % height)
            except Exception as exc:
                rospy.logerr('[TAKEOFF_RECOVERY_BLOCKED] uav=%s %s', self.uav_id, exc)
                return False
            recovery = self._takeoff_recovery = {
                'started': now, 'last_request': -1e18, 'auto_seen': False,
                'phase': 'auto', 'failed': False,
                'stream_since': now, 'stream_last': now}
            rospy.logwarn('[TAKEOFF_RECOVERY] uav=%s grounded stall; bounded AUTO.TAKEOFF '
                          'height=%.2fm, then OFFBOARD', self.uav_id, height)

        if recovery['failed']:
            if self.state.mode == 'OFFBOARD':
                self._send_vel(0., 0., vz=0.)
            return True
        # ParamGet above yields to ROS callbacks. A new pose can arrive after
        # the original `now`: comparing it to that old time falsely latches
        # failure. Snapshot the sample first, then read the current clock;
        # do not extend the original recovery deadline or allow future poses.
        pose_sample = getattr(self, '_local_prev_t', -1e18)
        now = rospy.Time.now().to_sec()
        mode = self.state.mode
        reasons = []
        if (getattr(self, '_localization_fault', False) or
                getattr(self, '_ekf_frozen', False)):
            reasons.append('localization_fault')
        if getattr(self, '_landing', False):
            reasons.append('landing')
        if not self.state.connected or not self.state.armed:
            reasons.append('disconnected_or_disarmed')
        if not 0. <= now-recovery['started'] <= 15.:
            reasons.append('clock_reset_or_timeout')
        if mode not in ('OFFBOARD', 'AUTO.TAKEOFF', 'AUTO.LOITER'):
            reasons.append('flight_mode')
        if (self.local_z is None or not math.isfinite(self.local_z) or
                self.local_z > ALT_HARD_CEIL-ALT_HARD_MARGIN):
            reasons.append('altitude')
        if not 0. <= now-pose_sample <= .3:
            reasons.append('pose_time')
        if reasons:
            recovery['failed'] = True
            rospy.logerr('[TAKEOFF_RECOVERY_FAILED] uav=%s reason=%s mode=%s z=%s '
                         'pose_age=%.4f elapsed=%.3f; no takeover of landing/failsafe',
                         self.uav_id, ','.join(reasons), mode, self.local_z,
                         now-pose_sample, now-recovery['started'])
            # The existing localization-failure path retains authority.
            if getattr(self, '_localization_fault', False):
                self._send_vel(0., 0.)
            elif mode in ('AUTO.TAKEOFF', 'AUTO.LOITER'):
                self.mode_srv(0, 'AUTO.LAND')
            elif mode == 'OFFBOARD':
                self._send_vel(0., 0., vz=0.)
            return True
        # Keep the normal zero-motion OFFBOARD prestream during automatic
        # ascent. PX4 ignores it until handback; no horizontal mission starts.
        self._send_vel(0., 0., vz=0.)
        if now-recovery['stream_last'] > OFFBOARD_PRESTREAM_MAX_GAP_S:
            recovery['stream_since'] = now
        recovery['stream_last'] = now
        if mode == 'AUTO.TAKEOFF':
            recovery['auto_seen'] = True
        if (recovery['auto_seen'] and self.local_z >= min(2., CLIMB_DONE_ALT) and
                now-recovery['stream_since'] >= OFFBOARD_PRESTREAM_S):
            recovery['phase'] = 'offboard'
        if recovery['phase'] == 'offboard' and mode == 'OFFBOARD':
            self._takeoff_recovery = None
            self._climb_progress = (now, self.local_z)
            rospy.loginfo('[TAKEOFF_RECOVERY_DONE] uav=%s z=%.2f; resume guarded climb',
                          self.uav_id, self.local_z)
            return False
        if now-recovery['last_request'] >= 1.:
            desired = 'AUTO.TAKEOFF' if recovery['phase'] == 'auto' else 'OFFBOARD'
            if mode != desired:
                recovery['last_request'] = now
                try:
                    sent = self.mode_srv(0, desired).mode_sent
                    rospy.loginfo('[TAKEOFF_RECOVERY_REQUEST] uav=%s mode=%s sent=%s; '
                                  'wait for observed FCU mode',
                                  self.uav_id, desired, sent)
                except Exception as exc:
                    rospy.logwarn('[TAKEOFF_RECOVERY_REQUEST] uav=%s %s', self.uav_id, exc)
        return True

    def _check_climb_progress(self):
        """Report lack of measured climb; an armed FCU is not proof of flight.

        Do not re-arm or override a PX4 landing/failsafe from this watchdog.
        The ordinary output guards retain authority over the climb command.
        """
        now = rospy.Time.now().to_sec()
        z = self.local_z
        baseline = getattr(self, '_climb_progress', None)
        if z is None or not math.isfinite(z):
            self._climb_progress = None
            return
        self._takeoff_max_z = max(getattr(self, '_takeoff_max_z', z), z)
        if baseline is None or now < baseline[0] or z >= baseline[1] + 0.3:
            self._climb_progress = (now, z)
            return
        if now - baseline[0] >= 10.0:
            rospy.logerr_throttle(
                5.0, '[TAKEOFF_STALLED] uav=%s no_climb_s=%.1f z=%.2f '
                'connected=%s armed=%s mode=%s pos_sp=%s; 未完成起飞，不参与派机',
                self.uav_id, now - baseline[0], z,
                getattr(self.state, 'connected', False),
                getattr(self.state, 'armed', False), getattr(self.state, 'mode', ''),
                getattr(self, '_pos_sp', None))

    @serialized_visual_task
    def _publish_status(self):
        world = self.world_xy
        if world is None and not getattr(self, '_localization_fault', False):
            return

        st = UavStatus()
        st.header.stamp = rospy.Time.now()
        st.uav_id = self.uav_id
        # On a sticky positioning fault send a disconnected status so the
        # manager releases search leases and tracking responsibility. The
        # coordinates are only historical placeholders; connected=False
        # forbids assigning work or crediting coverage from them.
        reported = world or getattr(self, '_last_good_world', None) or self.offset
        st.x = reported[0]
        st.y = reported[1]
        st.z = self.local_z if self.local_z is not None else 0.0
        # 飞控链路仍连接不代表此机可执行任务。EKF 熔断或坠机恢复期间
        # world_xy 可能严重漂移；继续向 manager 报可用会占住搜索格，
        # 甚至把从未看过的区域标成已覆盖。
        st.connected = bool(self.state.connected and
                            getattr(self.state, 'armed', False) and
                            self.state.mode == 'OFFBOARD' and
                            getattr(self, '_takeoff_done', False) and
                            not getattr(self, '_landing', False) and
                            world is not None and
                            not getattr(self, '_localization_fault', False) and
                            not getattr(self, '_ekf_frozen', False) and
                            not getattr(self, '_crash_flagged', False))

        # 用实际位置计算所在格子（而非被分配的格子）- 使用 10m 覆盖栅格
        # 覆盖判定由 manager 端用感知半径批量处理，agent 只需上报位置
        actual_cell = (self.cov_grid.world_to_cell(world[0], world[1])
                       if st.connected else None)
        if actual_cell is not None:
            st.cell_ix = actual_cell[0]
            st.cell_iy = actual_cell[1]
            # 1=当前格四向观察完成；0=仅到达，-1=偏航失败。manager 不得
            # 把位置接近等同于相机已观察，否则前视盲区会整局漏搜。
            st.confidence = (-1.0 if getattr(self, '_search_scan_failed', False) else
                             1.0 if getattr(self, '_search_scan_done', False) and
                             self.assignment is not None and
                             self.assignment.task_type == 0 else 0.0)
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

        self._segment_status(st)
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
        # 从官方固定起飞点参数确定「局部->世界」的恒定平移
        if not self._calibrate_offset():
            raise SystemExit(1)

        self._configure_fcu()
        if not self._arm_and_offboard():
            raise SystemExit(1)

        rospy.loginfo("[%s] 开始协同搜索（高度层 %.1f m），等待管理器分配任务", self.uav_id, self.altitude_layer)
        while not rospy.is_shutdown():
            # 收到任务完成广播后退出搜索循环（但如果有降落任务，先执行降落）
            if self._mission_finished and not self._landing:
                rospy.loginfo("[%s] 任务已完成，退出搜索循环", self.uav_id)
                break
            try:
                self._control()
                self._detect_targets()      # 规则3：几何判定，命中即上报管理器
                if (rospy.Time.now() - self._last_status_t).to_sec() >= 1.0 / PUB_RATE:
                    self._publish_status()
                    self._last_status_t = rospy.Time.now()
            except Exception as e:
                rospy.logerr("[%s] 主循环异常: %s\n%s", self.uav_id, e, traceback.format_exc())
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
    SwarmAgent(uav_id, model_name).run()
