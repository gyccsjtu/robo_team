#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集群搜索单机节点（每架无人机跑一个实例，自建实现）。

职责（步骤 1：两机共享状态）：
  1. 从 /gazebo/model_states 读自身模型的世界坐标（地图系，与生成器 metadata 一致），
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
import rospy
import math
import traceback
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from std_msgs.msg import String, Float32

from robocup_swarm.msg import UavStatus, SearchAssignment, TargetState, TargetDetection
from robocup_navigation.astar import load_metadata, GridMap, plan
from slam_grid import SlamGrid
from swarm_task import LineOfSight, DETECT_RADIUS, CoverageGrid, GRID_SIZE_M
from csv_logger import logger
# 2026-10-03：飞控参数读回校验（取自队友 codex/radar-coordination-20261003 分支）
from fcu_configuration import configure as configure_fcu_parameters

# 覆盖栅格参数（与 manager 一致）
MAP_X_MIN, MAP_X_MAX = -100.0, 100.0
MAP_Y_MIN, MAP_Y_MAX = -50.0, 50.0


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
SEARCH_SWEEP_DEG = float(os.environ.get('SEARCH_SWEEP_DEG', '35.0'))
SEARCH_SWEEP_PERIOD_S = float(os.environ.get('SEARCH_SWEEP_PERIOD_S', '6.0'))
SEARCH_SCAN_YAW_TOL = math.radians(float(os.environ.get('SEARCH_SCAN_YAW_TOL_DEG', '12')))
SEARCH_SCAN_HOLD_S = float(os.environ.get('SEARCH_SCAN_HOLD_S', '0.25'))
SEARCH_SCAN_TIMEOUT_S = float(os.environ.get('SEARCH_SCAN_TIMEOUT_S', '16.0'))
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
EST_FROZEN_N        = int(os.environ.get('EST_FROZEN_N', '50'))    # 累计 50 拍 ≈ 5s@10Hz 仍未恢复即熔断
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

# ---- 目标盘旋确认 ----
# 国家一等奖标准修复（2026-10-04）：
#   旧 ORBIT_RADIUS=5.0 + ORBIT_SPEED=0.19 → 线速度 = 0.19×5 = 0.95 m/s，**刚好压在 1.0 m/s
#   触发线上**。一旦 EKF 抖动或位置控制超调，飞机瞬时速度就会突破 1.0 m/s → actor 触发逃跑。
# 官方 control_actor.py 在无人机距演员 <7m 时会持续把演员推走。
# 下载目录的六目标开发场景快照采用 8m 观察圈；留 1m 缓冲，同时保持
# 在本队 9.5m 的近距裁判播报门内。
ORBIT_RADIUS    = 8.0     # 盘旋半径 m（> actor uav_push_radius=7.0）
# 线速度 = ORBIT_SPEED * ORBIT_RADIUS，必须严格 < 1.0 m/s（防触发逃跑判定）。
ORBIT_SPEED     = 0.11    # r=8m 时切向 0.88m/s，保留逃跑阈值余量
# 近距观察半径：09:15 本轮 red1 在 2.5~4.5m 时框中心落到 480px
# 图像的 y=466~477，随后失去真实观测、官方坐标误差持续扩大。8m 使人
# 保持在画面内部且位于 7m 驱离区之外，仍低于 9.5m 播报进入距离门。
ORBIT_RADIUS_CLOSE = float(os.environ.get('ORBIT_RADIUS_CLOSE', '8.0'))
ORBIT_SPEED_CLOSE  = 0.05
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


def horizontal_slew_limit(vx, vy, last_v, close_mode=False):
    """Limit command changes while allowing prompt braking at ring entry."""
    lvx, lvy = last_v
    max_acc = MAX_ACC
    old_speed_sq = lvx * lvx + lvy * lvy
    if (close_mode and old_speed_sq > CLOSE_TRANSIT_SPEED ** 2 and
            vx * vx + vy * vy < old_speed_sq and
            vx * lvx + vy * lvy < old_speed_sq):
        max_acc = max(MAX_ACC, CLOSE_BRAKE_ACC)
    max_dv = max_acc / CTRL_RATE
    dvx, dvy = vx - lvx, vy - lvy
    dv = math.hypot(dvx, dvy)
    if dv > max_dv and dv > 1e-9:
        return lvx + dvx * max_dv / dv, lvy + dvy * max_dv / dv
    return vx, vy


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
    return (best[1], best[2]) if best is not None else preferred


def close_follow_velocity(err_x, err_y, target_vx, target_vy, observation_age):
    """近距跟随速度：位置纠偏加目标速度前馈，最终低于逃跑触发线。"""
    vx, vy = POS_KP * err_x, POS_KP * err_y
    if (0.0 <= observation_age <= TARGET_TTL and
            math.isfinite(target_vx) and math.isfinite(target_vy)):
        target_speed = math.hypot(target_vx, target_vy)
        if target_speed > 1.2:
            target_vx *= 1.2 / target_speed
            target_vy *= 1.2 / target_speed
        vx += target_vx
        vy += target_vy
    speed = math.hypot(vx, vy)
    if speed > CLOSE_TRANSIT_SPEED:
        vx *= CLOSE_TRANSIT_SPEED / speed
        vy *= CLOSE_TRANSIT_SPEED / speed
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
                           target_vx=0.0, target_vy=0.0):
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
    if speed > CLOSE_TRANSIT_SPEED:
        vx *= CLOSE_TRANSIT_SPEED / speed
        vy *= CLOSE_TRANSIT_SPEED / speed
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
SEARCH_DECEL_M      = float(os.environ.get('SEARCH_DECEL_M', '2.0'))
# v23 重定标（2026-10-09，规则 PDF 深挖 + 官方 master e9e4ef8 代码核对）：
# 官方 control_actor.py 逃跑机制实为「一次性闩锁」——
#   1) 触发是瞬时判定：reported（官方首次有效播报 /find_actor_N）且任意 UAV
#      <20m 且地速>1.0m/s → 立即逃跑（无「持续 2s」判定，v21 注释有误）；
#   2) escape_triggered 置位后全场无任何复位点 → 每个 actor 全场至多逃一次，
#      跑完路线后回 1m/s 随机走且永远不会再逃；
#   3) 逃跑速度 escape_speed=2.0，路线=距最近 UAV 最远的地图角（可达 100m+）。
# 因此 v21「0.95 追逃等官方复位」模型作废（0.95 追 2.0 必丢 YOLO 12m 接触，
# 正是 v20/v21 confirming lost 的根因）。新策略：
#   - 真在逃（观测速度≥ESCAPE_RUN_SPEED）→ 全速咬住，14m 内切牧羊 pace；
#   - 逃跑跑完=「已花掉」→ 解除一切压速护栏，全速贴脸消除；
#   - 未武装（/find_actor_N 从未发布）目标不会逃 → 全速接近到播报闸门外缘；
#   - 反复确认被拒 → 主动花掉逃跑（SPEND_ENABLE），变永久温顺目标后从容消除。
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
STALE_SEARCH_GRACE = float(os.environ.get("STALE_SEARCH_GRACE", "25.0"))
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
# 背景：官方相机是前视广角 114.6°，4 个 typhoon 机型逐一读过，都只有一个相机。
#       若到点 dwell 期机头"随便指" = 相机视野对到哪算哪，跟队友多点方案脱节。
# 行为：
#   - 飞行段：复用 _pick_local_goal() 的 LOOKAHEAD 点作为 _look_at（机头预摆）
#   - 到点 dwell 段：冻结到点前最后一个 local_goal（=下一个原本要飞到的搜索点）
# 关闭（=0）：保持原行为，到点交回 PX4 自管偏航。
DWELL_LOOKAHEAD = os.environ.get("DWELL_LOOKAHEAD", "1") not in ("0", "false", "False", "")
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
        self._anchor_done = False   # EKF 稳定锚定完成后才启用跳变检测
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
        self._search_scan_key = None
        self._search_scan_yaw0 = None
        self._search_scan_phase = 0
        self._search_scan_settle_t = None
        self._search_scan_started_t = None
        self._search_scan_done = False
        self._search_scan_failed = False

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
        # 2026-10-05 国家一等奖：缓存到点前的最后一个 local_goal，dwell 期冻结视野
        self._last_local_goal = None

        # ---- 覆盖栅格（与 manager 一致，10m 格）----
        # 覆盖格与 manager 使用同一份 metadata 边界；固定 ±100 的旧边界
        # 会让 x>100 的扫描状态 confidence 恒为 0，边缘格永远无法完成。
        self.cov_grid = CoverageGrid(
            _b.get("x_min", MAP_X_MIN), _b.get("x_max", MAP_X_MAX),
            _b.get("y_min", MAP_Y_MIN), _b.get("y_max", MAP_Y_MAX),
            GRID_SIZE_M)

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
        # 2026-10-06 国家一等奖: 团队侧已确认 → 切贴脸 2m / 0.05rad/s
        # 让 yolo 误差 < 1m 持续 15s 触发官方 score_cal +100 消除分.
        # 收到 manager 的 /swarm/confirmed 且 tid 匹配自己 _orbit_target 时,
        # _confirm_close=True; _orbit_target 切换或 /left_actors 离场时清回 False.
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
        self._confirm_start = 0.0   # 连续确认开始时间
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
        self._friend_seen = {}

        # ---- MAVROS 服务 ----
        self.arm_srv = rospy.ServiceProxy("/%s/mavros/cmd/arming" % uav_id, CommandBool)
        self.mode_srv = rospy.ServiceProxy("/%s/mavros/set_mode" % uav_id, SetMode)
        self.param_srv = rospy.ServiceProxy("/%s/mavros/param/set" % uav_id, ParamSet)

        # ---- 订阅 ----
        # 注意：**不订阅 /gazebo/model_states**。它是 250Hz × 687 模型 × 6.4MB/s 的
        # 巨型消息，每个 agent 都要用 Python 反序列化，实测每机吃掉 ~37% CPU
        # （比 PX4 还高），6 机时是灾难。改为只订阅轻量的 MAVROS local_position，
        # 启动时用一次 model_states 标定「局部->世界」的恒定平移即可。
        rospy.Subscriber("/%s/mavros/state" % uav_id, State, self._state_cb)
        # 官方剩余 actor 清单：权威的「谁还在场上」，用来剔掉已被删除的目标
        rospy.Subscriber("/left_actors", String, self._left_actors_cb, queue_size=5)
        # 别机认领广播：防止多架机扎堆确认同一个目标
        rospy.Subscriber("/swarm/orbit_claim", String, self._claim_cb, queue_size=20)
        self._claim_pub = rospy.Publisher("/swarm/orbit_claim", String, queue_size=10)
        rospy.Subscriber("/%s/mavros/local_position/pose" % uav_id, PoseStamped, self._local_cb)
        rospy.Subscriber("/%s/scan" % uav_id, LaserScan, self._scan_cb,
                 queue_size=1)
        rospy.Subscriber("/swarm/assignment", SearchAssignment, self._assign_cb)
        rospy.Subscriber("/swarm/target_states", TargetState, self._target_cb)
        rospy.Subscriber("/swarm/finish", String, self._finish_cb)
        # 2026-10-06 国家一等奖: 团队侧已确认广播
        rospy.Subscriber("/swarm/confirmed", String, self._confirmed_cb, queue_size=10)
        rospy.Subscriber("/swarm/uav_status", UavStatus, self._friend_status_cb)

        # ---- 发布 ----
        self.status_pub = rospy.Publisher("/swarm/uav_status", UavStatus, queue_size=5)
        self.detect_pub = rospy.Publisher("/swarm/detection", TargetDetection, queue_size=10)
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
        """世界坐标 = 局部坐标 + 标定偏移（偏移未标定好前返回 None）。

        === 2026-10-05 比赛规则硬约束（修复撞墙/越界）===
        即使 _local_cb 做了 EKF 漂移重锚定，仍可能存在以下情形导致 world_xy
        偏离物理真实位置（>1.5 倍地图半径）：
          - 漂移重锚定时窗内极端跳变被错过
          - 多次小幅漂移串行叠加 (dt=0.05s*1.2*6=0.36m 每帧不触发，但累积 30 帧 = 10m)
          - 单帧 dt 异常 (例如 mavros 回调突发延迟 → dt=0.9s → single_th=6.5m 误放行 6m)
        这里做「硬限位」：world_xy 超出地图 1.5 倍范围视为病态 → 返回 None。
        调用方收到 None → 触发悬停/重锚定（防止把病态坐标送给 A* / 派机 / 护栏）。
        """
        if self.local_xy is None or self.offset is None:
            return None
        wx = self.local_xy[0] + self.offset[0]
        wy = self.local_xy[1] + self.offset[1]
        # === v18（2026-10-07）EKF 慢漂移速度钳制（动态阈值） ===
        # v17 实测 agent_2：EKF 以 2~7 m/s 持续漂移（低于单帧 3m 与累计 7.2m/s
        # 重锚定阈值），world 被污染 +104m（真值 (7.3,-12.5) vs 估计 (111.8,-63.2)），
        # SLAM 贡献错位、冗余确认全是垃圾数据。物理约束：world 隐含速度 ≈ 真实
        # 飞行速度 + 漂移，其中真实速度受指令 _last_cmd_v 约束。动态阈值 =
        # 底噪 1.2 + 指令速度峰值×1.3：悬停时阈值仅 1.2 → 7m/s 漂移立刻被钳；
        # 全速飞行时阈值 ~9 → 正常飞行不触发。峰值按 1.5m/s 每秒衰减，防急减速
        # 期间惯性位移被误钳。超限部分（漂移）从 offset 中抵消 —— world 连续
        # 物理可控，EKF 假位移被丢弃。重锚定分支会清 _world_smooth 重置基准。
        _clamp_base = float(os.environ.get("WORLD_VEL_CLAMP_BASE", "1.2"))
        _cv = getattr(self, "_last_cmd_v", None)
        _cmd_spd = math.hypot(_cv[0], _cv[1]) if _cv else 0.0
        _tnow0 = rospy.Time.now().to_sec()
        _peak = getattr(self, "_world_cmd_peak", 0.0)
        _pdts = _tnow0 - getattr(self, "_world_cmd_peak_t", _tnow0)
        _peak = max(_cmd_spd, _peak - 1.5 * max(_pdts, 0.0))
        self._world_cmd_peak = _peak
        self._world_cmd_peak_t = _tnow0
        _clamp_v = _clamp_base + _peak * 1.3
        _prev = getattr(self, "_world_smooth", None)
        if _prev is not None and _clamp_v > 0:
            _dt = _tnow0 - getattr(self, "_world_smooth_t", _tnow0)
            if _dt > 0.01:
                _dx, _dy = wx - _prev[0], wy - _prev[1]
                _dist = math.hypot(_dx, _dy)
                _max_step = _clamp_v * _dt
                if _dist > _max_step:
                    _s = _max_step / _dist
                    _keep_x, _keep_y = _dx * _s, _dy * _s
                    # 漂移部分从 offset 抵消（offset -= 截断向量）
                    self.offset = (self.offset[0] - (_dx - _keep_x),
                                   self.offset[1] - (_dy - _keep_y))
                    wx, wy = _prev[0] + _keep_x, _prev[1] + _keep_y
                    if _dist - _max_step >= WORLD_TRUST_CLAMP_EXCESS_M:
                        self._world_trust_until = _tnow0 + WORLD_TRUST_HOLD_S
                    rospy.logwarn_throttle(
                        5.0, "[%s] EKF 慢漂移钳制：world 隐含速度 %.1fm/s > %.1f"
                        "（cmd峰值%.1f），截断 %.2fm（offset 吸收）", self.uav_id,
                        _dist / _dt, _clamp_v, _peak, _dist - _max_step)
            self._world_smooth_t = _tnow0
        self._world_smooth = (wx, wy)
        # Judge map bounds come from metadata.  In the 13:35 match x_max=155,
        # while fixed MAP_X_MAX=100 made valid x=150..155 trigger this gate.
        _grid = getattr(self, 'grid', None)
        if _grid is not None:
            _hard_xmin, _hard_xmax, _hard_ymin, _hard_ymax = (
                estimated_world_hard_bounds(_grid))
        else:
            # Unit-test/minimal-agent construction before metadata load.
            _hard_xmin, _hard_xmax = MAP_X_MIN * 1.5, MAP_X_MAX * 1.5
            _hard_ymin, _hard_ymax = MAP_Y_MIN * 1.5, MAP_Y_MAX * 1.5
        if not (_hard_xmin <= wx <= _hard_xmax and _hard_ymin <= wy <= _hard_ymax):
            # 仅在错位刚发生时打一次，避免每秒刷屏
            _now = rospy.Time.now().to_sec()
            if not hasattr(self, '_world_bad_last_log') or (_now - self._world_bad_last_log) > 2.0:
                self._world_bad_last_log = _now
                rospy.logerr_throttle(2.0,
                                      "[%s] world_xy=(%.1f,%.1f) 病态超界，拒用并触发重锚定",
                                      self.uav_id, wx, wy)
                # === v13（2026-10-07）修复重锚定负反馈循环 ===
                # 旧逻辑：offset = _offset_param（起飞点）——但飞机飞行中 EKF 缓慢
                # 漂移（每帧 < 阈值不触发跳变检测）把 local_xy 带到 (92,61) 时，
                # offset 本来就是参数值 → 无操作 → world 恒超界恒 None →
                # A*/派格/守卫全瘫。若 offset 曾被跳变检测改过，拉回起飞点则让
                # world 瞬间跳变 → 派格全乱 → 负反馈。
                # 新逻辑：重锚定到「最后可信 world」——offset = last_good - local_now，
                # world 冻结在最后可信位置，后续 local 增量（真实运动）自然外推，
                # world 连续且方向正确；无 last_good（启动期）才退回起飞点参数。
                _lg = getattr(self, '_last_good_world', None)
                if _lg is not None and self.local_xy is not None:
                    self.offset = (_lg[0] - self.local_xy[0],
                                   _lg[1] - self.local_xy[1])
                    self._world_trust_until = rospy.Time.now().to_sec() + WORLD_TRUST_HOLD_S
                    rospy.logwarn("[%s] EKF 病态重锚定：world 冻结回最后可信 (%.1f,%.1f)，"
                                  "offset→(%.2f,%.2f)（外推模式）",
                                  self.uav_id, _lg[0], _lg[1],
                                  self.offset[0], self.offset[1])
                elif self._offset_param is not None:
                    self.offset = self._offset_param
                self._ekf_window = None
                # v18：重锚定后 world 基准已变，清速度钳制基准防误钳
                self._world_smooth = None
            return None
        # 合法 world：记录为最后可信位置（EKF 病态时的重锚定基准）
        self._last_good_world = (wx, wy)
        return (wx, wy)

    def _world_trusted(self):
        """R2（2026-10-09）：世界坐标当前是否可信。

        显著钳制/重锚定后 WORLD_TRUST_HOLD_S 内返回 False——防止用被污染的
        world_xy 触发"到圈即贴脸"（v26 agent_0 距 t4 真身 48m 处假到达）。
        返回 False 时调用方应悬停等世界坐标自愈，或只用保守的距离判据。
        """
        return rospy.Time.now().to_sec() > getattr(self, "_world_trust_until", 0.0)

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
        规则 §2.5(11) 审计红线：正式比赛严禁订阅 /gazebo/model_states；
        本函数仅在 SWARM_CALIB=truth 开发自验模式下启用，启动期打印审计警告。
        """
        rospy.logwarn(
            "[%s][AUDIT] SWARM_CALIB=truth 启用真值标定，订阅 /gazebo/model_states —— "
            "违反规则 §2.5(11)，仅限开发自验，正式比赛严禁启用！",
            self.uav_id)
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
        self.local_z = msg.pose.position.z
        new_xy = (msg.pose.position.x, msg.pose.position.y)
        now = rospy.Time.now().to_sec()
        # EKF 原点重置检测（方案 B 运行时半部分）：
        # local_xy 单帧跳变 > max(EKF_JUMP_MIN_M, 物理速度上限) → 非真实运动，
        # 平移 offset 吸收跳变，保持 world_xy 连续（否则越界护栏会把飞机锁死）。
        # _anchor_done 之前（EKF 未稳定）不检测——预热阶段的抖动由起飞锚定兜底。
        #
        # === 2026-10-05 比赛规则硬约束（修复累计漂移）===
        # 旧阈值 MAX_SPEED * dt * 3.0：dt=0.5s 时允许 9m/s，远超真实物理上限
        # (官方恐怖分子 2m/s, 本机 MAX_SPEED 6m/s)，慢速 EKF 累计漂移 (实测
        # uav_2 在 2s 内 world_xy 漂 24m) 无法被单帧跳变检测捕获。
        # 改用 MAX_SPEED * dt * 1.2 + 累积窗口 1s 双重判定：
        #   (a) 单帧 > MAX_SPEED*dt*1.2 (≈ 7.2m/s) 即触发 → 单帧跳变
        #   (b) 滑动 1s 内位移 > MAX_SPEED*1.2 (7.2m) → 累计漂移
        # 两者任一触发即吸收跳变重锚定 offset + 写 EKF_DRIFT_RESET 标记。
        if self._anchor_done and self.local_xy is not None and self.offset is not None:
            dt = now - self._local_prev_t
            if 0.0 < dt < 1.0:
                jump = math.hypot(new_xy[0] - self.local_xy[0],
                                  new_xy[1] - self.local_xy[1])
                # (a) 单帧阈值：严格上限 = 物理速度上限 * dt * 1.2
                single_th = max(EKF_JUMP_MIN_M, MAX_SPEED * dt * 1.2)
                # (b) 累积阈值：滑动窗口 ≤ 1s 内位移 > 7.2m (MAX_SPEED*1.2)
                cum_disp = jump
                cum_dt = dt
                cum_th_hit = False
                _past = getattr(self, '_ekf_window', None)
                if _past:
                    # 拼接：(xy, t, offset_at_that_time) 队列；dt 累计不超过 1s
                    while _past and (now - _past[0][1]) > 1.0:
                        _past.pop(0)
                    _past.append((new_xy, now, self.offset))
                    self._ekf_window = _past
                    # === 2026-10-05 国家一等奖修：累积判定要稳定窗口 ===
                    # 原条件 len(_past) >= 2：只有 2 项时 cum_dt 极小（0.06s），
                    # MAX_SPEED*0.06*1.2=0.4m，而 1.71m 单帧 EKF 抖动必然触发
                    # → 起飞后 1s 内反复重锚定 → 坐标系雪崩 → 飞机永远悬停。
                    # 改为：窗口至少 5 项（≈0.25s@20Hz）才做累积判定，确保
                    # 跨多个采样帧的真实漂移，而不是单帧抖动。
                    # v17：5→3 —— 窗口起点保持语义下触发=吸收漂移（不再固化），
                    # 更短窗口让 world 锯齿从 ±5.4m 缩到 ±2m；cum_dt>0.10 仍保留
                    # （等效 ≥4 项@30Hz），起飞抖动由吸收语义安全处理。
                    EKF_CUM_MIN_N = int(os.environ.get('EKF_CUM_MIN_N', '3'))
                    if len(_past) >= EKF_CUM_MIN_N:
                        first_xy, first_t = _past[0][0], _past[0][1]
                        cum_disp = math.hypot(new_xy[0] - first_xy[0],
                                              new_xy[1] - first_xy[1])
                        cum_dt = now - first_t
                        if cum_dt > 0.10 and cum_disp > MAX_SPEED * cum_dt * 1.2:
                            cum_th_hit = True
                else:
                    self._ekf_window = [(new_xy, now, self.offset)]

                if jump > single_th or cum_th_hit:
                    # === v17（2026-10-07）修复重锚定正反馈风暴 ===
                    # 旧逻辑无论单帧/累积触发都用「当前帧 world」作保持目标。
                    # EKF 原点持续漂移场景（v16 agent_5 原点 40m/s 雪崩）：
                    # 窗口重置导致中间 N-1 帧漂移不被吸收，而「当前 world」已含
                    # 这些未吸收漂移 → 每次重锚定把漂移固化进 world → world 以
                    # 漂移速率飞涨（v16 实测 world +27→+74→瞬跳-79.5，越界 93m
                    # 触发回收拉锯，8.6s 内冲到 -148）。
                    # 修复：窗口 ≥2 项时保持「窗口起点 world」（first_xy+offset）
                    # ——自上次重锚定以来的全部漂移一次性吸收，world 稳定在真值
                    # 附近小幅锯齿。窗口内位移速率必超物理上限（累积阈值
                    # 7.2m/s > MAX_SPEED），故窗口内变化全是漂移、无真实运动被误抹。
                    # 防御（v17b）：窗口记录每帧当时的 offset；窗口起点 offset
                    # ≠ 当前 offset → offset 被外部改过（world_xy 超界重锚定
                    # v13 修复 2），窗口起点 world 已失效 → 退回当前帧保持
                    # （当前 world = 新 offset 语义下的最新值，即 _last_good_world
                    # 外推基线）。旧版用「cand-cur 差 >30m」检测——数学上 offset
                    # 在差值中完全抵消，永远检测不到外改，已废弃。
                    _win = getattr(self, '_ekf_window', None)
                    _keep_cur = True
                    _ext = False
                    if _win and len(_win) >= 2:
                        _fx, _ft, _foff = _win[0]
                        if (abs(_foff[0] - self.offset[0]) < 0.01
                                and abs(_foff[1] - self.offset[1]) < 0.01):
                            wx = _fx[0] + self.offset[0]
                            wy = _fx[1] + self.offset[1]
                            _keep_cur = False
                        else:
                            _ext = True
                    if _keep_cur:
                        wx = self.local_xy[0] + self.offset[0]
                        wy = self.local_xy[1] + self.offset[1]
                    new_off = (wx - new_xy[0], wy - new_xy[1])
                    rospy.logwarn("[%s] EKF 原点跳变 %.2fm（dt=%.3fs，单帧阈值=%.2fm；累积 %.2fm/%.2fs）"
                                  "，offset 重锚定 (%.2f,%.2f)→(%.2f,%.2f)，world 保持 (%.2f,%.2f)%s",
                                  self.uav_id, jump, dt, single_th, cum_disp, cum_dt,
                                  self.offset[0], self.offset[1],
                                  new_off[0], new_off[1], wx, wy,
                                  "［窗口起点］" if not _keep_cur else
                                  ("［offset外改退回］" if _ext else ""))
                    self.offset = new_off
                    # 重置累积窗口：避免同一次跳变被连续多帧反复触发
                    self._ekf_window = [(new_xy, now, new_off)]
                    # v18：重锚定后 world 基准已变，清速度钳制基准防误钳
                    self._world_smooth = None
                    # R2（2026-10-09）：原点跳变重锚定同样污染 world，短时不可信
                    self._world_trust_until = now + WORLD_TRUST_HOLD_S
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
        # 跳变后估计重新可信 → 一次性校验 offset（修正量合理才采纳，见常量注释）
        pend = getattr(self, '_est_jump_pending', None)
        if pend is not None and self.offset is not None and self._est_trustworthy():
            new_off = (pend[0] - new_xy[0], pend[1] - new_xy[1])
            d = math.hypot(new_off[0] - self.offset[0],
                           new_off[1] - self.offset[1])
            if d <= EKF_REANCHOR_MAX_M:
                rospy.logwarn("[%s] 估计已重新稳定，offset 校正 (%.2f,%.2f)→"
                              "(%.2f,%.2f)，保持 world=(%.2f,%.2f)", self.uav_id,
                              self.offset[0], self.offset[1],
                              new_off[0], new_off[1], pend[0], pend[1])
                self.offset = new_off
            else:
                rospy.logerr("[%s] 跳变修正量 %.2fm > %.1fm → 判为估计失控，"
                             "不采纳 offset 修正，保持 (%.2f,%.2f)", self.uav_id, d,
                             EKF_REANCHOR_MAX_M, self.offset[0], self.offset[1])
            self._est_jump_pending = None

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
        if self._scan is None or self.world_xy is None:
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
                                         SLAM_MAX_RANGE, SLAM_DEPTH_CELLS)
        except Exception as exc:
            rospy.logerr_throttle(5.0, '[%s] 激光建图失败: %s', self.uav_id, exc)
            return
        if marked <= 0:
            return
        self._slam_dirty = True
        # 新障碍出现 → 当前路径可能穿障，重规划（节流 + 不覆盖 pending 请求）
        if self.path and self.path_target is not None \
                and now - self._slam_replan_t >= SLAM_REPLAN_SEC:
            with self._plan_lock:
                pending = self._plan_pending is not None
            if not pending:
                self._slam_replan_t = now
                self._request_plan(self.path_target)
        # 发布栅格快照（节流）
        if self._slam_dirty and now - self._slam_pub_t >= SLAM_PUB_SEC:
            self._slam_pub_t = now
            self._slam_dirty = False
            try:
                self.occ_pub.publish(String(data=json.dumps({
                    "uav_id": self.uav_id,
                    "width": self.slam.width,
                    "height": self.slam.height,
                    "resolution": self.slam.resolution,
                    "origin": [self.slam.origin[0], self.slam.origin[1]],
                    "rle": self.slam.to_rle(),
                })))
            except Exception:
                pass

    def _assign_cb(self, msg):
        if msg.uav_id != self.uav_id:
            return
        if msg.task_type == 255:
            old_tid = getattr(self, '_track_assigned_id', None)
            if old_tid and msg.target_id and msg.target_id != old_tid:
                return
            self.assignment = None
            self._search_scan_key = None
            self._search_scan_done = False
            self._search_scan_failed = False
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
            self.path = []
            self.path_target = None
            self._last_flight_v = (0.0, 0.0)
            with self._plan_lock:
                self._plan_pending = None
            return
        if msg.task_type == 0:
            scan_key = (msg.cell_ix, msg.cell_iy,
                        round(msg.target_x, 2), round(msg.target_y, 2))
            if scan_key != self._search_scan_key:
                self._search_scan_key = scan_key
                self._search_scan_yaw0 = None
                self._search_scan_phase = 0
                self._search_scan_settle_t = None
                self._search_scan_started_t = None
                self._search_scan_done = False
                self._search_scan_failed = False
        else:
            self._search_scan_key = None
            self._search_scan_done = False
            self._search_scan_failed = False
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

    def _target_cb(self, msg):
        """缓存恐怖分子真值位置（几何判定用；真机上是视觉/裁判给出的观测）。"""
        if msg.eliminated:
            self.targets.pop(msg.target_id, None)
            self._target_state.pop(msg.target_id, None)
            # BUGFIX: 目标消除后必须清除盘旋状态，否则飞机会卡在盘旋不动
            if self._orbit_target == msg.target_id:
                rospy.loginfo("[%s] 目标 %s 已消除，清除盘旋状态", self.uav_id, msg.target_id)
                self._orbit_target = None
                self._orbit_center = None
                self._confirm_start = 0.0
                # 2026-10-06 国家一等奖: 清贴脸标志
                if self._confirm_close:
                    self._confirm_close = False
                    self._confirm_close_tid = None
                    self._orbit_prev = None
            # 同步解除追踪指派去重：该目标已不存在
            if getattr(self, '_track_assigned_id', None) == msg.target_id:
                self._track_assigned_id = None
                self._target_to_orbit = None
            return
        self.targets[msg.target_id] = (msg.x, msg.y, msg.vx, msg.vy)
        now = rospy.Time.now().to_sec()
        # header.stamp 来自最后一次真实图像。bridge 的保活帧仍会到达，
        # 但不能重置目标新鲜度；否则 8s 内的冻结位置每帧都变成“新观测”。
        sample = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
        if sample <= 0.0 or sample > now:
            sample = now
        self._target_state[msg.target_id] = (int(msg.state), sample)
        self._t_seen[msg.target_id] = sample
        self._escape_bookkeeping(msg.target_id)   # v23：观测速度 → 逃跑生命周期

    def _friend_status_cb(self, msg):
        """接收友机位置和高度，用于避碰"""
        if msg.uav_id != self.uav_id:
            self._friend_positions[msg.uav_id] = (msg.x, msg.y, msg.z)
            self._friend_seen[msg.uav_id] = rospy.Time.now().to_sec()

    def _finish_cb(self, msg):
        """收到任务完成广播后退出搜索循环。"""
        if msg.data == "MISSION_FINISHED":
            rospy.loginfo("[%s] 收到任务完成广播", self.uav_id)
            self._mission_finished = True
            # 兜底：如果还没进入降落模式，则进入
            if not self._landing and self.assignment is not None and getattr(self.assignment, 'task_type', 0) == 3:
                self._landing = True

    def _confirmed_cb(self, msg):
        """2026-10-06 国家一等奖: manager 团队侧连续确认 20s → 切贴脸模式.

        String 格式: \"tid:<target_id>\".
        **关键修复 (2026-10-06 验证)**:
        旧逻辑只匹配 self._orbit_target == tid, 但 manager 在 4 秒间隔同时
        确认 t1/t2 时本机 _orbit_target 已被 reassign 切走 → 贴脸只持续 4s,
        远不够官方 15s streak 触发消除. 现在改为:
        - 匹配范围扩大到 (a) _orbit_target OR (b) _track_assigned_id OR
          (c) 本机是该 tid 的 _claims 发布者 — 真正"在追这个目标"的所有 agent
        - 锁定至少 30s (CONFIRM_LOCK_DURATION), 期间忽略新 confirm 切走逻辑
          (即使 manager 接着确认其他 tid 也不退出)
        - 仅在 (a) 30s 计时到 / (b) /left_actors 真离场 / (c) mission finished 才退出
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

    # ---------------- 目标检测（规则3：几何判定） ----------------
    def _detect_targets(self):
        """对每个已知目标做「距离 + 视线遮挡」判定，命中则发布 TargetDetection。

        规则3 的几何判定：水平距离 < DETECT_RADIUS **且** 中间无建筑遮挡。
        注意只用水平距离 —— 无人机在 6m 高度、目标在地面，垂直差恒定，
        水平距才是决定「能否看到」的量（与比赛判定的平面几何一致）。
        """
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

        configure_fcu_parameters(_pull, self._set_param, _get,
                                 {"NAV_RCL_ACT": 0, "COM_RCL_EXCEPT": 4})
        rospy.loginfo("[%s] 飞控参数已读回验证：NAV_RCL_ACT=0 COM_RCL_EXCEPT=4",
                      self.uav_id)

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
            # 但仍启用跳变检测，EKF 飞行中原点重置时保持 world_xy 连续。
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
        self._anchor_done = True   # 此后 _local_cb 启用跳变重锚定

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
        if now - self._scan_t > RADAR_FRESH_S:
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
                advance = min(RADAR_BACKOFF_MAX, max(speed, 0.8))
                advance_mode = '推进'
            elif best_r >= 1.0:
                advance = 0.5
                advance_mode = '横滑'
            elif best_r >= 0.5:
                advance = 0.3
                advance_mode = '慢滑'
            else:
                advance = 0.4    # 极端贴墙（best_r<0.5）：低速横滑 0.4 m/s 脱离
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
                        _bx = RADAR_JAM_ESCAPE_SPD * math.cos(self.yaw + math.pi)
                        _by = RADAR_JAM_ESCAPE_SPD * math.sin(self.yaw + math.pi)
                        guarded = (_bx, _by)
                        self._radar_retreating = True
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
            self._radar_retreating = True                          # 主流程旁路加速度限幅
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
            guarded = (guarded[0] - cy * RADAR_BACKOFF_SPD,
                       guarded[1] - sy * RADAR_BACKOFF_SPD)
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
        if scan is None or now - self._scan_t > RADAR_FRESH_S:
            return 0.0, 0.0
        body_angle = (math.atan2(vy, vx) - self.yaw + math.pi) % (2 * math.pi) - math.pi
        nearest = None
        nearest_any = None
        nearest_any_angle = None
        for i, raw in enumerate(scan.ranges):
            beam = scan.angle_min + i * scan.angle_increment
            delta = (beam - body_angle + math.pi) % (2 * math.pi) - math.pi
            r = float(raw)
            if math.isnan(r):
                continue
            if not math.isfinite(r) or r > scan.range_max or r < RADAR_SELF_ECHO_M:
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
        wx = self.world_xy
        if wx is None:
            return vx, vy
        # ---- v11 复盘（2026-10-07）自身格占用豁免 ----
        # 无人机自身所在格已被判占用（SLAM 自回波污染残留/贴墙膨胀区）时，
        # 锥形前瞻必然全堵 → 旧逻辑「前方堵死 → 刹停」= 永久死锁
        # （v11 六机各刹停 175 次全程 0 位移）。放行当前速度让飞机先离开
        # 占用区：真实近障由雷达守卫兜底，逃出路径由 A* 最近自由格重试规划。
        own_cell = self.grid.world_to_cell(wx)
        if own_cell is not None and not self.grid.is_free(own_cell):
            rospy.logwarn_throttle(5.0, '[%s] 栅格近障：自身格被占（膨胀区/污染残留）→ 豁免放行',
                                   self.uav_id)
            return vx, vy
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

    def _oscillation_breakout(self, vx, vy):
        """R1（2026-10-09）：守卫交替振荡脱困。

        雷达/栅格守卫在"近障->转向"路径上互相覆盖（雷达转向右侧、栅格判前方占
        再转回左侧），每 50ms 方向翻转一次，位置原地打转（v26 撞墙主因）。检测
        速度方向在窗口内频繁大角度翻转 → 强制沿最近一个目标航向直行，期间守卫
        只做速度限幅（由 _grid_guard_velocity 的互斥分支承载）。
        """
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
            _sp = getattr(self, "_osc_break_sp", OSC_BREAKOUT_SPD)
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
            self._osc_break_sp = OSC_BREAKOUT_SPD
            self._osc_break_until = _now + OSC_BREAKOUT_S
            self._osc_prev = None
            self._osc_flicks = 0
            self._osc_win_t0 = 0.0
            rospy.logwarn('[%s] 守卫交替振荡 %.1fs/%d次 → 强制脱困直行 %.0fs',
                          self.uav_id, _win_span, _flicks, OSC_BREAKOUT_S)
            return OSC_BREAKOUT_SPD * math.cos(hd), OSC_BREAKOUT_SPD * math.sin(hd)
        # 窗口横跨过长但翻转数不足（偶发转向/正常朝向调整）→ 重置窗口，
        # 避免"一次翻转 + 很久以后"误触发脱困。真振荡会持续翻转、窗口重开。
        if _win_span > max(OSC_WATCH_S, 3.0) * 2.0 and _flicks < OSC_SWITCH_N:
            self._osc_prev = None
            self._osc_flicks = 0
            self._osc_win_t0 = 0.0
        return vx, vy

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

        === 2026-10-05 比赛规则硬约束：分级回收速度 ===
        旧逻辑：固定 OOB_RECOVER_SPEED=3 m/s + 接近目标减速。
        比赛硬约束要求：「不能撞墙后停止行为」→ 必须能从越界状态可靠收回。
        分级回收：超出越多 → 回收越快（MAX_SPEED 上限），但接近 inset 时仍按
        旧逻辑减速（防回弹穿边）。
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
        # 分级速度：d>1m → MAX_SPEED 全力回收；d≤1m → 线性减速到 OOB_RECOVER_SPEED。
        # 不超过 OOB_RECOVER_SPEED + 2.0 m/s（防过冲）。
        if d > 1.0:
            spd = MAX_SPEED
        else:
            spd = min(OOB_RECOVER_SPEED, d) + 0.5 * d * (MAX_SPEED - OOB_RECOVER_SPEED)
        spd = min(spd, OOB_RECOVER_SPEED + 2.0)
        return spd * dx / d, spd * dy / d

    def _send_vel(self, vx, vy, vz=None):
        """下发 ENU 水平速度 + 自适应高度。

        vz=None 时按当前高度竖直 P 控制爬升到自适应高度；预热/悬停阶段显式传
        vz=0.0 发纯零速，避免 EKF 未收敛时带爬升速度导致 OFFBOARD 被拒。
        """
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
            _m = TwistStamped()
            _m.header.stamp = rospy.Time.now()
            _m.header.frame_id = 'world'
            _m.twist.linear.x = 0.0
            _m.twist.linear.y = 0.0
            _m.twist.linear.z = _vz
            if FLIGHT_OUTPUT == "pos":
                self._publish_pos_output(0.0, 0.0, min(target_alt, ALT_RECOVER_CEIL), _vz)
            else:
                self.vel_pub.publish(_m)
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
                if FLIGHT_OUTPUT == "pos":
                    self._publish_pos_output(0.0, 0.0, _alt, 0.0)
                else:
                    _m = TwistStamped()
                    _m.header.stamp = rospy.Time.now()
                    _m.header.frame_id = 'world'
                    self.vel_pub.publish(_m)
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
            if FLIGHT_OUTPUT == "pos":
                self._publish_pos_output(0.0, 0.0, _alt, 0.0)
            else:
                _m = TwistStamped()
                _m.header.stamp = rospy.Time.now()
                _m.header.frame_id = 'world'
                self.vel_pub.publish(_m)
            return

        # === 位置估计可信性闸门（EST_GUARD，2026-10-03，最高优先级）===
        # 只在自动飞行路径生效（vz is None；预热/降落都显式传 vz，不受影响）。
        # 估计不可信时绝不做任何基于 world_xy 的决策 —— 实测「越界主动回收 →
        # (0,3) 当前(63.2,-66.1)」是在追一个 100m 外的幻影，越追越远。
        # （上方已替换为本修复的雪崩熔断版；此注释保留说明历史脉络）

        # 最高优先级：越界主动回收，绕过其他守卫（界外无障碍）。
        rec = self._bounds_recovery_velocity()
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
            self._radar_retreating = False  # 重置标记
            self._radar_strict = False      # R1：每帧重置（雷达守卫内按需置 True）
            vx, vy = self._radar_guard_velocity(vx, vy)
            # 无激光时栅格兜底（雷达在线也再过一道，双保险）
            # R1：雷达强介入时 _grid_guard_velocity 内部已自动让位只夹速度
            vx, vy = self._grid_guard_velocity(vx, vy)
            vx, vy = self._map_guard_velocity(vx, vy)
            # R1 振荡死锁检测：守卫交替反向（雷达/栅格对阵）持续太久 → 强制
            # 沿当前自由扇区直行 2s，期间两守卫只做速度限幅（v26 agent_3 例证）
            if math.hypot(vx, vy) >= 0.05:
                vx, vy = self._oscillation_breakout(vx, vy)

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
        if not guard_stop and self._last_cmd_v is not None and not getattr(self, "_radar_retreating", False):
            vx, vy = horizontal_slew_limit(
                vx, vy, self._last_cmd_v,
                close_mode=bool(getattr(self, '_confirm_close', False)))
        if guard_stop:
            vx, vy = 0.0, 0.0
        vx, vy = self._final_scan_cap(vx, vy)
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
            # === v14 Fix D：连续爬升卡死闩锁 ===
            # EKF z 估计病态偏低（卡住不动）时 alt_error 恒正，P 控制恒输出爬升，
            # 真值一路冲破 6m（v14 agent_4 实测 236s 漂升）。若持续 CLIMB_LATCH_S
            # 仍在爬（vz>0.2）而高度几乎没变（|Δz|<CLIMB_LATCH_DZ），判为估计卡死，
            # 闩锁 vz≤0 直到误差收敛或高度恢复变化。
            _now_s = rospy.Time.now().to_sec()
            # === v18 修复：地面复飞豁免 ===
            # v17 实测（logs_20261007_213201）：agent_1 熔断 LAND 落地后 CRASH
            # 恢复爬升（vz=0.6）被本闩锁拦死（EKF z 病态 |Δz|=0.22<0.30）→
            # 永远趴地。6m 红线防护针对高空冲顶；z<1.5m 时爬升到 1.5m 距红线
            # 极远，豁免闩锁。爬过 1.5m 后若仍卡死，闩锁正常介入。
            if cmd.twist.linear.z > 0.2 and not self._climb_latch \
                    and (self.local_z is None or self.local_z > 1.5):
                if self._climb_t0 is None:
                    self._climb_t0 = _now_s
                    self._climb_z0 = self.local_z
                elif _now_s - self._climb_t0 >= CLIMB_LATCH_S:
                    if abs(self.local_z - self._climb_z0) < CLIMB_LATCH_DZ:
                        self._climb_latch = True
                        self._climb_latch_z = self.local_z
                        rospy.logerr('[%s] P 控制爬升卡死闩锁触发：持续 %.1fs vz=%.2f '
                                     '但 |Δz|=%.2f < %.2f → 闩锁 vz≤0 防冲红线',
                                     self.uav_id, CLIMB_LATCH_S, cmd.twist.linear.z,
                                     abs(self.local_z - self._climb_z0), CLIMB_LATCH_DZ)
                        # v16: 自适应降层 0.5m（累计 ≤1.0m，层高下限 3.5m），
                        # alt_error 收敛后闩锁解锁，不再反复顶同一层
                        if self._alt_sag < 1.0 and target_alt - 0.5 >= 3.5:
                            self._alt_sag += 0.5
                            rospy.logerr('[%s] 自适应降层 %.1fm → 层高 %.1f'
                                         '（EKF z 病态，保底可用高度）',
                                         self.uav_id, self._alt_sag,
                                         target_alt - self._alt_sag)
                    else:
                        self._climb_t0 = _now_s
                        self._climb_z0 = self.local_z
            elif cmd.twist.linear.z <= 0.2:
                self._climb_t0 = None
            if self._climb_latch:
                _moved = self._climb_latch_z is not None and \
                         abs(self.local_z - self._climb_latch_z) >= CLIMB_LATCH_DZ
                # v18: 地面豁免——趴地复飞时闩锁自动解除（防 vz≤0 死锁）
                if alt_error < 0.3 or _moved or \
                        (self.local_z is not None and self.local_z < 1.5):
                    self._climb_latch = False
                    self._climb_t0 = None
                    self._climb_latch_z = None
                else:
                    cmd.twist.linear.z = min(cmd.twist.linear.z, 0.0)
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
            _yerr = (_bear - self.yaw + math.pi) % (2.0 * math.pi) - math.pi
            _yr = YAW_KP * _yerr
            if _yr > YAW_RATE_MAX:
                _yr = YAW_RATE_MAX
            elif _yr < -YAW_RATE_MAX:
                _yr = -YAW_RATE_MAX
            cmd.twist.angular.z = _yr
        # 记录最终下发的水平速度（供路径未就绪时继续发，保证 offboard 不断流）
        self._last_flight_v = (vx, vy)
        if FLIGHT_OUTPUT == "pos":
            self._publish_pos_output(cmd.twist.linear.x, cmd.twist.linear.y,
                                     target_alt, cmd.twist.linear.z,
                                     cmd.twist.angular.z)
        else:
            self.vel_pub.publish(cmd)

    def _publish_pos_output(self, vx, vy, target_alt, vz=None, yaw_rate=0.0):
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
            m = TwistStamped()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = 'world'
            m.twist.linear.x = vx
            m.twist.linear.y = vy
            m.twist.linear.z = 0.0 if vz is None else vz
            m.twist.angular.z = yaw_rate
            self.vel_pub.publish(m)
            return
        step_x = vx * POS_SP_LEAD
        step_y = vy * POS_SP_LEAD
        step = math.hypot(step_x, step_y)
        if step > POS_SP_STEP_MAX and step > 1e-9:
            k = POS_SP_STEP_MAX / step
            step_x *= k
            step_y *= k
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
        sp.pose.position.z = max(0.0, min(z, ALT_RECOVER_CEIL))
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
    def _request_plan(self, goal_xy):
        """投递一个规划请求并立即返回（不阻塞控制循环）。多次请求只保留最新目标。"""
        with self._plan_lock:
            self._plan_pending = (float(goal_xy[0]), float(goal_xy[1]))
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
                self._plan_pending = None
                self._plan_event.clear()
            try:
                path, target, ok = self._compute_plan(goal)
            except Exception as exc:
                rospy.logerr("[%s] 规划线程异常: %s\n%s",
                             self.uav_id, exc, traceback.format_exc())
                ok = False
            if ok:
                # list/引用赋值在 GIL 下原子，主循环只会看到「旧路径」或「新路径」
                self.path = path
                self.path_target = target
            else:
                # 保留上一条已经通过 A* 的路径。移动目标或临时 GOAL_OCCUPIED
                # 不应让速度每次规划失败都归零，否则会产生「短脉冲 + 长悬停」；
                # 初始尚无安全路径时才悬停等待下一次规划。
                _wxy = self.world_xy
                _out = (_wxy is not None and
                        self.grid.world_to_cell(_wxy) is None)
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
                d = math.hypot(wx - cx, wy - cy)
                if d < best_d:
                    best_d = d
                    best = (wx, wy)
        return best

    def _pick_local_goal(self):
        """沿全局路径从最近点往前取 LOOKAHEAD 距离的引导目标。"""
        if not self.path or self.world_xy is None:
            return None
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
                return (x0 + (x1 - x0) * frac, y0 + (y1 - y0) * frac)
            acc += seg
        return self.path[-1]

    def _search_scan_step(self):
        """原地转满一圈，确认前视相机实际朝过四个方向后才报覆盖完成。"""
        assignment = getattr(self, 'assignment', None)
        if (assignment is None or getattr(assignment, 'task_type', None) != 0 or
                self._search_scan_key is None):
            self._send_vel(0.0, 0.0)
            return
        now = rospy.Time.now().to_sec()
        if self._search_scan_yaw0 is None:
            self._search_scan_yaw0 = self.yaw
            self._search_scan_started_t = now
            self._search_scan_phase = 0
            self._search_scan_settle_t = None
        if self._search_scan_done or self._search_scan_failed:
            self._send_vel(0.0, 0.0)
            return
        if now - self._search_scan_started_t > SEARCH_SCAN_TIMEOUT_S:
            self._search_scan_failed = True
            rospy.logwarn('[%s] 搜索格转向超时，拒绝标记已观察: %s',
                          self.uav_id, self._search_scan_key)
            self._send_vel(0.0, 0.0)
            return
        # 0/90/180/270 度，任意相邻两次视场有重叠；仅偏航，水平零速。
        desired = self._search_scan_yaw0 + self._search_scan_phase * math.pi / 2.0
        wx, wy = self.world_xy
        self._look_at = (wx + 10.0 * math.cos(desired),
                         wy + 10.0 * math.sin(desired))
        err = (desired - self.yaw + math.pi) % (2.0 * math.pi) - math.pi
        if abs(err) <= SEARCH_SCAN_YAW_TOL:
            if self._search_scan_settle_t is None:
                self._search_scan_settle_t = now
            elif now - self._search_scan_settle_t >= SEARCH_SCAN_HOLD_S:
                self._search_scan_phase += 1
                self._search_scan_settle_t = None
                if self._search_scan_phase >= 4:
                    self._search_scan_done = True
                    rospy.loginfo('[%s] 搜索格四向观察完成: %s',
                                  self.uav_id, self._search_scan_key)
        else:
            self._search_scan_settle_t = None
        self._send_vel(0.0, 0.0)

    def _control(self):
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
                # 2026-10-07 v12: 显式 vz 爬升（见 CLIMB_VZ 注释）——
                # 自适应目标高度可能被"贴楼降高 -0.5m"拖到 2.3m < 闸门 2.5m
                self._send_vel(0.0, 0.0, vz=CLIMB_VZ)
                return

        if self.world_xy is None:
            # === 2026-10-05 比赛规则硬约束：EKF 病态时不能一直悬停 ===
            # world_xy=None（EKF 漂移病态 / 偏移未标定）若持续 >1s，飞机将永久
            # 停滞不前，整轮任务失败。这里强制尝试重锚定 offset → world 立即跳回
            # 起飞点附近 → 下一帧世界坐标恢复正常 → 任务继续。
            _now = rospy.Time.now().to_sec()
            if (not hasattr(self, '_world_none_since') or
                    self._world_none_since is None):
                self._world_none_since = _now
            elif (_now - self._world_none_since) > 1.0 and self._offset_param is not None:
                # world_xy already reanchors to the last trusted world point.
                # Replacing that offset with the launch offset every second
                # undid recovery and trapped UAV1 in an x=150..155 loop.
                _lg = getattr(self, '_last_good_world', None)
                if _lg is not None and self.local_xy is not None:
                    self.offset = (_lg[0] - self.local_xy[0],
                                   _lg[1] - self.local_xy[1])
                    rospy.logwarn_throttle(
                        5.0, "[%s] world_xy 暂不可用 %.1fs，保持最后可信位置重锚定",
                        self.uav_id, _now - self._world_none_since)
                else:
                    self.offset = self._offset_param
                    rospy.logerr_throttle(
                        5.0, "[%s] world_xy 暂不可用 %.1fs，无可信历史位置，回起飞锚点",
                        self.uav_id, _now - self._world_none_since)
                self._ekf_window = None
                self._world_none_since = None
                # 重锚定后世界坐标恢复 → 不悬停，继续主流程
            self._send_vel(0.0, 0.0)
            return
        # 健康时清零 None 计时器
        self._world_none_since = None

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
                    tx0, ty0 = self._target_to_orbit
                    _d0 = math.hypot(tx0 - self.world_xy[0],
                                     ty0 - self.world_xy[1])
                    _stale_far = _d0 > STALE_ABORT_DIST_M
            if _in_cooldown or _stale_far:
                self._look_at = None
                self._send_vel(0.0, 0.0)
                return
            tx, ty = self._target_to_orbit
            self._look_at = (tx, ty)       # 接近阶段机头就对准目标
            dist = math.hypot(tx - self.world_xy[0], ty - self.world_xy[1])
            # Keep the close controller across small range-estimate changes.
            # The old exact 8 m switch ran A* pursuit whenever a walking
            # target moved just outside the ring, then rushed inside 6 m and
            # cropped the person out of the camera in the 10:54 match.
            _retain_close = (self._confirm_close and
                             self._confirm_close_tid == _track_id and
                             self._orbit_target == _track_id and
                             dist <= ORBIT_RADIUS_CLOSE + 1.5)
            if (self._confirm_close and self._confirm_close_tid == _track_id and
                    dist > ORBIT_RADIUS_CLOSE + 1.5 and
                    _now_s - self._t_seen.get(_track_id, 0.0) <= TARGET_TTL):
                self._confirm_close = False
                self._confirm_close_tid = None
                self._orbit_prev = None
                self._orbit_target = None
                self._confirm_start = 0.0
            # R2（2026-10-09）：world 不可信时禁止"到圈即贴脸"，改为悬停等待
            # 世界坐标自愈，防止用被污染的 world 判定假到达（v26 距真身 48m 贴脸）
            if dist < ORBIT_RADIUS and not self._world_trusted():
                rospy.logwarn_throttle(2.0,
                    '[%s] 到圈判定 dist=%.1fm 但 world 不可信（钳制/重锚定后 %.0fs 内）'
                    '→ 悬停等自愈，不切贴脸', self.uav_id, dist,
                    WORLD_TRUST_HOLD_S)
                self._send_vel(0.0, 0.0)
                return
            if dist < ORBIT_RADIUS or _retain_close:
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
                    self._confirm_start = rospy.Time.now().to_sec()
                    self._last_confirm_t = self._confirm_start
                    # 2026-10-07 五分钟冲刺: 到圈即贴脸, 不等 manager 团队确认 15s
                    if target_id:
                        self._enter_close_mode(target_id)
                # 不要清除 _target_to_orbit，_fly_orbit 需要用
                self._fly_orbit()
                return
            else:
                # 飞向目标（移动目标：A* 重规划节流）。异步投递，绝不阻塞指令流。
                # 2026-10-04：传入 tid，追逃（FLEE）时自动收紧重规划节流。
                if self._need_replan_track((tx, ty), tid=_track_id):
                    if rospy.Time.now().to_sec() - self._plan_fail_t >= 2.0:
                        self._request_plan((tx, ty))
                        self._last_track_goal = (tx, ty)
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
        # === 2026-10-05 国家一等奖：到点 dwell 期机头冻结到「下一个原本要飞到的搜索点」 ===
        # 原行为：_look_at = None → 交回 PX4 自管偏航 → 前视 114.6° 相机对到哪算哪。
        # 修复：DWELL_LOOKAHEAD=1 时复用 _pick_local_goal() 的 LOOKAHEAD 点，path
        #       清空时回退到 _last_local_goal（=到点前的下一个引导点）。
        if not DWELL_LOOKAHEAD:
            self._look_at = None

        # 搜索机遵守管理器的责任格。原先这里让所有距离任意视觉候选
        # 50m 内的搜索机私自改道，单个蓝色假目标曾吸走大半机队，
        # 导致真正未发现的目标活动区无人搜索。近距真实目标仍由
        # _update_orbit 接手；远距候选由 manager 按任务类型派追踪机。
        goal = (self.assignment.target_x, self.assignment.target_y)
        dist = math.hypot(goal[0] - self.world_xy[0], goal[1] - self.world_xy[1])

        if dist < SEARCH_ARRIVE_TOL:
            # 前视相机只覆盖一个扇区。抵达格心并不等于看过周围；原来 manager
            # 立即把半径 10m 的圆判成已搜索，蓝/白目标因此整局漏检。
            self.path = []
            self._check_start_orbit()
            if self._orbit_target is not None:
                self._fly_orbit()
            else:
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
        # 官方相机前视广角 114.6°（单相机），机头预摆可让到点瞬间就处于「即将
        # 进入下一个搜索格」的方向，与队友多点方案脱节问题修复。
        # 到点 dwell 段 path 会被 _check_start_orbit 清空 → _pick_local_goal
        # 返回 None → 上一句 self._look_at = self._last_local_goal 兜底冻结。
        if DWELL_LOOKAHEAD:
            # A forward camera sees only ±57°; a person beside the transit
            # path can be between two aircraft yet outside both views.  Sweep
            # the nose ±35° while keeping the same world-frame flight path.
            # Stop sweeping close to the waypoint so the four-direction scan
            # starts from a settled heading.  Radar and ORCA still constrain
            # the final velocity in _send_vel.
            if SEARCH_SWEEP_DEG > 0.0 and SEARCH_SWEEP_PERIOD_S > 0.0 \
                    and math.hypot(local_goal[0] - self.world_xy[0],
                                   local_goal[1] - self.world_xy[1]) > SEARCH_DECEL_M:
                _bearing = math.atan2(local_goal[1] - self.world_xy[1],
                                      local_goal[0] - self.world_xy[0])
                _sweep = math.radians(SEARCH_SWEEP_DEG) * math.sin(
                    2.0 * math.pi * rospy.Time.now().to_sec() / SEARCH_SWEEP_PERIOD_S)
                self._look_at = (self.world_xy[0] + 10.0 * math.cos(_bearing + _sweep),
                                 self.world_xy[1] + 10.0 * math.sin(_bearing + _sweep))
            else:
                self._look_at = local_goal
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
        """官方是否已首次有效播报该目标（/find_actor_N 曾发布 → actor 已武装可逃）。

        官方 control_actor 的 reported 由 /find_actor_N 首次发布置位（一次性闩锁），
        我方全员订阅该话题（_find_cb），故武装状态团队侧可见且一致。
        解析失败按已武装处理（保守）。"""
        try:
            idx = int(str(tid)[1:])
        except (IndexError, ValueError):
            return True
        return idx in self._find_t

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
        for tid, (tx, ty, vx, vy) in self.targets.items():
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

    # ---- 已消除目标清理 / 盘旋放弃 / 防扎堆（2026-09-27 新增）----
    def _left_actors_cb(self, msg):
        """官方 /left_actors：不在清单里的 actor = 已被官方确认并删除。

        这是「飞机被已消除目标占死」的权威兜底。official_target_bridge 只转发
        model_states，actor 被 del_model 后它就不再发该目标，而它发的
        TargetState.eliminated 恒为 False —— 于是 agent 的 self.targets 永不清理，
        _update_orbit 的「目标已消失」分支永远不触发，飞机对着一个已经不存在的
        actor 盘旋到超时（实测 uav_5/uav_6 对 t1 盘旋 571.3s，4/6 架机全程空转）。
        """
        ids = set(int(x) for x in re.findall(r'-?\d+', str(msg.data)))
        if not ids:
            return                      # 空清单不采信（可能只是还没发布 / 任务已结束）
        self._left_seen = True
        for tid in list(self.targets.keys()):
            m = re.match(r'^t(\d+)$', str(tid))
            if m is None:
                continue
            if int(m.group(1)) not in ids:
                self.targets.pop(tid, None)
                self._t_seen.pop(tid, None)
                self._giveup_until[tid] = float('inf')   # 已消除，永不追
                if self._orbit_target == tid:
                    self._abort_orbit('%s 已被官方消除，回归搜索' % tid)

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
        self._orbit_center = None
        self._confirm_start = 0.0
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
        # === 2026-10-03 我方补：先判"是不是根本够不着" ===
        # 官方裁判判据是"误差<1m 连续 15s"，而我方 yolo_target_bridge 加了
        # 距离闸门（默认 12m）：估计距离超出闸门时我们**故意不播报**，裁判自然
        # 收不到消息 → 15s 计时反复清零 → /find_actor_N 反复重置。
        # 这种"重置"不是"播报了但坐标错"，而是"还没飞到能看清的距离"。
        # 旧逻辑会据此判定播报被拒 → 退避 60s 去搜别处，目标永远丢；
        # 现在够不着就继续靠近（缩短 ORBIT 目标点、拉近观察距离），
        # 让距离闸门自然放开，而不是放弃目标。
        _wxy = self.world_xy
        _tgt = self._orbit_center or self._target_to_orbit
        if _wxy is not None and _tgt is not None:
            _d = math.hypot(_tgt[0] - _wxy[0], _tgt[1] - _wxy[1])
            if _d > CLOSE_ENOUGH_M:
                rospy.loginfo_throttle(
                    5.0,
                    '[%s] %s 确认被重置：距目标 %.1fm > %.1fm（还没飞到看得清的距离，'
                    '不放弃，继续靠近）',
                    self.uav_id, tid, _d, CLOSE_ENOUGH_M)
                self._reset_n[actor_idx] = 0      # 够不着不算"被拒绝"
                return
        if SPEND_ENABLE and self._reset_n[actor_idx] >= CONFIRM_RESET_MAX:
            # v23：官方逃跑闩锁是一次性的——反复重置说明「温顺接近」路线失败。
            # 主动花掉逃跑：全速冲进 20m 触发（一次性）逃跑 → 追至跑完 →
            # 目标变永久温顺（永不再逃，别机路过也不再惊逃）→ 全速贴脸消除。
            # 取代旧「退避 45s」：退避基于「逃跑可复位」的错误模型，且退避期间
            # 目标仍可能被任意友机路过惊逃，streak 永远立不起来。
            self._spend_mode.add(tid)
            self._abort_orbit('官方已重置 %d 次确认 → 花掉一次性逃跑'
                              '（全速触发+追至跑完+贴脸消除）'
                              % self._reset_n[actor_idx],
                              clear_path=False)
        elif BACKOFF_ENABLE and self._reset_n[actor_idx] >= CONFIRM_RESET_MAX:
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
        """当前盘旋目标是否已经没有位置更新（多半已被官方删除）。"""
        tid = self._orbit_target
        if tid is None:
            return False
        t_seen = self._t_seen.get(tid, 0.0)
        if not t_seen:
            return False
        return (rospy.Time.now().to_sec() - t_seen) > TARGET_STALE

    def _update_orbit(self):
        """更新盘旋状态：检查目标是否可见，更新确认时间

        国家一等奖标准改进（2026-10-03）：
        - 旧实现：任何 LOS 失败瞬时重置 _confirm_start → 1s 遮挡
          就能把累积 14s 的进度清零。修复：引入「LOS 宽容窗口」
          CONFIRM_LOS_GRACE（默认 2.0s），期间不重置累计。
        - 加入「视场对准」检查：相机水平 HFOV=90°（±45°），机头不
          对准目标（默认 yaw 没控制）时目标可能早出视场而仍被几何
          判定为可见。修复：检查 |target_bearing - yaw| ≤ 50°（略
          大于 HFOV/2 留余量），否则也不算确认。
        """
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
        # 检查是否能看到目标（距离 + LOS + 视场）
        in_radius = dist < DETECT_RADIUS
        in_los = in_radius and self.los.visible(wx[0], wx[1], tx, ty)
        in_fov = True
        if in_los:
            # 国家一等奖 BUG 修复（2026-10-03）：原代码写 self._yaw 但实际
            # _local_cb 把偏航角写入 self.yaw，结果 FOV 检查永远 None → 跳过
            # → 目标在飞机后方时仍报「已确认」→ 进度条虚假累计 → 裁判判定消除
            # 时坐标其实是被甩在身后的位置（实测 t3/t4 多机接力进度卡 30~40%）。
            # 修复：用 self.yaw；如未设置则降级为 True（避免 EKF 未就绪时直接卡死）
            # ---- 2026-10-03：再加强：yaw 未设置也不跳过 FOV 检查而是
            # 用 True（默认目标在视场中），保证队首起飞阶段的早期确认不被误杀；
            # yaw 就绪后仍走 50° 阈值判定。
            _yaw = getattr(self, 'yaw', None)
            if _yaw is not None:
                target_bearing = math.atan2(ty - wx[1], tx - wx[0])
                dy = target_bearing - _yaw
                # wrap 到 [-pi, pi]
                while dy > math.pi:
                    dy -= 2 * math.pi
                while dy < -math.pi:
                    dy += 2 * math.pi
                if abs(dy) > math.radians(50.0):
                    in_fov = False
        visible = in_radius and in_los and in_fov
        if visible:
            if self._confirm_start == 0.0:
                self._confirm_start = now
            self._last_confirm_t = now
            self._los_lost_t = 0.0
        else:
            # 国家一等奖：LOS 宽容窗口（2.0s），期间不重置累计进度
            if self._confirm_start != 0.0:
                if self._los_lost_t == 0.0:
                    self._los_lost_t = now
                elif now - self._los_lost_t > CONFIRM_LOS_GRACE:
                    # 超过宽容窗口才重置（避免被短时遮挡清零）
                    self._confirm_start = 0.0
                    self._los_lost_t = 0.0

    def _enter_close_mode(self, tid):
        """2026-10-07 五分钟冲刺: 统一的贴脸模式入口.

        到达 8m 观察圈即切稳定近距跟随，不再等 manager 团队确认 15s:
        - 贴脸稳态误差 <0.5m 天然满足官方判据 (err<1m AND 间隔≤1s),
          官方 15s 计时从贴脸首帧起算 → 每目标省 15s 串行等待.
        - 平稳过渡期由 CLOSE_TRANSIT_SPEED=0.95 限速；失联趋势另行追赶。
        - manager /swarm/confirmed 回调幂等 (已 True 不重复), 兜底仍在.
        """
        if self._confirm_close and self._confirm_close_tid == tid:
            return
        self._confirm_close = True
        self._confirm_close_tid = tid
        self._confirm_close_t0 = rospy.Time.now().to_sec()
        self._confirm_close_until = self._confirm_close_t0 + CONFIRM_LOCK_DURATION
        self._close_range_ref = None
        self._orbit_prev = None  # 清外推基线, 贴脸 angle 用当前位置重算
        rospy.loginfo(
            "[%s] 到达盘旋圈 → 直接切贴脸模式 (tid=%s) R=%.1fm/ω=%.3frad/s "
            "(线速度=%.3fm/s, 过渡cap=%.2fm/s), 锁定 %.0fs",
            self.uav_id, tid, ORBIT_RADIUS_CLOSE, ORBIT_SPEED_CLOSE,
            ORBIT_RADIUS_CLOSE * ORBIT_SPEED_CLOSE, CLOSE_TRANSIT_SPEED,
            CONFIRM_LOCK_DURATION,
        )

    def _check_start_orbit(self):
        """检查是否需要开始盘旋（发现可确认的目标）"""
        # R2（2026-10-09）：world 不可信（EKF 钳制/重锚定后）不开始新盘旋——
        # 检测距离判据基于被污染的 world_xy 会假到达/追错点
        if not self._world_trusted():
            return
        wx = self.world_xy
        if wx is None:
            return

        for tid, (tx, ty, _, _) in self.targets.items():
            if rospy.Time.now().to_sec() - self._t_seen.get(tid, 0.0) > TARGET_TTL:
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
            self._stale_search_until.pop(tid, None)  # v16: 全新宽限窗口
            self._confirm_start = rospy.Time.now().to_sec()
            self._last_confirm_t = self._confirm_start
            # 检测半径是 10m，稳定观察圈是 8m。10m 处就限 0.95m/s
            # 会使尚在逃跑的 actor 以更快速度甩开本机。
            if dist <= ORBIT_RADIUS:
                self._enter_close_mode(tid)
            rospy.loginfo("[%s] 开始盘旋确认目标 %s", self.uav_id, tid)
            break

    def _fly_orbit(self):
        """执行盘旋飞行：绕目标做圆周运动。

        国家一等奖标准改进（2026-10-03）：actor 持续 2m/s 移动时旧实
        现的圆心固定 → 飞机在轨道上 13s 跑一圈 actor 已跑 26m，
        远超 20m 视野。修复：用最近两次 actor 位置差分估算速度，
        把圆心提前 dt 秒外推到「飞机抵达时刻的 actor 位置」。
        """
        if self._orbit_target is None:
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

        # 目标长时间没有位置更新才判定已删除；短时遮挡要给 bridge/备份机接力窗口。
        # === v16 修复：到圈后 stale 宽限搜索 ===
        # v15 实测（agent_0 追 t1）：到圈瞬间 gap=56.6s 立即放弃——白飞 56s。
        # actor 静止时冻结位置即真实位置，首次 stale 不放弃，进入
        # STALE_SEARCH_GRACE 宽限窗口：继续用冻结位置盘旋（机头指向目标，
        # 视角变化可脱离楼体遮挡），YOLO 重捕获则 _t_seen 刷新自然退出本分支；
        # 窗口结束仍 stale 才放弃（原逻辑）。
        if self._orbit_stale():
            tid0 = self._orbit_target
            _now_s = rospy.Time.now().to_sec()
            gap = _now_s - self._t_seen.get(tid0, 0.0)
            _grace = self._stale_search_until.get(tid0, 0.0)
            if _grace == 0.0:
                self._stale_search_until[tid0] = _now_s + STALE_SEARCH_GRACE
                rospy.logwarn('[%s] 目标 %s 已 %.1fs 无更新 → %.0fs 宽限搜索'
                              '（actor 静止假设，盘旋重捕获）',
                              self.uav_id, tid0, gap, STALE_SEARCH_GRACE)
            elif _now_s >= _grace:
                self.targets.pop(tid0, None)
                self._giveup_until[tid0] = _now_s + GIVEUP_COOLDOWN
                self._stale_search_until.pop(tid0, None)
                # 保留路径：目标点未变，重算结果完全相同，纯浪费 CPU。
                self._abort_orbit('已 %.1fs 无位置更新，暂退避等待接力' % gap,
                                  clear_path=False)
                return
            # 宽限期内：targets 里仍有冻结位置，走正常盘旋流程

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
                # A normally walking actor can open range when the UAV is
                # braking or avoiding a wall.  Chasing that 1 m/s walk at
                # 2.6 m/s inside the 7 m push zone drives it to the boundary.
                # Reserve this burst for an actor whose 2 m/s escape has
                # actually been observed; the ordinary ring follower remains
                # below the escape trigger speed.
                _run_seen = self._flee_run_t.get(self._orbit_target)
                if (_run_seen is not None and now - _run_seen <= 3.0 and
                        self._orbit_target not in self._escape_spent and
                        close_target_is_receding(
                        orbit_dist, ref[2], now - ref[1], obs_age,
                        away_rate)):
                    self._close_chase_tid = self._orbit_target
                    self._close_chase_until = now + CLOSE_CHASE_HOLD_S
                    self._spend_mode.add(self._orbit_target)
                    rospy.logwarn_throttle(2.0,
                        '[%s] %s 近距目标远离 %.2fm/s（距离 %.1fm）→ 短时追赶',
                        self.uav_id, self._orbit_target,
                        (orbit_dist - ref[2]) / (now - ref[1]), orbit_dist)
                self._close_range_ref = (self._orbit_target, now, orbit_dist,
                                         tx, ty, wx, wy)
        chase_close = (self._confirm_close and
                       getattr(self, '_close_chase_tid', None) == self._orbit_target and
                       now < getattr(self, '_close_chase_until', 0.0) and
                       obs_age <= 1.0 and orbit_dist > 6.0)
        # The close cap is below a walking actor's nominal 1m/s speed.  If
        # that actor walks away, a 30s close-mode latch makes reacquisition
        # mathematically impossible.  Let the normal one-shot escape policy
        # decide whether faster pursuit is warranted once contact opens up.
        if (self._confirm_close and orbit_dist > ORBIT_RADIUS_CLOSE + 1.5 and
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
                orbit_dist <= ORBIT_RADIUS and
                self.los.visible(wx, wy, tx, ty)):
            self._enter_close_mode(self._orbit_target)

        # 国家一等奖：actor 速度估计 + 圆心外推到飞机飞行 dt 后
        prev = getattr(self, "_orbit_prev", None)
        if prev is None or prev[0] != self._orbit_target:
            self._orbit_prev = (self._orbit_target, tx, ty, now)
            prev_vx_t, prev_vy_t = 0.0, 0.0
        else:
            _tid, _px, _py, _pt = prev
            _dt_t = max(now - _pt, 0.05)
            prev_vx_t = (tx - _px) / _dt_t
            prev_vy_t = (ty - _py) / _dt_t
            _sp = math.hypot(prev_vx_t, prev_vy_t)
            if _sp > 2.5:
                prev_vx_t *= 2.5 / _sp
                prev_vy_t *= 2.5 / _sp
            self._orbit_prev = (self._orbit_target, tx, ty, now)
        # 计算当前相对于目标的角度（用外推后位置）
        dt = 1.0 / CTRL_RATE
        tx_c = tx + prev_vx_t * dt
        ty_c = ty + prev_vy_t * dt
        angle = math.atan2(wy - ty_c, wx - tx_c)

        xmin, ymin = self.grid.origin
        xmax = xmin + self.grid.width * self.grid.resolution
        ymax = ymin + self.grid.height * self.grid.resolution
        _ring_bounds = (xmin, xmax, ymin, ymax)

        # 更新角度（顺时针盘旋）
        # 近距模式维持 8m 观察距离：裁判判的是上报世界坐标精度，
        # 不要求无人机压到演员头顶；压进 7m 会触发演员驱离，反而断流。
        if self._confirm_close:
            # 小角速度让相机稳定对准演员，减少图像框和测距抖动。
            _radius = ORBIT_RADIUS_CLOSE
            _speed  = ORBIT_SPEED_CLOSE
            # 这里的 angle 只决定观察方向；移动目标的平移由速度前馈负责。
            angle = math.atan2(wy - ty_c, wx - tx_c)
            target_x = tx_c + _radius * math.cos(angle)
            target_y = ty_c + _radius * math.sin(angle)
            if not (xmin + MAP_GUARD_MARGIN <= target_x <= xmax - MAP_GUARD_MARGIN and
                    ymin + MAP_GUARD_MARGIN <= target_y <= ymax - MAP_GUARD_MARGIN):
                target_x, target_y = observation_ring_goal(
                    wx, wy, tx_c, ty_c, _radius,
                    _ring_bounds, MAP_GUARD_MARGIN + 0.5)
            # 平滑近距目标点，防止切换时位置指令突跳。
            target_x = wx + 0.3 * (target_x - wx)
            target_y = wy + 0.3 * (target_y - wy)
        else:
            _radius = ORBIT_RADIUS
            _speed  = ORBIT_SPEED
            angle += _speed * dt
            if angle > math.pi:
                angle -= 2 * math.pi
            # 目标位置（用外推后圆心，确保飞机真的"绕到 actor 未来位置上"）
            target_x = tx_c + _radius * math.cos(angle)
            target_y = ty_c + _radius * math.sin(angle)
            if not (xmin + MAP_GUARD_MARGIN <= target_x <= xmax - MAP_GUARD_MARGIN and
                    ymin + MAP_GUARD_MARGIN <= target_y <= ymax - MAP_GUARD_MARGIN):
                target_x, target_y = observation_ring_goal(
                    wx, wy, tx_c, ty_c, _radius,
                    _ring_bounds, MAP_GUARD_MARGIN + 0.5)

        # P 控制飞向盘旋点
        err_x = target_x - wx
        err_y = target_y - wy
        vx = POS_KP * err_x
        vy = POS_KP * err_y
        # 近距确认时，目标仍会以约 1 m/s 行走。仅靠经过 0.3 缩放的
        # 位置误差产生约 0.2~0.5 m/s，飞机必然逐渐落后并丢失画面。
        # 使用桥接轨迹的速度作前馈，再统一限到逃跑触发线以下；观测
        # 过期时停用前馈，避免沿冻结速度继续追鬼影。
        _urgent_backoff = False
        if self._confirm_close and not chase_close:
            _tvx, _tvy = (self.targets[self._orbit_target][2:4]
                          if self._orbit_target in self.targets else (0.0, 0.0))
            vx, vy = close_follow_velocity(
                err_x, err_y, _tvx, _tvy,
                now - self._t_seen.get(self._orbit_target, 0.0))
            vx, vy = ring_standoff_velocity(
                vx, vy, wx, wy, tx, ty, ORBIT_RADIUS_CLOSE,
                _tvx if now - self._t_seen.get(self._orbit_target, 0.0) <= TARGET_TTL else 0.0,
                _tvy if now - self._t_seen.get(self._orbit_target, 0.0) <= TARGET_TTL else 0.0)
            vx, vy, _urgent_backoff = urgent_close_backoff_velocity(
                vx, vy, wx, wy, tx, ty, _tvx, _tvy)
        elif chase_close:
            # The ring point is behind a retreating actor.  Follow the actual
            # line of sight until the range closes; all radar, map and friend
            # guards still run through _send_vel below.
            vx = FLEE_SHEPHERD_SPEED * (tx - wx) / orbit_dist
            vy = FLEE_SHEPHERD_SPEED * (ty - wy) / orbit_dist
        spd = math.hypot(vx, vy)
        # 近距稳定观察时速度上限 0.95 m/s；离圈后由追赶策略重新定速。
        # 严格低于官方逃跑触发阈值 1.0 m/s (机速>1.0 且 <20m 持续 2s → actor 逃跑).
        _v_cap = (FLEE_SHEPHERD_SPEED if chase_close else
                  CLOSE_BACKOFF_SPEED if _urgent_backoff else
                  CLOSE_TRANSIT_SPEED if self._confirm_close else
                  self._approach_cap(self._orbit_target, orbit_dist))
        if spd > _v_cap:
            vx *= _v_cap / spd
            vy *= _v_cap / spd

        # 友机避碰
        vx, vy = self._apply_friend_avoidance(vx, vy)

        self._send_vel(vx, vy)

        # 检查确认时间
        # ⚠ v22-A 回滚（2026-10-09 04:52）：加 _confirm_start>0 防御后，目标不可见时
        # confirm_duration=0 → 永不触发放弃 → 飞机卡死盘旋态（v22 仿真实证：6 机全卡、
        # manager 70s 后 0 分配、score 464.9→44.9 崩溃）。v21 的"sim 时间戳误判"实际是
        # 逃生阀：不可见时快速放弃回搜索池。保留原逻辑，不可见退出的正确修法是
        # stale→宽限→放弃链路（v16 已有），不动这里。
        confirm_duration = now - self._confirm_start
        # 2026-10-06 国家一等奖: 贴脸锁定时长到期检查
        if self._confirm_close and now >= self._confirm_close_until:
            self._confirm_close = False
            self._confirm_close_tid = None
            self._orbit_prev = None
            rospy.loginfo("[%s] 贴脸锁定 %.0fs 到期, 退出贴脸模式",
                          self.uav_id, CONFIRM_LOCK_DURATION)
        self._publish_claim()

        if confirm_duration >= CONFIRM_TIME:
            rospy.loginfo_throttle(
                5.0, "[%s] 目标 %s 驻留观察 %.1fs，等待官方消除反馈",
                self.uav_id, self._orbit_target, confirm_duration)
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
            if 0.5 < dist and math.hypot(dist, my_alt - friend_alt) < SAFE_3D:
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
                    # 半平面要求 v·n 为正，n 必须指向远离友机的一侧。
                    nx, ny = -px / dist, -py / dist
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
        if self.world_xy is None:
            return

        st = UavStatus()
        st.header.stamp = rospy.Time.now()
        st.uav_id = self.uav_id
        st.x = self.world_xy[0]
        st.y = self.world_xy[1]
        st.z = self.local_z if self.local_z is not None else 0.0
        # 飞控链路仍连接不代表此机可执行任务。EKF 熔断或坠机恢复期间
        # world_xy 可能严重漂移；继续向 manager 报可用会占住搜索格，
        # 甚至把从未看过的区域标成已覆盖。
        st.connected = bool(self.state.connected and
                            not getattr(self, '_ekf_frozen', False) and
                            not getattr(self, '_crash_flagged', False))

        # 用实际位置计算所在格子（而非被分配的格子）- 使用 10m 覆盖栅格
        # 覆盖判定由 manager 端用感知半径批量处理，agent 只需上报位置
        actual_cell = self.cov_grid.world_to_cell(self.world_xy[0], self.world_xy[1])
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
