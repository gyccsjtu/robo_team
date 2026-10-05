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
from swarm_task import LineOfSight, DETECT_RADIUS, CoverageGrid, GRID_SIZE_M
from csv_logger import logger
# 2026-10-03：飞控参数读回校验（取自队友 codex/radar-coordination-20261003 分支）
from fcu_configuration import configure as configure_fcu_parameters

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
ALT_HARD_CEIL     = float(os.environ.get('ALT_HARD_CEIL', '5.7'))   # 规则 §2.5(3)：飞行高度 ≤6m；硬护栏 5.7m 留 0.3m 抖动空间
ALT_PANIC         = float(os.environ.get('ALT_PANIC', '5.85'))       # 强制快降阈值 m（距 6m 留 0.15m）
ALT_PANIC_DESCENT = float(os.environ.get('ALT_PANIC_DESCENT', '-1.5'))  # 快降速度 m/s
ALT_PANIC_HSCALE  = float(os.environ.get('ALT_PANIC_HSCALE', '0.3'))    # 此时水平速度系数
MAX_ACC           = float(os.environ.get('MAX_ACC', '2.5'))         # 水平加速度限幅 m/s^2
ALT_HARD_DESCENT  = -1.0   # 强制下降速度 m/s（原 -0.6 下降太慢）
# === 6m 红线二次保险 ===
ALT_EMERG_CEIL     = float(os.environ.get('ALT_EMERG_CEIL', '5.92'))     # 规则 §2.5(3) 二次保险阈值（距 6m 留 0.08m）
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
PUB_RATE        = 10.0    # 状态发布频率 Hz
CTRL_RATE       = 20.0    # 控制频率 Hz
DETECT_RATE     = float(os.environ.get('DETECT_RATE', '3.0'))   # 规则 §2.5(10)：连续 15s 正确广播 → 3Hz × 15s = 45 个采样，远高于裁判认定的连续窗口
# 幽灵目标闸门：/swarm/target_states 停发（桥接 DROP_TIME）超过此时长后，
# 停止对该目标的几何检测上报。与 manager TRUTH_TTL 对齐——否则本字典里的
# 过期坐标会被几何检测持续上报，manager _detection_cb 当新检测反复派机
# （2026-10-01 22:41 局实测：t3 幽灵吸住 4 架 3 分钟，确认进度反复归零）。
# 国家一等奖优化（2026-10-03）：与 yolo_target_bridge DROP_TIME=8s 对齐，
# 防止 bridge 已停发但 agent 仍上报过期坐标的窗口期。
TARGET_TTL      = float(os.environ.get("TARGET_TTL", "8.0"))

# 2D 雷达安全层：由 swarm_agent 唯一发布 MAVROS 速度设定点，避免与
# radar_avoid 的 setpoint_position 双控制。雷达只在近障时修正当前速度。
RADAR_GUARD     = int(os.environ.get('RADAR_GUARD', '1'))
RADAR_FRESH_S   = float(os.environ.get('RADAR_FRESH_S', '0.5'))
RADAR_WARN_R    = float(os.environ.get('RADAR_WARN_R', '5.5'))   # 规则 §2.5(7)：碰撞扣30/次，雷达预警半径扩大到 5.5m（车体级别障碍），留 1.5m 减速带宽
RADAR_STOP_R    = float(os.environ.get('RADAR_STOP_R', '1.6'))   # 硬停距 ≥ 1.6m，确保横向漂移不会擦肩
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
RADAR_BACKOFF_R   = float(os.environ.get('RADAR_BACKOFF_R', '1.2'))   # 触发后退的 front m（= STOP_R）
RADAR_BACKOFF_SPD = float(os.environ.get('RADAR_BACKOFF_SPD', '0.6')) # 后退速度 m/s
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
# 修复：ORBIT_RADIUS=6.0（仍 < 7m 安全半径）+ ORBIT_SPEED=0.13 rad/s → 线速度 = 0.13×6
#   = 0.78 m/s，离触发线 22% 安全余量，足够吸收 EKF 抖动。
#   同时 r=6m 仍在 20m 视野中心偏内，相机（FOV 90°）始终覆盖目标。
ORBIT_RADIUS    = 6.0     # 盘旋半径 m（< actor uav_safety_radius=7.0 不被推）
# 线速度 = ORBIT_SPEED * ORBIT_RADIUS，必须严格 < 1.0 m/s（防触发逃跑判定）。
ORBIT_SPEED     = 0.13    # 盘旋角速度 rad/s（r=6m 时线速度 0.78 m/s，离 1.0 阈值 22% 安全余量）
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
# 目标已进入 FLEE（官方：UAV 广播其位置后 actor 以 2 m/s 逃跑），慢速 0.5 必然被甩开跟丢。
# 此时短暂提速咬住（须略 >2.0）；目标不在逃跑时仍用 SPOOK_SPEED 防惊吓。
# 国家一等奖标准修复（2026-10-04）：FLEE_CHASE_SPEED=3.5，给 SPOOK_SPEED=0.5 的亏空补回，
# 3.5 = actor 2 m/s × 1.75 倍 + DWA 安全留量，足以咬住又不会破坏 SP。
FLEE_CHASE_SPEED  = float(os.environ.get('FLEE_CHASE_SPEED', '3.5'))
# 国家一等奖标准修复（2026-10-04）：bridge 已把 state 语义对齐「被观测即逃跑」，
# FLEE_STATE_FRESH 不应只 3s（3s 后就回到 SPOOK_SPEED=0.5 慢速又被甩开）。
# 规则 §2.5(4)：30s 未消除才瞬移；这之前 actor 一直在 2 m/s 跑。给到 25s 留 5s 余量。
FLEE_STATE_FRESH  = float(os.environ.get('FLEE_STATE_FRESH', '25.0'))
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
INFLATE_M       = 1.5     # A* 障碍膨胀半径 m（覆盖桨尖0.37 + 建筑偏大0.45 + 切角/超调0.68）
LOOKAHEAD       = 3.0     # 路径跟踪前瞻距离 m（> 刹停距离 v²/2a=2.25m）
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
        self.yaw = 0.0              # 机体 yaw（ENU 弧度，供雷达 body->world）
        self._scan = None            # 最近一帧 2D LaserScan
        self._scan_t = 0.0           # 最近雷达帧的 ROS 时间
        self.state = State()
        self.assignment = None      # SearchAssignment 当前任务

        # ---- A* 避障 ----
        self.md, _ = load_metadata(METADATA_PATH)
        self.grid = inflate_grid(GridMap.from_metadata(self.md), INFLATE_M)
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

        # ---- 覆盖栅格（与 manager 一致，10m 格）----
        self.cov_grid = CoverageGrid(MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX, GRID_SIZE_M)

        # ---- 目标检测（规则3 几何判定：距离 + 视线遮挡）----
        # 用**未膨胀**的原始栅格做 LOS 判定（膨胀是给飞行留裕度的，
        # 判定遮挡要用真实建筑轮廓，否则会把建筑边缘 0.5m 内误判为遮挡）。
        raw_grid = GridMap.from_metadata(self.md)
        self.los = LineOfSight(
            lambda ix, iy: (not raw_grid.in_bounds((ix, iy)))
                           or (not raw_grid.is_free((ix, iy))),
            cell_size=raw_grid.resolution, origin=raw_grid.origin)
        self.targets = {}           # target_id -> (x, y, vx, vy)，来自 /swarm/target_states
        self._target_state = {}    # target_id -> (state, t)，目标运动状态（1=FLEE）
        self._detect_log_t = {}     # tid -> 上次 [ALGO] detect 日志时刻（按 target 分别节流）
        self._last_detect_t = 0.0

        # ---- 目标盘旋确认 ----
        self._orbit_target = None    # 当前盘旋目标 ID
        self._last_track_goal = None # 追踪时上次 A* 的目标点（用于节流）
        self._last_track_plan_t = 0.0  # 追踪时上次 A* 规划时刻
        self._plan_fallback_n = 0  # A* 失败退化直飞的次数
        self._orbit_center = None    # 盘旋中心 (x, y)
        self._confirm_start = 0.0   # 连续确认开始时间
        self._last_confirm_t = 0.0   # 上次确认时间
        self._target_to_orbit = None  # 待盘旋目标位置 (x, y)
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
        rospy.Subscriber("/swarm/uav_status", UavStatus, self._friend_status_cb)

        # ---- 发布 ----
        self.status_pub = rospy.Publisher("/swarm/uav_status", UavStatus, queue_size=5)
        self.detect_pub = rospy.Publisher("/swarm/detection", TargetDetection, queue_size=10)
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
        # 地图硬边界外 50% 容差；超出视为 EKF 病态 → 拒用
        _hard_xmin = MAP_X_MIN * 1.5
        _hard_xmax = MAP_X_MAX * 1.5
        _hard_ymin = MAP_Y_MIN * 1.5
        _hard_ymax = MAP_Y_MAX * 1.5
        if not (_hard_xmin <= wx <= _hard_xmax and _hard_ymin <= wy <= _hard_ymax):
            # 仅在错位刚发生时打一次，避免每秒刷屏
            _now = rospy.Time.now().to_sec()
            if not hasattr(self, '_world_bad_last_log') or (_now - self._world_bad_last_log) > 2.0:
                self._world_bad_last_log = _now
                rospy.logerr_throttle(2.0,
                                      "[%s] world_xy=(%.1f,%.1f) 病态超界，拒用并触发重锚定",
                                      self.uav_id, wx, wy)
                # 病态时把 offset 强制重锚定为「起飞点世界坐标」→ world 立即回到起飞点附近
                # 这样下一帧 _local_cb 重新计算 world 时不会延续病态
                if self._offset_param is not None:
                    self.offset = self._offset_param
                    self._ekf_window = None
            return None
        return (wx, wy)

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
                    # 拼接：(prev_xy, prev_t) 队列；dt 累计不超过 1s
                    while _past and (now - _past[0][1]) > 1.0:
                        _past.pop(0)
                    _past.append((new_xy, now))
                    self._ekf_window = _past
                    if len(_past) >= 2:
                        first_xy, first_t = _past[0]
                        cum_disp = math.hypot(new_xy[0] - first_xy[0],
                                              new_xy[1] - first_xy[1])
                        cum_dt = now - first_t
                        if cum_dt > 0.05 and cum_disp > MAX_SPEED * cum_dt * 1.2:
                            cum_th_hit = True
                else:
                    self._ekf_window = [(new_xy, now)]

                if jump > single_th or cum_th_hit:
                    wx = self.local_xy[0] + self.offset[0]
                    wy = self.local_xy[1] + self.offset[1]
                    new_off = (wx - new_xy[0], wy - new_xy[1])
                    rospy.logwarn("[%s] EKF 原点跳变 %.2fm（dt=%.3fs，单帧阈值=%.2fm；累积 %.2fm/%.2fs）"
                                  "，offset 重锚定 (%.2f,%.2f)→(%.2f,%.2f)，world 保持 (%.2f,%.2f)",
                                  self.uav_id, jump, dt, single_th, cum_disp, cum_dt,
                                  self.offset[0], self.offset[1],
                                  new_off[0], new_off[1], wx, wy)
                    self.offset = new_off
                    # 重置累积窗口：避免同一次跳变被连续多帧反复触发
                    self._ekf_window = [(new_xy, now)]
        # 维护位置估计轨迹（EST_GUARD 闸门的数据源；必须在 anchor 闸门之外每帧调用，
        # 否则 _est_hist 恒为空 → _est_trustworthy 恒 False → 飞机永远悬停不走）
        self._est_track(new_xy, now)
        self.local_xy = new_xy
        self._local_prev_t = now
        q = msg.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
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
        self._scan_t = rospy.Time.now().to_sec()

    def _assign_cb(self, msg):
        if msg.uav_id != self.uav_id:
            return
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
            # 同步解除追踪指派去重：该目标已不存在
            if getattr(self, '_track_assigned_id', None) == msg.target_id:
                self._track_assigned_id = None
                self._target_to_orbit = None
            return
        self.targets[msg.target_id] = (msg.x, msg.y, msg.vx, msg.vy)
        self._target_state[msg.target_id] = (
            int(msg.state), rospy.Time.now().to_sec())
        self._t_seen[msg.target_id] = rospy.Time.now().to_sec()

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
        - 追踪目标时：-1.0m（更好地观察）
        - 建筑附近：-0.5m（保持距离）
        - 开阔区域：0m（常规搜索）
        """
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
        """用 2D 雷达给当前 ENU 速度加一层近障安全约束。

        雷达角度在机体系内，0 弧度为机头方向。这里只改水平速度，
        不接管 OFFBOARD，也不另发 MAVROS 设定点。
        """
        if not RADAR_GUARD or self._scan is None:
            return vx, vy
        now = rospy.Time.now().to_sec()
        if now - self._scan_t > RADAR_FRESH_S:
            return vx, vy
        speed = math.hypot(vx, vy)
        if speed < 0.05:
            return vx, vy

        scan = self._scan
        front, left, right = scan.range_max, scan.range_max, scan.range_max
        for i, raw in enumerate(scan.ranges):
            r = float(raw)
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            deg = math.degrees(angle)
            if -30.0 <= deg <= 30.0:
                front = min(front, r)
            elif 30.0 < deg <= 100.0:
                left = min(left, r)
            elif -100.0 <= deg < -30.0:
                right = min(right, r)

        # 最近净空（三扇区最小值）→ 硬刹停速度上限 v <= sqrt(2*a*(d-安全间隙))
        d_min = min(front, left, right)
        v_cap = math.sqrt(max(0.0, 2.0 * MAX_ACC *
                              max(0.0, d_min - RADAR_SAFE_GAP)))
        if front >= RADAR_WARN_R:
            if speed > v_cap and speed > 1e-9:
                return vx * (v_cap / speed), vy * (v_cap / speed)
            return vx, vy

        # 优先选择净空更大的侧面；正前方两侧都未知时固定向左，避免左右抖动。
        side = 1.0 if left >= right else -1.0
        if abs(left - right) < 0.25:
            side = 1.0

        # ---- 国家一等奖修复（2026-10-04）：三面近障兜底后退 ----
        # 真实仿真日志（logs_20261004_013325）实锤：UAV_0 以 front=0.31 / left=0.30 /
        # right=0.32 三面全堵的姿态，被旧逻辑输出 vout=(0.07,1.59)「向左横移」，
        # 直接顶进 0.30m 外的左墙，位置 40s+ 纹丝不动（DWA 死锁）。
        # 根因：旧逻辑只比较 left/right 谁大，没有检查「两侧是否都堵死」。
        # 修复：两侧净空都低于 SIDE_BLOCK_R 时，唯一可行方向是后退——沿 -vin
        # （远离障碍）方向退，退到 front 恢复到 RADAR_WARN_R 之前一直保持后退。
        SIDE_BLOCK_R = 0.6   # 侧向净空低于此值视为侧墙堵死（m）
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
                    deg = math.degrees(scan.angle_min + i * scan.angle_increment)
                    if d0 <= deg < d0 + SECTOR and r < sec_min:
                        sec_min = r
                if sec_min > best_r:
                    best_r = sec_min
                    best_deg = d0 + SECTOR * 0.5
                d0 += SECTOR * 0.5
            if best_deg > 100.0:      # 全盲（无有效回波）：保底正后方不可选，退向左前
                best_deg, best_r = -60.0, 0.0
            retreat = max(speed, 1.0) if best_r > 0.8 else 0.8   # 净空差时慢退
            ang = math.radians(best_deg)                          # 机体系
            body_x = retreat * math.cos(ang)
            body_y = retreat * math.sin(ang)
            cy, sy = math.cos(self.yaw), math.sin(self.yaw)
            guarded = (cy * body_x - sy * body_y,
                       sy * body_x + cy * body_y)
            rospy.logwarn_throttle(2.0,
                                   '[%s] 2D雷达三面堵死 front=%.2f left=%.2f right=%.2f '
                                   '→ 扇区%.0f°净空%.2fm 后退 vout=(%.2f,%.2f)',
                                   self.uav_id, front, left, right,
                                   best_deg, best_r,
                                   guarded[0], guarded[1])
            return guarded

        forward = max(0.0, min(speed * 0.35,
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
        rospy.logwarn_throttle(2.0,
                               '[%s] 2D雷达近障 front=%.2fm left=%.2f right=%.2f '
                               '-> vin=(%.2f,%.2f) vout=(%.2f,%.2f)',
                               self.uav_id, front, left, right,
                               vx, vy, guarded[0], guarded[1])
        return guarded

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
        """
        if not GRID_GUARD:
            return vx, vy
        wx = self.world_xy
        if wx is None:
            return vx, vy
        spd = math.hypot(vx, vy)
        if spd < 0.05:
            return vx, vy
        # 高度豁免：起飞爬升期（z < SAFE_ALT）完全绕过栅格近障
        z_now = self.local_z
        if z_now is not None and z_now < SAFE_ALT:
            return vx, vy
        # 半豁免区（SAFE_ALT ~ ALT_CEILING）：衰减前瞻距离，减少高空误判
        guard_scale = 1.0
        if z_now is not None and ALT_CEILING > SAFE_ALT:
            guard_scale = max(0.0, min(1.0, (ALT_CEILING - z_now) / max(1e-6, ALT_CEILING - SAFE_ALT)))
        hd = math.atan2(vy, vx)
        # 刹停距离 v²/(2a) + 裕度；至少看 0.8m。裕度加大防建筑偏大/超调。
        # 半豁免区衰减前瞻：guard_scale<1 时缩短探测距离，允许飞机更快穿过近障区
        look = max(0.8, spd * spd / (2.0 * MAX_ACC) + 1.5) * guard_scale
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
        # === 位置估计可信性闸门（EST_GUARD，2026-10-03，最高优先级）===
        # 只在自动飞行路径生效（vz is None；预热/降落都显式传 vz，不受影响）。
        # 估计不可信时绝不做任何基于 world_xy 的决策 —— 实测「越界主动回收 →
        # (0,3) 当前(63.2,-66.1)」是在追一个 100m 外的幻影，越追越远。
        if vz is None and not self._est_trustworthy():
            self._est_hold_n = getattr(self, '_est_hold_n', 0) + 1
            _cap = min(ALT_TARGET_CAP, ALT_HARD_CEIL - ALT_HARD_MARGIN)
            _base = self.altitude_layer or ALT_BASE
            _alt = max(MIN_CRUISE_ALT, min(_cap, _base))
            rospy.logwarn_throttle(2.0,
                '[%s] 位置估计不可信（近 %.2fs 平均 %.1f m/s > %.1f m/s）→ 悬停'
                '等待 EKF 收敛（累计 %d 拍）', self.uav_id, EST_WIN_S,
                (self._est_recent_speed() or -1.0), MAX_SPEED * EST_V_RATIO,
                self._est_hold_n)
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
            vx, vy = self._radar_guard_velocity(vx, vy)
            # 无激光时栅格兜底（雷达在线也再过一道，双保险）
            vx, vy = self._grid_guard_velocity(vx, vy)
            vx, vy = self._map_guard_velocity(vx, vy)

        # === 2026-09-27：水平加速度限幅 ===
        # 避让增益提高后，ORCA 输出可能在相邻帧跳到近乎反向（22:29 轮事故：
        # iris_3 因此把高度超调从 0.2m 放大到 1.5m，冲过官方 6m 红线，score=0）。
        # 这里把「指令加速度」钉死，PX4 位置环才有能力跟踪，机身不再大幅倾斜。
        if not hasattr(self, "_last_cmd_v"):
            self._last_cmd_v = None
        if self._last_cmd_v is not None:
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
        if FLIGHT_OUTPUT == "pos":
            self._publish_pos_output(vx, vy, target_alt, vz)
        else:
            self.vel_pub.publish(cmd)

    def _publish_pos_output(self, vx, vy, target_alt, vz=None):
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
            m.twist.linear.z = 0.0
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
        # 高度：降落段必须真降，否则位置模式会一直保持 target_alt 悬停；
        # 其余情况用 target_alt（官方 >6m 判 0，这里目标 2.8m，天然安全）。
        if self._landing:
            z = max(0.0, self.local_z - 0.6)
        elif target_alt is not None:
            z = target_alt
        else:
            z = self.local_z
        sp.pose.position.z = z
        sp.pose.orientation.w = 1.0
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
                self._send_vel(0.0, 0.0)
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
                rospy.logerr_throttle(5.0,
                    "[%s] world_xy 病态持续 %.1fs，强制重锚定 offset 到起飞点",
                    self.uav_id, _now - self._world_none_since)
                self.offset = self._offset_param
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
            self._look_at = None
            self._send_vel(0.0, 0.0)
            return

        # === 任务类型处理 ===
        task_type = getattr(self.assignment, 'task_type', 0)

        # 目标追踪任务（task_type=1）：飞向目标位置
        if task_type == 1 and self._target_to_orbit is not None:
            # 冷却闸门：目标在 giveup 冷却内，或已 stale（无位置更新），
            # 安全悬停等待 manager 改派，绝不重入盘旋或朝过期点飞。
            _track_id = getattr(self.assignment, 'target_id', None)
            _in_cooldown = (_track_id and
                            rospy.Time.now().to_sec() < self._giveup_until.get(_track_id, 0.0))
            if _in_cooldown or self._orbit_stale():
                self._look_at = None
                self._send_vel(0.0, 0.0)
                return
            tx, ty = self._target_to_orbit
            self._look_at = (tx, ty)       # 接近阶段机头就对准目标
            dist = math.hypot(tx - self.world_xy[0], ty - self.world_xy[1])
            if dist < ORBIT_RADIUS:
                # 到达目标附近，开始盘旋
                self._orbit_center = (tx, ty)
                # 存真实 target_id
                target_id = getattr(self.assignment, 'target_id', None)
                self._orbit_target = target_id if target_id else "tracking"
                self._confirm_start = rospy.Time.now().to_sec()
                self._last_confirm_t = self._confirm_start
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
                # 接近 actor 时压速，避免触发官方逃跑机制（详见 SPOOK_SPEED 注释）。
                # 目标已逃跑则提速咬住，否则 0.8m/s 追 2.0m/s 必然跟丢。
                if self._target_fleeing(_track_id):
                    cap = FLEE_CHASE_SPEED
                else:
                    cap = SPOOK_SPEED if dist < SPOOK_DIST else MAX_SPEED
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

        # === 2026-10-03：搜索时主动追近距目标 ===
        # 若 target_states 已知 50m 内有目标，覆盖 manager 的搜索格目标，直接飞过去。
        # 否则单纯靠搜索格中心 80m 外派遣（manager 阈值改 80m 后），飞机仍要飞 30s
        # 才到 actor 区域，且不能保证任何一架恰好分到 actor 所在格。
        # 这里兜底：任意目标 ≤50m 且目标 alive 即把 goal 替换为目标位置，
        # 让 agent 主动接近 → 进入 20m 探测范围。
        _ntd = self._nearest_target_dist()
        _target_pursuit_m = 50.0
        _override_goal = None
        _override_dist = None
        if (_ntd is not None and _ntd[1] <= _target_pursuit_m
                and self._target_state.get(_ntd[0], (0, 0.0))[0] != 2):
            _tx, _ty, _vx_t, _vy_t = self.targets[_ntd[0]]
            _override_goal = (_tx, _ty)
            _override_dist = _ntd[1]

        if _override_goal is not None:
            goal = _override_goal
            dist = _override_dist
        else:
            goal = (self.assignment.target_x, self.assignment.target_y)
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
        err_x = local_goal[0] - self.world_xy[0]
        err_y = local_goal[1] - self.world_xy[1]
        vx = POS_KP * err_x
        vy = POS_KP * err_y
        spd = math.hypot(vx, vy)
        if spd > MAX_SPEED:
            vx *= MAX_SPEED / spd
            vy *= MAX_SPEED / spd
        # 已知目标在附近（即使 manager 尚未下发追踪指派）→ 提前压速，
        # 避免以搜索速度冲进 20m 触发演员逃跑（实测演员被惊到 3.87m/s）。
        # 例外：该目标已在逃跑 -> 提速咬住，避免慢速被甩开跟丢。
        _ntd = self._nearest_target_dist()
        if _ntd is not None and _ntd[1] < SPOOK_DIST:
            _cap = (FLEE_CHASE_SPEED if self._target_fleeing(_ntd[0])
                    else SPOOK_SPEED)
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

    def _check_start_orbit(self):
        """检查是否需要开始盘旋（发现可确认的目标）"""
        wx = self.world_xy
        if wx is None:
            return

        for tid, (tx, ty, _, _) in self.targets.items():
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
        """执行盘旋飞行：绕目标做圆周运动。

        国家一等奖标准改进（2026-10-03）：actor 持续 2m/s 移动时旧实
        现的圆心固定 → 飞机在轨道上 13s 跑一圈 actor 已跑 26m，
        远超 20m 视野。修复：用最近两次 actor 位置差分估算速度，
        把圆心提前 dt 秒外推到「飞机抵达时刻的 actor 位置」。
        """
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

        # 更新角度（顺时针盘旋）
        angle += ORBIT_SPEED * dt
        if angle > math.pi:
            angle -= 2 * math.pi

        # 目标位置（用外推后圆心，确保飞机真的"绕到 actor 未来位置上"）
        target_x = tx_c + ORBIT_RADIUS * math.cos(angle)
        target_y = ty_c + ORBIT_RADIUS * math.sin(angle)

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
