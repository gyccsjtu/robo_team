#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集群协同搜索集中式管理器（自建实现，步骤 1：两机共享状态）。

职责：
  1. 订阅各机 /swarm/uav_status，维护机队位置与任务格覆盖状态；
  2. 用 swarm_task.py 的 CoverageGrid + TaskAllocator 做集中式拍卖，给每机分配
     不重复的未搜索格；用 LeaseManager 处理掉线/卡死超时重分配；
  3. 发布 /swarm/assignment（SearchAssignment），每机按 uav_id 过滤自己的任务。

设计约束（答辩/查重）：
  - 只复用自研 swarm_task.py（纯逻辑，原创拍卖效用 + 租约），不复制 Crazyswarm2
    的任何节点/消息/类/配置。
  - 管理器只做「任务分配」，不做避障/轨迹，避障由各机 ESDF-DWA 自理。
"""

import os
import json
import rospy
import math
import time
import threading
from functools import wraps
from swarm_task import LineOfSight
import re
import traceback

from robocup_swarm.msg import UavStatus, SearchAssignment, TargetState, TargetDetection
from std_msgs.msg import String

# 直接 import 同目录的纯逻辑模块（scripts 目录已加进 PYTHONPATH）
from swarm_task import (CoverageGrid, TaskAllocator, LeaseManager,
                        STATE_FREE, STATE_ASSIGNED, STATE_COVERED, STATE_REVIEW,
                        W_GAIN, W_FLIGHT, W_OVERLAP, W_RISK, W_BALANCE, W_DISTANCE, W_ZONE, LEASE_DURATION,
                        CONFIRM_TIME, EVADE_TIME, DETECT_RADIUS, DETECT_RADIUS_MARGIN)
from cooperative_tracker import CooperativeTracker, CONFIRM_HOLD_TIMEOUT
from csv_logger import logger
from visual_evidence import parse_visual_evidence

# 地图（用于过滤「格中心落在建筑内」的格子，与 agent 的 A* 膨胀判定一致）
from robocup_navigation.astar import load_metadata, GridMap, plan

# ============================ 参数 ============================
MAP_X_MIN, MAP_X_MAX = -100.0, 100.0
MAP_Y_MIN, MAP_Y_MAX = -50.0, 50.0

# Search coverage comes from metadata x[-55,155], y[-65,65].  Waypoints are
# kept inside the actor plugin's reachable XY envelope with room to brake.
FLIGHT_X_MIN = float(os.environ.get('FLIGHT_X_MIN', '-50.0'))
FLIGHT_X_MAX = float(os.environ.get('FLIGHT_X_MAX', '150.0'))
FLIGHT_Y_MIN = float(os.environ.get('FLIGHT_Y_MIN', '-60.0'))
FLIGHT_Y_MAX = float(os.environ.get('FLIGHT_Y_MAX', '60.0'))
FLIGHT_INSET = float(os.environ.get('FLIGHT_INSET', '2.0'))


def flight_waypoint(x, y, inset=FLIGHT_INSET):
    inset = max(0.0, min(inset, (FLIGHT_X_MAX - FLIGHT_X_MIN) / 4.0,
                         (FLIGHT_Y_MAX - FLIGHT_Y_MIN) / 4.0))
    return (min(max(x, FLIGHT_X_MIN + inset), FLIGHT_X_MAX - inset),
            min(max(y, FLIGHT_Y_MIN + inset), FLIGHT_Y_MAX - inset))


def search_viewpoint(x, y):
    """Observe an outer search cell from camera range inside the flight box.

    Clamping its center just 2 m inside the boundary puts a horizontal
    camera almost above the cell, below its vertical field of view. Only
    search cells outside the safe flight inset get a standoff viewpoint;
    tracking keeps the original flight_waypoint behavior.
    """
    px, py = flight_waypoint(x, y)
    dx, dy = px - x, py - y
    distance = math.hypot(dx, dy)
    if distance < 1e-6 or distance >= 8.5:
        return px, py
    scale = 8.5 / distance
    return flight_waypoint(x + dx * scale, y + dy * scale)
# 2026-09-28：默认必须与 swarm_task.py 一致（那边已是 7.0）。
# 10.0 vs 7.0 会导致 manager 的栅格与 task 的拍卖栅格错位（不传 env 时必踩）。
GRID_SIZE_M = float(os.environ.get("GRID_SIZE_M", "7.0"))   # 搜索格边长（与 swarm_task 必须一致，都读同一个环境变量）
CRUISE_SPEED = 5.0              # 拍卖飞行时间估算用巡航速度（与 agent MAX_SPEED 一致）
ALLOC_PERIOD = float(os.environ.get("ALLOC_PERIOD", "1.5"))  # 国家一等奖：拍卖周期 1.5s（原 2.0，6 架 5 分钟下要尽量减少空闲）
LEASE_CHECK_PERIOD = float(os.environ.get("LEASE_CHECK_PERIOD", "0.5"))  # 租约到期检查周期缩短到 0.5s
TARGET_CHECK_PERIOD = float(os.environ.get("TARGET_CHECK_PERIOD", "1.0"))  # 目标确认计时更新周期 s（规则5 的 15s 按此粒度累计）
# 国家一等奖：开启「目标预瞄」（在 actor 已知位置周围 60m 内给邻居格加分，
# 引导飞机主动斜插追踪区）。开启=1 关闭=0。
HOT_TARGET_ENABLE = int(os.environ.get("HOT_TARGET_ENABLE", "1"))
# 国家一等奖：协同层主循环里强制把跟踪机优先写入 hot target，
# 而不是只在 detect/dispatch 时同步。避免「追踪机眼看要丢/已丢但
# 全队还在远处面覆盖」的协同层错误。
HOT_TARGET_TTL = float(os.environ.get("HOT_TARGET_TTL", "8.0"))

# ---- 确认期冗余派机（可全用环境变量关掉/调参，无需改代码）----
BACKUP_ENABLE = int(os.environ.get("BACKUP_ENABLE", "1"))   # 0=关闭
AUTO_LAND = int(os.environ.get("AUTO_LAND", "0"))   # 0=禁止自动降落（默认）
BACKUP_STALE  = float(os.environ.get("BACKUP_STALE",  "2.0"))  # 断流多少秒后加派（紧迫）
# === 2026-10-05 国家一等奖修：3.0 → 5.0 ===
# 原值 3s：实测「确认主追在 LOS 内追踪就触发 backup」→ 抢资源、backup 飞过去
# 时主追已经确认完，备份白飞且扰乱面覆盖。新值 5s：等主追稳定确认（确认 ~5s 达标）
# 后再判断是否真正受阻；stall=5s 才是真正需要 backup 的信号。
BACKUP_AFTER  = float(os.environ.get("BACKUP_AFTER",  "5.0"))  # 确认卡住5s才加派（让主追先打）
BACKUP_MAX    = int(os.environ.get("BACKUP_MAX",    "2"))    # 全场同时存在的备份机上限
SEARCH_RESERVE_MAX = int(os.environ.get("SEARCH_RESERVE_MAX", "2"))
BACKUP_MAX_DIST = float(os.environ.get("BACKUP_MAX_DIST", "20.0"))  # 远机留给附近新目标
                                                                    # （飞过去的时间比等还久，净亏损）

# 正式节点绝不接收仿真真值。保留环境变量仅用于拒绝旧启动配置。
SEED_TRUTH = int(os.environ.get("SEED_TRUTH", "0"))
# 正式模式中的 /swarm/target_states 来自本队 YOLO 桥：允许用新鲜视觉候选
# 提前派机接近，但绝不把该坐标作为裁判真值或内部误差校验基准。
VISUAL_DISPATCH = int(os.environ.get("VISUAL_DISPATCH", "1"))
MAP_BOUNDS_FROM_META = os.environ.get("MAP_BOUNDS_FROM_META", "1") not in ("0", "false", "False")
# 覆盖记忆：把各机当前位置探测半径内的格子标记为已覆盖。
# COVER_MARK=0 关闭（复现旧行为）；比例可调（1.0 = 用满 DETECT_RADIUS）
COVER_MARK = float(os.environ.get("COVER_MARK", "1.0"))
COVER_MARK_RATIO = float(os.environ.get("COVER_MARK_RATIO", "1.0"))
REOPEN_MIN_DIST = float(os.environ.get("REOPEN_MIN_DIST", "30.0"))  # 重开时跳过飞机脚下这个半径内的格
DEFAULT_UAV_IDS = ["uav_1", "uav_2"]  # 默认两机

METADATA_PATH = os.environ.get(
    "ROBOCUP_METADATA",
    os.path.join(os.environ.get("ROBOCUP_WS", "/home/ros/team_ws/robocup"),
                 "src/robocup_training_worlds/worlds/generated/robocup_base.json"))
INFLATE_M = float(os.environ.get('INFLATE_M', '2.0'))     # 与 agent 一致：A* 障碍膨胀半径 m（2026-10-05 灯杆排查：0.6m 灯杆半径+0.37m 桨尖+0.68m 切角超调+0.35m 余量，原 1.5 实测贴到 front=0.31m）

# ---- 合规 SLAM 栅格合并（2026-10-07 重构，规则 §2.4/§2.5 无预读）----
# metadata 障碍恒为空；阻塞格来自各 agent /swarm/occupancy_grid 上报的
# 雷达 SLAM 占用并集，随建图增长周期性重建。
SLAM_REBUILD_SEC = float(os.environ.get('SLAM_REBUILD_SEC', '5.0'))  # 阻塞格重建最小间隔 s

# ---- 2026-09-27：楼边修复（建筑内的 actor 藏身区曾是永久搜索黑洞）----
EDGE_ENABLE   = os.environ.get("EDGE_ENABLE", "1") not in ("0", "false", "False", "")
W_EDGE        = float(os.environ.get("W_EDGE", "0.6"))     # 楼边格效用偏置权重
EDGE_RANGE    = float(os.environ.get("EDGE_RANGE", "12.0"))  # 距障碍多远还算「楼边」 m
EDGE_NEAR     = float(os.environ.get("EDGE_NEAR", "2.0"))    # 距障碍 <= 此值给满分
# 派追踪机用的目标位置缓存有效期 s：agent /swarm/detection 上报后，
# 若超过此时长无新检测，认为目标已跟丢，不再用过期坐标派机（防追鬼）。
TRUTH_TTL = float(os.environ.get("TRUTH_TTL", "3.0"))  # 几何上报停止后及时撤销旧派机坐标
VISUAL_REACQUIRE_HOLD_S = float(os.environ.get("VISUAL_REACQUIRE_HOLD_S", "8.0"))
UNVERIFIED_VIEW_S = float(os.environ.get("UNVERIFIED_VIEW_S", "25.0"))
UNVERIFIED_VISUAL_GRACE_S = float(os.environ.get(
    "UNVERIFIED_VISUAL_GRACE_S", "3.0"))
UNVERIFIED_RADIUS_M = float(os.environ.get("UNVERIFIED_RADIUS_M", "8.0"))
VISUAL_STREAK_MAX_HOLD_S = float(os.environ.get(
    "VISUAL_STREAK_MAX_HOLD_S", "45.0"))
VISUAL_RETRY_COOLDOWN_S = float(os.environ.get(
    "VISUAL_RETRY_COOLDOWN_S", "8.0"))
TRACKING_MAX_TENURE_S = float(os.environ.get(
    "TRACKING_MAX_TENURE_S", "90.0"))
VIEW_RECOVERY_S = float(os.environ.get("VIEW_RECOVERY_S", "20.0"))


def visual_streak_should_release(done_t, now, fresh,
                                 min_hold=CONFIRM_HOLD_TIMEOUT,
                                 max_fresh_hold=VISUAL_STREAK_MAX_HOLD_S):
    """A team image streak never proves removal; release its lease on timeout."""
    elapsed = now - done_t
    return elapsed >= min_hold and (
        not fresh or elapsed >= max(min_hold, max_fresh_hold))
# 格中心落在建筑里时，去这些半径的环上找可达的替代航点（m）
WAYPOINT_RING = (2.0, 3.5, 5.0)
CLEARANCE_MAX = float(os.environ.get("CLEARANCE_MAX", "24.0"))  # 距离场最大计算范围 m

# ---- LOS 感知 next-best-view（2026-09-28）----
# 与 W_EDGE 的区别：EDGE 是静态的「贴楼加分」（实测成共同吸引子，6 架挤一起）；
# NBV 用真实处理图像记录久未见区域；派位本身不产生覆盖证据。
VIS_ENABLE = os.environ.get("VIS_ENABLE", "1") not in ("0", "false", "False", "")
VIS_RADIUS = float(os.environ.get("VIS_RADIUS", "10.0"))   # 与 DETECT_RADIUS 一致（2026-10-05: 20→10，同步收紧）
EDGE_COVER_STRIP_M = float(os.environ.get("EDGE_COVER_STRIP_M", "14.0"))
EDGE_COVER_RADIUS_M = float(os.environ.get("EDGE_COVER_RADIUS_M", "5.5"))
# 飞机抵达分配格的判定半径：距格代表点 < 此值即认为已搜索该格，
# 标记可视域覆盖并释放租约。太小会在格边来回振荡不释放，太大则还没飞到就算覆盖。
COVER_ARRIVE_M = float(os.environ.get("COVER_ARRIVE_M", "4.0"))  # 减小到达判定半径，适应7m格子
# 2026-10-07 v12: 租约最长连续持有时限（秒）。续租条件原为"3 秒内有上报"，
# 卡死/打不开爬升闸门的机持续上报 → 无条件续租 → 租约永不到期 → 全场只派
# 首轮 6 格后死锁（实测 sim 88s 后零新分配，agent1/5 悬停起飞点全程）。
# 超时不续租 → 租约按 TTL 到期回收 → 格子回候选池重新拍卖，形成调度周转。
LEASE_MAX_HOLD = float(os.environ.get("LEASE_MAX_HOLD", "90"))
LEASE_STALL_S = float(os.environ.get("LEASE_STALL_S", "20"))
LEASE_STALL_PROGRESS_M = float(os.environ.get("LEASE_STALL_PROGRESS_M", "2.0"))
# 派遣追踪机的距离余量：只有最近机距目标 < DETECT_RADIUS - DISPATCH_MARGIN 才派遣。
# 贴边派遣后目标一动就出视野跟丢，留余量保证追踪机进入感知纵深。
DISPATCH_MARGIN = float(os.environ.get("DISPATCH_MARGIN", "2.0"))  # 派阈值放近(2m)，首见即派不拖到近距
# 2026-10-03 国家一等奖改进：单一检测就立刻登记 + 派机，不再等 3 击确认
DISPATCH_ON_FIRST_HIT = int(os.environ.get("DISPATCH_ON_FIRST_HIT", "1"))  # 1=首见即派
# 派遣冗余距离：若最近机 > 此距离，就近派一名备份机（不是要等，确认期即可搭帮）
DISPATCH_BACKUP_DIST = float(os.environ.get("DISPATCH_BACKUP_DIST", "40.0"))
# 改进 2：actor 移动预测——基于过去 N 秒位置差分外推到飞机抵达时刻
PREDICT_ACTOR_MOTION = int(os.environ.get("PREDICT_ACTOR_MOTION", "1"))  # 1=启用
PREDICT_ACTOR_VMAX = float(os.environ.get("PREDICT_ACTOR_VMAX", "2.5"))  # 官方上限 2m/s，留 0.5m/s 余量


class _ClearanceField(object):
    """到最近障碍格的距离场（4 邻域多源 BFS，保守下界）。

    用途只有一个：把「距离建筑多远」变成一个 0~1 的分数，喂给任务拍卖的
    效用函数，让搜索队形优先贴着楼边铺开 —— actor 的出生点几乎全都
    贴着建筑（实测 6 个出生点到最近建筑的间距只有 0~11m）。
    """

    def __init__(self, g, max_m=CLEARANCE_MAX):
        from collections import deque
        self.w, self.h = g.width, g.height
        self.res = g.resolution
        self.ox, self.oy = g.origin[0], g.origin[1]
        limit = int(max_m / self.res)
        dist = [None] * (self.w * self.h)
        q = deque()
        row = 0
        for y in range(self.h):
            row = y * self.w
            for x in range(self.w):
                if g.cells[row + x] != 0:
                    dist[row + x] = 0
                    q.append((x, y))
        self.dist = dist
        while q:
            cx, cy = q.popleft()
            d = dist[cy * self.w + cx]
            if d >= limit:
                continue
            nd = d + 1
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.w and 0 <= ny < self.h:
                    idx = ny * self.w + nx
                    if dist[idx] is None:
                        dist[idx] = nd
                        q.append((nx, ny))

    def clearance(self, point):
        """世界坐标 -> 到最近障碍的距离(m)；不可达/越界返回 None。"""
        ix = int(math.floor((point[0] - self.ox) / self.res))
        iy = int(math.floor((point[1] - self.oy) / self.res))
        if not (0 <= ix < self.w and 0 <= iy < self.h):
            return None
        d = self.dist[iy * self.w + ix]
        return None if d is None else d * self.res



def inflate_grid(grid, inflation_m):
    """对障碍物向外膨胀 inflation_m（圆盘结构元素），返回新 GridMap。

    与 swarm_agent 的 inflate_grid 完全一致，保证 manager 过滤与 agent 规划
    用同一套「障碍」定义，避免 manager 分配了 agent 无法到达的格。
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



def _search_bounds():
    """搜索栅格的边界。

    硬编码的 ±100/±50 与官方 actor 可达范围（control_actor.py 写死的
    x[-50,130] / y[-60,60]）不一致：
      - x>100、y<-50、y>50 这三块 actor 真的会去的地方**永远不会被搜索**；
      - x[-100,-50] 那 5000 m² actor 根本到不了，却每轮都白扫。
    栅格元数据 robocup_base.json 的 bounds 就是按官方 actor 活动范围生成的，
    直接用它。读不到就退回硬编码。
    """
    if not MAP_BOUNDS_FROM_META:
        return MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX
    try:
        with open(METADATA_PATH) as _f:
            b = json.load(_f).get("bounds")
        if b:
            return (float(b["x_min"]), float(b["x_max"]),
                    float(b["y_min"]), float(b["y_max"]))
    except Exception as _exc:
        print("[manager] 读元数据 bounds 失败，用硬编码边界: %s" % _exc)
    return MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX


def joint_target_assignment(costs):
    """Exact small matching: maximize staffed targets, then minimize total cost.

    Six aircraft need at most 64 occupancy masks; no planner or ROS calls.
    costs maps target IDs to {available UAV ID: finite cost}.
    """
    uids = sorted({uid for edges in costs.values() for uid in edges})
    bits = {uid: 1 << i for i, uid in enumerate(uids)}
    states = {0: (0.0, ())}
    for tid in sorted(costs):
        next_states = dict(states)
        for mask, (cost, pairs) in states.items():
            for uid, extra in sorted(costs[tid].items()):
                if mask & bits[uid] or not math.isfinite(extra):
                    continue
                key = mask | bits[uid]
                candidate = (cost + extra, pairs + ((tid, uid),))
                if key not in next_states or candidate < next_states[key]:
                    next_states[key] = candidate
        states = next_states
    _, pairs = min(states.values(), key=lambda item: (-len(item[1]), item[0], item[1]))
    return dict(pairs)


def serialized_tasks(method):
    """ROS callbacks and the scheduler must not allocate the same UAV twice."""
    @wraps(method)
    def call(self, *args, **kwargs):
        lock = getattr(self, '_task_lock', None)
        if lock is None:
            lock = self.__dict__.setdefault('_task_lock', threading.RLock())
        with lock:
            return method(self, *args, **kwargs)
    return call


class SwarmManager(object):
    def __init__(self, uav_ids):
        self._task_lock = threading.RLock()
        self.uav_ids = uav_ids
        # 每机最新状态
        self.status = {}          # uav_id -> UavStatus
        # 每机最近一次上报时间（用于租约续租判定）
        self.last_report = {}     # uav_id -> rospy.Time
        self._active_leases = {}  # uav_id -> cell key currently leased to that UAV
        self._lease_hold_since = {}  # (uid, key) -> 首次续租时刻，用于 LEASE_MAX_HOLD
        self._lease_motion = {}  # uid -> (cell key, last meaningful progress time, x, y)
        # === 2026-10-05 国家一等奖修：追踪释放后冷却 ===
        # 实测问题：「释放追踪机 typhoon_h480_1 (tid=t5, 原因=confirmed)」紧接着
        # 同一行「分配 typhoon_h480_1 → 格 (2,6)」—— 飞机刚脱离追踪立刻被派回
        # 同一格（甚至更远），EKF 还在抖 + 当前速度尚未归零，刚发的 assign
        # 立即被 A* 路径覆盖、飞机在原地晃两拍就 stuck。改为：释放追踪后 N 秒
        # 内不再进入空闲池，等 EKF 完全收敛再接新任务。
        self._release_cooldown = {}  # uav_id -> rospy.Time
        # LEASE_RELEASE_COOLDOWN_S：默认 6.0s（与 EST_HOLD 600ms + 路径重规划 1s + 余量 4s 对齐）

        # ---- 纯逻辑模块 ----
        _bx0, _bx1, _by0, _by1 = _search_bounds()
        rospy.loginfo("[manager] 搜索栅格边界 x[%.0f,%.0f] y[%.0f,%.0f]（%s）",
                      _bx0, _bx1, _by0, _by1,
                      "元数据" if MAP_BOUNDS_FROM_META else "硬编码")
        self.grid = CoverageGrid(_bx0, _bx1, _by0, _by1, GRID_SIZE_M, len(uav_ids))
        self.allocator = TaskAllocator(self.grid, W_GAIN, W_FLIGHT, W_OVERLAP, W_RISK, W_BALANCE,
                                       w_distance=W_DISTANCE, w_zone=W_ZONE, cruise_speed=CRUISE_SPEED)
        self.lease = LeaseManager(self.grid, duration=LEASE_DURATION)

        # ---- 过滤「格中心落在建筑内」的格子：这些格 agent 的 A* 无法到达 ----
        # 合规重构（2026-10-07）：不再从 metadata 预读建筑真值；阻塞格来自
        # 各 agent 雷达 SLAM 占用栅格的并集（_occ_grid_cb），启动时为空
        # （未知=可通行），随建图增长由主循环周期重建。
        self._slam_lock = threading.Lock()
        self._slam_cells = None    # 多机最新 SLAM 快照的占用并集
        self._slam_sources = {}    # uav_id -> 最新完整占用栅格；允许清除旧回波
        self._slam_w = self._slam_h = 0
        self._slam_res = 0.5
        self._slam_origin = (0.0, 0.0)
        self._slam_dirty = False
        self._slam_rebuild_t = 0.0
        self._cell_waypoint = {}   # cell_key -> (wx,wy) 楼边格的可达代表点
        self._blind_viewpoints = {}  # 未成像格复查时的侧向观察航点
        self._cell_edge = {}       # cell_key -> 0~1 临楼度
        self._blocked_cells = self._find_blocked_cells()
        # 把「临楼度」交给拍卖器做效用偏置
        try:
            self.allocator.cell_edge = self._cell_edge
        except Exception:
            pass

        # ---- LOS 感知 NBV：预计算每个候选视点的可见集 ----
        self._vis_set = {}
        if VIS_ENABLE:
            try:
                self._vis_set = self._build_visibility()
                self.allocator.set_visibility(self._vis_set)
                _n = [len(v) for v in self._vis_set.values()]
                rospy.loginfo("[manager] NBV 可见集：%d 个视点，平均可见 %.1f 格（最少 %d / 最多 %d）",
                              len(self._vis_set),
                              (sum(_n) / float(len(_n))) if _n else 0.0,
                              min(_n) if _n else 0, max(_n) if _n else 0)
            except Exception as exc:
                rospy.logwarn("[manager] NBV 可见集构建失败: %s（退化为无 NBV）", exc)
                self._vis_set = {}

        # ---- 目标确认/消除（规则4/5）----
        self.tracker = CooperativeTracker(official_only=True, strict_samples=True, obs_ttl=1.0, gap_tol=1.0)
        if SEED_TRUTH:
            raise RuntimeError("SEED_TRUTH=1 被正式节点禁用；只能使用本队视觉观测")
        self._truth_cache = {}     # 历史字段名：仅存本队视觉估计 (x, y, 图像时间)
        self._cur_targets = {}    # target_id -> 本周期是否有检测（用于 lose 判定）
        self._eliminated = set()
        # 确认期冗余备份机：tid -> uav_id（见 _dispatch_backup）
        self._backup = {}
        # v18：confirm 超时黑名单（tid -> 黑名单截止仿真秒），期内不重派追踪机
        self._confirm_timeout_bl = {}
        self._visual_candidate = {}  # tid -> (first_seen, x, y)
        self._visual_observations = {}
        self._eligible_observations = {}  # last accepted image, never refreshed by a bad frame
        self._view_recovery = {}          # bounded recovery from unusable imagery
        self._last_detection_sample = {}
        self._visual_source = {}     # tid -> (camera uid, image time, x, y)
        self._hypothesis_switches = {}  # tid -> accepted bridge switch, original image time
        self._reacquire_sites = {}     # tid -> one bounded last-image search request
        self._visual_near_seen = {}  # tid -> (x, y, t) observed within 12m by a real UAV
        self._visual_quarantine = {}  # tid -> (x, y, expires)
        self._visual_conflict_sites = []  # (x, y, radius, expires, affected IDs)
        # tid -> 最近一次收到 /swarm/detection 的 wall 时刻（断流检测用）
        self._last_detect = {}
        # 本队真实图像连续观测时长仅用于维持视角，不代表裁判确认。
        self._confirm_done_t = {}
        self._backup_noted = set()  # 已提示过「无近机可派」的 tid（避免刷屏）
        self._tracking = {}       # target_id -> assigned_uav_id 当前追踪任务
        self._tracking_since = {} # target_id -> 首次派追踪机的仿真时刻
        # 2026-10-03 改进 2：actor 移动预测——每目标最近 N 次检测轨迹 (x,y,stamp)
        # 飞机抵达延迟（lead_s）内 actor 预计位置 ≈ 当前 + lead_s * 速度向量。
        from collections import deque
        self._target_hist = {}     # tid -> deque([(x,y,stamp), ...], maxlen=10)
        self._mission_finished = False  # 任务已完成（搜索格 + 目标全部结束）
        self._auction_cycle = 0
        self._csv = logger("algorithm", [
            "ros_time", "event", "auction_cycle", "target_id",
            "target_x", "target_y", "target_z", "uav_id", "uav_x",
            "uav_y", "uav_z", "distance_m", "capture_threshold_m",
            "estimation_error_m", "detect_radius_m", "is_captured", "assignment", "cost",
            "orca_vx", "orca_vy", "orca_vz", "tracker_event", "progress",
            "resets", "rejects", "covered", "live_observers", "gap_s",
            "error_ok", "gap_ok", "source"])

        # ---- 订阅 / 发布 ----
        rospy.Subscriber("/swarm/uav_status", UavStatus, self._status_cb)
        rospy.Subscriber("/swarm/detection", TargetDetection, self._detection_cb)
        rospy.Subscriber("/swarm/visual_source", String, self._visual_source_cb)
        self._discovery_state_pub = rospy.Publisher(
            '/swarm/discovery_states', TargetState, queue_size=12)
        rospy.Subscriber("/swarm/discovery_observation", String,
                         self._discovery_cb, queue_size=30)
        rospy.Subscriber("/swarm/visual_conflict", String,
                         self._visual_conflict_cb, queue_size=6)
        rospy.Subscriber("/swarm/visual_candidate_missed", String,
                         self._visual_candidate_missed_cb, queue_size=6)
        rospy.Subscriber("/swarm/processed_camera_frame", String,
                         self._processed_camera_frame_cb, queue_size=30)
        # 合规 SLAM：各 agent 的雷达占用栅格快照（RLE JSON），合并成全局地图
        rospy.Subscriber("/swarm/occupancy_grid", String, self._occ_grid_cb, queue_size=6)
        self.assign_pub = rospy.Publisher("/swarm/assignment", SearchAssignment, queue_size=10)
        # 本队连续图像仅提示保持观察，不证明官方精度或消除。
        self.confirm_pub = rospy.Publisher("/swarm/visual_streak", String, queue_size=10)
        self.reobserve_pub = rospy.Publisher("/swarm/reobserve", String, queue_size=6)

        # 仅接本队 YOLO 桥输出；启动脚本审计此话题的唯一发布者。
        rospy.Subscriber("/swarm/target_states", TargetState, self._truth_cb, queue_size=30)

        self._last_alloc_t = rospy.Time.now()
        self._last_lease_t = rospy.Time.now()
        self._last_target_t = rospy.Time.now()
        self._mission_finished = False  # 任务完成标志

    def _find_blocked_cells(self):
        """按 SLAM 占用栅格膨胀，给每格找一个「可达代表航点」，并算临楼度。

        合规重构（2026-10-07）：不再从 metadata 预读建筑真值（规则
        §2.4/§2.5 无预读随机地图）。地图 = 各 agent 雷达 SLAM 占用并集
        （_merged_grid）；无任何上报时返回空阻塞集（未知=可通行），
        随建图增长由主循环周期重建。

        旧实现只用「10m 格中心」判可达：建筑只要压住格中心，整格就被永久
        标记 STATE_COVERED，_reopen_covered_cells() 也跳过它 —— 场上留下
        一批永不被搜索的黑洞。做法：中心不可达时按 WAYPOINT_RING 在格内
        环形找替代可达点，只有整格都找不到可达点才真的判 dead。
        """
        blocked = set()
        waypoint = {}
        edge = {}
        try:
            g_raw = self._merged_grid()
            if g_raw is None:
                rospy.loginfo_throttle(
                    60, "[manager] SLAM 栅格未就绪（等待 agent 上报），暂不过滤建筑格")
                self._cell_waypoint = waypoint
                self._cell_edge = edge
                return blocked
            g = inflate_grid(g_raw, INFLATE_M)
            self._g_infl = g
            if EDGE_ENABLE:
                clearance = _ClearanceField(g)
                for key, c in self.grid.cells.items():
                    # Edge coverage cells may be outside the UAV flight box.
                    # Observe them from the nearest interior point instead.
                    px, py = search_viewpoint(c.cx, c.cy)
                    view_x, view_y = px, py
                    outer = math.hypot(px - c.cx, py - c.cy) > 0.5
                    ctr = g.world_to_cell((px, py))
                    wp = (px, py) if (ctr is not None and g.is_free(ctr)) else None
                    if wp is None:
                        for r in WAYPOINT_RING:
                            for a in range(12):
                                th = a * math.pi / 6.0
                                px, py = flight_waypoint(
                                    view_x + r * math.cos(th),
                                    view_y + r * math.sin(th))
                                if outer and not 7.8 <= math.hypot(
                                        px - c.cx, py - c.cy) <= 9.0:
                                    continue
                                pc = g.world_to_cell((px, py))
                                if pc is not None and g.is_free(pc):
                                    # 取离建筑尽量远的候选点（留足旋翼余量）
                                    if clearance.clearance((px, py)) is not None and \
                                            clearance.clearance((px, py)) < 1.0:
                                        continue
                                    wp = (px, py)
                                    break
                            if wp is not None:
                                break
                    if wp is None:
                        blocked.add(key)
                        continue
                    waypoint[key] = wp
                    d = clearance.clearance(wp)
                    if d is None or d >= EDGE_RANGE:
                        edge[key] = 0.0
                    elif d <= EDGE_NEAR:
                        edge[key] = 1.0
                    else:
                        edge[key] = (EDGE_RANGE - d) / (EDGE_RANGE - EDGE_NEAR)
            else:
                for key, c in self.grid.cells.items():
                    cell = g.world_to_cell(search_viewpoint(c.cx, c.cy))
                    if cell is None or not g.is_free(cell):
                        blocked.add(key)
        except Exception as exc:
            rospy.logwarn("[manager] 合并 SLAM 栅格过滤建筑格失败: %s（将不过滤）", exc)
        self._cell_waypoint = waypoint
        self._cell_edge = edge
        n_alt = 0
        for _k, _wp in waypoint.items():
            _c = self.grid.cell(_k)
            if _c is not None and (abs(_wp[0] - _c.cx) > 1e-6 or abs(_wp[1] - _c.cy) > 1e-6):
                n_alt += 1
        rospy.loginfo("[manager] 楼边修复（SLAM 栅格）：%d/%d 格完全不可达；%d 格改用替代航点；"
                      "%d 格临楼(edge>0)",
                      len(blocked), len(self.grid.cells), n_alt,
                      sum(1 for v in edge.values() if v > 0.0))
        return blocked

    # ---------------- SLAM 栅格合并（合规建图 2026-10-07）----------------
    def _occ_grid_cb(self, msg):
        """用每机最新完整快照重算并集，保留其他机器对同一墙的证据。"""
        try:
            d = json.loads(msg.data)
            uav_id = str(d["uav_id"])
            w, h = int(d["width"]), int(d["height"])
            res = float(d["resolution"])
            origin = (float(d["origin"][0]), float(d["origin"][1]))
            rle = d["rle"]
        except Exception:
            return
        if not uav_id or w <= 0 or h <= 0 or w * h > 4000000 or res <= 0.0:
            return
        # 先验 RLE 总长再写入：坏帧整帧丢弃，防半帧污染占用并集
        total = 0
        for run in rle:
            try:
                v, c = int(run[0]), int(run[1])
            except Exception:
                return
            if v not in (0, 1) or c <= 0:
                return
            total += c
        if total != w * h:
            return
        snapshot = bytearray(w * h)
        idx = 0
        for v, c in rle:
            c = int(c)
            if int(v):
                snapshot[idx:idx + c] = b"\x01" * c
            idx += c
        with self._slam_lock:
            if self._slam_cells is None:
                self._slam_w, self._slam_h = w, h
                self._slam_res, self._slam_origin = res, origin
                self._slam_cells = bytearray(w * h)
            elif (w != self._slam_w or h != self._slam_h or
                  abs(res - self._slam_res) > 1e-6 or
                  math.hypot(origin[0] - self._slam_origin[0],
                             origin[1] - self._slam_origin[1]) > 1e-6):
                return  # 异构地图不能逐格合并
            self._slam_sources[uav_id] = snapshot
            merged = bytearray(w * h)
            for source in self._slam_sources.values():
                for i, occupied in enumerate(source):
                    if occupied:
                        merged[i] = 1
            if merged != self._slam_cells:
                self._slam_cells = merged
                self._slam_dirty = True

    def _merged_grid(self):
        """多机 SLAM 占用并集 → GridMap（无任何上报时返回 None）。"""
        with self._slam_lock:
            if self._slam_cells is None:
                return None
            return GridMap(self._slam_w, self._slam_h, self._slam_res,
                           self._slam_origin, bytes(self._slam_cells), "map")

    def _rebuild_blocked_cells(self):
        """SLAM 栅格更新后重建阻塞格/航点/临楼度（主循环节流调用）。"""
        try:
            self._blocked_cells = self._find_blocked_cells()
            try:
                self.allocator.cell_edge = self._cell_edge
            except Exception:
                pass
        except Exception as exc:
            rospy.logwarn("[manager] SLAM 阻塞格重建失败: %s", exc)

    def _blind_viewpoint_valid(self, cell, point):
        """Require a free flight point and a clear camera ray to this cell."""
        g = getattr(self, '_g_infl', None)
        safe_point = flight_waypoint(*point)
        if (g is None or math.hypot(point[0] - safe_point[0],
                                    point[1] - safe_point[1]) > 1e-6):
            return False
        d = math.hypot(point[0] - cell.cx, point[1] - cell.cy)
        if not 8.0 <= d <= 9.0:
            return False
        # Keep a metre of additional clearance beyond the 2 m inflated map.
        for dx, dy in ((0.0, 0.0), (1.0, 0.0), (-1.0, 0.0),
                       (0.0, 1.0), (0.0, -1.0),
                       (0.7, 0.7), (0.7, -0.7),
                       (-0.7, 0.7), (-0.7, -0.7)):
            pc = g.world_to_cell((point[0] + dx, point[1] + dy))
            if pc is None or not g.is_free(pc):
                return False
        yaw = math.atan2(cell.cy - point[1], cell.cx - point[0])
        if not self._in_processed_view(*point, yaw, cell.cx, cell.cy,
                                       altitude=4.0, down_pitch=0.0):
            return False
        los = LineOfSight(lambda ix, iy: not g.is_free((ix, iy)),
                          g.resolution, g.origin)
        return los.visible(point[0], point[1], cell.cx, cell.cy)

    def _choose_blind_viewpoint(self, key, cell, observer_xy):
        """Move the next visit 8.5 m aside so the fixed front camera sees ground.

        Only an inner, reachable cell whose completed scan lacked processed
        image coverage enters here. Obstacle checks use the inflated lidar map;
        no map means we cannot safely choose a new flight point.
        """
        if (search_viewpoint(cell.cx, cell.cy) != (cell.cx, cell.cy) or
                getattr(self, '_g_infl', None) is None):
            return None
        normal = self._cell_waypoint.get(key)
        if normal is not None and math.hypot(normal[0] - cell.cx,
                                             normal[1] - cell.cy) > 1.0:
            return None
        old = getattr(self, '_blind_viewpoints', {}).get(key)
        candidates = []
        for i in range(8):
            angle = i * math.pi / 4.0
            point = (cell.cx + 8.5 * math.cos(angle),
                     cell.cy + 8.5 * math.sin(angle))
            if not self._blind_viewpoint_valid(cell, point):
                continue
            repeated = old is not None and math.hypot(
                point[0] - old[0], point[1] - old[1]) < 1.0
            # Cardinal views align exactly with the agent's four scan yaws;
            # prefer the nearest safe one, then change side after a failure.
            travel = math.hypot(point[0] - observer_xy[0],
                                point[1] - observer_xy[1])
            candidates.append((repeated, i % 2 != 0, travel, i, point))
        if not candidates:
            return None
        # A clear camera ray alone does not mean the aircraft can fly there.
        # Use the same inflated grid and no-corner-cutting A* as the agent.
        for _repeat, _diagonal, _travel, _index, point in sorted(candidates):
            if plan(self._g_infl, observer_xy, point,
                    max_expansions=3000).success:
                return point
        return None

    def _search_waypoint(self, key, cell):
        blind = getattr(self, '_blind_viewpoints', {}).get(key)
        if blind is not None:
            if self._blind_viewpoint_valid(cell, blind):
                return blind
            self._blind_viewpoints.pop(key, None)
        return self._cell_waypoint.get(key) or search_viewpoint(cell.cx, cell.cy)

    # ---------------- 回调 ----------------
    @serialized_tasks
    def _status_cb(self, msg):
        self.status[msg.uav_id] = msg
        self.last_report[msg.uav_id] = rospy.Time.now()

    @staticmethod
    def _tid_to_actor(tid):
        """'t3' -> 3；非 tN 形式返回 None。"""
        m = re.match(r'^t(\d+)$', str(tid))
        return int(m.group(1)) if m else None

    def _quarantined_visual(self, tid, x, y, now):
        sites = getattr(self, '_visual_conflict_sites', [])
        if sites:
            sites[:] = [site for site in sites if now < site[3]]
            if (not self._has_eligible_visual(tid, now) and
                    any(tid in ids and math.hypot(x - sx, y - sy) <= radius
                        for sx, sy, radius, _, ids in sites)):
                return True
        q = self._visual_quarantine.get(tid)
        if q is None:
            return False
        if now >= q[2]:
            self._visual_quarantine.pop(tid, None)
            return False
        return math.hypot(x - q[0], y - q[1]) <= UNVERIFIED_RADIUS_M

    @serialized_tasks
    def _visual_conflict_cb(self, msg):
        """Briefly isolate one contradictory location, never an entire color.

        The bridge emits this only after temporally separated camera samples
        from the same UAV classified the same static place as two colors.
        A recent eligible independent camera track retains its observer.
        """
        try:
            data = json.loads(msg.data)
            x, y, sample = (float(data[k]) for k in ('x', 'y', 'sample_s'))
            radius = min(2.0, max(0.3, float(data.get('radius_m', 1.0))))
            ttl = min(20.0, max(1.0, float(data.get('ttl_s', 12.0))))
            now = rospy.Time.now().to_sec()
            ids = set(data['target_ids'])
            if (not all(math.isfinite(v) for v in (x, y, sample)) or
                    not 0.0 <= now - sample <= 2.0 or
                    len(ids) < 2 or not ids.issubset({'t0', 't1', 't2', 't3'})):
                return
            sites = getattr(self, '_visual_conflict_sites', None)
            if sites is None:
                sites = self._visual_conflict_sites = []
            affected = {tid for tid in ids
                        if not self._has_eligible_visual(tid, now)}
            if not affected:
                return
            sites.append((x, y, radius, now + ttl, affected))
            for tid in affected:
                # Keep any independently imaged eligible track.
                position = self._get_target_pos(tid, now)
                if position[0] is None or math.hypot(position[0] - x,
                                                     position[1] - y) > radius + 0.5:
                    continue
                if tid in self._tracking or tid in self._backup:
                    self._release_tracker(tid, reason='visual_conflict')
                self.tracker.targets.pop(tid, None)
                self._truth_cache.pop(tid, None)
                self._cur_targets.pop(tid, None)
                self._visual_candidate.pop(tid, None)
                self._visual_source.pop(tid, None)
                self._target_hist.pop(tid, None)
            rospy.logwarn('[manager] 视觉颜色冲突 (%.1f,%.1f) %s：位置隔离 %.0fs',
                          x, y, sorted(affected), ttl)
        except (ValueError, KeyError, TypeError, AttributeError):
            return

    def _visual_candidate_missed_cb(self, msg):
        """Queue a short re-image of the aircraft viewpoint after failed cue.

        This coordinate is the aircraft position, not an estimated person
        position, and must never enter target tracking or judge reporting.
        """
        try:
            data = json.loads(msg.data)
            uid = data['source_uav']
            x, y, sample = (float(data[k]) for k in ('x', 'y', 'sample_s'))
            now = rospy.Time.now().to_sec()
            st = self.status.get(uid)
            if (uid not in self.uav_ids or st is None or
                    not getattr(st, 'connected', False) or
                    not all(math.isfinite(v) for v in (x, y, sample)) or
                    not 0.0 <= now - sample <= 2.0 or
                    math.hypot(st.x - x, st.y - y) > 5.0):
                return
            key = self.allocator.queue_revisit(
                'unconfirmed', x, y, now=now, delay_s=8.0, ttl_s=35.0,
                blocked=self._blocked_cells)
            if key is not None:
                rospy.loginfo('[manager] 候选补看失败：格 %s 进入一次性视角复查', key)
        except (ValueError, KeyError, TypeError, AttributeError):
            return

    def _camera_cover_reachable(self, ux, uy, cx, cy):
        """Do not mark an outer cell observed from beyond usable camera range."""
        edge = min(cx - self.grid.x_min, self.grid.x_max - cx,
                   cy - self.grid.y_min, self.grid.y_max - cy)
        if edge >= EDGE_COVER_STRIP_M:
            return True
        return math.hypot(cx - ux, cy - uy) <= EDGE_COVER_RADIUS_M

    @staticmethod
    def _in_processed_view(ux, uy, yaw, cx, cy, altitude=None, down_pitch=None):
        dx, dy = cx - ux, cy - uy
        distance = math.hypot(dx, dy)
        if distance > 9.0 or distance < 0.5:
            return False
        if altitude is not None and down_pitch is not None:
            if abs(math.atan2(altitude, distance) - down_pitch) > math.radians(27.0):
                return False
        bearing = math.atan2(dy, dx)
        return abs((bearing - yaw + math.pi) % (2.0 * math.pi) - math.pi) <= math.radians(45.0)

    def _processed_camera_frame_cb(self, msg):
        """Credit cells imaged twice during transit, after the actual YOLO pass."""
        try:
            data = json.loads(msg.data)
            if data.get('processed') is not True:
                return
            uid = data['source_uav']
            if uid not in self.uav_ids:
                return
            sample, ux, uy, yaw, altitude, down_pitch = (float(data[k]) for k in
                ('sample_s', 'x', 'y', 'yaw', 'z', 'down_pitch'))
            now = rospy.Time.now().to_sec()
            st = self.status.get(uid)
            if (st is None or not getattr(st, 'connected', False) or
                    float(getattr(st, 'z', 0.0) or 0.0) < 2.0 or
                    not all(math.isfinite(v) for v in
                            (sample, ux, uy, yaw, altitude, down_pitch)) or
                    altitude <= 0.0 or
                    not -0.05 <= now - sample <= 0.5 or
                    math.hypot(ux - st.x, uy - st.y) > 5.0):
                return
            if not hasattr(self, '_slam_lock'):
                return
            grid_map = self._merged_grid()
            if grid_map is None:
                return
            los = LineOfSight(lambda ix, iy: not grid_map.is_free((ix, iy)),
                              grid_map.resolution,
                              (grid_map.origin[0], grid_map.origin[1]))
            evidence = getattr(self, '_camera_cell_evidence', None)
            if evidence is None:
                evidence = self._camera_cell_evidence = {}
            for key, cell in self.grid.cells.items():
                cx, cy = cell.cx, cell.cy
                # Actual image direction and LOS allow observing the outer
                # strip from the safe inner row. The older 5.5 m edge rule
                # applies only to geometry-based arrival coverage.
                if (not self._in_processed_view(ux, uy, yaw, cx, cy,
                                                altitude, down_pitch) or
                        not los.visible(ux, uy, cx, cy)):
                    continue
                prior = evidence.get((uid, key))
                if prior is None:
                    evidence[(uid, key)] = (sample, sample)
                    continue
                first, last = prior
                if sample <= last:
                    continue
                if sample - last > 0.7:
                    first = sample
                evidence[(uid, key)] = (first, sample)
                if sample - first < 0.2:
                    continue
                owner = cell.owner if cell.state == STATE_ASSIGNED else None
                newly_observed = cell.state != STATE_COVERED
                if newly_observed:
                    self.grid.is_covered(cx, cy, 0.01)
                else:
                    cell.covered_t = sample
                if hasattr(self.allocator, 'observed_cell'):
                    self.allocator.observed_cell(key, sample)
                else:
                    self.allocator.visit_time[key] = sample
                    self.allocator.last_seen[key] = sample
                audit = getattr(self, '_csv', None)
                if newly_observed and audit is not None:
                    # Flight/assignment logs cannot reconstruct camera
                    # coverage. Record only actual two-image + LOS credits,
                    # separately from scan completion and dispatched cells.
                    audit.write(ros_time=sample, event='camera_coverage',
                                target_id='cell_%d_%d' % key,
                                target_x=cx, target_y=cy, uav_id=uid,
                                uav_x=ux, uav_y=uy, uav_z=altitude,
                                covered=True, source='processed_camera_frame')
                getattr(self, '_blind_viewpoints', {}).pop(key, None)
                if owner is not None:
                    if hasattr(self.allocator, 'complete_waypoint'):
                        self.allocator.complete_waypoint(
                            owner, key, observed=True, now=sample)
                    if self._active_leases.get(owner) == key:
                        self._active_leases.pop(owner, None)
                        self._lease_hold_since.pop((owner, key), None)
                        self._lease_motion.pop(owner, None)
                evidence.pop((uid, key), None)
        except (ValueError, KeyError, TypeError, AttributeError, RuntimeError):
            return

    def _remember_visual_candidate(self, tid, x, y, now):
        candidate = self._visual_candidate.get(tid)
        if candidate is None or math.hypot(x - candidate[1], y - candidate[2]) > UNVERIFIED_RADIUS_M:
            self._visual_candidate[tid] = (now, x, y)

    def _release_unverified_visuals(self, now):
        """Free an observer with prolonged unusable imagery, then revisit soon.

        A real stationary actor can produce cropped boxes, so absence of any
        external confirmation must never blacklist the place for the match.
        """
        for tid, (first, x, y) in list(self._visual_candidate.items()):
            attempt_age = now - first
            if attempt_age < UNVERIFIED_VIEW_S:
                continue
            # Brief gaps get the recovery budget below. Occasional accepted
            # frames cannot reset that entire budget; a sustained local view
            # can. Neither outcome classifies the person as a decoy.
            uid = self._tracking.get(tid)
            if self._stable_owner_view(uid, tid, now):
                getattr(self, '_view_recovery', {}).pop(tid, None)
                continue
            st = self.status.get(uid) if uid is not None else None
            near = getattr(self, '_visual_near_seen', {}).get(tid)
            close_witness = (near is not None and near[2] >= first and
                             now - near[2] <= 3.0 and
                             now - self._last_detect.get(tid, -1e18) <= 3.0 and
                             math.hypot(near[0] - x, near[1] - y) <= UNVERIFIED_RADIUS_M)
            # One brief encounter followed by 25 s out of view is not 25 s
            # of evidence that this location is a static decoy.  In the 20:21
            # match a true-looking white at the east edge was quarantined
            # after its only nearby observer had already left for a red backup.
            if not close_witness or st is None or math.hypot(st.x - x, st.y - y) > 12.0:
                continue
            if self._recover_tracking_view(tid, now, 'unverified_visual'):
                continue
            # Bad imagery is not evidence of a decoy. Revisit this view, but
            # never suppress all cameras' reports at this world position.
            self._release_tracker(tid, reason="unverified_visual")
            self._confirm_done_t.pop(tid, None)
            self._visual_candidate.pop(tid, None)
            rospy.logwarn("[manager] %s 在 (%.1f,%.1f) 补看仍失败；"
                          "保留候选等待其他视角，不屏蔽上报", tid, x, y)

    def _build_visibility(self):
        """预计算「从每个候选视点能看见哪些格」（20m 半径 + LOS 不穿楼）。

        视点取该格的**可达航点**（楼边格的中心可能在建筑里，见 _find_blocked_cells），
        被观察对象取格中心。这一步只在启动时算一次。
        """
        g = getattr(self, "_g_infl", None)
        if g is None:
            return {}
        los = LineOfSight(lambda ix, iy: not g.is_free((ix, iy)),
                          g.resolution, (g.origin[0], g.origin[1]))
        try:
            self.allocator.set_los(los)
        except Exception:
            pass
        cells = self.grid.cells
        keys = [k for k in cells if k not in self._blocked_cells]
        vis = {}
        r2 = VIS_RADIUS * VIS_RADIUS
        for k in keys:
            wp = self._search_waypoint(k, cells[k])
            out = []
            for k2 in keys:
                c2 = cells[k2]
                dx = c2.cx - wp[0]
                dy = c2.cy - wp[1]
                if dx * dx + dy * dy > r2:
                    continue
                if los.visible(wp[0], wp[1], c2.cx, c2.cy):
                    out.append(k2)
            vis[k] = out
        return vis

    def _reopen_covered_cells(self, force=False):
        """所有格都已标记覆盖、但目标还没找全时，把覆盖状态重置重新扫一遍。

        不加这一步的话：日志会说"继续搜索"，但没有未覆盖格可分配，
        飞机实际上拿不到新任务，只能原地待命到超时 —— 等于还是收工。

        国家一等奖标准改进（2026-10-03）：
        - 同时 reopen_review()：超时未复查的 REVIEW 格强制回 FREE，
          避免「已知 actor 区域曾低置信扫过、再也没有机会复查」漏报。
        - 让底层 grid._reopen_covered_cells 走每个格独立的 covered_t
          时间戳判断（避免「绝对 mission 时间」导致 60s 后再不重置）。
        """
        # Keep working through genuinely unvisited cells before reopening old
        # ones.  The previous every-second reopen while tracking repeatedly
        # sent aircraft over the same near cells and left distant zones unseen.
        if not force and any(c.state in (STATE_FREE, STATE_REVIEW)
               for key, c in self.grid.cells.items()
               if key not in self._blocked_cells):
            return 0
        # 跳过就在飞机脚下的格：否则拍卖的 W_FLIGHT(距离) 又会把飞机派回原地，
        # 变成"重开=原地打转"（第二轮实测 136 次派位全是同一批格子）。
        near = set()
        for _st in self.status.values():
            if not getattr(_st, "connected", False):
                continue
            for _key, _c in self.grid.cells.items():
                if math.hypot(_c.cx - _st.x, _c.cy - _st.y) < REOPEN_MIN_DIST:
                    near.add(_key)
        n_cov = 0
        for key, c in self.grid.cells.items():
            if key in self._blocked_cells or key in near:
                continue
            if c.state == STATE_COVERED:
                c.state = STATE_FREE
                n_cov += 1
        # 国家一等奖：把"超时未复查 REVIEW"也重开为 FREE
        n_rev = 0
        if hasattr(self.grid, "reopen_review"):
            n_rev = self.grid.reopen_review(max_age=60.0)
        if n_cov or n_rev:
            rospy.loginfo("[manager] 已重新开放 %d 个 COVERED + %d 个 REVIEW（跳过 %d 个近距格）",
                          n_cov, n_rev, len(near))
        return n_cov + n_rev

    @serialized_tasks
    def _discovery_cb(self, msg):
        """Finite visual hypotheses for dispatch only; no official output path."""
        try:
            now = rospy.Time.now().to_sec()
            raw = json.loads(msg.data)
            if (raw.get('purpose') != 'discovery_only' or
                    raw.get('geometry_ok') is not True or
                    raw.get('official_view_ok') is not False or
                    int(raw.get('hits', 0)) < 4):
                return
            data = parse_visual_evidence(msg.data, now, self.uav_ids)
            tid, uid = data['target_id'], data['source_uav']
            if not self._hypothesis_sample_current(tid, data['sample_s'], data['x'], data['y']):
                return
            rng = float(raw['range_m'])
            if (tid not in ('t0', 't1', 't2', 't3') or
                    not math.isfinite(rng) or not 0. < rng <= 12. or
                    data['confidence'] < .55):
                return
            pool = {k:v for k,v in getattr(self, '_discovery_pool', {}).items()
                    if 0. <= now-v['sample_s'] <= 1.}
            key = (tid, uid, str(raw['candidate_id']))
            prior = pool.get(key)
            if prior is not None and data['sample_s'] <= prior['sample_s']:
                return
            pool[key] = dict(data, range_m=rng)
            # All retained hypotheses remain distinct. Cap memory even if a
            # noisy detector continuously changes IDs during a single second.
            pool = dict(sorted(pool.items(), key=lambda kv: kv[1]['sample_s'])[-72:])
            self._discovery_pool = pool
            anchor = self._get_target_pos(tid, now)
            anchored = anchor is not None and anchor[0] is not None
            active = tid in self._tracking
            candidates = []
            for (target, _, _), item in pool.items():
                if target != tid:
                    continue
                dx = (math.hypot(item['x']-anchor[0], item['y']-anchor[1])
                      if anchored else float('inf'))
                if active and (not anchored or dx > 3.):
                    continue  # another same-colour person cannot steal a live task
                if (self._quarantined_visual(tid, item['x'], item['y'], now) or
                        not self._failed_view_dispatch_ok(tid, item['x'], item['y'], now)):
                    continue
                candidates.append((0 if dx <= 3. else 1, -item['sample_s'],
                                   -item['confidence'], item['range_m'], item['source_uav'], item))
            if not candidates:
                return
            selected = min(candidates, key=lambda x:x[:-1])[-1]
            # Do not overwrite fresh report-qualified evidence with exploratory
            # data, or use one cached image repeatedly to extend a task lease.
            precise = self._visual_observations.get((selected['source_uav'], tid))
            if (precise is not None and precise.get('eligible') and
                    0. <= now-precise['sample_s'] <= .5):
                return
            samples = getattr(self, '_discovery_selected_s', None)
            if samples is None:
                samples = self._discovery_selected_s = {}
            if selected['sample_s'] <= samples.get(tid, -1e18):
                return
            samples[tid] = selected['sample_s']
            payload = dict(selected, real_detection=True, official_view_ok=False)
            self._visual_source_cb(String(data=json.dumps(payload)))
            state = TargetState()
            state.header.stamp = rospy.Time.from_sec(selected['sample_s'])
            state.target_id, state.x, state.y = tid, selected['x'], selected['y']
            state.vx = state.vy = 0.
            state.state, state.eliminated = 0, False
            self._discovery_state_pub.publish(state)
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError):
            return

    @serialized_tasks
    def _visual_source_cb(self, msg):
        """Dispatch directly from accepted camera samples, including distant ones."""
        try:
            now = rospy.Time.now().to_sec()
            data = parse_visual_evidence(msg.data, now, self.uav_ids)
            tid, uid, sample = data['target_id'], data['source_uav'], data['sample_s']
            if tid in getattr(self, '_eliminated', set()):
                return
            event = data.get('hypothesis_switch')
            previous = getattr(self, '_hypothesis_switches', {}).get(tid)
            if (not self._hypothesis_sample_current(tid, sample, data['x'], data['y']) or
                    (previous is not None and event is not None and
                     event['switch_s'] < previous['switch_s'])):
                return
            if self._quarantined_visual(tid, data['x'], data['y'], now):
                return
            switched = bool(event is not None and data['eligible'] and
                            (previous is None or event['switch_s'] > previous['switch_s']))
            if switched:
                self._reset_visual_hypothesis(tid, event)
            key = (uid, tid)
            old = self._visual_observations.get(key)
            if old is not None:
                upgrade = (sample == old['sample_s'] and data['eligible'] and
                           not old['eligible'] and
                           math.hypot(data['x']-old['x'], data['y']-old['y']) <= .02)
                if sample <= old['sample_s'] and not upgrade:
                    return
            self._visual_observations[key] = data
            if data['eligible']:
                if not hasattr(self, '_seen_target_slots'):
                    self._seen_target_slots = set()
                self._seen_target_slots.add(tid)
                accepted = getattr(self, '_eligible_observations', None)
                if accepted is None:
                    accepted = self._eligible_observations = {}
                prev = accepted.get(key)
                continuous = prev is not None and 0.0 < sample - prev['sample_s'] <= 1.0
                accepted[key] = dict(data, hits=prev.get('hits', 1) + 1 if continuous else 1,
                                     since_s=prev.get('since_s', prev['sample_s'])
                                     if continuous else sample)
            old_source = self._visual_source.get(tid)
            if old_source is None or sample > old_source[1]:
                self._visual_source[tid] = (uid, sample, data['x'], data['y'])
            self._remember_reacquire_site(tid, sample, data['x'], data['y'])
            if switched:
                self._dispatch_hypothesis_switch(tid, data['x'], data['y'], now)
            detection = TargetDetection()
            detection.header.stamp = rospy.Time.from_sec(sample)
            detection.uav_id, detection.target_id = uid, tid
            detection.x, detection.y = data['x'], data['y']
            detection.confidence, detection.source = data['confidence'], 1
            self._detection_cb(detection)
        except (ValueError, KeyError, TypeError, AttributeError):
            return

    def _hypothesis_sample_current(self, tid, sample, x, y):
        event = getattr(self, '_hypothesis_switches', {}).get(tid)
        return (event is None or
                (sample >= event['switch_s'] and
                 (sample >= event['trial_until_s'] or
                  math.hypot(x-event['old_x'], y-event['old_y']) > 5.)))

    def _reset_visual_hypothesis(self, tid, event):
        """Separate disjoint camera hypotheses without inventing observations."""
        self.__dict__.setdefault('_hypothesis_switches', {})[tid] = dict(event)
        self._discard_reacquire_site(tid)
        for name in ('_visual_observations', '_eligible_observations',
                     '_last_detection_sample', '_qualified_detection_sample'):
            records = getattr(self, name, {})
            for key in list(records):
                if key[1] == tid:
                    records.pop(key, None)
        for name in ('_visual_source', '_truth_cache', '_target_hist',
                     '_visual_near_seen', '_last_detect', '_visual_candidate',
                     '_cur_targets', '_tracking_since', '_approach_leases',
                     '_backup_approach_leases', '_view_recovery', '_tenure_recovery',
                     '_confirm_timeout_bl', '_failed_views', '_lost_view_attempts',
                     '_confirm_done_t', '_reobserve_sent', '_discovery_selected_s'):
            getattr(self, name, {}).pop(tid, None)
        pool = getattr(self, '_discovery_pool', {})
        for key in list(pool):
            if key[0] == tid:
                pool.pop(key, None)
        ct = self.tracker.targets.get(tid)
        if ct is None:
            ct = self.tracker.add_target(tid, event['switch_s'])
        else:
            ct.relocate(event['switch_s'])
            ct.fx = ct.fy = None
            ct.vx = ct.vy = 0.0
            ct.confirmation_pending = False
        ct.set_observers([])

    def _dispatch_hypothesis_switch(self, tid, tx, ty, now):
        """Reallocate a new place atomically, protecting every other primary."""
        clean = self._sanitize_dispatch_xy(tx, ty, tid)
        if clean is None:
            return
        tx, ty = clean
        old_owners = {uid for uid in (self._tracking.get(tid), self._backup.get(tid))
                      if uid is not None}
        other_main = {uid for target, uid in self._tracking.items() if target != tid}
        other_backup = {uid for target, uid in self._backup.items() if target != tid}
        busy = other_main | other_backup
        available = self._dispatch_available(now, busy, require_airborne=True)
        # A seeing backup may help only when no uncommitted aircraft is near.
        # The primary of its existing target must still have a fresh image.
        released_backup = None
        if not any(uid not in old_owners and
                   math.hypot(self.status[uid].x-tx, self.status[uid].y-ty) <= 20.
                   for uid in available):
            for uid, distance in self._camera_candidates(tid, tx, ty, now, other_main):
                old_tid = next((target for target, owner in self._backup.items()
                                if owner == uid and target != tid), None)
                if (old_tid is None or distance > 12. or
                        float(getattr(self.status[uid], 'z', 0.) or 0.) < 1.):
                    continue
                main = self._tracking.get(old_tid)
                if (self._tracking_uav_available(main, now) and
                        self._recent_eligible(old_tid, now, 1., main)):
                    available.append(uid)
                    released_backup = (uid, old_tid)
                    break
        cost_busy = busy - ({released_backup[0]} if released_backup is not None else set())
        costs = self._target_assignment_costs({tid: (tx, ty)}, now, cost_busy, available)
        chosen = joint_target_assignment(costs).get(tid)
        # Keep ownership reserved until the choice is complete; no callback
        # can see this target's old aircraft as free halfway through selection.
        self._tracking.pop(tid, None)
        self._backup.pop(tid, None)
        if chosen is not None:
            if released_backup is not None and chosen == released_backup[0]:
                _, old_tid = released_backup
                self._backup.pop(old_tid, None)
                getattr(self, '_backup_approach_leases', {}).pop(old_tid, None)
                old_ct = self.tracker.targets.get(old_tid)
                if old_ct is not None:
                    old_ct.observers.discard(chosen)
            self._commit_target_assignment(tid, chosen, tx, ty, now)
        # The winning former backup receives its new task before an old-task
        # cancellation. Agents already reject cancels for a different target.
        if released_backup is not None and chosen == released_backup[0]:
            self._publish_target_cancel(chosen, released_backup[1], now)
        for uid in sorted(old_owners - {chosen}):
            self._free_search_lease(uid)
            self._release_cooldown[uid] = rospy.Time.from_sec(now) + rospy.Duration(1.)
            self._publish_target_cancel(uid, tid, now)
        if chosen is None:
            self._queue_lost_revisit(tid, now)
        rospy.loginfo('[HYPOTHESIS_DISPATCH] target=%s owner=%s old=%s '
                      'xy=(%.2f,%.2f); other primaries preserved',
                      tid, chosen, sorted(old_owners), tx, ty)

    def _publish_target_cancel(self, uid, tid, now):
        msg = SearchAssignment()
        msg.header.stamp = rospy.Time.from_sec(now)
        msg.uav_id, msg.target_id, msg.task_type = uid, tid, 255
        msg.cell_ix = msg.cell_iy = -1
        self.assign_pub.publish(msg)

    def _discard_reacquire_site(self, tid):
        site = getattr(self, '_reacquire_sites', {}).pop(tid, None)
        jobs = getattr(getattr(self, 'allocator', None), 'revisit_jobs', {})
        if site is not None and site.get('queued_key') is not None:
            job = ('lost', site['queued_key'])
            record = jobs.get(job)
            if record is not None and record[1] == site['expires_s']:
                jobs.pop(job, None)

    def _remember_reacquire_site(self, tid, sample, x, y):
        sites = self.__dict__.setdefault('_reacquire_sites', {})
        old = sites.get(tid)
        if old is not None and sample <= old['sample_s']:
            return
        self._discard_reacquire_site(tid)
        sites[tid] = dict(sample_s=sample, x=x, y=y, expires_s=sample+25., queued=False)

    def _queue_lost_revisit(self, tid, now):
        sites = self.__dict__.setdefault('_reacquire_sites', {})
        site = sites.get(tid)
        # Also accept an existing original-camera record when upgrading an
        # already running manager; this never changes its image timestamp.
        if site is None:
            source = getattr(self, '_visual_source', {}).get(tid)
            if source is not None and 0. <= now-source[1] < 25.:
                self._remember_reacquire_site(tid, source[1], source[2], source[3])
                site = sites[tid]
        if site is None:
            return
        if now >= site['expires_s']:
            self._discard_reacquire_site(tid)
            return
        site['requested'] = True
        allocator = getattr(self, 'allocator', None)
        if site.get('queued') or not hasattr(allocator, 'queue_revisit'):
            return
        key = allocator.queue_revisit('lost', site['x'], site['y'], now=now,
                                      ttl_s=site['expires_s']-now,
                                      blocked=getattr(self, '_blocked_cells', set()))
        site['queued'], site['queued_key'] = key is not None, key
        jobs = getattr(allocator, 'revisit_jobs', {})
        if key is not None and ('lost', key) in jobs:
            # queue_revisit normally grants at least one second. These jobs
            # instead expire exactly 25 s after the last real camera image.
            ready, _ = jobs[('lost', key)]
            jobs[('lost', key)] = (ready, site['expires_s'])

    def _camera_candidates(self, tid, tx, ty, now, busy):
        """Keep every fresh source, so a busy latest camera hides no free one."""
        sources = {uid: (d['sample_s'], d['x'], d['y'])
                   for (uid, target), d in
                   getattr(self, '_visual_observations', {}).items() if target == tid}
        latest = getattr(self, '_visual_source', {}).get(tid)
        if latest is not None:
            uid, sample, sx, sy = latest
            sources.setdefault(uid, (sample, sx, sy))
        candidates = []
        for uid, (sample, sx, sy) in sources.items():
            if (uid in busy or not -0.05 <= now - sample <= 1.5 or
                    not self._tracking_uav_available(uid, now) or
                    now < getattr(self, '_release_cooldown', {}).get(uid, rospy.Time(0)).to_sec() or
                    math.hypot(sx - tx, sy - ty) > 4.0):
                continue
            st = self.status[uid]
            distance = math.hypot(st.x - tx, st.y - ty)
            if distance <= 20.0:
                candidates.append((uid, distance))
        return sorted(candidates, key=lambda pair: (pair[1], pair[0]))

    def _preferred_camera(self, tid, tx, ty, now, busy):
        candidates = self._camera_candidates(tid, tx, ty, now, busy)
        return candidates[0] if candidates else None

    def _free_camera_backup_for_new_target(self, tid, tx, ty, now):
        """Let a backup pursue a new person that its own camera just saw.

        A primary stays on the old person.  Do this only when no uncommitted
        aircraft is close enough; otherwise keep the backup in place.  In the
        21:03 run UAV5 saw brown at 594.9 s while backing up green, and the
        dispatcher instead selected a searcher 60 m away.  Brown's 15-second
        judge window then began too late.
        """
        busy = set(self._tracking.values()) | set(getattr(self, '_backup', {}).values())
        for free_uid, free_st in self.status.items():
            if (free_uid not in busy and self._tracking_uav_available(free_uid, now) and
                    now >= getattr(self, '_release_cooldown', {}).get(free_uid, rospy.Time(0)).to_sec() and
                    math.hypot(free_st.x - tx, free_st.y - ty) <= 20.0):
                return False
        # Examine all seeing backups, not just the most recent publisher.
        for uid, distance in self._camera_candidates(tid, tx, ty, now, set(self._tracking.values())):
            if distance > 12.0:
                continue
            old_tid = next((other for other, bid in getattr(self, '_backup', {}).items()
                            if bid == uid and other != tid), None)
            if old_tid is None:
                continue
            main = self._tracking.get(old_tid)
            if (not self._tracking_uav_available(main, now) or
                    not self._recent_eligible(old_tid, now, 1.0, main)):
                continue
            self._backup.pop(old_tid, None)
            old_ct = self.tracker.targets.get(old_tid)
            if old_ct is not None:
                old_ct.observers.discard(uid)
            # A freed backup may not win the batch; cancel its old task now.
            msg = SearchAssignment()
            msg.header.stamp = rospy.Time.from_sec(now)
            msg.uav_id, msg.target_id, msg.task_type = uid, old_tid, 255
            msg.cell_ix = msg.cell_iy = -1
            self.assign_pub.publish(msg)
            rospy.loginfo('[manager] 相机 %s 发现新目标 %s，解除 %s 备份占用；'
                          '原主追机仍有新鲜图像', uid, tid, old_tid)
            return True
        return False

    @serialized_tasks
    def _truth_cb(self, msg):
        """兼容旧回调名；只缓存本队 YOLO 桥的新鲜图像目标位置。

        视觉槽 t4/t5 是两条轨迹，不等同于官方 actor_4/5 身份。
        TargetState.eliminated 不能证明裁判已消除任何目标。
        """
        if not VISUAL_DISPATCH or msg.eliminated:
            return
        tid = str(msg.target_id)
        now = rospy.Time.now().to_sec()
        if self._quarantined_visual(tid, msg.x, msg.y, now):
            return
        sample = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
        if sample <= 0.0 or sample > now + 0.05 or now - sample > 1.0:
            return  # 桥的保活帧不可冒充新视觉发现
        if not self._hypothesis_sample_current(tid, sample, msg.x, msg.y):
            return
        prev = self._truth_cache.get(tid)
        if prev is not None and sample <= prev[2]:
            return
        self._truth_cache[tid] = (msg.x, msg.y, sample)
        self._remember_visual_candidate(tid, msg.x, msg.y, now)
        if tid not in self.tracker.targets:
            self.tracker.add_target(tid, now)
            rospy.loginfo("[manager] 新鲜视觉候选 %s @ (%.1f, %.1f)，提前派机接近",
                          tid, msg.x, msg.y)

    def _get_target_pos(self, tid, now=None):
        """取本队视觉估计位置，过期后退回新鲜融合估计。"""
        if tid in self._truth_cache:
            x, y, t = self._truth_cache[tid]
            if now is None or (now - t) <= TRUTH_TTL:
                return (x, y)
            # 缓存过期后不能把旧坐标伪装成新检测。
            self._truth_cache.pop(tid, None)
        ct = self.tracker.targets.get(tid)
        if ct is not None:
            fx = ct.fused(now) if now is not None else None
            if fx is not None:
                return fx
        return None, None

    def _last_eligible(self, uid, tid):
        accepted = getattr(self, '_eligible_observations', {}).get((uid, tid))
        latest = getattr(self, '_visual_observations', {}).get((uid, tid))
        if latest and latest.get('eligible') and (
                accepted is None or latest['sample_s'] > accepted['sample_s']):
            return latest
        return accepted

    def _recent_eligible(self, tid, now, ttl=1.0, uid=None):
        keys = (set(getattr(self, '_eligible_observations', {})) |
                set(getattr(self, '_visual_observations', {})))
        for key in keys:
            if key[1] != tid or (uid is not None and uid != key[0]):
                continue
            image = self._last_eligible(*key)
            if image is not None and 0.0 <= now - image['sample_s'] <= ttl:
                return True
        return False

    def _has_fresh_visual(self, tid, now):
        """Last eligible original image; an intervening bad box cannot erase it."""
        return self._recent_eligible(tid, now)

    def _stable_owner_view(self, uid, tid, now):
        image = self._last_eligible(uid, tid)
        return (image is not None and 0.0 <= now-image['sample_s'] <= 1.0 and
                image.get('hits', 0) >= 4 and
                image['sample_s']-image.get('since_s', image['sample_s']) >= 3.0)

    def _defer_failed_view(self, tid, owners, now, max_age=4.0):
        """Back off a failed visual location, never suppress its detections."""
        source = getattr(self, '_visual_source', {}).get(tid)
        if source is None or not 0.0 <= now-source[1] <= max_age:
            return
        x, y = source[2:4]
        if not all(math.isfinite(v) for v in (x, y)):
            return
        records = getattr(self, '_failed_views', None)
        if records is None:
            records = self._failed_views = {}
        old = records.get(tid)
        repeated = (old is not None and now-old['failed_s'] <= 180.0 and
                    math.hypot(x-old['x'], y-old['y']) <= 3.0)
        attempts = min(3, old['attempts']+1) if repeated else 1
        sources = set(old['sources']) if repeated else set()
        sources.update(uid for uid in owners if uid is not None)
        delay = 15.0 * 2**(attempts-1)
        records[tid] = dict(x=x, y=y, failed_s=now, until=now+delay,
                            attempts=attempts, sources=sources)
        rospy.logwarn('[VIEW_RETRY_DEFER] tid=%s xy=(%.2f,%.2f) retry=%.0fs '
                      'attempt=%d; 暂缓同点重派，保留观测和其他新位置',
                      tid, x, y, delay, attempts)

    def _failed_view_dispatch_ok(self, tid, x, y, now, uid=None):
        old = getattr(self, '_failed_views', {}).get(tid)
        if (old is None or now >= old['until'] or
                math.hypot(x-old['x'], y-old['y']) > 3.0):
            return True
        # A different camera may rescue a genuine stationary person, but a
        # replayed frame or the same failed source cannot immediately restart.
        for (camera_uid, key_tid), image in getattr(self, '_eligible_observations', {}).items():
            if (key_tid == tid and (uid is None or uid == camera_uid) and
                    camera_uid not in old['sources'] and
                    image.get('since_s', -1e18) >= old['failed_s'] and
                    self._stable_owner_view(camera_uid, tid, now) and
                    math.hypot(image.get('x', float('inf'))-x,
                               image.get('y', float('inf'))-y) <= 3.0):
                return True
        return False

    def _note_lost_view(self, tid, now):
        """Repeated short failed looks get the same bounded retry as timeouts.

        This is a place-specific scheduling delay, not a decoy classification.
        A single occlusion or a moving candidate must remain reacquirable.
        """
        uid = self._tracking.get(tid)
        source = getattr(self, '_visual_source', {}).get(tid)
        st = self.status.get(uid)
        if (source is None or st is None or not 0. <= now-source[1] <= 10. or
                math.hypot(st.x-source[2], st.y-source[3]) > 15.):
            return
        records = getattr(self, '_lost_view_attempts', None)
        if records is None:
            records = self._lost_view_attempts = {}
        old = records.get(tid)
        repeated = (old is not None and now-old[0] <= 90. and
                    math.hypot(source[2]-old[1], source[3]-old[2]) <= 3.)
        records[tid] = (now, source[2], source[3])
        if repeated:
            self._defer_failed_view(tid, (uid, self._backup.get(tid)), now, max_age=10.)

    def _recover_tracking_view(self, tid, now, reason):
        """Keep contact through a timeout; hand off only after backup images.

        This is task health, not judge accuracy or elimination. No synthetic
        observations are generated and a totally unusable view is time bounded.
        """
        main = self._tracking.get(tid)
        if main is None:
            return False
        bounded_tenure = reason in ('tracking_tenure', 'confirm_hold_timeout')
        recovery_name = '_tenure_recovery' if bounded_tenure else '_view_recovery'
        recovery = getattr(self, recovery_name, None)
        if recovery is None:
            recovery = {}
            setattr(self, recovery_name, recovery)
        backup = getattr(self, '_backup', {}).get(tid)
        first = recovery.setdefault(tid, now)
        exhausted = now-first >= VIEW_RECOVERY_S
        main_seen = self._recent_eligible(tid, now, UNVERIFIED_VISUAL_GRACE_S, main)
        other = self._last_eligible(backup, tid)
        ready = (backup is not None and self._tracking_uav_available(backup, now) and
                 other is not None and 0.0 <= now - other['sample_s'] <= 1.0 and
                 other.get('hits', 0) >= 3 and
                 other['sample_s'] - other.get('since_s', other['sample_s']) >= 0.4)
        ct = self.tracker.targets.get(tid)
        fragmented_primary = (
            reason == 'unverified_visual' and getattr(ct, 'resets', 0) >= 3 and
            now-self._tracking_since.get(tid, now) >= self.tracker.confirm_time+BACKUP_AFTER and
            not self._stable_owner_view(main, tid, now) and
            self._stable_owner_view(backup, tid, now))
        if ready and (not main_seen or bounded_tenure or fragmented_primary):
            self._tracking[tid] = backup
            self._backup.pop(tid, None)
            self._tracking_since[tid] = now
            self._cancel_unavailable_owner(main, tid, now)
            self.tracker.assign_observers(tid, [backup])
            recovery.pop(tid, None)
            getattr(self, '_view_recovery', {}).pop(tid, None)
            getattr(self, '_tenure_recovery', {}).pop(tid, None)
            rospy.logwarn('[VIEW_HANDOFF] tid=%s old=%s new=%s reason=%s '
                          'image=%.3f; 新机已连续见人，旧机才释放',
                          tid, main, backup, reason, other['sample_s'])
            return True
        if main_seen and not exhausted:
            # A single accepted frame every few seconds must not renew an
            # unsuccessful observation attempt forever. Hard tenure also
            # remains bounded even when the classifier keeps seeing a decoy.
            if not bounded_tenure and self._stable_owner_view(main, tid, now):
                recovery.pop(tid, None)
            # Prolonged image continuity does not prove coordinate accuracy.
            # Reimage from a different safe angle while retaining ownership.
            pub = getattr(self, 'reobserve_pub', None)
            sent = getattr(self, '_reobserve_sent', None)
            if sent is None:
                sent = self._reobserve_sent = {}
            if pub is not None and now - sent.get(tid, -1e18) >= VISUAL_STREAK_MAX_HOLD_S:
                pub.publish(String(json.dumps(dict(
                    target_id=tid, uav_id=main, sample_s=now, reason=reason))))
                sent[tid] = now
            # Do not reset image continuity or manufacture a new observation.
            return True
        if main_seen and not bounded_tenure and self._stable_owner_view(main, tid, now):
            recovery.pop(tid, None)
            return True
        if (exhausted and self._tracking_uav_available(backup, now) and
                self._has_fresh_visual(tid, now) and
                self._hold_approaching_target(tid, now, backup=True)):
            # A remote relief aircraft needs its finite travel budget. Do not
            # cancel it at the original observer's 20-second recovery deadline.
            return True
        if now - first < VIEW_RECOVERY_S:
            rospy.logwarn_throttle(5.0, '[VIEW_RECOVERY] tid=%s main=%s backup=%s '
                                  'reason=%s elapsed=%.1f; 保持目标补看，等待可靠接力',
                                  tid, main, backup, reason, now - first)
            return True
        recovery.pop(tid, None)
        self._queue_lost_revisit(tid, now)
        return False

    def _has_eligible_visual(self, tid, now):
        """Retain tracks only while an actual camera still sees this target."""
        return self._has_fresh_visual(tid, now)

    def _hold_near_visual_target(self, tid, now):
        """Give a nearby observer time to back out of a cropped/occluded view.

        A target briefly seen directly below the camera can lose its only
        usable full-body box before the judge's first find event.  Releasing
        its observer at the 3 s position TTL aborts the agent's wider orbit
        while it is moving to regain the view.  Hold only a recently seen,
        nearby assigned observer; this never creates a detection or report.
        """
        uid = self._tracking.get(tid)
        near = self._visual_near_seen.get(tid)
        detected = self._last_detect.get(tid)
        observer = self.status.get(uid) if uid is not None else None
        if near is None or detected is None or observer is None:
            return False
        return (0.0 <= now - near[2] <= VISUAL_REACQUIRE_HOLD_S and
                0.0 <= now - detected <= VISUAL_REACQUIRE_HOLD_S and
                math.hypot(observer.x - near[0], observer.y - near[1]) <= 12.0)

    @serialized_tasks
    def _dispatch_pending_targets(self):
        """给「已知但未消除」的目标派追踪机（只派一次，已在追的只更新位置）。

        国家一等奖标准改进（2026-10-03）：
        - 写入 hot target：派追踪机时同步把 actor 位置写进 allocator 的
          _hot_targets，附近 60m 内格效用 +4 → 飞机主动飞向追踪区。
        - 派遣距离判定从「最近机 < DETECT_RADIUS - DISPATCH_MARGIN」放宽到
          「< DETECT_RADIUS + DETECT_RADIUS_MARGIN」：原 18m 太紧，飞机
          距离目标 20m 但在视野里就被拒派，要绕 5s 才到，浪费时间。
        - 释放空闲追踪机：目标消除/跟丢时，明确把 _tracking[tid] 移除，
          并 publish cancel_msg = -1 → agent 立即转入新拍卖分配。
        """
        now = rospy.Time.now().to_sec()
        for tid, site in list(getattr(self, '_reacquire_sites', {}).items()):
            if now >= site['expires_s']:
                self._discard_reacquire_site(tid)
            elif site.get('requested') and not site.get('queued') and tid not in self._tracking:
                self._queue_lost_revisit(tid, now)
        pending = []
        for tid in list(self.tracker.targets.keys()):
            if tid in self._eliminated:
                continue
            # v18：confirm 超时黑名单期内不重派（等新检测/位置更新再介入）
            if now < self._confirm_timeout_bl.get(tid, 0.0):
                continue
            ct = self.tracker.targets.get(tid)
            if ct is None or ct.eliminated:
                # 目标已消除/不存在 → 释放追踪/备份机
                if tid in self._tracking or tid in self._backup:
                    self._release_tracker(tid, reason="eliminated")
                continue
            tx, ty = self._get_target_pos(tid, now)
            if tx is None:
                # 目标已跟丢（缓存过期 + 融合无观测）：释放追踪/备份机去搜别的，
                # 避免飞机继续追一个不存在的过期坐标。
                if (self._hold_approaching_target(tid, now) or
                        self._hold_near_visual_target(tid, now)):
                    continue
                if tid in self._tracking or tid in self._backup:
                    self._release_tracker(tid, reason="lost")
                continue
            # 国家一等奖：写入 hot target（让其它飞机知道往这边靠）
            if HOT_TARGET_ENABLE and hasattr(self.allocator, "register_hot_target"):
                try:
                    self.allocator.register_hot_target(tx, ty, now=now)
                except Exception:
                    pass
            if tid in self._tracking:
                self._handoff_distant_tracker(tid, tx, ty, ct)
                self._update_tracker_position(tid, tx, ty)
                continue
            pending.append((tid, tx, ty))
        self._dispatch_target_batch(pending, now)

    def _handoff_distant_tracker(self, tid, tx, ty, ct):
        """已追踪目标旁出现更近的空闲机时，及时接棒远处的主追机。

        上轮 t2 的主追机一直在 30~45m 外赶路；t0 消除后附近机已空出，
        但原指派永久占位，导致这架近机又回搜索。只有差距足够明显时接棒，
        避免两机距离接近时任务来回切换。
        """
        old = self._tracking.get(tid)
        old_st = self.status.get(old)
        if old_st is None:
            return
        old_d = math.hypot(old_st.x - tx, old_st.y - ty)
        if old_d <= 25.0 or self._recent_eligible(
                tid, rospy.Time.now().to_sec(), 1.0, old):
            return
        busy = set(self._tracking.values()) | set(self._backup.values())
        best, best_d = None, float(os.environ.get("HANDOFF_NEAR_M", "20.0"))
        now = rospy.Time.now()
        for uid, st in self.status.items():
            if uid in busy or not self._tracking_uav_available(uid, now.to_sec()):
                continue
            image = self._last_eligible(uid, tid)
            if (image is None or not 0.0 <= now.to_sec() - image['sample_s'] <= 1.0 or
                    image.get('hits', 0) < 3 or
                    image['sample_s'] - image.get('since_s', image['sample_s']) < 0.4 - 1e-6 or
                    math.hypot(image['x'] - tx, image['y'] - ty) > 4.0):
                continue
            if float(getattr(st, "z", 0.0) or 0.0) < 1.0:
                continue
            if now < self._release_cooldown.get(uid, rospy.Time(0)):
                continue
            d = math.hypot(st.x - tx, st.y - ty)
            if d < best_d and old_d - d >= 12.0:
                best, best_d = uid, d
        if best is None:
            return
        self._tracking[tid] = best
        self._record_approach(tid, best, tx, ty, now.to_sec())
        self._tracking_since[tid] = now.to_sec()
        self._free_search_lease(best)
        self._release_cooldown[old] = now + rospy.Duration(1.0)
        ct.observers.discard(old)
        ct.observers.add(best)
        new_msg = SearchAssignment()
        new_msg.header.stamp = now
        new_msg.uav_id, new_msg.target_id, new_msg.task_type = best, tid, 1
        new_msg.cell_ix = new_msg.cell_iy = -1
        new_msg.target_x, new_msg.target_y = flight_waypoint(tx, ty)
        self.assign_pub.publish(new_msg)
        msg = SearchAssignment()
        msg.header.stamp = now
        msg.uav_id = old
        msg.cell_ix = -1
        msg.cell_iy = -1
        msg.target_x = 0.0
        msg.target_y = 0.0
        msg.target_id = tid
        msg.task_type = 255
        self.assign_pub.publish(msg)
        rospy.loginfo("[manager] 近机接棒 %s：%s(%.1fm) → %s(%.1fm)",
                      tid, old, old_d, best, best_d)

    def _record_approach(self, tid, uid, tx, ty, now, backup=False):
        """Travel budget is separate from visual freshness and judge evidence."""
        name = '_backup_approach_leases' if backup else '_approach_leases'
        if not hasattr(self, name):
            setattr(self, name, {})
        if not hasattr(self, '_tracking_since'):
            self._tracking_since = {}
        if not backup:
            self._tracking_since.setdefault(tid, now)
            getattr(self, '_view_recovery', {}).pop(tid, None)
            getattr(self, '_tenure_recovery', {}).pop(tid, None)
        st = self.status[uid]
        distance = math.hypot(st.x - tx, st.y - ty)
        speed = max(0.5, float(os.environ.get('SEARCH_CRUISE_SPEED', '3.0')))
        budget = min(90.0, max(6.0, distance / speed + 6.0))
        getattr(self, name)[tid] = dict(uid=uid, x=tx, y=ty,
            until=now + budget, progress_s=now, anchor=(st.x, st.y))

    def _hold_approaching_target(self, tid, now, backup=False):
        leases = '_backup_approach_leases' if backup else '_approach_leases'
        owners = self._backup if backup else self._tracking
        lease = getattr(self, leases, {}).get(tid)
        if lease is None or owners.get(tid) != lease['uid']:
            return False
        st = self.status.get(lease['uid'])
        if st is None or not getattr(st, 'connected', False) or now >= lease['until']:
            return False
        if math.hypot(st.x - lease['x'], st.y - lease['y']) <= 12.0:
            close_since = lease.setdefault('close_s', now)
            return now - close_since <= 4.0  # allow the first full camera view
        ax, ay = lease['anchor']
        if math.hypot(st.x - ax, st.y - ay) >= 1.0:
            lease['anchor'], lease['progress_s'] = (st.x, st.y), now
        return 0.0 <= now - lease['progress_s'] <= 5.0

    def _release_tracker(self, tid, reason=""):
        """国家一等奖：明确释放追踪/备份机，立即 publish cancel (-1)。

        原实现分散在两处（_dispatch_pending_targets 内部和 _check_lost_targets
        间接路径），删除时容易遗漏。统一从这一处发布。
        """
        if reason == 'lost':
            # A recently imaged actor may have moved during occlusion. Keep
            # one short search viewpoint near the *last image*, not a frozen
            # tracking coordinate, and never reserve multiple aircraft for it.
            now_s = rospy.Time.now().to_sec()
            self._note_lost_view(tid, now_s)
            self._queue_lost_revisit(tid, now_s)
        if reason in ('unverified_visual', 'tracking_tenure', 'confirm_hold_timeout'):
            self._defer_failed_view(tid, (self._tracking.get(tid), self._backup.get(tid)),
                                    rospy.Time.now().to_sec())
        uid = self._tracking.pop(tid, None)
        buid = self._backup.pop(tid, None)
        getattr(self, '_tracking_since', {}).pop(tid, None)
        getattr(self, '_approach_leases', {}).pop(tid, None)
        getattr(self, '_backup_approach_leases', {}).pop(tid, None)
        getattr(self, '_view_recovery', {}).pop(tid, None)
        getattr(self, '_tenure_recovery', {}).pop(tid, None)
        # v18（2026-10-07）：backup 机也必须收到 cancel + 冷却。旧实现 uid=None
        # 时直接 return —— backup 被 pop 却收不到 cancel，agent 侧永远停在追踪
        # assignment（v17 实测 agent_2/agent_5 脱管追 t3 直到比赛结束）。
        if uid is None and buid is None:
            return
        # === 2026-10-05 国家一等奖修：设置冷却 ===
        # 释放追踪后 N 秒内该机不进空闲池，等待 EKF 收敛 + 速度归零 + 路径清空。
        _cd = float(os.environ.get("LEASE_RELEASE_COOLDOWN_S", "6.0"))
        for _rid in (uid, buid):
            if _rid is None:
                continue
            self._release_cooldown[_rid] = rospy.Time.now() + rospy.Duration(_cd)
            self._free_search_lease(_rid)
            rospy.loginfo("[manager] %s 释放追踪冷却 %.1fs (tid=%s, 原因=%s)",
                          _rid, _cd, tid, reason)
        # 写入清除 hot target（避免飞机继续往这里斜插）
        if HOT_TARGET_ENABLE and hasattr(self.allocator, "clear_hot_target"):
            try:
                # 用 tracker 历史里的最近位置（如果还在）
                last_pos = self._truth_cache.get(tid)
                if last_pos:
                    self.allocator.clear_hot_target(last_pos[0], last_pos[1])
            except Exception:
                pass
        # 发布取消（task_type=-1 让 agent 立即转入新分配）
        # v18：main 和 backup 都要收到 cancel
        for _rid in (uid, buid):
            if _rid is None:
                continue
            try:
                msg = SearchAssignment()
                msg.header.stamp = rospy.Time.now()
                msg.uav_id = _rid
                msg.cell_ix = -1
                msg.cell_iy = -1
                msg.target_x = 0.0
                msg.target_y = 0.0
                if hasattr(msg, "target_id"):
                    msg.target_id = tid
                msg.task_type = 255  # uint8 消息；255 专用取消码
                self.assign_pub.publish(msg)
            except Exception:
                pass
        if reason:
            rospy.loginfo("[manager] 释放追踪机 %s (tid=%s, 原因=%s)",
                          [r for r in (uid, buid) if r], tid, reason)

    def _free_search_lease(self, uid):
        key = getattr(self, '_active_leases', {}).pop(uid, None)
        getattr(self, '_lease_motion', {}).pop(uid, None)
        if key is not None:
            self.lease.force_expire(uid, key)
            self._lease_hold_since.pop((uid, key), None)

    # ---------------- 目标检测与消除（规则4/5） ----------------
    @serialized_tasks
    def _detection_cb(self, msg):
        """收到某机对某目标的检测 → 喂给 CooperativeTracker。

        只维护本队真实图像的连续性和融合；没有真值，无法判定裁判的
        1 米误差门限。_update_targets 周期更新任务健康，不宣布官方消除。
        """
        # Geometry echoes and predicted states are never camera observations.
        if msg.source != 1 or msg.target_id in self._eliminated:
            return
        tid, now = msg.target_id, rospy.Time.now().to_sec()
        sample = msg.header.stamp.to_sec()
        key = (msg.uav_id, tid)
        evidence = self._visual_observations.get(key)
        previous_sample = self._last_detection_sample.get(key, -1e18)
        qualified = getattr(self, '_qualified_detection_sample', {})
        upgrade = (evidence is not None and evidence['eligible'] and
                   sample == previous_sample and sample > qualified.get(key, -1e18))
        if (evidence is None or not all(math.isfinite(v) for v in
                                        (sample, msg.x, msg.y, msg.confidence)) or
                abs(sample-evidence['sample_s']) > 1e-6 or
                not 0.0 <= now-sample <= 1.0 or
                math.hypot(msg.x-evidence['x'], msg.y-evidence['y']) > 0.02 or
                (sample <= previous_sample and not upgrade)):
            return
        self._last_detection_sample[key] = sample
        if self._quarantined_visual(tid, msg.x, msg.y, now):
            return
        self._remember_visual_candidate(tid, msg.x, msg.y, now)
        self._last_detect[tid] = max(sample, self._last_detect.get(tid, -1e18))
        observer = self.status.get(msg.uav_id)
        if observer is not None and math.hypot(observer.x - msg.x,
                                                observer.y - msg.y) <= 12.0:
            self._visual_near_seen[tid] = (msg.x, msg.y, sample)
        cached = self._truth_cache.get(tid)
        if cached is None or sample > cached[2]:
            self._truth_cache[tid] = (msg.x, msg.y, sample)

        # CooperativeTracker 还不知道这个目标？登记
        if tid not in self.tracker.targets:
            self.tracker.add_target(tid, now)
            rospy.loginfo("[manager] 发现新目标 %s @ (%.1f, %.1f)，由 %s 首次检测",
                          tid, msg.x, msg.y, msg.uav_id)

        ct = self.tracker.targets[tid]
        # 把检测的 UAV 登记为 observer，否则 CooperativeTracker 会忽略它的观测
        if msg.uav_id not in ct.observers:
            self.tracker.assign_observers(tid, list(ct.observers) + [msg.uav_id])

        # Only a fresh eligible original camera sample feeds visual continuity.
        # Absolute positioning error is unknown to this node.
        if evidence["eligible"]:
            qualified[key] = sample
            self._qualified_detection_sample = qualified
            self.tracker.report(msg.uav_id, tid, sample, msg.x, msg.y, truth=None)
        self._cur_targets[tid] = (msg.x, msg.y, msg.uav_id)

        # 已有负责人立即更新目标位置；无负责人的目标进入快速联合派遣。
        if tid in self._tracking:
            self._update_tracker_position(tid, msg.x, msg.y)
        # Unowned detections wait at most one 0.2 s batch period. The seeing
        # agent holds its own camera while the manager compares all people.

    def _sanitize_dispatch_xy(self, tx, ty, target_id=""):
        """派机坐标硬护栏：超界坐标夹回地图内。

        === 2026-10-05 比赛规则硬约束 ===
        truth_cache 在 EKF 漂移 / 上报噪声下可能落到地图外（实测 actor 坐标
        偶尔被写到 (135, -81)）。把这种坐标原样发给 agent 会触发 A*
        START_OUT_OF_BOUNDS + 撞墙停止行为。这里做兜底：
        - 落在地图内（含 inset）：原样返回
        - 落在地图外：夹到最近边界内 + log 一次
        - 完全离谱（> 1.5×MAP）：返回 None 让调用方放弃派机
        """
        try:
            bx0, bx1, by0, by1 = _search_bounds()
        except Exception:
            bx0, bx1, by0, by1 = MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX
        # 极端离谱
        if (tx < bx0 * 1.5 or tx > bx1 * 1.5 or
                ty < by0 * 1.5 or ty > by1 * 1.5):
            rospy.logerr_throttle(2.0,
                "[manager] 目标 %s 坐标 (%.1f, %.1f) 极端离谱 > 1.5xMAP，放弃派机",
                target_id, tx, ty)
            return None
        # 轻微越界 -> 夹回
        if (tx < bx0 or tx > bx1 or ty < by0 or ty > by1):
            cx, cy = flight_waypoint(tx, ty)
            rospy.logwarn_throttle(2.0,
                "[manager] 目标 %s 坐标 (%.1f, %.1f) 越过搜索范围，飞行航点夹到 (%.1f, %.1f)",
                target_id, tx, ty, cx, cy)
            return (cx, cy)
        # A target may be visible in the wider search grid, but the aircraft
        # assigned to it must receive an interior flight waypoint.
        return flight_waypoint(tx, ty)

    @serialized_tasks
    def _dispatch_tracker(self, target_id, tx, ty):
        # Kept as a single-target entry for tools; runtime discoveries are
        # batched by the scheduler before backup allocation.
        self._dispatch_target_batch([(target_id, tx, ty)], rospy.Time.now().to_sec())

    @serialized_tasks
    def _dispatch_target_batch(self, pending, now):
        positions = {}
        for tid, tx, ty in pending:
            if tid in self._tracking:
                continue
            clean = self._sanitize_dispatch_xy(tx, ty, tid)
            if clean is not None and self._failed_view_dispatch_ok(tid, *clean, now):
                positions[tid] = clean
        for tid, (tx, ty) in positions.items():
            self._free_camera_backup_for_new_target(tid, tx, ty, now)
        busy = set(self._tracking.values()) | set(getattr(self, '_backup', {}).values())
        available = self._dispatch_available(now, busy)
        costs = self._target_assignment_costs(positions, now, busy, available)
        chosen = joint_target_assignment(costs)
        for tid, uid in sorted(chosen.items()):
            tx, ty = positions[tid]
            self._commit_target_assignment(tid, uid, tx, ty, now)
            rospy.loginfo('[DISPATCH_BATCH] target=%s owner=%s cost_s=%.2f '
                          'pending=%d; 一机一主追', tid, uid, costs[tid][uid], len(positions))

    def _dispatch_available(self, now, busy, require_airborne=False):
        return [uid for uid in self.uav_ids
                if uid not in busy and self._tracking_uav_available(uid, now)
                and (not require_airborne or
                     float(getattr(self.status[uid], 'z', 0.) or 0.) >= 1.)
                and now >= getattr(self, '_release_cooldown', {}).get(uid, rospy.Time(0)).to_sec()]

    def _target_assignment_costs(self, positions, now, busy, available):
        # The whole legal arena is dispatchable. Distance is a cost, not a
        # permanent veto on west-side aircraft helping an east-side target.
        limit = float(os.environ.get('DISPATCH_FAR_LIMIT', '0.0'))
        speed = max(.5, float(os.environ.get('SEARCH_CRUISE_SPEED', '3.0')))
        costs = {}
        for tid, (tx, ty) in positions.items():
            cameras = {uid for uid, _ in self._camera_candidates(tid, tx, ty, now, busy)}
            edges = {}
            for uid in available:
                if not self._failed_view_dispatch_ok(tid, tx, ty, now, uid=uid):
                    continue
                st = self.status[uid]
                distance = math.hypot(tx - st.x, ty - st.y)
                if math.isfinite(distance) and (limit <= 0. or distance < limit):
                    # One second preference for a camera already seeing this
                    # person. Travel is an estimate; agents still plan safely.
                    edges[uid] = distance / speed + (0.0 if uid in cameras else 1.0)
            costs[tid] = edges
        return costs

    def _commit_target_assignment(self, tid, uid, tx, ty, now):
        # Reserve ownership before publishing; callbacks cannot observe an
        # unreserved aircraft between assignment messages.
        self._tracking[tid] = uid
        self._record_approach(tid, uid, tx, ty, now)
        self._free_search_lease(uid)
        tracker = getattr(self, 'tracker', None)
        ct = tracker.targets.get(tid) if tracker is not None else None
        if ct is not None:
            ct.observers.add(uid)
        msg = SearchAssignment()
        msg.header.stamp = rospy.Time.from_sec(now)
        msg.uav_id, msg.target_id, msg.task_type = uid, tid, 1
        msg.cell_ix = msg.cell_iy = -1
        msg.target_x, msg.target_y = flight_waypoint(tx, ty)
        self.assign_pub.publish(msg)

    def _predict_target(self, tid, tx, ty, lead_s):
        """基于过去 1.5 秒位置差分估算 lead_s 秒后 actor 位置。

        国家一等奖标准改进（2026-10-03）：
        - 旧实现：只用最早 / 最晚两条点做线性外推 → actor 突然停下
          或反向时，速度矢量"惯性"使飞机追过头。修复：用最近 1.5s 内的
          中位速度（去掉首尾两个噪声点），对突然的反向突变有抗性。
        - 速度被裁到 PREDICT_ACTOR_VMAX。预测长度被裁到 4.0s 以内。
        - 历史少于 3 条或最近 1.0s 内没新点 → 不外推（避免噪声点）。
        """
        from collections import deque
        now = rospy.Time.now().to_sec()
        h = self._target_hist.setdefault(tid, deque(maxlen=12))
        h.append((tx, ty, now))
        # 去掉超过 2.0s 的旧点（避免全靠很老的数据外推）
        while h and (now - h[0][2]) > 2.0:
            h.popleft()
        if len(h) < 3:
            return tx, ty
        # 计算每相邻两点间的瞬时速度
        speeds = []
        vecs = []
        for i in range(1, len(h)):
            x0, y0, t0 = h[i - 1]
            x1, y1, t1 = h[i]
            dt = max(t1 - t0, 0.05)
            vx_, vy_ = (x1 - x0) / dt, (y1 - y0) / dt
            vecs.append((vx_, vy_))
            speeds.append(math.hypot(vx_, vy_))
        # 用中位速度（去掉头尾 25%）抗噪声
        speeds_sorted = sorted(speeds)
        mid = speeds_sorted[len(speeds_sorted) // 2]
        # 找到速度接近中位的向量（最贴近中位的两条）
        pairs = sorted(zip(speeds, vecs), key=lambda p: abs(p[0] - mid))[:max(1, len(vecs) // 2)]
        vx = sum(v[0] for _, v in pairs) / len(pairs)
        vy = sum(v[1] for _, v in pairs) / len(pairs)
        sp = math.hypot(vx, vy)
        if sp > PREDICT_ACTOR_VMAX:
            scale = PREDICT_ACTOR_VMAX / sp
            vx, vy = vx * scale, vy * scale
        # 限幅：lead_s 上界 4.0s（飞机不可能在 4s 内还没追上）
        lead = min(lead_s, 4.0)
        # 如果中位速度极小（< 0.3m/s），actor 几乎静止 → 不外推
        if mid < 0.3:
            return tx, ty
        return tx + vx * lead, ty + vy * lead

    def _update_tracker_position(self, target_id, tx, ty):
        """更新追踪机的目标位置。

        改进 2（2026-10-03）：actor 移动预测
        真实链路下 actor 持续 2 m/s 移动，飞到 actor 当前位 actor 已跑 8m —— 飞机追不上。
        这里根据"过去 N 秒的位置差分"外推到"飞机按 max_speed 飞过去"这一段时间后，
        actor 的预计位置，让飞机直接飞向"未来位置"，而非"当前位置"。
        """
        if target_id not in self._tracking:
            return
        # 推算"飞机抵达时 actor 大约在哪"——飞机以 MAX_SPEED(6m/s) 飞过去这段时间
        uid = self._tracking[target_id]
        st = self.status.get(uid)
        if st is not None and PREDICT_ACTOR_MOTION:
            d_to = math.hypot(tx - st.x, ty - st.y)
            lead_s = min(d_to / 6.0, 4.0)  # 上限 4s（避免过度预测 actor 反向）
            tx, ty = self._predict_target(target_id, tx, ty, lead_s)
        # === 2026-10-05 比赛规则硬约束：派机坐标硬护栏 ===
        _san = self._sanitize_dispatch_xy(tx, ty, target_id)
        if _san is None:
            return
        tx, ty = _san
        # 发布更新后的追踪任务
        # 2026-10-01 修复：补 target_id（agent 靠它对齐 _orbit_target / 消除清理）
        msg = SearchAssignment()
        msg.header.stamp = rospy.Time.now()
        msg.uav_id = uid
        msg.cell_ix = -1
        msg.cell_iy = -1
        msg.target_x, msg.target_y = flight_waypoint(tx, ty)
        if hasattr(msg, "target_id"):
            msg.target_id = target_id
        msg.task_type = 1  # 目标确认/追踪
        self.assign_pub.publish(msg)
        # 备份机同步更新目标位置，否则它会飞向旧坐标
        bid = self._backup.get(target_id)
        if bid is not None:
            msg2 = SearchAssignment()
            msg2.header.stamp = rospy.Time.now()
            msg2.uav_id = bid
            msg2.cell_ix = -1
            msg2.cell_iy = -1
            msg2.target_x, msg2.target_y = flight_waypoint(tx, ty)
            if hasattr(msg2, "target_id"):
                msg2.target_id = target_id
            msg2.task_type = 1
            self.assign_pub.publish(msg2)

    def _dispatch_backup(self):
        """按本队图像断流、反复短观察及补看超时请求第二视角。

        CooperativeTracker 的 resets/confirm_since 是本队任务健康指标，
        不是裁判计时或坐标准确性的证据；正式节点不读取裁判反馈。
        """
        if not BACKUP_ENABLE:
            return
        now = rospy.Time.now().to_sec()

        needs = self.tracker.needs_backup(now,
                                          reset_thresh=1,
                                          stall_thresh=BACKUP_AFTER)
        recovering = (set(getattr(self, '_view_recovery', {})) |
                      set(getattr(self, '_tenure_recovery', {})))
        needs += [(tid, 'view_recovery') for tid in recovering
                  if tid not in {item[0] for item in needs}]
        if not needs:
            return

        busy = set(self._tracking.values()) | set(self._backup.values())

        for tid, reason in needs:
            if not self._backup_search_capacity():
                break
            if tid in self._backup:
                continue
            # v18：confirm 超时黑名单期内不冗余派机
            if now < self._confirm_timeout_bl.get(tid, 0.0):
                continue
            ct = self.tracker.targets.get(tid)
            if ct is None or ct.eliminated:
                continue
            main = self._tracking.get(tid)
            if main is None:
                continue  # 先派主追，不能给无主目标占第二架机
            primary = self._last_eligible(main, tid)
            last_primary = self._tracking_since.get(tid, now)
            if primary is not None and primary.get('eligible'):
                last_primary = max(last_primary, primary['sample_s'])
            # Fresh images are not evidence of successful coordinate reports.
            # Several *current* short failed streaks warrant a second angle;
            # one old reset must never pull a healthy searcher off its route.
            relief = self._camera_view_needs_relief(tid, now)
            if now - last_primary < BACKUP_STALE and not relief:
                continue
            if len(self._backup) >= BACKUP_MAX:
                continue
            tx, ty = self._get_target_pos(tid, now)
            if tx is None:
                continue
            # === 2026-10-05 比赛规则硬约束：派机坐标硬护栏 ===
            _san = self._sanitize_dispatch_xy(tx, ty, tid)
            if _san is None:
                continue
            tx, ty = _san

            best, best_d, best_seen = None, None, False
            for uid, st in self.status.items():
                if not self._tracking_uav_available(uid, now) or uid in busy:
                    continue
                if now < getattr(self, '_release_cooldown', {}).get(uid, rospy.Time(0)).to_sec():
                    continue
                d = math.hypot(st.x - tx, st.y - ty)
                if (self._sole_first_pass_zone_searcher(uid, busy) and
                        not (relief and d <= BACKUP_MAX_DIST)):
                    continue
                if (d > BACKUP_MAX_DIST and
                        (len(self._observed_target_slots()) < 6 or
                         not self._has_fresh_visual(tid, now))):
                    continue
                visual = self._last_eligible(uid, tid)
                seen = bool(visual is not None and visual.get('eligible') and
                            0.0 <= now - visual['sample_s'] <= 2.0)
                if best_d is None or (seen and not best_seen) or \
                        (seen == best_seen and d < best_d):
                    best, best_d, best_seen = uid, d, seen
            if best is None:
                continue

            # 关键：backup 机也必须登记为 observer，否则 CooperativeTracker 会忽略它的观测
            ct.observers.add(best)
            self._backup[tid] = best
            self._record_approach(tid, best, tx, ty, now, backup=True)
            self._free_search_lease(best)
            busy.add(best)
            tx_c, ty_c = flight_waypoint(tx, ty)
            msg = SearchAssignment()
            msg.header.stamp = rospy.Time.now()
            msg.uav_id = best
            msg.cell_ix = -1
            msg.cell_iy = -1
            msg.target_x = tx_c
            msg.target_y = ty_c
            msg.task_type = 1
            if hasattr(msg, "target_id"):
                msg.target_id = tid
            self.assign_pub.publish(msg)
            rospy.loginfo("[manager] 冗余派机：%s 协同确认 %s @ (%.1f,%.1f), "
                          "距离 %.1fm, 原因=%s, observers=%s",
                          best, tid, tx_c, ty_c, best_d, reason,
                          ",".join(sorted(ct.observers)))

    def _camera_view_needs_relief(self, tid, now):
        """Only own-image task health; never read judge success or actor truth."""
        if tid in getattr(self, '_view_recovery', {}) or tid in getattr(self, '_tenure_recovery', {}):
            return True
        started = getattr(self, '_tracking_since', {}).get(tid, now)
        ct = self.tracker.targets.get(tid)
        if ct is None or now-started < self.tracker.confirm_time+BACKUP_AFTER:
            return False
        since, last = ct.confirm_since, ct.last_ok_t
        # The strict tracker uses accepted original camera timestamps. Its
        # progress is merely a health metric and cannot declare elimination.
        span = 0. if since is None or last is None else max(0., last-since)
        return ct.resets >= 3 and span < BACKUP_AFTER

    def _observed_target_slots(self):
        """Camera-qualified slots seen this run; not official actor identities.

        Losing a view does not make it a never-discovered target again. Keep
        this allocation history separate from all live reporting timestamps.
        """
        known = getattr(self, '_seen_target_slots', set())
        for book in ('_eligible_observations', '_visual_observations'):
            known.update(tid for (_, tid), image in getattr(self, book, {}).items()
                         if image.get('eligible') and tid in ('t0','t1','t2','t3','t4','t5'))
        self._seen_target_slots = known
        return known

    def _backup_search_capacity(self):
        """Reserve searchers for slots never observed, not temporarily out of view."""
        unknown_count = 6 - len(self._observed_target_slots())
        reserve = min(max(0, SEARCH_RESERVE_MAX), unknown_count)
        busy = set(self._tracking.values()) | set(self._backup.values())
        searching = sum(1 for uid, st in self.status.items()
                        if getattr(st, "connected", False) and uid not in busy
                        and float(getattr(st, "z", 0.0) or 0.0) >= 1.0)
        return searching > reserve

    def _sole_first_pass_zone_searcher(self, uid, busy):
        """Do not borrow the only searcher of an unseen sector for redundancy."""
        grid = getattr(self, 'grid', None)
        if grid is None or not hasattr(grid, 'zone_cells'):
            return False
        # Once all six visual slots have supplied eligible camera images,
        # searchers can travel farther to provide another view. Slot history
        # does not establish six official identities or any elimination.
        if len(self._observed_target_slots()) == 6:
            return False
        zone = grid.get_uav_zone(uid)
        visit = getattr(self.allocator, 'visit_time', {})
        grid_visit = getattr(grid, 'visit_time', {})
        active = {u for u, st in self.status.items()
                  if u not in busy and getattr(st, 'connected', False)}
        if any(other != uid and grid.get_uav_zone(other) == zone
               for other in active):
            return False
        for key in grid.zone_cells.get(zone, ()):
            if key in self._blocked_cells:
                continue
            cell = grid.cell(key)
            if cell is None or cell.state == STATE_COVERED:
                continue
            if cell.state == STATE_ASSIGNED and cell.owner in active \
                    and cell.owner != uid:
                return False
            if max(visit.get(key, 0.0), grid_visit.get(key, 0.0)) <= 0.0:
                return True
        return False

    def _tracking_uav_available(self, uid, now):
        """A position-faulted or silent aircraft cannot own a target."""
        if uid is None:
            return False
        st = self.status.get(uid)
        if st is None or not getattr(st, 'connected', False):
            return False
        last = self.last_report.get(uid)
        return last is not None and 0.0 <= now - last.to_sec() < 3.0

    def _cancel_unavailable_owner(self, uid, tid, now):
        """Cancel one failed owner without interrupting a healthy backup."""
        if uid is None:
            return
        self._free_search_lease(uid)
        self._release_cooldown[uid] = rospy.Time(now) + rospy.Duration(6.0)
        msg = SearchAssignment()
        msg.header.stamp = rospy.Time(now)
        msg.uav_id = uid
        msg.cell_ix = -1
        msg.cell_iy = -1
        msg.target_x = 0.0
        msg.target_y = 0.0
        if hasattr(msg, 'target_id'):
            msg.target_id = tid
        msg.task_type = 255
        self.assign_pub.publish(msg)

    def _release_unavailable_trackers(self, now):
        """Free dead tracking owners before the next dispatch in this tick."""
        for tid in set(self._tracking) | set(self._backup):
            main = self._tracking.get(tid)
            backup = self._backup.get(tid)
            main_ok = self._tracking_uav_available(main, now)
            backup_ok = self._tracking_uav_available(backup, now)
            if main_ok and (backup is None or backup_ok):
                continue
            if not main_ok and backup_ok:
                self._tracking[tid] = backup
                self._backup.pop(tid, None)
                self._tracking_since[tid] = now
                getattr(self, '_view_recovery', {}).pop(tid, None)
                getattr(self, '_tenure_recovery', {}).pop(tid, None)
                self._cancel_unavailable_owner(main, tid, now)
                self.tracker.assign_observers(tid, [backup])
                rospy.logwarn('[manager] %s 主追机 %s 失信，备份机 %s 接管',
                              tid, main, backup)
            elif main_ok:
                self._backup.pop(tid, None)
                self._cancel_unavailable_owner(backup, tid, now)
                self.tracker.assign_observers(tid, [main])
                rospy.logwarn('[manager] %s 备份机 %s 失信，主追机 %s 继续',
                              tid, backup, main)
            else:
                self._release_tracker(tid, reason='uav_unavailable')

    @serialized_tasks
    def _update_targets(self):
        """按周期更新每个目标的确认计时，处理规则4/5。"""
        now = rospy.Time.now().to_sec()
        self._release_unavailable_trackers(now)
        self._release_unverified_visuals(now)
        # 先给已知视觉目标分配主追，再考虑备份。
        self._dispatch_pending_targets()
        # 确认期断流/卡住 → 加派第二架协同确认
        self._dispatch_backup()

        # CooperativeTracker.update(now) 内部处理规则 4/5 全部逻辑：
        # 三门槛判定（误差 1m / 间隔 1s / 连续 15s）、confirm_since 重置、
        # 墙钟 25s 瞬移、B1/B2/B3/B4 全修齐。返回 tid → event 字典。
        # 接口适配：CooperativeTracker.update 返回 [(tid, event), ...]，
        # 本管理器按 tid 查事件 -> 转字典（不改 tracker 及其自测契约）
        events = dict(self.tracker.update(now))

        for tid in list(self.tracker.targets.keys()):
            if tid in self._eliminated:
                continue
            ct = self.tracker.targets.get(tid)
            if ct is None:
                continue
            ev = events.get(tid)

            # 这是捕获算法内部的距离，不是 Gazebo 画面或 UAV 真值距离。
            _tpos = self._get_target_pos(tid, now)
            _uid = self._tracking.get(tid) or self._backup.get(tid)
            _st = self.status.get(_uid) if _uid else None
            _dist = None
            if _tpos is not None and _tpos[0] is not None and _st is not None:
                _dist = math.sqrt((_st.x - _tpos[0]) ** 2 +
                                  (_st.y - _tpos[1]) ** 2 + _st.z ** 2)
            _captured = False  # Only official removal establishes success.
            _error_ok = None if ct.last_err is None else ct.last_err <= ct.err_tol
            _gap_s = ((now - ct.last_ok_t) if ct.last_ok_t is not None else None)
            _gap_ok = ct.last_ok_t is not None and 0.0 <= _gap_s <= ct.gap_tol
            self._csv.write(
                ros_time=now, event="visual_streak", auction_cycle=self._auction_cycle,
                target_id=tid,
                target_x=_tpos[0] if _tpos is not None and _tpos[0] is not None else None,
                target_y=_tpos[1] if _tpos is not None and _tpos[1] is not None else None,
                target_z=0.0 if _tpos is not None and _tpos[0] is not None else None,
                uav_id=_uid,
                uav_x=_st.x if _st is not None else None,
                uav_y=_st.y if _st is not None else None,
                uav_z=_st.z if _st is not None else None,
                distance_m=_dist,
                capture_threshold_m=ct.err_tol,
                estimation_error_m=ct.last_err,
                detect_radius_m=DETECT_RADIUS,
                is_captured=_captured,
                assignment="tracking" if _uid else "none",
                tracker_event=ev or "none", progress=ct.progress(now),
                resets=ct.resets, rejects=ct.rejects, covered=ct.covered(now),
                live_observers=ct.n_live(now), gap_s=_gap_s,
                error_ok=_error_ok, gap_ok=_gap_ok,
                source="cooperative_tracker")

            if ev == "confirmed":
                # 这只是本队连续图像样本，绝不等同裁判通过；有新鲜视觉
                # 就继续稳定视角和上报，失去视觉后限时释放飞机补搜。
                self._confirm_done_t[tid] = now
                # 2026-10-06 国家一等奖: 通知 swarm_agent 切贴脸模式
                # (只有当前 _orbit_target == tid 的那架 UAV 会响应)
                try:
                    self.confirm_pub.publish(String("tid:%s" % tid))
                except Exception as e:
                    rospy.logwarn("[manager] confirm_pub.publish 失败: %s", e)
                rospy.loginfo("[manager] 目标 %s 独立图像连续观测 %.0fs（绝对定位误差未验证），"
                              "保留追踪机继续上报，视觉过期后再释放",
                              tid, CONFIRM_TIME)
                # 写一行 CSV 方便复盘
                try:
                    self._csv.write(
                        ros_time=now, event="confirm_held", auction_cycle=self._auction_cycle,
                        target_id=tid,
                        target_x=_tpos[0] if _tpos is not None and _tpos[0] is not None else None,
                        target_y=_tpos[1] if _tpos is not None and _tpos[1] is not None else None,
                        target_z=0.0 if _tpos is not None and _tpos[0] is not None else None,
                        uav_id=_uid,
                        uav_x=_st.x if _st is not None else None,
                        uav_y=_st.y if _st is not None else None,
                        uav_z=_st.z if _st is not None else None,
                        distance_m=_dist,
                        capture_threshold_m=ct.err_tol,
                        estimation_error_m=ct.last_err,
                        detect_radius_m=DETECT_RADIUS,
                        is_captured=False,
                        assignment="tracking" if _uid else "none",
                        tracker_event=ev or "none", progress=ct.progress(now),
                        resets=ct.resets, rejects=ct.rejects, covered=ct.covered(now),
                        live_observers=ct.n_live(now), gap_s=_gap_s,
                        error_ok=_error_ok, gap_ok=_gap_ok,
                        source="cooperative_tracker")
                except Exception:
                    pass
                continue
            if ev == "evade":
                # 位置变化事件仍以新鲜视觉为准；不能猜测裁判内部状态。
                if self._has_fresh_visual(tid, now):
                    rospy.loginfo("[manager] 目标 %s 发现后 %.0fs 仍有新鲜视觉，"
                                  "继续按观测追踪", tid, EVADE_TIME)
                else:
                    rospy.loginfo("[manager] 目标 %s 发现后 %.0fs 视觉已断，"
                                  "清除旧位置等待重捕获", tid, EVADE_TIME)
                    self._confirm_done_t.pop(tid, None)
                    self._cur_targets.pop(tid, None)
                continue
            if ev == "reset":
                rospy.loginfo_throttle(3,
                    "[manager] 目标 %s 本队合格图像连续性中断 → confirm_since 清零；"
                    "官方误差与消除结果未知", tid)

            prog = ct.progress(now)
            if prog > 0:
                cs = ct.confirm_since
                cs_elapsed = ct.last_ok_t - cs if cs is not None and ct.last_ok_t is not None else 0.0
                rospy.loginfo_throttle(5,
                    "[manager] 目标 %s 真实图像连续观测进度 %.0f%% "
                    "(confirm_since=%.1fs, resets=%d, rejects=%d, observers=%s)",
                    tid, prog * 100, cs_elapsed, ct.resets, ct.rejects,
                    ",".join(sorted(ct.observers)))

        # 内部 15s 图像连续不建立消除事实。图像断流后释放租约继续搜，
        # 同一视觉槽再次出现时可重新派机，红色两个槽也独立处理。
        for tid in list(self._confirm_done_t.keys()):
            done_t = self._confirm_done_t.get(tid)
            if done_t is None:
                continue
            fresh = ((tid in self._tracking or tid in self._backup) and
                     self._has_fresh_visual(tid, now))
            # A healthy view gets time to finish the judge's 15 s window,
            # but even a decoy or persistently inaccurate report cannot own
            # an aircraft forever. Rotate/reimage after a bounded tenure.
            if not visual_streak_should_release(done_t, now, fresh):
                continue
            if self._recover_tracking_view(tid, now, 'confirm_hold_timeout'):
                continue
            if fresh:
                self._confirm_timeout_bl[tid] = now + VISUAL_RETRY_COOLDOWN_S
            released_uavs = (self._tracking.get(tid), self._backup.get(tid))
            rospy.logwarn("[manager] %s 视觉观察持有 %.0fs，%s；释放追踪机补搜",
                          tid, now - done_t,
                          "超过单轮观察时限" if fresh else "图像已断流")
            t = self.tracker.targets.get(tid)
            if t is not None:
                t.confirmation_pending = False
                t.confirm_since = None
                t.last_ok_t = None
                t._first_confirm_t = None
            try:
                self._release_tracker(tid, reason="confirm_hold_timeout")
            except Exception:
                pass
            if fresh:
                # Give another camera a chance to reimage from a different
                # angle; do not assign the same nearby UAV right back.
                for uid in released_uavs:
                    if uid is not None:
                        self._release_cooldown[uid] = (
                            rospy.Time.now() +
                            rospy.Duration(VISUAL_RETRY_COOLDOWN_S + 4.0))
            # 清理内部状态,避免下一周期重复触发
            self._confirm_done_t.pop(tid, None)
            self._truth_cache.pop(tid, None)
            self._cur_targets.pop(tid, None)

        # A visible but intermittently unusable person/decoy may never reach
        # the 15 s local streak. Bound that task as well and rotate the view.
        for tid, start in list(getattr(self, '_tracking_since', {}).items()):
            if tid not in self._tracking and tid not in self._backup:
                self._tracking_since.pop(tid, None)
                continue
            if tid in self._confirm_done_t or now - start < TRACKING_MAX_TENURE_S:
                continue
            if self._recover_tracking_view(tid, now, 'tracking_tenure'):
                continue
            self._confirm_timeout_bl[tid] = now + VISUAL_RETRY_COOLDOWN_S
            released_uavs = (self._tracking.get(tid), self._backup.get(tid))
            self._release_tracker(tid, reason="tracking_tenure")
            for uid in released_uavs:
                if uid is not None:
                    self._release_cooldown[uid] = (
                        rospy.Time.now() +
                        rospy.Duration(VISUAL_RETRY_COOLDOWN_S + 4.0))
            rospy.logwarn("[manager] %s 追踪 %.0fs 未形成持续可用图像，"
                          "轮换观察机并继续搜索", tid, now - start)

        # === v18（2026-10-07）confirming 永不收敛超时释放 ===
        # v17 实测：t3 从 116s、t1 从 186s 进入 confirming，直到 436s 比赛结束
        # 既未 confirmed 也未释放 —— 追踪机/备份机被永久占用，叠加租约 renew
        # 循环 bug 后全队 202s 起零分配。用 _first_confirm_t（首次进入确认状态
        # 时刻，仅瞬移重置，不随 reset 清零）做硬超时：超时未消除 → 释放全部
        # 追踪机 + 黑名单期内不重派（等新检测再重新介入，避免 90s 空转循环）。
        _cto = float(os.environ.get("TRACK_CONFIRM_TIMEOUT", "90"))
        for tid, ct in list(self.tracker.targets.items()):
            if ct.eliminated or tid in self._eliminated:
                continue
            _fct = getattr(ct, "_first_confirm_t", None)
            if _fct is None:
                continue
            if (now - _fct) < _cto:
                continue
            if (tid in self._tracking or tid in self._backup) and \
                    self._has_fresh_visual(tid, now):
                continue
            if self._recover_tracking_view(tid, now, 'confirm_timeout'):
                continue
            rospy.logwarn("[manager] 目标 %s 首次确认后 %.0fs 仍未消除（超时 %.0fs）"
                          "→ 视觉已断，释放追踪机并等待重新发现", tid, now - _fct, _cto)
            # A long blacklist suppresses a newly reacquired person for most
            # of the remaining match.  The release cooldown already prevents
            # immediate thrashing; reset the internal timer for a new attempt.
            self._confirm_timeout_bl.pop(tid, None)
            ct._first_confirm_t = None
            try:
                self._release_tracker(tid, reason="confirm_timeout")
            except Exception:
                pass

        # 清空本周期检测缓存（下一周期重新收集）
        self._cur_targets = {}
        self._last_target_t = rospy.Time.now()
        # 国家一等奖 v2（2026-10-05）：即使全队在追踪（_idle_uavs 空 → _allocate
        # 早 return → _reopen_covered_cells 不会被调到）, 也要按 REV_COVER_AGE
        # 主动 reopen covered cells。否则 5 分钟后期所有格子都被标过 COVERED,
        # 接力机返回时没有空闲格可拍, 整个队形停摆到 600s 超时。
        if int(os.environ.get("REOPEN_WHEN_TRACKING", "0")):
            try:
                self._reopen_covered_cells()
            except Exception:
                pass
        # 2026-10-05 国家一等奖修复：5 分钟时间墙保护 —— 每 25s 强制重开一次，
        # 防止"已覆盖格时间戳假阳性"（飞过但没真正看见 LOS 的格被标 COVERED
        # 后永不重扫）造成 actor 漏检。已覆盖时间 >30s 的格被强制 reopen，
        # 与 REV_COVER_AGE 一致；同时调用底层 _reopen_covered_cells 走单个格
        # 独立时间戳判断，比之前「按 mission 时间」更稳。
        if not hasattr(self, "_last_force_reopen"):
            self._last_force_reopen = rospy.Time.now().to_sec()
        _now_s = rospy.Time.now().to_sec()
        if _now_s - self._last_force_reopen > 25.0:
            self._last_force_reopen = _now_s
            try:
                if hasattr(self.grid, "_reopen_covered_cells") and not any(
                        c.state in (STATE_FREE, STATE_REVIEW)
                        for key, c in self.grid.cells.items()
                        if key not in self._blocked_cells):
                    n = self.grid._reopen_covered_cells(max_age=30.0)
                    if n:
                        rospy.loginfo("[manager] 5 分钟时间墙强制重开 %d 个老格", n)
            except Exception:
                pass

    # ---------------- 拍卖分配 ----------------
    def _idle_uavs(self):
        """返回「空闲机」列表：无活跃任务（没有 owner==自己的 STATE_ASSIGNED 格，且不在追踪目标）。

        这样可避免一台机还在飞往旧格途中就被重复分配新格、旧格无人接管。
        机完成某格（confidence≥1.0 → 格变 STATE_COVERED，owner 清空）后自动变空闲。
        正在追踪目标的机也不应被分配搜索格。
        """
        idle = []
        # === 2026-10-05 国家一等奖修：追踪释放后冷却期跳过 ===
        # LEASE_RELEASE_COOLDOWN_S：刚释放追踪的飞机需冷却 N 秒，等 EKF 收敛、
        # 当前速度归零、A* 路径清空，才能接新任务。否则会立刻被派到 100m+
        # 的同一格，飞机在原地 EKF 闸门 + 路径抖动里卡死。
        _now = rospy.Time.now()
        _cool = float(os.environ.get("LEASE_RELEASE_COOLDOWN_S", "6.0"))
        for uid in self.uav_ids:
            if uid not in self.status or not self.status[uid].connected:
                continue
            # 2026-10-07 五分钟冲刺: 起飞稳定门槛 — z<1.0 说明还在地面 EKF 预热/
            # 解锁爬升阶段, 此时派 100m+ 远格会让 A* 在 EKF 抖动里反复重规划卡死.
            # z>1.0 = 已离地稳定爬升, assignment 会缓存到主循环接管.
            if float(getattr(self.status[uid], 'z', 0.0) or 0.0) < 1.0:
                continue
            # 冷却期内（除非其他机都不在，否则这只机先不接任务）
            _cd_until = self._release_cooldown.get(uid)
            if _cd_until is not None and _now < _cd_until:
                continue
            # 如果正在追踪目标，视为不空闲
            if uid in self._tracking.values() or uid in self._backup.values():
                continue
            busy = False
            for c in self.grid.cells.values():
                if c.state == STATE_ASSIGNED and c.owner == uid:
                    busy = True
                    break
            if not busy:
                idle.append(uid)
        return idle

    @serialized_tasks
    def _allocate(self):
        """只给空闲机拍卖分配未搜索格 → 授租约 → 发布 assignment。"""
        now = rospy.Time.now()

        # 到期租约先回退（掉线/卡死的机占用的格释放回候选池）
        expired = self.lease.expire(now.to_sec())
        if expired:
            rospy.loginfo("[manager] 租约到期重分配 %d 格", len(expired))
            # v13：过期格写入 allocator.failed_visit（utility 60s 内压 novelty 0.2），
            # 打断「追踪打断搜索→租约过期→同格无限续派」循环（agent_1 同格 32 次）
            for key in expired:
                self.allocator.failed_visit[key] = now.to_sec()
            for uid, key in list(self._active_leases.items()):
                if key in expired:
                    self._active_leases.pop(uid, None)
                    self._lease_hold_since.pop((uid, key), None)
                    self._lease_motion.pop(uid, None)

        # 建筑内格中心标记为 STATE_COVERED（agent 无法到达，视为无需搜索）
        for key in self._blocked_cells:
            c = self.grid.cell(key)
            if c is not None and c.state == STATE_FREE:
                c.state = STATE_COVERED

        # 只对空闲机分配
        idle = self._idle_uavs()
        if not idle:
            return

        uavs = {uid: (self.status[uid].x, self.status[uid].y) for uid in idle}

        # Visibility is credited only by _processed_camera_frame_cb. A UAV
        # status or completed scan has no camera yaw/pitch evidence, so using
        # its XY position here would falsely erase unseen building sides.

        # 拍卖（每机取剩余格中自身效用最大者，不重复）
        assign = self.allocator.allocate(uavs)
        self._auction_cycle += 1
        for _d in self.allocator.last_allocation:
            _key = _d["cell_key"]
            self._csv.write(
                ros_time=now.to_sec(), event="auction", auction_cycle=self._auction_cycle,
                target_id=("cell_%d_%d" % _key) if _key is not None else None,
                target_x=_d["target_x"], target_y=_d["target_y"], target_z=None,
                uav_id=_d["uav_id"], uav_x=_d["uav_x"], uav_y=_d["uav_y"],
                uav_z=self.status[_d["uav_id"]].z if _d["uav_id"] in self.status else None,
                distance_m=_d["distance_m"], capture_threshold_m=DETECT_RADIUS,
                estimation_error_m=None,
                detect_radius_m=DETECT_RADIUS,
                is_captured=False,
                assignment=("cell_%d_%d" % _key) if _key is not None else "none",
                cost=_d["utility"], covered=None, live_observers=None,
                gap_s=None, error_ok=None, gap_ok=None, source="task_allocator")
        all_assigned = True
        for uid, key in assign.items():
            if key is None:
                # 无可用格，不发布任务
                all_assigned = False
                continue
            # A previous cell may have become COVERED through mark_seen
            # without passing the arrival path.  Its inactive lease timer
            # must not carry into a later assignment of the same cell.
            old_key = self._active_leases.pop(uid, None)
            self._lease_motion.pop(uid, None)
            if old_key is not None:
                self.lease.force_expire(uid, old_key)
            for hold_key in list(self._lease_hold_since):
                if hold_key[0] == uid:
                    self._lease_hold_since.pop(hold_key, None)
            # 授租约（自适应时长：飞行时间 × 1.5 + 10s 缓冲）
            uav_pos = uavs[uid]
            cell = self.grid.cell(key)
            _wp0 = self._search_waypoint(key, cell)
            dist = math.hypot(_wp0[0] - uav_pos[0], _wp0[1] - uav_pos[1])
            duration = max(10.0, dist / float(os.environ.get("LEASE_CRUISE_SPEED", "5.0")) * 1.5 + 10.0)  # LEASE_CRUISE_SPEED 默认 5.0（与 agent MAX_SPEED 对齐：原 3.0 把租约算长 67%, 接力损耗严重）
            self.lease.grant(uid, key, now.to_sec(), duration)
            self._active_leases[uid] = key
            self._lease_hold_since[(uid, key)] = now.to_sec()
            self._lease_motion[uid] = (key, now.to_sec(), uav_pos[0], uav_pos[1])
            rospy.loginfo("[manager] 分配 %s → 格 (%d,%d) 飞行距离 %.1fm 租约 %.1fs",
                          uid, key[0], key[1], dist, duration)
            self._publish_assignment(uid, key, cell)

        # 无裁判反馈时，视觉槽全消失也不能证明六名 actor 已消除。
        # 无格可分时重开已观察区域，持续补漏直到裁判终止比赛。
        if not all_assigned:
            rospy.loginfo_throttle(
                20, "[manager] 暂无可分配搜索格；无裁判反馈可证明任务完成，继续复查")
            self._reopen_covered_cells(force=True)

        self._last_alloc_t = now

    # ---------------- 续租 ----------------
    @serialized_tasks
    def _renew_leases(self):
        """对持续上报的机续租（防误判到期）。

        Arrival and completed scan release the lease. Only actual processed
        camera footprints may mark cells covered; unseen cells stay in review.
        """
        now = rospy.Time.now()
        for uid in self.uav_ids:
            if uid not in self.last_report:
                continue
            # 3 秒内无上报 → 不续租，等 _allocate 里 expire 回收
            if (now - self.last_report[uid]).to_sec() >= 3.0:
                continue
            key = self._active_leases.get(uid)
            if key is None:
                continue
            st = self.status.get(uid)
            if st is None or not getattr(st, 'connected', False):
                # agent 的飞控链路可能仍在，但 EKF 熔断/坠机恢复时它会
                # 上报 connected=False。此时不许用漂移坐标判到格，也不应
                # 继续续租占住搜索资源。
                self.lease.force_expire(uid, key)
                self._active_leases.pop(uid, None)
                self._lease_hold_since.pop((uid, key), None)
                getattr(self, '_lease_motion', {}).pop(uid, None)
                rospy.logwarn('[manager] %s 暂不可执行任务，回收格 (%s,%s) 租约',
                              uid, key[0], key[1])
                continue
            if (getattr(st, 'assigned_cell_ix', -1),
                    getattr(st, 'assigned_cell_iy', -1)) != key:
                # 状态消息可能比新指派早一帧；旧格的扫描完成证明不能用于新格。
                continue
            if getattr(st, 'confidence', 0.0) < 0.0:
                self.lease.force_expire(uid, key)
                self._active_leases.pop(uid, None)
                self._lease_hold_since.pop((uid, key), None)
                getattr(self, '_lease_motion', {}).pop(uid, None)
                self.allocator.failed_visit[key] = now.to_sec()
                rospy.logwarn('[manager] %s 搜索格 (%s,%s) 观察失败，回收重派',
                              uid, key[0], key[1])
                continue
            # 到达判定：用格的可达代表点（楼边格中心可能在建筑内）
            # Normal reachable waypoints need no cell lookup. In particular,
            # status-only lease checks can run before the coverage grid is
            # ready; a blind revisit always has a real coverage cell.
            _wp = (None if key in getattr(self, '_blind_viewpoints', {})
                   else self._cell_waypoint.get(key))
            if _wp is None:
                _c = self.grid.cell(key)
                _wp = self._search_waypoint(key, _c) if _c is not None else None
            if _wp is not None and st is not None:
                _wp = flight_waypoint(*_wp)
                _d = math.hypot(st.x - _wp[0], st.y - _wp[1])
                if _d < COVER_ARRIVE_M and getattr(st, 'confidence', 0.0) >= 1.0:
                    # Completion means that four camera headings were
                    # processed, not that the ground directly beneath the
                    # aircraft was in a horizontal camera. Cell coverage is
                    # credited only by _processed_camera_frame_cb, which
                    # checks the vertical/horizontal view and SLAM LOS.
                    observed = self.grid.cell(key).state == STATE_COVERED
                    self.lease.force_expire(uid, key)
                    if not observed:
                        self.grid.mark_review(key, 0.0)
                        side = self._choose_blind_viewpoint(
                            key, self.grid.cell(key), (st.x, st.y))
                        if side is not None:
                            if not hasattr(self, '_blind_viewpoints'):
                                self._blind_viewpoints = {}
                            self._blind_viewpoints[key] = side
                            rospy.loginfo('[manager] 格 %s 未成像：下一次从侧向 (%.1f,%.1f) 补看',
                                          key, side[0], side[1])
                    allocator = getattr(self, 'allocator', None)
                    if hasattr(allocator, 'complete_waypoint'):
                        allocator.complete_waypoint(
                            uid, key, observed=observed, now=now.to_sec())
                    if not observed and hasattr(allocator, 'queue_revisit'):
                        cell = self.grid.cell(key)
                        allocator.queue_revisit(
                            'blind', cell.cx, cell.cy, now=now.to_sec(),
                            delay_s=8.0, ttl_s=90.0,
                            blocked=self._blocked_cells)
                    self._active_leases.pop(uid, None)
                    self._lease_hold_since.pop((uid, key), None)
                    getattr(self, '_lease_motion', {}).pop(uid, None)
                    rospy.loginfo("[manager] %s 完成格 (%s,%s) 扫描，图像覆盖=%s，释放租约",
                                  uid, key[0], key[1], observed)
                    continue
            # A distant assignment needs its full flight time, but a drone
            # circling a wall must not hold its sector for the entire 90 s cap.
            # Displacement alone is not progress: a 2 m sideways/backward
            # move used to reset this timer indefinitely. Require the
            # remaining distance to the reachable waypoint to shrink.
            motion = getattr(self, '_lease_motion', None)
            if motion is not None and _wp is not None:
                sample = motion.get(uid)
                if sample is None or sample[0] != key:
                    motion[uid] = (key, now.to_sec(), st.x, st.y)
                elif (math.hypot(sample[2] - _wp[0], sample[3] - _wp[1]) -
                      _d >= LEASE_STALL_PROGRESS_M):
                    motion[uid] = (key, now.to_sec(), st.x, st.y)
                elif now.to_sec() - sample[1] >= LEASE_STALL_S:
                    self.lease.force_expire(uid, key)
                    self._active_leases.pop(uid, None)
                    self._lease_hold_since.pop((uid, key), None)
                    motion.pop(uid, None)
                    self.allocator.failed_visit[key] = now.to_sec()
                    rospy.logwarn('[manager] %s 搜索格 (%s,%s) %.0fs 未向航点推进 %.1fm，提前回收重派',
                                  uid, key[0], key[1], LEASE_STALL_S, LEASE_STALL_PROGRESS_M)
                    continue
            # 2026-10-07 v12: 最长持有时限 —— 到达判定只放行"真到了"的机，
            # 卡死机（爬升闸门没开/建筑线刹停）会持续上报被无条件续租，
            # 租约永不到期 → 全场只派首轮 6 格后死锁。超时停止续租，
            # 租约按 TTL 到期回收，格子回候选池重新拍卖（可能换机/换格）。
            _hk = (uid, key)
            _since = self._lease_hold_since.setdefault(_hk, now.to_sec())
            if now.to_sec() - _since > LEASE_MAX_HOLD:
                self._lease_hold_since.pop(_hk, None)
                # v18（2026-10-07）：直接强制回收租约。旧逻辑只「停止续租等 TTL
                # 到期回收」，但 pop 后下一周期 setdefault 重置计时又恢复续租 →
                # TTL 永远追不上（v17 实测 agent_0 (3,7) 每 91s 循环告警一次、
                # 全程 0 次「租约到期重分配」、6 架全被 ASSIGNED 格锁死 →
                # 202s 后 _idle_uavs 恒空、零分配死锁）。
                expired = self.lease.force_expire(uid, key)
                self._active_leases.pop(uid, None)
                getattr(self, '_lease_motion', {}).pop(uid, None)
                if expired:
                    self.allocator.failed_visit[key] = now.to_sec()
                    rospy.logwarn("[manager] %s 持有格 (%s,%s) 超 %.0fs 未抵达 → "
                                  "强制回收租约重派", uid, key[0], key[1], LEASE_MAX_HOLD)
                continue
            if not self.lease.renew(uid, key, now.to_sec()):
                # Another coverage observation already removed ownership.
                # Keeping the stale active key made its hold timer trigger
                # repeated "90s" failures immediately after new grants.
                self._active_leases.pop(uid, None)
                self._lease_hold_since.pop((uid, key), None)
                getattr(self, '_lease_motion', {}).pop(uid, None)
        self._last_lease_t = now

    # ---------------- 发布 ----------------
    def _publish_assignment(self, uid, key, cell):
        msg = SearchAssignment()
        msg.header.stamp = rospy.Time.now()
        msg.uav_id = uid
        msg.cell_ix = key[0]
        msg.cell_iy = key[1]
        _wp = self._search_waypoint(key, cell)
        msg.target_x, msg.target_y = flight_waypoint(*_wp)
        msg.task_type = 0  # 搜索
        self.assign_pub.publish(msg)
        rospy.loginfo("[manager] 分配 %s → 格 (%d,%d) 中心 (%.1f,%.1f)",
                      uid, key[0], key[1], cell.cx, cell.cy)

    def _publish_rtl(self, uavs):
        """发布 RTL 返航任务（发给所有连接的飞机，不只是 idle 的）

        注意：有剩余目标时不设置 _mission_finished，因为还要继续跟踪目标确认计时
        """
        # 不设置 _mission_finished = True，因为还有剩余目标需要追踪
        # 发给所有连接的飞机，不只是 idle 的
        rtl_uavs = [uid for uid in self.uav_ids if uid in self.status and self.status[uid].connected]
        for uid in rtl_uavs:
            msg = SearchAssignment()
            msg.header.stamp = rospy.Time.now()
            msg.uav_id = uid
            msg.cell_ix = -1
            msg.cell_iy = -1
            msg.target_x = 0.0
            msg.target_y = 0.0
            msg.task_type = 2  # RTL
            self.assign_pub.publish(msg)
        rospy.loginfo("[manager] RTL 返航: %s", rtl_uavs)

    def _publish_land(self, uavs):
        """发布降落任务（发给所有连接的飞机，不只是 idle 的）

        全部任务完成，设置 _mission_finished = True
        """
        self._mission_finished = True
        # 发给所有连接的飞机，不只是 idle 的
        land_uavs = [uid for uid in self.uav_ids if uid in self.status and self.status[uid].connected]
        for uid in land_uavs:
            msg = SearchAssignment()
            msg.header.stamp = rospy.Time.now()
            msg.uav_id = uid
            msg.cell_ix = -1
            msg.cell_iy = -1
            msg.target_x = 0.0
            msg.target_y = 0.0
            msg.task_type = 3  # 降落
            self.assign_pub.publish(msg)
        rospy.loginfo("[manager] 降落: %s", uavs)

    # ---------------- 主循环 ----------------
    def run(self):
        rate = rospy.Rate(10)
        rospy.loginfo("[manager] 集群管理器启动，机队: %s", self.uav_ids)
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            try:
                # 任务已完成，跳过分配
                if self._mission_finished:
                    rate.sleep()
                    continue

                # 定期续租
                if (now - self._last_lease_t).to_sec() >= LEASE_CHECK_PERIOD:
                    self._renew_leases()

                if now.to_sec() - getattr(self, '_last_dispatch_s', -1e18) >= 0.2:
                    self._dispatch_pending_targets()
                    self._last_dispatch_s = now.to_sec()

                # 目标确认计时（规则4/5）
                if (now - self._last_target_t).to_sec() >= TARGET_CHECK_PERIOD:
                    self._update_targets()

                # 合规 SLAM：栅格有更新 → 周期重建阻塞格/航点/临楼度
                if self._slam_dirty and \
                        (now.to_sec() - self._slam_rebuild_t) >= SLAM_REBUILD_SEC:
                    self._slam_rebuild_t = now.to_sec()
                    self._slam_dirty = False
                    self._rebuild_blocked_cells()

                # 定期拍卖（有未覆盖格且距上次分配超周期）
                if (now - self._last_alloc_t).to_sec() >= ALLOC_PERIOD:
                    self._allocate()

                # ===== DEBUG 2026-10-05：5min 复盘日志，每 10s 仿真打一行 =====
                if not hasattr(self, "_dbg_last_t"):
                    self._dbg_last_t = now
                if (now - self._dbg_last_t).to_sec() >= 10.0:
                    self._dbg_last_t = now
                    try:
                        # These are only internal visual slots, not judge score.
                        _tu = self.tracker.targets if hasattr(self, "tracker") and self.tracker is not None else {}
                        active_tids = sorted([tid for tid in _tu
                                              if self._has_fresh_visual(tid, now.to_sec())])
                        rospy.loginfo("[DEBUG-10s] sim=%.1f visual_active=%s "
                                      "tracked=%s total_visual_slots=%d",
                                      now.to_sec(), active_tids,
                                      sorted(self._tracking), len(_tu))
                    except Exception as _e:
                        rospy.logwarn("[DEBUG-10s] 复盘日志出错: %s", _e)
            except Exception as e:
                rospy.logerr("[manager] 主循环异常: %s\n%s", e, traceback.format_exc())

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("swarm_manager")
    uav_ids = rospy.get_param("~uav_ids", DEFAULT_UAV_IDS)
    if isinstance(uav_ids, str):
        uav_ids = uav_ids.split(",")
    rospy.loginfo("swarm_manager 启动，机队: %s", uav_ids)
    SwarmManager(uav_ids).run()
