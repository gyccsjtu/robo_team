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
from swarm_task import LineOfSight
import re
import traceback

from robocup_swarm.msg import UavStatus, SearchAssignment, TargetState, TargetDetection
from std_msgs.msg import String, Float32

# 直接 import 同目录的纯逻辑模块（scripts 目录已加进 PYTHONPATH）
from swarm_task import (CoverageGrid, TaskAllocator, LeaseManager,
                        STATE_FREE, STATE_ASSIGNED, STATE_COVERED,
                        W_GAIN, W_FLIGHT, W_OVERLAP, W_RISK, W_BALANCE, W_DISTANCE, W_ZONE, LEASE_DURATION,
                        CONFIRM_TIME, EVADE_TIME, DETECT_RADIUS, DETECT_RADIUS_MARGIN)
from cooperative_tracker import CooperativeTracker, CONFIRM_HOLD_TIMEOUT
from csv_logger import logger

# 地图（用于过滤「格中心落在建筑内」的格子，与 agent 的 A* 膨胀判定一致）
from robocup_navigation.astar import load_metadata, GridMap

# ============================ 参数 ============================
MAP_X_MIN, MAP_X_MAX = -100.0, 100.0
MAP_Y_MIN, MAP_Y_MAX = -50.0, 50.0
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
BACKUP_MAX_DIST = float(os.environ.get("BACKUP_MAX_DIST", "60.0"))  # 距离超过此值就不派
                                                                    # （飞过去的时间比等还久，净亏损）

# ---- 真值播种开关 ----
# 1=把官方 actor 真值直接播种给 tracker（现状，等于"上帝视角"直接派机）
# 0=只能靠 /swarm/detection（飞机自己的观测）发现目标 —— 这才是接 YOLO 后的真实链路
# 关掉它才能量出协同搜索算法的真实能力，否则覆盖栅格/拍卖/分区全是摆设。
SEED_TRUTH = int(os.environ.get("SEED_TRUTH", "1"))
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
TRUTH_TTL = float(os.environ.get("TRUTH_TTL", "8.0"))  # 延长缓存时间，减少因网络延迟导致的误判
# 格中心落在建筑里时，去这些半径的环上找可达的替代航点（m）
WAYPOINT_RING = (2.0, 3.5, 5.0)
CLEARANCE_MAX = float(os.environ.get("CLEARANCE_MAX", "24.0"))  # 距离场最大计算范围 m

# ---- LOS 感知 next-best-view（2026-09-28）----
# 与 W_EDGE 的区别：EDGE 是静态的「贴楼加分」（实测成共同吸引子，6 架挤一起）；
# NBV 是动态的「从这个视点能看见多少久未见的区域」，且派单即承诺 → 次模贪心自动分散。
VIS_ENABLE = os.environ.get("VIS_ENABLE", "1") not in ("0", "false", "False", "")
VIS_RADIUS = float(os.environ.get("VIS_RADIUS", "10.0"))   # 与 DETECT_RADIUS 一致（2026-10-05: 20→10，同步收紧）
VIS_COMMIT = os.environ.get("VIS_COMMIT", "1") not in ("0", "false", "False", "")
# 飞机抵达分配格的判定半径：距格代表点 < 此值即认为已搜索该格，
# 标记可视域覆盖并释放租约。太小会在格边来回振荡不释放，太大则还没飞到就算覆盖。
COVER_ARRIVE_M = float(os.environ.get("COVER_ARRIVE_M", "4.0"))  # 减小到达判定半径，适应7m格子
# 2026-10-07 v12: 租约最长连续持有时限（秒）。续租条件原为"3 秒内有上报"，
# 卡死/打不开爬升闸门的机持续上报 → 无条件续租 → 租约永不到期 → 全场只派
# 首轮 6 格后死锁（实测 sim 88s 后零新分配，agent1/5 悬停起飞点全程）。
# 超时不续租 → 租约按 TTL 到期回收 → 格子回候选池重新拍卖，形成调度周转。
LEASE_MAX_HOLD = float(os.environ.get("LEASE_MAX_HOLD", "90"))
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


class SwarmManager(object):
    def __init__(self, uav_ids):
        self.uav_ids = uav_ids
        # 每机最新状态
        self.status = {}          # uav_id -> UavStatus
        # 每机最近一次上报时间（用于租约续租判定）
        self.last_report = {}     # uav_id -> rospy.Time
        self._active_leases = {}  # uav_id -> cell key currently leased to that UAV
        self._lease_hold_since = {}  # (uid, key) -> 首次续租时刻，用于 LEASE_MAX_HOLD
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
        self._slam_cells = None    # 多机 SLAM 占用并集 bytearray（首帧上报时按尺寸初始化）
        self._slam_w = self._slam_h = 0
        self._slam_res = 0.5
        self._slam_origin = (0.0, 0.0)
        self._slam_dirty = False
        self._slam_rebuild_t = 0.0
        self._cell_waypoint = {}   # cell_key -> (wx,wy) 楼边格的可达代表点
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
        self.tracker = CooperativeTracker()
        self._truth_cache = {}     # target_id -> (x, y, t) 缓存，t 为检测上报时间，超 TRUTH_TTL 作废
        self._truth_pos = {}      # target_id -> (x, y) 仅真值源写入（SEED_TRUTH=1），tracker 误差门槛用
        self._cur_targets = {}    # target_id -> 本周期是否有检测（用于 lose 判定）
        self._eliminated = set()
        # 确认期冗余备份机：tid -> uav_id（见 _dispatch_backup）
        self._backup = {}
        # v18：confirm 超时黑名单（tid -> 黑名单截止仿真秒），期内不重派追踪机
        self._confirm_timeout_bl = {}
        # tid -> 最近一次收到 /swarm/detection 的 wall 时刻（断流检测用）
        self._last_detect = {}
        # tid -> 官方 /find_actor_N 首次发布时刻（= 进入 15s 确认期）
        self._confirm_since = {}
        # tid -> 团队侧已连续确认 15s 的时刻（"规则5 团队侧已确认"）
        # 区别于 _confirm_since：后者是"官方 find_actor_N 进入 15s 倒计时";
        # 前者是"我们已经按误差<1m+间隔<1s 持续报满 15s,可向官方申请消除"。
        # 关键修复（2026-10-05 击毁 0 分 bug）：不在这里立刻调 _release_tracker
        # 也不再发 "eliminate:%s" 给 target_sim_node，而是等官方 /left_actors 真
        # 确认后由 _left_cb → _release_finished 统一释放。详见 _update_targets
        # "confirmed" 分支的注释。
        self._confirm_done_t = {}
        self._backup_noted = set()  # 已提示过「无近机可派」的 tid（避免刷屏）
        self._tracking = {}       # target_id -> assigned_uav_id 当前追踪任务
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
        # 合规 SLAM：各 agent 的雷达占用栅格快照（RLE JSON），合并成全局地图
        rospy.Subscriber("/swarm/occupancy_grid", String, self._occ_grid_cb, queue_size=6)
        self.assign_pub = rospy.Publisher("/swarm/assignment", SearchAssignment, queue_size=10)
        # 消除指令发给 target_sim_node（"eliminate:<target_id>"）
        self.cmd_pub = rospy.Publisher("/swarm/target_command", String, queue_size=10)
        # 任务完成广播
        self.finish_pub = rospy.Publisher("/swarm/finish", String, queue_size=10)
        # 2026-10-06 国家一等奖: 「团队侧已确认」贴脸模式广播
        # swarm_agent 收到且匹配自己 _orbit_target == tid 时, 把
        # ORBIT_RADIUS 从 6m 切到 2m, ORBIT_SPEED 从 0.13 切到 0.05 rad/s,
        # 让 yolo 误差 < 1m 持续 15s, 触发官方 score_cal.py:122-168 的
        # err_threshold=1m 判定 → +100 消除分 (官方"未完成计分"公式).
        self.confirm_pub = rospy.Publisher("/swarm/confirmed", String, queue_size=10)

        # 官方裁判的剩余 actor 清单（权威）。
        # 用途：兜住「提前收工」死结 —— manager 的 tracker 只在收到 /swarm/detection 后
        # 才登记目标，于是「一个都还没发现」时 remaining_targets 恒为空，格子一刷完就
        # 广播 MISSION_FINISHED、全体降落，从此再也不可能发现任何目标。
        self._left_actors = []
        self._left_seen = False   # 是否已收到过第一条 /left_actors（防空清单误释放）
        rospy.Subscriber("/left_actors", String, self._left_cb, queue_size=5)
        # 官方 /find_actor_N（Float32）：首次发布 = 该 actor 进入 15s 连续确认期
        for _i in range(6):
            rospy.Subscriber("/find_actor_%d" % _i, Float32,
                             self._make_find_cb(_i), queue_size=5)
        # 官方 actor 真值（official_target_bridge 从 /gazebo/model_states 转发）
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
                    ctr = g.world_to_cell((c.cx, c.cy))
                    wp = (c.cx, c.cy) if (ctr is not None and g.is_free(ctr)) else None
                    if wp is None:
                        for r in WAYPOINT_RING:
                            for a in range(12):
                                th = a * math.pi / 6.0
                                px = c.cx + r * math.cos(th)
                                py = c.cy + r * math.sin(th)
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
                    cell = g.world_to_cell((c.cx, c.cy))
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
        """合并 agent 上报的 SLAM 占用栅格（多机并集）→ 触发阻塞格重建。"""
        try:
            d = json.loads(msg.data)
            w, h = int(d["width"]), int(d["height"])
            res = float(d["resolution"])
            origin = (float(d["origin"][0]), float(d["origin"][1]))
            rle = d["rle"]
        except Exception:
            return
        # 先验 RLE 总长再写入：坏帧整帧丢弃，防半帧污染占用并集
        total = 0
        for run in rle:
            try:
                total += int(run[1])
            except Exception:
                return
        if total != w * h:
            return
        with self._slam_lock:
            if self._slam_cells is None:
                self._slam_w, self._slam_h = w, h
                self._slam_res, self._slam_origin = res, origin
                self._slam_cells = bytearray(w * h)
            elif w != self._slam_w or h != self._slam_h:
                return  # 尺寸不一致（异构地图/旧节点），丢弃
            idx = 0
            for run in rle:
                v, c = int(run[0]), int(run[1])
                if v:
                    self._slam_cells[idx:idx + c] = b"\x01" * c
                idx += c
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

    # ---------------- 回调 ----------------
    def _status_cb(self, msg):
        self.status[msg.uav_id] = msg
        self.last_report[msg.uav_id] = rospy.Time.now()

    @staticmethod
    def _tid_to_actor(tid):
        """'t3' -> 3；非 tN 形式返回 None。"""
        m = re.match(r'^t(\d+)$', str(tid))
        return int(m.group(1)) if m else None

    def _make_find_cb(self, actor_id):
        """官方 /find_actor_N 回调 → 对齐 CooperativeTracker 的规则4 计时起点。

        manager 自己的 _confirm_since 字典已被 CooperativeTracker.targets[tid]._first_confirm_t
        取代，后者才是规则4 真正用的计时起点。
        """
        def cb(msg):
            now = rospy.Time.now().to_sec()
            tid = "t%d" % actor_id
            self.tracker.mark_confirmed(tid, now)
            rospy.loginfo("[manager] 官方首次发现 %s → 规则4 计时起点对齐 %.1f", tid, now)
        return cb

    def _release_finished(self, left_ids):
        """官方已确认并从场上删除的 actor → 立刻释放它的追踪机。

        背景（第六轮实测故障）：agent 连续确认满 15s 后只在自己日志里打印
        「目标 t5 已连续确认，可消除」，manager 的 tracker 却没有把它推进
        _eliminated。后果是 _tracking 里 t3/t4/t5（早已被官方删掉的 actor）
        一直占着飞机 —— 而 _idle_uavs() 把「正在追踪」的机视为不空闲，
        这些机永远拿不到新任务，只能对着旧坐标空转；唯一剩下的 actor_1
        （/left_actors="[1]"）反倒没有飞机去追，官方收不到检测，任务卡到超时。

        国家一等奖改进（2026-10-03）：统一走 _release_tracker 发布取消
        指令（含 cancel msg + hot target 清理），让 agent 立刻转回搜索
        任务，而不是继续对旧坐标盘旋。
        """
        for tid in list(self._tracking.keys()):
            aid = self._tid_to_actor(tid)
            if aid is None or aid in left_ids:
                continue
            bu = self._backup.pop(tid, None)
            self._eliminated.add(tid)
            t = self.tracker.targets.get(tid)
            if t is not None:
                t.eliminated = True
            # 统一释放（pop + cancel msg + hot target 清理）
            self._release_tracker(tid, reason="left_actors=%s" % list(left_ids))
            # 官方已确认 → 清掉团队侧的"等官方"时间戳,避免超时兜底再次触发
            self._confirm_done_t.pop(tid, None)
        # 顺带把 tracker 里同样已消除、但当时没派机的目标也标掉
        for tid in list(self.tracker.targets.keys()):
            aid = self._tid_to_actor(tid)
            if aid is not None and aid not in left_ids:
                self._eliminated.add(tid)
                self._confirm_done_t.pop(tid, None)

    def _left_cb(self, msg):
        """官方裁判发布的剩余 actor：'[]' 或 '[0, 2, 5]'。"""
        ids = [int(x) for x in re.findall(r'-?\d+', str(msg.data))]
        if ids != self._left_actors:
            rospy.loginfo("[manager] 官方剩余 actor: %s", ids)
        self._left_actors = ids
        # 第一条之前不动作：空清单不等于「全部已消除」
        if not self._left_seen:
            if ids:
                self._left_seen = True
            return
        self._release_finished(ids)

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
            wp = self._cell_waypoint.get(k) or (cells[k].cx, cells[k].cy)
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

    def _reopen_covered_cells(self):
        """所有格都已标记覆盖、但目标还没找全时，把覆盖状态重置重新扫一遍。

        不加这一步的话：日志会说"继续搜索"，但没有未覆盖格可分配，
        飞机实际上拿不到新任务，只能原地待命到超时 —— 等于还是收工。

        国家一等奖标准改进（2026-10-03）：
        - 同时 reopen_review()：超时未复查的 REVIEW 格强制回 FREE，
          避免「已知 actor 区域曾低置信扫过、再也没有机会复查」漏报。
        - 让底层 grid._reopen_covered_cells 走每个格独立的 covered_t
          时间戳判断（避免「绝对 mission 时间」导致 60s 后再不重置）。
        """
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

    def _truth_cb(self, msg):
        """actor 位置（真值桥或 YOLO 桥）→ 缓存位置 + 给 CooperativeTracker 播种目标 ID。

        P0 修复（2026-10-01）：SEED_TRUTH 开关此前定义了但在此未生效 —— 无论开关
        如何都无条件播种 + 缓存位置，等于"上帝视角"直接派机，覆盖栅格/拍卖/NBV
        全部沦为摆设。现在：

        - SEED_TRUTH=1（开发/仿真）：行为不变 —— 播种目标 + 缓存真值位置，
          _truth_pos 供 CooperativeTracker 做误差门槛（规则5 的 1m 判定）。
        - SEED_TRUTH=0（正式比赛，run_match.sh 已 export）：本回调完全静默。
          目标发现只走 /swarm/detection（swarm_agent 对 /swarm/target_states
          做 20m+LOS 几何门控后上报自己的观测）；派机位置退回 CooperativeTracker
          融合估计；消除以官方 /left_actors 为权威。

        CooperativeTracker 不存目标位置（它只管融合和计时），位置缓存在
        self._truth_cache 里，派追踪机时用。确认计时仍然只有 UAV 真的
        观测到才累计（规则 5 的连续 15s）。
        """
        if not SEED_TRUTH:
            return
        tid = str(msg.target_id)
        now = rospy.Time.now().to_sec()
        self._truth_cache[tid] = (msg.x, msg.y, now)
        self._truth_pos[tid] = (msg.x, msg.y)
        if msg.eliminated:
            self._truth_cache.pop(tid, None)
            self._truth_pos.pop(tid, None)
            return
        if tid in self._eliminated:
            return
        if tid not in self.tracker.targets:
            now = rospy.Time.now().to_sec()
            self.tracker.add_target(tid, now)
            rospy.loginfo("[manager] 真值播种目标 %s @ (%.1f, %.1f)", tid, msg.x, msg.y)

    def _get_target_pos(self, tid, now=None):
        """取目标最新位置：优先真值缓存 → 退回 CooperativeTracker 融合估计。"""
        if tid in self._truth_cache:
            x, y, t = self._truth_cache[tid]
            if now is None or (now - t) <= TRUTH_TTL:
                return (x, y)
            # 缓存过期：pop 掉避免堆积，回退到 CooperativeTracker 融合估计。
            # _truth_pos 必须同步过期：它只由 _truth_cb 写入，若长留，tracker
            # 的误差门槛会拿过期坐标当"真值"——agent 几何检测上报的正是同一
            # 过期坐标，err≈0 恒通过，形成自洽的幽灵确认循环
            # （2026-10-01 22:41 局 t3 实测：resets 17→20、进度卡 7%）。
            self._truth_cache.pop(tid, None)
            self._truth_pos.pop(tid, None)
        ct = self.tracker.targets.get(tid)
        if ct is not None:
            fx = ct.fused(now) if now is not None else None
            if fx is not None:
                return fx
        return None, None

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
                self._update_tracker_position(tid, tx, ty)
                continue
            busy = set(self._tracking.values()) | set(self._backup.values())
            best, best_d = None, None
            for uid, st in self.status.items():
                if not getattr(st, "connected", False) or uid in busy:
                    continue
                d = math.hypot(st.x - tx, st.y - ty)
                if best_d is None or d < best_d:
                    best, best_d = uid, d
            if best is None:
                continue
            # 距离余量：放宽到 DETECT_RADIUS + DETECT_RADIUS_MARGIN
            # 飞机距离目标 20m 但已在视野里也应该派去（DISPATCH_ON_FIRST_HIT）
            _disp_limit = DETECT_RADIUS + DETECT_RADIUS_MARGIN
            if best_d >= _disp_limit:
                rospy.loginfo_throttle(5.0,
                    "[manager] 目标 %s 最近机 %s 距 %.1fm ≥ 派遣阈值 %.1fm，暂不派遣",
                    tid, best, best_d, _disp_limit)
                continue
            self._tracking[tid] = best
            # CooperativeTracker 的协同关键：派出去的追踪机必须登记为 observer，
            # 否则它看到的观测会被忽略（规则5 需要多机接力 + 误差/间隔判定）。
            existing = list(ct.observers)
            if best not in existing:
                self.tracker.assign_observers(tid, existing + [best])
            # actor 真值可能落在栅格外（官方 actor 可达 y[-60,60]，栅格可能更小）
            # 直接派出去会让 A* 报 GOAL_OUT_OF_BOUNDS 原地打转，先 clamp 到栅格内
            tx_c = min(max(tx, self.grid.x_min + 0.5), self.grid.x_max - 0.5)
            ty_c = min(max(ty, self.grid.y_min + 0.5), self.grid.y_max - 0.5)
            msg = SearchAssignment()
            msg.header.stamp = rospy.Time.now()
            msg.uav_id = best
            msg.cell_ix = -1
            msg.cell_iy = -1
            msg.target_x = tx_c
            msg.target_y = ty_c
            if hasattr(msg, "target_id"):
                msg.target_id = tid
            msg.task_type = 1  # 目标确认/追踪
            self.assign_pub.publish(msg)
            rospy.loginfo("[manager] 派追踪：%s → 目标 %s @ (%.1f, %.1f), 距离 %.1fm",
                          best, tid, tx_c, ty_c, best_d)

    def _release_tracker(self, tid, reason=""):
        """国家一等奖：明确释放追踪/备份机，立即 publish cancel (-1)。

        原实现分散在两处（_dispatch_pending_targets 内部和 _check_lost_targets
        间接路径），删除时容易遗漏。统一从这一处发布。
        """
        uid = self._tracking.pop(tid, None)
        buid = self._backup.pop(tid, None)
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
                msg.task_type = -1  # 取消
                self.assign_pub.publish(msg)
            except Exception:
                pass
        if reason:
            rospy.loginfo("[manager] 释放追踪机 %s (tid=%s, 原因=%s)",
                          [r for r in (uid, buid) if r], tid, reason)

    # ---------------- 目标检测与消除（规则4/5） ----------------
    def _detection_cb(self, msg):
        """收到某机对某目标的检测 → 喂给 CooperativeTracker。

        CooperativeTracker 内部维护多机融合 + 三门槛（误差 1m / 间隔 1s / 连续 15s）
        计时，_update_targets 周期性调 update() 统一处理规则 4/5 事件。
        """
        if msg.target_id in self._eliminated:
            return
        tid = msg.target_id
        now = rospy.Time.now().to_sec()
        self._last_detect[tid] = now
        # 国家一等奖 v2 修复（2026-10-05）：_truth_cache[tid] = (msg.x, msg.y, now)
        # 这一行会让派机坐标 = 看到目标的飞机的坐标 → 飞机追自己 → 永远'接近'但不动。
        # 改为仅在 CooperativeTracker 还没产生 fused 估计前, 用上报位置做兜底。
        if tid not in self.tracker.targets:
            self._truth_cache[tid] = (msg.x, msg.y, now)
        else:
            ct = self.tracker.targets[tid]
            fx = ct.fused(now) if hasattr(ct, "fused") else None
            if fx is not None:
                self._truth_cache[tid] = (fx[0], fx[1], now)

        # CooperativeTracker 还不知道这个目标？登记
        if tid not in self.tracker.targets:
            self.tracker.add_target(tid, now)
            rospy.loginfo("[manager] 发现新目标 %s @ (%.1f, %.1f)，由 %s 首次检测",
                          tid, msg.x, msg.y, msg.uav_id)

        ct = self.tracker.targets[tid]
        # 把检测的 UAV 登记为 observer，否则 CooperativeTracker 会忽略它的观测
        if msg.uav_id not in ct.observers:
            self.tracker.assign_observers(tid, list(ct.observers) + [msg.uav_id])

        # 喂观测 → CooperativeTracker 内部做 alpha-beta 融合 + 三门槛判定。
        # truth 只在 SEED_TRUTH=1（真值源在跑）时提供：_truth_cache 里掺着检测
        # 位置（派机用），拿它当真值会让 last_err=|fused-检测|≈0，误差门槛
        # 恒通过 —— 等于从后门绕过 cooperative_tracker 的 B3 修复。
        # SEED_TRUTH=0 时传 None，tracker 退化为「看得见即合格」口径（无真值）。
        tp = self._truth_pos.get(tid)
        truth = tp if (SEED_TRUTH and tp is not None) else None
        self.tracker.report(msg.uav_id, tid, now, msg.x, msg.y, truth=truth)
        self._cur_targets[tid] = (msg.x, msg.y, msg.uav_id)

        # 已在追踪？更新盘旋位置；否则立刻派遣。
        # 2026-10-03 国家一等奖：首见即派（DISPATCH_ON_FIRST_HIT=1），
        # 不再等确认 3 击才派，否则首击→首派延迟 6-9s，确认窗口吃掉 1/3 预算。
        if tid in self._tracking:
            self._update_tracker_position(tid, msg.x, msg.y)
        else:
            self._dispatch_tracker(msg.target_id, msg.x, msg.y)
            # 改进 4：派机后若发现机距 > 派遣备份阈值，立刻就近派一架备份机。
            # 不等 BACKUP_AFTER（3s）——直接派，避免中途 2s 漏检即"跟丢→确认清零"。
            self._maybe_dispatch_immediate_backup(tid, msg)

    def _maybe_dispatch_immediate_backup(self, tid, msg):
        """首击即加派备份机（改进 4）—— 不等 3s 停滞。

        立即派机的依据：首见检测机的位置（msg.uav_id 自己坐标）若距目标
        > DISPATCH_BACKUP_DIST，则另派一架更近的机立即飞过去做"接棒观察"。
        双机即接力的状态可用 100%，单架断流/被建筑遮挡也维持确认进度条。
        """
        if not DISPATCH_ON_FIRST_HIT:
            return
        if tid in self._backup:
            return  # 已有备份
        if tid in self._eliminated:
            return
        # 派完主追后主追机本身可能不空闲 → 直接计算"第二近的机"
        main = self._tracking.get(tid)
        if main is None:
            return
        main_st = self.status.get(main)
        if main_st is None:
            return
        main_d = math.hypot(main_st.x - msg.x, main_st.y - msg.y)
        if main_d <= DISPATCH_BACKUP_DIST:
            return  # 主追机本身已经在 40m 内，单机足够
        # 选第二近的空闲机做备份
        busy = set(self._tracking.values()) | set(self._backup.values())
        best = None
        best_d = float('inf')
        for uid, st in self.status.items():
            if not getattr(st, "connected", False) or uid == main or uid in busy:
                continue
            d = math.hypot(st.x - msg.x, st.y - msg.y)
            if d < best_d:
                best, best_d = uid, d
        if best is None or best_d >= DETECT_RADIUS:
            return
        # 登记为 observer + 派任务
        self._backup[tid] = best
        ct = self.tracker.targets.get(tid)
        if ct is not None and best not in ct.observers:
            self.tracker.assign_observers(tid, list(ct.observers) + [best])
        tx_c = min(max(msg.x, self.grid.x_min + 0.5), self.grid.x_max - 0.5)
        ty_c = min(max(msg.y, self.grid.y_min + 0.5), self.grid.y_max - 0.5)
        m = SearchAssignment()
        m.header.stamp = rospy.Time.now()
        m.uav_id = best
        m.cell_ix = -1
        m.cell_iy = -1
        m.target_x = tx_c
        m.target_y = ty_c
        if hasattr(m, "target_id"):
            m.target_id = tid
        m.task_type = 1
        self.assign_pub.publish(m)
        rospy.loginfo("[manager] 即时增派备份：%s → 目标 %s (主追 %s 距 %.1fm，备份距 %.1fm)",
                      best, tid, main, main_d, best_d)

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
            cx = min(max(tx, bx0 + 1.0), bx1 - 1.0)
            cy = min(max(ty, by0 + 1.0), by1 - 1.0)
            rospy.logwarn_throttle(2.0,
                "[manager] 目标 %s 坐标 (%.1f, %.1f) 越界，夹到 (%.1f, %.1f)",
                target_id, tx, ty, cx, cy)
            return (cx, cy)
        return (tx, ty)

    def _dispatch_tracker(self, target_id, tx, ty):
        """派遣最近空闲机去追踪目标（盘旋确认）

        如果没有空闲机，则中断一台正在搜索的飞机（避免目标因无人确认而瞬移）。
        """
        # === 2026-10-05 比赛规则硬约束：派机坐标硬护栏 ===
        _san = self._sanitize_dispatch_xy(tx, ty, target_id)
        if _san is None:
            return
        tx, ty = _san
        # P1 修复：检查目标是否已被追踪
        if target_id in self._tracking:
            existing_uav = self._tracking[target_id]
            rospy.loginfo("[manager] 目标 %s 已被 %s 追踪，跳过派遣", target_id, existing_uav)
            return

        idle = self._idle_uavs()

        # 优先用空闲机
        best_uav = None
        best_dist = float('inf')
        for uid in idle:
            if uid not in self.status:
                continue
            ux, uy = self.status[uid].x, self.status[uid].y
            dist = math.hypot(tx - ux, ty - uy)
            if dist < best_dist:
                best_dist = dist
                best_uav = uid

        # 没有空闲机？中断正在搜索的飞机
        if best_uav is None:
            rospy.loginfo("[manager] 无空闲机，尝试中断搜索机去追踪 %s", target_id)
            # 找正在执行搜索任务的飞机，优先选预计到达时间 ≤ 15s 的
            candidate_uavs = []
            for uid in self.uav_ids:
                if uid not in self.status or not self.status[uid].connected:
                    continue
                # 跳过已在追踪的
                if uid in self._tracking.values():
                    continue
                # 找有搜索任务的
                has_search_task = False
                for c in self.grid.cells.values():
                    if c.state == STATE_ASSIGNED and c.owner == uid:
                        has_search_task = True
                        break
                if not has_search_task:
                    continue
                # 计算预计到达时间
                ux, uy = self.status[uid].x, self.status[uid].y
                dist = math.hypot(tx - ux, ty - uy)
                flight_time = dist / CRUISE_SPEED  # 用 CRUISE_SPEED 估算
                candidate_uavs.append((uid, dist, flight_time))

            if candidate_uavs:
                # 按飞行时间排序，选最快的（15s 内能到的优先）
                candidate_uavs.sort(key=lambda x: x[2])
                best_uav, best_dist, flight_time = candidate_uavs[0]
                rospy.loginfo("[manager] 中断 %s 的搜索任务去追踪 %s（预计 %.1fs）",
                              best_uav, target_id, flight_time)
                # 释放该机的搜索格租约
                for key, c in self.grid.cells.items():
                    if c.state == STATE_ASSIGNED and c.owner == best_uav:
                        c.state = STATE_FREE
                        c.owner = None
                        c.lease_until = 0.0
            else:
                rospy.loginfo("[manager] 无可中断的搜索机，无法派遣追踪 %s", target_id)
                return

        if best_uav is None:
            return

        # P2 修复：派遣距离余量。DETECT_RADIUS=20m 是感知上限，贴边派遣后
        # 目标稍微移动就出视野，6s 内跟丢（实测 dist=19.924 派遣后 6s 跟丢）。
        # 要求 best_dist < DETECT_RADIUS - DISPATCH_MARGIN 才派遣，否则等更近的
        # 飞机/目标靠近，避免无效派遣占机。
        _disp_limit = DETECT_RADIUS - DISPATCH_MARGIN
        if best_dist >= _disp_limit:
            rospy.loginfo(
                "[manager] 目标 %s 最近机 %s 距 %.1fm ≥ 派遣阈值 %.1fm，暂不派遣",
                target_id, best_uav, best_dist, _disp_limit)
            return

        # 发布追踪任务（task_type=1）
        # 2026-10-01 修复：① target_id 必须填上——agent 侧 _orbit_target 靠它
        # 对齐真实目标（消除广播后正确清盘旋），空值会让 agent 退到 "tracking"
        # 占位符；② 真值可能落在栅格外（actor 官方可达 y[-60,60]），不 clamp
        # 会让 agent 的 A* 报 GOAL_OUT_OF_BOUNDS 原地打转（对齐 _dispatch_pending_targets）。
        tx_c = min(max(tx, self.grid.x_min + 0.5), self.grid.x_max - 0.5)
        ty_c = min(max(ty, self.grid.y_min + 0.5), self.grid.y_max - 0.5)
        msg = SearchAssignment()
        msg.header.stamp = rospy.Time.now()
        msg.uav_id = best_uav
        msg.cell_ix = -1
        msg.cell_iy = -1
        msg.target_x = tx_c
        msg.target_y = ty_c
        if hasattr(msg, "target_id"):
            msg.target_id = target_id
        msg.task_type = 1  # 目标确认/追踪
        self.assign_pub.publish(msg)
        # 记录追踪任务
        self._tracking[target_id] = best_uav
        # === 算法层日志：派遣时的目标/无人机/距离/阈值/捕获判定 ===
        # 重点：打印「算法内部计算的距离」，排查坐标系/单位错位导致的不消除。
        _ux, _uy = self.status[best_uav].x, self.status[best_uav].y
        _dist = math.hypot(tx - _ux, ty - _uy)
        _captured = _dist < DETECT_RADIUS
        rospy.loginfo(
            "[ALGO] dispatch target=%s pos=(%.2f,%.2f) | uav=%s pos=(%.2f,%.2f) "
            "| dist=%.3fm | detect_radius=%.1fm | is_captured=%s | flight_time_est=%.1fs",
            target_id, tx, ty, best_uav, _ux, _uy, _dist,
            DETECT_RADIUS, _captured, best_dist / CRUISE_SPEED)
        rospy.loginfo("[manager] 派遣 %s 追踪目标 %s @ (%.1f, %.1f), 距离 %.1fm",
                      best_uav, target_id, tx_c, ty_c, best_dist)

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
        msg.target_x = tx
        msg.target_y = ty
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
            msg2.target_x = tx
            msg2.target_y = ty
            if hasattr(msg2, "target_id"):
                msg2.target_id = target_id
            msg2.task_type = 1
            self.assign_pub.publish(msg2)

    def _dispatch_backup(self):
        """确认期冗余派机：用 CooperativeTracker.needs_backup() 定向增派。

        CooperativeTracker 自己维护了两个精确判据（替换掉旧版 manager 自己算的 stale/stuck）：
          (a) resets >= reset_thresh 次：被裁判多次重置，单机扛不住；
          (b) confirm_since 停滞 >= stall_thresh 秒：进度条卡住。

        两机只要有一架保持可见，上报链就不中断 —— 这正是「协同追踪」真正要落地的地方。
        """
        if not BACKUP_ENABLE:
            return
        now = rospy.Time.now().to_sec()

        needs = self.tracker.needs_backup(now,
                                          reset_thresh=1,
                                          stall_thresh=BACKUP_AFTER)
        if not needs:
            return

        busy = set(self._tracking.values()) | set(self._backup.values())

        for tid, reason in needs:
            if tid in self._backup:
                continue
            # v18：confirm 超时黑名单期内不冗余派机
            if now < self._confirm_timeout_bl.get(tid, 0.0):
                continue
            aid = self._tid_to_actor(tid)
            if self._left_seen and aid is not None and aid not in self._left_actors:
                continue
            ct = self.tracker.targets.get(tid)
            if ct is None or ct.eliminated:
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

            best, best_d = None, None
            for uid, st in self.status.items():
                if not getattr(st, "connected", False) or uid in busy:
                    continue
                d = math.hypot(st.x - tx, st.y - ty)
                if best_d is None or d < best_d:
                    best, best_d = uid, d
            if best is None:
                continue
            if best_d > BACKUP_MAX_DIST:
                if tid not in self._backup_noted:
                    self._backup_noted.add(tid)
                    rospy.loginfo("[manager] %s 确认受阻（%s），但最近空闲机 %.1fm "
                                  "> BACKUP_MAX_DIST=%.0fm，暂不冗余派机",
                                  tid, reason, best_d, BACKUP_MAX_DIST)
                continue

            # 关键：backup 机也必须登记为 observer，否则 CooperativeTracker 会忽略它的观测
            ct.observers.add(best)
            self._backup[tid] = best
            busy.add(best)
            tx_c = min(max(tx, self.grid.x_min + 0.5), self.grid.x_max - 0.5)
            ty_c = min(max(ty, self.grid.y_min + 0.5), self.grid.y_max - 0.5)
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

    def _update_targets(self):
        """按周期更新每个目标的确认计时，处理规则4/5。"""
        now = rospy.Time.now().to_sec()
        # 先给「已知但未消除」的目标派追踪机（真值播种后才有这一步）
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
            _captured = (ct.confirm_since is not None and
                         (ct.last_err is None or ct.last_err <= ct.err_tol))
            _error_ok = ct.last_err is None or ct.last_err <= ct.err_tol
            _gap_s = ((now - ct.last_ok_t) if ct.last_ok_t is not None else None)
            _gap_ok = ct.last_ok_t is None or _gap_s <= ct.gap_tol
            self._csv.write(
                ros_time=now, event="capture", auction_cycle=self._auction_cycle,
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
                self._eliminated.add(tid)
                aid = self._tid_to_actor(tid)
                # 国家一等奖修复（2026-10-05 击毁积分 0 分 bug）：
                # 之前这里直接调 `_release_tracker` + `cmd_pub.publish("eliminate:%s" % tid)`，
                # 这两步会让官方 score_cal 永远拿不到消除信号：
                #   1) 释放追踪机 ⇒ 所有机离开该目标 ⇒ /coordination/target_report
                #      停止上报 ⇒ yolo_target_bridge 的 _emit 因 ev["eliminated"]=True
                #      停发 /actor_<color>_info
                #   2) 发 eliminate 给 target_sim_node ⇒ 仿真把该 actor 从 Gazebo 删除
                #      ⇒ score_cal 的 actors_pos[aid] 变成 None ⇒ _reset_detection 清零
                # 官方 score_cal.py 不订阅任何消除指令,消除唯一入口是「/actor_<color>_info
                # 持续 15s 误差<1m 且间隔≤1s 的上报」(score_cal.py:122-168)。我们必须
                # 让飞机继续上报,直到官方真的从 /left_actors 里把它去掉 (→ _left_cb)。
                # 新策略:
                #   · 仅把 tid 标 _eliminated、记 confirm_done_t (供超时兜底);
                #   · **不**调 _release_tracker (飞机继续绕目标 8m 盘旋);
                #   · **不**给 target_sim_node 发 eliminate (Gazebo 模型留着,
                #     score_cal 才能用真值做 err_threshold=1m 的判定);
                #   · 等 _left_cb → _release_finished 触发释放（官方真消除的权威信号）;
                #   · 超时兜底：CONFIRM_HOLD_TIMEOUT 默认 30s —— 比官方 15s 多一倍，
                #     防止上报链路偶发断流超 15s 后仍然无消除,我们不能无限占着飞机。
                self._confirm_done_t[tid] = now
                # 2026-10-06 国家一等奖: 通知 swarm_agent 切贴脸模式
                # (只有当前 _orbit_target == tid 的那架 UAV 会响应)
                try:
                    self.confirm_pub.publish(String("tid:%s" % tid))
                except Exception as e:
                    rospy.logwarn("[manager] confirm_pub.publish 失败: %s", e)
                rospy.loginfo("[manager] 规则5：目标 %s 团队侧连续确认 %.0fs，"
                              "保留追踪机继续上报直到官方 /left_actors 确认 (aid=%s)",
                              tid, CONFIRM_TIME, aid)
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
                        is_captured=True,
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
                rospy.loginfo("[manager] 规则4：目标 %s 首次确认后墙钟 %.0fs 未消除 → 瞬移，"
                              "relocate 计时重置", tid, EVADE_TIME)
                # 瞬移后旧位置/身份的"已确认"无意义 → 清掉等待兜底的戳,避免 30s 后误触发
                self._confirm_done_t.pop(tid, None)
                self._cur_targets.pop(tid, None)
                continue
            if ev == "reset":
                rospy.loginfo_throttle(3,
                    "[manager] 目标 %s 规则5 被重置（误差或间隔超限）→ confirm_since 清零", tid)

            prog = ct.progress(now)
            if prog > 0:
                cs = ct.confirm_since
                cs_elapsed = (now - cs) if cs is not None else 0.0
                rospy.loginfo_throttle(5,
                    "[manager] 目标 %s 确认进度 %.0f%% "
                    "(confirm_since=%.1fs, resets=%d, rejects=%d, observers=%s)",
                    tid, prog * 100, cs_elapsed, ct.resets, ct.rejects,
                    ",".join(sorted(ct.observers)))

        # === 国家一等奖修复（2026-10-05 击毁 0 分 bug）===
        # 超时兜底：团队侧已确认满 15s 但官方 /left_actors 还没把它去掉（说明上报链路
        # 出问题了: 误差偶发>1m、间隔>1s、飞机跑出视场等）,超过 CONFIRM_HOLD_TIMEOUT
        # (默认 30s) 还没消除 → 不能无限占着飞机。
        # 此时:
        #   1) ~~给 target_sim_node 发 "eliminate:%s" 让仿真把这个 actor 移走~~ (已废)
        #      (原设计：官方 score_cal 也只有此时才能从 /left_actors 把它去掉 —— 模型不在了
        #      get_model_state 返回 success=False,score_cal 内层逻辑会跟着停判)
        #      国家一等奖 v3 (2026-10-06): 上述"反向污染"路径违反「消除必须裁判系统说了算」
        #      原则——manager 删 Gazebo 模型 → score_cal 拿不到 actor → 自动从 /left_actors
        #      移除 → GUI 显示 LEFT TARGET,但这不是裁判判的!真实判定权被 manager 偷走。
        #      修正: 仅释放追踪机回搜索, Gazebo 模型和 /left_actors 一律保留,等待
        #      真正的裁判权威信号到达 (_left_cb → _release_finished)。
        #   2) 释放追踪机回搜索 (保留 — 不让一架飞机永远占着一个团队侧已确认的目标)
        # 这是次优路径(不指望真打 100 分,只求不为这个目标把全队卡死)。
        for tid in list(self._confirm_done_t.keys()):
            done_t = self._confirm_done_t.get(tid)
            if done_t is None:
                continue
            if now - done_t < CONFIRM_HOLD_TIMEOUT:
                continue
            aid = self._tid_to_actor(tid)
            # 如果官方已经确认,只是 manager 这边还没收到(竞争窗口),直接跳过
            if self._left_seen and aid is not None and aid in self._left_actors:
                rospy.loginfo_throttle(5,
                    "[manager] 超时兜底跳过：%s 官方仍在 left_actors (aid=%s),继续保留",
                    tid, aid)
                continue
            rospy.logwarn("[manager] 规则5超时兜底：%s 团队侧确认后 %.0fs 仍未被官方消除,"
                          "释放追踪机回搜索 + 内部标 eliminated,等 /left_actors 裁判权威 (aid=%s)",
                          tid, now - done_t, aid)
            # 国家一等奖 v3 (2026-10-06): 不再发 cmd_pub.publish("eliminate:%s")
            # —— 让 Gazebo 模型保留, score_cal 才能继续用真值做 15s 误差<1m 的
            # 判定。manager 只负责内部清理 + 释放飞机:
            #   · _eliminated.add: 防 _dispatch_pending_targets 重复派机(协同必需)
            #   · t.eliminated=True: 防 tracker 再累加 confirm_since(协同必需)
            #   · _release_tracker: 释放飞机回搜索任务
            #   · 等 _left_cb 裁判权威到达 → _release_finished 真正收尾
            self._eliminated.add(tid)
            t = self.tracker.targets.get(tid)
            if t is not None:
                t.eliminated = True
            try:
                self._release_tracker(tid, reason="confirm_hold_timeout")
            except Exception:
                pass
            # 清理内部状态,避免下一周期重复触发
            self._confirm_done_t.pop(tid, None)
            self._truth_cache.pop(tid, None)
            self._truth_pos.pop(tid, None)
            self._cur_targets.pop(tid, None)

        # === v18（2026-10-07）confirming 永不收敛超时释放 ===
        # v17 实测：t3 从 116s、t1 从 186s 进入 confirming，直到 436s 比赛结束
        # 既未 confirmed 也未释放 —— 追踪机/备份机被永久占用，叠加租约 renew
        # 循环 bug 后全队 202s 起零分配。用 _first_confirm_t（首次进入确认状态
        # 时刻，仅瞬移重置，不随 reset 清零）做硬超时：超时未消除 → 释放全部
        # 追踪机 + 黑名单期内不重派（等新检测再重新介入，避免 90s 空转循环）。
        _cto = float(os.environ.get("TRACK_CONFIRM_TIMEOUT", "90"))
        _bl_hold = float(os.environ.get("CONFIRM_TIMEOUT_BLACKLIST", "60"))
        for tid, ct in list(self.tracker.targets.items()):
            if ct.eliminated or tid in self._eliminated:
                continue
            _fct = getattr(ct, "_first_confirm_t", None)
            if _fct is None:
                continue
            if (now - _fct) < _cto:
                continue
            rospy.logwarn("[manager] 目标 %s 首次确认后 %.0fs 仍未消除（超时 %.0fs）"
                          "→ 释放追踪机 + 黑名单 %.0fs", tid, now - _fct, _cto, _bl_hold)
            self._confirm_timeout_bl[tid] = now + _bl_hold
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
                if hasattr(self.grid, "_reopen_covered_cells"):
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

        # 用**所有**在飞飞机的实时位置刷新「真正看见过」的区域（不只空闲机）
        if VIS_ENABLE:
            try:
                # 国家一等奖 v3（2026-10-07）：关键修复 —— 不要传 wall clock！
                # 之前写 _tn = time.time() 会让 mark_seen 写入的 visit_time 与
                # score_cell 读取用的 _now() = rospy.get_time()（仿真钟）不同源，
                # novelty = (仿真钟 - 墙钟) / 90s ≈ 极大负数 → clamp 到 0 → 全图
                # novelty 都被压低 → (1,5) 等 spawn 区格反复被派 32 次。修复：不传
                # now，让 mark_seen 内部用 _now()（仿真钟），与 score_cell 一致。
                for _uid, _st in self.status.items():
                    if not getattr(_st, "connected", False):
                        continue
                    self.allocator.mark_seen(_st.x, _st.y)
            except Exception:
                pass

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
            # 授租约（自适应时长：飞行时间 × 1.5 + 10s 缓冲）
            uav_pos = uavs[uid]
            cell = self.grid.cell(key)
            _wp0 = self._cell_waypoint.get(key) or (cell.cx, cell.cy)
            dist = math.hypot(_wp0[0] - uav_pos[0], _wp0[1] - uav_pos[1])
            duration = max(10.0, dist / float(os.environ.get("LEASE_CRUISE_SPEED", "5.0")) * 1.5 + 10.0)  # LEASE_CRUISE_SPEED 默认 5.0（与 agent MAX_SPEED 对齐：原 3.0 把租约算长 67%, 接力损耗严重）
            self.lease.grant(uid, key, now.to_sec(), duration)
            self._active_leases[uid] = key
            rospy.loginfo("[manager] 分配 %s → 格 (%d,%d) 飞行距离 %.1fm 租约 %.1fs",
                          uid, key[0], key[1], dist, duration)
            self._publish_assignment(uid, key, cell)

        # === 终局行为 ===
        if not all_assigned:
            # 没有更多搜索格可用，检查是否有未消除的目标
            remaining_targets = [t for t in self.tracker.targets if t not in self._eliminated]
            n_left = len(self._left_actors)
            if n_left > 0 and not remaining_targets:
                # tracker 空 + 裁判说还有 actor = 一个都没搜到，绝不能收工
                rospy.loginfo_throttle(
                    15, "[manager] tracker 无目标，但官方裁判还剩 %d 个 actor %s "
                        "—— 判定为未发现，继续巡逻", n_left, self._left_actors)
                remaining_targets = ['official_left:%d' % n_left]
            if remaining_targets:
                # 有未消除目标：继续搜索，不要 RTL
                # 原因：20m 半径覆盖可能有盲区，目标可能未被"偶遇"
                rospy.loginfo("[manager] 搜索格已覆盖完毕，剩余目标 %d 个，继续搜索",
                              len(remaining_targets))
                # 不返回，继续分配搜索任务（让飞机继续巡逻）
                # 下次分配会重新扫描所有未覆盖格子 —— 但此时已经「没有未覆盖格」了，
                # 所以必须显式把覆盖状态重置，否则飞机拿不到任务只能待命。
                # 只有在没有追踪任务时才重开：否则「重开→拍卖又选最近的脚下那格」
                # 会让 6 架飞机原地打转（实测 136 次派位全是同一批格子）
                # 以前只在"没有追踪任务"时才重开，怕重开后飞机又回原地打转。
                # 现在重开会跳过脚下的格（见 _reopen_covered_cells），可以无条件重开 ——
                # 否则只要有一架在追踪（无播种时一发现目标就是这样），格子又都覆盖了，
                # 剩下的机就彻底没任务可做（实测 32 次派位后停摆到 600s 超时）。
                self._reopen_covered_cells()
            else:
                # 全部完成，发布降落
                # 2026-09-28：实测这里误判过 —— tracker 瞬间为空 + _left_actors 还没到
                # 就被当成「搜完了」，把空闲机降下来，一架落地就再也救不回来（见
                # patch_guard2）。默认不再自动降落，宁可继续巡逻。
                # 国家一等奖 v2（2026-10-05）：双保险 —— REQUIRE_LEFT_FOR_FINISH=1 时
                # 必须官方剩余清单确认空 + tracker 已空 才发 MISSION_FINISHED，
                # 防止 tracker 偶发空导致 agent 全部退出搜索循环（_finish_cb 会 break）。
                _require_left = int(os.environ.get("REQUIRE_LEFT_FOR_FINISH", "1"))
                if _require_left and (not self._left_seen or self._left_actors):
                    rospy.loginfo_throttle(
                        20, "[manager] tracker 空但官方剩余清单未确认（_left_seen=%s, _left_actors=%s），"
                            "不发 MISSION_FINISHED，继续巡逻",
                            self._left_seen, self._left_actors)
                else:
                    if not AUTO_LAND:
                        rospy.loginfo_throttle(
                            20, "[manager] 疑似全部完成（tracker 空且剩余 0），但 AUTO_LAND=0"
                                " → 继续巡逻，不降落")
                    else:
                        rospy.loginfo("[manager] 全部任务完成，各机降落")
                        self._publish_land(idle)
                    # 广播任务完成消息
                    self.finish_pub.publish(String(data="MISSION_FINISHED"))

        self._last_alloc_t = now

    # ---------------- 续租 ----------------
    def _renew_leases(self):
        """对持续上报的机续租（防误判到期）。

        若飞机已抵达分配格（距格代表点 < COVER_ARRIVE_M），则把该格可视域
        标记为 STATE_COVERED 并释放租约，使飞机重回空闲池拿下一格。
        否则租约永不到期 → 全部飞机拿到首格后死锁（实测 5min+ 无新调度）。
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
            # 到达判定：用格的可达代表点（楼边格中心可能在建筑内）
            _wp = self._cell_waypoint.get(key)
            if _wp is None:
                _c = self.grid.cell(key)
                _wp = (_c.cx, _c.cy) if _c is not None else None
            st = self.status.get(uid)
            if _wp is not None and st is not None:
                _d = math.hypot(st.x - _wp[0], st.y - _wp[1])
                if _d < COVER_ARRIVE_M:
                    # 标记可视域已覆盖（与 mark_seen 同半径）。
                    # is_covered 会把格 state 置为 STATE_COVERED、owner 清空，
                    # 因此无需再调 lease.release（该方法不存在）。
                    n_cov = self.grid.is_covered(st.x, st.y, VIS_RADIUS)
                    # v18：把覆盖格同步进 allocator.visit_time —— score_cell 读的是
                    # TaskAllocator.visit_time（与 CoverageGrid.visit_time 是两个
                    # 独立对象），不同步则「覆盖即刷新 novelty」永不生效，spawn 区
                    # 格 reopen 后 novelty 恒 1.0 反复被派（v12 根因之一）。
                    try:
                        _t_now = now.to_sec()
                        for _ck in (n_cov or []):
                            self.allocator.visit_time[_ck] = _t_now
                    except Exception:
                        pass
                    self._active_leases.pop(uid, None)
                    self._lease_hold_since.pop((uid, key), None)
                    rospy.loginfo("[manager] %s 抵达格 (%s,%s)，覆盖 %s 格，释放租约",
                                  uid, key[0], key[1], len(n_cov))
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
                if self.lease.force_expire(uid, key):
                    self._active_leases.pop(uid, None)
                    self.allocator.failed_visit[key] = now.to_sec()
                    rospy.logwarn("[manager] %s 持有格 (%s,%s) 超 %.0fs 未抵达 → "
                                  "强制回收租约重派", uid, key[0], key[1], LEASE_MAX_HOLD)
                continue
            self.lease.renew(uid, key, now.to_sec())
        self._last_lease_t = now

    # ---------------- 发布 ----------------
    def _publish_assignment(self, uid, key, cell):
        msg = SearchAssignment()
        msg.header.stamp = rospy.Time.now()
        msg.uav_id = uid
        msg.cell_ix = key[0]
        msg.cell_iy = key[1]
        _wp = self._cell_waypoint.get(key)
        if _wp is None:
            _wp = (cell.cx, cell.cy)
        msg.target_x = _wp[0]
        msg.target_y = _wp[1]
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
                        n_elim = len(self._eliminated)
                        # tracker.targets 是 dict: target_id -> CaptureTarget
                        _tu = self.tracker.targets if hasattr(self, "tracker") and self.tracker is not None else {}
                        confirming_tids = sorted([tid for tid, t in _tu.items()
                                                  if not getattr(t, "eliminated", False)])
                        rospy.loginfo("[DEBUG-10s] sim=%.1f eliminated=%d(%s) confirming=%s total=%d",
                                      now.to_sec(),
                                      n_elim, sorted(self._eliminated), confirming_tids, len(_tu))
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
