#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集群协同搜索任务分配层（纯逻辑，原创实现，仅依赖标准库）。

把 200×100 m 场地划分为 10×10 m 搜索格，用「集中式拍卖」给每架无人机分配
不重复的未搜索格，并用「任务租约」防止重复搜索/掉线卡死。发现目标后进入
「追踪接力」：只派一架机做 15s 确认，其余继续搜索，追踪机失能时按「最快
到达观察点」选接力机。

设计约束（答辩/查重）：
  - 只依赖标准库（math），无 ROS / numpy，可独立单元测试。
  - 不复制 Crazyswarm2 的节点/消息/类/配置/API；其「多机状态同步、集中调度、
    可观测性」思想仅作概念参考。本文件自建数据结构与效用函数。
  - 拍卖效用 U_{i,c} = w1·未搜索收益 − w2·预计飞行时间 − w3·与他机重复率 − w4·路径风险。

坐标系约定：世界/地图坐标（米），与生成器 metadata 一致（ENU，x:-100~100，y:-50~50）。
"""

import math
import os
import time
import rospy


def _now():
    """仿真钟:与全栈驻留计时口径一致,避免 RTF 偏低(RTF=0.15)时业务判定失真。

    仿真钟 vs 墙钟:
      - 墙钟 `time.time()`: 真实时间,仿真下走得快(慢)
      - 仿真钟 `rospy.get_time()`: Gazebo 内部时间,与代码逻辑秒数(8/45/90/300 等)一一对应

    /use_sim_time=true 时 rospy.get_time() 返回仿真钟,否则返回墙钟,等价安全。
    2026-10-05 修复:所有驻留计时统一改用 _now(),消除 RTF 偏差带来的:
      - _hot_target_ttl=8s 几乎瞬间过期(目标接力失效)
      - novelty_horizon=90s / _PRIORITY_DECAY=300s 阈值偏移
      - visit_time 写入/读取时钟不同源导致新颖性永久=1.0
    """
    return rospy.get_time()

# ============================ 参数 ============================
GRID_SIZE_M   = float(os.environ.get("GRID_SIZE_M", "7.0"))   # 搜索格边长 m（batch10 实测 7.0 优于 10.0：中位 157.5s vs 286.1s）
NUM_ZONES     = 6      # 搜索分区数量（与飞机数一致）
# === 国家一等奖标准：默认权重调整 ===
# 原 W_GAIN=1.0 + W_NOVELTY=1.2 + W_SPREAD=1.5：收益项被分散奖励压死
# 修复：让「未搜索收益」与「新颖性」形成清晰梯度，飞机在新一轮重开时先扫从未去过的角落
W_GAIN        = float(os.environ.get("W_GAIN", "1.6"))      # 提升未搜索收益权重（防盖过其他项）
W_FLIGHT      = float(os.environ.get("W_FLIGHT", "0.025"))  # 飞行时间权重（小幅下调，鼓励远距离搜索）
FLIGHT_CAP    = float(os.environ.get("FLIGHT_CAP", "45.0"))  # 飞行时间惩罚上限 s（远距格不被一票否决）
W_NOVELTY     = float(os.environ.get("W_NOVELTY", "0.8"))    # 「最久未访问」优先权重（2026-10-07 五分钟冲刺: 1.5→0.8, 原值压过 zone/spread 导致 6 机一线起飞后各自原地打转找"最久未访问"格; 降权后 spread 主导快速散开）
NOVELTY_HORIZON = float(os.environ.get("NOVELTY_HORIZON", "90.0"))  # 新颖性饱和时间 s（缩短，逼迫轮转）
W_OVERLAP     = float(os.environ.get("W_OVERLAP", "2.0"))    # 与他机重复率权重（提升，避免扎堆）
W_RISK        = float(os.environ.get("W_RISK", "0.5"))      # 路径风险权重
W_BALANCE     = float(os.environ.get("W_BALANCE", "1.2"))   # 任务均衡权重（提升，均衡压力）
W_DISTANCE    = float(os.environ.get("W_DISTANCE", "1.0"))   # 机间距离惩罚权重（适度，避免扎堆）
W_SPREAD      = float(os.environ.get("W_SPREAD", "2.0"))     # 分散奖励权重（继续提升，6 机均匀覆盖）
SPREAD_SAT    = float(os.environ.get("SPREAD_SAT", "60.0"))  # 分散奖励饱和距离 m
W_ZONE        = float(os.environ.get("W_ZONE", "1.2"))       # 区域责任权重（0.6→1.2，国家一等奖：原值让 zone_bonus 差异仅 0.6×0.2=0.12 被 flight/novelty 压过，导致 uav_2 跑进 uav_1 区；新值与 spread=2.0 平级，确保 Voronoi 区域分配真正主导）

# 机间安全距离（与格子边长对齐）
MIN_UAV_DIST  = float(os.environ.get("MIN_UAV_DIST", "14.0"))  # 最小机间距 m（提升到14m，6机充分分散）
# 任务租约（国家一等奖标准：避免格子过度争抢）
LEASE_DURATION    = float(os.environ.get("LEASE_DURATION", "20.0"))  # 提升到20s，飞行+扫描+衔接余量
LEASE_MIN_SPEED   = 3.0    # 租约计算用的巡航速度 m/s
LEASE_FACTOR     = float(os.environ.get("LEASE_FACTOR", "1.8"))  # 提升系数，确保远距格能飞到
CONFIRM_TIME      = 15.0   # 规则5：连续 15s 正确广播 ID+坐标 → 判定消除
EVADE_TIME        = 30.0   # 规则4：被感知 30s 仍未消除 → 目标瞬移躲藏
# 追踪接力
RELAY_SAFE_DIST   = float(os.environ.get("RELAY_SAFE_DIST", "5.0"))  # 观察环安全半径 m
RELAY_LOSE_DIST   = float(os.environ.get("RELAY_LOSE_DIST", "12.0"))  # 丢失判定距离 m（收紧到12m，避免慢速接力误触发）
# === 国家一等奖标准：目标运动预测 ===
PREDICT_LEAD_TIME   = float(os.environ.get("PREDICT_LEAD_TIME", "3.5"))  # 派遣预测提前量 s（飞机飞行时间）
PREDICT_MAX_SPEED   = float(os.environ.get("PREDICT_MAX_SPEED", "2.5"))  # actor 速度上限（官方2m/s+余量）
PREDICT_MIN_SPEED   = float(os.environ.get("PREDICT_MIN_SPEED", "0.3"))  # 最小启动速度（避免抖动噪声放大）
# 目标检测（规则3：几何判定，比赛主判定方式）
# 国家一等奖（2026-10-05 修复）：原 DETECT_RADIUS=20m → 触发「被感知」过早；
# 官方规则是「感知→目标 2m/s 逃跑 + 向同伴广播」+「30s 未消除则瞬移」——
# 在 20m 处开报等于「提前启动 30s 倒计时」「在位置误差最大处开报」「把全城惊动」。
# 比赛视频实测：t=18.6s 首次 find → t≈24s 首个消除，只隔 5s（先贴上去再报），
# 印证原半径过宽。这里收紧到 10m（用户给定的 8~10m 区间上沿，保留测量噪声余量）。
# 注意：盘旋确认仍需看清 ID，所以 10m 不再设得过小；配合 LOS 遮挡判定，误报可压制。
DETECT_RADIUS     = float(os.environ.get("DETECT_RADIUS", "10.0"))  # 几何检测半径 m（2026-10-05: 20→10，比赛实测过早触发）
DETECT_RADIUS_MARGIN = float(os.environ.get("DETECT_RADIUS_MARGIN", "3.0"))  # 派遣半径余量（贴边派遣后防跟丢）
# ---- LOS 感知 next-best-view（2026-09-28）----
VIS_ENABLE     = os.environ.get("VIS_ENABLE", "1") not in ("0", "false", "False", "")
W_VIS          = float(os.environ.get("W_VIS", "1.5"))       # 可见增益权重（0=关闭）
VIS_STALE      = float(os.environ.get("VIS_STALE", "60.0"))  # 陈旧饱和时间 s
VIS_RADIUS     = float(os.environ.get("VIS_RADIUS", "10.0")) # 视点可见半径 m（2026-10-05: 20→10，与 DETECT_RADIUS 对齐，避免评分偏差）
VIS_COMMIT     = os.environ.get("VIS_COMMIT", "1") not in ("0", "false", "False", "")
VIS_GAIN_SAT   = float(os.environ.get("VIS_GAIN_SAT", "12.0"))  # 增益饱和格数：看得再多也不超过 1.0（中位可见 10 格）
TARGET_WALK_SPEED = 1.0    # 规则2：未感知无人机时的随机走动速度 m/s
TARGET_FLEE_SPEED = 2.0    # 规则3：感知到无人机后的逃跑速度 m/s
# 网格状态枚举
STATE_FREE      = 0    # 未分配
STATE_ASSIGNED  = 1    # 执行中（有租约）
STATE_COVERED   = 2    # 已覆盖
STATE_REVIEW    = 3    # 需复查（置信度不足）


class SearchCell(object):
    """单个搜索格：状态 + 租约 + 覆盖置信度。"""
    __slots__ = ("cx", "cy", "state", "owner", "lease_until", "confidence",
                 "review_t", "covered_t")

    def __init__(self, cx, cy):
        self.cx = cx          # 格中心 x（世界坐标 m）
        self.cy = cy          # 格中心 y
        self.state = STATE_FREE
        self.owner = None     # 执行机 id
        self.lease_until = 0.0
        self.confidence = 0.0 # 覆盖置信度 [0,1]
        self.review_t = 0.0   # REVIEW 状态进入时刻（仿真钟）
        self.covered_t = 0.0  # COVERED 状态进入时刻（仿真钟，供 reopen）


class CoverageGrid(object):
    """200×100 覆盖栅格：20×10 个 10×10 m 搜索格。"""

    def __init__(self, x_min, x_max, y_min, y_max, cell_m=GRID_SIZE_M, num_uavs=6):
        self.x_min, self.x_max = float(x_min), float(x_max)
        self.y_min, self.y_max = float(y_min), float(y_max)
        self.cell_m = float(cell_m)
        self.nx = int(math.ceil((self.x_max - self.x_min) / self.cell_m))
        self.ny = int(math.ceil((self.y_max - self.y_min) / self.cell_m))
        self.num_uavs = num_uavs
        self.cells = {}
        for ix in range(self.nx):
            for iy in range(self.ny):
                c = SearchCell(self.x_min + (ix + 0.5) * self.cell_m,
                               self.y_min + (iy + 0.5) * self.cell_m)
                self.cells[(ix, iy)] = c

        # ---- 区域分割：把地图分成 num_uavs 个区域 ----
        self.zone_cells = self._compute_zones()

        # v18（2026-10-07）：is_covered 会写 visit_time（覆盖即刷新 novelty），
        # 但本类从未初始化该属性（只有 TaskAllocator 有）→ 首次有飞机抵达任意格
        # 即 AttributeError，manager 主循环被连杀（v17 实测 81.1~89.7s 共 12 次，
        # 每次崩溃跳过整个主循环迭代含 _allocate），且 v3「覆盖即刷新 novelty」
        # 从未生效 → spawn 区格 reopen 后 novelty 恒 1.0 反复被派。
        self.visit_time = {}

    # ---- 坐标 ↔ 格索引 ----
    def world_to_cell(self, x, y):
        ix = int(math.floor((x - self.x_min) / self.cell_m))
        iy = int(math.floor((y - self.y_min) / self.cell_m))
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            return (ix, iy)
        return None

    def cell_index(self, key):
        return key

    # ---- 查询 ----
    def cell(self, key):
        return self.cells.get(key)

    def uncovered_cells(self):
        """返回所有「未分配 / 需复查」的格 key 列表（可拍卖候选）。

        国家一等奖标准改进（2026-10-03）：原实现把 STATE_REVIEW 视为可分配。
        REVIEW 状态的格子置信度不足，原意是「要再扫一次」，但实际行为是正常拍卖
        → 派飞机去时没有降级提示 → 飞机以为这是新格子，扫完仍 confidence<1
        → 永不被清理。修复：REVIEW 也参与拍卖但记为「gain=0.5」让飞机愿意
        优先扫未访问过的真 FREE 格（TaskAllocator.utility 已实现）。
        """
        out = []
        for key, c in self.cells.items():
            if c.state in (STATE_FREE, STATE_REVIEW):
                out.append(key)
        return out

    def _reopen_covered_cells(self, max_age=None):
        """国家一等奖标准改进（2026-10-03）：超时 COVERED 强制回 FREE。

        原实现使用「绝对 mission 时间戳 (now - self._start_time) > 60s」
        作为 reopen 条件，但 self._start_time 在 5 分钟任务内单调，
        所有在 60s 后被扫的格子全部不重置 → 任务后期没有可拍格子，
        飞机空闲。修复：用每个格子独立的 covered_t 时间戳（is_covered
        时写入），过 60s 自动重置回 FREE → 飞机重新巡查。

        国家一等奖 v2（2026-10-05）：max_age 接受环境变量 REV_COVER_AGE，
        5 分钟场地默认改 30s（让接力机返回时有未覆盖格可拍）。
        """
        if max_age is None:
            max_age = float(os.environ.get("REV_COVER_AGE", "60.0"))
        now = _now()
        n = 0
        for key, c in self.cells.items():
            if c.state == STATE_COVERED and (now - c.covered_t) > max_age:
                c.state = STATE_FREE
                c.confidence = 0.0
                c.owner = None
                c.lease_until = 0.0
                n += 1
        return n

    def reopen_review(self, max_age=60.0):
        """国家一等奖改进：超时未复查的 REVIEW 格强制回到 FREE。

        原版 STATE_REVIEW 永不被 reopen，只在 _reopen_covered_cells 里把
        STATE_COVERED 重置。五分钟赛事长时间运行下，已知 actor 区域在
        早些时候被「低置信扫描」过、再也没有机会复查 → 漏报。修复：
        REVIEW 格 60s 未复查强制回 FREE，重新拍卖（用独立 review_t
        字段记录进入时刻，避免污染 confidence 语义）。
        """
        now = _now()
        n = 0
        for key, c in self.cells.items():
            if c.state == STATE_REVIEW and (now - c.review_t) > max_age:
                c.state = STATE_FREE
                c.confidence = 0.0
                c.owner = None
                c.lease_until = 0.0
                n += 1
        return n

    def _compute_zones(self):
        """把地图分成 num_uavs 个区域，每个区域尽量均衡。

        国家一等奖标准改进（2026-10-03）：原版按列蛇形轮询，对 6 架、200×100m
        场地会得到极度不规则的区域（机 1 占 0-3 列、机 6 占 15-19 列），
        而 actor 出生点几乎都贴近建筑角落。区域形状与 actor 分布失配 → 某
        架机被卡到无目标区、另几架挤一起。

        修复：基于"飞机初始均匀分布质心"做 Lloyd 一次迭代的 Voronoi 划分
        （用 4 邻域 BFS 标注 Voronoi 单元格），既保证区域连通，又让 actor
        密集的角落被覆盖概率均衡。
        """
        zone_cells = {i: [] for i in range(self.num_uavs)}
        if not self.cells:
            return zone_cells
        # 1. 把 num_uavs 个种子均匀摆在地图上：按列均分 x，按行均分 y
        seeds = []
        rows = int(math.ceil(math.sqrt(self.num_uavs)))
        cols = int(math.ceil(self.num_uavs / float(rows)))
        for i in range(self.num_uavs):
            r = i // cols
            c = i % cols
            sx = self.x_min + (c + 0.5) * (self.x_max - self.x_min) / cols
            sy = self.y_min + (r + 0.5) * (self.y_max - self.y_min) / rows
            seeds.append((sx, sy))

        # 2. 一次 Voronoi 划分：每格归属距其最近的种子
        for key in self.cells.keys():
            c = self.cells[key]
            best_z, best_d = 0, float("inf")
            for z, (sx, sy) in enumerate(seeds):
                d = (c.cx - sx) ** 2 + (c.cy - sy) ** 2
                if d < best_d:
                    best_d = d
                    best_z = z
            zone_cells[best_z].append(key)

        # 3. 重算质心做一次 Lloyds 迭代修正边界（避免边缘格全部划给一架）
        new_seeds = []
        for z in range(self.num_uavs):
            cells = zone_cells[z]
            if not cells:
                new_seeds.append(seeds[z])
                continue
            mx = sum(self.cells[k].cx for k in cells) / len(cells)
            my = sum(self.cells[k].cy for k in cells) / len(cells)
            new_seeds.append((mx, my))
        # 第二次 Voronoi 划分（用更新后的质心）
        zone_cells = {i: [] for i in range(self.num_uavs)}
        for key in self.cells.keys():
            c = self.cells[key]
            best_z, best_d = 0, float("inf")
            for z, (sx, sy) in enumerate(new_seeds):
                d = (c.cx - sx) ** 2 + (c.cy - sy) ** 2
                if d < best_d:
                    best_d = d
                    best_z = z
            zone_cells[best_z].append(key)
        return zone_cells

    def get_zone_id(self, cell_key):
        """返回给定格子属于哪个区域"""
        for zone_id, keys in self.zone_cells.items():
            if cell_key in keys:
                return zone_id
        return 0

    def get_uav_zone(self, uav_id):
        """返回给定飞机负责的区域（uav_1 -> zone 0, uav_2 -> zone 1, ...）"""
        # 从 uav_1, uav_2, ... 提取数字
        try:
            num = int(uav_id.rsplit('_', 1)[-1])
            return num % self.num_uavs
        except:
            return 0

    def is_covered(self, x, y, radius_m, visible_fn=None):
        """(x,y) 处观测半径 radius_m 覆盖到的所有格标记为已覆盖，返回覆盖的格列表。

        国家一等奖 v3（2026-10-07）：同时刷新 visit_time。
        之前只更新 covered_t，visit_time 仍为 0 → score_cell 里 novelty = 1.0（从未访问过）
        → spawn 区格 (1,5) 被派 32 次仍派同一个。现在覆盖时一并写入，让 spawn 区 cell
        在 reopen 后 novelty 立刻衰减（被访问过），强制分散到远端。
        """
        covered = []
        now = _now()
        for key, c in self.cells.items():
            d = math.hypot(c.cx - x, c.cy - y)
            if (d <= radius_m and c.state != STATE_COVERED and
                    (visible_fn is None or visible_fn(c.cx, c.cy))):
                c.state = STATE_COVERED
                c.confidence = 1.0
                c.owner = None
                c.lease_until = 0.0
                c.covered_t = now
                self.visit_time[key] = now  # 关键：覆盖即刷新 visit_time，spawn 区不再 1.0 novelty
                covered.append(key)
        return covered

    def mark_review(self, key, confidence):
        """置信度不足 → 标记需复查。"""
        c = self.cells.get(key)
        if c is not None:
            c.state = STATE_REVIEW
            c.confidence = confidence
            c.owner = None
            c.lease_until = 0.0
            c.review_t = _now()


import os
import time

# ---- 早期角落优先（见 TaskAllocator._priority_bonus）----
def _envf(name, default):
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default

_PRIORITY_ON = os.environ.get('PRIORITY_CORNER', '1') not in ('0', 'false', 'False', '')
SEARCH_HARD_HOME_ZONE = os.environ.get('SEARCH_HARD_HOME_ZONE', '1') not in ('0', 'false', 'False', '')
SEARCH_HOME_HARD_S = float(os.environ.get('SEARCH_HOME_HARD_S', '120.0'))
SEARCH_Y_EDGE_DEFER_M = float(os.environ.get('SEARCH_Y_EDGE_DEFER_M', '7.0'))
_PRIORITY_DECAY = _envf('PRIORITY_DECAY', 300.0)
# 2026-10-05 国家一等奖修复：_PRIORITY_RADIUS 30→22.0。原 30m 在 (-35,-28) 单点偏置
# 下,优先级区是 x[-65,-5]×y[-58,2] 200×60=12000 m²,actor 聚集区
# (x[8,35]×y[-8,2] ≈ 27×10=270 m²) 完全被排斥在外 → 6 架机初始搜索格全在西
# 半部,t4 (8.4,-5.6) 这种东侧 actor 在 5 分钟内首次被发现的概率近 0。
# 半径 22m 让偏置集中在 (-35,-28) 附近 1500 m²,actor 区域也能被正常的
# Voronoi zone / utility() 主导。
_PRIORITY_RADIUS = _envf('PRIORITY_RADIUS', 22.0)
# 2026-10-05 国家一等奖修复：_PRIORITY_BONUS 60→40。原 60 的偏置太重压过 Voronoi
# zone + flight distance,所有 6 架机都被吸到同一个角落 → t4 类 actor 漏掉。
# 40 让 priority 与 zone/flight 大致同级,既有「早期优先」效果,又不破坏队形。
_PRIORITY_BONUS = _envf('PRIORITY_BONUS', 40.0)
_PRIORITY_MAX_UAV = int(_envf('PRIORITY_MAX_UAV', 2.0))   # 每轮最多几架被引到优先区
_PRIORITY_POINTS = []
# 2026-10-05 国家一等奖修复：默认优先级点由 (-35,-28) 改为 (-35,-22)。
# 原 (-35,-28) 在地图最西角;实测 actor 初始分布横跨 x[-50,40]、y[-30,10],
# 优先级点 -28y 偏离 actor 中心区导致优先级区也偏离 6m。
# -22 与 actor 主体区 (y[-8,2]) 距离 ≤20m,让优先覆盖既覆盖 actor 概率高的
# 西侧角落,又不排斥东侧 actor 的 Voronoi zone 主导。
for _tok in os.environ.get('PRIORITY_POINTS', '-35,-22').split(';'):
    try:
        _a, _b = _tok.split(',')
        _PRIORITY_POINTS.append((float(_a), float(_b)))
    except ValueError:
        pass


class TaskAllocator(object):
    """集中式拍卖任务分配器（每机独立效用，管理器按最高效用分配）。

    U_{i,c} = w1·收益 − w2·飞行时间 − w3·重复率 − w4·风险 + w5·均衡
    """

    def __init__(self, grid, w_gain=W_GAIN, w_flight=W_FLIGHT,
                 w_overlap=W_OVERLAP, w_risk=W_RISK, w_balance=W_BALANCE,
                 w_distance=W_DISTANCE, w_zone=W_ZONE, cruise_speed=3.0,
                 w_novelty=W_NOVELTY, novelty_horizon=NOVELTY_HORIZON,
                 flight_cap=FLIGHT_CAP):
        self.grid = grid
        self.w_gain = w_gain
        self.w_flight = w_flight
        self.w_overlap = w_overlap
        self.w_risk = w_risk
        self.w_balance = w_balance
        self.w_distance = w_distance
        self.w_spread = W_SPREAD
        self.spread_sat = max(1.0, SPREAD_SAT)
        self.w_zone = w_zone
        self.cruise_speed = cruise_speed
        self.w_novelty = w_novelty
        self.novelty_horizon = max(1.0, novelty_horizon)
        self.flight_cap = flight_cap
        self.task_count = {}  # 跟踪每架飞机的任务数
        self.visit_time = {}  # cell_key -> 上次被派位的时刻（wall），用于「最久未访问优先」
        # v13（2026-10-07）：派了但没飞到的格子（租约过期释放）记录——
        # agent_1 派 (1,5) 32 次 / agent_3 派 22 次的根因：追踪任务反复打断搜索，
        # 格子租约 90s 过期回 FREE，novelty 又涨满 → 同格无限续派。
        # expire() 写入此表，utility() 对 60s 内失败格把 novelty 压到 0.2 倍。
        self.failed_visit = {}  # cell_key -> 失败时刻（wall）
        self.failed_novelty_ttl = float(os.environ.get("FAILED_NOVELTY_TTL", "60.0"))
        # v13：近距离惩罚——飞机刚覆盖完脚下，脚下格（ft 很小）不该再拿高分。
        # 破「72% 派格 x=-44.5 / y 全 ≤-5.5」的 spawn 区锁死。
        self.min_flight_s = float(os.environ.get("MIN_FLIGHT_S", "8.0"))
        self.near_penalty = float(os.environ.get("NEAR_PENALTY", "1.0"))
        # 2026-09-27：临楼度 {cell_key: 0~1}，由 manager 在建PlanningGrid 时填充。
        # actor 出生点几乎全贴着建筑（实测 0~11m），给这类格一点正向偏置，
        # 搜索队形才会主动沿楼边铺开而不是在开阔地重复扫。
        self.cell_edge = {}
        self.w_edge = float(os.environ.get("W_EDGE", "0.6"))
        # 国家一等奖 v3（2026-10-07）：任务起始时刻（仿真钟），用于「早期近距离偏向」
        # 修复 visit_time 时钟 bug 后，远端格 novelty 立即有效 → 6 架机过早分散，
        # spawn 区 actor_4/5 蹲守不足 → find_finish 反而下降 (v12c 3→v12d 1)。
        # 解决方案：任务前 90s 给近距离格加 ~0.8/ft 的额外偏向，6 架机先在 spawn 区
        # 充分搜索找到 actor_4/5，90s 后再分散到远端 (actor_1/2/3 在 (70,22)/(18,-18)/(72,32))。
        # v24 修正（2026-10-09）：90s 蹲守实测失效 —— actor_4/5 出生即跑（第一段
        # 路径直指东区），spawn 区蹲守找不到它们，反而把 5min 覆盖锁定在 17%
        # （x∈[-44.5,43]）。缩到 30s：出生区短暂停留后立即散开，结合搜索巡航提速
        # （SEARCH_CRUISE_SPEED=3.0）显著扩大覆盖面积。
        self.mission_start = None
        self.early_phase_sec = float(os.environ.get("EARLY_PHASE_SEC", "30.0"))
        # ---- NBV ----
        self.vis_enable = VIS_ENABLE
        # ---- 国家一等奖标准：目标预瞄 ----
        # 在 utility() 中给「靠近已知 actor」的格子加正向偏置，引导飞
        # 机主动斜插到追踪接力区。{ (x,y): wall_timestamp }，
        # 超过 8s 自动失效（actor 移动 / 死亡）。
        self._hot_targets = {}
        self._hot_target_ttl = float(os.environ.get("HOT_TARGET_TTL", "8.0"))
        # 国家一等奖 v2（2026-10-05）：本轮已分配的「被 hot target 吸引」机数计数，
        # 配合 HOT_TARGET_MAX_UAV 防止 6 架全被同一目标吸过去导致搜索/接力队形坍塌。
        self._hot_used = 0
        self._hot_max = int(float(os.environ.get("HOT_TARGET_MAX_UAV", "2")))
        # 国家一等奖 v3（2026-10-07）：任务起始时刻（仿真钟），用于早期近距离偏向
        # 从首架可执行搜索任务的飞机开始计时。manager 在飞机起飞前就创建分配器，
        # 若在构造时计时，30s 分区期会在首轮派位前耗尽。
        self.mission_start = None
        self.w_vis = W_VIS
        self.vis_stale = max(1.0, VIS_STALE)
        self.vis_radius = VIS_RADIUS
        self.vis_commit = VIS_COMMIT
        self.vis_gain_sat = max(1.0, VIS_GAIN_SAT)
        self.visible_set = {}   # cell_key -> [从该视点可见的 cell_key]
        self.last_seen = {}     # cell_key -> 上次被真正看见（有 LOS）的 wall time
        self._los = None        # LineOfSight 实例，由 manager 注入
        self.last_allocation = []  # 本轮最终分配诊断，供 manager 落盘

    # ---------------- NBV：LOS 感知的可见增益 ----------------
    def set_visibility(self, visible_set):
        """注入预计算的可见集 {视点格 key: [可见格 key...]}。"""
        self.visible_set = visible_set or {}

    def set_los(self, los):
        """注入 LineOfSight 实例（用于飞行途中的实时可见更新）。"""
        self._los = los

    # === 国家一等奖标准：目标预瞄接口 ===
    def register_hot_target(self, x, y, now=None):
        """manager 写入「已知 actor 位置」。超过 TTL 自动失效。

        让 utility() 给邻近该位置的格子加分，引导飞机主动飞向追踪区，
        而不是继续做无效面覆盖。注意：位置坐标要做一次量化到 1m 网格，
        避免同一目标高频抖动把整张表重写。
        """
        if now is None:
            now = _now()
        # 清理过期项
        stale = [k for k, t in self._hot_targets.items() if now - t > self._hot_target_ttl]
        for k in stale:
            del self._hot_targets[k]
        key = (round(x, 1), round(y, 1))
        self._hot_targets[key] = now

    def clear_hot_target(self, x, y):
        key = (round(x, 1), round(y, 1))
        self._hot_targets.pop(key, None)

    def mark_seen(self, x, y, now=None):
        """飞机在 (x,y) 时真正看见（有 LOS）的格，刷新 last_seen。返回刷新格数。"""
        if not self.vis_enable or self._los is None:
            return 0
        if now is None:
            now = _now()
        n = 0
        for k, c in self.grid.cells.items():
            if math.hypot(c.cx - x, c.cy - y) > self.vis_radius:
                continue
            if not self._los.visible(x, y, c.cx, c.cy):
                continue
            self.last_seen[k] = now
            n += 1
        return n

    def commit_visible(self, cell_key, now=None):
        """派单即承诺：把该视点可见集的 last_seen 刷成 now。

        次模贪心的关键一步 —— 别的机再看同一片区域增益≈0，队形自动散开，
        不会像 W_EDGE 那样把 6 架吸到同一批格子上。
        """
        if not self.vis_enable or not self.vis_commit:
            return 0
        if now is None:
            now = _now()
        keys = self.visible_set.get(cell_key)
        if not keys:
            return 0
        for k in keys:
            self.last_seen[k] = now
        return len(keys)

    def vis_gain(self, cell_key, now=None):
        """从该视点能看见多少「久未见」的区域。

        注意是**计数**不是平均 —— 平均会让「看见 2 格」和「看见 20 格」
        在初始状态下都等于 1.0，失去区分度（自检已复现该 bug）。
        计数后用 VIS_GAIN_SAT 饱和，避免大视点一票压死其他项。
        """
        if not self.vis_enable:
            return 0.0
        keys = self.visible_set.get(cell_key)
        if not keys:
            return 0.0
        if now is None:
            now = _now()
        raw = 0.0
        for k in keys:
            t = self.last_seen.get(k, 0.0)
            if t <= 0.0:
                raw += 1.0
            else:
                d = (now - t) / self.vis_stale
                raw += 1.0 if d >= 1.0 else (d if d > 0.0 else 0.0)
        return min(1.0, raw / self.vis_gain_sat)

    def _flight_time(self, uav_x, uav_y, cell):
        """预计飞行时间 = 直线距离 / 巡航速度（秒）。"""
        return math.hypot(cell.cx - uav_x, cell.cy - uav_y) / self.cruise_speed

    def _overlap_rate(self, cell, assigned_positions, other_uav_positions=None):
        """与他机重复率：本格到「其他机已分配格」或「其他机当前位置」的接近度 [0,1]。

        同时考虑：
        1. 已分配格的位置（将要去的格子）
        2. 其他飞机的当前位置（正在执行的格子）
        这样避免多机同时飞向相邻格子导致冲突。
        """
        if not assigned_positions and not other_uav_positions:
            return 0.0

        # 合并所有位置
        all_positions = list(assigned_positions)
        if other_uav_positions:
            all_positions.extend(other_uav_positions.values())

        # 找最近的冲突源
        nearest = min(math.hypot(cell.cx - px, cell.cy - py)
                      for px, py in all_positions)

        # 阈值缩小到 1.5 倍格子边长，更严格避免冲突
        threshold = 1.5 * self.grid.cell_m
        if nearest >= threshold:
            return 0.0
        return max(0.0, 1.0 - nearest / threshold)

    def _path_risk(self, cell, risk_map):
        """路径风险：risk_map 给出每格风险值 [0,1]（建筑/密集区风险高）。"""
        if not risk_map:
            return 0.0
        return risk_map.get((cell.cx, cell.cy), 0.0)

    def utility(self, uav_id, uav_x, uav_y, cell_key, assigned_positions, risk_map=None, task_counts=None, other_uavs=None, other_executing=None):
        """单机对单格效用 U_{i,c}。收益：未搜索=1，需复查=0.5。

        U = w1·gain - w2·flight_time - w3·overlap - w4·risk + w5·balance - w6·distance_penalty

        other_uavs: {uav_id: (x,y)} 其他飞机的当前位置，用于计算机间距离惩罚。
        """
        cell = self.grid.cell(cell_key)
        if cell is None:
            return float("-inf")
        gain = 1.0 if cell.state == STATE_FREE else (0.5 if cell.state == STATE_REVIEW else 0.0)
        if gain == 0.0:
            return float("-inf")   # 已覆盖/执行中不可再分配
        ft = self._flight_time(uav_x, uav_y, cell)
        # 封顶：否则远距格的惩罚量级远超收益，拍卖永远只选脚下那一格
        if ft > self.flight_cap:
            ft = self.flight_cap
        # 传递其他飞机当前位置，用于计算与"正在飞向的格子"的冲突
        ov = self._overlap_rate(cell, assigned_positions, other_executing)
        rk = self._path_risk(cell, risk_map)
        # 均衡因子：任务少的飞机获得加成（任务数最少时 balance=1，最多时 balance=0）
        if task_counts is not None and len(task_counts) > 0:
            min_count = min(task_counts.values())
            max_count = max(task_counts.values())
            if max_count > min_count:
                # 线性映射：最少任务 → 1，最多任务 → 0
                balance = (max_count - task_counts.get(uav_id, 0)) / (max_count - min_count)
            else:
                balance = 1.0  # 任务数相同，都给满分
        else:
            balance = 0.0
        # 机间距离惩罚：距离其他飞机太近扣分
        dist_penalty = 0.0
        if other_uavs:
            for oid, (ox, oy) in other_uavs.items():
                if oid == uav_id:
                    continue
                d = math.hypot(cell.cx - ox, cell.cy - oy)
                if d < MIN_UAV_DIST:
                    # 距离越近惩罚越大
                    dist_penalty += (MIN_UAV_DIST - d) / MIN_UAV_DIST

        # 分散奖励：到「最近的其他飞机」越远越好。
        # 只有 dist_penalty（12m 内才非零）是不够的 —— 超过 12m 后零梯度，
        # 机队会长期挤在 20~30m 内互相重叠，实测吞吐掉 4 倍（63 格 → 37 格）。
        spread = 0.0
        _dmin = None
        if other_uavs:
            for oid, (ox, oy) in other_uavs.items():
                if oid == uav_id:
                    continue
                d = math.hypot(cell.cx - ox, cell.cy - oy)
                if _dmin is None or d < _dmin:
                    _dmin = d
        if _dmin is not None:
            spread = min(_dmin, self.spread_sat) / self.spread_sat

        # 区域因子：搜索阶段优先选择自己区域内的格子
        uav_zone = self.grid.get_uav_zone(uav_id)
        cell_zone = self.grid.get_zone_id(cell_key)
        # v20（2026-10-09）：起飞期强分区铺开。
        # v19 尸检（logs_20261009_000759）：brown 东区 (11,-16) 297s 才首检、red2 全场零检测
        # （被 12~20m 惊动盲区吓跑后一路逃到 x=120 深处）——根因是起飞期 zone_bonus 差异仅
        # 0.5×1.2=0.6，近格 gain+novelty 全满直接压过 zone，6 架全被派在西区 col 2~4
        # （manager 分配日志实锤），东区/北区责任机 h480_1/3/5 全被留在西区。
        # 修复：early_phase 内外区 0.0（差异 1.2×W_ZONE=1.2，配合 spread 扎堆惩罚，
        # 本区远格效用稳定压过外区近格 → 起飞即直奔本责任区，90s 内 6 区铺满）；
        # early_phase 结束后恢复 0.5 软约束，允许跨区支援。
        if cell_zone == uav_zone:
            zone_bonus = 1.0
        elif self.mission_start is not None and (_now() - self.mission_start) < self.early_phase_sec:
            zone_bonus = 0.0   # 起飞期：只搜本责任区（强倾斜，非硬过滤——外区格子仍有 gain/novelty/spread）
        else:
            zone_bonus = 0.5   # 常规期：软约束

        # 新颖性：越久没被派过（含从未派过）分越高 —— 面覆盖真正的主排序键
        _last = self.visit_time.get(cell_key, 0.0)
        if _last <= 0.0:
            novelty = 1.0
        else:
            novelty = (_now() - _last) / self.novelty_horizon
            if novelty > 1.0:
                novelty = 1.0
            elif novelty < 0.0:
                novelty = 0.0
        # v13：复查格（REVIEW）novelty 打折——复查价值远低于首搜，
        # 否则近处复查格靠 novelty+spread 全额奖励反复刷屏（agent_1 同格 32 次）
        if gain < 1.0:
            novelty *= 0.3
        # v13：租约过期（派了没飞到）的格子 60s 内 novelty 压到 0.2 倍，
        # 打断「追踪打断搜索→租约过期→同格续派」死循环
        _fv = self.failed_visit.get(cell_key)
        if _fv is not None:
            if _now() - _fv < self.failed_novelty_ttl:
                novelty *= 0.2
            else:
                del self.failed_visit[cell_key]

        # === 国家一等奖标准：目标预瞄奖励 ===
        # 当 manager 已知 actor 位置时（通过 dispatch_target 写入），
        # 邻近 actor 的格子大幅加分，使飞机主动「斜插」去追踪接力区域，
        # 而不是纯面覆盖。但只对**离 actor 在 60m 内**的格子加分，
        # 避免把全队吸到单一目标。
        # 国家一等奖 v2（2026-10-05）：HOT_TARGET_MAX_UAV 上限 —— 当本轮已被吸引的
        # 机数 >= 上限, 后续飞机不再拿 target_bonus, 避免 6 架全被吸到一处。
        target_bonus = 0.0
        if hasattr(self, "_hot_targets") and self._hot_targets \
                and getattr(self, "_hot_used", 0) < getattr(self, "_hot_max", 2):
            for (tx, ty), _age in list(self._hot_targets.items()):
                d = math.hypot(cell.cx - tx, cell.cy - ty)
                if d <= 60.0:
                    target_bonus += 4.0 * (1.0 - d / 60.0)
        # 限幅：避免相邻 actor 让单格加分爆炸
        if target_bonus > 6.0:
            target_bonus = 6.0

        # 临楼偏置：贴着建筑的格子更容易藏着 actor，同等条件下优先扫
        edge_bonus = self.w_edge * float(self.cell_edge.get(cell_key, 0.0))

        # LOS 感知 NBV：这个视点能看见多少「久未见」的区域（0~1）。
        # 与 edge_bonus 的区别：它是**动态**的，看过的/别机承诺要看的会立刻贬值。
        vis_term = self.w_vis * self.vis_gain(cell_key)

        # 国家一等奖 v3（2026-10-07）：早期近距离偏向 —— 任务前 90s 给近距离格加权。
        # 修复 visit_time 时钟 bug 后，远端格 novelty 立即有效 → 6 架机过早分散。
        # 用 1/ft 形式的距离反比项，让 6 架机先蹲守 spawn 区（actor_4/5 在 -32,-27 / -35,-28），
        # 之后再随时间衰减 → 远端 actor_1/2/3 (70,22)/(18,-18)/(72,32) 自然被覆盖。
        # 早期权重 1.0/ft ≈ 0.07~0.33（15~60m），足以压过 spawn 区 - 0.025*ft 的飞行代价。
        early_close_bonus = 0.0
        if self.mission_start is not None:
            _mt = _now() - self.mission_start
            if _mt < self.early_phase_sec:
                _decay = 1.0 - _mt / self.early_phase_sec
                # v13：封顶 0.35——旧式 1/max(ft,0.5) 在 ft=0.5s 时 bonus=2.0，
                # 直接翻倍近格效用，是 spawn 区锁死的主因之一。
                # 封顶后近格最多 +0.35，远格（ft=30s）+0.03，梯度保留但不翻转排序。
                early_close_bonus = _decay * min(1.0 / max(ft, 0.5), 0.35)

        # v13：近距离惩罚——ft < 8s（12m）的格子减分，推飞机向外铺开。
        # 起飞阶段最近未搜格通常 >12m 不受影响；覆盖完脚下后不再原地续派。
        near_pen = 0.0
        if ft < self.min_flight_s:
            near_pen = self.near_penalty * (1.0 - ft / self.min_flight_s)

        return (self.w_gain * gain
                + self.w_novelty * novelty
                - self.w_flight * ft
                - self.w_overlap * ov
                - self.w_risk * rk
                + self.w_balance * balance
                - self.w_distance * dist_penalty
                + self.w_zone * zone_bonus
                + edge_bonus
                + self.w_spread * spread
                + vis_term
                + target_bonus
                + early_close_bonus
                - near_pen)

    def _in_priority(self, key):
        c = self.grid.cell(key)
        if c is None or not _PRIORITY_ON:
            return False
        for px, py in _PRIORITY_POINTS:
            if math.hypot(c.cx - px, c.cy - py) <= _PRIORITY_RADIUS:
                return True
        return False

    def _priority_bonus(self, key, n_prio=0, now=None, uav_id=None):
        """早期偏置：让「已知最晚才被发现的角落」在任务前段就被覆盖。
        
        背景：actor_5 出生在 (-35,-28) 的建筑区，是搜索最晚触达的角落；
        elapsed = max(各目标发现时刻)，它一个人决定了扣分。
        
        注意：建筑物内部的格会被 manager 直接标成 STATE_COVERED（飞不进去，
        永不参与拍卖），所以不能给「角落格本身」加偏置 —— 要给**建筑周边**
        的空格加，让飞机沿建筑边缘扫，靠探测半径覆盖到里面。
        
        环境变量（默认开启，关掉用 PRIORITY_CORNER=0）：
          PRIORITY_POINTS  优先覆盖的坐标，默认 '-35,-28'
          PRIORITY_RADIUS  影响半径(米)，默认 30
          PRIORITY_BONUS   效用偏置，默认 60（约等于「多飞 60 秒也值得」）
          PRIORITY_DECAY   偏置在多少秒内线性衰减到 0，默认 300
        """
        if not _PRIORITY_ON:
            return 0.0
        # 优先角落的巨额奖励只能给其责任区飞机。飞机是分批起飞/逐架进入
        # 拍卖的，按「本轮最多 N 架」计数会在每轮清零，导致六架先后奔向同一角落。
        if uav_id is not None and self.grid.get_zone_id(key) != self.grid.get_uav_zone(uav_id):
            return 0.0
        if n_prio >= _PRIORITY_MAX_UAV:
            return 0.0   # 本轮额度用完了，别把所有机都吸过去
        c = self.grid.cell(key)
        if c is None:
            return 0.0
        if not hasattr(self, '_t0'):
            self._t0 = _now()
        if now is None:
            now = _now()
        age = now - self._t0
        if age >= _PRIORITY_DECAY:
            return 0.0
        best = 0.0
        for px, py in _PRIORITY_POINTS:
            d = math.hypot(c.cx - px, c.cy - py)
            if d <= _PRIORITY_RADIUS:
                b = _PRIORITY_BONUS * (1.0 - d / _PRIORITY_RADIUS)
                if b > best:
                    best = b
        return best * (1.0 - age / _PRIORITY_DECAY)

    def allocate(self, uavs, risk_map=None):
        """集中式分配一轮。uavs: {id: (x,y)}，返回 {id: cell_key 或 None}。

        贪心：按当前「收益最高」逐机分配（每机取剩余格中自身效用最大者），
        已分配格即时从候选池移除（降低重复率），实现轻量拍卖。
        """
        if uavs and self.mission_start is None:
            self.mission_start = _now()
        # 跟踪各机任务数（已分配+执行中的）
        task_counts = {uid: 0 for uid in uavs}
        for c in self.grid.cells.values():
            if c.state == STATE_ASSIGNED and c.owner in task_counts:
                task_counts[c.owner] += 1

        remaining = set(self.grid.uncovered_cells())
        # A camera one cell inside the north/south border still sees across
        # that border (the visual search radius is 10 m).  The previous
        # allocator sent UAV5 alternately to y=57.5 and y=64.5 for ~200 s,
        # while blue/white actors at y=33..37 remained unseen. Defer the
        # outer row until interior candidates are exhausted; keep x-edge
        # cells because actors can stop against the east/west boundary.
        if SEARCH_Y_EDGE_DEFER_M > 0.0:
            inner = {key for key in remaining
                     if self.grid.y_min + SEARCH_Y_EDGE_DEFER_M <= self.grid.cell(key).cy <=
                     self.grid.y_max - SEARCH_Y_EDGE_DEFER_M}
            if inner:
                remaining = inner
        assigned = {}          # uav_id -> cell_key
        assigned_positions = []  # [(cx,cy)] 已分配格中心，供重复率计算
        self.last_allocation = []
        # 国家一等奖 v2（2026-10-05）：每轮拍卖重置 hot target 配额, 由 utility()
        # 内部检查 _hot_used >= _hot_max 时主动放弃该偏置, 防止 6 架全被吸到一处。
        self._hot_used = 0

        # 预先获取所有飞机的位置，用于计算机间距离惩罚
        all_uavs = dict(uavs)

        # 预先获取正在执行任务的飞机的目标位置（避免多机同时飞向相邻格子）
        executing_uav_targets = {}  # uav_id -> (target_x, target_y)
        for c in self.grid.cells.values():
            if c.state == STATE_ASSIGNED and c.owner:
                executing_uav_targets[c.owner] = (c.cx, c.cy)

        n_prio = 0
        for uav_id, (ux, uy) in uavs.items():
            best_key = None
            best_u = float("-inf")

            # 本机分配时，排除已分配格的位置，但不排除其他飞机的当前位置
            other_uavs = {oid: pos for oid, pos in all_uavs.items() if oid != uav_id}
            # 也排除正在执行任务的其他飞机的目标位置
            other_executing = {oid: pos for oid, pos in executing_uav_targets.items() if oid != uav_id}

            # Spread into six sectors at takeoff, then allow nearest-aircraft
            # support across sector lines. A sector can have many remaining
            # cells even when its aircraft is needed beside a live target.
            hard_home = (SEARCH_HARD_HOME_ZONE and
                         self.mission_start is not None and
                         _now() - self.mission_start < SEARCH_HOME_HARD_S)
            home_zone = self.grid.get_uav_zone(uav_id)
            home = [key for key in remaining
                    if self.grid.get_zone_id(key) == home_zone]
            candidates = home if hard_home and home else remaining
            for key in candidates:
                u = self.utility(uav_id, ux, uy, key, assigned_positions, risk_map, task_counts, other_uavs, other_executing) + self._priority_bonus(key, n_prio, uav_id=uav_id)
                if u > best_u:
                    best_u = u
                    best_key = key
            if best_key is not None:
                assigned[uav_id] = best_key
                self.visit_time[best_key] = _now()
                self.commit_visible(best_key)
                c = self.grid.cell(best_key)
                c.state = STATE_ASSIGNED
                c.owner = uav_id
                remaining.discard(best_key)
                assigned_positions.append((c.cx, c.cy))
                task_counts[uav_id] += 1  # 更新任务计数
                if self._in_priority(best_key):
                    n_prio += 1
                # 国家一等奖 v2（2026-10-05）：被 hot target 吸引过的飞机
                # 在本轮内计入 _hot_used, 超过 HOT_TARGET_MAX_UAV 后 utility()
                # 不再给该机加 target_bonus, 防止 6 架全被吸到一处。
                # best_u 已被 +self._priority_bonus 干扰, 这里用几何判定更稳：
                # best_key 是否在 hot target 60m 范围内？
                if self._hot_targets:
                    _bc = self.grid.cell(best_key)
                    if _bc is not None:
                        for (tx, ty), _age in self._hot_targets.items():
                            if math.hypot(_bc.cx - tx, _bc.cy - ty) <= 60.0:
                                self._hot_used += 1
                                break
                cell = self.grid.cell(best_key)
                self.last_allocation.append({
                    "uav_id": uav_id,
                    "cell_key": best_key,
                    "target_x": cell.cx,
                    "target_y": cell.cy,
                    "uav_x": ux,
                    "uav_y": uy,
                    "distance_m": math.hypot(cell.cx - ux, cell.cy - uy),
                    "utility": best_u,
                })
            else:
                assigned[uav_id] = None
                self.last_allocation.append({
                    "uav_id": uav_id,
                    "cell_key": None,
                    "target_x": None,
                    "target_y": None,
                    "uav_x": ux,
                    "uav_y": uy,
                    "distance_m": None,
                    "utility": None,
                })
        return assigned


class LeaseManager(object):
    """任务租约：掉线/卡死/超时 → 租约到期自动重分配，防重复搜索。"""

    def __init__(self, grid, duration=LEASE_DURATION):
        self.grid = grid
        self.duration = duration

    def grant(self, uav_id, cell_key, now, duration=None):
        """授租约，duration 默认自适应（基于飞行距离）。"""
        c = self.grid.cell(cell_key)
        if c is None:
            return False
        # 自适应租约：飞行时间 × 系数 + 缓冲，最短 30s
        if duration is None:
            # 国家一等奖 bug 修复（2026-10-04）：原代码用 self.grid.origin
            # 但 CoverageGrid 没有 origin 属性（只有 x_min/y_min）→ 离线
            # 自测 + 真实运行的第 1 次 grant 就 AttributeError → 租约制完全失效，
            # 所有飞机各自抢格、多机扎堆。修复：使用 (x_min, y_min) 作为参考原点。
            dist = math.hypot(c.cx - self.grid.x_min, c.cy - self.grid.y_min)
            duration = max(LEASE_DURATION, dist / LEASE_MIN_SPEED * LEASE_FACTOR + 10.0)
        c.state = STATE_ASSIGNED
        c.owner = uav_id
        c.lease_until = now + duration
        return duration  # 返回实际租约时长（用于日志）

    def renew(self, uav_id, cell_key, now):
        """执行机持续上报 → 续租。"""
        c = self.grid.cell(cell_key)
        if c is not None and c.owner == uav_id and c.state == STATE_ASSIGNED:
            c.lease_until = now + self.duration
            return True
        return False

    def expire(self, now):
        """收集所有租约到期的格，回退为未分配，返回待重分配的格 key 列表。"""
        expired = []
        for key, c in self.grid.cells.items():
            if c.state == STATE_ASSIGNED and c.lease_until < now:
                c.state = STATE_FREE
                c.owner = None
                c.lease_until = 0.0
                expired.append(key)
                # v13：调用方（manager）负责把过期格写入 allocator.failed_visit
                # （LeaseManager 与 TaskAllocator 是独立对象，此处无引用）
        return expired

    def force_expire(self, uav_id, cell_key):
        """v18（2026-10-07）：立即回收指定租约（不等 TTL 自然到期）。

        背景：manager._renew_leases 的 LEASE_MAX_HOLD 超时分支旧逻辑只「停止
        续租等 TTL 到期」，但它 pop 掉 _lease_hold_since 后下一周期 setdefault
        重置计时又恢复续租 → TTL 永远追不上（v17 实测 agent_0 (3,7) 每 91s
        循环告警一次、全程 0 次「租约到期重分配」、6 架全被 ASSIGNED 格锁死
        → 202s 后零分配死锁）。超时分支必须主动强制回收。
        """
        c = self.grid.cell(cell_key)
        if c is not None and c.state == STATE_ASSIGNED and c.owner == uav_id:
            c.state = STATE_FREE
            c.owner = None
            c.lease_until = 0.0
            return True
        return False


class LineOfSight(object):
    """视线（LOS）遮挡判定：无人机与目标之间是否被建筑挡住。

    规则3 的「几何判定」核心：仅当水平距离 < DETECT_RADIUS **且** 视线无遮挡
    才算感知到目标（否则隔着楼也算看到，不符合实际）。

    实现用自写的栅格 DDA 射线步进（Amanatides-Woo 思路，变量名与注释自建）：
    从观测者沿连线逐格步进，途中若遇到障碍栅格 → 判定被遮挡。
    只依赖「障碍栅格查询函数」+ 栅格原点/分辨率，不绑定具体地图实现，便于单测。

    注意 origin：地图栅格原点通常不是 (0,0)（本项目为 (-100,-50)），
    坐标→栅格必须减去 origin，否则会整体错位（曾导致 LOS 恒为 False）。
    """

    def __init__(self, is_blocked, cell_size=0.5, origin=(0.0, 0.0)):
        """
        is_blocked : callable(ix, iy) -> bool，判断栅格是否被占用
        cell_size  : 栅格边长 m（与调用方地图一致）
        origin     : 栅格原点 (x0, y0)（与调用方地图一致）
        """
        self.is_blocked = is_blocked
        self.cell_size = float(cell_size)
        self.origin = (float(origin[0]), float(origin[1]))

    def _to_cell(self, x, y):
        """世界坐标 -> 栅格索引（必须减 origin）。"""
        return (int(math.floor((x - self.origin[0]) / self.cell_size)),
                int(math.floor((y - self.origin[1]) / self.cell_size)))

    def visible(self, ox, oy, tx, ty):
        """(ox,oy) 能否看见 (tx,ty)（无遮挡返回 True）。

        端点所在格不算遮挡（目标/观测者可能贴着障碍边缘）。
        """
        ix, iy = self._to_cell(ox, oy)
        gx, gy = self._to_cell(tx, ty)
        if (ix, iy) == (gx, gy):
            return True   # 同一格必然可见

        dx, dy = tx - ox, ty - oy
        step_x = 1 if dx > 0 else (-1 if dx < 0 else 0)
        step_y = 1 if dy > 0 else (-1 if dy < 0 else 0)

        # 到下一条格边界的参数距离（用「格边界的绝对坐标」减去起点，故含 origin）
        inf = float("inf")
        if step_x != 0:
            next_x = self.origin[0] + (ix + (1 if step_x > 0 else 0)) * self.cell_size
            t_max_x = (next_x - ox) / dx
            t_delta_x = self.cell_size / abs(dx)
        else:
            t_max_x, t_delta_x = inf, inf
        if step_y != 0:
            next_y = self.origin[1] + (iy + (1 if step_y > 0 else 0)) * self.cell_size
            t_max_y = (next_y - oy) / dy
            t_delta_y = self.cell_size / abs(dy)
        else:
            t_max_y, t_delta_y = inf, inf

        # 逐格步进，直到抵达目标格（含端点豁免逻辑）
        guard = 0
        max_steps = int(math.hypot(gx - ix, gy - iy)) + 4
        while (ix, iy) != (gx, gy) and guard < max_steps:
            guard += 1
            if t_max_x < t_max_y:
                ix += step_x
                t_max_x += t_delta_x
            else:
                iy += step_y
                t_max_y += t_delta_y
            if (ix, iy) == (gx, gy):
                break          # 到达目标格，不算遮挡
            if self.is_blocked(ix, iy):
                return False   # 中途撞障碍 → 被遮挡
        return True


class TargetTracker(object):
    """目标观测 + 确认消除 + 追踪接力（对应比赛规则 3/4/5）。

    规则映射：
      规则 3  感知到无人机 → 目标逃跑 2m/s 并向同伙广播（仿真侧实现，本类只记录）
      规则 4  被持续感知 30s 仍未消除 → 目标瞬移躲藏（本类超时判定 → 清空确认计时）
      规则 5  连续 15s 正确广播 ID + 坐标 → 消除（本类累计「连续确认时长」）

    关键：确认计时是**连续**的。中途丢失视野（超过 lose_dist 或无人观测）
    必须清零重来，否则规则 5 的「连续 15 秒」就形同虚设。
    """

    def __init__(self, confirm_time=CONFIRM_TIME, safe_dist=RELAY_SAFE_DIST,
                 lose_dist=RELAY_LOSE_DIST, evade_time=EVADE_TIME):
        self.confirm_time = confirm_time     # 规则5：连续确认 15s 判定消除
        self.safe_dist = safe_dist
        self.lose_dist = lose_dist
        self.evade_time = evade_time         # 规则4：被感知 30s 未消除 → 瞬移
        self.targets = {}   # target_id -> dict(...)

    # ---------------- 目标生命周期 ----------------
    def add_target(self, target_id, x, y, vx=0.0, vy=0.0, conf=1.0, tracker=None, now=0.0):
        """登记一个新目标。`now` 视为**首次被感知时刻**（管理器是因检测才得知它），
        故连续确认计时从此刻起算 —— 否则规则5 的 15s 会被推迟一个采样周期。"""
        self.targets[target_id] = {
            "x": x, "y": y, "vx": vx, "vy": vy,
            "conf": conf, "tracker": tracker,
            "confirm_until": now + self.confirm_time,   # 兼容旧接口：名义确认截止
            "confirm_since": now,    # 连续确认起始时刻（已被感知）
            "tracked_since": now,    # 首次被感知时刻（规则4 的 30s 计时起点）
            "evaded": False,         # 是否已触发瞬移（需重新搜索）
            "eliminated": False,     # 是否已判定消除
        }

    def remove_target(self, target_id):
        return self.targets.pop(target_id, None) is not None

    # ---------------- 规则 5：连续 15s 确认 → 消除 ----------------
    def observe(self, target_id, now, observer=None):
        """记录一次「本时刻有无人机正确观测到该目标」。

        返回 True 表示本次观测使该目标达成连续确认（可判定消除）。
        """
        t = self.targets.get(target_id)
        if t is None or t["eliminated"]:
            return False
        if t["confirm_since"] is None:
            t["confirm_since"] = now      # 开始/重新开始连续计时
        if observer is not None:
            t["tracker"] = observer
        held = now - t["confirm_since"]
        t["conf"] = min(1.0, held / self.confirm_time)   # 确认进度作为置信度
        if held >= self.confirm_time:
            t["eliminated"] = True
            t["conf"] = 1.0
            return True
        return False

    def lose(self, target_id, now):
        """本时刻没有观测到该目标 → 连续确认计时清零（规则5 要求「连续」）。"""
        t = self.targets.get(target_id)
        if t is None or t["eliminated"]:
            return
        t["confirm_since"] = None
        t["conf"] = 0.0

    def confirm_progress(self, target_id, now):
        """当前连续确认进度 [0,1]，用于可视化/日志。"""
        t = self.targets.get(target_id)
        if t is None or t["confirm_since"] is None:
            return 0.0
        return min(1.0, (now - t["confirm_since"]) / self.confirm_time)

    # ---------------- 规则 4：被感知 30s 未消除 → 瞬移 ----------------
    def check_evade(self, target_id, now):
        """规则4：目标被感知累计超过 30s 仍未消除 → 应瞬移。

        注意：这里的 30s 是「被感知」的累计时长（tracked_since 起算），
        与规则 5 的「连续确认」是两个独立计时。
        返回 True 表示本次调用触发了瞬移（调用方负责搬移目标并重置状态）。
        """
        t = self.targets.get(target_id)
        if t is None or t["eliminated"] or t["evaded"]:
            return False
        if now - t["tracked_since"] >= self.evade_time:
            t["evaded"] = True
            t["confirm_since"] = None    # 瞬移后确认计时必须清零
            t["conf"] = 0.0
            return True
        return False

    def relocate(self, target_id, x, y, now):
        """瞬移目标到新位置，并重置所有计时（重新开始搜索/确认）。"""
        t = self.targets.get(target_id)
        if t is None:
            return False
        t["x"], t["y"] = x, y
        t["vx"] = t["vy"] = 0.0
        t["tracker"] = None
        t["confirm_since"] = None
        t["tracked_since"] = now     # 重新开始 30s 计时
        t["evaded"] = False
        t["conf"] = 0.0
        return True

    def pending_targets(self):
        """返回所有「未被消除」的目标 id（仍在场上的）。"""
        return [tid for tid, t in self.targets.items() if not t["eliminated"]]

    # ---------------- 追踪接力（保留原有设计） ----------------
    def predict(self, target_id, dt):
        """卡尔曼预测位置的简化版：匀速外推（自写，非完整卡尔曼）。"""
        t = self.targets.get(target_id)
        if t is None:
            return None
        return (t["x"] + t["vx"] * dt, t["y"] + t["vy"] * dt, t["vx"], t["vy"])

    def needs_relay(self, target_id, now):
        """追踪机是否需要接力：确认超时 或 追踪机为 None。"""
        t = self.targets.get(target_id)
        if t is None:
            return False
        if t["tracker"] is None:
            return True
        return now > t["confirm_until"]

    def pick_relay(self, target_id, uavs, cruise_speed=3.0):
        """选「预计最快到达观察点」的无人机接力（返回 uav_id 或 None）。

        uavs: {id: (x,y)}，观察点取目标预测位置处。
        """
        t = self.targets.get(target_id)
        if t is None:
            return None
        tx, ty, _, _ = self.predict(target_id, 0.0)
        best_id, best_t = None, float("inf")
        for uav_id, (ux, uy) in uavs.items():
            if uav_id == t["tracker"]:
                continue
            ft = math.hypot(tx - ux, ty - uy) / cruise_speed
            if ft < best_t:
                best_t = ft
                best_id = uav_id
        return best_id

    def handover(self, target_id, new_tracker, now):
        """接力：新机继承同一目标 ID 与确认计时（重新计时 15s）。"""
        t = self.targets.get(target_id)
        if t is None:
            return False
        t["tracker"] = new_tracker
        t["confirm_until"] = now + self.confirm_time
        return True


# ============================ 单元自测 ============================
if __name__ == "__main__":
    # 国家一等奖 bug 修复（2026-10-04）：测试 1 用 (-50,0)/(50,0) 分配，
    # 开启 PRIORITY_CORNER 时 corner=(-35,-28) 30m 半径会让 (9,2)=(-33.5,-32.5)
    # 拿到 _PRIORITY_BONUS=60 的偏置压过 Voronoi zone，把 uav_2 吸到 (9,2)。
    # 测试断言只验证「不重复 + uav_1→左 / uav_2→右」，与 priority corner 无关
    # → 测试入口用 sys.modules 直接拿到本模块并修改 _PRIORITY_ON。
    import sys as _sys
    _self = _sys.modules[__name__]
    _self._PRIORITY_ON = False
    # 建 200×100m 栅格（GRID_SIZE_M=7.0m → 29×15 格，ceil 上取整）
    g = CoverageGrid(-100, 100, -50, 50)
    assert g.nx == 29 and g.ny == 15, (g.nx, g.ny)
    print("栅格尺寸 OK: %d×%d 格（GRID_SIZE_M=%.1fm），共 %d 格" %
          (g.nx, g.ny, GRID_SIZE_M, len(g.cells)))

    # ---- 1) 拍卖分配：两机不重复搜索 ----
    alloc = TaskAllocator(g)
    uavs = {"uav_1": (-50.0, 0.0), "uav_2": (50.0, 0.0)}
    assign = alloc.allocate(uavs)
    keys = list(assign.values())
    assert len(set(keys)) == 2 and None not in keys, assign
    # 各机应被分到离自己近的格
    c1, c2 = g.cell(assign["uav_1"]), g.cell(assign["uav_2"])
    assert c1.cx < c2.cx, (c1.cx, c2.cx)
    print("拍卖分配 OK: uav_1→(%.0f,%.0f) uav_2→(%.0f,%.0f)，不重复" %
          (c1.cx, c1.cy, c2.cx, c2.cy))

    # ---- 2) 任务租约：到期重分配 ----
    lm = LeaseManager(g)
    # 拍卖只置 state/owner，租约需管理器随后授予（两机都授）
    lm.grant("uav_1", assign["uav_1"], now=0.0)
    lm.grant("uav_2", assign["uav_2"], now=0.0)
    assert g.cell(assign["uav_1"]).owner == "uav_1"
    # 续租 → 未到期
    lm.renew("uav_1", assign["uav_1"], now=20.0)
    assert lm.expire(25.0) == [], "续租后不应到期"
    # 不续租 → 30s 到期（uav_1 续到 50s 后断联，55s 时到期）
    expired = lm.expire(55.0)
    assert assign["uav_1"] in expired, expired
    print("任务租约 OK: 续租后不到期，断联 %ds 后到期重分配" % LEASE_DURATION)

    # ---- 3) 覆盖标记：观测半径覆盖 ----
    # 观测点 (0,0) 半径 8m：格中心 (±5,±5) 距原点 7.07m，应覆盖 4 个相邻格
    covered = g.is_covered(0.0, 0.0, radius_m=8.0)
    assert len(covered) == 4, covered
    assert all(g.cell(k).state == STATE_COVERED for k in covered)
    print("覆盖标记 OK: 观测半径 8m 覆盖 %d 格" % len(covered))

    # ---- 4) 追踪接力：选最快到达机 + 继承确认计时 ----
    tt = TargetTracker()
    tt.add_target("t0", x=0.0, y=0.0, tracker="uav_1", now=100.0)
    # uav_1 距目标远（丢失），uav_2 更近
    relay = tt.pick_relay("t0", {"uav_2": (2.0, 0.0), "uav_3": (30.0, 0.0)})
    assert relay == "uav_2", relay
    assert tt.handover("t0", "uav_2", now=105.0)
    assert tt.targets["t0"]["tracker"] == "uav_2"
    assert tt.targets["t0"]["confirm_until"] == 105.0 + CONFIRM_TIME
    print("追踪接力 OK: 最快到达机 uav_2 继承目标 t0，确认计时重置 %.0fs" % CONFIRM_TIME)

    # ---- 5) 规则5：连续 15s 确认 → 消除（中断必须清零） ----
    tt2 = TargetTracker()
    tt2.add_target("t1", x=10.0, y=10.0, now=0.0)
    # 连续观测 15s（每 1s 一次）→ 最后一刻达成消除
    met = False
    for k in range(1, int(CONFIRM_TIME) + 1):
        met = tt2.observe("t1", now=float(k), observer="uav_1")
    assert met and tt2.targets["t1"]["eliminated"], "连续 15s 应判定消除"
    print("规则5 OK: 连续观测 %ds → 目标消除" % CONFIRM_TIME)

    # 中断清零：观测到 10s 后丢失，再从头计时
    tt3 = TargetTracker()
    tt3.add_target("t2", x=0.0, y=0.0, now=0.0)
    for k in range(1, 11):
        tt3.observe("t2", now=float(k), observer="uav_1")
    tt3.lose("t2", now=11.0)
    assert tt3.targets["t2"]["confirm_since"] is None, "丢失后计时必须清零"
    for k in range(12, 22):   # 只再观测 10s，不应消除
        met = tt3.observe("t2", now=float(k), observer="uav_1")
    assert not tt3.targets["t2"]["eliminated"], "中断后重新计时，10s 不应消除"
    print("规则5 OK: 中途丢失 → 计时清零，不误判消除")

    # ---- 6) 规则4：被感知 30s 未消除 → 瞬移躲藏 ----
    tt4 = TargetTracker()
    tt4.add_target("t3", x=0.0, y=0.0, now=0.0)
    assert not tt4.check_evade("t3", now=20.0), "20s 未到瞬移阈值"
    assert tt4.check_evade("t3", now=EVADE_TIME), "30s 应触发瞬移"
    assert tt4.targets["t3"]["evaded"]
    # 瞬移后必须重置所有计时
    assert tt4.relocate("t3", x=50.0, y=-30.0, now=EVADE_TIME)
    t3 = tt4.targets["t3"]
    assert (t3["x"], t3["y"]) == (50.0, -30.0) and t3["tracked_since"] == EVADE_TIME
    assert not t3["evaded"] and t3["confirm_since"] is None
    print("规则4 OK: 被感知 %.0fs 未消除 → 瞬移到新位置并重置计时" % EVADE_TIME)

    # ---- 7) 视线遮挡：中间有建筑 → 不可见 ----
    # 造一个 0.5m 栅格的小地图（原点 0,0），在 x=5 处放一堵竖墙
    def _blocked(ix, iy):
        return ix == 10   # x = 5.0m 处的整列障碍
    los = LineOfSight(_blocked, cell_size=0.5, origin=(0.0, 0.0))
    assert not los.visible(0.0, 0.0, 10.0, 0.0), "穿墙应判为不可见"
    assert los.visible(0.0, 0.0, 4.0, 0.0), "墙前应可见"
    assert los.visible(6.0, 0.0, 10.0, 0.0), "墙后同侧应可见"
    print("视线遮挡 OK: 穿墙不可见，无遮挡可见")

    # ---- 8) 视线遮挡：原点非 (0,0) 时坐标转换必须正确 ----
    # 复现真实地图情形（origin=(-100,-50)）：x=-100 处的墙
    def _blocked_off(ix, iy):
        return ix == 20   # origin -100 + 20*0.5 = -90.0m 处的墙
    los2 = LineOfSight(_blocked_off, cell_size=0.5, origin=(-100.0, -50.0))
    assert not los2.visible(-95.0, 0.0, -85.0, 0.0), "穿墙应不可见（含 origin 偏移）"
    assert los2.visible(-95.0, 0.0, -91.0, 0.0), "墙前应可见（含 origin 偏移）"
    print("LOS 原点偏移 OK: 非 (0,0) 原点下坐标转换正确")

    print("\nswarm_task 自测全部通过")
