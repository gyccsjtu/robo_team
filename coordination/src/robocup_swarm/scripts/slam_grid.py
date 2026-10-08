#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""雷达 SLAM 占用栅格（合规地图：运行时由 2D 激光实时构建）。

背景（2026-10-07 合规重构）：
  旧链路把 black_box.txt（官方随机化输出的建筑矩形真值）预读成 metadata
  障碍图 —— 在规则 §2.4/§2.5「无预读随机地图」下违规。
  新链路：metadata 只保留规则定值（bounds/frame/spawn/goal_candidates），
  障碍恒为空；本模块在 swarm_agent 运行时用 /scan 射线投射实时建图：
      扫到（有效回波）→ mark occupied（含增量圆盘膨胀 + 墙体深度填充）

设计要点：
  1. 只标占用、不标自由（保守 SLAM）：位姿误差可能把墙"擦掉"，导致 A*
     穿墙；多标只会绕远，不会撞。未知区域视为可通行 —— A* 先直飞，
     雷达近障守卫（RADAR_GUARD）兜底，发现新障碍即触发重规划。
  2. 增量膨胀：新占用 cell 只盖一次半径 r 圆盘到 inflated 栅格，
     避免每次规划前全图重膨胀（380x260 全图膨胀 ~60 万次写）。
  3. 接口与 robocup_navigation.astar.GridMap 对齐（width/height/
     resolution/origin/cells/in_bounds/is_free/world_to_cell/
     cell_to_world），可直接当 self.grid 传给 plan()/grid_guard。
  4. 线程模型：mark_scan() 在 ROS 回调线程（_scan_cb，节流）调用；
     规划线程/控制循环并发读。bytearray 单字节读写 GIL 原子，读侧
     最坏看到半张盘的膨胀 —— 只影响一帧，无害，故不加锁。

只依赖标准库（纯逻辑，可离线单测，与 swarm_task.LineOfSight 同风格）。
"""

import math


class SlamGrid(object):
    """增量式 2D 激光占用栅格（is_free = 膨胀后视图，is_free_raw = 原始视图）。"""

    def __init__(self, x_min, y_min, x_max, y_max, resolution, inflation_m,
                 frame_id="map"):
        self.resolution = float(resolution)
        if self.resolution <= 1e-6:
            raise ValueError("SlamGrid resolution 必须为正: %r" % resolution)
        self.origin = (float(x_min), float(y_min))
        self.frame_id = frame_id
        self.width = int(round((float(x_max) - float(x_min)) / self.resolution))
        self.height = int(round((float(y_max) - float(y_min)) / self.resolution))
        if self.width <= 0 or self.height <= 0 or self.width * self.height > 4000000:
            raise ValueError("SlamGrid 非法栅格 %dx%d" % (self.width, self.height))
        n = self.width * self.height
        self.cells = bytearray(n)        # 原始占用（0=未知/自由，1=占用）
        self.infl_cells = bytearray(n)   # 膨胀后占用（A* / grid_guard 用）
        self._inflate_r = max(0, int(math.ceil(float(inflation_m) / self.resolution)))
        self._disk = self._make_disk(self._inflate_r)
        # 遥测
        self.total_cells = 0             # 累计占用 cell 数（含深度填充）
        self.new_cells = 0               # 自上次 take_new_cells() 以来的新增数
        self.mark_frames = 0             # 已处理的雷达帧数

    # ------------------------------------------------------------------
    # GridMap 兼容接口
    # ------------------------------------------------------------------
    def in_bounds(self, cell):
        if cell is None:
            return False
        return (0 <= cell[0] < self.width and 0 <= cell[1] < self.height)

    def is_free(self, cell):
        """膨胀后视图（A*/grid_guard 语义，None/越界=不可自由）。"""
        if not self.in_bounds(cell):
            return False
        return self.infl_cells[cell[1] * self.width + cell[0]] == 0

    def is_free_raw(self, cell):
        """原始视图（LOS 遮挡判定语义，None/越界=遮挡，与旧 raw_grid lambda 一致）。"""
        if not self.in_bounds(cell):
            return False
        return self.cells[cell[1] * self.width + cell[0]] == 0

    def world_to_cell(self, point):
        x, y = float(point[0]), float(point[1])
        x_max = self.origin[0] + self.width * self.resolution
        y_max = self.origin[1] + self.height * self.resolution
        if not (self.origin[0] <= x < x_max and self.origin[1] <= y < y_max):
            return None
        return (int(math.floor((x - self.origin[0]) / self.resolution)),
                int(math.floor((y - self.origin[1]) / self.resolution)))

    def cell_to_world(self, cell):
        return (self.origin[0] + (cell[0] + 0.5) * self.resolution,
                self.origin[1] + (cell[1] + 0.5) * self.resolution)

    # ------------------------------------------------------------------
    # 建图
    # ------------------------------------------------------------------
    @staticmethod
    def _make_disk(r):
        """预计算半径 r 的圆盘偏移（含圆心），供增量膨胀盖章。"""
        if r <= 0:
            return ((0, 0),)
        disk = []
        r2 = r * r
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r2:
                    disk.append((dx, dy))
        return tuple(disk)

    def mark_point(self, wx, wy):
        """标记世界坐标点占用（含膨胀），返回是否为新占用。"""
        cell = self.world_to_cell((wx, wy))
        if cell is None:
            return False
        idx = cell[1] * self.width + cell[0]
        if self.cells[idx]:
            return False
        self.cells[idx] = 1
        cx, cy = cell
        w, h = self.width, self.height
        for dx, dy in self._disk:
            x, y = cx + dx, cy + dy
            if 0 <= x < w and 0 <= y < h:
                self.infl_cells[y * w + x] = 1
        self.total_cells += 1
        self.new_cells += 1
        return True

    def mark_scan(self, pose, yaw, scan, max_range, depth_cells=0,
                  min_range=0.5):
        """把一帧 LaserScan 射线投射进栅格。

        pose    : (x, y) 雷达世界坐标（机体原点，2D 雷达共面）
        yaw     : 机体 ENU 偏航（弧度），beam 世界角 = yaw + angle_min + i*inc
        scan    : sensor_msgs/LaserScan
        max_range : 建图最大距离 m（更远的回波噪声大，丢弃）
        depth_cells : 命中点沿射线方向再填充的 cell 数（墙体/室内深度，
                      防止只标到迎弹面、A* 从背面穿墙）
        min_range : 自回波地板 m（v11 复盘 2026-10-07）。雷达平面仍会扫到
                    机身残余部件（云台/盒顶边缘，实测 0.31~0.34m 恒定回波，
                    SDF 抬高后仍存在）。若不滤除，每帧在机体周围标出
                    2.3m 占用盘（0.32m 命中 + 2m 深度填充），膨胀 2.5m 后
                    ≈4.8m 死区 → 自身格恒占用 → A* START_OCCUPIED +
                    栅格守卫刹停 = 六机全程 0 位移。机体半对角 0.47m，
                    真实障碍物理上不可能 <0.5m 而无碰撞，地板 0.5m 安全。

        返回本帧新标记的占用 cell 数（含深度填充）。
        """
        if scan is None or not scan.ranges:
            return 0
        px, py = float(pose[0]), float(pose[1])
        a0 = float(scan.angle_min)
        inc = float(scan.angle_increment)
        rmin = float(scan.range_min) if scan.range_min > 0 else 0.1
        rmax = min(float(scan.range_max) if scan.range_max > 0 else max_range,
                   float(max_range))
        res = self.resolution
        marked = 0
        for i, r in enumerate(scan.ranges):
            if r is None:
                continue
            try:
                r = float(r)
            except (TypeError, ValueError):
                continue
            if math.isnan(r) or math.isinf(r) or r < rmin or r > rmax:
                continue
            if r < min_range:
                # 自回波地板：机身残余部件回波不建图（见 docstring）
                continue
            th = yaw + a0 + inc * i
            c, s = math.cos(th), math.sin(th)
            # 命中点 + 沿射线深度填充（墙体厚度/室内）
            for k in range(int(depth_cells) + 1):
                d = r + k * res
                if self.mark_point(px + c * d, py + s * d):
                    marked += 1
        if marked:
            self.mark_frames += 1
        return marked

    # ------------------------------------------------------------------
    # 遥测 / 序列化
    # ------------------------------------------------------------------
    def take_new_cells(self):
        """取走并清零新增计数（重规划触发 / 发布节流用）。"""
        n = self.new_cells
        self.new_cells = 0
        return n

    def occupied_count(self):
        return sum(1 for v in self.cells if v)

    def to_rle(self):
        """原始占用 → RLE [[value, count], ...]，供 /swarm/occupancy_grid 发布。"""
        runs = []
        last = None
        cnt = 0
        for v in self.cells:
            if v == last:
                cnt += 1
            else:
                if last is not None:
                    runs.append([int(last), cnt])
                last = v
                cnt = 1
        if last is not None:
            runs.append([int(last), cnt])
        return runs

    @classmethod
    def from_rle(cls, width, height, resolution, origin, rle, frame_id="map"):
        """从 RLE 重建（manager 合并多机栅格时不用此类，仅供测试/工具）。"""
        g = cls(origin[0], origin[1],
                origin[0] + width * resolution, origin[1] + height * resolution,
                resolution, 0.0, frame_id=frame_id)
        idx = 0
        for v, c in rle:
            if v:
                g.cells[idx:idx + c] = b"\x01" * c
            idx += c
        if idx != width * height:
            raise ValueError("RLE 长度 %d != %d" % (idx, width * height))
        return g
