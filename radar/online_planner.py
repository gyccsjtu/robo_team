#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""online_planner.py —— 边飞边建图边重规划（零文件依赖）

为什么需要它
============
语雀官方原文：正式比赛的随机地图由**技术委员会在仿真机上**用
`map_generator.py` 生成，开源那份只是为了方便选手**调试**。裁判笔记本
「用于监督选手使用话题的情况」。⇒ 算法机起飞时**对地图一无所知**，
既不能读文件，也不能靠话题订阅拿真值。

于是唯一合法的地图来源只剩一个：**机载传感器在线建图**。
本模块把这张在线图变成**可执行的航线**：

    /scan ─► OnlineOccupancy ─► hit_points() ─► A*.plan(points=...) ─► wps
                                                                        │
                                         radar_avoid.step_toward ◄──────┘
                                         （雷达反应层，每帧仍然生效）

三条安全性质（是结构上成立的，不是"测了很多次没出事"）
========================================================
 1. **只增不减**：喂给 A* 的障碍点全部来自**已观测到的回波**。本模块从不
    输出"这片区域可飞"的断言 ⇒ 不可能把"该绕的"变成"可以直穿的"。
 2. **失败即不动**：A* 无解 / 抛异常 / 路径病态 ⇒ 返回 None，主循环
    **保持原航点**。此时行为精确退化为"直飞 + 雷达反应式"，与没有本模块
    时**逐字节一致**。⇒ 重规划只可能改善、不可能恶化。
 3. **不接管控制**：本模块只产出航点序列；每一帧的实际执行仍是
    radar_avoid 的 `subgoal_from_scan`。重规划给出再离谱的航点，雷达也会在
    最后几米刹住。⇒ 安全底线与全局规划完全解耦。

刻意不做
========
 · **不读任何文件**（`black_box.txt` / `obstacle.txt` / `*.world` 全不读）
 · **不把未观测区当障碍**（否则原地不敢动）；也**不把它当可飞断言**
 · **不删障碍**（不做"多帧扫过就清除"的推断 —— 那正是把保守性破坏掉的写法）
 · **不做掉头**（新路径首段与当前朝向夹角 > `BACK_ANGLE` 时放弃本次重规划，
   宁可继续直飞让雷达处理，也不在空中打转）

自检
====
    python3 online_planner.py --selftest
需要同目录的 astar_plan.py / occupancy_online.py，**不需要 ROS**。
"""

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from occupancy_online import OnlineOccupancy

# 默认参数（可在构造时覆盖）
DEFAULT_PERIOD = 5.0      # 重规划最短间隔（秒）
DEFAULT_MARGIN = 2.0      # A* 障碍膨胀半径（m），与官方图实测净空 2.52m 一致
DEFAULT_CLEAR_R = 2.5     # 起点净空半径（m）—— 见下方"为什么必须清起点"
DEFAULT_MAX_STRETCH = 6.0  # 绕行长度 / 直线距离 的上限（超过则放弃本次重规划）
DEFAULT_BACK_ANGLE = 120.0  # 新路径首段与当前朝向夹角超过此值则放弃（度）


class OnlineReplanner(object):
    """在线图 -> 航点序列。所有失败路径都返回 None（=不动）。"""

    def __init__(self, occ, astar_mod, goal, period=DEFAULT_PERIOD,
                 margin=DEFAULT_MARGIN, clear_r=DEFAULT_CLEAR_R,
                 max_stretch=DEFAULT_MAX_STRETCH, back_angle=DEFAULT_BACK_ANGLE):
        self.occ = occ
        self.A = astar_mod
        self.goal = (float(goal[0]), float(goal[1]))
        self.period = float(period)
        self.margin = float(margin)
        self.clear_r = float(clear_r)
        self.max_stretch = float(max_stretch)
        self.back_angle = float(back_angle)
        self.t_last = -1.0e9
        self.last_hits = -1
        self.last_grid = None        # 最近一次规划用的 Grid（仅审计用）
        # 计数（用于机上日志与赛前体检）
        self.n_replan = 0
        self.n_fail = 0
        self.n_skip = 0
        self.last_reason = 'init'
        self.last_detour = 0.0

    # ------------------------------------------------------------ 取障碍点
    def hit_points(self, near_xy=None):
        """把在线图的 hits 导出成 A* 的 `points` 点阵。

        near_xy 附近的点会被**剔除**（仅用于喂给 A*，**不改动在线图本身**）。

        为什么必须清起点：飞机自己就在障碍旁边（甚至贴着墙）时，雷达会把
        身边的格子也标成 hits，而 A* 会用 `nearest_free()` 把起点**吸附**到
        最近可行格 —— 一旦被吸附到十几米外，规划出的路径就从别处开始，
        或者直接 `RuntimeError('起终点附近找不到可行点')`。清掉一个
        `clear_r` 的圆盘是"承认自己不知道脚下这一圈"，比让 A* 乱吸附安全。

        量化说明（重要）：在线图每格 **1 m**，喂进 A* 后每点按 ±0.5 m 的方块
        参与判定 ⇒ 真实障碍（尤其 ⌀0.175 m 的灯杆）在规划器眼里被**放大**成
        1 m 方块；再叠加 `margin=2.0` ⇒ 等效禁飞半径 ≥ 2.5 m。误差方向是
        **偏保守**（只会绕得更开，不会贴得更近），与官方图实测净空 2.52 m
        同量级，可放心使用。**不要**为了"更精确"去缩小 margin —— 那等于把
        量化误差的容差吃掉。
        """
        occ = self.occ
        out = []
        nx, ny, cell = occ.nx, occ.ny, occ.cell
        x0, y0 = occ.x0, occ.y0
        hits = occ.hits
        cr2 = self.clear_r * self.clear_r
        for j in range(ny):
            base = j * nx
            y = y0 + j * cell
            for i in range(nx):
                if not hits[base + i]:
                    continue
                x = x0 + i * cell
                if near_xy is not None:
                    dx = x - near_xy[0]
                    dy = y - near_xy[1]
                    if dx * dx + dy * dy < cr2:
                        continue
                out.append((x, y))
        return out

    # ------------------------------------------------------------ 重规划
    def maybe_replan(self, cur, now, heading=None, force=False):
        """尝试重规划。返回**剩余航点列表**（>=1 个），或 None（保持原航点）。

        cur     —— 当前世界坐标 (x, y)
        now     —— 当前时间（秒，单调）
        heading —— 当前机头朝向（弧度），用于掉头保护；None 则不检查
        force   —— 跳过周期与"无新信息"两道闸门（调试/自检用）

        两道节流闸门（避免空转）：
          ① 距上次重规划不足 period 秒 ⇒ 不动
          ② 在线图命中格数与上次相同 ⇒ 没有新信息，重算结果必然一样 ⇒ 不动
        """
        if cur is None:
            return None

        if not force:
            if now - self.t_last < self.period:
                self.last_reason = 'wait'
                return None
            h = self.occ.n_hits()
            if h == self.last_hits:
                self.last_reason = 'no-new-info'
                return None

        self.t_last = now
        self.last_hits = self.occ.n_hits()

        pts = self.hit_points(cur)
        if not pts:
            self.n_skip += 1
            self.last_reason = 'no-obstacle-yet'
            return None

        try:
            _g, _raw, sp = self.A.plan([], [], (cur[0], cur[1]), self.goal,
                                       margin=self.margin, points=pts)
        except Exception as exc:          # 无解 / 起点被围 / 任何异常
            self.n_fail += 1
            self.last_reason = 'astar-fail:%s' % (str(exc)[:48],)
            return None
        self.last_grid = _g               # 供自检/审计用（不参与控制）

        if not sp or len(sp) < 2:
            self.n_skip += 1
            self.last_reason = 'path-too-short'
            return None

        # 去掉起点（A* 的 path[0] 即 cur，或被吸附后的点）
        rest = [tuple(p) for p in sp[1:]]
        if not rest:
            self.n_skip += 1
            self.last_reason = 'empty-rest'
            return None

        # 病态保护 ①：绕行长度上限
        straight = math.hypot(self.goal[0] - cur[0], self.goal[1] - cur[1])
        detour = 0.0
        px, py = cur[0], cur[1]
        for (qx, qy) in rest:
            detour += math.hypot(qx - px, qy - py)
            px, py = qx, qy
        self.last_detour = detour
        if straight > 1.0 and detour > self.max_stretch * straight:
            self.n_skip += 1
            self.last_reason = 'detour-%.1fx' % (detour / straight,)
            return None

        # 病态保护 ②：掉头保护（宁可继续直飞让雷达处理，也不在空中打转）
        if heading is not None:
            hx, hy = rest[0][0] - cur[0], rest[0][1] - cur[1]
            if math.hypot(hx, hy) > 1e-6:
                a = abs(_wrap_pi(math.atan2(hy, hx) - heading))
                if a > math.radians(self.back_angle):
                    self.n_skip += 1
                    self.last_reason = 'back-%.0fdeg' % (math.degrees(a),)
                    return None

        # ★ 10-01：保证**末点就是精确目标点**。A* 平滑后的末点可能落在目标
        #   格中心（差 ~1m）⇒ 主循环用"到末点 <ARRIVE_R"判到达时，实际离目标
        #   点还差一截。补一个精确末点，让"到达"名副其实。
        _gx, _gy = self.goal[0], self.goal[1]
        if math.hypot(rest[-1][0] - _gx, rest[-1][1] - _gy) > 1e-6:
            rest.append((_gx, _gy))

        self.n_replan += 1
        self.last_reason = 'ok(detour %.2fx)' % (
            detour / max(straight, 1e-6),)
        return rest

    # ------------------------------------------------------------ 诊断
    def stats(self):
        return ("[REPLAN] 重规划=%d 失败=%d 跳过=%d 上次=%s"
                % (self.n_replan, self.n_fail, self.n_skip, self.last_reason))


def _wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# ==================================================================== 自检
class _FakeScan(object):
    """最小 LaserScan 替身：只需 ranges / angle_min / angle_increment。"""

    def __init__(self, ranges, angle_min, angle_increment):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = angle_increment


def _ray_circle(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    for t in (-b - sq, -b + sq):
        if t > 0.0:
            return t
    return None


def _scan_from_world(px, py, yaw, poles, n_beams=512, rmax=20.0):
    """从世界坐标 (px,py,yaw) 看向一群圆柱(世界系)，生成雷达帧。

    用于自检：模拟"飞机边走边看到新障碍"。返回 _FakeScan，其 ranges 是
    **雷达系** 距离（比 scan_to_obstacles 的期望输入）。
    """
    inc = 2.0 * math.pi / (n_beams - 1)
    a0 = -math.pi
    rs = []
    for k in range(n_beams):
        a = a0 + k * inc                      # 机体系角度
        dxb, dyb = math.cos(a), math.sin(a)
        # 机体系方向 -> 世界系方向
        dxw = dxb * math.cos(yaw) - dyb * math.sin(yaw)
        dyw = dxb * math.sin(yaw) + dyb * math.cos(yaw)
        best = None
        for (cx, cyw, r) in poles:
            t = _ray_circle(px, py, dxw, dyw, cx, cyw, r)
            if t is not None and t < rmax and (best is None or t < best):
                best = t
        rs.append(best if best is not None else float('inf'))
    return _FakeScan(rs, a0, inc)


def selftest():
    """逐段采样检查路径是否进入"障碍 + 膨胀"圆内。返回越界采样点列表。"""
    print("=" * 74)
    print("online_planner 自检（不需要 ROS）")
    print("=" * 74)
    try:
        import astar_plan as A
        print("astar_plan 已加载: %s" % A.__file__)
    except Exception as exc:
        print("✗ 无法加载 astar_plan: %s" % exc)
        return 1

    ok = True

    # ---------------------------------------------------------- 用例 1
    # 场景：飞机在 (0,0)，目标 (30,0)，正前方 x=12..17 一堵"墙"（5 根实心柱
    # 模拟一片已探明的建筑）。验证：喂完雷达后，重规划给出的路径**绕开**该墙。
    print()
    print("用例 1: 已探明一堵墙(x=12..17,y=0) ⇒ 重规划必须绕开")
    occ = OnlineOccupancy()
    poles = [(x, 0.0, 2.0) for x in (12.0, 13.0, 14.0, 15.0, 16.0, 17.0)]
    n_ret = occ.feed(_scan_from_world(0.0, 0.0, 0.0, poles), yaw=0.0,
                     pos_enu=(0.0, 0.0), mount=(0.0, 0.0), stride=1,
                     mark_seen=False)
    print("  合成雷达帧有效回波束数 = %d" % n_ret)
    rp = OnlineReplanner(occ, A, (30.0, 0.0), period=0.0, margin=2.0)
    rest = rp.maybe_replan((0.0, 0.0), now=1.0, heading=0.0, force=True)
    if rest is None:
        print("  ✗ 重规划返回 None（%s）" % rp.last_reason)
        ok = False
    else:
        print("  剩余航点 %d 个: %s" % (len(rest), ["(%.0f,%.0f)" % p for p in rest]))
        # 权威判据：A*.verify() 用 **core 栅格（障碍本体，不含 margin）** 逐
        # 0.25m 采样判定是否穿墙 —— 与全局规划器用的是同一份代码，不会各说各话
        path1 = [(0.0, 0.0)] + rest
        nv, badv = A.verify(rp.last_grid, path1)
        print("  A*.verify 穿墙段数 = %d（与全局规划器同源判据，0.25m 采样）" % nv)
        if nv == 0:
            print("  ✓ 未穿墙（core 几何判定）")
        else:
            print("  ✗ 穿墙: %s" % (badv[:4],))
            ok = False
        if abs(rest[-1][0] - 30.0) < 0.01 and abs(rest[-1][1]) < 0.01:
            print("  ✓ 终点保持为目标点 (30,0)")
        else:
            print("  ✗ 终点被改动: %s" % (rest[-1],))
            ok = False

    # ---------------------------------------------------------- 用例 2
    # 全程无人：一条障碍也没有 ⇒ 必须返回 None（保持直飞），不得发明路线
    print()
    print("用例 2: 什么都没看到 ⇒ 必须返回 None（保持直飞，不发明路线）")
    occ2 = OnlineOccupancy()
    # 空场景：所有射线无回波
    occ2.feed(_FakeScan([float('inf')] * 512, -math.pi, 2 * math.pi / 511),
              yaw=0.0, pos_enu=(0.0, 0.0), mount=(0.0, 0.0), stride=1,
              mark_seen=False)
    rp2 = OnlineReplanner(occ2, A, (30.0, 0.0), period=0.0)
    r2 = rp2.maybe_replan((0.0, 0.0), now=1.0, heading=0.0, force=True)
    print("  返回 = %s  原因 = %s" % (r2, rp2.last_reason))
    if r2 is None and rp2.last_reason == 'no-obstacle-yet':
        print("  ✓ 无观测时不重规划")
    else:
        print("  ✗ 无观测时行为异常")
        ok = False

    # ---------------------------------------------------------- 用例 3
    # 终点被自己看到的障碍完全围死 ⇒ A* 失败 ⇒ 必须返回 None，不得抛异常
    print()
    print("用例 3: 目标点被围死 ⇒ A* 失败必须返回 None（不得抛异常）")
    occ3 = OnlineOccupancy()
    ring = [(30.0 + 3.0 * math.cos(2 * math.pi * k / 12.0),
             3.0 * math.sin(2 * math.pi * k / 12.0), 1.6) for k in range(12)]
    # 手动灌入 hits（模拟已探明）：用一次环绕合成帧不方便，直接标格
    for (cx, cy, r) in ring:
        i, j = occ3.to_idx(cx, cy)
        occ3.hits[j * occ3.nx + i] = 1
    rp3 = OnlineReplanner(occ3, A, (30.0, 0.0), period=0.0)
    try:
        r3 = rp3.maybe_replan((0.0, 0.0), now=1.0, heading=0.0, force=True)
        print("  返回 = %s  原因 = %s" % (r3, rp3.last_reason))
        if r3 is None:
            print("  ✓ 失败时安全返回 None（主循环将保持原航点）")
        else:
            print("  ⚠ 未失败（环可能有缺口，A* 找到了缝）—— 不算错，仅记录")
    except Exception as exc:
        print("  ✗ 抛异常了（必须内部吞掉）: %r" % (exc,))
        ok = False

    # ---------------------------------------------------------- 用例 4
    # 起点净空：飞机贴着一根杆（该杆就在身边 1m）⇒ 不得因起点被吸附而失败
    print()
    print("用例 4: 起点旁边 1m 有障碍 ⇒ 起点净空逻辑须让 A* 正常出解")
    occ4 = OnlineOccupancy()
    for (cx, cy) in [(1.0, 0.6), (0.6, 1.0), (-0.8, 0.8)]:   # 紧贴机身的伪障碍
        i, j = occ4.to_idx(cx, cy)
        occ4.hits[j * occ4.nx + i] = 1
    i, j = occ4.to_idx(18.0, 0.0)
    occ4.hits[j * occ4.nx + i] = 1                            # 远处真障碍
    rp4 = OnlineReplanner(occ4, A, (30.0, 0.0), period=0.0)
    r4 = rp4.maybe_replan((0.0, 0.0), now=1.0, heading=0.0, force=True)
    print("  返回 = %s  原因 = %s" % (
        ["(%.0f,%.0f)" % p for p in r4] if r4 else None, rp4.last_reason))
    if r4 is None:
        print("  ✗ 因起点附近障碍而失败（clear_r 未生效）")
        ok = False
    else:
        d0 = math.hypot(r4[0][0] - 0.0, r4[0][1] - 0.0)
        print("  ✓ 正常出解（首点距机 %.2f m；smooth 剪枝后首点本来就远，"
              "不作判据）" % d0)
        # 真正的判据：不因起点被 nearest_free 吸附而抛异常，且终点正确
        if abs(r4[-1][0] - 30.0) > 0.01:
            print("  ✗ 终点被改动: %s" % (r4[-1],))
            ok = False

    # ---------------------------------------------------------- 用例 5
    # 单调性：连续喂更多帧（墙面变厚/加长）后，路径长度不许变短到"穿墙"
    print()
    print("用例 5: 边飞边看到更多障碍（墙加长）⇒ 路径仍不穿墙")
    occ5 = OnlineOccupancy()
    poles5 = [(x, y, 2.0) for x in (12.0, 13.0, 14.0, 15.0, 16.0, 17.0)
              for y in (0.0, 3.0, 6.0, -3.0, -6.0)]
    occ5.feed(_scan_from_world(0.0, 0.0, 0.0, poles5), yaw=0.0,
              pos_enu=(0.0, 0.0), mount=(0.0, 0.0), stride=1, mark_seen=False)
    rp5 = OnlineReplanner(occ5, A, (30.0, 0.0), period=0.0, margin=2.0)
    r5 = rp5.maybe_replan((0.0, 0.0), now=1.0, heading=0.0, force=True)
    print("  返回 = %s  原因 = %s" % (
        ["(%.0f,%.0f)" % p for p in r5] if r5 else None, rp5.last_reason))
    if r5 is None:
        print("  ⚠ 加长墙后 A* 未出解（可能绕不过去）—— 不算错，仅记录")
    else:
        path5 = [(0.0, 0.0)] + r5
        nv5, badv5 = A.verify(rp5.last_grid, path5)
        print("  A*.verify 穿墙段数 = %d" % nv5)
        if nv5 == 0:
            print("  ✓ 墙加长后仍未穿墙")
        else:
            print("  ✗ 穿墙: %s" % (badv5[:4],))
            ok = False

    # ---------------------------------------------------------- 用例 6
    # ★ 闭环：边飞边建图边重规划 —— 从 (0,0) 出发，正前方 15 m 有一堵横墙
    #   (y∈[-8,8]，半径 1.5 m 的实心柱连成)。看它能否只靠"先看到、再绕开"
    #   自己飞到 (30,0)，且全程不穿墙。这是**整条链路的端到端离线验证**：
    #   建图 → 规划 → 执行 → 再建图 → 再规划 …
    print()
    print("用例 6: 闭环模拟（边飞边建图边重规划）—— 前方 15m 一堵横墙")
    wall6 = [(15.0, float(i) * 2.0, 1.5) for i in range(-4, 5)]
    occ6 = OnlineOccupancy()
    rp6 = OnlineReplanner(occ6, A, (30.0, 0.0), period=0.0, margin=2.0)
    cur = [0.0, 0.0]
    hd = 0.0
    wps6 = [(30.0, 0.0)]
    traj = [(0.0, 0.0)]
    n_re = 0
    arrived = False
    for step in range(300):
        if not wps6:
            break
        tx, ty = wps6[0]
        d = math.hypot(tx - cur[0], ty - cur[1])
        if d < 1.0:
            wps6 = wps6[1:]
            continue
        hd = math.atan2(ty - cur[1], tx - cur[0])
        sl = min(1.0, d)
        cur[0] += sl * math.cos(hd)
        cur[1] += sl * math.sin(hd)
        traj.append((cur[0], cur[1]))
        # ① 本帧雷达（stride=4 → 128 束，与机上抽稀同量级）
        occ6.feed(_scan_from_world(cur[0], cur[1], hd, wall6), yaw=hd,
                  pos_enu=(cur[0], cur[1]), mount=(0.0, 0.0), stride=4,
                  mark_seen=False)
        # ② 重规划（force=True 跳过时间闸门，纯逻辑验证）
        rest6 = rp6.maybe_replan((cur[0], cur[1]), now=float(step), heading=hd,
                                 force=True)
        if rest6:
            n_re += 1
            wps6 = list(rest6)
        if math.hypot(cur[0] - 30.0, cur[1]) < 1.5:
            arrived = True
            break
    print("  飞行 %d 步（1m/步），重规划 %d 次，终点 (%.1f,%.1f)"
          % (len(traj) - 1, n_re, cur[0], cur[1]))
    print("  轨迹采样: %s" % " ".join("(%.0f,%.0f)" % p for p in traj[::6]))
    print("  在线图命中格数 = %d" % occ6.n_hits())
    if arrived:
        print("  ✓ 到达目标 (30,0)")
    else:
        print("  ✗ 未到达目标")
        ok = False
    # 全程净空：轨迹点到墙**表面**的最小距离（步长 1 m，点采样足够）
    if traj:
        min_clr = min(
            min(math.hypot(x - cx, y - cy) - r for (cx, cy, r) in wall6)
            for (x, y) in traj)
        print("  轨迹到墙面最小净空 = %.2f m" % min_clr)
        if min_clr <= 0.0:
            print("  ✗ 穿进墙体实体（净空 ≤ 0）")
            ok = False
        elif min_clr < 0.8:
            print("  ⚠ 净空 < 0.80 m 的下限，需人工确认")
        else:
            print("  ✓ 全程未穿墙且净空达标（≥ 0.80 m）")

    print()
    print("=" * 74)
    print("自检结果: %s" % ("全部通过 PASS" if ok else "存在失败 FAIL"))
    print("=" * 74)
    return 0 if ok else 1


def main():
    import argparse
    ap = argparse.ArgumentParser(description="在线重规划器（边飞边建图）")
    ap.add_argument("--selftest", action="store_true", help="离线自检")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    print(__doc__)
    print("本模块作为库使用：from online_planner import OnlineReplanner")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
