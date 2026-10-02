#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""闭环仿真：让虚拟飞机带着雷达避障穿过障碍区，统计碰撞次数。

为什么必须做这个
----------------
离线单测只验证"给定一组障碍点，shift 算得对不对"。
但真实系统是**闭环**的：飞机动了 => 雷达读数变了 => shift 变了。
单点正确不代表闭环稳定（可能震荡、可能卡死、可能越让越撞）。

本脚本用纯几何模拟一个"完美雷达"（射线与矩形求交），
把 radar_avoid 的控制律跑完整航程，统计：
  - 与障碍的最近距离（必须 > CLEARANCE）
  - 碰撞次数（最近距离 < BODY_RADIUS 记为撞）
  - 到达目标点的比例
  - 是否存在原地震荡/停滞

不需要 ROS、不需要 Gazebo。
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# astar_plan 的可选路径（本脚本自身不用它，但 test_two_layer 会通过
# 本模块的 sys.path 间接受益；这里只是保持与 test_two_layer 一致的
# 可移植性，避免 VM 上路径不同导致 import 失败）。
for _c in (os.environ.get('RADAR_DEMO_DIR', ''),
           os.path.join(os.path.dirname(HERE), '_demo2026'),
           os.path.expanduser('~/robocup_real/_demo2026'),
           os.path.expanduser('~/_demo2026')):
    if _c and os.path.isfile(os.path.join(_c, 'astar_plan.py')):
        sys.path.insert(0, _c)
        break

import test_radar_offline as T

T.install_stubs()
import radar_avoid as R


# ============================ 虚拟世界 ============================
class Box(object):
    """轴对齐矩形障碍（对标 base.world 里的建筑）。"""

    def __init__(self, x0, y0, x1, y1, name='box'):
        self.x0, self.y0 = min(x0, x1), min(y0, y1)
        self.x1, self.y1 = max(x0, x1), max(y0, y1)
        self.name = name

    def contains(self, x, y):
        return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1

    def closest_dist(self, x, y):
        """点到矩形的最近距离（在内部时为负）。"""
        dx = max(self.x0 - x, 0.0, x - self.x1)
        dy = max(self.y0 - y, 0.0, y - self.y1)
        if dx == 0.0 and dy == 0.0:
            # 在内部：返回到最近边的负距离
            return -min(x - self.x0, self.x1 - x, y - self.y0, self.y1 - y)
        return math.hypot(dx, dy)

    def ray_hit(self, ox, oy, ang, max_range):
        """从 (ox,oy) 沿 ang 发射线，返回到矩形表面的距离；无交返回 None。

        用 slab method。
        """
        dx, dy = math.cos(ang), math.sin(ang)
        tmin, tmax = 0.0, max_range
        for lo, hi, o, d in ((self.x0, self.x1, ox, dx),
                             (self.y0, self.y1, oy, dy)):
            if abs(d) < 1e-12:
                if o < lo or o > hi:
                    return None
                continue
            t1, t2 = (lo - o) / d, (hi - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return None
        return tmin if tmin >= 0 else None


class World(object):
    def __init__(self, boxes):
        self.boxes = boxes

    def scan_ranges(self, x, y, yaw, n=512, ang_range=math.pi, rmax=20.0, rmin=0.5):
        """生成一帧 LaserScan 风格的 ranges（带机体 yaw）。

        ang_range=pi 表示 ±180°（hokuyo 全向 360°）。

        🔴 09-29 性能优化（**不改变结果**）：先按 `closest_dist` 预筛出
        rmax 以内的框。`closest_dist >= rmax` 的框不可能被 <= rmax 的射线
        击中（slab method 也只在 [0, rmax] 上求交），故预筛与全量等价。
        为什么必须加：官方随机图有 47 个框（14 细杆 + 13 建筑 + 20 rover），
        不预筛时 512 束 × 47 框 = 24064 次/帧，一条 100 m 航线（1300 步）
        就是 3100 万次求交；预筛后通常只剩 2~5 个候选框。
        """
        inc = 2.0 * ang_range / n
        cand = [b for b in self.boxes if b.closest_dist(x, y) < rmax]
        out = []
        for i in range(n):
            a_body = -ang_range + i * inc
            a_world = yaw + a_body          # 雷达装正，与机头同向
            best = None
            for b in cand:
                t = b.ray_hit(x, y, a_world, rmax)
                if t is not None and (best is None or t < best):
                    best = t
            if best is None or best < rmin:
                out.append(float('inf'))
            else:
                out.append(best)
        return out


class FakeMsg(object):
    def __init__(self, ranges, ang_range=math.pi):
        n = len(ranges)
        self.angle_min = -ang_range
        self.angle_max = ang_range
        self.angle_increment = 2.0 * ang_range / n
        self.ranges = ranges
        self.header = type('H', (), {'frame_id': 'laser_2d'})()
        self.range_min = R.RANGE_MIN
        self.range_max = R.RANGE_MAX


# ============================ 仿真主体 ============================
def simulate(world, start, goal, speed=1.5, dt=1.0 / 20.0, max_steps=20000,
             stride=4, verbose=False):
    """跑一次闭环避障。

    🔴 控制律必须与 radar_avoid.RadarPilot.step_toward **逐行一致**，
    否则测的不是同一个东西。此处直接复用 R.subgoal_from_scan()。
    """
    x, y = start
    gx, gy = goal
    yaw = math.atan2(gy - y, gx - x)

    min_clear = 1e9
    collisions = 0
    history = []
    stall = 0
    prev_x, prev_y = x, y
    commit = R.SideCommit()          # 与实飞同样带滞环状态

    for step in range(max_steps):
        d_goal = math.hypot(gx - x, gy - y)
        if d_goal < R.ARRIVE_R:
            return dict(ok=True, steps=step, x=x, y=y, min_clear=min_clear,
                        collisions=collisions, history=history, stall=stall)

        # --- 造一帧雷达（机体 yaw 下）---
        ranges = world.scan_ranges(x, y, yaw, n=R.BEAM_COUNT)
        msg = FakeMsg(ranges)

        # --- 与 radar_avoid 完全一致的处理链（共用同一函数）---
        sub_x, sub_y, shift, nearest, blocked = R.subgoal_from_scan(
            msg, yaw, (x, y), (gx, gy), R.MOUNT_DEFAULT, stride=stride,
            commit=commit)

        # 朝子目标走一步
        sdx, sdy = sub_x - x, sub_y - y
        sn = math.hypot(sdx, sdy)
        if sn < 1e-9:
            break

        # 🔴 速度随前向净空衰减（09-27 修"贴角点擦墙"）
        # ------------------------------------------
        # 定速 1.5 m/s 前进时，横向机动能力有限：飞机在 0.5m 余量处
        # 需要 0.3~0.5s 才能横移让开，这段时间里已经前进 0.5~0.8m
        # ⇒ 擦墙（闭环仿真场景 2 实测净空 -1.73m）。
        # 真实 UAV 遇到近障碍也会减速（PX4 的 MPC_XY_VEL 限幅同理）。
        # 这里让速度随"最近障碍净空"线性衰减到 V_MIN_SPEED。
        v = speed
        if nearest is not None:
            gap_now = nearest - R.CLEARANCE
            if gap_now < R.SLOWDOWN_RANGE:
                f = max(0.0, gap_now / R.SLOWDOWN_RANGE)
                v = R.V_MIN_SPEED + (speed - R.V_MIN_SPEED) * f

        step_len = min(v * dt, d_goal)
        x += sdx / sn * step_len
        y += sdy / sn * step_len

        # 🔴 机头只能"有限速率"转向（真实机身横滚/偏航都有惯性）。
        #    早期版本直接 yaw = atan2(子目标方向)，导致：
        #      机头瞬间对齐子目标 -> 下一帧机体系下通道角又变 ->
        #      自我激发震荡（实测 yaw 在 ∓20° 逐帧摆动、横向净位移≈0）。
        #    限制每帧最大转 Rate_YAW_MAX*dt，符合真实动力学，也消除这个
        #    人为的正反馈。
        want_yaw = math.atan2(sdy, sdx)
        dyaw = math.atan2(math.sin(want_yaw - yaw), math.cos(want_yaw - yaw))
        lim = R.YAW_RATE_MAX * dt
        if dyaw > lim:
            dyaw = lim
        elif dyaw < -lim:
            dyaw = -lim
        yaw = math.atan2(math.sin(yaw + dyaw), math.cos(yaw + dyaw))

        # --- 统计安全性 ---
        for b in world.boxes:
            cd = b.closest_dist(x, y)
            if cd < min_clear:
                min_clear = cd
            if cd < R.BODY_RADIUS:
                collisions += 1
                break
        history.append((x, y, shift, nearest))

        # --- 停滞检测 ---
        if math.hypot(x - prev_x, y - prev_y) < 1e-4:
            stall += 1
        else:
            stall = 0
        prev_x, prev_y = x, y
        if stall > 200:
            return dict(ok=False, reason='stall', steps=step, x=x, y=y,
                        min_clear=min_clear, collisions=collisions,
                        history=history, stall=stall)

    return dict(ok=False, reason='max_steps', steps=max_steps, x=x, y=y,
                min_clear=min_clear, collisions=collisions,
                history=history, stall=stall)


def main():
    # 场景 1：单个大障碍挡在正前方（必须绕过）
    box = Box(10, -6, 24, 6, 'wall')
    w = World([box])
    r = simulate(w, (0.0, 0.0), (34.0, 0.0), speed=1.5)
    print('=== 场景 1：正前方一堵墙 (x 10~24, y -6~6)，起点 (0,0) → 终点 (34,0) ===')
    print('  到达=%s 步数=%d 终点=(%.1f,%.1f) 最近距离=%.3f 碰撞帧=%d' %
          (r['ok'], r['steps'], r['x'], r['y'], r['min_clear'], r['collisions']))
    print('  安全余量要求 CLEARANCE=%.2f  => %s' %
          (R.CLEARANCE, 'PASS' if r['min_clear'] > R.CLEARANCE else
           ('擦过' if r['min_clear'] > R.BODY_RADIUS else 'FAIL 撞')))

    # 场景 2：三个障碍排成阵列（模拟建筑群，必须 S 形穿行）
    boxes = [Box(8, -2, 16, 8, 'a'), Box(20, -8, 28, 2, 'b'),
             Box(32, -2, 40, 8, 'c')]
    w2 = World(boxes)
    r2 = simulate(w2, (0.0, 0.0), (48.0, 0.0), speed=1.5)
    print()
    print('=== 场景 2：三障碍阵列，起点 (0,0) → 终点 (48,0) ===')
    print('  到达=%s 步数=%d 终点=(%.1f,%.1f) 最近距离=%.3f 碰撞帧=%d' %
          (r2['ok'], r2['steps'], r2['x'], r2['y'], r2['min_clear'],
           r2['collisions']))
    print('  %s' % ('PASS' if r2['min_clear'] > R.CLEARANCE else
                    ('擦过' if r2['min_clear'] > R.BODY_RADIUS else 'FAIL 撞')))

    # 场景 3：斜穿障碍区（更接近真实航线）
    boxes3 = [Box(5, 5, 14, 14, 'a'), Box(16, -14, 25, -5, 'b'),
              Box(27, 2, 36, 11, 'c')]
    w3 = World(boxes3)
    r3 = simulate(w3, (0.0, 0.0), (42.0, 20.0), speed=1.5)
    print()
    print('=== 场景 3：斜穿障碍区，起点 (0,0) → 终点 (42,20) ===')
    print('  到达=%s 步数=%d 终点=(%.1f,%.1f) 最近距离=%.3f 碰撞帧=%d' %
          (r3['ok'], r3['steps'], r3['x'], r3['y'], r3['min_clear'],
           r3['collisions']))
    print('  %s' % ('PASS' if r3['min_clear'] > R.CLEARANCE else
                    ('擦过' if r3['min_clear'] > R.BODY_RADIUS else 'FAIL 撞')))

    # 场景 4：狭窄通道（必须精确居中通过）
    boxes4 = [Box(10, 3, 30, 20, 'up'), Box(10, -20, 30, -3, 'down')]
    w4 = World(boxes4)
    r4 = simulate(w4, (0.0, 0.0), (40.0, 0.0), speed=1.5)
    print()
    print('=== 场景 4：狭窄通道（y∈[-3,3]，宽 6m），起点 (0,0) → 终点 (40,0) ===')
    print('  到达=%s 步数=%d 终点=(%.1f,%.1f) 最近距离=%.3f 碰撞帧=%d' %
          (r4['ok'], r4['steps'], r4['x'], r4['y'], r4['min_clear'],
           r4['collisions']))
    print('  %s' % ('PASS' if r4['min_clear'] > R.CLEARANCE else
                    ('擦过' if r4['min_clear'] > R.BODY_RADIUS else 'FAIL 撞')))

    # 场景 5：无障碍直线（不能误触发绕行）
    w5 = World([])
    r5 = simulate(w5, (0.0, 0.0), (30.0, 0.0), speed=1.5)
    print()
    print('=== 场景 5：无障碍，起点 (0,0) → 终点 (30,0)（验证不误绕） ===')
    print('  到达=%s 步数=%d 终点=(%.1f,%.1f)' %
          (r5['ok'], r5['steps'], r5['x'], r5['y']))
    # 直线应恰好 30m / (1.5*0.05) = 400 步
    print('  步数应≈400（直线），实测 %d ⇒ %s' %
          (r5['steps'], 'PASS 无多余绕行' if r5['steps'] < 430 else 'WARN 有绕行'))

    print()
    print('=' * 60)
    results = [('场景1 单墙', r), ('场景2 三障碍', r2), ('场景3 斜穿', r3),
               ('场景4 窄通道', r4), ('场景5 无障碍', r5)]
    worst = min(x[1]['min_clear'] for x in results)
    total_coll = sum(x[1]['collisions'] for x in results)
    all_arrive = all(x[1]['ok'] for x in results)
    print('汇总：5 个场景 全部到达=%s，总碰撞帧=%d，最小净空=%.3f m' %
          (all_arrive, total_coll, worst))
    print('结论：%s' % ('✅ 闭环避障通过（净空 > CLEARANCE %.2f）' % R.CLEARANCE
                      if worst > R.CLEARANCE and total_coll == 0
                      else '⚠ 需调参：净空 %.3f / 碰撞 %d' % (worst, total_coll)))
    return 0 if (worst > R.CLEARANCE and total_coll == 0) else 1


if __name__ == '__main__':
    sys.exit(main())
