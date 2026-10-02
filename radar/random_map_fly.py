#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""随机地图（官方 map_generator 产出）下的两级闭环避障验证。

回答的问题：**比赛是随机图，A* + 雷达这套还能不能避障？**

做法（不碰 Gazebo，纯几何闭环，可反复跑）：
  1. 用官方 `map_generator.py` 生成 N 张随机图，每张 = `robocup.world` +
     `black_box.txt` + `obstacle.txt`；
  2. 障碍取**官方 black_box.txt 的 47 个轴对齐矩形真值**（= 官方认定的全部
     障碍：14 细杆/横杆 + 13 栋建筑 + 20 辆 rover），不用 mesh、不解析 world；
  3. A* 用同一份真值规划航点；雷达闭环用 `sim_radar_closed_loop` 的完美射线
     模型 + `radar_avoid` 的真实控制律（`subgoal_from_scan`，与实飞同一函数）；
  4. 统计：A* 航点穿墙数 / 是否到达 / 最小净空 / 碰撞帧 / 步数。

用法：
  python random_map_fly.py --gen-dir ../_gen_20260929 --n-route 3 --seed 11
  python random_map_fly.py --gen-dir ../_gen_20260929 --n-route 3 --speed 1.2
"""
import argparse
import glob
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
# test_two_layer / sim_radar_closed_loop 靠它找 astar_plan.py
os.environ.setdefault('RADAR_DEMO_DIR', HERE)

import test_radar_offline as T          # noqa: E402

T.install_stubs()
import radar_avoid as R                 # noqa: E402
import sim_radar_closed_loop as S       # noqa: E402
import test_two_layer as T2             # noqa: E402
import astar_plan as A                  # noqa: E402

# 官方 6 个起飞点（map_generator.py:uav_takeoff_points）
TAKEOFF = [(0.0, -3.0), (3.0, -3.0), (0.0, 0.0), (3.0, 0.0), (0.0, 3.0), (3.0, 3.0)]


def find_maps(gen_dir):
    """找 gen_dir 下可用的随机图（非空 world + black_box.txt）。"""
    out = []
    for d in sorted(glob.glob(os.path.join(gen_dir, 'r*'))):
        w = os.path.join(d, 'robocup.world')
        bb = os.path.join(d, 'black_box.txt')
        if not os.path.isfile(bb):
            continue
        sz = os.path.getsize(w) if os.path.isfile(w) else 0
        out.append((os.path.basename(d), bb, sz))
    return out


def boxes_of(bb_path):
    """官方 black_box.txt -> sim_radar_closed_loop.Box 列表。

    ⚠ 签名坑：`A.load_black_box` 返回 (x0, x1, y0, y1)，
       而 `S.Box.__init__` 是 (x0, y0, x1, y1) —— 顺序不同，必须显式换位。
    """
    out = []
    for i, (bx0, bx1, by0, by1) in enumerate(A.load_black_box(bb_path)):
        out.append(S.Box(bx0, by0, bx1, by1, 'bb%02d' % i))
    return out


def make_routes(bld, margin, n_route, seed, min_len=45.0):
    """在自由栅格里取 n_route 条航线（起点用官方起飞点，终点随机且够远）。"""
    g = A.Grid(bld, [], margin=margin)
    rng = random.Random(seed)
    free = [g.to_xy(i, j) for j in range(g.ny) for i in range(g.nx)
            if g.is_free(i, j)]
    if not free:
        return []
    routes = []
    for k in range(n_route):
        s = TAKEOFF[k % len(TAKEOFF)]
        t = rng.choice(free)
        for _ in range(800):
            t = rng.choice(free)
            if math.dist(s, t) >= min_len:
                break
        routes.append((s, t))
    return routes


def takeoff_blocked_by(bld, s):
    """起点是否被某个障碍矩形覆盖 —— 返回该矩形名，否则 None。

    🔴 09-29 实测发现的**官方生成器缺陷**：`map_generator.py` 的
    `create_point()` 只保证 13 栋建筑**彼此**不重叠（判据 `size_box_judge`
    +3 m 余量），**完全不检查与 6 个起飞点 (0,±3)/(3,±3) 的距离**。
    `create_human_point()` 有 `uav_start_clearance = 8.0` 的行人避让，
    建筑放置却没有。
    后果：6 张官方随机图里 2 张（33%）把 30×25 的 fast_food_93 盖在
    起飞点上（实测框 [-11,19]×[1.5,26.5] 与 [-9,21]×[1.5,26.5] 均含 (0,3)）。
    此时换任何算法都无解 —— 飞机在楼里，而楼高 7.86 m > 6 m 限高，
    既飞不出去也压不过顶。
    """
    for (nm, cx, cy, hx, hy, _y) in bld:
        if abs(s[0] - cx) <= hx and abs(s[1] - cy) <= hy:
            return nm
    return None


def run_one(bld, grid, bb_path, start, goal, speed, margin):
    """A* 规划 + 雷达闭环飞一遍。返回统计字典。"""
    out = {'start': start, 'goal': goal}
    out['blocked_by'] = takeoff_blocked_by(bld, start)
    try:
        g2, raw, path = A.plan(bld, [], start, goal, margin)
    except Exception as e:
        out['err'] = 'A* %s: %s' % (type(e).__name__, e)
        return out
    nbad, bad = A.verify(g2, path)
    out['n_wp'] = len(path)
    out['n_wall'] = nbad
    out['bad'] = bad[:2]
    world = S.World(boxes_of(bb_path))
    # ⚠ 起点被楼压住时，A* 的 nearest_free 已把 path[0] 吸附到楼外。
    #   闭环模拟从**吸附点**起飞 —— 否则第 0 帧就记在楼内，把
    #   "官方把楼盖在起飞点"的账算到避障算法头上（实测第一次就是
    #   第 0 步撞 bb19 fast_food）。这样两类问题就能分开看。
    sim_start = path[0] if path else start
    out['sim_start'] = sim_start
    r = T2.simulate_wps(world, sim_start, list(path), speed=speed)
    out['ok'] = r['ok']
    out['reason'] = r.get('reason')
    out['steps'] = r['steps']
    out['end'] = (round(r['x'], 1), round(r['y'], 1))
    out['min_clear'] = r['min_clear']
    out['coll'] = r['collisions']
    out['d_goal'] = math.hypot(goal[0] - r['x'], goal[1] - r['y'])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gen-dir', required=True, help='含 r1..rN 随机图子目录')
    ap.add_argument('--n-route', type=int, default=3, help='每张图几条航线')
    ap.add_argument('--seed', type=int, default=11)
    ap.add_argument('--speed', type=float, default=1.2, help='前进速度 m/s（同实飞）')
    ap.add_argument('--margin', type=float, default=2.0)
    a = ap.parse_args()

    maps = find_maps(a.gen_dir)
    print('找到随机图 %d 张：%s' % (len(maps),
          ', '.join('%s(%s)' % (n, '空' if sz < 1000 else '%dKB' % (sz // 1024))
                    for n, _b, sz in maps)))
    usable = [(n, b) for n, b, sz in maps if sz >= 1000]
    if not usable:
        print('⛔ 没有非空 world（官方生成器 fallback bug 会产出 0 字节 world）')
        return 1
    print('可用 %d 张（%d 张是官方 fallback 产出的空世界，跳过）'
          % (len(usable), len(maps) - len(usable)))
    print('障碍源 = 官方 black_box.txt（47 矩形真值）  '
          '速度 %.1f m/s  余量 %.1f m' % (a.speed, a.margin))
    print()

    n_tot = n_arr = n_clean = n_wallfree = n_blocked = 0
    worst_clear = 1e9
    worst_case = None
    tot_coll = 0
    for mi, (name, bb) in enumerate(usable):
        bld = A.boxes_from_black_box(bb)
        g = A.Grid(bld, [], margin=a.margin)
        routes = make_routes(bld, a.margin, a.n_route, a.seed + mi * 7)
        print('=== 图 %s   障碍 %d 个矩形' % (name, len(bld)))
        for ri, (s, t) in enumerate(routes):
            r = run_one(bld, g, bb, s, t, a.speed, a.margin)
            n_tot += 1
            if 'err' in r:
                print('  航线%d (%.0f,%.0f)->(%.0f,%.0f)  ❌ %s'
                      % (ri, s[0], s[1], t[0], t[1], r['err']))
                continue
            n_arr += 1 if r['ok'] else 0
            tot_coll += r['coll']
            n_wallfree += 1 if r['n_wall'] == 0 else 0
            if r['blocked_by']:
                n_blocked += 1
            if r['min_clear'] > R.CLEARANCE and r['coll'] == 0:
                n_clean += 1
            if r['min_clear'] < worst_clear:
                worst_clear = r['min_clear']
                worst_case = (name, ri, s, t)
            tag = '✅' if (r['ok'] and r['n_wall'] == 0 and r['coll'] == 0
                           and r['min_clear'] > R.CLEARANCE) else '⚠'
            blk = ('  ⛔起飞点被 %s 压住(A*已吸附到 %.0f,%.0f)'
                   % (r['blocked_by'], r['sim_start'][0], r['sim_start'][1])
                   if r['blocked_by'] else '')
            print('  航线%d (%.0f,%.0f)->(%.0f,%.0f)  A*航点%2d 穿墙%d  '
                  '到达=%-5s 步数%5d 终点(%.1f,%.1f) 距目标%6.1fm  '
                  '净空%+.3f 碰撞%d  %s%s'
                  % (ri, s[0], s[1], t[0], t[1], r['n_wp'], r['n_wall'],
                     r['ok'], r['steps'], r['end'][0], r['end'][1], r['d_goal'],
                     r['min_clear'], r['coll'], tag, blk))
        print()

    print('=' * 72)
    print('汇总：%d 组航线（%d 张随机图 × %d 条）' % (n_tot, len(usable), a.n_route))
    print('  A* 航点 0 穿墙 : %d/%d' % (n_wallfree, n_tot))
    print('  闭环到达      : %d/%d' % (n_arr, n_tot))
    print('  净空>%.2f 且 0 碰撞 : %d/%d' % (R.CLEARANCE, n_clean, n_tot))
    print('  累计碰撞帧    : %d' % tot_coll)
    print('  全局最小净空  : %.3f m  %s' % (worst_clear, worst_case or ''))
    print()
    print('  ⛔ 官方生成器缺陷：起飞点被建筑压住 %d/%d 组'
          '（map_generator 不检查建筑与起飞点距离）' % (n_blocked, n_tot))
    ok = (n_wallfree == n_tot and n_arr == n_tot and n_clean == n_tot)
    print('结论：%s' % ('✅ 随机地图下两级避障全部通过（A* 零穿墙 + 闭环 0 碰撞）'
                      if ok else '⚠ 有未达标项，见上表'))
    if n_blocked:
        print('     （起飞点被压的 %d 组已从 A* 吸附点起飞，不记为算法失败，'
              '但比赛现场会直接无解，须反馈组委会）' % n_blocked)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
