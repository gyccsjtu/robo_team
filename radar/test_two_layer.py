#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两级集成验证：全局 A* 航点 + 局部雷达避障。

为什么必须做这个
----------------
纯局部避障（只用雷达）在"多障碍交错 + 窄缝"场景下会走错侧：
闭环仿真场景 2 实测，飞机从 a(8~16, -2~8) 与 b(20~28, -8~2) 之间的
4m 缝穿过时走了 y≈2.7，而 b 的上边界正好在 y=2 ⇒ 擦角。
从上方绕 b 会撞 a，唯一通路在 b 下方（y<-2）。

⇒ 这属于**全局选路**问题，局部避障解决不了。
真实系统里这由 astar_plan.py 的全局航点负责（离线、0 算力）。
本脚本验证"全局航点引导 + 局部雷达绕障"的组合是否安全。
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# 🔴 astar_plan.py 的查找路径（09-27 加）：
#   本机开发时它在 ../_demo2026/，而 VM 上在 ~/robocup_real/_demo2026/。
#   用一个可覆盖的搜索列表，避免为了跑测试去改代码。
#   可用环境变量 RADAR_DEMO_DIR 显式指定。
_DEMO_CANDIDATES = [
    os.environ.get('RADAR_DEMO_DIR', ''),
    os.path.join(os.path.dirname(HERE), '_demo2026'),      # 本机
    os.path.expanduser('~/robocup_real/_demo2026'),         # VM
    os.path.expanduser('~/_demo2026'),
    '/data/robocup_real/_demo2026',
]
for _c in _DEMO_CANDIDATES:
    if _c and os.path.isfile(os.path.join(_c, 'astar_plan.py')):
        sys.path.insert(0, _c)
        break
else:
    raise SystemExit(
        '找不到 astar_plan.py。请指定环境变量 RADAR_DEMO_DIR 指向 _demo2026 目录。\n'
        '已尝试:\n  ' + '\n  '.join(c for c in _DEMO_CANDIDATES if c))

import test_radar_offline as T

T.install_stubs()
import radar_avoid as R
import sim_radar_closed_loop as S
import astar_plan as A


def simulate_wps(world, start, wps, speed=1.5, dt=1.0 / 20.0,
                 max_steps=40000, stride=4):
    """沿一串航点飞，每个航点段内用雷达实时绕障。"""
    x, y = start
    yaw = math.atan2(wps[0][1] - y, wps[0][0] - x) if wps else 0.0
    commit = R.SideCommit()
    min_clear = 1e9
    collisions = 0
    wi = 0
    seg_steps = 0
    for step in range(max_steps):
        if wi >= len(wps):
            return dict(ok=True, steps=step, x=x, y=y, min_clear=min_clear,
                        collisions=collisions, wi=wi)
        gx, gy = wps[wi]
        if math.hypot(gx - x, gy - y) < R.ARRIVE_R:
            wi += 1
            seg_steps = 0
            continue
        seg_steps += 1
        if seg_steps > 6000:        # 单个航点段超时，判失败
            return dict(ok=False, reason='wp_timeout', steps=step, x=x, y=y,
                        min_clear=min_clear, collisions=collisions, wi=wi)

        ranges = world.scan_ranges(x, y, yaw, n=R.BEAM_COUNT)
        msg = S.FakeMsg(ranges)
        sx, sy, sh, nr, bl = R.subgoal_from_scan(
            msg, yaw, (x, y), (gx, gy), R.MOUNT_DEFAULT, stride=stride,
            commit=commit)
        sdx, sdy = sx - x, sy - y
        sn = math.hypot(sdx, sdy)
        if sn < 1e-9:
            continue
        x += sdx / sn * (speed * dt)
        y += sdy / sn * (speed * dt)
        wy = math.atan2(sdy, sdx)
        dy = math.atan2(math.sin(wy - yaw), math.cos(wy - yaw))
        lim = R.YAW_RATE_MAX * dt
        dy = max(-lim, min(lim, dy))
        yaw = math.atan2(math.sin(yaw + dy), math.cos(yaw + dy))

        for b in world.boxes:
            cd = b.closest_dist(x, y)
            if cd < min_clear:
                min_clear = cd
            if cd < R.BODY_RADIUS:
                collisions += 1
                break
    return dict(ok=False, reason='max_steps', steps=max_steps, x=x, y=y,
                min_clear=min_clear, collisions=collisions, wi=wi)


def boxes_to_buildings(boxes):
    """把测试用的 Box 转成 astar_plan 期望的 6 元组 (name, cx, cy, hx, hy, yaw)。

    🔴 格式来源：astar_plan.buildings_from_world() 第 77 行
       `out.append((nm, cx, cy, hx, hy, 0.0))` —— 4 元组会 unpack 失败
       （实测 `ValueError: not enough values to unpack (expected 6, got 4)`）。
    """
    bld = []
    for b in boxes:
        cx = (b.x0 + b.x1) / 2.0
        cy = (b.y0 + b.y1) / 2.0
        hx = (b.x1 - b.x0) / 2.0
        hy = (b.y1 - b.y0) / 2.0
        bld.append((b.name, cx, cy, hx, hy, 0.0))
    return bld


def probe_astar_api():
    """探查 astar_plan 暴露的函数签名，避免猜错参数。"""
    names = [n for n in dir(A) if not n.startswith('_')]
    print('astar_plan 公开符号:', ', '.join(sorted(names)))
    import inspect
    for n in sorted(names):
        f = getattr(A, n)
        if callable(f):
            try:
                print('   %s%s' % (n, inspect.signature(f)))
            except (TypeError, ValueError):
                print('   %s(...)' % n)


def main():
    probe_astar_api()
    print()

    cases = [
        # (名称, 障碍, 起点, 终点)
        ('场景 2 三障碍交错',
         [S.Box(8, -2, 16, 8, 'a'), S.Box(20, -8, 28, 2, 'b'),
          S.Box(32, -2, 40, 8, 'c')],
         (0.0, 0.0), (48.0, 0.0)),
        ('场景 4 窄通道(宽 6m)',
         [S.Box(10, 3, 30, 20, 'up'), S.Box(10, -20, 30, -3, 'down')],
         (0.0, 0.0), (40.0, 0.0)),
        ('场景 1 单墙',
         [S.Box(10, -6, 24, 6, 'wall')],
         (0.0, 0.0), (34.0, 0.0)),
        ('场景 3 斜穿',
         [S.Box(5, 5, 14, 14, 'a'), S.Box(16, -14, 25, -5, 'b'),
          S.Box(27, 2, 36, 11, 'c')],
         (0.0, 0.0), (42.0, 20.0)),
    ]

    npass = 0
    for desc, boxes, start, goal in cases:
        world = S.World(boxes)
        bld = boxes_to_buildings(boxes)
        print('=== %s 全局 A* + 局部雷达 ===' % desc)
        print('  建筑: %s' % [(b[0], b[1], b[2], b[3], b[4]) for b in bld])
        try:
            g, raw, path = A.plan(bld, [], start, goal)
            nb, nm = A.verify(g, path)
            print('  A* 航点 %d 个（原始 %d）穿墙 %d 处' % (len(path), len(raw), nb))
            print('    %s' % [(round(p[0], 1), round(p[1], 1)) for p in path])
        except Exception as e:
            print('  ❌ A* 失败: %s: %s' % (type(e).__name__, e))
            continue
        r = simulate_wps(world, start, list(path), speed=1.5)
        ok = (r['ok'] and r['min_clear'] > R.CLEARANCE and r['collisions'] == 0)
        if ok:
            npass += 1
        print('  结果: 到达=%s 步数=%d 终点=(%.1f,%.1f) 净空=%.3f 碰撞帧=%d => %s'
              % (r['ok'], r['steps'], r['x'], r['y'], r['min_clear'],
                 r['collisions'], '✅ PASS' if ok else '⚠ 净空 %.3f/碰撞 %d'
                 % (r['min_clear'], r['collisions'])))
        print()

    print('=' * 60)
    print('两级集成：%d/%d 通过' % (npass, len(cases)))
    return 0 if npass == len(cases) else 1


if __name__ == '__main__':
    sys.exit(main())
