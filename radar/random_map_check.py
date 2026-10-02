#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""随机地图离线验证：官方 map_generator 现生成 -> astar_plan 现解析 -> A* 规划 0 穿墙。

流程（与比赛一致）：
  1. 用官方 robocup/map_generator.py 生成一份全新的随机 robocup.world
     （模型位置由脚本随机生成，规则 2.5(5)）
  2. 用 astar_plan.buildings_from_world / poles_from_world 解析当次 world
  3. 建筑 footprint 用官方 size_box（map_generator.py 里的权威尺寸表）覆盖
     —— 本机无 Gazebo mesh，HALF 兜底值与 mesh 实差 2~5 倍，不能当验证几何
  4. 起飞点可达性 + 20 组随机起终点 A* 规划，验收 0 穿墙、不出场地

用法：
  python random_map_check.py --gen-dir <官方map_generator.py所在目录> [--maps 9]

退出码 0 = 全部 PASS。
"""
import argparse
import math
import os
import random
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import astar_plan as A

# 官方 map_generator.py 的 size_box（name_list 顺序，整 footprint 宽×长，米）。
# 这是组委会生成 black_box 用的权威尺寸，比 HALF 拍脑袋值可靠。
OFFICIAL_SIZE = {
    'house_1_146': (16.0, 12.0),
    'house_3_156': (5.0, 17.0),
    'house_3_157': (17.0, 5.0),
    'house_3_158': (5.0, 17.0),
    'gas_station_73': (20.0, 15.0),
    'fast_food_93': (30.0, 25.0),
    'house_1_66': (16.0, 12.0),
    'house_1_67': (18.0, 18.0),
    'house_1_146_clone': (18.0, 18.0),
    'house_2_71': (11.0, 9.0),
    'house_2_125': (11.0, 9.0),
    'house_2_126': (11.0, 9.0),
    'house_3_68': (14.0, 5.0),
}

TAKEOFF_POINTS = [(0.0, -3.0), (3.0, -3.0), (0.0, 0.0), (3.0, 0.0),
                  (0.0, 3.0), (3.0, 3.0)]


def official_rects(world_path):
    """用我方解析器取位置/yaw，用官方 size_box 取尺寸 -> [(name,cx,cy,hx,hy,yaw)]"""
    raw = A.buildings_from_world(world_path, use_mesh=False)
    out = []
    for (nm, cx, cy, _hx, _hy, yaw) in raw:
        key = next((k for k in OFFICIAL_SIZE if nm.lower().startswith(k.lower())),
                   None)
        if key is None:
            raise RuntimeError('解析出名单外建筑 %r' % nm)
        w, l = OFFICIAL_SIZE[key]
        hx, hy = w / 2.0, l / 2.0
        # 与 astar_plan.buildings_from_world 相同的 ±90° 互换规则
        if abs(abs(yaw) - math.pi / 2) < 0.4:
            hx, hy = hy, hx
        out.append((nm, cx, cy, hx, hy, yaw))
    return out


def gen_world(gen_dir, workdir, seed=None):
    """在 workdir 里跑官方生成器，返回 robocup.world 路径。

    seed 不为 None 时给官方脚本固定随机种子（可复现调试）。"""
    src = os.path.join(gen_dir, 'map_generator.py')
    with open(src, encoding='utf-8') as f:
        code = f.read()
    # 官方把 robocup.world 写到 ~/PX4_Firmware/...，本地验证改为当前目录
    code = code.replace(
        "output_path = os.path.expanduser('~/PX4_Firmware/Tools/sitl_gazebo/worlds/')",
        "output_path = './'")
    if 'output_path = ' not in code:
        raise RuntimeError('map_generator.py 里没找到 output_path，请检查官方脚本是否改版')
    if seed is not None:
        # 官方脚本用裸 random.*，种子插在 import 后即可复现
        code = code.replace('import os\n', 'import os\nrandom.seed(%d)\n' % seed,
                            1)
    local = os.path.join(workdir, 'map_generator_local.py')
    with open(local, 'w', encoding='utf-8') as f:
        f.write(code)
    # 模板文件 symlink/copy 到 workdir
    for fn in ('base1.world', 'base2.world', 'base3.world', 'rover_static.world'):
        s = os.path.join(gen_dir, fn)
        if not os.path.isfile(s):
            raise RuntimeError('缺少官方模板 %s' % fn)
        with open(s, 'rb') as fi, open(os.path.join(workdir, fn), 'wb') as fo:
            fo.write(fi.read())
    r = subprocess.run([sys.executable, local], cwd=workdir,
                       capture_output=True, text=True, timeout=300)
    if r.returncode != 0:
        raise RuntimeError('map_generator 运行失败:\n%s' % r.stderr[-800:])
    out = os.path.join(workdir, 'robocup.world')
    if not os.path.isfile(out):
        raise RuntimeError('未生成 robocup.world')
    return out


def check_one(world_path, n_random=20, seed=7):
    """返回 (errors, stats)"""
    errs, stats = [], {}
    buildings = official_rects(world_path)
    stats['buildings'] = len(buildings)
    if len(buildings) != 13:
        errs.append('建筑数 %d != 13（随机地图模板应有 13 栋）' % len(buildings))
    poles = A.poles_from_world(world_path)
    stats['poles'] = len(poles)

    g = A.Grid(buildings, poles, margin=2.0)

    # 1) 起飞点必须自由且在栅格内
    for (tx, ty) in TAKEOFF_POINTS:
        nm = g.point_hits_building(tx, ty)
        if nm:
            errs.append('起飞点 (%.0f,%.0f) 落在建筑 %s 内' % (tx, ty, nm))

    # 2) 随机起终点 A*，0 穿墙、不出场地
    rng = random.Random(seed)
    ok = 0
    for k in range(n_random):
        s, t = None, None
        for _ in range(4000):
            x = rng.uniform(-45.0, 125.0)
            y = rng.uniform(-45.0, 45.0)
            if g.point_hits_building(x, y) is None:
                s = (x, y)
                break
        for _ in range(4000):
            x = rng.uniform(-45.0, 125.0)
            y = rng.uniform(-45.0, 45.0)
            if g.point_hits_building(x, y) is None:
                t = (x, y)
                break
        try:
            _, _, sp = A.plan(buildings, poles, s, t, margin=2.0)
        except RuntimeError as e:
            errs.append('第%d组 A* 无解: %s' % (k, e))
            continue
        nb, bad = A.verify(g, sp)
        if nb:
            errs.append('第%d组 穿墙%d 处: %s' % (k, nb, bad[:2]))
            continue
        # 不出官方场地（围墙 x∈{-50,130}、y=±50，留 1 m 余量）
        for (px, py) in sp:
            if not (-49.0 <= px <= 131.0 and -49.0 <= py <= 49.0):
                errs.append('第%d组 航点 (%.1f,%.1f) 出场地' % (k, px, py))
                break
        else:
            ok += 1
    stats['planned_ok'] = ok
    stats['planned_total'] = n_random
    return errs, stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gen-dir', required=True,
                    help='官方 map_generator.py + base1-3.world + rover_static.world 所在目录')
    ap.add_argument('--maps', type=int, default=9, help='随机生成几份图验证（默认 9）')
    ap.add_argument('--pairs', type=int, default=20, help='每份图随机起终点组数')
    a = ap.parse_args()

    total_err = 0
    for i in range(a.maps):
        world = None
        for attempt in range(4):
            with tempfile.TemporaryDirectory() as wd:
                try:
                    w = gen_world(a.gen_dir, wd, seed=1000 + i * 10 + attempt)
                except RuntimeError as e:
                    print('[map %d.%d] 生成失败: %s' % (i, attempt, e))
                    continue
                # 官方脚本 find_solution 失败 10 次后会走 "Base world is used"
                # 分支，但那个分支重复 readlines 已耗尽的文件句柄 -> house.world
                # 为空 -> robocup.world 缺全部建筑。检出即重Roll，别让坏图进验证。
                if len(official_rects(w)) == 0:
                    print('[map %d.%d] 官方生成器产出空图（其 Base-world '
                          'fallback 的已知 bug），重roll' % (i, attempt))
                    continue
                world = w
                # 拷贝出临时目录，验证期间不被清理
                import shutil
                keep = os.path.join(tempfile.mkdtemp(), 'robocup_%d.world' % i)
                shutil.copy(w, keep)
                world = keep
                break
        if world is None:
            print('[map %d] 4 次重roll均失败' % i)
            total_err += 1
            continue
        errs, stats = check_one(world, n_random=a.pairs, seed=7 + i)
        status = 'PASS' if not errs else 'FAIL'
        print('[map %d] %s  建筑%d 细杆%d  规划 %d/%d 组 0 穿墙'
              % (i, status, stats['buildings'], stats['poles'],
                 stats['planned_ok'], stats['planned_total']))
        for e in errs:
            print('    ✗', e)
        total_err += len(errs)

    print()
    if total_err == 0:
        print('==> 全部随机地图 PASS（%d 份图 × %d 组起终点）' % (a.maps, a.pairs))
        return 0
    print('==> FAIL：共 %d 个问题' % total_err)
    return 1


if __name__ == '__main__':
    sys.exit(main())
