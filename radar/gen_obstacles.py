#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从官方 base.world 解析静态障碍物 -> 生成 ObstacleAvoid_master.py 要读的障碍表。

官方 ObstacleAvoid_master.py 第 10 行:  self.obstlist = numpy.loadtxt('2024.txt')
  -> 每行两个数 (x, y)，单位米，世界坐标系。官方没给这个文件。

⚠ 关键：base.world 里每个模型出现两次 ——
   ① world 直接子元素 <model name='house_1_66'> ... 是【模型定义】(无 pose)
   ② world/state/model[name='house_1_66']/pose 是【Gazebo 世界状态快照】= 真实位姿
   解析必须走 <state>，否则拿到的坐标全是 0 或错的。

用法:  python gen_obstacles.py <base.world> <out_prefix>
"""
import math
import sys
import xml.etree.ElementTree as ET

# 建筑类（矩形体，按角点展开）
BLOCK_KEYS = (
    'house_1', 'house_2', 'house_3', 'gas_station', 'fast_food',
)
# 细杆/小型（单点即可）
THIN_KEYS = (
    'lamp_post', 'stop_sign', 'fire_hydrant', 'Shelves', 'shelves',
    'dumpster', 'cardboard_box', 'EuroPallet', 'europallet',
)
# 明确不是障碍
SKIP_KEYS = (
    'city_terrain', 'ocean', 'asphalt_plane', 'ground_plane', 'sidewalk', 'road',
)

# 各类建筑的半宽（米）—— 取自 base.world 里 <scale> 与 meshes 常见尺寸
HALF = {
    'house_1': (9.0, 7.5),
    'house_2': (9.0, 7.5),
    'house_3': (7.5, 5.5),
    'gas_station': (7.0, 5.0),
    'fast_food': (7.0, 5.0),
}


def classify(name):
    low = name.lower()
    for k in SKIP_KEYS:
        if low.startswith(k.lower()):
            return 'skip'
    for k in BLOCK_KEYS:
        if low.startswith(k.lower()):
            return 'block'
    for k in THIN_KEYS:
        if low.startswith(k.lower()):
            return 'thin'
    return 'other'


def parse_state(path):
    """返回 [(name, x, y, z, yaw), ...]，取自 world/state 块（真实位姿）。"""
    world = ET.parse(path).getroot().find('world')
    if world is None:
        raise SystemExit('no <world> in %s' % path)
    state = world.find('state')
    if state is None:
        raise SystemExit('no <state> in %s —— 该 world 没保存状态快照' % path)
    out = []
    for m in state.findall('model'):
        nm = m.get('name')
        p = m.find('pose')
        if nm is None or p is None or not p.text:
            continue
        v = p.text.split()
        try:
            x, y, z = float(v[0]), float(v[1]), float(v[2])
        except (ValueError, IndexError):
            continue
        yaw = 0.0
        if len(v) >= 6:
            try:
                yaw = float(v[5])
            except ValueError:
                pass
        out.append((nm, x, y, z, yaw))
    return out


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    src, prefix = sys.argv[1], sys.argv[2]
    items = parse_state(src)

    blocks, thin, others = [], [], []
    for it in items:
        k = classify(it[0])
        if k == 'block':
            blocks.append(it)
        elif k == 'thin':
            thin.append(it)
        elif k == 'other':
            others.append(it)

    # 去重（state 中同名只应出现一次，保险）
    def dedup(rows):
        seen, out = set(), []
        for r in rows:
            if r[0] in seen:
                continue
            seen.add(r[0])
            out.append(r)
        return out
    blocks, thin, others = dedup(blocks), dedup(thin), dedup(others)

    # ---- 中心点表 ----
    cpath = prefix + '_center.txt'
    n_c = 0
    with open(cpath, 'w', encoding='utf-8', newline='\n') as fh:
        for (nm, x, y, z, yaw) in blocks + thin:
            fh.write('%.4f %.4f\n' % (x, y))
            n_c += 1

    # ---- 角点表（矩形 4 角 + 细杆 1 点）----
    epath = prefix + '_corner.txt'
    rows = []
    for (nm, x, y, z, yaw) in blocks:
        key = next(k for k in BLOCK_KEYS if nm.lower().startswith(k.lower()))
        hx, hy = HALF[key]
        c, s = math.cos(yaw), math.sin(yaw)
        for dx, dy in ((hx, hy), (hx, -hy), (-hx, hy), (-hx, -hy)):
            rows.append((x + dx * c - dy * s, y + dx * s + dy * c))
    for (nm, x, y, z, yaw) in thin:
        rows.append((x, y))
    with open(epath, 'w', encoding='utf-8', newline='\n') as fh:
        for (x, y) in rows:
            fh.write('%.4f %.4f\n' % (x, y))

    # ---- 轮廓采样表（沿矩形周长每 STEP 米一点）★避障要用的就是这份 ----
    # 原因：ObstacleAvoid 是「点避障」。若只给建筑中心点，它会把中心当成
    # 一个 0 尺寸的障碍，无人机照样从房子中间穿过去。必须把矩形轮廓打散成点。
    STEP = 1.2
    opath = prefix + '_outline.txt'
    orows = []
    for (nm, x, y, z, yaw) in blocks:
        key = next(k for k in BLOCK_KEYS if nm.lower().startswith(k.lower()))
        hx, hy = HALF[key]
        c, s = math.cos(yaw), math.sin(yaw)
        # 沿 4 条边采样
        edges = []
        for (dx0, dy0, dx1, dy1) in ((hx, hy, hx, -hy), (hx, -hy, -hx, -hy),
                                     (-hx, -hy, -hx, hy), (-hx, hy, hx, hy)):
            n = max(2, int(round(2 * max(hx, hy) / STEP)) + 1)
            for i in range(n):
                t = i / (n - 1.0)
                edges.append((dx0 + t * (dx1 - dx0), dy0 + t * (dy1 - dy0)))
        for (dx, dy) in edges:
            orows.append((x + dx * c - dy * s, y + dx * s + dy * c))
    for (nm, x, y, z, yaw) in thin:
        orows.append((x, y))
    # 去掉重复点（相邻边共享角点）
    uniq, seen = [], set()
    for (px, py) in orows:
        kk = (round(px, 2), round(py, 2))
        if kk in seen:
            continue
        seen.add(kk)
        uniq.append((px, py))
    with open(opath, 'w', encoding='utf-8', newline='\n') as fh:
        for (px, py) in uniq:
            fh.write('%.4f %.4f\n' % (px, py))
    orows = uniq

    # ---- 细杆表（只含灯杆/路牌/消防栓等小件，供 A* 单独膨胀）----
    # 注意：outline 表把建筑轮廓也打散了，不能直接拿去当"细杆"用，
    # 否则建筑会被重复膨胀两次（矩形 + 一圈点）。
    ppath = prefix + '_poles.txt'
    with open(ppath, 'w', encoding='utf-8', newline='\n') as fh:
        for (nm, x, y, z, yaw) in thin:
            fh.write('%.4f %.4f\n' % (x, y))

    print('源: %s' % src)
    print('  <state> 中模型数 = %d' % len(items))
    print('  建筑(矩形) = %d   细杆/小件 = %d   其他未归类 = %d' % (len(blocks), len(thin), len(others)))
    print()
    print('  -> %s  点数=%d' % (cpath, n_c))
    print('  -> %s  点数=%d' % (epath, len(rows)))
    print('  -> %s  点数=%d  ★避障用这份（沿建筑周长每 %.1fm 采样）' %
          (opath, len(orows), STEP))
    print('  -> %s  点数=%d  细杆表（A* 单独用，别混进轮廓表）' % (ppath, len(thin)))
    print()
    print('  建筑明细:')
    for (nm, x, y, z, yaw) in blocks:
        print('    %-24s (%8.2f, %8.2f)  yaw=%6.2f' % (nm, x, y, yaw))
    print()
    print('  细杆/小件明细:')
    for (nm, x, y, z, yaw) in thin:
        print('    %-24s (%8.2f, %8.2f)' % (nm, x, y))
    if others:
        print()
        print('  未归类（不写进障碍表）: %s' % ', '.join(o[0] for o in others))

    xs = [r[1] for r in blocks + thin]
    ys = [r[2] for r in blocks + thin]
    if xs:
        print()
        print('  坐标范围: x [%.1f, %.1f]  y [%.1f, %.1f]   (官方场地 200m x 100m)' %
              (min(xs), max(xs), min(ys), max(ys)))


if __name__ == '__main__':
    main()
