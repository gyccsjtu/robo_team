#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""官方 base.world 场景下的 A* 绕障航点规划器（确定性、可离线验证）。

为什么不用官方 ObstacleAvoid_master.py：
  实测（roll_test.py）它在 13 栋建筑场景下要么不避（穿墙 25~43 次），
  要么直接 ValueError: math domain error（点密时必崩）。原因见 roll_test.py 注释。
  => 演示用这套自研 A*。

输入：obstacles_* 表（由 gen_obstacles.py 从 base.world 解析）
输出：折线航点 [(x, y), ...]，起点 -> 若干绕障拐点 -> 终点

用法：
  python astar_plan.py <obstacles.txt> --start X Y --goal X Y [--margin 2.0] [--png out.png]
  python astar_plan.py <obstacles.txt> --selftest        # 跑 20 组随机起终点自检 0 穿墙
"""
import argparse
import heapq
import math
import os
import sys

HALF = {
    'house_1': (9.0, 7.5),
    'house_2': (9.0, 7.5),
    'house_3': (7.5, 5.5),
    'gas_station': (7.0, 5.0),
    'fast_food': (7.0, 5.0),
}
BLOCK_KEYS = tuple(HALF.keys())

# 场地栅格边界：覆盖官方随机地图全场。
# 依据 gitee.com/robin_shaun/XTDrone 的 robocup/map_generator.py：
#   · obstacle.txt 的围墙在 x∈{-50,130}、y=±50
#   · 13 栋建筑由 rand_x/create_point 生成，分布在 x∈[-34,96]、y∈[-33,32]
#   · 6 机起飞点 (0,±3)/(3,±3)
# 旧边界 (-48..118, -52..38) 只覆盖 base.world 城区，且不含北侧围墙，
# 随机图下 A* 可能把航点规划到场地外。扩到官方场地全幅 + 2 m 余量。
X0, X1 = -52.0, 132.0
Y0, Y1 = -52.0, 52.0
CELL = 1.0

# 细杆类障碍（<state> 里的模型名前缀）：随机地图只挪 13 栋建筑，
# 灯杆/牌/栓随模板固定，但直接解析当次 world 文件，
# 避免复用旧 obstacles_*.txt（从 base.world 导出）在新模板上的错位。
POLE_KEYS = ('lamp_post', 'stop_sign', 'fire_hydrant')

# ---------------------------------------------------------------- 官方权威真值
# 来源：gitee.com/robin_shaun/XTDrone 的 robocup/map_generator.py
#   · name_list（第 273-275 行）= 13 个建筑实例名，顺序固定
#   · size_box （obstacle_list 内，第 111-112 行）= 上表逐项对应的 (x 宽, y 宽)
#   · 官方 black_box.txt / obstacle.txt 就是拿这两者按
#         x_min = cx - size_box[i][0]/2 ,  x_max = cx + size_box[i][0]/2
#     生成的**世界系轴对齐**矩形（不旋转、不交换长短边）。
# 🔴 实测（VM 09-29）：这不是"理论值"，是官方唯一的障碍定义 —— 官方
#   ObstacleAvoid.py 直接 np.loadtxt('obstacle.txt') 绕行。
# 🔴 实测要害：它与 mesh 真实外廓**互有大小**，所以二者必须取并集：
#   · house_1 族 mesh 14.56×12.93（且中心偏 5.63 m）vs 官方 16×12 / 18×18
#     ⇒ 官方 hous_1_67 / _clone 那两栋 mesh 明显更小（用 mesh 会欠保护）
#   · house_2 族 mesh 17.15×12.68 vs 官方 11×9
#     · gas_station mesh 17.52×25.54 vs 官方 20×15
#     ⇒ 这两族 mesh 更小？不 —— 是官方更"瘦"，用 mesh 反而更保守
OFFICIAL_NAME_ORDER = (
    'house_1_146', 'house_3_156', 'house_3_157', 'house_3_158', 'gas_station_73',
    'fast_food_93', 'house_1_66', 'house_1_67', 'house_1_146_clone',
    'house_2_71', 'house_2_125', 'house_2_126', 'house_3_68',
)
OFFICIAL_SIZE_BOX = {
    'house_1_146':       (16.0, 12.0),
    'house_3_156':       (5.0, 17.0),
    'house_3_157':       (17.0, 5.0),
    'house_3_158':       (5.0, 17.0),
    'gas_station_73':    (20.0, 15.0),
    'fast_food_93':      (30.0, 25.0),
    'house_1_66':        (16.0, 12.0),
    'house_1_67':        (18.0, 18.0),
    'house_1_146_clone': (18.0, 18.0),
    'house_2_71':        (11.0, 9.0),
    'house_2_125':       (11.0, 9.0),
    'house_2_126':       (11.0, 9.0),
    'house_3_68':        (14.0, 5.0),
}
OFFICIAL_ROVER_NUM = 20
OFFICIAL_ROVER_SIZE = (2.0, 2.0)
# 官方 field：obstacle.txt 的围墙写死在 x∈{-50,130}、y=±50
FIELD_X0, FIELD_X1 = -50.0, 130.0
FIELD_Y0, FIELD_Y1 = -50.0, 50.0


def load_official_obstacle_txt(path):
    """读官方 map_generator.py 产出的 `obstacle.txt` —— **首选**障碍源。

    为什么首选：
      · 官方权威（官方 ObstacleAvoid.py 就是 np.loadtxt 这个文件绕行）；
      · 零解析歧义：不必猜 mesh、key、yaw、尺寸；
      · 内容是**全集**：4 条围墙 + 14 个固定细杆/横杆 + 13 栋建筑
        + OFFICIAL_ROVER_NUM 辆 rover，共 3600 点（实测）。

    格式：每行 `x y `（尾部空格，可能 CRLF）。
    还原精度：官方 `change_list(black_box, 1.0)` 对每个框从 min+0.5 起步、
    步长 1.0、直到 <= max ⇒ **每点以 ±0.5 m 取并集即逐点精确复原原框**
    （实测 47 个框复原误差 = 0）。故调用方按 `±(cell/2 + margin)` 填格。
    """
    pts = []
    for ln in open(path, encoding='utf-8', errors='ignore'):
        v = ln.replace('\r', '').split()
        if len(v) < 2:
            continue
        try:
            pts.append((float(v[0]), float(v[1])))
        except ValueError:
            continue
    return pts


def load_black_box(path):
    """读官方 `black_box.txt`（python 字面量，形如 [[[x0,x1],[y0,y1]], ...]）。

    比 obstacle.txt 更直接 —— 就是 47 个**世界系轴对齐矩形**真值：
      前 14 个 = 固定细杆/横杆，其后 13 个 = 建筑，最后 = 20 辆 rover。
    返回 [(x0, x1, y0, y1), ...]；解析失败返回 []。
    """
    import ast
    try:
        txt = open(path, encoding='utf-8', errors='ignore').read().strip()
        raw = ast.literal_eval(txt)
    except Exception:
        return []
    out = []
    for b in raw:
        try:
            (x0, x1), (y0, y1) = b[0], b[1]
            out.append((float(x0), float(x1), float(y0), float(y1)))
        except Exception:
            continue
    return out


def boxes_from_black_box(path):
    """官方 `black_box.txt` -> astar_plan 的 6 元组建筑列表（**推荐主路径**）。

    为什么这条最干净（09-29 实测，四条坑一次绕开）：
      ① 它是**世界系轴对齐矩形真值**，不必猜 mesh，不必按 yaw 换长短边；
      ② mesh 与官方 size_box 互有大小（house_2 族 mesh 17.15×12.68 vs 官方
         11×9；house_1_67/clone 官方 18×18 vs mesh 14.56×12.93）⇒ 单用任一
         族都会在一半建筑上错，而 black_box 是官方认定值；
      ③ 🔴 **官方 map_generator.py 只改 model 块的 <pose>，且改坏了**：
         实测 12/13 栋 model 块 pose 被写成同一个 (-27.5, 25.5)
         （gas_station_73 甚至没有 pose）—— 只有 <state> 块与 black_box 一致。
         所以**读 model 块必错**，读 state 才行，而 black_box 连 state 都不用读；
      ④ 它是**全集**：14 个固定细杆/横杆 + 13 栋建筑 + OFFICIAL_ROVER_NUM
         辆 rover，正是官方 ObstacleAvoid 眼里的"所有障碍"。
    """
    out = []
    for i, (x0, x1, y0, y1) in enumerate(load_black_box(path)):
        out.append(('bb_%02d' % i, (x0 + x1) / 2.0, (y0 + y1) / 2.0,
                    (x1 - x0) / 2.0, (y1 - y0) / 2.0, 0.0))
    return out


def load_obstacles(path):
    """返回 (buildings, poles)。
    buildings: [(name, cx, cy, hx, hy, yaw)]  已知矩形（用 HALF 补尺寸）
    poles:     [(x, y)]                       细杆类
    """
    buildings, poles = [], []
    for line in open(path, encoding='utf-8'):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        v = line.split()
        if len(v) < 2:
            continue
        x, y = float(v[0]), float(v[1])
        poles.append((x, y))
    return buildings, poles


# ---------------------------------------------------------------- 真实几何外廓
# 🔴 为什么不能只用 HALF：
#   实测（real_aabb.py，09-27）14 个建筑实例**全部**越出 HALF 框。两条根因：
#     ① HALF 是拍脑袋的整数，与 mesh 实差 2~5 倍（gas_station 真实 17.5×25.5，
#        HALF 写的是 7.0×5.0 ⇒ y 方向单边少挡 16.5 m）；
#     ② 3D Warehouse 模型的几何中心**普遍不在原点**（最多偏 5.6 m），
#        "中心±半宽"的对称假设必然漏边（house_1_146_clone 单边漏 4.0 m）。
#   后果：A* 以为能走的路实际被墙挡住。HALF 现在只作"mesh 找不到"时的兜底。
_MODEL_ROOTS_CACHE = None
_MESH_BBOX_CACHE = {}


def _model_roots():
    """Gazebo 模型搜索路径（优先级与 env_robocup.sh 的 GAZEBO_MODEL_PATH 一致）。"""
    global _MODEL_ROOTS_CACHE
    if _MODEL_ROOTS_CACHE is not None:
        return _MODEL_ROOTS_CACHE
    roots = []
    for d in os.environ.get('GAZEBO_MODEL_PATH', '').split(':'):
        if d:
            roots.append(os.path.expanduser(d))
    roots += [
        os.path.expanduser('~/.gazebo/models'),
        os.path.expanduser('~/XTDrone/sitl_config/models'),
        '/data/PX4_Firmware/Tools/sitl_gazebo/models',
        '/data/XTDrone/sitl_config/models',
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'models'),
    ]
    seen, out = set(), []
    for r in roots:
        r = os.path.abspath(r)
        if r not in seen and os.path.isdir(r):
            seen.add(r)
            out.append(r)
    _MODEL_ROOTS_CACHE = out
    return out


def _model_sdf(name):
    for r in _model_roots():
        d = os.path.join(r, name)
        if not os.path.isdir(d):
            continue
        sdf = os.path.join(d, 'model.sdf')
        if os.path.isfile(sdf):
            return sdf
        cfg = os.path.join(d, 'model.config')
        if os.path.isfile(cfg):
            try:
                import xml.etree.ElementTree as ET
                for e in ET.parse(cfg).getroot().iter():
                    if e.tag.rsplit('}', 1)[-1] == 'sdf':
                        f = os.path.join(d, (e.text or '').strip())
                        if os.path.isfile(f):
                            return f
            except Exception:
                pass
    return None


def _resolve_mesh(uri):
    """model://X/a/b.dae -> 真实文件路径；非 model:// 原样返回。"""
    if not uri:
        return None
    if uri.startswith('model://'):
        rest = uri[len('model://'):]
        for r in _model_roots():
            p = os.path.join(r, rest)
            if os.path.isfile(p):
                return p
        return None
    return uri if os.path.isfile(uri) else None


def _dae_bbox(path):
    """COLLADA 顶点位置包围盒（**已应用 <unit meter>**）。

    ⚠ house_2.dae 的 <unit meter="0.0254"> 是英寸 —— 不乘 unit 尺寸会大 39 倍。
    ⚠ 不能用"stride==3 的 float_array"猜顶点：那会把法线/UV 一起算进去。
      正解是跟 <vertices> 里的 <input semantic="POSITION"> -> <source> 走。
    """
    import xml.etree.ElementTree as ET
    try:
        root = ET.parse(path).getroot()
    except Exception:
        return None

    def ln(t):
        return t.rsplit('}', 1)[-1]

    arrays, unit = {}, 1.0
    for e in root.iter():
        t = ln(e.tag)
        if t == 'float_array' and e.get('id'):
            try:
                arrays[e.get('id')] = [float(x) for x in (e.text or '').split()]
            except Exception:
                pass
        elif t == 'unit':
            try:
                unit = float(e.get('meter', '1'))
            except Exception:
                unit = 1.0
    src_map = {}
    for e in root.iter():
        if ln(e.tag) == 'source' and e.get('id'):
            for c in e:
                if ln(c.tag) == 'float_array' and c.get('id') in arrays:
                    src_map[e.get('id')] = c.get('id')
    pos = []
    for v in root.iter():
        if ln(v.tag) == 'vertices':
            for i in v:
                if ln(i.tag) == 'input' and i.get('semantic') == 'POSITION':
                    s = (i.get('source') or '').lstrip('#')
                    pos.append(src_map.get(s, s))
    if not pos:
        return None
    mn, mx = [1e30] * 3, [-1e30] * 3
    n = 0
    for s in pos:
        pts = arrays.get(s)
        if not pts:
            continue
        n += len(pts) // 3
        for i in range(len(pts) // 3):
            for k in range(3):
                val = pts[3 * i + k] * unit
                if val < mn[k]:
                    mn[k] = val
                if val > mx[k]:
                    mx[k] = val
    return (mn, mx) if n else None


def _mesh_collision_bbox(key):
    """模型**局部系**下的 collision 包围盒（已含 <scale>，未含 pose）。"""
    if key in _MESH_BBOX_CACHE:
        return _MESH_BBOX_CACHE[key]
    res = None
    sdf = _model_sdf(key)
    if sdf:
        import xml.etree.ElementTree as ET
        try:
            root = ET.parse(sdf).getroot()
        except Exception:
            root = None
        if root is not None:
            def ln(t):
                return t.rsplit('}', 1)[-1]

            mn, mx = [1e30] * 3, [-1e30] * 3
            for col in root.iter():
                if ln(col.tag) != 'collision':
                    continue
                for geo in col:
                    if ln(geo.tag) != 'geometry':
                        continue
                    for g in geo:
                        t = ln(g.tag)
                        sc = [1.0, 1.0, 1.0]
                        txt = {}
                        for c in g:
                            txt[ln(c.tag)] = (c.text or '').strip()
                        if txt.get('scale'):
                            v = [float(x) for x in txt['scale'].split()]
                            if len(v) >= 3:
                                sc = v[:3]
                        if t == 'mesh':
                            bb = _dae_bbox(_resolve_mesh(txt.get('uri', '')))
                            if not bb:
                                continue
                            for k in range(3):
                                lo, hi = bb[0][k] * sc[k], bb[1][k] * sc[k]
                                if lo < mn[k]:
                                    mn[k] = lo
                                if hi > mx[k]:
                                    mx[k] = hi
                        elif t == 'box':
                            try:
                                sz = [float(x) for x in
                                      (txt.get('size') or '0 0 0').split()][:3]
                            except Exception:
                                sz = [0.0, 0.0, 0.0]
                            while len(sz) < 3:
                                sz.append(0.0)
                            for k in range(3):
                                if -sz[k] / 2.0 < mn[k]:
                                    mn[k] = -sz[k] / 2.0
                                if sz[k] / 2.0 > mx[k]:
                                    mx[k] = sz[k] / 2.0
                        elif t == 'cylinder':
                            try:
                                r = float(txt.get('radius') or 0.0)
                                l = float(txt.get('length') or 0.0)
                            except Exception:
                                r = l = 0.0
                            mn[0] = min(mn[0], -r)
                            mx[0] = max(mx[0], r)
                            mn[1] = min(mn[1], -r)
                            mx[1] = max(mx[1], r)
                            mn[2] = min(mn[2], -l / 2.0)
                            mx[2] = max(mx[2], l / 2.0)
            if mn[0] < 1e29:
                res = (mn, mx)
    _MESH_BBOX_CACHE[key] = res
    return res


def _real_world_aabb(key, pose):
    """由 mesh collision + pose(yaw) 求**世界系轴对齐** AABB。

    返回 (x0, x1, y0, y1)；mesh 找不到时返回 None（调用方退回 HALF）。
    """
    bb = _mesh_collision_bbox(key)
    if bb is None:
        return None
    lo, hi = bb
    x, y = pose[0], pose[1]
    yaw = pose[5] if len(pose) >= 6 else 0.0
    c, s = math.cos(yaw), math.sin(yaw)
    xs, ys = [], []
    for cx in (lo[0], hi[0]):
        for cy in (lo[1], hi[1]):
            xs.append(c * cx - s * cy + x)
            ys.append(s * cx + c * cy + y)
    return min(xs), max(xs), min(ys), max(ys)


def buildings_from_world(world_path, use_mesh=None):
    """从 world 的 `<state>` 取建筑矩形（**兜底路径** —— 优先 black_box.txt）。

    三条实测教训（09-29，均已在 VM/官方生成器上复现）：
    🔴 ① **位置只能读 `<state>`**。官方 map_generator.py 只改 model 块的
       `<pose>`，而且改坏了：实测 12/13 栋的 model 块 pose 被写成同一个
       (-27.5, 25.5)，gas_station_73 连 pose 都没有；只有 `<state>` 与官方
       `black_box.txt` 真值逐栋一致。⇒ 读 model 块会得出"所有楼堆在一处"。
    🔴 ② **外廓取「官方 size_box ∪ mesh 世界 AABB」并集**。VM 实测 mesh
       局部尺寸：house_1 14.56×12.93（中心还偏 5.63 m）、house_2 17.15×12.68、
       house_3 12.64×4.80、gas_station 17.52×25.54、fast_food 25.16×16.58，
       与官方 size_box（16×12 / 11×9 / 5×17 / 20×15 / 30×25）**互有大小**：
       只用 mesh ⇒ house_1_67 / _clone（官方 18×18）欠保护；只用官方 ⇒
       house_2、gas_station 少挡 5~10 m。⇒ 并集，宁大勿小。
    🔴 ③ **yaw 不是 0**：8/13 栋带 ±90°/180°（base/base1/base2/base3 四套
       模板完全一致）⇒ HALF 分支里的"±90° 换长短边"是活代码。
    """
    import xml.etree.ElementTree as ET
    if use_mesh is None:
        use_mesh = os.environ.get('ASTAR_NO_MESH', '').strip().lower() \
            not in ('1', 'true', 'yes', 'on')
    world = ET.parse(world_path).getroot().find('world')
    state = world.find('state')
    if state is None:
        return []
    out = []
    for m in state.findall('model'):
        nm = m.get('name') or ''
        key = next((k for k in BLOCK_KEYS if nm.lower().startswith(k.lower())), None)
        if key is None:
            continue
        p = m.find('pose')
        if p is None or not p.text:
            continue
        v = p.text.split()
        cx, cy = float(v[0]), float(v[1])
        yaw = float(v[5]) if len(v) >= 6 else 0.0
        rects = []
        # ① 官方 size_box 真值（仅 13 栋建筑有）
        sz = OFFICIAL_SIZE_BOX.get(nm)
        if sz:
            rects.append((cx - sz[0] / 2.0, cx + sz[0] / 2.0,
                          cy - sz[1] / 2.0, cy + sz[1] / 2.0))
        # ② mesh 真实世界 AABB
        if use_mesh:
            bb = _real_world_aabb(key, [float(t) for t in v[:6]])
            if bb is not None:
                rects.append(bb)
        # ③ 两者都没有 ⇒ HALF 兜底（yaw ±90° 时换长短边）
        if not rects:
            hx, hy = HALF[key]
            if abs(abs(yaw) - math.pi / 2) < 0.4:
                hx, hy = hy, hx
            rects.append((cx - hx, cx + hx, cy - hy, cy + hy))
        x0 = min(r[0] for r in rects)
        x1 = max(r[1] for r in rects)
        y0 = min(r[2] for r in rects)
        y1 = max(r[3] for r in rects)
        out.append((nm, (x0 + x1) / 2.0, (y0 + y1) / 2.0,
                    (x1 - x0) / 2.0, (y1 - y0) / 2.0, 0.0))
    return out


def poles_from_world(world_path):
    """从 world 的 <state> 提取细杆类障碍中心 [(x, y), ...]。

    与 buildings_from_world 同款解析：真实位姿在 <state>/<model>/<pose>。
    随机地图每次尝试重生成 world 文件，细杆位置虽随模板不变，
    但必须从当次文件解析，不能沿用历史 obstacles_*.txt。
    """
    import xml.etree.ElementTree as ET
    world = ET.parse(world_path).getroot().find('world')
    state = world.find('state')
    if state is None:
        return []
    out = []
    for m in state.findall('model'):
        nm = (m.get('name') or '').lower()
        if not nm.startswith(POLE_KEYS):
            continue
        p = m.find('pose')
        if p is None or not p.text:
            continue
        v = p.text.split()
        out.append((float(v[0]), float(v[1])))
    return out


class Grid(object):
    def __init__(self, buildings, poles, margin=2.0, pole_r=1.0, cell=CELL,
                 points=None):
        self.cell = cell
        self.nx = int((X1 - X0) / cell) + 1
        self.ny = int((Y1 - Y0) / cell) + 1
        self.blocked = bytearray(self.nx * self.ny)   # 含 margin —— A* 规划用
        # 🔴 09-29 新增：`core` = **障碍本体**（不含 margin）—— 只给 verify 用。
        #   没有它的话，"官方 obstacle.txt 点阵"路径下 buildings 为空 ⇒
        #   point_hits_building 恒不命中 ⇒ 验收**假绿**（这是最危险的失败模式）。
        self.core = bytearray(self.nx * self.ny)
        self.buildings = buildings
        self.poles = poles
        self.points = list(points or [])
        # 建筑：矩形外扩 margin（同时登记本体）
        for (nm, cx, cy, hx, hy, yaw) in buildings:
            self._fill_rect(cx - hx - margin, cy - hy - margin,
                            cx + hx + margin, cy + hy + margin)
            self._fill_rect(cx - hx, cy - hy, cx + hx, cy + hy, core=True)
        # 细杆：本体登记 + **外扩 margin** 阻塞
        # 🔴 09-29 修：旧版两次 `_fill_rect` **参数完全相同** ⇒ `blocked == core`
        #   ⇒ 细杆**零安全余量**（建筑与官方点阵都加了 margin，唯独这里没有）。
        #   实测后果：6/6 随机图细杆净空不达标，全局最小 **0.826 m**，
        #   只比 `CLEARANCE=0.80` 高 2.6 cm；同期建筑净空 2.24~2.50 m 全达标。
        self.pole_r = pole_r
        for (px, py) in poles:
            self._fill_rect(px - pole_r, py - pole_r, px + pole_r, py + pole_r,
                            core=True)
            self._fill_rect(px - pole_r - margin, py - pole_r - margin,
                            px + pole_r + margin, py + pole_r + margin)
        # 官方 obstacle.txt 点阵：每点 ±cell/2 逐点倒回官方框（误差 0），
        # 再整体外扩 margin 给 A*。这是官方唯一认可的障碍定义。
        h = cell / 2.0
        for (px, py) in self.points:
            self._fill_rect(px - h, py - h, px + h, py + h, core=True)
            self._fill_rect(px - h - margin, py - h - margin,
                            px + h + margin, py + h + margin)

    def _fill_rect(self, x0, y0, x1, y1, core=False):
        buf = self.core if core else self.blocked
        i0 = max(0, int((x0 - X0) / self.cell))
        i1 = min(self.nx - 1, int((x1 - X0) / self.cell))
        j0 = max(0, int((y0 - Y0) / self.cell))
        j1 = min(self.ny - 1, int((y1 - Y0) / self.cell))
        for j in range(j0, j1 + 1):
            base = j * self.nx
            for i in range(i0, i1 + 1):
                buf[base + i] = 1

    def to_idx(self, x, y):
        i = int(round((x - X0) / self.cell))
        j = int(round((y - Y0) / self.cell))
        return max(0, min(self.nx - 1, i)), max(0, min(self.ny - 1, j))

    def to_xy(self, i, j):
        return X0 + i * self.cell, Y0 + j * self.cell

    def is_free(self, i, j):
        return self.blocked[j * self.nx + i] == 0

    def nearest_free(self, x, y, radius=25):
        i, j = self.to_idx(x, y)
        if self.is_free(i, j):
            return i, j
        best = None
        bestd = 1e18
        for dj in range(-radius, radius + 1):
            for di in range(-radius, radius + 1):
                ii, jj = i + di, j + dj
                if 0 <= ii < self.nx and 0 <= jj < self.ny and self.is_free(ii, jj):
                    d = di * di + dj * dj
                    if d < bestd:
                        bestd, best = d, (ii, jj)
        return best

    def seg_clear(self, a, b, step=0.25):
        """线段 a->b 是否全程可行（用连续采样，比栅格视线更准）。

        🔴 09-29 修：step 由 **0.5 → 0.25**，与 `verify()` 的采样步长统一。
          旧步长下长线段**斜擦**膨胀区/悬垂角落时会被跳过，实测铁证
          （r2 第 6 组段1 len=35.51m）：`seg_clear(0.5)=True` 而 0.25 细扫
          =False ⇒ 平滑层认为能走、实际穿过 blocked。
        """
        d = math.dist(a, b)
        n = max(2, int(d / step) + 1)
        for k in range(n + 1):
            t = k / float(n)
            i, j = self.to_idx(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            if not self.is_free(i, j):
                return False
        return True

    def point_hits_building(self, x, y, shrink=0.0):
        """点是否落在某个建筑矩形内（用于最终验收，不经栅格）。"""
        for (nm, cx, cy, hx, hy, yaw) in self.buildings:
            if abs(x - cx) <= hx - shrink and abs(y - cy) <= hy - shrink:
                return nm
        return None

    def point_hits_core(self, x, y):
        """点是否落在**障碍本体**内（含官方点阵；**不含** margin）。

        verify 必须用它而不是只看 buildings —— 官方 obstacle.txt 路径下
        buildings 为空，只看矩形会得到"0 穿墙"的假绿。

        🔴 09-29 修：改为**几何精确判定**，不再查 `core` 栅格。
          原因：`_fill_rect` 用 `int()` 截断 ⇒ 栅格相对真实矩形**外扩实测
          0.51 m**（最坏 1 格 = 1 m）⇒ 每次跑都会报"穿墙"假阳性。
          （注意：**不要**用"腐蚀栅格"来消假阳性 —— 那会把细杆 margin
          这类真缺陷一起藏掉，等于把安全阀拧松。）
        """
        for (_nm, cx, cy, hx, hy, _yaw) in self.buildings:
            if abs(x - cx) <= hx and abs(y - cy) <= hy:
                return True
        pr = getattr(self, 'pole_r', 1.0)
        for (px, py) in self.poles:
            if math.hypot(x - px, y - py) <= pr:
                return True
        h = self.cell / 2.0
        for (px, py) in self.points:
            if abs(x - px) <= h and abs(y - py) <= h:
                return True
        return False


def astar(g, start, goal):
    s = g.nearest_free(*start)
    t = g.nearest_free(*goal)
    if s is None or t is None:
        raise RuntimeError('起终点附近找不到可行点')
    sx, sy = s
    tx, ty = t
    if (sx, sy) == (tx, ty):
        return [start, goal]
    nb = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
          (1, 1, 1.41421356), (1, -1, 1.41421356),
          (-1, 1, 1.41421356), (-1, -1, 1.41421356))
    INF = float('inf')
    W = g.nx * g.ny
    gscore = [INF] * W
    came = [-1] * W
    start_id = sy * g.nx + sx
    goal_id = ty * g.nx + tx
    gscore[start_id] = 0.0

    def h(i, j):
        return math.hypot(i - tx, j - ty)

    pq = [(h(sx, sy), 0.0, start_id)]
    closed = bytearray(W)
    while pq:
        _, gc, cur = heapq.heappop(pq)
        if closed[cur]:
            continue
        closed[cur] = 1
        if cur == goal_id:
            break
        ci, cj = cur % g.nx, cur // g.nx
        for (di, dj, cost) in nb:
            ni, nj = ci + di, cj + dj
            if ni < 0 or nj < 0 or ni >= g.nx or nj >= g.ny:
                continue
            nid = nj * g.nx + ni
            if closed[nid] or not g.is_free(ni, nj):
                continue
            # 斜走时不允许"切角"
            if di and dj:
                if not g.is_free(ci + di, cj) or not g.is_free(ci, cj + dj):
                    continue
            ng = gc + cost
            if ng < gscore[nid] - 1e-9:
                gscore[nid] = ng
                came[nid] = cur
                heapq.heappush(pq, (ng + h(ni, nj), ng, nid))
    if came[goal_id] < 0 and goal_id != start_id:
        raise RuntimeError('A* 无解')
    # 回溯
    path = []
    cur = goal_id
    guard = 0
    while cur != -1 and guard < W:
        ci, cj = cur % g.nx, cur // g.nx
        path.append(g.to_xy(ci, cj))
        cur = came[cur]
        guard += 1
    path.reverse()
    # 起点/终点若落在障碍矩形内（典型情况：目标点写到了建筑中心），
    # 必须换成吸附到的最近可行点，否则首/末段会直接穿过建筑。
    # 🔴 随机地图新增（09-29）：起点/终点落在"膨胀带"内（距建筑 < margin
    # 但不在建筑里）时同样必须吸附 —— 否则把 path[0] 改写回真实起点后，
    # 首段从膨胀带内斜插到吸附点，会切掉建筑真实矩形的角
    # （random_map_check 实测：起点距 house_3_156 仅 0.4 m 时
    #   首段穿真实矩形，verify 0.25 m 采样抓到 (-31.0,-22.7) 在建筑内）。
    # 判据：改写后的首/末段必须仍满足栅格视线；不满足就保留吸附点。
    if len(path) > 1:
        start_ok = (g.point_hits_building(start[0], start[1]) is None
                    and g.seg_clear(tuple(start), path[1]))
        goal_ok = (g.point_hits_building(goal[0], goal[1]) is None
                   and g.seg_clear(path[-2], tuple(goal)))
    else:
        start_ok = goal_ok = False
    path[0] = tuple(start) if start_ok else g.to_xy(sx, sy)
    path[-1] = tuple(goal) if goal_ok else g.to_xy(tx, ty)
    return path


def smooth(g, path):
    """视线剪枝：能直达就跳过中间点。"""
    if len(path) <= 2:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if g.seg_clear(path[i], path[j]):
                break
            j -= 1
        out.append(path[j])
        i = j
    return out


def plan(buildings, poles, start, goal, margin=2.0, points=None):
    g = Grid(buildings, poles, margin=margin, points=points)
    raw = astar(g, start, goal)
    sp = smooth(g, raw)
    return g, raw, sp


def verify(g, path):
    """返回 (穿墙段数, 命中列表)

    命中判据 = 落在 buildings 矩形内 **或** 落在 core 栅格（官方点阵/细杆）内。
    """
    bad = []
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        d = math.dist(a, b)
        n = max(2, int(d / 0.25) + 1)
        for k in range(n + 1):
            t = k / float(n)
            x = a[0] + t * (b[0] - a[0])
            y = a[1] + t * (b[1] - a[1])
            nm = g.point_hits_building(x, y)
            if nm is None and g.point_hits_core(x, y):
                nm = 'core'
            if nm:
                bad.append((i, nm, round(x, 1), round(y, 1)))
                break
    return len(bad), bad


def draw(g, paths, labels, out_png, start=None, goal=None):
    from PIL import Image, ImageDraw
    SC = 5  # 像素/米
    W = int((X1 - X0) * SC)
    H = int((Y1 - Y0) * SC)
    im = Image.new('RGB', (W, H), (245, 245, 240))
    dr = ImageDraw.Draw(im)

    def P(x, y):
        return ((x - X0) * SC, (Y1 - y) * SC)

    # 栅格障碍（半透明感觉用浅灰）
    for j in range(g.ny):
        for i in range(g.nx):
            if not g.is_free(i, j):
                x, y = g.to_xy(i, j)
                x0, y0 = P(x - 0.5, y + 0.5)
                x1, y1 = P(x + 0.5, y - 0.5)
                dr.rectangle([x0, y0, x1, y1], fill=(205, 205, 205))
    # 建筑实体
    for (nm, cx, cy, hx, hy, yaw) in g.buildings:
        dr.rectangle([P(cx - hx, cy + hy), P(cx + hx, cy - hy)],
                     fill=(150, 150, 155), outline=(90, 90, 95))
        dr.text(P(cx - hx + 0.5, cy + hy - 1.5), nm[:16], fill=(30, 30, 30))
    # 细杆
    for (px, py) in g.poles:
        dr.ellipse([P(px - 0.45, py + 0.45), P(px + 0.45, py - 0.45)],
                   fill=(120, 120, 120))
    # 路径
    colors = [(220, 40, 40), (30, 110, 220), (20, 150, 60)]
    for k, path in enumerate(paths):
        c = colors[k % len(colors)]
        for i in range(len(path) - 1):
            dr.line([P(*path[i]), P(*path[i + 1])], fill=c, width=3)
        for (px, py) in path:
            dr.ellipse([P(px - 0.6, py + 0.6), P(px + 0.6, py - 0.6)], fill=c)
        if k < len(labels):
            dr.text(P(path[0][0] + 1.5, path[0][1] - 1.5), labels[k], fill=c)
    if start:
        dr.ellipse([P(start[0] - 1.2, start[1] + 1.2), P(start[0] + 1.2, start[1] - 1.2)],
                   fill=(0, 0, 0))
    if goal:
        dr.rectangle([P(goal[0] - 1.2, goal[1] + 1.2), P(goal[0] + 1.2, goal[1] - 1.2)],
                     fill=(0, 0, 0))
    im.save(out_png)
    print('  图已写出: %s  (%dx%d)' % (out_png, W, H))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('obstacles', nargs='?', default=None,
                    help='obstacles_*.txt（可选；不给则从 --world 自动提取细杆）')
    ap.add_argument('--world', default=None,
                    help='base.world / robocup.world（提供建筑矩形；'
                         '⚠ 只在拿不到官方 black_box.txt / obstacle.txt 时用）')
    ap.add_argument('--black-box', default=None, dest='black_box',
                    help='官方 black_box.txt（**首选**：47 个世界系轴对齐'
                         '矩形真值 = 14 细杆 + 13 建筑 + 20 rover）')
    ap.add_argument('--obstacle-txt', default=None, dest='obstacle_txt',
                    help='官方 obstacle.txt（点阵，含围墙/rover；次选）')
    ap.add_argument('--start', nargs=2, type=float)
    ap.add_argument('--goal', nargs=2, type=float)
    ap.add_argument('--margin', type=float, default=2.0)
    ap.add_argument('--png')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--no-mesh', action='store_true',
                    help='退回旧的 HALF 对称假设（仅用于 A/B 对比，实飞不要用）')
    ap.add_argument('--list-buildings', action='store_true',
                    help='打印建筑矩形后退出（核对 mesh 外廓用）')
    a = ap.parse_args()

    # ---- 障碍源优先级：black_box.txt > obstacle.txt > world 解析 ----
    # 官方 map_generator.py 每次生成都同时产出 black_box.txt / obstacle.txt，
    # 前两者是官方**自己认定**的障碍（ObstacleAvoid.py 读 obstacle.txt），
    # 且不依赖 mesh / key / yaw / 双份 pose，故一律优先。
    points = None
    poles = []
    if a.black_box:
        buildings = boxes_from_black_box(a.black_box)
        print('[障碍源] 官方 black_box.txt -> %d 个轴对齐矩形' % len(buildings))
        if not buildings:
            print('  ⛔ black_box.txt 解析为空，退回 world'); a.black_box = None
    if not a.black_box and a.obstacle_txt:
        points = load_official_obstacle_txt(a.obstacle_txt)
        buildings = []
        print('[障碍源] 官方 obstacle.txt -> %d 个点阵（含围墙/rover）' % len(points))
        if not points:
            print('  ⛔ obstacle.txt 解析为空，退回 world'); a.obstacle_txt = None
    if not a.black_box and not a.obstacle_txt:
        if not a.world:
            print('需要 --black-box / --obstacle-txt / --world 之一')
            sys.exit(1)
        buildings = buildings_from_world(a.world, use_mesh=not a.no_mesh)
        if a.obstacles:
            _, poles = load_obstacles(a.obstacles)
        else:
            poles = poles_from_world(a.world)
        print('[障碍源] world 解析 -> %d 栋建筑 / %d 细杆（外廓 %s）'
              % (len(buildings), len(poles),
                 'HALF(兜底)' if a.no_mesh else 'mesh∪官方真值'))
    print('建筑 %d 个 / 细杆 %d 个 / 点阵 %s   安全余量 %.1f m' %
          (len(buildings), len(poles), len(points) if points else '-', a.margin))

    if a.list_buildings:
        print('%-22s %9s %9s %8s %8s' % ('name', 'cx', 'cy', 'hx', 'hy'))
        for (nm, cx, cy, hx, hy, _y) in buildings:
            print('%-22s %9.3f %9.3f %8.3f %8.3f' % (nm, cx, cy, hx, hy))
        return

    if a.selftest:
        import random
        random.seed(7)
        g = Grid(buildings, poles, margin=a.margin, points=points)

        def free_pt():
            """在自由栅格里随机取一点（避免起点/终点本身就落在房子里）。"""
            for _ in range(4000):
                i = random.randrange(g.nx)
                j = random.randrange(g.ny)
                if g.is_free(i, j):
                    return g.to_xy(i, j)
            raise RuntimeError('栅格几乎全被占')

        ok = 0
        fail = 0
        for k in range(20):
            s = free_pt()
            t = free_pt()
            try:
                _, _, sp = plan(buildings, poles, s, t, a.margin, points=points)
                nb, bad = verify(g, sp)
                if nb == 0:
                    ok += 1
                else:
                    fail += 1
                    print('  ✗ 穿墙%d 起(%.1f,%.1f) 终(%.1f,%.1f)  %s'
                          % (nb, s[0], s[1], t[0], t[1], bad[:2]))
            except Exception as e:
                fail += 1
                print('  ! 异常 起(%.1f,%.1f) 终(%.1f,%.1f) : %s'
                      % (s[0], s[1], t[0], t[1], e))
        print()
        print('  自检 20 组：0 穿墙 %d / 失败 %d' % (ok, fail))
        return

    if not a.start or not a.goal:
        print('需要 --start 和 --goal')
        sys.exit(1)
    g, raw, sp = plan(buildings, poles, a.start, a.goal, a.margin, points=points)
    nb, bad = verify(g, sp)
    straight = math.dist(a.start, a.goal)
    L = sum(math.dist(sp[i], sp[i + 1]) for i in range(len(sp) - 1))
    print('  原始栅格路径 %d 点 -> 剪枝后 %d 点' % (len(raw), len(sp)))
    print('  长度 %.1f m (直线 %.1f m, +%.0f%%)' %
          (L, straight, (L / straight - 1) * 100 if straight else 0))
    print('  穿墙 = %d %s' % (nb, '✓' if nb == 0 else ('✗ ' + str(bad[:3]))))
    print('  航点:')
    for p in sp:
        print('    (%.1f, %.1f)' % p)
    if a.png:
        draw(g, [sp], ['path'], a.png, a.start, a.goal)


if __name__ == '__main__':
    main()
