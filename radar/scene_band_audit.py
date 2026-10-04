#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene_band_audit.py —— 2D 雷达"高度盲带"场景体检器（B2 专用）

为什么需要它
------------
typhoon_h480 的 2D 雷达装在机体**顶面 +0.080 m**，机体最低碰撞点（起落架）在
机体中心 **-0.251 m**。因此：

    扫描面 = 机体高度 + 0.080
    机体下沿 = 机体高度 - 0.251
    盲带   = [机体高度 - 0.251, 机体高度 + 0.080)      宽 0.331 m

顶面落在这个区间里的物体：雷达扫描面**从它上方掠过 ⇒ 完全没有回波**
（不是被滤掉，是根本没有信号），而机体下沿**低于它的顶面 ⇒ 必撞**。

⇒ 隐形障碍在雷达里没有回波，**任何过滤器 / 速度守卫都无法拦截**。
   这不是算法问题，是"单平面 2D 传感器 + 机顶贴装"的物理盲区。
   唯一可控的变量是**巡航高度**与**已知场景先验**。

本工具把该变量变成可复核的体检：解析一个 .world，算出每个静态障碍的世界 AABB，
列出在给定巡航高度下"掉进盲带 / 差一点掉进盲带"的对象清单。

用法
----
    # 在装了 Gazebo 模型缓存的机器上（VM / WSL）跑，结论才完整：
    python scene_band_audit.py <world> --model-root ~/.gazebo/models ^
        --model-root /root/robocup_resources/models --alt-world 2.55

    # 只看显式尺寸几何（无需模型缓存，覆盖率低）：
    python scene_band_audit.py <world> --alt-world 2.55

参数
----
  --alt-world   机体中心的**世界**高度(m)。默认 2.55（≈ local 2.2 + 0.35 偏移）
  --margin      顶面低于盲带上沿不超过该值也预警（"差一点"）。默认 0.15
  --model-root  可重复。把 model://xxx/... 解析为本地文件，读 mesh 的真实包围盒

高度取自**碰撞几何**（与物理一致），不是视觉网格。
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

MOUNT_Z = 0.080
LEG_DROP = 0.251
BAND_W = MOUNT_Z + LEG_DROP

_RE_MODEL = re.compile(r"<model\s+name=['\"]([^'\"]+)['\"]\s*>")
_RE_LINK = re.compile(r"<link\s+name=['\"]([^'\"]+)['\"]\s*>")
_RE_COL = re.compile(r"<collision\s+name=['\"]([^'\"]+)['\"]\s*>")
_RE_POSE = re.compile(r"<pose[^>]*>([^<]+)</pose>")
_RE_SIZE = re.compile(r"<size>([^<]+)</size>")
_RE_RADIUS = re.compile(r"<radius>([^<]+)</radius>")
_RE_LENGTH = re.compile(r"<length>([^<]+)</length>")
_RE_URI = re.compile(r"<uri>([^<]+)</uri>")
_RE_SCALE = re.compile(r"<scale>([^<]+)</scale>")


# ------------------------------------------------------------------ 数学

def vec6(text):
    v = [float(x) for x in text.split()[:6]]
    while len(v) < 6:
        v.append(0.0)
    return v


def rot(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def xform(M, t, v):
    return [M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2] + t[i] for i in range(3)]


def matmul(A, B):
    return [[sum(A[i][k] * B[j][k] for k in range(3)) for j in range(3)] for i in range(3)]


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def absrow(M):
    return [sum(abs(M[i][j]) for j in range(3)) for i in range(3)]


def ident():
    return [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]


# -------------------------------------------------------------- COLLADA

def _m4id():
    return [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0]


def _m4mul(A, B):
    """4x4 行主序矩阵相乘 A·B。"""
    return [sum(A[4 * i + k] * B[4 * k + j] for k in range(4)) for i in range(4) for j in range(4)]


def _m4apply(M, p):
    x, y, z = p
    return (M[0] * x + M[4] * y + M[8] * z + M[12],
            M[1] * x + M[5] * y + M[9] * z + M[13],
            M[2] * x + M[6] * y + M[10] * z + M[14])


def _m4rot(axis, deg):
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    ax, ay, az = ax / n, ay / n, az / n
    a = math.radians(deg)
    c, s, t = math.cos(a), math.sin(a), 1.0 - math.cos(a)
    return [t * ax * ax + c, t * ax * ay + s * az, t * ax * az - s * ay, 0,
            t * ax * ay - s * az, t * ay * ay + c, t * ay * az + s * ax, 0,
            t * ax * az + s * ay, t * ay * az - s * ax, t * az * az + c, 0,
            0, 0, 0, 1]


def dae_bbox(path):
    """DAE 网格在**米制 Z-up** 下的 AABB。返回 (lo, hi) 或 None。

    正确处理：<unit meter=...> 单位换算；<node> 的 <matrix> 与 T/R/S 变换；
    <instance_geometry> 引用；层级嵌套。
    """
    try:
        root = ET.parse(path).getroot()
    except Exception:
        return None

    def tag(e):
        return e.tag.rsplit("}", 1)[-1]

    # ---- 单位（inch=0.0254 / cm=0.01 都要换算成米，否则高度差 100 倍！）
    unit = 1.0
    for e in root.iter():
        if tag(e) == "unit":
            try:
                unit = float(e.get("meter", "1") or "1")
            except ValueError:
                unit = 1.0
            break

    # ---- 每个 <source> 的顶点
    src_pts = {}
    for e in root.iter():
        if tag(e) != "source":
            continue
        sid = e.get("id")
        fa = None
        for ch in e:
            if tag(ch) == "float_array":
                fa = ch
                break
        if fa is None or not fa.text:
            continue
        try:
            nums = [float(x) for x in fa.text.split()]
        except ValueError:
            continue
        if len(nums) < 3 or len(nums) % 3:
            continue
        src_pts[sid] = [(nums[i], nums[i + 1], nums[i + 2]) for i in range(0, len(nums), 3)]

    # ---- geometry id -> 顶点（经 <vertices> 的 POSITION input）
    geoms = {}
    for g in root.iter():
        if tag(g) != "geometry":
            continue
        pts = []
        for verts in g.iter():
            if tag(verts) != "vertices":
                continue
            for inp in verts:
                if tag(inp) == "input" and inp.get("semantic") == "POSITION":
                    ref = (inp.get("source") or "").lstrip("#")
                    pts.extend(src_pts.get(ref, []))
        if pts:
            geoms[g.get("id")] = pts
    if not geoms:
        return None

    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    found = False

    def walk(node, M):
        nonlocal found
        Ts, Rs, Ss, Ms = [], [], [], []
        for ch in node:
            t = tag(ch)
            txt = (ch.text or "").split()
            if t == "matrix" and len(txt) == 16:
                Ms.append([float(x) for x in txt])
            elif t == "rotate" and len(txt) >= 4:
                Rs.append(_m4rot([float(x) for x in txt[:3]], float(txt[3])))
            elif t == "translate" and len(txt) >= 3:
                v = [float(x) for x in txt[:3]]
                T = _m4id()
                T[12], T[13], T[14] = v
                Ts.append(T)
            elif t == "scale" and len(txt) >= 3:
                v = [float(x) for x in txt[:3]]
                S = _m4id()
                S[0], S[5], S[10] = v
                Ss.append(S)
        # COLLADA 语义：L = T · R · S
        L = _m4id()
        for X in Ts + Rs + Ss:
            L = _m4mul(L, X)
        for X in Ms:
            L = _m4mul(L, X)
        M2 = _m4mul(M, L)
        for ch in node:
            t = tag(ch)
            if t == "instance_geometry":
                url = (ch.get("url") or "").lstrip("#")
                for p in geoms.get(url, ()):
                    q = _m4apply(M2, p)
                    for k in range(3):
                        lo[k] = min(lo[k], q[k])
                        hi[k] = max(hi[k], q[k])
                    found = True
            elif t == "node":
                walk(ch, M2)

    for e in root.iter():
        if tag(e) == "visual_scene":
            for node in e:
                if tag(node) == "node":
                    walk(node, _m4id())
            break

    if not found:
        return None
    return [x * unit for x in lo], [x * unit for x in hi]


# ------------------------------------------------------------ 模型查找

def resolve_model_path(uri, roots):
    """model://a/b/c.dae -> 本地路径"""
    if uri.startswith("model://"):
        rel = uri[len("model://"):]
    elif uri.startswith("file://"):
        rel = uri[len("file://"):].lstrip("/")
    else:
        return None
    for r in roots:
        p = os.path.join(r, *rel.split("/"))
        if os.path.isfile(p):
            return p
    return None


# ------------------------------------------------------------ 世界解析

def split_models(src, offset=0):
    """按 <model>/</model> 深度配对，返回 [(name, body, start)]。解决嵌套问题。"""
    out = []
    stack = []
    for m in re.finditer(r"<model\s+name=['\"]([^'\"]+)['\"]\s*>|</model\s*>", src):
        if m.group(1) is not None:
            stack.append((m.group(1), m.start()))
        else:
            if stack:
                name, st = stack.pop()
                if not stack:               # 只有最外层才算"顶层模型"
                    out.append((name, src[st:m.end()], st + offset))
    return out


def aabb_add(abb, center, half, M, exact=True):
    r = absrow(M)
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                p = xform(M, center, [sx * half[0], sy * half[1], sz * half[2]])
                for k in range(3):
                    abb[0][k] = min(abb[0][k], p[k])
                    abb[1][k] = max(abb[1][k], p[k])
    if not exact:
        abb[2] = False


def parse_world(path, roots):
    src = open(path, encoding="utf-8", errors="ignore").read()
    m = re.search(r"<state\b", src)
    defs = src[:m.start()] if m else src
    state = src[m.start():] if m else ""

    # state 里每个模型的世界位姿（同名取第一条）
    st = {}
    for name, body, _ in split_models(state):
        pm = _RE_POSE.search(body)
        if pm and name not in st:
            st[name] = vec6(pm.group(1))

    results, unknowns = [], {}
    for name, body, _ in split_models(defs):
        mp = st.get(name)
        if mp is None:
            pm = _RE_POSE.search(body)
            mp = vec6(pm.group(1)) if pm else [0.0] * 6
        Mw, tw = rot(mp[3:6]), mp[0:3]

        abb = [[math.inf] * 3, [-math.inf] * 3, True]
        got = False
        for _, lbody, _ in split_models(body):
            lm = _RE_POSE.search(lbody)
            lp = vec6(lm.group(1)) if lm else [0.0] * 6
            Ml, tl = rot(lp[3:6]), lp[0:3]
            M = matmul(Mw, Ml)
            base = xform(Mw, tw, tl)

            for cm in re.finditer(r"<collision\s+name=['\"]([^'\"]+)['\"]\s*>", lbody):
                cend = lbody.find("</collision>", cm.start())
                # </collision> 可能不存在（自闭合），退化为取其后 1200 字符
                cbody = lbody[cm.start(): cend if cend > 0 else cm.start() + 1500]
                cpm = _RE_POSE.search(cbody)
                cp = vec6(cpm.group(1)) if cpm else [0.0] * 6
                Mc = matmul(M, rot(cp[3:6]))
                cc = xform(M, base, cp[0:3])

                g = re.search(r"<geometry>(.*?)</geometry>", cbody, re.S)
                if not g:
                    continue
                g = g.group(1)
                if "<box>" in g:
                    sm = _RE_SIZE.search(g)
                    if not sm:
                        continue
                    h = [float(x) / 2.0 for x in sm.group(1).split()[:3]]
                    while len(h) < 3:
                        h.append(0.0)
                    aabb_add(abb, cc, h, Mc)
                    got = True
                elif "<cylinder>" in g:
                    rm, lm2 = _RE_RADIUS.search(g), _RE_LENGTH.search(g)
                    if not (rm and lm2):
                        continue
                    rr = float(rm.group(1).split()[0])
                    ll = float(lm2.group(1).split()[0])
                    aabb_add(abb, cc, [rr, rr, ll / 2.0], Mc)
                    got = True
                elif "<sphere>" in g:
                    rm = _RE_RADIUS.search(g)
                    if not rm:
                        continue
                    rr = float(rm.group(1).split()[0])
                    aabb_add(abb, cc, [rr, rr, rr], Mc)
                    got = True
                elif "<plane>" in g:
                    continue
                elif "<mesh>" in g:
                    um = _RE_URI.search(g)
                    sm = _RE_SCALE.search(g)
                    sc = [1.0, 1.0, 1.0]
                    if sm:
                        sc = [float(x) for x in sm.group(1).split()[:3]]
                        while len(sc) < 3:
                            sc.append(1.0)
                    bl = None
                    if um:
                        p = resolve_model_path(um.group(1), roots)
                        if p:
                            bl = dae_bbox(p)
                    if bl:
                        lo, hi = bl
                        c = [(lo[k] + hi[k]) / 2.0 * sc[k] for k in range(3)]
                        h = [(hi[k] - lo[k]) / 2.0 * sc[k] for k in range(3)]
                        aabb_add(abb, xform(Mc, cc, c), h, Mc, exact=False)
                        got = True
                    else:
                        unknowns.setdefault(name, um.group(1) if um else "?")
                        for k in range(3):
                            abb[0][k] = min(abb[0][k], cc[k])
                            abb[1][k] = max(abb[1][k], cc[k])
        if got:
            results.append((name, abb))
    return results, unknowns


# ------------------------------------------------------------------ 主

def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("world")
    ap.add_argument("--alt-world", type=float, default=2.55)
    ap.add_argument("--margin", type=float, default=0.15)
    ap.add_argument("--model-root", action="append", default=[])
    a = ap.parse_args(argv)

    lo, hi = a.alt_world - LEG_DROP, a.alt_world + MOUNT_Z
    print(f"# 2D 雷达高度盲带体检 — `{os.path.basename(a.world)}`\n")
    print(f"| 项 | 值 |\n|---|---|")
    print(f"| 机体中心世界高度 | {a.alt_world:.3f} m |")
    print(f"| 机体下沿（最低碰撞点） | {lo:.3f} m |")
    print(f"| 雷达扫描面 | {hi:.3f} m |")
    print(f"| **盲带** | **[{lo:.3f}, {hi:.3f})  宽 {BAND_W:.3f} m** |\n")

    res, unk = parse_world(a.world, a.model_root)
    print(f"- 顶层模型解析成功 {len(res)} 个；mesh 未能定高 {len(unk)} 个\n")

    invisible, near, tall, low = [], [], [], []
    for name, abb in res:
        zlo, zhi = abb[0][2], abb[1][2]
        cxy = (0.5 * (abb[0][0] + abb[1][0]), 0.5 * (abb[0][1] + abb[1][1]))
        rec = (name, zlo, zhi, cxy)
        if zlo >= hi:
            pass                                   # 完全在扫描面之上，飞不到
        elif zhi <= lo:
            if lo - zhi <= a.margin:
                near.append(rec)
            else:
                low.append(rec)
        elif zhi < hi:                             # 顶面落在盲带内
            invisible.append(rec)
        else:
            tall.append(rec)                       # 跨过盲带，正常可见

    def table(rows, cols, fmt, limit=None, sort=lambda r: r[2]):
        rows = sorted(rows, key=sort)
        head = "| " + " | ".join(c[0] for c in cols) + " |"
        sep = "|" + "---|" * len(cols)
        out = [head, sep]
        for r in (rows[:limit] if limit else rows):
            out.append("| " + " | ".join(c[1](r) for c in cols) + " |")
        return "\n".join(out) + "\n"

    print("## 1. 致命 — 顶面落在盲带内（雷达完全看不见，机体下沿会撞）\n")
    if invisible:
        print(table(invisible, [
            ("模型", lambda r: f"`{r[0]}`"),
            ("底面 z", lambda r: f"{r[1]:+.3f}"),
            ("顶面 z", lambda r: f"{r[2]:+.3f}"),
            ("中心 XY", lambda r: f"({r[3][0]:.1f}, {r[3][1]:.1f})"),
        ], None))
    else:
        print("（无）\n")

    print(f"## 2. 预警 — 顶面在盲带下沿之下 {a.margin:.2f} m 内（姿态/高度误差即命中）\n")
    if near:
        print(table(near, [
            ("模型", lambda r: f"`{r[0]}`"),
            ("顶面 z", lambda r: f"{r[2]:+.3f}"),
            ("净空", lambda r: f"{lo - r[2]:+.3f}"),
            ("中心 XY", lambda r: f"({r[3][0]:.1f}, {r[3][1]:.1f})"),
        ], None, sort=lambda r: -r[2]))
    else:
        print("（无）\n")

    print(f"## 3. 正常可见（顶部跨过扫描面）— 共 {len(tall)} 个，列前 20\n")
    print(table(tall, [
        ("模型", lambda r: f"`{r[0]}`"),
        ("顶面 z", lambda r: f"{r[2]:+.3f}"),
        ("中心 XY", lambda r: f"({r[3][0]:.1f}, {r[3][1]:.1f})"),
    ], None, limit=20)[:0] or table(tall, [
        ("模型", lambda r: f"`{r[0]}`"),
        ("顶面 z", lambda r: f"{r[2]:+.3f}"),
        ("中心 XY", lambda r: f"({r[3][0]:.1f}, {r[3][1]:.1f})"),
    ], None, limit=20))

    print(f"## 4. 低矮可越（顶面远离盲带）— 共 {len(low)} 个，列前 15\n")
    print(table(low, [
        ("模型", lambda r: f"`{r[0]}`"),
        ("顶面 z", lambda r: f"{r[2]:+.3f}"),
        ("净空", lambda r: f"{lo - r[2]:+.3f}"),
    ], None, limit=15, sort=lambda r: -r[2]))

    if unk:
        print(f"## 5. 未能定高（mesh 未找到）— 共 {len(unk)} 个\n")
        print("在装了 Gazebo 模型缓存的机器上加 `--model-root` 重跑即可定高。\n")
        for n, u in sorted(unk.items())[:60]:
            print(f"- `{n}`  ← {u}")
        print()

    print("## 结论\n")
    if invisible:
        print(f"**当前高度 {a.alt_world:.2f} m 有 {len(invisible)} 个隐形障碍**："
              + "、".join(f"`{r[0]}`" for r in invisible[:12]))
        print("\n⇒ 只能靠**换巡航高度**或**水平绕行**，雷达无解。")
    else:
        print(f"当前高度 {a.alt_world:.2f} m **无顶面落入盲带**。")
        if near:
            print(f"但 {len(near)} 个模型顶面距盲带下沿不足 {a.margin:.2f} m，"
                  "高度漂移/姿态误差会命中。")
        if unk:
            print(f"另有 {len(unk)} 个 mesh 未定高，**结论尚不完整**，请在 VM 内补跑。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
