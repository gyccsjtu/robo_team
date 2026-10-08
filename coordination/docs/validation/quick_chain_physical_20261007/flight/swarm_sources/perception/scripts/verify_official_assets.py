#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""官方资产核对：确认本仓库依赖的人物模型 / 相机 / 类别映射都是官方的、没标错。

为什么需要这个脚本
------------------
我们交付的不只是权重和代码，还依赖一批**官方资产**：

    walker/walk_0..5.dae   人物模型（官方 6 个目标的外观）
    stereo_camera          双目相机（数据集和上机都用它）
    base.world             <actor> 的 skin / init_pose

一旦这些被替换成非官方版本（例如自己造的 person_*、或者改过色的 dae），
**训练数据的外观就不再等于比赛时的外观**，模型会静默掉点。这种错误很难在
赛场上当场发现，所以做成可一键核对的脚本。

核对哪些事
----------
  1. walk_0..5.dae 各自的**实际色值**（从 dae 的 <effect> 里读，不听信文件名）
     → 反推它是什么颜色 → 对照官方 score_cal.py 的 actor_id_dict
  2. dae 是否来自 XTDrone 官方 git 跟踪目录
  3. stereo_camera 是否存在、分辨率/内参是否与感知节点一致（752x480 / fx=fy=376）
  4. base.world 里 6 个 <actor> 的 skin uri 是否就是 model://walker/walk_N.dae
  5. （可选）数据集 gt.json 里 actor 编号 ↔ color 是否与官方编号一致

用法
----
    python3 verify_official_assets.py                    # 自动探测
    python3 verify_official_assets.py --xtdrone ~/XTDrone
    python3 verify_official_assets.py --gt ~/robocup_dataset_gt_v3/gt.json
    python3 verify_official_assets.py --base-world ~/XTDrone/robocup/base.world

退出码 0 = 全部通过；1 = 有 FAIL（说明资产被换过或路径不对）。
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys

# ---------------------------------------------------------------- 官方口径

# score_cal.py 第 16 行（官方原文）
OFFICIAL_ACTOR_ID_DICT = {'green': [0], 'blue': [1], 'brown': [2], 'white': [3], 'red': [4, 5]}


def id2color():
    """actor_id -> 颜色名。"""
    m = {}
    for c, ids in OFFICIAL_ACTOR_ID_DICT.items():
        for i in ids:
            m[i] = c
    return m


# walk_N.dae 里 "sweater/jeans" 材质的 diffuse 实测值（官方原文件，作参考锚点）
KNOWN_DIFFUSE = {
    0: (0.09837106, 0.254824,   0.07500499),
    1: (0.05706184, 0.09248417, 0.3508265),
    2: (0.1969495,  0.07701492, 0.03641987),
    3: (0.6220113,  0.6209189,  0.64),
    4: (0.53706184, 0.01248417, 0.0108265),
    5: (0.53706184, 0.01248417, 0.0108265),
}

# stereo_camera/model.sdf 里 plugin 段写死的官方内参
CAM_INTRINSICS = {"Fx": 376.0, "Fy": 376.0, "Cx": 376.0, "Cy": 240.0}
CAM_SIZE = (752, 480)
CAM_BASELINE = 0.12


def classify_color(r, g, b):
    """从 diffuse 色值反推颜色名（独立于文件名，也不依赖 KNOWN_DIFFUSE）。"""
    mx, mn = max(r, g, b), min(r, g, b)
    if mx < 0.12:
        return "black"
    if mx - mn < 0.08:                       # 近中性
        return "white" if mx > 0.4 else "gray"
    if b >= r and b >= g:
        return "blue"
    if g >= r and g >= b:
        return "green"
    if r > g and r > b:
        # red 与 brown 都是 R 最大，靠 G/R 比区分：red≈0.02，brown≈0.39（差约 20 倍）
        return "red" if (g / max(r, 1e-9)) < 0.1 else "brown"
    return "?"


def banner(t):
    print()
    print("=" * 72)
    print(t)
    print("=" * 72)


def read(path):
    with open(path, encoding="utf-8", errors="ignore") as f:
        return f.read()


# ---------------------------------------------------------------- 各项检查

def check_walker(walker_dir):
    banner("1. 人物模型 walk_N.dae（外观来源）")
    if not os.path.isdir(walker_dir):
        print("FAIL  目录不存在：%s" % walker_dir)
        print("      需要 XTDrone 官方包：git clone https://gitee.com/robin_shaun/XTDrone")
        return False

    ok, idc = True, id2color()
    for i in range(6):
        p = os.path.join(walker_dir, "walk_%d.dae" % i)
        if not os.path.isfile(p):
            print("FAIL  walk_%d.dae 缺失" % i)
            ok = False
            continue

        src = read(p)
        m = re.search(r"<library_effects>(.*?)</library_effects>", src, re.S)
        diffuse = None
        if m:
            for eid, body in re.findall(r'<effect id="([^"]+)"[^>]*>(.*?)</effect>', m.group(1), re.S):
                # 只看衣服材质（sweater / jeans）—— 眼睛与皮肤是 6 个人共用的
                if "sweater" not in eid and "jeans" not in eid:
                    continue
                d = re.search(r"<diffuse>\s*<color[^>]*>\s*([^<]+?)\s*</color>", body, re.S)
                if d:
                    diffuse = tuple(float(x) for x in d.group(1).split()[:3])
                    break

        if diffuse is None:
            print("FAIL  walk_%d.dae 里没找到衣服材质色值" % i)
            ok = False
            continue

        got, want = classify_color(*diffuse), idc.get(i, "?")
        hit = (got == want)
        # 顺带与已知锚点比一下（官方文件没被改过就应当逐位接近）
        ref = KNOWN_DIFFUSE.get(i)
        drift = max(abs(a - b) for a, b in zip(diffuse, ref)) if ref else 0.0
        ok = ok and hit and drift < 0.02
        print("  %s  walk_%d.dae  官方应为 %-5s | 实测 (%.3f, %.3f, %.3f) -> %-5s  偏移 %.4f"
              % ("OK  " if (hit and drift < 0.02) else "FAIL",
                 i, want, diffuse[0], diffuse[1], diffuse[2], got, drift))
    return ok


def check_walker_official(xtdrone_root):
    banner("2. 模型是否来自官方 git 跟踪目录")
    if not xtdrone_root or not os.path.isdir(os.path.join(xtdrone_root, ".git")):
        print("SKIP  没找到 XTDrone git 仓库（用 --xtdrone 指定）")
        return True
    try:
        out = subprocess.check_output(["git", "ls-files", "sitl_config/models/"],
                                      cwd=xtdrone_root, stderr=subprocess.STDOUT).decode()
    except Exception as e:
        print("SKIP  git 查询失败：%s" % e)
        return True

    tops = {l.split("sitl_config/models/", 1)[1].split("/")[0]
            for l in out.splitlines() if l.startswith("sitl_config/models/")}
    ok = True
    for name in ("walker", "stereo_camera"):
        hit = name in tops
        ok = ok and hit
        print("  %s  %-16s %s" % ("OK  " if hit else "FAIL", name,
                                  "官方 git 跟踪" if hit else "不在官方列表里！"))
    extra = sorted(t for t in tops if t.startswith("person_"))
    if extra:
        print("  !!  官方列表里没有 person_*，但目录存在：%s" % ", ".join(extra))
        print("      这些是本队自造的手动插入模型，**不要**用它们采数据或调试。")
    return ok


def check_stereo(models_dir, perception_py=None):
    banner("3. 双目相机 stereo_camera")
    d = os.path.join(models_dir, "stereo_camera")
    sdf = os.path.join(d, "model.sdf")
    if not os.path.isfile(sdf):
        print("FAIL  找不到 %s" % sdf)
        return False
    print("  OK    模型目录存在：%s" % d)
    src = read(sdf)
    ok = True

    for cam in ("left", "right"):
        hit = ('<camera name="%s">' % cam) in src
        ok = ok and hit
        print("  %s  <camera name=\"%s\">" % ("OK  " if hit else "FAIL", cam))

    m = re.search(r"<image>\s*<width>(\d+)</width>\s*<height>(\d+)</height>", src)
    if m:
        w, h = int(m.group(1)), int(m.group(2))
        hit = (w, h) == CAM_SIZE
        ok = ok and hit
        print("  %s  分辨率 %dx%d（期望 %dx%d）" % ("OK  " if hit else "FAIL", w, h, *CAM_SIZE))
    else:
        print("  FAIL  sdf 里没有 <image><width>/<height>")
        ok = False

    hf = re.search(r"<horizontal_fov>([\d.]+)</horizontal_fov>", src)
    if hf:
        deg = math.degrees(float(hf.group(1)))
        hit = abs(deg - 90.0) < 0.5
        ok = ok and hit
        print("  %s  horizontal_fov = %.1f deg（期望 90）" % ("OK  " if hit else "FAIL", deg))

    # 真正的内参锚点：plugin 里写死的 Fx/Fy/Cx/Cy 与基线
    for tag, want in CAM_INTRINSICS.items():
        mm = re.search(r"<%s>([-\d.eE+]+)</%s>" % (tag, tag), src)
        got = float(mm.group(1)) if mm else None
        hit = (got is not None and abs(got - want) < 1e-6)
        ok = ok and hit
        print("  %s  %-12s = %s（期望 %g）"
              % ("OK  " if hit else "FAIL", tag, "-" if got is None else ("%g" % got), want))

    mm = re.search(r"<hackBaseline>([-\d.eE+]+)</hackBaseline>", src)
    if mm:
        b = float(mm.group(1))
        hit = abs(b - CAM_BASELINE) < 1e-6
        ok = ok and hit
        print("  %s  hackBaseline = %g m（期望 %g，左右目间距）"
              % ("OK  " if hit else "FAIL", b, CAM_BASELINE))

    # 与感知节点对标：数据集标定与上机推理必须用同一套内参
    if perception_py and os.path.isfile(perception_py):
        p = read(perception_py)
        # 只匹配代码行：注释里常写着「旧值（cgo3 单目）：FX=FY=205.47」之类，会污染匹配
        p = "\n".join(l for l in p.splitlines() if not l.lstrip().startswith("#"))
        found = {}
        for pat, keys in ((r"FX\s*=\s*FY\s*=\s*([\d.]+)", ("FX", "FY")),
                          (r"CX\s*,\s*CY\s*=\s*([\d.]+)\s*,\s*([\d.]+)", ("CX", "CY")),
                          (r"IMG_W\s*,\s*IMG_H\s*=\s*(\d+)\s*,\s*(\d+)", ("IMG_W", "IMG_H"))):
            mm = re.search(pat, p)
            if mm:
                for k, g in zip(keys, mm.groups()):
                    found[k] = float(g)
        if found:
            print("  --   与感知节点 %s 对标：" % os.path.basename(perception_py))
            for k, want in (("FX", 376.0), ("FY", 376.0), ("CX", 376.0), ("CY", 240.0),
                            ("IMG_W", 752.0), ("IMG_H", 480.0)):
                if k not in found:
                    continue
                hit = abs(found[k] - want) < 1e-6
                ok = ok and hit
                print("       %s  %-6s = %g（官方 %g）"
                      % ("OK  " if hit else "FAIL", k, found[k], want))
        else:
            print("  --   在 %s 里没解析到 FX/CX 常量" % os.path.basename(perception_py))
    return ok


def check_base_world(world):
    banner("4. base.world 里 actor 的 skin")
    if not world or not os.path.isfile(world):
        print("SKIP  没给 base.world（用 --base-world 指定）")
        return True
    src = read(world)
    actors = re.findall(r'<actor name="(actor_(\d+))">(.*?)</actor>', src, re.S)
    if not actors:
        print("FAIL  没有解析到任何 <actor>")
        return False
    ok = True
    for full, idx, body in actors:
        m = re.search(r"<skin>\s*<filename>([^<]+)</filename>", body)
        uri = m.group(1).strip() if m else "(none)"
        want = "model://walker/walk_%s.dae" % idx
        hit = (uri == want)
        ok = ok and hit
        print("  %s  %-8s skin = %s" % ("OK  " if hit else "FAIL", full, uri))
    return ok


def check_gt(gt_path):
    banner("5. 数据集标注 actor 编号 ↔ 颜色")
    if not gt_path or not os.path.isfile(gt_path):
        print("SKIP  没给 gt.json（用 --gt 指定）")
        return True
    d = json.load(open(gt_path, encoding="utf-8"))
    cfg = d.get("config", {})
    print("  相机: %s  fx=%s fy=%s  %sx%s"
          % (cfg.get("camera", "?"), cfg.get("fx", "?"), cfg.get("fy", "?"),
             cfg.get("W", "?"), cfg.get("H", "?")))
    # 数据集标定的内参必须与官方相机一致
    intr_ok = (cfg.get("fx") == 376.0 and cfg.get("fy") == 376.0
               and cfg.get("cx") == 376.0 and cfg.get("cy") == 240.0)
    print("  %s  数据集内参与官方一致（fx=fy=376, cx=376, cy=240）"
          % ("OK  " if intr_ok else "FAIL"))

    pairs, objs = {}, 0
    for _k, v in d.get("images", {}).items():
        for o in v.get("objects", []):
            objs += 1
            a, c = o.get("actor"), o.get("color")
            if a is not None and c is not None:
                pairs.setdefault(a, set()).add(c)

    # gt 里的 actor 编号与官方 actor_id_dict 同为 0-based，直接严格比对
    ok = intr_ok
    idc = id2color()
    for a in sorted(pairs):
        got = "/".join(sorted(pairs[a]))
        want = idc.get(a)
        hit = (want is not None and got == want)
        ok = ok and hit
        print("  %s  数据集中 actor_%d 标注为 %-6s（官方 %s）"
              % ("OK  " if hit else "FAIL", a, got, want or "该编号不在官方表里"))
    print("  共 %d 个目标框，覆盖 %d 个 actor 编号（官方表 %d 个：%s）。"
          % (objs, len(pairs), len(idc),
             ", ".join("actor_%d=%s" % (i, idc[i]) for i in sorted(idc))))
    return ok


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="核对 RoboCup 官方资产与类别映射")
    ap.add_argument("--xtdrone", default=os.path.expanduser("~/XTDrone"),
                    help="XTDrone 根目录（默认 ~/XTDrone）")
    ap.add_argument("--base-world", default=None,
                    help="官方 base.world（默认 <xtdrone>/robocup/base.world）")
    ap.add_argument("--gt", default=None, help="数据集 gt.json（可选）")
    ap.add_argument("--models", default=None,
                    help="models 目录（默认 <xtdrone>/sitl_config/models）")
    ap.add_argument("--perception", default=None,
                    help="感知节点路径（默认自动找 ../perception_real.py）")
    args = ap.parse_args()

    xt = os.path.expanduser(args.xtdrone)
    models = os.path.expanduser(args.models) if args.models else \
        os.path.join(xt, "sitl_config", "models")
    walker = os.path.join(models, "walker")
    bw = args.base_world or os.path.join(xt, "robocup", "base.world")
    perc = args.perception or os.path.join(os.path.dirname(here), "perception_real.py")

    results = [
        ("人物模型色值 ↔ 官方映射", check_walker(walker)),
        ("模型来自官方 git 目录", check_walker_official(xt)),
        ("双目相机 stereo_camera", check_stereo(models, perc)),
        ("base.world skin 引用", check_base_world(bw)),
        ("数据集标注映射", check_gt(args.gt)),
    ]

    banner("结论")
    bad = 0
    for name, ok in results:
        print("  %s  %s" % ("PASS" if ok else "FAIL", name))
        bad += (not ok)
    print()
    if bad:
        print("有 %d 项未通过 —— 资产可能被换过、路径不对、或标注与官方口径不一致。" % bad)
        print("在改任何代码之前先把这些对齐，否则训练/比赛会静默掉点。")
    else:
        print("全部通过：人物模型、相机、skin 引用、标注映射都与官方一致。")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
