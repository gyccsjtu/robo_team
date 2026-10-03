#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
几何可分性诊断 —— 回答一个问题：**真人和场景误检（招牌 / 建筑条 / 墙）在几何上能分开吗？**

原理（纯物理，不调参）：
  相机高度 pz、水平前视时，画面第 v 行对应的地面水平距离
      D = pz * fy / (v - cy)
  人站在地面上、身高 H，则框的像素高 h_px 与 H 的关系
      H = h_px * D / fy = h_px * pz / (v_bottom - cy)
  也就是说：**把框的像素高按透视归一化后，真人应落在 ~1.7 m 附近**，
  而招牌 / 建筑色条要么明显更高（几米），要么明显更矮。

  注意这个式子与 fy 无关（水平和垂直共用一个 fy），所以不依赖内参精度，
  只依赖相机高度 pz —— 而 pz 是我们自己控制的飞行高度，可靠。

输出：每个检测框的 (implied_H, aspect=h/w, conf, cls)，按类别统计分布，
用于定 H_MIN / H_MAX / AR 门限。
"""
import glob
import os
import sys

import cv2

FRAMES = os.environ.get("DG_FRAMES", os.path.expanduser("~/robocup_real/_m2_frames"))
WEIGHTS = os.environ.get("PR_WEIGHTS", os.path.expanduser("~/robocup_runs/robocup5_v6/weights/best.pt"))
CAM_H = float(os.environ.get("DG_CAM_H", "5.5"))     # 存帧时的飞行高度
CY = 180.50
CONF = float(os.environ.get("DG_CONF", "0.25"))      # 诊断放低，把候选全放进来
ROI_TOP = 58
CLASSES = ["green", "blue", "brown", "white", "red", "red"]


def main():
    from ultralytics import YOLO
    model = YOLO(WEIGHTS)

    files = sorted(glob.glob(os.path.join(FRAMES, "*.jpg")))
    if not files:
        print("[dg] 没有帧: %s" % FRAMES)
        return
    print("[dg] %d 帧  相机高=%.1fm  conf=%.2f" % (len(files), CAM_H, CONF))
    print("[dg] %-6s %5s %6s %7s %6s %7s  %s" % ("cls", "conf", "h_px", "impl_H", "h/w", "D(m)", "frame"))

    by_cls = {}
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        res = model(img, conf=CONF, verbose=False)[0]
        for b in res.boxes:
            cid = int(b.cls)
            conf = float(b.conf)
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
            if y2 < ROI_TOP:
                continue
            h = y2 - y1
            w = max(1e-3, x2 - x1)
            vbot = y2
            dv = vbot - CY
            if dv <= 1.0:
                continue
            D = CAM_H * 205.47 / dv
            impl_H = h * CAM_H / dv
            name = CLASSES[cid] if cid < len(CLASSES) else "?"
            by_cls.setdefault(name, []).append((impl_H, h / w, conf, D))
            print("[dg] %-6s %5.2f %6.1f %7.2f %6.2f %7.1f  %s"
                  % (name, conf, h, impl_H, h / w, D, os.path.basename(f)))

    print()
    print("[dg] ===== 按类别汇总（impl_H = 归一化身高，真人应 ≈1.7 m）=====")
    print("[dg] %-6s %5s  %8s %8s %8s  %8s %8s" % ("cls", "n", "H_min", "H_med", "H_max", "AR_med", "conf_med"))
    for name, rows in sorted(by_cls.items()):
        hs = sorted(r[0] for r in rows)
        ars = sorted(r[1] for r in rows)
        cs = sorted(r[2] for r in rows)
        n = len(hs)
        print("[dg] %-6s %5d  %8.2f %8.2f %8.2f  %8.2f %8.2f"
              % (name, n, hs[0], hs[n // 2], hs[-1], ars[n // 2], cs[n // 2]))

    # 可分性：真人区间 [1.0, 2.4] 内外的分布
    print()
    lo, hi = 1.0, 2.4
    for name, rows in sorted(by_cls.items()):
        inside = sum(1 for r in rows if lo <= r[0] <= hi)
        print("[dg] %-6s 人体区间[%.1f,%.1f]m 内 %d/%d (%.0f%%)"
              % (name, lo, hi, inside, len(rows), 100.0 * inside / len(rows)))


if __name__ == "__main__":
    main()
