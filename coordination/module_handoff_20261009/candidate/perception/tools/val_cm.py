#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在 v6 模型的 val 集上跑一遍，输出混淆矩阵。

目的：定量确认 brown / red 的互混程度。
背景：brown 轮实测「盯着官方 actor_2(brown) 却报出红衣人」，
      brown 命中 3.3% / 误差中位 31 m，而 red 流异常活跃。
      训练集 red 框数(141) 仅为 brown(283) 的一半，与"场景里 red 有 2 个
      actor、brown 只有 1 个"的预期正好相反 —— 需要看混淆矩阵定性。
"""
import os

import numpy as np
from ultralytics import YOLO

W = os.environ.get("VC_W", os.path.expanduser("~/robocup_runs/robocup5_v6/weights/best.pt"))
DS = os.environ.get("VC_DS", os.path.expanduser("~/robocup_dataset_v6/dataset.yaml"))
OUT = os.environ.get("VC_OUT", "/tmp/val_cm.txt")

print("[vc] 权重 %s" % W, flush=True)
m = YOLO(W)
r = m.val(data=DS, split="val", imgsz=640, plots=True,
          project="/tmp/val_v6", name="cm", exist_ok=True, verbose=False)
cm = r.confusion_matrix
M = cm.matrix
names = cm.names
lab = [names[i] for i in sorted(names)] + ["背景"]

lines = []
lines.append("=== 混淆矩阵 行=真值 列=预测（最后一列=漏检, 最后一行=误检）===")
lines.append("%-10s" % "" + "".join("%9s" % n for n in lab))
for i, row in enumerate(M):
    nm = lab[i] if i < len(lab) else "?"
    lines.append("%-10s" % nm + "".join("%9d" % int(v) for v in row))

lines.append("")
lines.append("=== 按真值行归一化（每个目标最终被当成了什么）===")
lines.append("%-10s" % "" + "".join("%9s" % n for n in lab))
for i, row in enumerate(M):
    tot = int(row.sum())
    if tot == 0:
        continue
    nm = lab[i] if i < len(lab) else "?"
    lines.append("%-10s" % nm + "".join("%8.1f%%" % (100.0 * v / tot) for v in row))

txt = "\n".join(lines)
print(txt, flush=True)
with open(OUT, "w") as f:
    f.write(txt + "\n")
np.save("/tmp/val_cm_matrix.npy", M)
print("\n[vc] 已写 %s" % OUT, flush=True)
