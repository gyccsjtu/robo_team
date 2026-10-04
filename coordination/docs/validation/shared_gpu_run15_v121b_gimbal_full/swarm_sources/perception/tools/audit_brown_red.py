#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""审计 brown / red 两类的标注质量。

动机（2026-09-15 实测）：
  v6 数据集里 red 只有 152 个框、brown 有 323 个框，而场景里 red 有 2 个
  actor(4,5)、brown 只有 1 个(actor_2) —— red 本该是 brown 的约 2 倍。
  这个倒挂说明两类标注很可能系统性互相污染。下游后果已经实测到：
  「盯着 actor_2（官方 brown）却报出红衣人」，brown 命中 3.3%、误差中位 31 m。

做法：把 class 2(brown) 与 class 4(red) 的框各随机裁若干张，拼成对照网格。
      绿线画的是原始标注框。人工目视即可判断标注是否可信 ——
      如果 class2 那行里混着明显的亮红衣，或 class4 那行里混着暗褐衣，即坐实污染。
"""
import glob
import os
import random

import cv2
import numpy as np

DS = os.path.expanduser(os.environ.get("AB_DS", "~/robocup_dataset_v6"))
SPLIT = os.environ.get("AB_SPLIT", "train")
N = int(os.environ.get("AB_N", "14"))
OUT = os.environ.get("AB_OUT", "/tmp/audit_brown_red.png")
CELL = 128
PAD = 0.30                      # 向外扩一点，保留上下文（看得出"是不是一个人"）
random.seed(int(os.environ.get("AB_SEED", "3")))

img_dir = os.path.join(DS, "images", SPLIT)
lbl_dir = os.path.join(DS, "labels", SPLIT)

buckets = {2: [], 4: []}
n_img = 0
for lp in sorted(glob.glob(os.path.join(lbl_dir, "*.txt"))):
    stem = os.path.splitext(os.path.basename(lp))[0]
    ip = None
    for ext in (".jpg", ".png", ".jpeg"):
        cand = os.path.join(img_dir, stem + ext)
        if os.path.exists(cand):
            ip = cand
            break
    if ip is None:
        continue
    n_img += 1
    try:
        with open(lp) as f:
            lines = f.read().splitlines()
    except Exception:
        continue
    for line in lines:
        v = line.split()
        if len(v) < 5:
            continue
        try:
            c = int(float(v[0]))
        except ValueError:
            continue
        if c in buckets:
            buckets[c].append((ip, [float(x) for x in v[1:5]]))

print("扫描 %d 张图 | brown(class2)=%d 框  red(class4)=%d 框"
      % (n_img, len(buckets[2]), len(buckets[4])), flush=True)
if buckets[2] and buckets[4]:
    rb = len(buckets[4]) / float(len(buckets[2]))
    print("red/brown 框数比 = %.2f  (按 actor 数(2 vs 1) 预期应 ≈2.0)" % rb, flush=True)


def crop(ip, box):
    im = cv2.imread(ip)
    if im is None:
        return None
    H, W = im.shape[:2]
    cx, cy, w, h = box
    x1, x2 = int((cx - w / 2.0) * W), int((cx + w / 2.0) * W)
    y1, y2 = int((cy - h / 2.0) * H), int((cy + h / 2.0) * H)
    mx, my = int((x2 - x1) * PAD), int((y2 - y1) * PAD)
    X1, X2 = max(0, x1 - mx), min(W, x2 + mx)
    Y1, Y2 = max(0, y1 - my), min(H, y2 + my)
    if X2 - X1 < 4 or Y2 - Y1 < 4:
        return None
    tile = im[Y1:Y2, X1:X2].copy()
    cv2.rectangle(tile, (x1 - X1, y1 - Y1), (x2 - X1, y2 - Y1), (0, 255, 0), 1)
    return cv2.resize(tile, (CELL, CELL))


rows = {}
for c in (2, 4):
    random.shuffle(buckets[c])
    tiles = []
    for ip, b in buckets[c][:N * 3]:
        t = crop(ip, b)
        if t is not None:
            tiles.append(t)
        if len(tiles) >= N:
            break
    rows[c] = tiles
    print("class %d: 裁出 %d 张" % (c, len(tiles)), flush=True)

ncol = max(1, max(len(rows[2]), len(rows[4])))
ROWH = CELL + 24
canvas = np.full((ROWH * 2 + 10, (CELL + 6) * ncol + 6, 3), 235, np.uint8)
for r, c in enumerate((2, 4)):
    tag = "class 2  brown  (actor_2)" if c == 2 else "class 4  red  (actor_4/5)"
    base = r * ROWH + 6
    cv2.putText(canvas, tag, (8, base + 14), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (20, 20, 20), 1, cv2.LINE_AA)
    for i, t in enumerate(rows[c]):
        x = i * (CELL + 6) + 3
        canvas[base + 20:base + 20 + CELL, x:x + CELL] = t

cv2.imwrite(OUT, canvas)
print("已写 %s  shape=%s" % (OUT, canvas.shape), flush=True)
