#!/bin/bash
# 白色诊断重放：色框 → 人体 → 衣服 → 几何，含 conf 扫描
PY=/root/robo_team_build/vision_cuda_20261003/bin/python
REPO=/mnt/d/a/.robocup/robo_team
R=/root/robocup_runs/codex_legacy_chain_v140_20261008/round_1_seed_8159
IMG="$R/flight/algorithm/evidence_typhoon_h480_2_white_fail/frame_000001_1791448706145.png"
echo "image: $IMG"
ls -la "$IMG"

PR_WEIGHTS="$REPO/weights/best_yolo11n_bino_v1.pt" \
PR_PERSON_VERIFY_WEIGHTS="$REPO/weights/yolo11n_person.pt" \
"$PY" - "$IMG" << 'PYEOF'
import sys, os, json, math
sys.path.insert(0, '/mnt/d/a/.robocup/robo_team/perception')
import numpy as np, cv2
from ultralytics import YOLO

img_path = sys.argv[1]
img = cv2.imread(img_path)
H, W = img.shape[:2]
print('image size:', W, 'x', H)

color = YOLO(os.environ['PR_WEIGHTS'])
print('color names:', color.names)
res = color(img, conf=0.25, verbose=False, device='0')[0]
print('--- colour boxes (conf>=0.25) ---')
col_boxes = []
for b in res.boxes:
    cid = int(b.cls); xy = [float(v) for v in b.xyxy[0]]
    col_boxes.append((cid, float(b.conf), xy))
    print('  cls=%s(%s) conf=%.4f xyxy=%s w=%.1f h=%.1f' % (
        cid, color.names.get(cid), float(b.conf), [round(v,1) for v in xy],
        xy[2]-xy[0], xy[3]-xy[1]))

person = YOLO(os.environ['PR_PERSON_VERIFY_WEIGHTS'])
print('person names[0]:', person.names.get(0))
print('--- person model conf sweep ---')
sweeps = {}
for c in (0.25, 0.10, 0.05, 0.02, 0.01):
    r = person(img, classes=[0], conf=c, verbose=False, device='0')[0]
    ppl = [[float(v) for v in b.xyxy[0]] + [float(b.conf)] for b in r.boxes]
    sweeps[c] = ppl
    print('  conf>=%.2f -> %d person box(es)' % (c, len(ppl)))
    for p in ppl[:6]:
        print('      xyxy=%s conf=%.4f w=%.1f h=%.1f' % (
            [round(v,1) for v in p[:4]], p[4], p[2]-p[0], p[3]-p[1]))

def iou(a, b):
    ix1, iy1 = max(a[0],b[0]), max(a[1],b[1])
    ix2, iy2 = min(a[2],b[2]), min(a[3],b[3])
    iw, ih = max(0., ix2-ix1), max(0., iy2-iy1)
    inter = iw*ih
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter/ua if ua > 0 else 0.0

print('--- IoU of each colour box vs person boxes at each conf ---')
for cid, cconf, xy in col_boxes:
    line = '  cls=%s conf=%.3f:' % (color.names.get(cid), cconf)
    for c in (0.25, 0.10, 0.05, 0.02):
        best = max([iou(xy, p[:4]) for p in sweeps[c]], default=0.0)
        line += '  @pconf%.2f bestIoU=%.3f' % (c, best)
    print(line)

# 尺寸/尺度核对：fx=205.47 时 1.75m 人在该像素高对应的距离
print('--- scale check (fx=205.47 / 640x360) ---')
for cid, cconf, xy in col_boxes:
    h = xy[3]-xy[1]; w = xy[2]-xy[0]
    implied_range = 1.75*205.47/h if h > 0 else float('inf')
    print('  cls=%s h=%.1fpx w=%.1fpx ar=%.2f -> implied range for 1.75m person = %.1f m' % (
        color.names.get(cid), h, w, h/max(w,1e-6), implied_range))
print('--- opencv default box (all classes, higher conf) ---')
PYEOF