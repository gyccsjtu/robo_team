#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
真机感知节点 v3 —— M3+：用人体几何约束干掉场景误检。

============================ v2 -> v3 改了什么 ============================
M3 验收暴露：red 命中率 51.5%、误差中位 0.85 m，明显差于 white（70.4% / 0.49 m）。
证据帧目视 + `diag_geom.py` 量化后定位到根因 —— **不是检不到红衣人**（证据帧里
`red 0.83` 稳稳跟着路上的人），而是**红色场景物件太多**：

    同一帧里加油站招牌 red 0.46、建筑红条 red 0.56、红墙 red 0.30……
    它们的世界坐标与真人相距不远，跟踪器关联被抢偏，误差就冲上 1 m。
    这推翻了 M2 的结论「误检不影响判分」——那只对 white 成立（白墙误检离人远），
    red 的误检与真目标共处窄区域，会**间接污染**正确目标。

============================ 当前人体几何 =============================
完整框按图像时刻相机姿态、地面脚点的竖线与头部像素行投影求身高。
脚点斜距只用于量程；倾斜相机的头脚深度不同，不能用 h_px * range / fy。
框边不等于精确关键点，几何通过也不证明绝对定位误差小于裁判门限。

============================ v3.0 的教训：过滤过度会断链 =============================
v3.0 把四道闸一起收紧，跑出来的结果一半喜一半忧：

    red 误差中位 0.85 -> 0.68 m   ✓ 精度确实上去了
    red 命中率 51.5% -> 57.9%     ✓
    red 上报条数 914 -> 183       ✗ 只剩 20%！
    最长连续 10.7 -> 6.1 s        ✗ 反而退步

根因：**过滤太激进导致 track 频繁死亡重建**。
  - MIN_HITS 2->3、GATE 3.0->2.0 让关联失败率飙升（新建 track 数是关联成功数的 2/3）
  - 几何硬门连真人的框也拦（框偶尔不准 -> H 越界 -> 漏检 -> coast -> 外推漂移）
  - 于是间隔最大冲到 25.6 s，远破 1 s 门限

**官方口径下，断流比误差更致命**——连续 15 s 里断一帧就全部清零。
所以 v3.1 的原则改为三句话：

    **新建严**：新 track 必须用严格几何门限，误检建不了轨
    **跟踪松**：已确认的 track（hits >= CONFIRM_HITS）用宽松门限 + 大 GATE，
                宁可放进噪声也绝不打断链路，反正选择阶段会挑对的那个
    **选择智**：发布时按 conf × 身高似然选，误检即使建了轨也抢不到发布权

同时参数退回接近 v2 的宽松值（GATE 3.0 / MIN_HITS 2），
因为**精度已经由"选择智"保证了，不需要靠"过滤狠"来换**。
=====================================================================
"""
import json
import math
import os
import sys
import threading
import time
import uuid

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
# 官方裁判订阅 ros_actor_cmd_pose_plugin_msgs/ActorInfo（cls、float32 x/y）。
# 这里的直发出口仅用于独立感知验收；正式比赛由桥接器融合后上报。
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from std_msgs.msg import String
# 集中指派消息 + 目标融合状态（coordination 工作区）。导入失败时退化为仅距离闸门。
try:
    from robocup_swarm.msg import SearchAssignment, TargetState
except Exception as _exc:
    SearchAssignment = None
    TargetState = None
    _search_assign_import_err = _exc

_CSV_HELPER = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "coordination", "src", "robocup_swarm", "scripts"))
if _CSV_HELPER not in sys.path:
    sys.path.insert(0, _CSV_HELPER)
from csv_logger import logger
from tracking_view import box_from_foot, bound_pixel_candidate
from camera_attention import ImagePublishClock
from height_datum import GroundHeightDatum

# ---------------- 参数 ----------------
# 权重查找顺序（本仓库自包含优先，找不到才退回部署位置）：
#   1. <仓库根>/weights/best_yolo11n_bino_v1.pt   —— 与 perception/ 同级，clone 即用
#   2. ~/catkin_ws/src/yolov11_ros/weights/best.pt —— 部署进 catkin 工作空间后的常规位置
# 想让感知节点指向别处（比如自己重训的权重），设 PR_WEIGHTS 即可，跳过这两步。
def _resolve_weights():
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [os.path.join(here, os.pardir, "weights", "best_yolo11n_bino_v1.pt"),
             os.path.expanduser("~/catkin_ws/src/yolov11_ros/weights/best.pt")]
    for c in cands:
        c = os.path.abspath(c)
        if os.path.isfile(c):
            return c
    # 两处都没有：返回首选路径，让报错信息直接指向应该放权重的地方
    return os.path.abspath(cands[0])

def parse_cpu_thread_count(value):
    """Reject invalid thread settings before starting perception publishers."""
    message = "PR_TORCH_THREADS must be a positive integer; got %r" % (value,)
    if isinstance(value, bool) or not isinstance(value, (str, int)):
        raise ValueError(message)
    try:
        count = int(value)
    except ValueError:
        raise ValueError(message) from None
    if count < 1:
        raise ValueError(message)
    return count


def configure_cpu_inference_threads(model, count):
    """Cap CPU work after Ultralytics select_device resets PyTorch's threads."""
    count = parse_cpu_thread_count(count)
    import torch
    logged = False

    def on_predict_start(predictor):
        nonlocal logged
        if predictor.device.type != 'cpu':
            return
        before = torch.get_num_threads()
        if before != count:
            torch.set_num_threads(count)
        actual = torch.get_num_threads()
        if actual != count:
            raise RuntimeError("PR_TORCH_THREADS=%d did not take effect; actual=%d" %
                               (count, actual))
        if not logged:
            print('[pr] CPU_THREADS uav=%s configured=%d actual=%d before=%d' %
                  (UAV, count, actual, before), flush=True)
            logged = True

    model.add_callback('on_predict_start', on_predict_start)
    return on_predict_start


TORCH_THREADS = parse_cpu_thread_count(os.environ.get("PR_TORCH_THREADS", "2"))
WEIGHTS = os.environ.get("PR_WEIGHTS") or _resolve_weights()
CONF = float(os.environ.get("PR_CONF", "0.40"))
# 白衣人历史真检置信度常在 0.34～0.40；统一 0.40 推理阈值会在建轨前
# 丢掉这些框。仅对白类放宽，其他颜色仍按 CONF 过滤。
WHITE_CONF = float(os.environ.get("PR_WHITE_CONF", "0.30"))
UAV = os.environ.get("PR_UAV", "typhoon_h480_0")
# ---- 2026-09-19 双目改造 ----
# 相机不再走 cgo3（单目 640x360 云台相机），改成平台上双目相机的**左目**
# （752x480, fx=fy=376, HFOV=90°，本节内参来自 sdf 里的 <Fx>/<Cx> 实测值）。
# 这台相机是直接塞进 base_link 的 <sensor>（没有独立 link 也没有云台）。
# 飞机世界位姿由本机 MAVROS 局部 ENU 位姿 + 官方固定出生点取得。
POSE_TOPIC = os.environ.get("PR_POSE_TOPIC", "/%s/mavros/local_position/pose" % UAV)
CAM_OFF_BL = np.array([float(x) for x in
                       os.environ.get("PR_CAM_OFF_BL", "0.12,0.06,-0.05").split(",")])
# MAVROS 与相机外参已经以米表示。旧 0.941 是旧定位链某次远距观测的
# 经验补偿，沿用到图像时刻的 MAVROS 位姿会把 8m 地面射线缩成 7.528m，
# 对裁脚后的头顶射线偏差更大。保持物理单位；明确标定才显式覆盖。
# 局部高度原点偏差是另一项误差，不能用比例系数冒充高度原点修正。
PZ_SCALE = float(os.environ.get("PR_PZ_SCALE", "1.0"))
CAM_TOPIC = os.environ.get("PR_CAM_TOPIC", "/%s/stereo_camera/left/image_raw" % UAV)
ROI_TOP = int(os.environ.get("PR_ROI_TOP", "60"))      # 忽略画面顶部：无人机自身部件常被误检成 blue
ROI_BOT = int(os.environ.get("PR_ROI_BOT", "478"))
MAX_RANGE = float(os.environ.get("PR_MAX_RANGE", "60"))
# 类别顺序必须是**训练时**的顺序（~/yolo_ds/data.yaml = red0 green1 blue2 white3 brown4）。
# 旧版这里是 ["green","blue","brown","white","red","red"]（单目 v6 模型的顺序 + red 双槽）。
# 顺序错位不会报错，只会让所有颜色系统性认反 —— 换权重时必须同步改这里。
CLASSES = ["red", "green", "blue", "white", "brown"]


def target_class_mapping(names):
    """Resolve detector IDs by label, never by an assumed training order.

    All five actor appearances must be present. Additional labels have no
    actor identity; the appearance resolver can inspect distractor_person
    before those boxes leave the actor-only path.
    """
    if isinstance(names, (list, tuple)):
        names = dict(enumerate(names))
    if not isinstance(names, dict):
        raise ValueError('Detector weights must expose class names')
    mapping, seen = {}, set()
    for key, label in names.items():
        cid = int(key)
        name = str(label).strip().lower()
        if cid < 0 or cid in mapping:
            raise ValueError('Invalid or duplicate detector class ID')
        mapping[cid] = None
        if name in CLASSES:
            if name in seen:
                raise ValueError('Duplicate actor class: ' + name)
            mapping[cid] = CLASSES.index(name)
            seen.add(name)
    missing = set(CLASSES) - seen
    if missing:
        raise ValueError('Weights missing actor labels: ' + ', '.join(sorted(missing)))
    return mapping


def target_boxes_only(boxes, mapping):
    """Adapt boxes without modifying Ultralytics tensors or their class IDs."""
    from types import SimpleNamespace
    targets = []
    for box in boxes:
        cid = mapping.get(int(box.cls))
        if cid is not None:
            targets.append(SimpleNamespace(cls=cid, conf=box.conf, xyxy=box.xyxy))
    return targets


def _box_iou(a, b):
    ax, ay, ar, ab = a
    bx, by, br, bb = b
    intersection = max(0., min(ar, br)-max(ax, bx)) * max(0., min(ab, bb)-max(ay, by))
    return intersection / max((ar-ax)*(ab-ay)+(br-bx)*(bb-by)-intersection, 1e-9)


def same_appearance_instance(a, b):
    """Only near-identical boxes can contradict one physical appearance.

    Ordinary overlap between two people is not an identity match. These
    conservative defaults need same-set validation before model deployment.
    """
    aw, ah, bw, bh = a[2]-a[0], a[3]-a[1], b[2]-b[0], b[3]-b[1]
    if min(aw, ah, bw, bh) <= 0.:
        return False
    return (_box_iou(a, b) >= .90 and .85 <= aw/bw <= 1.18 and
            .85 <= ah/bh <= 1.18 and
            abs(a[0]+a[2]-b[0]-b[2]) <= .16*min(aw, bw) and
            abs(a[1]+a[3]-b[1]-b[3]) <= .16*min(ah, bh))


class ActorDistractorResolver:
    """Resolve unique same-object conflicts, never global cross-class NMS.

    Three fresh images spanning .30 s must agree at >=.90 confidence with
    a >=.20 class margin. Until then the actor box remains attention-only.
    Motion, world locations and track longevity are not class evidence.
    """

    def __init__(self):
        self.history = []
        self.last_sample = -1.

    def resolve(self, boxes, mapping, names, sample_s):
        boxes = list(boxes)
        labels = dict(enumerate(names)) if isinstance(names, (list, tuple)) else names
        distractor_ids = {int(i) for i, name in labels.items()
                          if str(name).strip().lower() == 'distractor_person'}
        targets = target_boxes_only(boxes, mapping)
        distractors = [(tuple(float(v) for v in b.xyxy[0]), float(b.conf))
                       for b in boxes if int(b.cls) in distractor_ids and
                       math.isfinite(float(b.conf)) and float(b.conf) >= .50]
        actor_boxes = [tuple(float(v) for v in b.xyxy[0]) for b in targets]
        matches = [[j for j, (db, _) in enumerate(distractors)
                    if same_appearance_instance(ab, db)] for ab in actor_boxes]
        reverse = {j: sum(j in choices for choices in matches)
                   for j in range(len(distractors))}
        fresh = sample_s > self.last_sample
        previous = [h for h in self.history if 0. < sample_s-h['sample_s'] <= .60]
        previous_owners = {j: sum(targets[i].cls == h['cls'] and
                                 _box_iou(ab, h['box']) >= .50
                                 for i, ab in enumerate(actor_boxes))
                           for j, h in enumerate(previous)}
        following, used, result = [], set(), []
        evidence = [dict(box=list(db), confidence=dc, status='distractor')
                    for db, dc in distractors]
        # Deterministic traversal also makes temporal matching independent of
        # detector class/order. No history may be lent to two nearby instances.
        for i in sorted(range(len(targets)), key=lambda k: (targets[k].cls, actor_boxes[k])):
            actor, ab = targets[i], actor_boxes[i]
            actor.appearance_status = 'actor'
            actor.appearance_confidence = float(actor.conf)
            actor.distractor_confidence = max((distractors[j][1] for j in matches[i]), default=0.)
            if not matches[i]:
                result.append(actor)
                continue
            actor.appearance_status = 'ambiguous'
            unique = len(matches[i]) == 1 and reverse[matches[i][0]] == 1
            winner = None
            ac, dc = float(actor.conf), actor.distractor_confidence
            if unique and dc >= .90 and dc-ac >= .20-1e-9:
                winner = 'distractor'
            elif unique and ac >= .90 and ac-dc >= .20-1e-9:
                winner = 'actor'
            candidates = [(j, h) for j, h in enumerate(previous) if j not in used and
                          previous_owners[j] == 1 and h['cls'] == actor.cls and
                          _box_iou(ab, h['box']) >= .50]
            samples = []
            if fresh and winner and len(candidates) == 1:
                j, old = candidates[0]
                used.add(j)
                if old['winner'] == winner:
                    samples = [s for s in old['samples'] if sample_s-s <= 1.1]
            if fresh and winner:
                samples.append(float(sample_s))
                following.append(dict(cls=actor.cls, box=ab, sample_s=sample_s,
                                      winner=winner, samples=samples))
            if len(samples) >= 3 and samples[-1]-samples[0] >= .30-1e-9:
                actor.appearance_status = winner
            evidence.append(dict(box=list(ab), confidence=dc,
                                 status=actor.appearance_status))
            if actor.appearance_status != 'distractor':
                result.append(actor)
        if fresh:
            self.last_sample, self.history = float(sample_s), following
        return result, evidence

# --- 人体几何门限（diag_geom.py 实测标定）---
# 严格档：用于**新建** track（误检从 2.6 m 起步，真人 1.7~2.1 m）
H_MIN = float(os.environ.get("PR_H_MIN", "1.15"))
H_MAX = float(os.environ.get("PR_H_MAX", "2.10"))
AR_MIN = float(os.environ.get("PR_AR_MIN", "0.55"))
AR_MAX = float(os.environ.get("PR_AR_MAX", "2.20"))
# ---- 我方 10-03 修正：远距离小框的 AR 上限单独放宽 ----
# 实测（低空 2.8m、actor 水平 16m）：白球框 19×46px ⇒ AR=2.42，被严格档
# AR_MAX=2.20 拒掉 ⇒ 2996 次 yolo_detection 却 0 条 track（建轨只认严格档）。
# 成因：远距离目标的 YOLO 框只框到躯干/腿部窄条，AR 天然偏大（真人真实 AR≈0.4~0.6）。
# 处置：**只对"小框"放宽**（框高 < SMALL_BOX_H_PX，即目标足够远、几何不可靠），
# 近距离大框仍用原 AR_MAX 2.20，避免把近距离的竖条状误检放进来。
AR_MAX_SMALL = float(os.environ.get("PR_AR_MAX_SMALL", "3.20"))
SMALL_BOX_H_PX = float(os.environ.get("PR_SMALL_BOX_H", "60"))
# 宽松档：用于**已确认** track 的关联（保链路优先，宁可放进噪声）
H_MIN_L = float(os.environ.get("PR_H_MIN_L", "0.80"))
H_MAX_L = float(os.environ.get("PR_H_MAX_L", "3.20"))
AR_MIN_L = float(os.environ.get("PR_AR_MIN_L", "0.40"))
AR_MAX_L = float(os.environ.get("PR_AR_MAX_L", "3.00"))
CONFIRM_HITS = int(os.environ.get("PR_CONFIRM_HITS", "5"))   # 达到后转宽松档
# A three-frame red detection at (-19,-2) in the 10:27 match was a static
# object, yet it replaced a real red actor near (129,60) and dispatched two
# aircraft across the arena.  Real red tracks had >=8 hits on first report.
RED_REPORT_HITS = int(os.environ.get("PR_RED_REPORT_HITS", "6"))


def report_ready_hits(cls, hits):
    return hits >= (RED_REPORT_HITS if cls == "red" else CONFIRM_HITS)


def pixel_cue_box_ok(cls, confidence, x1, y1, x2, y2,
                     allow_small_continuation=False):
    """Image association input gate; small boxes still cannot establish a cue.

    The opt-in path only hands 32--40 px boxes to PixelCueTracker, which must
    independently prove a recent unique established instance before using one.
    """
    if (cls not in CLASSES or
            not all(math.isfinite(q) for q in (confidence, x1, y1, x2, y2)) or
            not .45 <= confidence <= 1.0):
        return False
    width, height = x2 - x1, y2 - y1
    return (y2 >= ROI_TOP and y1 <= ROI_BOT and width >= 9.0 and
            height >= (32.0 if allow_small_continuation else 40.0) and y1 >= 4.0 and
            1.1 <= height / max(width, 1.0) <= 4.0)


def foot_ground_ray(u, bottom, rotation):
    """Project the observed foot pixel, without compensating an altitude datum.

    Raising every full-box foot by 5% made the ray shallower. That happened
    to offset the old negative height-origin error; after ground calibration
    it systematically increases range. Cropped feet use a separate head ray.
    The legacy uv/wh wire representation remains inset for box reconstruction.
    """
    return rotation @ M_OPT2LINK @ np.array([
        (u-CX)/FX, (bottom-CY)/FY, 1.])


def cue_body_height(box, camera_z, rotation):
    """Optional full-box geometry for attention; abstain on crops/shallow rays."""
    x1, y1, x2, y2 = box
    if (not 2.0 <= camera_z <= 6.0 or x1 < 6. or x2 > IMG_W-6. or
            y1 < 6. or y2 > IMG_H-6. or y2-y1 < 40.):
        return None
    ray = foot_ground_ray((x1+x2)/2., y2, rotation)
    if not np.isfinite(ray).all() or -ray[2]/np.linalg.norm(ray) < .30:
        return None
    return projected_body_height((-camera_z/ray[2])*ray, y1, rotation)


class PixelCueTracker:
    """Associate fresh YOLO boxes by world bearing before ground range is known."""

    def __init__(self):
        self.tracks = []
        self.next_id = 0
        self.last_frame = -1.0
        self.rejections = []

    @staticmethod
    def _association_distance(box, tr, sample_s):
        """Existing image/bearing gates, shared by ordinary and small views."""
        if tr['cls'] != box['cls']:
            return None
        # Pixel u can sweep across the whole frame as the UAV yaws. Keep
        # vertical/size gates: azimuth alone cannot distinguish two people.
        if (math.isfinite(box.get('bearing', float('nan'))) and
                math.isfinite(tr.get('bearing', float('nan')))):
            angle = abs(math.atan2(math.sin(box['bearing'] - tr['bearing']),
                                   math.cos(box['bearing'] - tr['bearing'])))
            dt = max(0.0, sample_s - tr['sample_s'])
            angle_gate = 0.12
            if all(math.isfinite(q) for q in (
                    box.get('cam_x', float('nan')), box.get('cam_y', float('nan')),
                    tr.get('cam_x', float('nan')), tr.get('cam_y', float('nan')))):
                # Compare rays over plausible depth rather than widening the
                # bearing gate when the observing aircraft translates.
                depth = 2.0 * FY / max(tr['h'], 40.0)
                near = max(2.5, 0.55 * depth)
                far = min(20.0, 2.0 * depth)
                errors = []
                for i in range(9):
                    r = near + (far - near) * i / 8.0
                    expected = math.atan2(
                        tr['cam_y'] + r * math.sin(tr['bearing']) - box['cam_y'],
                        tr['cam_x'] + r * math.cos(tr['bearing']) - box['cam_x'])
                    errors.append(abs(math.atan2(math.sin(box['bearing'] - expected),
                                                 math.cos(box['bearing'] - expected))))
                angle = min(errors)
                angle_gate = 0.08 + 0.10 * min(dt, 0.8)
            # Braking changes camera pitch and moves a person vertically in
            # pixels without changing their world ray. Use image-time attitude
            # for elevation, as for azimuth, rather than splitting that ID.
            elevations = (box.get('elevation', float('nan')),
                          tr.get('elevation', float('nan')))
            vdiff = (FY * abs(elevations[0] - elevations[1])
                     if all(math.isfinite(v) for v in elevations)
                     else abs(box['v'] - tr['v']))
            ratio = box['h'] / max(tr['h'], 1.0)
            if (angle > angle_gate or vdiff > max(90.0, 0.8 * box['h']) or
                    not 0.55 <= ratio <= 1.8):
                return None
            return angle / angle_gate + vdiff / max(90.0, 0.8 * box['h'])
        distance = math.hypot(box['u'] - tr['u'], box['v'] - tr['v'])
        return distance if distance < max(65.0, 1.2 * box['h']) else None

    @staticmethod
    def _small_input_ok(box):
        bounds = box.get('box')
        return (box.get('small_box_continuation_only') is True and
                32.0 <= box['h'] < 40.0 and bounds is not None and len(bounds) == 4 and
                abs((bounds[3]-bounds[1])-box['h']) <= 1e-6 and
                pixel_cue_box_ok(box['cls'], box['conf'], *bounds,
                                 allow_small_continuation=True) and
                box.get('appearance_status', 'actor') == 'actor' and
                math.isfinite(box.get('bearing', float('nan'))))

    def update(self, sample_s, boxes):
        self.rejections = []
        if not math.isfinite(sample_s) or sample_s <= self.last_frame:
            return []
        self.last_frame = sample_s
        self.tracks = [tr for tr in self.tracks
                       if sample_s - tr['sample_s'] <= 1.1]
        used = set()
        cues = []
        boxes = sorted(boxes, key=lambda b: b['conf'], reverse=True)
        # Freeze associations before this image updates any track. A small
        # box needs a unique match in both directions, independent of ordering.
        scores = [{tr['id']: self._association_distance(box, tr, sample_s)
                   for tr in self.tracks} for box in boxes]
        for box_index, box in enumerate(boxes):
            best, best_d = None, float('inf')
            small = box['h'] < 40.0 or box.get('small_box_continuation_only') is True
            if small:
                if not self._small_input_ok(box):
                    continue
                matches = [tr for tr in self.tracks if scores[box_index].get(tr['id']) is not None]
                if len(matches) != 1:
                    continue
                best = matches[0]
                anchor = best.get('normal_anchor_s')
                if (best['id'] in used or anchor is None or
                        not 0.0 < sample_s-best['sample_s'] <= .35+1e-9 or
                        not 0.0 <= sample_s-anchor <= 2.0+1e-9 or
                        not math.isfinite(best.get('bearing', float('nan'))) or
                        sum(score.get(best['id']) is not None for score in scores) != 1):
                    continue
            for tr in self.tracks:
                if small:
                    break
                if tr['id'] in used or tr['cls'] != box['cls']:
                    continue
                d = self._association_distance(box, tr, sample_s)
                if d is not None and d < best_d:
                    best, best_d = tr, d
            if best is None:
                best = dict(id=self.next_id, cls=box['cls'], history=[])
                self.next_id += 1
                self.tracks.append(best)
            best.update(sample_s=sample_s, u=box['u'], v=box['v'],
                        h=box['h'], bearing=box.get('bearing', float('nan')),
                        elevation=box.get('elevation', float('nan')),
                        cam_x=box.get('cam_x', float('nan')),
                        cam_y=box.get('cam_y', float('nan')))
            best['history'] = [p for p in best['history'] if sample_s - p[0] <= 1.1]
            height_m = box.get('body_height_m')
            contradiction = (height_m is not None and math.isfinite(height_m) and
                             (height_m < H_MIN or height_m > H_MAX_L))
            best['history'].append((sample_s, box['conf'], contradiction))
            normal_history = [p for p in best.get('normal_history', [])
                              if sample_s-p[0] <= 1.1]
            if not small:
                normal_history.append((sample_s, box['conf'], contradiction))
            best['normal_history'] = normal_history
            used.add(best['id'])
            # Select by elapsed image time, not a fixed last-N frame count.
            recent = best['history']
            first = 0
            for start in range(max(0, len(recent)-2)):
                if recent[-1][0]-recent[start][0] < .3-1e-9:
                    break
                first = start
            recent = recent[first:]
            # Three independent full-body estimates contradicting human size
            # stop a hydrant from repeatedly interrupting search. Missing or
            # clipped geometry abstains; stationary people are never excluded
            # because they stand still, and one bad frame cannot veto a cue.
            if (len(recent) >= 3 and recent[-1][0]-recent[0][0] >= .3-1e-9 and
                    all(p[2] for p in recent)):
                self.rejections.append(dict(cls=box['cls'], box=box.get('box'),
                    confidence=box['conf'], height_m=height_m))
                continue
            # All clothing colours need this image-only entry. Requiring
            # .85 before association discarded .48-.76 nearby blue boxes
            # while the aircraft continued its search route.
            bounded_attention = (len(recent) >= 3 and
                                 min(p[1] for p in recent) >= 0.45 and
                                 box['conf'] >= 0.55)
            normal_cue = min(p[1] for p in recent) >= 0.85
            if (len(recent) >= 3 and recent[-1][0] - recent[0][0] >= 0.3-1e-9 and
                    (normal_cue or bounded_attention)):
                if (not small and box.get('appearance_status', 'actor') == 'actor' and
                        len(normal_history) >= 3 and
                        normal_history[-1][0]-normal_history[0][0] >= .3-1e-9 and
                        min(p[1] for p in normal_history) >= .45):
                    best['normal_anchor_s'] = sample_s
                cues.append(dict(candidate_id='%s:%d' % (box['cls'], best['id']),
                                 sample_s=sample_s, bearing=box['bearing'],
                                 confidence=box['conf'], hits=len(recent),
                                 u=box['u'], v=box['v'],
                                 attention_only=(small or not normal_cue or
                                     box.get('appearance_status', 'actor') != 'actor')))
                if small:
                    cues[-1].update(small_box_continuation=True,
                                    normal_anchor_s=best['normal_anchor_s'],
                                    report_eligible=False, footpoint_valid=False)
                for key in ('box', 'width', 'height', 'camera_tilt',
                            'appearance_status', 'appearance_confidence',
                            'distractor_confidence'):
                    if key in box:
                        cues[-1][key] = box[key]
        return cues


def pixel_promoted_strict(cues, cls, u, v, conf, rng, height_m,
                          aspect, box_h, x1, x2, y1, image_width):
    """Link an odd-shaped full-head box after image agreement; flag its range."""
    if (not all(math.isfinite(q) for q in
                (u, v, conf, rng, height_m, aspect, box_h, x1, x2, y1)) or
            not H_MIN_L <= height_m <= H_MAX_L or
            not 0.0 < rng <= 12.0 or conf < 0.85 or
            box_h < 60.0 or not 1.1 <= aspect <= 4.0 or
            x1 < 6.0 or x2 > image_width - 6.0 or y1 < 6.0):
        return False
    return any(c['candidate_id'].startswith(cls + ':') and
               not c.get('small_box_continuation', False) and
               not c.get('attention_only', False) and
               abs(c['u'] - u) < 20.0 and abs(c['v'] - v) < 30.0
               for c in cues)


def near_cropped_strict_ok(head_ground_used, range_m, conf, height_m, ar):
    """Admit a high-confidence nearby head-ray box to the new-track gate.

    Cropped feet make a real person appear too thin for the ordinary strict
    aspect-ratio gate.  This does not grant official-report eligibility;
    official_view_ok still needs a stable run of accurate head-ray positions.
    """
    return (head_ground_used and 0.0 < range_m <= STATIC_OK_RANGE and
            conf >= STATIC_OK_CONF and H_MIN <= height_m <= H_MAX and
            AR_MIN_L <= ar <= AR_MAX_L)


def near_narrow_person_strict_ok(cls, range_m, conf, height_m, ar,
                                 box_h, x1, x2, y1, image_width):
    """Admit a clear, narrow full-body box to tracking before the UAV passes it.

    The 20:21 match produced ten consecutive 0.86-0.91 blue detections at
    (150, 51), then white and green at the same map edge.  Their 63-130 px
    tall boxes were rejected solely because h/w was 2.3-3.0.  The weak blue
    false positive at (89, -27) was only 0.40-0.64 confidence.  Keep this
    relaxation for strongly classified people with an intact head;
    the ordinary verdict and official-view gates still apply afterwards.
    """
    return (cls in ("blue", "green", "white", "brown", "red") and
            0.0 < range_m <= 12.0 and conf >= 0.80 and
            H_MIN <= height_m <= 2.40 and
            AR_MAX < ar <= 3.20 and box_h >= 60.0 and
            x1 >= 6.0 and x2 <= image_width - 6.0 and y1 >= 6.0)


def skip_attached_verdict(cls, assigned_tid, range_m):
    """The assigned person stays in frame when the camera follows it."""
    need_tid = TID_OF_COLOR.get(cls)
    assigned = (assigned_tid in ("t4", "t5") if cls == "red"
                else assigned_tid is not None and assigned_tid == need_tid)
    return assigned and range_m <= ATTACH_SKIP_RANGE


def official_view_ok(uv, wh, width=752, height=480, range_m=None,
                     clipped_stable=True):
    """True when the person box has enough border for accurate ground range."""
    u, v = uv
    w, h = wh
    # v is the foot pixel after the 5% box-height inset; restore the actual
    # bottom edge.  Cropped feet or torso at an image edge gave >1 m errors
    # on actor_3/4 in the 10:27 run, despite high YOLO confidence.
    x1, x2 = u - w / 2.0, u + w / 2.0
    y1, y2 = v - 0.95 * h, v + 0.05 * h
    if not (x1 >= 6.0 and x2 <= width - 6.0 and y1 >= 6.0):
        return False
    if y2 <= height - 6.0:
        return True
    # At short range, a clipped foot has already been localized from the
    # visible head ray (clipped_head_ground).  Suppressing all such boxes kept
    # the exact, continuously observed red actor at the map edge invisible to
    # the judge for >20 s.  Reject distant/partial boxes and missing heads.
    # Track geometry can return numpy.bool_.  Python's `and` returns that
    # operand on a false branch; json.dumps then kills the entire camera node.
    # Preserve the decision, but guarantee the wire field is a native bool.
    return bool(clipped_stable and range_m is not None and 0.0 < range_m <= 6.0 and
                h >= FEET_CLIP_MIN_PX and y2 <= height + 2.0)


def continuation_view_ok(track, detection, sample_s, residual_m):
    """Keep camera control during a brief gait/confidence change of one track.

    Admission of a NEW person still uses strict geometry/appearance. A known
    person's changing width is not evidence that its visible foot ray moved.
    Crops, shallow rays, implausible height, old images and spatial jumps do
    not receive this continuation. Strict evidence expires independently;
    accepting a continuation cannot renew its own identity evidence. Replay
    found extra >1m errors when used for official qualification: it is ONLY
    camera-control geometry, never permission to report a coordinate.
    """
    if detection.get('loc_ok', False):
        return True
    try:
        gap = sample_s - track.observed_s
        since_strict = sample_s - track.last_strict_s
        u, v = detection['uv']
        w, h = detection['wh']
        return bool(
            track.hits >= CONFIRM_HITS and not track.loc_suspect and
            detection['cls'] == track.cls and
            track.appearance_status == detection['appearance_status'] == 'actor' and
            0.0 < gap <= .7 and 0.0 <= since_strict <= 2.0 and
            detection['range_method'] == 'foot_ground' and
            0.0 < detection['range'] <= 12.0 and
            .30 <= detection['sin_down'] <= 1.0 and
            H_MIN <= detection['h'] <= H_MAX and
            detection['conf'] >= (WHITE_CONF if track.cls == 'white' else CONF) and
            h >= 60.0 and w > 0.0 and AR_MIN_L <= h/w <= 3.2 and
            math.isfinite(residual_m) and 0.0 <= residual_m <= min(1.0, .6 + 1.5*gap) and
            official_view_ok((u, v), (w, h), IMG_W, IMG_H,
                             detection['range'], clipped_stable=False))
    except (KeyError, TypeError, ValueError, AttributeError):
        return False


def blue_white_torso_vote(image, xyxy):
    """Use only a clear shirt color to disambiguate duplicate person boxes."""
    x1, y1, x2, y2 = (float(v) for v in xyxy)
    width, height = x2 - x1, y2 - y1
    if width < 12 or height < 24:
        return None
    image_h, image_w = image.shape[:2]
    xa = max(0, min(image_w, int(x1 + 0.30 * width)))
    xb = max(0, min(image_w, int(x1 + 0.70 * width)))
    ya = max(0, min(image_h, int(y1 + 0.20 * height)))
    yb = max(0, min(image_h, int(y1 + 0.52 * height)))
    if xb - xa < 4 or yb - ya < 5:
        return None
    pixels = image[ya:yb, xa:xb].astype(np.float32)
    blue, green, red = pixels[:, :, 0], pixels[:, :, 1], pixels[:, :, 2]
    high = np.maximum(np.maximum(blue, green), red)
    low = np.minimum(np.minimum(blue, green), red)
    blue_fraction = np.mean((blue > 55.0) & (blue > 1.35 * green) &
                            (blue > 1.50 * red))
    white_fraction = np.mean((low > 90.0) &
                             ((high - low) < 0.25 * high))
    if blue_fraction >= 0.35 and blue_fraction > 1.8 * white_fraction:
        return "blue"
    if white_fraction >= 0.35 and white_fraction > 1.8 * blue_fraction:
        return "white"
    return None


def filter_blue_white_duplicate_boxes(image, boxes):
    """Drop a cross-class duplicate only when visible shirt color is decisive."""
    dropped = set()
    for i, first in enumerate(boxes):
        if i in dropped or int(first.cls) not in (2, 3):
            continue
        a = [float(v) for v in first.xyxy[0]]
        area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
        for j in range(i + 1, len(boxes)):
            second = boxes[j]
            if j in dropped or {int(first.cls), int(second.cls)} != {2, 3}:
                continue
            b = [float(v) for v in second.xyxy[0]]
            area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
            overlap = max(0.0, min(a[2], b[2]) - max(a[0], b[0])) * \
                      max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
            iou = overlap / max(area_a + area_b - overlap, 1e-6)
            if iou < 0.75:
                continue
            votes = [blue_white_torso_vote(image, box) for box in (a, b)]
            votes = [vote for vote in votes if vote is not None]
            if not votes or len(set(votes)) != 1:
                continue
            preferred = 2 if votes[0] == "blue" else 3
            dropped.add(i if int(first.cls) != preferred else j)
            if i in dropped:
                break
    return [box for i, box in enumerate(boxes) if i not in dropped]

H_REF = float(os.environ.get("PR_H_REF", "1.75"))      # 身高似然中心 (m)
H_SIG = float(os.environ.get("PR_H_SIG", "0.70"))      # 身高似然宽度 (m)
H_GATE = float(os.environ.get("PR_H_GATE", "1.50"))    # 关联时身高一致性门限 (m)，v3.0 是 0.90 太紧

# --- 运动性判据（v3.3 新增）---
# 为什么需要它：v3.2 让 conf 优先，结果 red/green 变好、white 崩了
# （94.4% -> 42.2%）。根因是 white 的 conf 分布反常：
#     actor_3 白衣人 conf 0.34~0.40，而白墙/白窗误检 conf 更高
#   -> 谁的 conf 高谁赢 = 把发布权送给墙。
# 颜色相关的判据都不可靠，但有一条与颜色无关的：**真人在走（~1 m/s），场景不动**。
# 用「净位移 / 存活时长」而不是瞬时速度 —— 单目测距抖动是零均值的，
# 净位移对其免疫，而瞬时速度会被抖动灌满。
MOTION_REF = float(os.environ.get("PR_MOTION_REF", "1.0"))    # 满分速度 (m/s)，actor 约 1 m/s
MOTION_FLOOR = float(os.environ.get("PR_MOTION_FLOOR", "0.40"))  # 完全静止时的打分折扣

# --- 跟踪器参数（v3.1 退回接近 v2 的宽松值：链路优先）---
DETECT_EVERY = int(os.environ.get("PR_DETECT_EVERY", "2"))
ALPHA = float(os.environ.get("PR_ALPHA", "0.5"))
BETA = float(os.environ.get("PR_BETA", "0.15"))
GATE = float(os.environ.get("PR_GATE", "3.0"))               # v3.0 收到 2.0 -> 关联崩了
# --- 帧间跳变门限（v3.6 新增，2026-09-15 实测驱动）---
# 已确认的 track 原来用固定 GATE=3.0 m 收关联，太宽了。实测（brown 干净轮）：
#   · 真人连续跟踪段：帧间世界位移 中位 0.098 / p99 0.501 / **最大 0.577 m**（dt≈0.11 s）
#     —— 人走 ~0.7 m/s，0.1 s 最多挪 0.07 m，剩下的是单目测距抖动；158 段里 >1 m 的 **0 次**。
#   · 含误检段：13 次 >1 m 的跳变，最大 **3.29 m**，其中 11 次发生在 dt≤0.15 s。
# 也就是说"一个走路的人不可能在 0.1 s 内位移 3 m"（那等于 30 m/s）。
# 而 3.0 m 的固定门限刚好放行——实测 C 轮 t≈18.8 s，brown 的已确认 track（hits=92）
# 被一个**突然出现的近处大黑物**（框 38→53 px、55 px 横跳、H 2.11→2.57、D 12.9→10.0）
# 一口吃掉，随后被拖着一路膨胀到 88 px / H 2.88 m / D 6.7 m，track 报废、整类静默 60 s。
# 所以改成按 dt 缩放：门限 = GATE_MIN + GATE_MULT·dt，并仍不超过 GATE。
#   dt=0.11 -> 1.02 m（真人最大 0.577，留 1.8 倍余量）
#   dt=0.20 -> 1.20 m   dt=0.50 -> 1.80 m   dt≥1.1 -> 退回 3.0 m
# 只对**已确认** track 生效；未确认 track 本来只吃严格档，不受影响。
# 单帧被拒的代价只是 coast 一帧（<0.2 s），远小于断流；而放进来的代价是整条链报废。
GATE_DYN = int(os.environ.get("PR_GATE_DYN", "1"))
GATE_MIN = float(os.environ.get("PR_GATE_MIN", "1.5"))
GATE_MULT = float(os.environ.get("PR_GATE_MULT", "3.0"))


def association_score(distance, similarity, gate, confirmed, appearance_weight=.3):
    if not 0. <= distance < gate:
        return None
    return distance-appearance_weight*similarity*gate if confirmed and similarity > .3 else distance


def assoc_gate(dt, hits, confirm_hits=CONFIRM_HITS, gate=GATE,
               gate_min=GATE_MIN, gate_mult=GATE_MULT, dyn=GATE_DYN):
    """已确认 track 的关联门限（按 dt 缩放）。纯函数，便于离线单测。

    未确认 track 返回固定 gate（它们本来只吃严格档，不受这一改动影响）。
    """
    if not dyn or hits < confirm_hits:
        return gate
    return max(gate_min, min(gate, gate_min + gate_mult * dt))
TRACK_TIMEOUT = float(os.environ.get("PR_TRACK_TIMEOUT", "3.0"))
MIN_HITS = int(os.environ.get("PR_MIN_HITS", "2"))           # v3.0 提到 3 -> 建链太慢
V_MAX = float(os.environ.get("PR_V_MAX", "3.0"))
PUB_HZ = float(os.environ.get("PR_PUB_HZ", "10.0"))
# 连续 coast 超过这么多次检测就不再发布/不再参与选优。
# 检测频率是 PUB_HZ / DETECT_EVERY，而不是 PUB_HZ；旧的固定 8 帧在当前
# 5Hz 检测下实际会外推 1.6s，超过官方 1s 断流门槛，短暂遮挡就会清零 15s。
# 默认留约 2.0s 外推余量（原 0.8s 太短，目标短暂离开视野即丢轨，
# 实测 hits=62 v=0.71 也因 coast 超 4 帧被丢弃，pub=0）。
# 仍可用 PR_MAX_COAST_PUB 做现场 A/B 调整。
_coast_budget = max(1, int(2.0 * PUB_HZ / max(1, DETECT_EVERY)))
MAX_COAST_PUB = int(os.environ.get("PR_MAX_COAST_PUB", str(_coast_budget)))

# ---- 时间滞后补偿（发布前把坐标沿目标速度前推这么多秒）----
# 为什么需要：官方 score_cal_master.py 第 136 行是拿 **接收瞬间的当前真值**
#   和 msg.x/y 比，判据 <1m。而我们的链路存在相位滞后（实测 0.4~0.5s 仿真时间，
#   折算墙钟约 1.0~1.2s，像是多级流水线积压）。这段滞后在目标走路时直接变成
#   定位误差，且**正比于目标速度**：
#       1 m/s -> 偏差 ~0.4m ；  2 m/s（规则 line 131-134 的躲逃速度）-> ~0.9m
#   不做补偿，2 m/s 下的误差中位 1.43m、单帧达标仅 30%，15s 连续判据必挂。
#
# 实测（422 条真实观测，飞控真飞快照离线重放；见 _evidence/fly_yolo/）：
#       不补偿             1m/s 中位 0.85m / 单帧<1m 59%   |  2m/s 1.43m / 30%
#       本补偿 k=0.3       1m/s 中位 0.72m / 单帧<1m 67%   |  2m/s 1.27m / 35%
#       真值速度 k=0.5（天花板） 1m/s 0.71m / 73%           |  2m/s 1.20m / 38%
#   => 补偿是真收益，但**只值 8 个百分点**，别指望它单独解决 2m/s。
#      它拿走了大部分"可补偿"的部分；剩下的是与速度无关的测距/几何噪声。
#
# 用 Track.predict()（alpha-beta 状态里本来就有 vx/vy，只是发布时没用），
# 不引入任何新状态。目标静止时 v~0 => 补偿量~0，对静态误检无副作用。
# 置 0 即完全回到改动前行为。
LAG_COMP = float(os.environ.get("PR_LAG_COMP", "0.3"))

# 对外发布坐标的 EMA 平滑系数（0=关闭）。裁判要求连续 15s 上报误差 <1m，
# alpha-beta 滤波后仍有单帧跳变（实测 brown 跳 1.8m），这里在出口处再压一道。
# 0.5 是轻量平滑：新观测占 50%，既压抖动又不引入过多滞后（LAG_COMP 已补偿链路滞后）。
PUB_EMA = float(os.environ.get("PR_PUB_EMA", "0.5"))

# ---- 官方话题发布仲裁（2026-10-01 复盘新增）----
# 复盘：6 个感知节点各自直发 /actor_<color>_info，同色多机同发且位置相差
# 数十米（brown 100 冲突帧、red 43、blue 5），裁判每收到错位消息即清零
# 15s 计时（score_cal.py err_threshold=1m）。且大量发布是 17~21m 远程误检。
# 双闸门：①只有被 manager 指派追踪该目标的飞机可发；②目标必须在近距离内。
OFFICIAL_ARBITRATED = int(os.environ.get("PR_OFFICIAL_ARBITRATED", "0"))
ACTOR_PUB_RANGE = float(os.environ.get("PR_ACTOR_PUB_RANGE", "45.0"))
# ---- 我方 10-03 追加（取自队友 mavros-port-iris 接入侧的裁判门控）：确认门 ----
# 官方判分 score_cal.py 只比 x,y 误差（<1m）与上报间隔（<1s），**完全不看 confidence**。
# hits < CONFIRM_HITS 的未确认 track 绝大多数是活不过几秒的误检（灯柱/招牌/红消防栓），
# 一旦发到 /actor_*_info，就会以 20~40m 的坐标误差把裁判 15s 连续计时清零
# （清零 = 永远消除不了）。队友 20260929T041507Z 实测：tid=6 @36m、tid=18 @20m
# 都是 hits=2~3 的假 track。
#   0（默认）= 旧行为，未确认 track 也上报；
#   1 = 仅已确认 track（hits >= CONFIRM_HITS）上报。
# 与 ACTOR_PUB_RANGE 是互补的两道门：距离门挡"远而持续"的静态误检（灯柱在
# 17~23m，而真目标在 6~9m），确认门挡"近而短命"的误检（位置不固定，靠距离切不开）。
ACTOR_CONFIRM_ONLY = int(os.environ.get("PR_ACTOR_CONFIRM_ONLY", "0"))
# ---- 我方 10-02 追加：官方话题直发总开关 ----
# 比赛链路里 /actor_<color>_info 的唯一合法发布者是 yolo_target_bridge（它做多机
# 融合后单点 10Hz 统一发）。本机默认 0=不直发，避免与 bridge 双发布导致裁判反复
# 清零 15s 计时。只有在"单机独立跑官方裁判做验收"时（没有 bridge）才设 1。
DIRECT_ACTOR_INFO = int(os.environ.get("PR_DIRECT_ACTOR_INFO", "0"))
# ---- 我方 10-03：对外发布坐标平滑参数（见 ActorInfo 发布处注释）----
PUB_SMOOTH_ALPHA = float(os.environ.get("PR_PUB_SMOOTH_ALPHA", "0.35"))
PUB_MAX_STEP = float(os.environ.get("PR_PUB_MAX_STEP", "0.30"))   # 单帧最大位移 m
_pub_smooth = {}                                                # (cls,slot) -> (x,y)
# 第三闸门——空间身份一致：待发布坐标必须与 /swarm/target_states 中该指派
# 目标的 YOLO 融合估计相差 <= IDENTITY_M，且状态新鲜 <= TSTATE_FRESH_S。
# 2026-10-01 复盘：演员逃远后，飞机近处的别的演员被误判成同一颜色仍顶着
# 该 cls 发布（实测把 5m 外的 actor_3/4 发成 blue，误差 93m），裁判反复重置。
IDENTITY_GATE = int(os.environ.get("PR_IDENTITY_GATE", "0"))
IDENTITY_M = float(os.environ.get("PR_IDENTITY_M", "2.5"))
TSTATE_FRESH_S = float(os.environ.get("PR_TSTATE_FRESH_S", "3.0"))
# 颜色 -> 目标 tid；red 槽位：slot0(/actor_red1_info)->t5(actor_5)，
# slot1(/actor_red2_info)->t4(actor_4) —— 官方 red 映射是反的。
TID_OF_COLOR = {"green": "t0", "blue": "t1", "brown": "t2", "white": "t3"}
TID_OF_RED_SLOT = {0: "t5", 1: "t4"}

# red 双流：空槽补位时的"空间连续性"半径 (m)，见 assign_slots 注释。
# 0 = 关闭。两个红衣人相距约 14 m，取 6 既够跨过一次 track 重建，又不会串人。
RED_STICKY_R = float(os.environ.get("PR_RED_STICKY_R", "6.0"))

# 官方像素内参（双目左目, 752x480, HFOV=90°, 与 sdf 里 Fx/Fy/Cx/Cy 一致）
# 旧值（cgo3 单目 640x360）：FX=FY=205.47, CX=320.5, CY=180.5
# fy 变大 ⇒ 同样的框像素高换算出更小的 H？不：H=h_px·z/fy，fy 大 ⇒ H 偏小，
# 但角分辨率也高了 1.83 倍（同一目标像素高也大 1.83 倍），两者相抵，H 仍然准。
FX = FY = 376.0
CX, CY = 376.0, 240.0
# ---- 我方 10-02 修正：近目标测距 ----
# 原几何假设「目标双脚站在 z=0 平面」（t = -pz/D_z 求视线与地面的交点）。目标很近
# 时双脚会落到画面下沿之外，检测框的 y2 被裁在图像边缘（实测日志里 uv=(596,480)），
# 求出来的地面交点远在真实位置之后 ⇒ 测距系统性偏长。
# 实测：真实水平距 7.09m / 相机高 5.3m 的目标，报 D=10.61m、位置偏 2.0m，
# 官方裁判 err_threshold=1m 永远不通过。近目标改用身高反推：
#     h_px = FY · H_real / rng   ⇒   rng = FY · H_real / h_px
H_ASSUMED = float(os.environ.get("PR_H_ASSUMED", "1.70"))   # 官方 actor 实际身高约 1.70~1.75m
FEET_CLIP_PX = float(os.environ.get("PR_FEET_CLIP", "3.0"))  # y2 距图像下沿多少像素算"脚被裁"
# 护栏：只有"盒子够大"才允许走身高反推。远处静态误检（17~21m 招牌/建筑色条）
# 的框只有 ~30px，一旦被反推成 H=1.70m 的标准人形就会凭空造出真人轨
# （实测 hits=530 的假轨）。40px 对应 1.7m 的人在 ~16m 内。
FEET_CLIP_MIN_PX = float(os.environ.get("PR_FEET_CLIP_MIN_PX", "40.0"))
IMG_W, IMG_H = 752, 480

M_OPT2LINK = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])

# --- 标注帧（只画"那个人"，不画误检）---
# 为什么要在线画：离线标注脚本拿不到深度，算不出归一化身高 H，
# 也就无法区分"真人"和"招牌/墙/建筑色条"，结果满屏乱框。
# 感知节点这里深度、身高、track 打分全都有，画出来的 == 上报给裁判的那个目标。
# 2026-09-19 双目改造：默认**关闭**标注存帧。
#  ① 本项目硬规矩"图上绝不画框"，验证阶段不该产出任何画框图；
#  ② VM 只有 8 个核，还要跑 Gazebo+PX4+6 个 actor，落盘 JPEG 是纯浪费。
# 需要证据帧时显式 PR_ANNOT_ON=1 打开（帧尺寸已变成 752x480，目录也换新，别和旧帧混）。
ANNOT_ON = int(os.environ.get("PR_ANNOT_ON", "0"))
# YOLO 实时视角：把「即将上报裁判」的检测框画到图像上，通过 ROS 话题发布
# （rqt_image_view 选 /<uav>/perception/yolo_view 即可，不落地、不喂回检测器）。
YOLO_VIEW = int(os.environ.get("PR_YOLO_VIEW", "1"))
YOLO_VIEW_TOPIC = os.environ.get(
    "PR_YOLO_VIEW_TOPIC", "/%s/perception/yolo_view" % UAV)
ANNOT_EVERY = float(os.environ.get("PR_ANNOT_EVERY", "0.5"))    # 存帧间隔 (s, 墙钟)
ANNOT_TOP = int(os.environ.get("PR_ANNOT_TOP", "1"))            # 只画得分最高的 N 个；0 = 每类最优全画
ANNOT_CONFIRM = int(os.environ.get("PR_ANNOT_CONFIRM", "0"))    # 1=只画已确认(hits>=CONFIRM_HITS)的 track
ANNOT_DIR = os.path.expanduser(os.environ.get("PR_ANNOT_DIR", "~/robocup_real/_bino_frames/only"))
ANNOT_RAW_DIR = ANNOT_DIR + "_raw"
ANNOT_CLS = os.environ.get("PR_ANNOT_CLS", "")                  # 只看某类，如 "red"；空=全部
# 相机高度低于此值时不存标注帧：观测任务要求飞起来看，
# 贴地时相机对地，整屏都是地面纹理，检出的框毫无意义（实测会画出地面误检）。
ANNOT_MIN_Z = float(os.environ.get("PR_ANNOT_MIN_Z", "2.5"))

# --- 「是不是人」判决（v3.5 新增，专治标注里出现非人）---
# ============================ 为什么还要再加一道 ============================
# v3.4 已经做到「只画 best_of 里得分最高的 1 个框」，但证据帧里仍然出现了：
#     white 0.66  H=2.32m  D=15.8m  -> 框在**白色建筑构件**上
#     white 0.75  H=1.96m  D=14.9m  -> 框在**无人机自己的白色起落架**上
# 两者的归一化身高 H 都落在「像人」区间（真人 1.70~2.11 m），
# 所以**任何单帧几何判据都分不开**（H / 宽高比 / conf 全试过）。
# 也试过"自身机体静态掩膜"：用多点平移求逐像素时域方差自动提取机身，
# 结果失败 —— 机身在画面里的位置会随姿态/云台角变化（同一套参数下，
# 起落架一会儿在右侧大片桨叶处、一会儿跑到左下右下两根黑杠），
# 静态掩膜根本覆盖不住。详见 calib_selfmask.py 头部的三次实验记录。
#
# 剩下唯一与颜色、与几何都无关的判据：**运动**。
#   真人以 ~1 m/s 在走（actor AI 速度 v=1）；
#   墙面/招牌/建筑色条在世界上不动；
#   钉住悬停时，机身算出来的"世界坐标"同样不动。
# 判据用**净位移 / 存活时长**（不是瞬时速度）：单目测距抖动是零均值的，
# 净位移对它免疫，瞬时速度却会被抖动灌满。
ANNOT_VERDICT = int(os.environ.get("PR_ANNOT_VERDICT", "1"))    # 1=标注只画通过"人"判决的
# 是否**同时**保存未画框的原图 -> <ANNOT_DIR>_raw/。
# 为什么需要：标注帧上已经画了框和文字，不能再喂回检测器做复核
# （画出来的框本身会被检成目标）。2026-09-15 想复核"某个可见的人为什么没被检到"时
# 才发现所有落盘帧都是画过框的，只能靠人工看。存一份原图，这个问题就变成可自动化的。
ANNOT_RAW = int(os.environ.get("PR_ANNOT_RAW", "0"))
PUB_VERDICT = int(os.environ.get("PR_PUB_VERDICT", "1"))        # 1=发布也过同一道判决
VERDICT_H_MIN = float(os.environ.get("PR_VERDICT_H_MIN", "1.30"))
VERDICT_H_MAX = float(os.environ.get("PR_VERDICT_H_MAX", "2.40"))
# 净速度门限：真人实测 0.28~0.48 m/s（观测点钉死时，净位移比路径短），
# 静止误检的"世界坐标"只在框抖动范围内漂 -> 折 60 s 后 sp≈0.01。
# 所以 0.15 就足够分开，不用卡到 0.25（那会把真人一起卡掉）。
VERDICT_SP = float(os.environ.get("PR_VERDICT_SP", "0.15"))     # 净位移速度门限 (m/s)
# ⚠ 只卡瞬时净速度会误伤"走了一段然后停下的人"。实测踩到过：
#   actor 的逃跑 AI 会朝地图边界狂奔，跑到 y=±50 就**永久卡死**，
#   于是真人 100 s 一动不动 -> 静态判据把真人一起拦了（日志里"静止=698"）。
# 所以改成"当前在动 **或** 曾经动过"：只要这个 track 生命期内
# 与世界起点的最大位移 ≥ VERDICT_DISP，就承认它是人。
# 静止误检的最大位移只有框抖动带来的 ~0.3~0.5 m，2.5 m 足够区分。
VERDICT_DISP = float(os.environ.get("PR_VERDICT_DISP", "2.5"))  # 生命期最大位移 (m)
# --- 视差判据（v3.5b 新增，专治"机身跟着观测机转向而漏网"）---
# 踩到的漏网实例：观测机盯人时会不停转向，机身的"假世界坐标"就在以 13 m 为半径
# 绕圈 -> 净速度 0.74 m/s，运动判据不但拦不住，反而把它判成"在动"。
# 但有一条它绝对逃不掉：**机身相对相机是静止的**。
# 所以把物体坐标换到【相机系】再比较：
#     真物体  -> 相机一平移/转向，它的相机系坐标就跟着变（视差）
#     机身    -> 相机系坐标恒定（刚性附着），d_rel ≈ 0
# 判据：视点变化 d_view 已经足够大（>1.5 m，否则测不出来），
#       而物体的相机系位移 d_rel < 0.35 * d_view  -> 判定为附着在机身上的东西。
VERDICT_ATTACH_ON = int(os.environ.get("PR_VERDICT_ATTACH", "1"))
ATTACH_MIN_VIEW = float(os.environ.get("PR_ATTACH_MIN_VIEW", "1.5"))
ATTACH_FRAC = float(os.environ.get("PR_ATTACH_FRAC", "0.35"))
# 被指派追踪该目标且 rng <= 该值（盘旋确认段）时跳过 attach 视差检查。
ATTACH_SKIP_RANGE = float(os.environ.get("PR_ATTACH_SKIP_RANGE", "12.0"))
# 量程闸门：只画"我们真正在观测"的目标。
# 本项目的观测策略锁定 12~20 m（≥20 m 是为了不惊动 actor 的逃跑 AI，见 MEMORY）。
# 超出量程还能被检到的，基本是远方路边的小牌子/小构件 —— 实测 27 m 处一个小标志牌
# 被检成 brown 0.56 H=1.43m，靠 5 px 框的抖动把净速度刷到 1.79 m/s，
# 把运动判据和视差判据都骗过去了。它既不在我们观测量程内，也不值得为它调参，直接不画。
VERDICT_RANGE_MAX = float(os.environ.get("PR_VERDICT_RANGE_MAX", "22.0"))
# 打分下限：white 的 conf 本来就低（0.34~0.45，白墙误检反而更高），
# 所以这里只当"别把太弱的框画出来"的兜底，真正的判据是运动性。
VERDICT_SCORE = float(os.environ.get("PR_VERDICT_SCORE", "0.20"))
# 静止演员仍须进入视觉候选。10:54 局在东侧边界拍到蓝色演员
# D=5.87m/hits=4、白色演员 D=1.62m/hits=4，原 hits>=8 使二者
# 在发送候选前被丢弃，飞机飞过后再也无法积累到 8 次。用更严格的
# 近距离限制允许 4 次真实检测通过；人体高度、置信度、视差和后续
# 官方坐标精度门槛仍分别检查，远处静态建筑误检继续被挡住。
STATIC_OK = int(os.environ.get("PR_STATIC_OK", "1"))
STATIC_OK_HITS = int(os.environ.get("PR_STATIC_OK_HITS", "4"))
STATIC_OK_RANGE = float(os.environ.get("PR_STATIC_OK_RANGE", "6.5"))
STATIC_OK_CONF = float(os.environ.get("PR_STATIC_OK_CONF", "0.75"))
ANNOT_MAX_COAST = int(os.environ.get("PR_ANNOT_MAX_COAST", "2"))  # 画框要求刚检出（别画外推）

COLORS = {"green": (0, 255, 0), "blue": (255, 128, 0), "brown": (19, 69, 139),
          "white": (255, 255, 255), "red": (0, 0, 255)}

# ---- M5 硬前置：每类要发几条流 ----
# 官方裁判 score_cal.py 对 red 是**特殊对待**的：
#   actor_{blue,green,white,brown}_info -> actor_info_callback     （单条流）
#   actor_red1_info / actor_red2_info   -> actor_info1/2_callback  （两条流）
# 因为 actor_id_dict = {... 'red':[4,5]}，两个红衣人各占一个 actor id，
# 而裁判只订阅两条 red 流 —— 一条流覆盖不了两个人。
#
# ⚠ red 的 callback 里还埋了一条**清零机制**（普通 callback 没有）：
#     red_cnt = 0
#     for i in [4,5]:
#         if 坐标离 actor_i 在 1m 内 and 间隔<1s:  命中，flag_1 = i
#         else: red_cnt += 1
#     if red_cnt == 2 and flag_1 != 0:            # 这一帧两个 actor 都不匹配
#         count_flag[flag_1] = False              # 连续两次报偏 -> 连续计时清零
# 所以两条 red 流必须**各自锁定一个红衣人、绑定持久化**。若逐帧按分数重排
# （谁分高谁发 red1），流与人的对应关系会在帧间互换，flag_1 反复重置，
# 15 s 连续判定永远累不到 —— M5 直接零分。
TOPIC_OF = {
    "green": ["/actor_green_info"],
    "blue": ["/actor_blue_info"],
    "brown": ["/actor_brown_info"],
    "white": ["/actor_white_info"],
    "red": ["/actor_red1_info", "/actor_red2_info"],
}

# --- Internal coordination input, schema 2: one complete camera frame. ---
# Each candidate retains the old measurement fields plus a process-scoped
# identity. yolo_target_bridge alone reduces physical hypotheses to official
# colour streams; the final ActorInfo contract remains unchanged.
COORD_ON = int(os.environ.get("PR_COORD_ON", "1"))
COORD_HZ = float(os.environ.get("PR_COORD_HZ", "2.0"))   # 对齐队友替身；核心按 tick 消费，灌太快会积压
# Confidence always remains the detector probability; track_confirmed is separate.
TARGET_Z = float(os.environ.get("PR_TARGET_Z", "1.25"))
_CSV = logger("perception_%s" % UAV, [
    "ros_time", "event", "target_id", "class_name", "confidence",
    "bbox_u", "bbox_v", "bbox_w", "bbox_h", "target_x", "target_y",
    "target_z", "range_m", "height_m", "strict", "track_id", "hits",
    "image_stamp", "image_age_s", "inference_s", "source",
    "sin_down", "range_method", "assoc_residual_m", "assoc_gate_m",
    "observation_gap_s"])
# 调试快照**另开一条话题**：它原来和契约挤在同一条 topic 上，形状完全不同。
# 调试通道与契约通道必须分开，否则要么队友解析不了、要么我自己的复盘脚本全废。
DEBUG_TOPIC = os.environ.get("PR_DEBUG_TOPIC", "/perception/debug_snapshot")
FAST_TRACKER_SHADOW = os.environ.get('PR_FAST_TRACKER_SHADOW', '0') == '1'

_lock = threading.Lock()
_latest = {"img": None, "stamp": 0.0, "recv_wall": 0.0, "recv_ros": 0.0}
_pose_history = []  # (pose header ROS time, world camera x/y/z, rotation matrix)
_pose_fault = None  # in-flight EKF discontinuity: fail closed until node restart
_model = None

# 统计：几何门限拦下多少
_stat = {"n_det": 0, "n_geo_rej": 0, "n_assoc": 0, "n_new": 0, "n_new_rej": 0}


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(rotation):
    """Return a normalized xyzw quaternion for the interpolated camera pose."""
    m = rotation
    trace = float(np.trace(m))
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = ((m[2, 1] - m[1, 2]) / s,
             (m[0, 2] - m[2, 0]) / s,
             (m[1, 0] - m[0, 1]) / s, s / 4.0)
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            q = (s / 4.0, (m[0, 1] + m[1, 0]) / s,
                 (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
        elif i == 1:
            s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            q = ((m[0, 1] + m[1, 0]) / s, s / 4.0,
                 (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
        else:
            s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s,
                 s / 4.0, (m[1, 0] - m[0, 1]) / s)
    norm = math.sqrt(sum(v * v for v in q))
    return tuple(v / norm for v in q)


def interpolate_rotation(first, second, alpha):
    """Slerp avoids bearing error from linearly averaging yaw matrices."""
    q0 = np.array(R_to_quat(first), dtype=float)
    q1 = np.array(R_to_quat(second), dtype=float)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1, dot = -q1, -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        q = (1.0 - alpha) * q0 + alpha * q1
    else:
        theta = math.acos(dot)
        q = (math.sin((1.0 - alpha) * theta) * q0 +
             math.sin(alpha * theta) * q1) / math.sin(theta)
    q /= np.linalg.norm(q)
    return quat_to_R(*q)


def camera_pose_from_local(msg, world_offset):
    """Convert this aircraft's MAVROS local ENU pose to the left-camera world pose."""
    p, o = msg.pose.position, msg.pose.orientation
    values = (p.x, p.y, p.z, o.x, o.y, o.z, o.w) + tuple(world_offset)
    if (not all(math.isfinite(float(v)) for v in values) or
            sum(float(v) * float(v) for v in (o.x, o.y, o.z, o.w)) < 0.5):
        raise ValueError("invalid MAVROS local pose")
    rotation = quat_to_R(o.x, o.y, o.z, o.w)
    camera_offset = rotation @ CAM_OFF_BL
    xyz = (p.x + world_offset[0] + camera_offset[0],
           p.y + world_offset[1] + camera_offset[1],
           (p.z + world_offset[2] + camera_offset[2]) * PZ_SCALE)
    return xyz + (rotation,)


def on_pose(msg, world_offset):
    global _pose_fault
    stamp = msg.header.stamp.to_sec()
    if stamp <= 0.0:
        stamp = rospy.Time.now().to_sec()
    if stamp <= 0.0:
        return
    try:
        offset = world_offset
        datum = globals().get('_height_datum')
        if datum is not None:
            o = msg.pose.orientation
            rotation = quat_to_R(o.x, o.y, o.z, o.w)
            tilt = math.acos(max(-1., min(1., float(rotation[2, 2]))))
            z_offset = datum.observe(stamp, msg.pose.position.z,
                                     globals().get('_height_armed'), tilt)
            if z_offset is None:
                return  # ground reference not established; no invented range
            offset = (world_offset[0], world_offset[1], z_offset)
            if not globals().get('_height_logged'):
                print('[pr] HEIGHT_DATUM offset_z=%.4f known_body_height=%.4f' %
                      (z_offset, datum.body_height), flush=True)
                globals()['_height_logged'] = True
        sample = (stamp,) + camera_pose_from_local(msg, offset)
    except (TypeError, ValueError, OverflowError):
        return
    with _lock:
        if _pose_fault is not None:
            return
        if _pose_history and stamp <= _pose_history[-1][0]:
            return  # duplicate/out-of-order sample must never rewind the camera
        if (_pose_history and stamp - _pose_history[0][0] >= 1.0 and
                min(sample[3], _pose_history[-1][3]) > 1.0):
            previous = _pose_history[-1]
            dt = stamp - previous[0]
            distance = math.sqrt(sum((sample[i] - previous[i]) ** 2
                                     for i in (1, 2, 3)))
            # A 0.8-2.4 m jump in one 30 ms EKF update was observed in the
            # failed flight.  It must not turn into a fresh camera report.
            # Allow ground estimator settling, then latch an in-flight fault.
            if distance > max(0.55, 8.0 * dt):
                _pose_fault = (stamp, distance, dt)
                _pose_history.clear()
                rospy.logerr("[pr] LOCALIZATION_FAULT pose jumped %.2fm in %.3fs; "
                             "camera reports disabled until restart", distance, dt)
                return
        _pose_history.append(sample)
        while len(_pose_history) > 2 and (
                len(_pose_history) > 180 or stamp - _pose_history[0][0] > 3.0):
            del _pose_history[0]


def on_img(msg):
    received_wall = time.time()
    received_ros = rospy.Time.now().to_sec()
    try:
        img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except Exception:
        return
    with _lock:
        _latest["img"] = img
        _latest["stamp"] = (msg.header.stamp.to_sec()
                             if msg.header.stamp.to_sec() > 0.0
                             else rospy.Time.now().to_sec())
        _latest["recv_wall"] = received_wall
        _latest["recv_ros"] = received_ros


def pose_at_stamp(stamp, fallback=None):
    """Interpolate a camera pose at image capture time, with <=50 ms endpoint slack."""
    if stamp <= 0.0:
        return None
    with _lock:
        if _pose_fault is not None:
            return None
        history = tuple(_pose_history)
    if not history:
        return None
    if len(history) == 1:
        return history[0][1:] if abs(stamp - history[0][0]) <= 0.05 else None
    if stamp < history[0][0] - 0.05 or stamp > history[-1][0] + 0.05:
        return None
    if stamp < history[0][0]:
        first, second = history[:2]
    else:
        first, second = history[-2:]
        for earlier, later in zip(history, history[1:]):
            if earlier[0] <= stamp <= later[0]:
                first, second = earlier, later
                break
    dt = second[0] - first[0]
    if dt <= 1e-6:
        return None
    alpha = (stamp - first[0]) / dt
    xyz = tuple(first[i] + alpha * (second[i] - first[i]) for i in (1, 2, 3))
    # Rotation must refer to the same image time as translation.
    return xyz + (interpolate_rotation(first[4], second[4], alpha),)


def ground_position_uncertainty(height, sin_down, slant_range, box_height):
    """Conservative sensitivity to 10 cm datum, 1 degree tilt and foot pixels.

    Used only for comparing calibrated camera sources, never as referee truth.
    Full-box foot uncertainty includes the existing 5% inset magnitude.
    """
    if not all(math.isfinite(v) for v in (height, sin_down, slant_range, box_height)):
        return None
    if height <= 0. or not .3 <= sin_down <= 1. or slant_range <= 0.:
        return None
    horizontal = slant_range*math.sqrt(max(0., 1.-sin_down*sin_down))
    angular = math.hypot(math.radians(1.), max(2., box_height*.05)/FY)
    return math.hypot(.10*horizontal/height, height*angular/(sin_down*sin_down))


def projected_body_height(foot_relative_world, y_top, rotation):
    """Vertical world segment through the measured foot, ending on the head row.

    Uses full perspective (different head/foot optical depths under pitch).
    The result is geometric evidence, not a claim that a detector box has
    exact anatomical endpoints. Slant range is reserved for range checks.
    """
    transform = np.asarray(rotation) @ M_OPT2LINK
    foot = transform.T @ np.asarray(foot_relative_world)
    vertical = transform.T @ np.array([0., 0., 1.])
    v = (float(y_top)-CY)/FY
    denominator = vertical[1]-v*vertical[2]
    if not np.isfinite(foot).all() or not np.isfinite(vertical).all() or abs(denominator) < 1e-6:
        return None
    height = (v*foot[2]-foot[1])/denominator
    head_depth = foot[2]+height*vertical[2]
    if not math.isfinite(height) or height <= 0. or head_depth <= .1:
        return None
    return float(height)


def clipped_head_ground(u, y_top, px, py, pz, rotation):
    """Estimate ground XY from a visible head when the feet are outside the image."""
    if y_top <= FEET_CLIP_PX or pz <= H_ASSUMED:
        return None
    head_ray = rotation @ (M_OPT2LINK @ np.array(
        [(u - CX) / FX, (y_top - CY) / FY, 1.0], dtype=float))
    if head_ray[2] >= -1e-5:
        return None
    scale = (H_ASSUMED - pz) / head_ray[2]
    if scale <= 0.0:
        return None
    wx, wy = px + scale * head_ray[0], py + scale * head_ray[1]
    distance = math.hypot(wx - px, wy - py)
    return (wx, wy, distance) if 0.5 < distance < MAX_RANGE else None


def person_likelihood(h):
    """身高似然：越接近 H_REF 越像人。真人实测 1.70~2.11 -> 0.93~0.99；
    招牌 4.6 m 以上 -> <0.01。"""
    return math.exp(-((h - H_REF) / H_SIG) ** 2)


def extract_appearance_feat(img_crop, bins=16):
    """提取检测框图像的外观特征：HSV颜色直方图
    
    原理：红色/白色目标在HSV空间有显著差异，可用于外观关联辅助
    - 红色：H通道集中在0和180附近
    - 白色：S通道低，V通道高
    - 其他颜色：有独特的H通道分布
    
    参数:
        img_crop: 检测框裁剪的BGR图像
        bins: 直方图bin数，默认16（兼顾区分度与计算效率）
    返回:
        归一化的特征向量 (bins*3,) 或 None（图像无效时）
    """
    if img_crop is None or img_crop.size == 0:
        return None
    try:
        h, w = img_crop.shape[:2]
        if h < 10 or w < 10:
            return None
        # 转换到HSV空间
        hsv = cv2.cvtColor(img_crop, cv2.COLOR_BGR2HSV)
        # 分别计算H、S、V通道直方图
        hist_h = cv2.calcHist([hsv], [0], None, [bins], [0, 180])
        hist_s = cv2.calcHist([hsv], [1], None, [bins], [0, 256])
        hist_v = cv2.calcHist([hsv], [2], None, [bins], [0, 256])
        # 归一化
        cv2.normalize(hist_h, hist_h, 0, 1, cv2.NORM_MINMAX)
        cv2.normalize(hist_s, hist_s, 0, 1, cv2.NORM_MINMAX)
        cv2.normalize(hist_v, hist_v, 0, 1, cv2.NORM_MINMAX)
        # 拼接成特征向量
        feat = np.concatenate([hist_h.flatten(), hist_s.flatten(), hist_v.flatten()]).astype(np.float32)
        return feat
    except Exception:
        return None


def appearance_similarity(feat1, feat2):
    """计算两个外观特征的余弦相似度
    
    原理：余弦相似度对光照变化更鲁棒，适合跨帧目标匹配
    返回: 相似度 [0, 1]，0表示完全不相似，1表示完全相同
    """
    if feat1 is None or feat2 is None:
        return 0.0
    norm1 = np.linalg.norm(feat1)
    norm2 = np.linalg.norm(feat2)
    if norm1 < 1e-6 or norm2 < 1e-6:
        return 0.0
    return float(np.dot(feat1, feat2) / (norm1 * norm2))


def cam_rel(ux, uy, uyaw, ox, oy):
    """把物体世界坐标 (ox,oy) 换成【相机水平系】坐标（平移 + 反向偏航）。

    之所以用相机系而不是世界系：机身与相机是刚性连接，它在相机系里**恒定不动**，
    而任何真实物体都会因为相机平移/转向而产生视差。见 VERDICT_ATTACH_ON 的注释。
    """
    dx, dy = ox - ux, oy - uy
    c, s = math.cos(-uyaw), math.sin(-uyaw)
    return (c * dx - s * dy, s * dx + c * dy)


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Track(object):
    """alpha-beta 匀速跟踪器：状态 (x, y, vx, vy) + 身高 H + 外观特征（v4 新增多层次关联）。"""
    _seq = 0

    def __init__(self, cls, x, y, conf, uv, wh, rng, h, t, uav=(0.0, 0.0, 0.0), appearance_feat=None, observation_s=None, loc_ok=True, range_method='foot_ground', appearance_status='actor', distractor_confidence=0., position_uncertainty_m=None):
        """appearance_feat: 外观特征向量（颜色直方图/深度特征），用于外观关联"""
        Track._seq += 1
        self.id = Track._seq
        self.cls = cls
        self.x, self.y = x, y
        self.vx, self.vy = 0.0, 0.0
        self.t = t
        self.last_seen_t = t
        self.observed_s = t if observation_s is None else observation_s
        self.hits = 1
        self.miss = 0
        self.conf = conf
        self.appearance_status = appearance_status
        self.distractor_confidence = float(distractor_confidence)
        self.uv, self.wh, self.rng = uv, wh, rng
        self.h = h                      # 归一化身高 (m)
        self.loc_suspect = not loc_ok
        self.observation_loc_ok = bool(loc_ok)
        self.observation_view_ok = bool(loc_ok)
        self.last_strict_s = self.observed_s if loc_ok else -float('inf')
        self.world_ready = False
        self.range_method = range_method
        self.position_uncertainty_m = position_uncertainty_m
        self.loc_good_streak = 3 if loc_ok else 0
        self.x0, self.y0, self.t0 = x, y, t   # 起点，用于算净位移速度
        self.sp = 0.0                   # 净位移速度 (m/s)：真人 ~1，静止误检 ~0
        self.max_disp = 0.0             # 生命期内最大位移 (m)：专治"走一段就停下的人"
        # 视差判据用：出生时的相机位姿与物体在相机系下的位置
        self.uav = uav
        self.uav0 = uav
        self.rel0 = cam_rel(uav[0], uav[1], uav[2], x, y)
        self.d_rel = 0.0                # 物体在相机系里的位移（真物体有视差，机身≈0）
        self.d_view = 0.0               # 相机自身视点变化量
        self.score = conf * person_likelihood(h) * MOTION_FLOOR
        self.score_ema = self.score     # 平滑后的打分，用于选轨（避免单帧抖动抢位）
        self._pub_sample = None
        self._pub_result = None
        self.pub_ema = None             # 对外发布坐标的 EMA（降低抖动，保 15s 连续<1m）
        # 只供裁判近距裁脚框使用。移动目标的头顶框抖动会造成 >1m
        # 深度误差；静止目标在边界仍需保留可上报的机会。
        self.recent_observed_xy = [(self.observed_s, x, y)]
        # v4 新增：外观特征（颜色直方图/深度特征）用于多层次关联
        self.appearance = appearance_feat if appearance_feat is not None else np.zeros(32, dtype=np.float32)
        self.appear_updated = 0         # 外观特征更新帧数

    def predict(self, dt):
        # v24b：前推速度限幅 2.2m/s（actor 物理上限）——YOLO 跳变差分可产生
        # 10m/s+ 假速度，LAG_COMP 0.3s 前推就偏 3m+，裁判 far_dist 清零。
        _sp = math.hypot(self.vx, self.vy)
        if _sp > 2.2:
            _k = 2.2 / _sp
            return self.x + self.vx * _k * dt, self.y + self.vy * _k * dt
        return self.x + self.vx * dt, self.y + self.vy * dt

    def observation_xy(self):
        """Unshifted measurement at observed_s for the timestamped bridge input.

        Association state may already have coasted to a processing-loop time;
        pub_xy also applies legacy prediction/smoothing. Neither is a raw
        camera measurement at the image timestamp.
        """
        stamp, x, y = self.recent_observed_xy[-1]
        if stamp != self.observed_s:
            raise ValueError('camera position and image timestamp disagree')
        return float(x), float(y)

    def pub_xy(self):
        """**对外发布**用的世界坐标 —— 先按 LAG_COMP 沿速度前推，抵消链路相位滞后。

        为什么要单独一个方法而不是直接用 self.x/self.y：裁判拿"接收瞬间的真值"
        和我们比（score_cal_master.py L136），而我们的坐标落后链路滞后那么多秒，
        目标一走就成了系统性偏差，且正比于速度。详见 LAG_COMP 的定义处注释。

        内部状态（关联、选优、诊断）**照旧用 self.x/self.y**，只在出口处偏置，
        这样不会污染滤波与判决逻辑。

        最后再过一道 PUB_EMA 指数平滑：裁判要求上报坐标与真值连续 15s <1m，
        实测 brown 坐标单帧跳 1.8m（(0.55,-40.13)→(-1.28,-40.35)），alpha-beta
        滤波后仍有残差；在出口处轻量 EMA 把抖动压到 <1m，避免 count_flag 反复重置。
        """
        # Output frequency and coast callbacks cannot re-weight one image.
        if self._pub_sample == self.observed_s:
            return self._pub_result
        self._pub_sample = self.observed_s
        if LAG_COMP > 0.0:
            px, py = self.predict(LAG_COMP)
        else:
            px, py = self.x, self.y
        if PUB_EMA > 0.0:
            if self.pub_ema is None:
                self.pub_ema = (px, py)
            else:
                ex, ey = self.pub_ema
                self.pub_ema = (PUB_EMA * ex + (1.0 - PUB_EMA) * px,
                                PUB_EMA * ey + (1.0 - PUB_EMA) * py)
            self._pub_result = self.pub_ema
        else:
            self._pub_result = (px, py)
        return self._pub_result

    def update(self, zx, zy, dt, conf, uv, wh, rng, h, t, uav=None, appearance_feat=None, observation_s=None, loc_ok=True, range_method='foot_ground', appearance_status='actor', distractor_confidence=0., position_uncertainty_m=None, view_ok=None):
        """更新track状态，可选更新外观特征用于多层次关联"""
        sample_s = t if observation_s is None else observation_s
        if t <= self.t or sample_s <= self.observed_s or dt <= 0.0:
            return False
        observation_dt = sample_s - self.observed_s
        self.observed_s = sample_s
        self.observation_loc_ok = bool(loc_ok)
        self.observation_view_ok = bool(loc_ok if view_ok is None else view_ok)
        if loc_ok:
            self.last_strict_s = sample_s
        self.appearance_status = appearance_status
        self.distractor_confidence = float(distractor_confidence)
        self.range_method = range_method
        self.position_uncertainty_m = position_uncertainty_m
        self.recent_observed_xy = [p for p in self.recent_observed_xy
                                   if sample_s - p[0] <= 2.5]
        self.recent_observed_xy.append((sample_s, zx, zy))
        px, py = self.predict(dt)
        rx, ry = zx - px, zy - py
        self.x = px + ALPHA * rx
        self.y = py + ALPHA * ry
        # Coast advances the state clock between images. Its last loop step
        # is not the measurement interval: dividing the image residual by
        # that step made identical detections alternate between +/- V_MAX
        # when the loop ran faster. Predict over the remaining state dt,
        # but correct velocity over elapsed time between real images.
        if observation_dt > 1e-3:
            self.vx += BETA * rx / observation_dt
            self.vy += BETA * ry / observation_dt
        sp = math.hypot(self.vx, self.vy)
        if sp > V_MAX:
            self.vx *= V_MAX / sp
            self.vy *= V_MAX / sp
        self.conf, self.uv, self.wh, self.rng = conf, uv, wh, rng
        self.h = h
        self.score = conf * person_likelihood(h) * self.motion_factor(t)
        self.score_ema = 0.7 * self.score_ema + 0.3 * self.score
        self.t, self.hits, self.miss = t, self.hits + 1, 0
        self.last_seen_t = t
        if self.loc_suspect:
            self.loc_good_streak = self.loc_good_streak + 1 if loc_ok else 0
            if self.loc_good_streak >= 3:
                self.loc_suspect = False
        if uav is not None:
            self.uav = uav
        # v4: 更新外观特征（使用滑动平均融合新特征，避免单帧噪声）
        if appearance_feat is not None:
            if self.appear_updated == 0:
                self.appearance = appearance_feat.copy()
            else:
                # EMA融合：越新的特征权重越高
                self.appearance = 0.7 * self.appearance + 0.3 * appearance_feat
            self.appear_updated += 1
        self.remember_travel()
        self.remember_parallax()
        return True

    def can_report_image(self):
        """Track longevity cannot make this frame's bad geometry trustworthy."""
        return bool(self.appearance_status == 'actor' and
                    self.miss == 0 and self.observation_loc_ok and
                    not self.loc_suspect and official_view_ok(
                        self.uv, self.wh, IMG_W, IMG_H, self.rng,
                        clipped_stable=self.clipped_report_stable()))

    def world_candidate_ready(self):
        """Establish one real image sequence before the first world dispatch.

        Lifetime hits can include isolated boxes seconds apart. The 17:32
        scaffold false track reached three hits this way and claimed a UAV.
        Established tracks retain identity across brief occlusion; this check
        never grants current-frame geometry or official reporting eligibility.
        """
        if self.world_ready:
            return True
        samples = []
        for item in reversed(self.recent_observed_xy):
            if self.observed_s-item[0] > 1.1 or (samples and samples[-1][0]-item[0] > .7):
                break
            samples.append(item)
        samples.reverse()
        if (len(samples) >= 3 and samples[-1][0]-samples[0][0] >= .3-1e-9 and
                all(0. < b[0]-a[0] <= .7 and
                    math.hypot(b[1]-a[1], b[2]-a[2]) <= .6+2.5*(b[0]-a[0])
                    for a, b in zip(samples, samples[1:]))):
            self.world_ready = True
        return self.world_ready

    def clipped_report_stable(self):
        """Only trust a cropped head ray after a stationary, sustained view."""
        samples = self.recent_observed_xy
        if len(samples) < 4 or samples[-1][0] - samples[0][0] < 1.2:
            return False
        xs = [p[1] for p in samples]
        ys = [p[2] for p in samples]
        # The 19:35 brown actor walked about 1 m/s while its cropped-foot
        # estimates alternated between y=4 and y=5.2; passing those samples
        # to the judge reset the mandatory 15 s streak.
        return (math.hypot(xs[-1] - xs[0], ys[-1] - ys[0]) < 0.45 and
                max(math.hypot(x - xs[-1], y - ys[-1])
                    for x, y in zip(xs, ys)) < 0.75)

    def stationary_candidate_stable(self):
        """Short, tight observation series for dispatch, never for range repair."""
        # Preserve the original 5 Hz four-frame window of .6 s. Selecting
        # only .5 s at higher rates would call a 1 m/s walk stationary under
        # the unchanged .55 m displacement gate. Select by image time so a
        # faster detector neither disables nor weakens this exception.
        # Keep the shortest qualified suffix so old ranging jitter ages out.
        history = self.recent_observed_xy
        if len(history) < 4:
            return False
        start = len(history) - 4
        while start > 0 and history[-1][0] - history[start][0] < .6 - 1e-9:
            start -= 1
        samples = history[start:]
        if (not all(math.isfinite(v) for p in samples for v in p) or
                not 0. <= self.t - samples[-1][0] <= .5 or
                samples[-1][0] - samples[0][0] < .6 - 1e-9 or
                any(not 0. < b[0] - a[0] <= .7 + 1e-9
                    for a, b in zip(samples, samples[1:]))):
            return False
        xs = [p[1] for p in samples]
        ys = [p[2] for p in samples]
        return (math.hypot(xs[-1] - xs[0], ys[-1] - ys[0]) < 0.55 and
                max(math.hypot(x - xs[-1], y - ys[-1])
                    for x, y in zip(xs, ys)) < 0.85)

    def remember_travel(self):
        """记录生命期内到出生点的最大位移（静止误检只有框抖动带来的零点几米）"""
        d = math.hypot(self.x - self.x0, self.y - self.y0)
        if d > self.max_disp:
            self.max_disp = d

    def remember_parallax(self):
        """记录相机系下的物体位移 d_rel 与相机自身视点变化 d_view（见 VERDICT_ATTACH_ON）"""
        ux, uy, uyaw = self.uav
        rx, ry = cam_rel(ux, uy, uyaw, self.x, self.y)
        self.d_rel = math.hypot(rx - self.rel0[0], ry - self.rel0[1])
        dyaw = wrap_pi(uyaw - self.uav0[2])
        self.d_view = (math.hypot(ux - self.uav0[0], uy - self.uav0[1])
                       + self.rng * abs(dyaw))

    def attached_to_cam(self):
        """物体是不是"长在机身上"的（视差判据）。视点变化不够大时判不了，返回 False=不表态。"""
        if self.d_view < ATTACH_MIN_VIEW:
            return False
        # Nose-locked observation can keep a real, stationary person's image
        # coordinates almost fixed while the aircraft moves around it.  Its
        # independent world-space observations stay fixed, whereas a camera-
        # attached false box moves through the world with the aircraft.  Use
        # this independent evidence before applying the camera-relative gate.
        if self.stationary_candidate_stable():
            return False
        return self.d_rel < ATTACH_FRAC * self.d_view

    def coast(self, dt):
        self.x, self.y = self.predict(dt)
        self.t += dt
        self.miss += 1
        self.remember_travel()

    def motion_factor(self, now=None):
        """净位移速度归一到 [MOTION_FLOOR, 1]。
        真人 ~1 m/s -> 1.0；墙面/招牌等静态误检 -> 接近 MOTION_FLOOR。
        存活不足 1 s 时给中性值，避免新轨一出生就被压分。
        顺带把净速度存进 self.sp，供"是不是人"判决直接用。"""
        now = self.t if now is None else now
        dt = now - self.t0
        if dt < 1.0:
            self.sp = 0.0
            return 0.5 * (1.0 + MOTION_FLOOR)
        sp = math.hypot(self.x - self.x0, self.y - self.y0) / dt
        self.sp = sp
        return MOTION_FLOOR + (1.0 - MOTION_FLOOR) * min(sp / MOTION_REF, 1.0)

    def verdict(self, now=None, max_coast=None, attach_check=True,
                motion_independent=False):
        """「是不是人」判决 —— 只用于「要不要画这个框 / 要不要发这个坐标」。

        attach_check=False：跳过「长在机身上」视差判据。盘旋确认时必须关闭 —
        该机头持续 yaw 对准目标，演员在相机系恒居中（d_rel≈0）而视点持续移动
        （d_view 增大），真目标会被误判成机身附着物，造成官方话题 6~11s 断流
        （2026-10-01 实测复盘）。

        ⚠ 注意它与"链路"的分工：这里**只管输出闸门**，不影响关联与新建。
        道理见 v3.0 的教训：过滤太狠会让 track 频繁死亡重建，
        连续 15 s 里断一帧就全清零，断流比误差致命。
        所以判决放在**最后一步**：链路照旧宽松地跟，输出时才严格筛。

        返回 (是否通过, 未通过的原因)。带原因是为了能统计到底哪一条卡住了
        —— 调参时最怕"不知道是谁拦的"。
        """
        if max_coast is None:
            max_coast = MAX_COAST_PUB
        if self.hits < CONFIRM_HITS:
            return False, "hits"
        if self.loc_suspect:
            return False, "localization"
        if self.miss > max_coast:
            return False, "coast"
        # 视差判据放在几何判据之前：它是"这东西长在机身上"的铁证，
        # 而机身的归一化身高恰好会落在人头区间（实测 1.63~2.32 m），几何拦不住。
        if attach_check and VERDICT_ATTACH_ON and self.attached_to_cam():
            return False, "self"
        if not (VERDICT_H_MIN <= self.h <= VERDICT_H_MAX):
            return False, "height"
        if self.rng > VERDICT_RANGE_MAX:
            return False, "range"
        if motion_independent:
            # A stopped actor and a sculpture can both have zero velocity.
            # Classification is appearance evidence; retain every physical
            # gate above and the same score floor without the motion penalty.
            if self.conf < (WHITE_CONF if self.cls == 'white' else CONF):
                return False, "confidence"
            if self.conf * person_likelihood(self.h) < VERDICT_SCORE:
                return False, "score"
            return True, ""
        self.motion_factor(now)                 # 刷新 self.sp
        static_near = (STATIC_OK and self.hits >= STATIC_OK_HITS and
                       self.conf >= STATIC_OK_CONF and
                       (self.rng <= STATIC_OK_RANGE or
                        (self.cls in ("blue", "green", "white", "brown") and
                         self.rng <= 12.0 and self.conf >= 0.80 and
                         self.stationary_candidate_stable())))
        # 当前在动，或者生命期内动过 —— 见 VERDICT_DISP 的注释（治 actor 卡死）
        if self.sp < VERDICT_SP and self.max_disp < VERDICT_DISP:
            # 静止例外（我方 10-02）：近距离、已确认、像人的静止目标照样发布，
            # 否则这套 world 里静止的 actor 永远进不了发布通道，裁判的
            # "连续 15s、误差<1m" 攒不满 ⇒ 0 分。远处静态误检继续被距离门挡住。
            if not static_near:
                return False, "static"
        # score_ema 本身乘了 motion_factor。边界真演员静止后，即使近距
        # 例外已经放行，0.84 置信度也会被运动因子压到 <0.20，重新判成噪声。
        # 仅对连续稳定、近距且高置信度的静止人体免去这一重复运动门槛。
        static_score_ok = (static_near and self.conf >= STATIC_OK_CONF and
                           (self.clipped_report_stable() or
                            (self.cls in ("blue", "green", "white", "brown") and
                             self.conf >= 0.80 and
                             self.stationary_candidate_stable())))
        if self.score_ema < VERDICT_SCORE and not static_score_ok:
            return False, "score"
        return True, ""

    def is_person(self, now=None, max_coast=None):
        return self.verdict(now, max_coast)[0]


def assign_slots(cands, slots, confirm_hits, slot_xy=None, sticky_r=0.0):
    """把候选 track **稳定**绑定到固定槽位上（red 双流用）。

    为什么不能逐帧"分高的发 red1"：裁判的 red callback 有一句
        if red_cnt == 2 and flag_1 != 0:  count_flag[flag_1] = False
    只要这一帧的坐标离 actor_4、actor_5 都超过 1 m 就 +1，累计到 2 就把
    "连续 15 s"计时清零。槽位若在帧间互换（这一次 A→red1、下一次 B→red1），
    每条流都在报"别人的"坐标，计时被反复清零，永远到不了 15 s。

    规则（顺序不可颠倒）：
      1) **旧绑定优先** —— 只要该槽锁定的 track 还在候选里，就绝不换槽；
      2) 空槽补位分两级（2026-09-15 补的第二级）：
         a. **空间连续性优先** —— 先挑离该槽"最后已知世界坐标"最近的候选
            （距离 < sticky_r 才认）。这一步是被实测数据逼出来的：
            槽位绑的是 track，而 track 会因检测断续频繁重建（实测 tid
            在 129/131/134 之间反复换），按"分数最高"补位时，新 track
            **有可能落在另一个红衣人身上** —— 于是 red1 流一会儿指 actor_4、
            一会儿指 actor_5（实测 red1 归属 261:224 几乎对半），
            双流同时有效覆盖只有 1.5 s。
         b. 没有空间上说得通的候选时，才退回按 (已确认, 打分, 命中数) 排序。
      3) 一个 track 至多占一个槽。

    cands    : list[Track]，本帧通过基础过滤的候选（内部排序，不改原列表）
    slots    : list，元素为该槽锁定的 track id 或 None；**原地更新**
    slot_xy  : 与 slots 等长，记录该槽最后已知世界坐标 (x,y) 或 None；
               **原地更新**（槽空出来后仍保留最后一个值，供下次补位判连续）
    sticky_r : 空间连续性半径 (m)。取 0 表示关闭该级（退回纯打分）。
               两个红衣人相距约 14 m，6 m 既够跨过一次重建，又不会串人。
    返回     : [(槽号, track)]，按槽号升序
    """
    order = sorted(cands, key=lambda t: (t.hits >= confirm_hits, t.score_ema, t.hits),
                   reverse=True)
    by_id = {t.id: t for t in order}
    bound, used = [], set()
    for si in range(len(slots)):                  # 1) 沿用旧绑定
        tid = slots[si]
        t = by_id.get(tid) if tid is not None else None
        if t is not None and tid not in used:
            bound.append((si, t))
            used.add(tid)
        else:
            slots[si] = None                      # track 没了 -> 释放槽（坐标留着）
    for si in range(len(slots)):                  # 2) 空槽按序补
        if slots[si] is not None:
            continue
        pick = None
        if sticky_r > 0 and slot_xy is not None and slot_xy[si] is not None:
            best, bd = None, 1e9
            for t in order:
                if t.id in used:
                    continue
                d = math.hypot(t.x - slot_xy[si][0], t.y - slot_xy[si][1])
                if d < bd:
                    best, bd = t, d
            if best is not None and bd < sticky_r:
                pick = best
        if pick is None:
            for t in order:
                if t.id in used:
                    continue
                pick = t
                break
        if pick is not None:
            slots[si] = pick.id
            bound.append((si, pick))
            used.add(pick.id)
    if slot_xy is not None:                       # 刷新每个槽的最后位置
        for si, t in bound:
            slot_xy[si] = (t.x, t.y)
    bound.sort(key=lambda p: p[0])
    return bound


def image_world_bearing(u, v, rotation):
    ray = rotation @ (M_OPT2LINK @ np.array([(u-CX)/FX, (v-CY)/FY, 1.]))
    return math.atan2(ray[1], ray[0])


def control_tracks(tracks, red_slots, sample):
    """Image steering candidates, independent of world-report/person verdicts.

    Keep same-colour alternatives; the assigned agent performs spatial
    association. Red slot identity must already be established, never guessed.
    """
    result = []
    for tr in tracks:
        if (tr.miss or tr.observed_s != sample or tr.hits < 3 or
                not .45 <= tr.conf <= 1.):
            continue
        if tr.cls == 'red':
            if tr.id not in red_slots:
                continue
            slot = red_slots.index(tr.id)
        else:
            if tr.cls not in TID_OF_COLOR:
                continue
            slot = 0
        result.append((tr.cls, slot, tr))
    return result


def candidate_identity(session, track):
    return '%s:%s' % (session, track.id)


def fast_tracker_seed_frame(cues, source, sample, session):
    """YOLO pixel seeds for a read-only experiment; no location or task credit.

    Called once per independently inferred image. These are measured boxes,
    not coasted world tracks. The shadow worker must match the original image
    timestamp before tracking into newer frames.
    """
    detections = []
    for cue in cues:
        if (cue.get('appearance_status', 'actor') != 'actor' or
                float(cue.get('sample_s', -1.)) != sample):
            continue
        box = cue.get('box')
        confidence = cue.get('confidence')
        if (box is None or len(box) != 4 or
                not all(math.isfinite(float(v)) for v in box) or
                not 0. < float(confidence) <= 1.):
            continue
        detections.append(dict(
            id='%s:%s' % (session, cue['candidate_id']),
            label=cue['candidate_id'].split(':', 1)[0],
            box=list(box), confidence=float(confidence)))
    return dict(schema=1, source_uav=source, image_stamp=sample,
                purpose='shadow_only', real_detection=True,
                control_eligible=False, report_eligible=False,
                detections=detections)


def appearance_fields(track):
    return dict(appearance_status=getattr(track, 'appearance_status', 'actor'),
                appearance_confidence=float(track.conf),
                distractor_confidence=float(getattr(track, 'distractor_confidence', 0.)))


def candidate_frame(source, sample, candidates, purpose=None):
    """One complete camera frame; neither callback nor detector order ranks it."""
    payload = dict(schema_version=2, source_uav=source, sample_s=float(sample),
                   real_detection=True, candidates=sorted(
                       candidates, key=lambda c: (c['target_id'], c['candidate_id'])))
    if purpose is not None:
        payload['purpose'] = purpose
    return payload


def report_candidate_frame(entries, source, sample, session, observer_xy,
                           assigned_target=None):
    """Preserve all eligible instances; the bridge owns colour arbitration.

    Entries retain established red slot identities. Coordinates are fresh raw
    image measurements, never coasting or the legacy display extrapolation.
    """
    candidates = []
    for cls, slot, track in entries:
        if (track.miss or track.observed_s != sample or track.hits < MIN_HITS or
                getattr(track, 'appearance_status', 'actor') == 'distractor' or
                not track.world_candidate_ready()):
            continue
        x, y = track.observation_xy()
        tag = 'red%d' % (slot+1) if cls == 'red' else cls
        cid = candidate_identity(session, track)
        conf = max(0., min(1., float(track.conf)))
        fields = appearance_fields(track)
        person_ok = track.verdict(sample, max_coast=0, attach_check=not
                                  skip_attached_verdict(cls, assigned_target, track.rng),
                                  motion_independent=True)[0]
        candidates.append(dict(
            target_id=tag, candidate_id=cid, frame_id='world_enu',
            xyz=[round(x, 2), round(y, 2), TARGET_Z], confidence=conf,
            visual_confidence=round(float(track.conf), 3),
            track_confirmed=bool(report_ready_hits(cls, track.hits)),
            sample_s=float(sample), source_uav=source, real_detection=True,
            position_uncertainty_m=getattr(track, 'position_uncertainty_m', None),
            track_hits=int(track.hits), range_m=round(math.hypot(
                x-observer_xy[0], y-observer_xy[1]), 2),
            official_view_ok=bool(person_ok and report_ready_hits(cls, track.hits) and
                                  track.can_report_image()),
            observation_id='obs-%s-%s-%.9f' % (source, cid, sample), **fields))
    return candidate_frame(source, sample, candidates)


def discovery_candidate_frame(entries, source, sample, session):
    candidates = []
    for cls, slot, track in entries:
        if (cls == 'red' or track.miss or track.observed_s != sample or
                not track.observation_loc_ok or track.loc_suspect or
                track.conf < .55 or track.hits < 4 or not 0. < track.rng <= 12. or
                track.observed_s-track.t0 < .5 or
                getattr(track, 'appearance_status', 'actor') == 'distractor' or
                not track.world_candidate_ready()):
            continue
        x, y = track.observation_xy()
        candidates.append(dict(
            source_uav=source, target_id=TID_OF_COLOR[cls],
            candidate_id=candidate_identity(session, track),
            box=box_from_foot(track.uv, track.wh),
            real_detection=True, sample_s=float(sample), confidence=float(track.conf),
            hits=track.hits, x=x, y=y, range_m=float(track.rng), geometry_ok=True,
            official_view_ok=False, purpose='discovery_only', **appearance_fields(track)))
    return candidate_frame(source, sample, candidates, purpose='discovery_only')


def main():
    global _model
    rospy.init_node("perception_real", anonymous=True)
    try:
        world_offset = tuple(float(os.environ[key]) for key in (
            "PR_WORLD_OFFSET_X", "PR_WORLD_OFFSET_Y")) + (
                float(os.environ.get("PR_WORLD_OFFSET_Z", "0.0")),)
        if not all(math.isfinite(v) for v in world_offset):
            raise ValueError("non-finite offset")
    except (KeyError, ValueError) as exc:
        raise RuntimeError(
            "perception requires PR_WORLD_OFFSET_X/Y from the official UAV spawn "
            "coordinates; refusing unanchored world reports (%s)" % exc)

    # ---- 官方话题发布仲裁状态：本机当前被指派追踪的目标 tid ----
    # 收到 task_type=1/2 且 target_id 非空 -> 持有发布权；
    # 收到搜索任务/其他指派 -> 立即撤销（与 agent 状态机一致）。
    arb = {"tid": None}
    arb_lock = threading.Lock()
    # 目标融合状态缓存：tid -> (x, y, stamp_secs)
    tstates = {}
    tstate_lock = threading.Lock()

    def _arb_cb(msg):
        if str(msg.uav_id) != str(UAV):
            return
        with arb_lock:
            if msg.task_type in (1, 2) and msg.target_id:
                arb["tid"] = str(msg.target_id)
            else:
                arb["tid"] = None

    def _tstate_cb(msg):
        with tstate_lock:
            tstates[str(msg.target_id)] = (
                float(msg.x), float(msg.y), msg.header.stamp.to_sec())

    print("[pr] 裁判上报门控：距离门=%.0fm | 确认门=%s（hits>=%d）| 协同上报 confidence=%s"
          % (ACTOR_PUB_RANGE,
             "ON" if ACTOR_CONFIRM_ONLY else "off",
             CONFIRM_HITS,
             "原始检测概率；track_confirmed独立"), flush=True)
    if SearchAssignment is not None:
        rospy.Subscriber("/swarm/assignment", SearchAssignment, _arb_cb)
        print("[pr] 指派订阅已连接；仲裁开关=%s，直发官方开关=%s"
              % (bool(OFFICIAL_ARBITRATED), bool(DIRECT_ACTOR_INFO)), flush=True)
    else:
        print("[pr] WARN: 未导入 SearchAssignment（%s），官方话题仅过 %.0fm 距离闸门"
              % (_search_assign_import_err, ACTOR_PUB_RANGE), flush=True)
    if TargetState is not None:
        rospy.Subscriber("/swarm/target_states", TargetState, _tstate_cb)
        print("[pr] 空间身份闸门=%s (<=%.1fm, 新鲜<=%.0fs)"
              % (bool(IDENTITY_GATE), IDENTITY_M, TSTATE_FRESH_S), flush=True)

    _load_wall = time.monotonic()
    print("[pr] 加载权重 %s" % WEIGHTS, flush=True)
    print("[pr] 双目左目适配 | 图像 %s | 本机位姿 %s | 世界偏移 %s | "
          "相机外参 %s | 内参 fx=%.1f cx=%.1f cy=%.1f 图 %dx%d"
          % (CAM_TOPIC, POSE_TOPIC, world_offset, CAM_OFF_BL.tolist(),
             FX, CX, CY, IMG_W, IMG_H), flush=True)
    print("[pr] 类别顺序 %s" % CLASSES, flush=True)
    from ultralytics import YOLO
    _model = YOLO(WEIGHTS)
    configure_cpu_inference_threads(_model, TORCH_THREADS)
    class_mapping = target_class_mapping(_model.names)
    from distractor_verifier import SecondaryAppearance
    secondary_appearance = SecondaryAppearance(configure_model=lambda model:
        configure_cpu_inference_threads(model, TORCH_THREADS))
    print('[pr] CLASS_MAP names=%s canonical=%s; non-target classes excluded' %
          (_model.names, class_mapping), flush=True)
    print("[pr] MODEL_READY load_s=%.3f" % (time.monotonic() - _load_wall),
          flush=True)

    # pubs[类][槽号]：单流类只有 1 个槽，red 有 2 个（red1/red2）
    # 2026-10-06 修：仅 PR_DIRECT_ACTOR_INFO=1 时才创建 /actor_*_info Publisher。
    # rospy.Publisher 创建即 advertise —— 此前 6 个感知节点即使从不直发（默认 0，
    # 交给 bridge 融合后统一发布），也被 rostopic info 计成发布者 ⇒ 启动审计
    # "发布者数量为 7（要求唯一 yolo_target_bridge）" 误判 → 启动中断全场被杀。
    # 槽位数改用 NSLOTS 供 tag 命名 / 标注逻辑使用（与 pubs 解耦）。
    NSLOTS = {c: len(topics) for c, topics in TOPIC_OF.items()}
    if DIRECT_ACTOR_INFO:
        pubs = {c: [rospy.Publisher(t, ActorInfo, queue_size=1) for t in topics]
                for c, topics in TOPIC_OF.items()}
    else:
        pubs = None
    # Schema 2 is one complete frame; the bridge selects physical hypotheses.
    coord = rospy.Publisher("/coordination/target_report", String, queue_size=50)
    cue_pub = rospy.Publisher("/swarm/visual_candidate", String, queue_size=6)
    frame_pub = rospy.Publisher("/swarm/processed_camera_frame", String, queue_size=6)
    view_pub = rospy.Publisher("/swarm/tracking_view", String, queue_size=6)
    discovery_pub = rospy.Publisher("/swarm/discovery_observation", String, queue_size=30)
    view_csv = logger('tracking_view_%s' % UAV, [
        'ros_time', 'sample_s', 'target_id', 'track_id', 'x1', 'y1', 'x2', 'y2',
        'camera_x', 'camera_y', 'camera_z', 'qx', 'qy', 'qz', 'qw',
        'target_x', 'target_y', 'range_method', 'geometry_ok', 'report_ok'])
    # 调试通道：一帧一条、含全部 dets 的快照，供 rec_snap.py / analyze_snap.py 离线复盘
    dbg = rospy.Publisher(DEBUG_TOPIC, String, queue_size=5)
    fast_seed_pub = (rospy.Publisher('/perception/fast_tracker_seeds', String,
                                    queue_size=2)
                     if FAST_TRACKER_SHADOW else None)
    # YOLO 实时视角（带检测框，调试可视化）
    yolo_view_pub = (rospy.Publisher(YOLO_VIEW_TOPIC, Image, queue_size=1)
                     if YOLO_VIEW else None)

    rospy.Subscriber(CAM_TOPIC, Image, on_img, queue_size=1)
    if int(os.environ.get('PR_HEIGHT_CALIBRATE', '0')):
        from mavros_msgs.msg import State
        globals()['_height_datum'] = GroundHeightDatum(float(os.environ['PR_GROUND_BODY_HEIGHT']))
        def height_state(msg):
            globals()['_height_armed'] = bool(msg.armed)
            globals()['_height_datum'].set_armed(bool(msg.armed))
        rospy.Subscriber('/%s/mavros/state' % UAV, State, height_state, queue_size=1)
    rospy.Subscriber(POSE_TOPIC, PoseStamped, on_pose,
                     callback_args=world_offset, queue_size=100)

    tracks = []
    # red 双流的槽位绑定：red_slots[i] = 该槽当前锁定的 track id（None = 空）。
    # 绑定只在 track 死亡/跟丢时才释放，绝不逐帧重排（原因见 TOPIC_OF 注释）。
    # red_slot_xy[i] = 该槽最后已知世界坐标，供"空间连续性"补位（见 assign_slots）。
    red_slots = [None] * len(TOPIC_OF["red"])
    red_slot_xy = [None] * len(TOPIC_OF["red"])
    # 断流诊断（只记 red，且只在状态**变化**时打一行，不会刷屏）。
    # 为什么必须单独记：汇总统计只能看到"无归属率 21%""最大间隔 8.8s"这两个**结果**，
    # 看不出断流发生在什么时刻、槽位当时是掉空了还是换到了别人身上 ——
    # 而这两者的修法完全不同（前者要查遮挡/量程，后者要查槽位误换）。
    red_prev_tid = [None] * len(TOPIC_OF["red"])
    red_silent_t = [None] * len(TOPIC_OF["red"])
    red_diag_t = [0.0] * len(TOPIC_OF["red"])
    print("[pr] v4.0(双目) 几何+运动+人判决+red双流 | conf=%.2f white_conf=%.2f  detect=%.0fHz  pub=%.0fHz"
          % (CONF, WHITE_CONF, PUB_HZ / DETECT_EVERY, PUB_HZ), flush=True)
    print("[pr] 人判决: 标注%s 发布%s | H∈[%.2f,%.2f] 净速度≥%.2fm/s 打分≥%.2f 画框要求coast≤%d"
          % ("ON" if ANNOT_VERDICT else "off", "ON" if PUB_VERDICT else "off",
             VERDICT_H_MIN, VERDICT_H_MAX, VERDICT_SP, VERDICT_SCORE, ANNOT_MAX_COAST),
          flush=True)
    print("[pr] 严格档(新建) H∈[%.2f,%.2f] AR∈[%.2f,%.2f] | 宽松档(hits>=%d) "
          "H∈[%.2f,%.2f] AR∈[%.2f,%.2f]"
          % (H_MIN, H_MAX, AR_MIN, AR_MAX, CONFIRM_HITS,
             H_MIN_L, H_MAX_L, AR_MIN_L, AR_MAX_L), flush=True)
    print("[pr] H_ref=%.2f±%.2f H_gate=%.2f | 跟踪 alpha=%.2f beta=%.2f gate=%.1fm "
          "min_hits=%d timeout=%.1fs"
          % (H_REF, H_SIG, H_GATE, ALPHA, BETA, GATE, MIN_HITS, TRACK_TIMEOUT), flush=True)
    print("[pr] 帧间跳变门限(已确认 track) %s: gate=min(%.1f, %.1f + %.1f·dt)"
          % ("ON" if GATE_DYN else "off", GATE, GATE_MIN, GATE_MULT), flush=True)

    rate = rospy.Rate(PUB_HZ)
    n_loop = 0
    _t_live = 0.0            # 心跳上次打印墙钟（5s 一次，证明主线程活着）
    _last_live_loop = 0      # 实际循环频率；配置频率不能代表 CPU 满载时的吞吐
    _last_live_det = 0
    coord_clock = ImagePublishClock(COORD_HZ)
    timing_csv = logger('pipeline_timing_%s' % UAV, [
        'image_stamp', 'receive_ros_s', 'inference_start_ros_s', 'report_ros_s',
        'receive_wall_s', 'inference_start_wall_s', 'inference_end_wall_s',
        'geometry_end_wall_s', 'report_wall_s', 'inference_s', 'secondary_inference_s', 'image_age_s',
        'candidate_count', 'qualified_count'])
    candidate_session = uuid.uuid4().hex
    annot_img = None          # 最近一次参与检测的图像，供标注帧使用
    annot_z = 0.0             # 该帧对应的相机高度
    frame_stamp = 0.0         # 当前检测图像的 ROS 采集时间
    processed_image_stamp = 0.0
    cue_tracker = PixelCueTracker()
    appearance_resolver = ActorDistractorResolver()
    frame_age = None
    inference_s = None
    _first_inference_logged = False
    uav = (0.0, 0.0, 0.0)     # 最近一次取到的相机位姿 (x, y, 偏航)，供视差判据使用
    # 相机完整位姿 (x,y,z, qx,qy,qz,qw)：只写进调试快照，供**离线**复算
    # "真值目标本该出现在画面哪个像素"。为什么必须落盘：2026-09-15 brown 轮里
    # 某个框从 30px 一路膨胀到 88px、H 从 2.05m 涨到 2.88m、D 从 12.9m 掉到 6.7m，
    # 光看 uv/wh 无法判定它是"无人机自身机体"还是"真人框被遮挡/合并"——
    # 两者的修法完全相反（前者要加自身判决，后者要修几何）。有了相机位姿就能算：
    #   真人在该帧的应有像素 (u*,v*) 与框的 (u,v) 差多少、框底反投到地面的点在哪。
    cam_full = None
    # 最后一次有效的飞机水平位置 (x, y)。清理段要用它判"目标是否离飞机过远"；
    # 位姿取失败时不更新 —— 否则那一帧会把 (0,0) 当成飞机位置，把 track 全清空。
    uav_last = None
    _annot_t = 0.0            # 上次存标注帧的墙钟时刻
    if ANNOT_ON:
        try:
            os.makedirs(ANNOT_DIR, exist_ok=True)
            if ANNOT_RAW:
                os.makedirs(ANNOT_RAW_DIR, exist_ok=True)
            print("[pr] 标注帧 -> %s (TOP=%d every=%.1fs 高度>%.1fm%s%s)" %
                  (ANNOT_DIR, ANNOT_TOP, ANNOT_EVERY, ANNOT_MIN_Z,
                   " 只看" + ANNOT_CLS if ANNOT_CLS else "",
                   " +原图->" + ANNOT_RAW_DIR if ANNOT_RAW else ""), flush=True)
        except Exception as e:
            print("[pr] 建标注目录失败: %s" % e, flush=True)

    while not rospy.is_shutdown():
        n_loop += 1
        now = rospy.Time.now().to_sec()
        dets = []
        raw_cue_boxes = []
        frame_cues = []
        frame_was_processed = False

        # ---------- 1) 检测 + 几何过滤 ----------
        if n_loop % DETECT_EVERY == 0:
            with _lock:
                img = None if _latest["img"] is None else _latest["img"].copy()
                frame_stamp = _latest["stamp"]
                frame_receive_wall = _latest['recv_wall']
                frame_receive_ros = _latest['recv_ros']
            frame_age = max(0.0, now - frame_stamp) if frame_stamp > 0.0 else None
            if frame_stamp <= processed_image_stamp or frame_age is None or frame_age > 0.5:
                img = None
            else:
                processed_image_stamp = frame_stamp
            if img is not None:
                try:
                    pose = pose_at_stamp(frame_stamp)
                    if pose is None:
                        raise ValueError("no MAVROS camera pose at image timestamp %.3f"
                                         % frame_stamp)
                    px, py, pz, R = pose
                    # 相机水平偏航（视差判据要用）：取旋转矩阵第一列的水平分量
                    yaw_cam = math.atan2(R[1, 0], R[0, 0])
                    uav = (px, py, yaw_cam)
                    uav_last = (px, py)          # 供清理段判"离飞机过远"
                    q = R_to_quat(R)
                    cam_full = [round(px, 3), round(py, 3), round(pz, 3),
                                *(round(v, 5) for v in q)]
                    # 位姿取到了才更新标注帧：与检测同帧，保证所见即所判
                    annot_img, annot_z = img, pz
                except Exception as e:
                    rospy.logwarn_throttle(5.0, "[pr] 图像位姿同步失败: %s", str(e))
                    R, (px, py, pz) = None, (0, 0, 0)
                    uav = (0.0, 0.0, 0.0)
                    cam_full = None

                if R is not None:
                    inference_start_ros = rospy.Time.now().to_sec()
                    _infer_t0 = time.time()
                    res = _model(img, conf=min(CONF, WHITE_CONF),
                                 agnostic_nms=False, verbose=False)[0]
                    frame_was_processed = True
                    inference_end_wall = time.time()
                    inference_s = inference_end_wall - _infer_t0
                    if not _first_inference_logged:
                        print("[pr] FIRST_INFERENCE image_stamp=%.3f latency_s=%.3f"
                              % (frame_stamp, inference_s), flush=True)
                        _first_inference_logged = True
                    # 几何、轨迹和上报共用图像采集时刻 frame_stamp；推理耗时
                    # 只用于评估延迟，不能再从采集时间减去半个推理周期。
                    # v4新增：对同类别的检测结果做IOU去重（解决YOLO同一目标多次检测问题）
                    actor_boxes, appearance_evidence = appearance_resolver.resolve(
                        res.boxes, class_mapping, _model.names, frame_stamp)
                    secondary_start = time.monotonic()
                    try:
                        actor_boxes, secondary_evidence = secondary_appearance.filter(
                            img, actor_boxes, CLASSES, frame_stamp)
                        appearance_evidence.extend(secondary_evidence)
                        if any(e['status'] == 'secondary_distractor'
                               for e in secondary_evidence):
                            rospy.loginfo_throttle(2.,
                                '[DISTRACTOR_VETO] uav=%s image=%.3f mode=%s; three fresh images',
                                UAV, frame_stamp, secondary_appearance.mode)
                    except Exception as exc:
                        # Never reuse old negative evidence after an inference
                        # failure, or suppress primary actors on missing data.
                        secondary_appearance.histories.clear()
                        rospy.logwarn_throttle(2., '[DISTRACTOR_FALLBACK] %s',
                                               type(exc).__name__)
                    secondary_inference_s = time.monotonic() - secondary_start
                    for evidence in appearance_evidence:
                        ex1, ey1, ex2, ey2 = evidence['box']
                        _CSV.write(ros_time=rospy.Time.now().to_sec(),
                            event='appearance_evidence', class_name='distractor_person',
                            confidence=evidence['confidence'],
                            bbox_u=(ex1+ex2)/2., bbox_v=(ey1+ey2)/2.,
                            bbox_w=ex2-ex1, bbox_h=ey2-ey1, image_stamp=frame_stamp,
                            source=evidence['status'])
                    boxes_by_class = {}
                    for b in actor_boxes:
                        cid = int(b.cls)
                        if cid not in boxes_by_class:
                            boxes_by_class[cid] = []
                        boxes_by_class[cid].append(b)
                    # 对每个类别分别做NMS
                    filtered_boxes = []
                    for cid, boxes_list in boxes_by_class.items():
                        if len(boxes_list) == 1:
                            filtered_boxes.extend(boxes_list)
                            continue
                        # 按置信度排序
                        boxes_list = sorted(boxes_list, key=lambda x: float(x.conf), reverse=True)
                        keep = []
                        for bi, b in enumerate(boxes_list):
                            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
                            area = (x2 - x1) * (y2 - y1)
                            keep_b = True
                            for kb in keep:
                                kx1, ky1, kx2, ky2 = [float(v) for v in kb.xyxy[0]]
                                # 计算IOU
                                inter_x1, inter_y1 = max(x1, kx1), max(y1, ky1)
                                inter_x2, inter_y2 = min(x2, kx2), min(y2, ky2)
                                if inter_x2 > inter_x1 and inter_y2 > inter_y1:
                                    inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
                                    union_area = area + (kx2 - kx1) * (ky2 - ky1) - inter_area
                                    iou = inter_area / max(union_area, 1e-6)
                                    if iou > 0.85:  # IOU阈值
                                        keep_b = False
                                        break
                            if keep_b:
                                keep.append(b)
                        filtered_boxes.extend(keep)
                    # A single white shirt produced nearly identical blue and
                    # white boxes in the 13:52 match.  Same-class NMS cannot
                    # remove them; use visible torso color only when decisive.
                    filtered_boxes = filter_blue_white_duplicate_boxes(img, filtered_boxes)
                    # Associate boxes before ground projection: a cropped or
                    # unusual person can be kept in sight without treating an
                    # unstable ray as a dispatchable world coordinate.
                    for b in filtered_boxes:
                        cid = int(b.cls)
                        conf = float(b.conf)
                        if cid < 0 or cid >= len(CLASSES):
                            continue
                        bx1, by1, bx2, by2 = [float(v) for v in b.xyxy[0]]
                        bw, bh = bx2 - bx1, by2 - by1
                        ordinary_cue = pixel_cue_box_ok(CLASSES[cid], conf,
                                                        bx1, by1, bx2, by2)
                        if (not ordinary_cue and not pixel_cue_box_ok(CLASSES[cid], conf,
                                bx1, by1, bx2, by2, allow_small_continuation=True)):
                            continue
                        bu, bv = (bx1 + bx2) / 2.0, (by1 + by2) / 2.0
                        centre_ray = foot_ground_ray(bu, bv, R)
                        raw_cue_boxes.append(dict(
                            cls=CLASSES[cid], conf=conf, u=bu, v=bv,
                            small_box_continuation_only=not ordinary_cue,
                            h=bh, bearing=image_world_bearing(bu, bv, R),
                            elevation=math.atan2(float(centre_ray[2]),
                                math.hypot(float(centre_ray[0]), float(centre_ray[1]))),
                            cam_x=px, cam_y=py, box=[bx1, by1, bx2, by2],
                            appearance_status=b.appearance_status,
                            appearance_confidence=b.appearance_confidence,
                            distractor_confidence=b.distractor_confidence,
                            body_height_m=cue_body_height(
                                (bx1, by1, bx2, by2), pz, R),
                            width=IMG_W, height=IMG_H,
                            camera_tilt=math.acos(max(-1., min(1., float(R[2, 2]))))))
                    frame_cues = cue_tracker.update(frame_stamp, raw_cue_boxes)
                    if fast_seed_pub is not None:
                        fast_seed_pub.publish(String(data=json.dumps(
                            fast_tracker_seed_frame(frame_cues, UAV, frame_stamp,
                                                    candidate_session))))
                    for rejection in cue_tracker.rejections:
                        _CSV.write(ros_time=rospy.Time.now().to_sec(),
                            event='candidate_geometry_reject',
                            class_name=rejection['cls'], confidence=rejection['confidence'],
                            height_m=rejection['height_m'], image_stamp=frame_stamp,
                            image_age_s=frame_age, source='three_full_body_contradictions')
                    # 遍历过滤后的boxes
                    for b in filtered_boxes:
                        cid = int(b.cls)
                        conf = float(b.conf)
                        if conf < (WHITE_CONF if cid == 3 else CONF):
                            continue
                        x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
                        if y2 < ROI_TOP or y1 > ROI_BOT:
                            continue
                        _stat["n_det"] += 1

                        u = (x1 + x2) / 2.0
                        # 用框真实底部 y2，不截断到 ROI_BOT：
                        # 目标在远处时视线接近水平，v_world[2]≈0，t=-pz/v_world[2] 很大，
                        # 框底部 2 像素的截断会被放大成 >1m 的坐标偏差（实测导致官方不认）。
                        # ROI_BOT 仍用于 L794 的过滤（框完全在底部带则跳过）。
                        # uv/wh 保留旧的内缩表示，供框还原、图像跟踪与裁脚判据使用。
                        # 测距只使用实际框底 y2，不能再用这项经验补偿覆盖高度零点。
                        v = float(y2) - 0.05 * float(y2 - y1)
                        v_world = foot_ground_ray(u, float(y2), R)
                        if v_world[2] >= -1e-6:
                            continue
                        # 2026-10-06 八轮复盘：俯角护栏。视线接近水平时（俯角小），
                        # t=-pz/v_world[2] 的地面交点对 pz/姿态误差极度敏感，
                        # 实测低空（1.9m）跟踪时偏差 ±7-10m。俯角正弦 <0.30
                        # （俯角 <17.5°，如 4m 高 13m 外 / 1.9m 高 6m 外）一律拒绝：
                        # 几何不可靠的观测宁可不报，也不给裁判喂错位置。
                        sin_down = (-v_world[2]) / float(np.linalg.norm(v_world))
                        if sin_down < 0.30:
                            _stat["n_pitch_rej"] = _stat.get("n_pitch_rej", 0) + 1
                            _CSV.write(
                                ros_time=rospy.Time.now().to_sec(), event='projection_reject',
                                class_name=CLASSES[cid], confidence=conf,
                                bbox_u=u, bbox_v=v, bbox_w=float(x2-x1),
                                bbox_h=float(y2-y1), image_stamp=frame_stamp,
                                image_age_s=frame_age, inference_s=inference_s,
                                sin_down=sin_down, range_method='foot_ground',
                                source='sin_down_below_0.30')
                            continue
                        t = -pz / v_world[2]
                        if t <= 0:
                            continue
                        # 【2026-09-19 几何修正】v_opt 第 3 分量固定为 1，所以
                        # |v_world| = |v_opt| >= 1：t 只是"沿视线的参数"，**不是距离**。
                        # 画面角落 (u=752,v=480, fx=376,cx=376,cy=240) 处 |v_opt|=1.55，
                        # 于是 t<=60 实际放行到 93 m —— 这正是历史误报里世界坐标能
                        # 冲到 100 m+ 的来源。这里显式换算成真实欧氏距离再判量程。
                        n_v = float(np.linalg.norm(v_world))
                        rng = t * n_v
                        if rng > MAX_RANGE:
                            continue

                        # --- v3 核心：把框的像素高换算成实际身高 ---
                        # 注意：这里只打标、不丢弃。是否采用由关联/新建阶段按档位决定
                        # （v3.0 在这里硬丢，结果把真人的框也丢了，链路断得比 v2 还狠）
                        w_px, h_px = x2 - x1, y2 - y1
                        impl_h = projected_body_height(t*v_world, y1, R)
                        if impl_h is None:
                            continue  # pixel candidate still survives for bounded reimaging
                        ar = h_px / max(1e-3, w_px)
                        # ---- 我方 10-02：脚被画面下沿裁掉时，改用身高反推距离 ----
                        # 见 FX/FY 处的说明：近目标双脚在画面外，"双脚落地"假设失效，
                        # 会把 7m 的目标报成 10.6m（位置偏 2m ⇒ 裁判 1m 判据必挂）。
                        if FEET_CLIP_PX > 0.0 and h_px > 4.0 \
                                and h_px >= FEET_CLIP_MIN_PX \
                                and y2 >= (img.shape[0] - FEET_CLIP_PX):
                            # 下沿截断时 h_px 只是可见身高，不能反推完整身高距离。
                            # 用仍可见的头顶射线与人体高度平面求交；头顶也截断则放弃。
                            head = clipped_head_ground(u, y1, px, py, pz, R)
                            if head is None:
                                continue
                            # 后续统一计算 wx/wy；头顶估计直接覆盖地面落点。
                            head_x, head_y, ground_dist = head
                            t = None
                            rng = ground_dist
                            impl_h = H_ASSUMED
                        # 远距离小框的 AR 上限单独放宽（见 AR_MAX_SMALL 处的实测记录）
                        _ar_max = AR_MAX_SMALL if h_px < SMALL_BOX_H_PX else AR_MAX
                        strict_base = ((H_MIN <= impl_h <= H_MAX) and
                                  (AR_MIN <= ar <= _ar_max)) or \
                                 near_cropped_strict_ok(t is None, rng, conf,
                                                        impl_h, ar) or \
                                 near_narrow_person_strict_ok(
                                     CLASSES[cid], rng, conf, impl_h, ar, h_px,
                                     x1, x2, y1, img.shape[1])
                        cue_assisted = (not strict_base and
                            pixel_promoted_strict(
                                frame_cues, CLASSES[cid], u, (y1 + y2) / 2.0,
                                conf, rng, impl_h, ar, h_px,
                                x1, x2, y1, img.shape[1]))
                        strict = strict_base or cue_assisted
                        loose = (H_MIN_L <= impl_h <= H_MAX_L) and (AR_MIN_L <= ar <= AR_MAX_L)
                        if not loose and not strict:        # 连获准的新建档都过不了 -> 纯噪声
                            _stat["n_geo_rej"] += 1
                            continue

                        if t is None:
                            wx, wy = head_x, head_y
                        else:
                            wx = px + t * v_world[0]
                            wy = py + t * v_world[1]
                        # v4 新增：提取外观特征（HSV颜色直方图）用于多层次关联
                        img_crop = img[int(y1):int(y2), int(x1):int(x2)] if x2 > x1 and y2 > y1 else None
                        appearance_feat = extract_appearance_feat(img_crop) if img_crop is not None else None
                        dets.append({"cls": CLASSES[cid] if cid < len(CLASSES) else "?",
                                     "conf": round(conf, 3),
                                     "uv": [round(u, 1), round(v, 1)],
                                     "wh": [round(w_px, 1), round(h_px, 1)],
                                     "xyz": [round(wx, 2), round(wy, 2)],
                                     "range": round(rng, 2),
                                     "h": round(impl_h, 2),
                                     "strict": strict,
                                     "loc_ok": strict_base,
                                     "sin_down": float(sin_down),
                                     # Geometry sensitivity is a source ranking metric,
                                     # not a certified absolute error bound. No quality
                                     # takeover is enabled for assumed-height crops.
                                     "position_uncertainty_m": (
                                         ground_position_uncertainty(pz, sin_down, rng, h_px)
                                         if t is not None and globals().get('_height_datum') is not None
                                         and globals()['_height_datum'].offset is not None else None),
                                     "range_method": 'head_height' if t is None else 'foot_ground',
                                     "score": round(conf * person_likelihood(impl_h), 3),
                                     "appearance_status": b.appearance_status,
                                     "distractor_confidence": b.distractor_confidence,
                                     "appearance": appearance_feat})
                        _CSV.write(
                            ros_time=rospy.Time.now().to_sec(), event="yolo_detection",
                            class_name=CLASSES[cid] if cid < len(CLASSES) else "?",
                            confidence=conf, bbox_u=u, bbox_v=v, bbox_w=w_px,
                            bbox_h=h_px, target_x=wx, target_y=wy,
                            target_z=TARGET_Z, range_m=rng, height_m=impl_h,
                            strict=strict, image_stamp=frame_stamp,
                            image_age_s=frame_age, inference_s=inference_s,
                            sin_down=sin_down,
                            range_method='head_height' if t is None else 'foot_ground',
                            source="yolo_raw")

        if frame_was_processed:
            geometry_end_wall = time.time()
            frame_pub.publish(String(data=json.dumps({
                'source_uav': UAV, 'sample_s': frame_stamp,
                'x': px, 'y': py, 'z': pz, 'yaw': yaw_cam,
                'down_pitch': math.atan2(-R[2, 0],
                    math.hypot(R[0, 0], R[1, 0])),
                'processed': True})))
            for cue in frame_cues:
                cue.update(source_uav=UAV, real_detection=True,
                           purpose='camera_reacquire')
                cue_pub.publish(String(data=json.dumps(cue)))
                _CSV.write(
                    ros_time=rospy.Time.now().to_sec(), event='camera_candidate',
                    class_name=cue['candidate_id'].split(':', 1)[0],
                    confidence=cue['confidence'], bbox_u=cue['u'], bbox_v=cue['v'],
                    bbox_w=cue['box'][2]-cue['box'][0] if 'box' in cue else '',
                    bbox_h=cue['box'][3]-cue['box'][1] if 'box' in cue else '',
                    track_id=cue['candidate_id'], hits=cue['hits'],
                    image_stamp=frame_stamp, image_age_s=frame_age,
                    source=('pixel_small_continuation_only' if cue.get('small_box_continuation')
                            else 'pixel_attention_only'))

        # ---------- 2) 关联（位置 + 身高 + 外观；v4新增多层次匹配）----------
        # 原理：先用位置粗关联，再用身高和外观精细筛选，解决密集目标串扰问题
        used = [False] * len(dets)
        # 外观相似度权重（国赛创新点：颜色特征辅助关联）
        for tk in tracks:
            dt = max(1e-3, now - tk.t)
            tx, ty = tk.predict(dt)
            # 已确认 track 用 dt 缩放门限（见 GATE_DYN 注释）：走路的人 0.1 s 挪不了 3 m
            gate = assoc_gate(dt, tk.hits)
            best, bd = -1, gate
            best_appear_score = 0.0
            confirmed = tk.hits >= CONFIRM_HITS      # 已确认 -> 宽松档，保链路
            dyn_rej = 0
            for j, d in enumerate(dets):
                if used[j] or d["cls"] != tk.cls:
                    continue
                if not confirmed and not d["strict"]:
                    continue                          # 未确认 track 只吃严格档
                if abs(d["h"] - tk.h) > H_GATE:       # 身高突变 -> 不是同一个东西
                    continue
                dd = math.hypot(d["xyz"][0] - tx, d["xyz"][1] - ty)
                # v4新增：外观相似度计算（余弦相似度）
                appear_sim = appearance_similarity(tk.appearance, d.get("appearance"))
                # 综合评分：距离越近、外观越相似分数越高
                # 位置门限内的候选才考虑外观
                if dd < gate:
                    score = association_score(dd, appear_sim, gate, confirmed)
                    if score < bd:
                        best, bd, best_appear_score = j, score, appear_sim
                elif confirmed and dd >= gate:
                    dyn_rej += 1                      # 被动态门限拒掉：统计用
                    _CSV.write(
                        ros_time=rospy.Time.now().to_sec(), event='association_reject',
                        class_name=tk.cls, confidence=d['conf'], track_id=tk.id,
                        hits=tk.hits, target_x=d['xyz'][0], target_y=d['xyz'][1],
                        bbox_u=d['uv'][0], bbox_v=d['uv'][1],
                        bbox_w=d['wh'][0], bbox_h=d['wh'][1],
                        image_stamp=frame_stamp, image_age_s=frame_age,
                        range_method=d['range_method'], assoc_residual_m=dd,
                        assoc_gate_m=gate, observation_gap_s=frame_stamp-tk.observed_s,
                        source='same_class_pair_outside_gate')
            if dyn_rej:
                _stat["n_dyn_rej"] = _stat.get("n_dyn_rej", 0) + dyn_rej
            if best >= 0:
                d = dets[best]
                used[best] = True
                _stat["n_assoc"] += 1
                # v4：统计外观关联使用情况
                appear_feat = d.get("appearance")
                if appear_feat is not None:
                    _stat["n_appear_feat"] = _stat.get("n_appear_feat", 0) + 1
                # v4：传递外观特征用于后续关联
                view_ok = continuation_view_ok(
                    tk, d, frame_stamp,
                    math.hypot(d['xyz'][0] - tx, d['xyz'][1] - ty))
                if view_ok and not d.get('loc_ok', False):
                    _CSV.write(ros_time=now, event='tracking_view_continuation',
                               class_name=tk.cls, confidence=d['conf'],
                               track_id=tk.id, hits=tk.hits,
                               image_stamp=frame_stamp, image_age_s=frame_age,
                               target_x=d['xyz'][0], target_y=d['xyz'][1],
                               source='confirmed_full_body_shape_change')
                tk.update(d["xyz"][0], d["xyz"][1], dt, d["conf"],
                          d["uv"], d["wh"], d["range"], d["h"], now, uav,
                          appearance_feat=appear_feat, observation_s=frame_stamp,
                          loc_ok=d.get('loc_ok', False), view_ok=view_ok,
                          range_method=d['range_method'],
                          appearance_status=d['appearance_status'],
                          distractor_confidence=d['distractor_confidence'],
                          position_uncertainty_m=d.get('position_uncertainty_m'))
            else:
                tk.coast(dt)

        # ---------- 2b) 新建（只认严格档，误检建不了轨）----------
        for j, d in enumerate(dets):
            if not used[j]:
                if not d["strict"]:
                    _stat["n_new_rej"] += 1
                    continue
                _stat["n_new"] += 1
                # v4：创建track时保存外观特征
                tracks.append(Track(d["cls"], d["xyz"][0], d["xyz"][1], d["conf"],
                                    d["uv"], d["wh"], d["range"], d["h"], now, uav,
                                    appearance_feat=d.get("appearance"),
                                    observation_s=frame_stamp,
                                    loc_ok=d.get('loc_ok', True), range_method=d['range_method'],
                                    appearance_status=d['appearance_status'],
                                    distractor_confidence=d['distractor_confidence'],
                          position_uncertainty_m=d.get('position_uncertainty_m')))

        # ---------- 3) 清理 ----------
        # 原判据 math.hypot(tk.x, tk.y) 是"距世界原点"的距离，与飞机在哪无关：
        # 飞机飞到 (-57,25) 后，靠近原点的合法目标会被误删，而正前方 100 m 外的
        # 误检反被留下（"MAX_RANGE*2=120" 也就是这么来的）。改成"距本帧飞机位置"，
        # 并用最后一次有效位姿，避免位姿取失败那帧拿 (0,0) 当飞机位置清空全部 track。
        if uav_last is None:
            tracks = [tk for tk in tracks if (now - tk.last_seen_t) < TRACK_TIMEOUT]
        else:
            _ux, _uy = uav_last
            tracks = [tk for tk in tracks
                      if (now - tk.last_seen_t) < TRACK_TIMEOUT
                      and math.hypot(tk.x - _ux, tk.y - _uy) < MAX_RANGE]

        # ---------- 4) 发布 ----------
        # Keep every physical candidate for the bridge. best_of is retained
        # only for the legacy display/direct-output path.
        # red（双流）：走槽位持久绑定，两条流各锁一个红衣人（理由见 TOPIC_OF 注释）。
        best_of = {}
        report_pool = []
        red_cands = []
        with arb_lock:
            _cur_tid_for_verdict = arb["tid"]
        for tk in tracks:
            if tk.hits < MIN_HITS:
                continue
            if tk.miss > MAX_COAST_PUB:   # 跟丢太久，坐标已是纯外推，不让它代表本类发布
                continue
            # Retain the physical/person gates without using zero motion as
            # sculpture evidence. The legacy default verdict remains callable
            # for old diagnostics; appearance classification resolves decoys.
            if PUB_VERDICT:
                # 盘旋确认段跳过 attach 判据（见 verdict/ATTACH_SKIP_RANGE 注释）。
                # Red has two local slots and no entry in TID_OF_COLOR.
                # Looking it up there returned None, so even the assigned
                # red person was called "attached to camera" during a
                # nose-locked follow.  The 20:38 run lost its red track this
                # way after 89 hits, breaking the judge's 15-second streak.
                _skip_attach = skip_attached_verdict(
                    tk.cls, _cur_tid_for_verdict, tk.rng)
                _person_ok, _reject_reason = tk.verdict(
                    now, max_coast=MAX_COAST_PUB,
                    attach_check=not _skip_attach, motion_independent=True)
                if not _person_ok:
                    _CSV.write(
                        ros_time=rospy.Time.now().to_sec(), event="track_reject",
                        target_id=("%s%d" % (tk.cls, 0) if tk.cls != "red" else "red"),
                        class_name=tk.cls, confidence=tk.conf,
                        bbox_u=tk.uv[0], bbox_v=tk.uv[1], bbox_w=tk.wh[0],
                        bbox_h=tk.wh[1], target_x=tk.x, target_y=tk.y,
                        target_z=TARGET_Z, range_m=tk.rng, height_m=tk.h,
                        strict=True, track_id=tk.id, hits=tk.hits,
                        image_stamp=frame_stamp, image_age_s=frame_age,
                        inference_s=inference_s, source="verdict_%s" % _reject_reason)
                    continue
            # 运动性是随时间累积的，coast 期间净位移也在涨，这里按当前时刻重算
            tk.score = tk.conf * person_likelihood(tk.h) * tk.motion_factor(now)
            if tk.cls == "red":
                red_cands.append(tk)      # red 不在这里决出，交给下面的槽位绑定
                continue
            report_pool.append((tk.cls, 0, tk))
            cur = best_of.get(tk.cls)
            # v2  按 (hits, conf)    -> 偏向"持续存在的误检"（招牌 hits 也多）
            # v3.0按 (hits, score)   -> score 含身高似然，但 hits 仍优先
            # v3.2按 (score_ema, hits)：green 一轮暴露了问题 ——
            #      真目标 D≈12m conf 0.67~0.72，却被近处误检 D≈7m conf 0.45~0.60
            #      用 hits 优先抢了位。实测真目标的 conf 明显更高，
            #      所以让 score（conf × 身高似然）优先，hits 只作并列时的裁决。
            if cur is None or (tk.score_ema, tk.hits) > (cur.score_ema, cur.hits):
                best_of[tk.cls] = tk

        # --- red 双流槽位绑定（逻辑见 assign_slots）---
        bound = assign_slots(red_cands, red_slots, CONFIRM_HITS,
                             red_slot_xy, RED_STICKY_R)
        report_pool.extend(('red', si, tk) for si, tk in bound)

        # --- 断流/换轨诊断：让"某条流突然静默 8s"变成一个可归因的事件 ---
        for si in range(len(red_slots)):
            tid_now, tid_prev = red_slots[si], red_prev_tid[si]
            if tid_now == tid_prev:
                continue
            if now - red_diag_t[si] < 0.5:      # 换轨抖动时限流，避免刷屏
                continue
            red_diag_t[si] = now
            last_xy = red_slot_xy[si]
            loc = "无" if last_xy is None else "(%.1f,%.1f)" % (last_xy[0], last_xy[1])
            if tid_now is None:
                if red_silent_t[si] is None:
                    red_silent_t[si] = now
                print("[pr] red 断流 slot=%d 掉tid=%s 最后位置=%s | 原因=-"
                      "（无可用候选：被遮挡 / 超量程 / 跟丢 / 人判决拦下）"
                      % (si, tid_prev, loc), flush=True)
            elif tid_prev is None:
                gap = 0.0 if red_silent_t[si] is None else now - red_silent_t[si]
                red_silent_t[si] = None
                print("[pr] red 复流 slot=%d 新tid=%s 静默时长=%.2fs 位置=%s"
                      % (si, tid_now, gap, loc), flush=True)
            else:
                print("[pr] red 换轨 slot=%d %s -> %s（槽位没锁住，疑 track 重建）"
                      % (si, tid_prev, tid_now), flush=True)
            red_prev_tid[si] = tid_now

        pub_list = [(cls, 0, tk) for cls, tk in best_of.items()] + \
                   [("red", si, tk) for si, tk in bound]

        # Control sees all associated real images, before person/report verdicts
        # and per-colour winner selection. The assigned agent associates by
        # world position and keeps the nearest matching image for this frame.
        control_views = control_tracks(tracks, red_slots, frame_stamp)
        if frame_was_processed:
            discovery_batch = discovery_candidate_frame(
                control_views, UAV, frame_stamp, candidate_session)
            discovery_pub.publish(String(data=json.dumps(discovery_batch)))
            discovery_ids = {c['candidate_id'] for c in discovery_batch['candidates']}
            for cls, si, tk in control_views:
                tid = TID_OF_RED_SLOT[si] if cls == 'red' else TID_OF_COLOR[cls]
                box = box_from_foot(tk.uv, tk.wh)
                # uv/wh are rounded diagnostics; restore and bound sub-pixel
                # rounding at the image edge before the controller validates it.
                box = [max(0., min(float(IMG_W if i % 2 == 0 else IMG_H), v))
                       for i, v in enumerate(box)]
                u, v = (box[0]+box[2])/2., (box[1]+box[3])/2.
                ox, oy = tk.observation_xy()
                # Discovery is not a judge report. Preserve same-colour
                # hypotheses before best_of/verdict suppresses them. Red
                # identity continues through the bridge's two-person matcher.
                if candidate_identity(candidate_session, tk) in discovery_ids:
                    _CSV.write(ros_time=rospy.Time.now().to_sec(),event='discovery_report',
                        target_id=tid,class_name=cls,confidence=tk.conf,
                        bbox_u=tk.uv[0],bbox_v=tk.uv[1],bbox_w=tk.wh[0],bbox_h=tk.wh[1],
                        target_x=ox,target_y=oy,range_m=tk.rng,track_id=tk.id,hits=tk.hits,
                        image_stamp=tk.observed_s,source='discovery_only')
                view_pub.publish(String(data=json.dumps(dict(
                    source_uav=UAV, target_id=tid, track_id=tk.id,
                    candidate_id=bound_pixel_candidate(frame_cues,cls,box),
                    world_candidate_id=candidate_identity(candidate_session, tk),
                    real_detection=True, sample_s=tk.observed_s,
                    confidence=float(tk.conf), hits=tk.hits, box=box,
                    width=IMG_W, height=IMG_H, fy=FY,
                    camera_tilt=math.acos(max(-1., min(1., float(R[2, 2])))),
                    geometry_ok=bool(tk.observation_view_ok),
                    report_ok=bool(tk.can_report_image()),
                    bearing=image_world_bearing(u, v, R), x=ox, y=oy,
                    **appearance_fields(tk)))))
                view_csv.write(ros_time=rospy.Time.now().to_sec(), sample_s=tk.observed_s,
                    target_id=tid, track_id=tk.id, x1=box[0], y1=box[1], x2=box[2], y2=box[3],
                    camera_x=cam_full[0], camera_y=cam_full[1], camera_z=cam_full[2],
                    qx=cam_full[3], qy=cam_full[4], qz=cam_full[5], qw=cam_full[6],
                    target_x=ox, target_y=oy, range_method=tk.range_method,
                    geometry_ok=tk.observation_view_ok, report_ok=tk.can_report_image())

        # ---- 官方话题闸门：指派仲裁 + 近距离 + 空间身份一致 ----
        # 未过闸不发布 ActorInfo（在进入内部调试快照链路前剔除）。
        with arb_lock:
            cur_tid = arb["tid"]
        now_secs = rospy.Time.now().to_sec()
        gated_list = []
        for cls, si, tk in report_pool:
            need_tid = TID_OF_RED_SLOT[si] if cls == "red" else TID_OF_COLOR.get(cls)
            if tk.rng > ACTOR_PUB_RANGE:
                continue
            if ACTOR_CONFIRM_ONLY and not report_ready_hits(cls, tk.hits):
                continue
            if OFFICIAL_ARBITRATED and cur_tid != need_tid:
                continue
            if IDENTITY_GATE:
                with tstate_lock:
                    ts_ent = tstates.get(need_tid)
                if ts_ent is None or (now_secs - ts_ent[2]) > TSTATE_FRESH_S:
                    continue
                if math.hypot(tk.x - ts_ent[0], tk.y - ts_ent[1]) > IDENTITY_M:
                    continue
            gated_list.append((cls, si, tk))
        if len(gated_list) != len(report_pool):
            rospy.loginfo_throttle(
                5, "[pr] 官方话题闸门外挂 %d/%d 条（指派=%s, rng<=%.0fm）",
                len(report_pool) - len(gated_list), len(report_pool), cur_tid, ACTOR_PUB_RANGE)
        report_pool = gated_list
        pub_list = [entry for entry in pub_list if entry in report_pool and
                    entry[2].appearance_status == 'actor']

        snap = []
        for cls, si, tk in pub_list:
            px, py = tk.pub_xy()          # 补偿后的对外坐标（见 LAG_COMP / Track.pub_xy）
            # 2026-10-01: 本机不再直发 /actor_<color>_info，改由 yolo_target_bridge
            # 用多机融合坐标统一发布（单一仲裁者，避免 6 机同色位置冲突让裁判反复重置）。
            # 2026-10-02: 由 PR_DIRECT_ACTOR_INFO 控制（默认 0=交给 bridge；
            # 单机独立验收官方裁判时设 1，否则裁判收不到消息）。
            if DIRECT_ACTOR_INFO:
                # ---- 我方 10-03：发布坐标平滑（裁判判据是"每一条"消息误差 <1m）----
                # 实测：修完测距标定后位置误差只剩 0.26~0.69m（达标），但 30 帧内
                # 坐标标准差 0.38m、峰峰 1.45m ⇒ 偶发单帧越界就把裁判 15s 计时清零。
                # 这里只对外发布做 EMA + 单帧限速，**不动内部跟踪**（滤波已在 Track 内）。
                _key = (cls, si)
                _raw = (px, py)
                _prev = _pub_smooth.get(_key)
                if _prev is None:
                    _pub_smooth[_key] = _raw
                else:
                    _sx = PUB_SMOOTH_ALPHA * _raw[0] + (1.0 - PUB_SMOOTH_ALPHA) * _prev[0]
                    _sy = PUB_SMOOTH_ALPHA * _raw[1] + (1.0 - PUB_SMOOTH_ALPHA) * _prev[1]
                    _dx, _dy = _sx - _prev[0], _sy - _prev[1]
                    _d = math.hypot(_dx, _dy)
                    if _d > PUB_MAX_STEP:          # 单帧位移限速：真人 2m/s × 0.1s = 0.2m
                        k = PUB_MAX_STEP / _d
                        _sx, _sy = _prev[0] + _dx * k, _prev[1] + _dy * k
                    _pub_smooth[_key] = (_sx, _sy)
                m = ActorInfo(cls=cls, x=round(_sx, 2), y=round(_sy, 2))
                pubs[cls][si].publish(m)
            tag = ("%s%d" % (cls, si + 1)) if NSLOTS[cls] > 1 else cls
            snap.append({"cls": cls, "slot": si, "tag": tag, "conf": round(tk.conf, 3),
                         "uv": tk.uv, "wh": tk.wh,
                         # xyz = 真正发出去的值；xyz_est = 滤波估计值（未补偿）。
                         # 两者都留，便于离线验证补偿到底有没有用、以及追查异常。
                         "xyz": [round(px, 2), round(py, 2)],
                         "xyz_est": [round(tk.x, 2), round(tk.y, 2)],
                         "range": tk.rng, "h": tk.h, "score": round(tk.score, 3),
                         "s_ema": round(tk.score_ema, 3), "sp": round(tk.sp, 2),
                         "tid": tk.id, "hits": tk.hits, "coast": tk.miss})
            _CSV.write(
                ros_time=rospy.Time.now().to_sec(), event="track_publish",
                target_id=tag, class_name=cls, confidence=tk.conf,
                bbox_u=tk.uv[0], bbox_v=tk.uv[1], bbox_w=tk.wh[0], bbox_h=tk.wh[1],
                target_x=px, target_y=py, target_z=TARGET_Z, range_m=tk.rng,
                height_m=tk.h, strict=True, track_id=tk.id, hits=tk.hits,
                image_stamp=frame_stamp, image_age_s=frame_age,
                inference_s=inference_s,
                source="perception_track")

        # YOLO 实时视角：即使本帧没有有效目标也持续发布原图，避免 rqt
        # 画面在目标暂时丢失时冻结。检测框只作为当前帧的叠加层。
        if yolo_view_pub is not None and annot_img is not None:
            _view = annot_img.copy()
            for _s in snap:
                _u, _vv = _s["uv"]
                _ww, _hh = _s["wh"]
                _x1 = int(_u - _ww / 2)
                _y1 = int(_vv - _hh)
                _x2 = int(_u + _ww / 2)
                _y2 = int(_vv)
                _col = COLORS.get(_s["cls"], (0, 255, 255))
                cv2.rectangle(_view, (_x1, _y1), (_x2, _y2), _col, 2)
                cv2.putText(_view,
                            "%s (%.0f,%.0f)" % (_s["tag"],
                                                _s["xyz"][0], _s["xyz"][1]),
                            (_x1, max(12, _y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, _col, 1)
            # 本机 OpenCV4 与 noetic 旧 cv_bridge 编号不匹配，传 "bgr8"
            # 会 KeyError(16)；用 passthrough 转换后手动声明编码，数据布局一致。
            _msg = CvBridge().cv2_to_imgmsg(np.ascontiguousarray(_view))
            _msg.encoding = "bgr8"
            _msg.header.stamp = rospy.Time.now()
            _msg.header.frame_id = "%s/stereo_camera_left_frame" % UAV
            yolo_view_pub.publish(_msg)

        if snap:
            dbg.publish(String(data=json.dumps({
                "stamp": round(time.time(), 2),
                "cam": cam_full,
                "uav": UAV, "dets": snap})))
            if n_loop % 10 == 0:
                for s in snap:
                    print("[pr] %-6s c=%.2f H=%.2fm 世界=(%7.2f,%7.2f) D=%5.2fm v=%.2fm/s "
                          "tid=%d hits=%d coast=%d %s"
                          % (s["tag"], s["conf"], s["h"], s["xyz"][0], s["xyz"][1],
                             s["range"], s["sp"], s["tid"], s["hits"], s["coast"],
                             "CNF" if s["hits"] >= CONFIRM_HITS else "new"), flush=True)
                print("[pr] red 槽位=%s (槽0/red1 槽1/red2; None=空)" % red_slots, flush=True)
                print("[pr] 统计 检出=%d 噪声拦下=%d 关联成功=%d 新建=%d 新建被几何拒=%d "
                      "帧间跳变拒=%d 外观特征=%d"
                      % (_stat["n_det"], _stat["n_geo_rej"], _stat["n_assoc"],
                         _stat["n_new"], _stat["n_new_rej"], _stat.get("n_dyn_rej", 0),
                         _stat.get("n_appear_feat", 0)),
                      flush=True)
                if ANNOT_VERDICT and ANNOT_ON:
                    # 统计只在标注帧块里更新（ANNOT_ON=1）；关闭时打印会显示全 0，误导排查。
                    print("[pr] 人判决 通过=%d  未过: 命中不足=%d 跟丢=%d 附着机身=%d "
                          "身高不像=%d 超量程=%d 静止=%d 打分低=%d"
                          % (_stat.get("v_pass", 0), _stat.get("v_hits", 0),
                             _stat.get("v_coast", 0), _stat.get("v_self", 0),
                             _stat.get("v_height", 0), _stat.get("v_range", 0),
                             _stat.get("v_static", 0), _stat.get("v_score", 0)), flush=True)

        # One complete frame preserves competing instances before colour
        # arbitration. ImagePublishClock still limits actual image samples.
        if COORD_ON and frame_was_processed and coord_clock.ready(frame_stamp):
            report_batch = report_candidate_frame(
                report_pool, UAV, frame_stamp, candidate_session, uav_last,
                assigned_target=cur_tid)
            coord.publish(String(data=json.dumps(report_batch)))
            report_wall = time.time()
            report_ros = rospy.Time.now().to_sec()
            timing_csv.write(
                image_stamp=frame_stamp, receive_ros_s=frame_receive_ros,
                inference_start_ros_s=inference_start_ros, report_ros_s=report_ros,
                receive_wall_s=frame_receive_wall, inference_start_wall_s=_infer_t0,
                inference_end_wall_s=inference_end_wall,
                geometry_end_wall_s=geometry_end_wall, report_wall_s=report_wall,
                inference_s=inference_s, secondary_inference_s=secondary_inference_s,
                image_age_s=report_ros-frame_stamp,
                candidate_count=len(report_batch['candidates']),
                qualified_count=sum(c['official_view_ok'] for c in report_batch['candidates']))
            report_by_id = {c['candidate_id']: c for c in report_batch['candidates']}
            for _cls, _si, _tk in report_pool:
                report = report_by_id.get(candidate_identity(candidate_session, _tk))
                if report is None:
                    continue
                _CSV.write(
                    ros_time=now_secs, event="coord_report", target_id=report['target_id'],
                    class_name=_cls, confidence=report['confidence'],
                    bbox_u=_tk.uv[0], bbox_v=_tk.uv[1],
                    bbox_w=_tk.wh[0], bbox_h=_tk.wh[1],
                    target_x=report['xyz'][0], target_y=report['xyz'][1],
                    target_z=TARGET_Z, range_m=report['range_m'],
                    height_m=_tk.h, strict=report['official_view_ok'],
                    range_method=_tk.range_method, track_id=_tk.id, hits=_tk.hits,
                    image_stamp=_tk.observed_s,
                    image_age_s=now_secs-_tk.observed_s, inference_s=inference_s,
                    source="coordination/target_report")
                rospy.loginfo_throttle(
                    0.5, "[PERC] uav=%s target=%s candidate=%s xy=(%.2f,%.2f) "
                    "appearance=%s official=%s img_ts=%.3f",
                    UAV, report['target_id'], report['candidate_id'],
                    report['xyz'][0], report['xyz'][1], report['appearance_status'],
                    report['official_view_ok'], frame_stamp)

        # ---------- 5) 兼容原每类最优标注，仅用于诊断显示 ----------
        # 与正式人物判据一样，不能因为目标静止就隐藏其框。
        # 完整实例集合通过 report_pool 发布，由 bridge 决定报告资格；
        # 此处画框不表示已向裁判上报或已消除。
        if ANNOT_ON and annot_img is not None and annot_z >= ANNOT_MIN_Z:
            cands = []                     # [(类, 槽号, track)]
            for _cls, _si, _tk in pub_list:
                if ANNOT_CLS and _cls != ANNOT_CLS:
                    continue
                if ANNOT_VERDICT:
                    ok, why = _tk.verdict(
                        now, max_coast=ANNOT_MAX_COAST,
                        attach_check=not skip_attached_verdict(_cls, cur_tid, _tk.rng),
                        motion_independent=True)
                    if not ok:
                        _stat["v_" + why] = _stat.get("v_" + why, 0) + 1
                        continue
                    _stat["v_pass"] = _stat.get("v_pass", 0) + 1
                elif ANNOT_CONFIRM and _tk.hits < CONFIRM_HITS:
                    continue
                cands.append((_cls, _si, _tk))
            if cands:
                if ANNOT_TOP > 0:
                    # 单流类：每类只画分最高的 ANNOT_TOP 个。
                    # 多流类（red）：两条流各自就是上报给裁判的目标，全部画 ——
                    # 否则 red1 上框、red2 没框，图上看不出两个红衣人都跟上了。
                    keep = []
                    for _c in sorted(set(c[0] for c in cands)):
                        sub = [c for c in cands if c[0] == _c]
                        if NSLOTS[_c] > 1:
                            keep += sub
                        else:
                            keep += sorted(sub, key=lambda c: -c[2].score_ema)[:ANNOT_TOP]
                    cands = keep
                if time.time() - _annot_t >= ANNOT_EVERY:
                    _annot_t = time.time()
                    try:
                        vis = annot_img.copy()
                        for _cls, _si, tk in cands:
                            u, vb = tk.uv
                            w, h = tk.wh
                            # uv = [框中心 x, 框底边 y]；wh = [宽, 高]
                            x1, x2 = int(u - w / 2.0), int(u + w / 2.0)
                            y1, y2 = int(vb - h), int(vb)
                            col = COLORS.get(_cls, (0, 255, 255))
                            nm = ("%s%d" % (_cls, _si + 1)) if NSLOTS[_cls] > 1 else _cls
                            cv2.rectangle(vis, (x1, y1), (x2, y2), col, 2)
                            txt = "%s %.2f H=%.2fm D=%.1fm v=%.2fm/s" % (
                                nm, tk.conf, tk.h, tk.rng, tk.sp)
                            cv2.putText(vis, txt, (x1, max(14, y1 - 6)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 2, cv2.LINE_AA)
                        cv2.imwrite(os.path.join(ANNOT_DIR,
                                                 "a_%09.2f.jpg" % rospy.get_time()), vis)
                        if ANNOT_RAW:
                            cv2.imwrite(os.path.join(ANNOT_RAW_DIR,
                                                     "r_%09.2f.jpg" % rospy.get_time()),
                                        annot_img)
                    except Exception as e:
                        print("[pr] 存标注帧失败: %s" % e, flush=True)

        # 5s 心跳：证明主线程活着、相机有帧、检测在跑。
        # 零输出先看心跳，禁止猜死锁；n_det 增长=YOLO在跑，pub_list 非空=有可上报目标。
        _now = rospy.get_time()
        if _now - _t_live >= 5.0:
            _live_dt = _now - _t_live if _t_live > 0.0 else 0.0
            _loop_hz = ((n_loop - _last_live_loop) / _live_dt
                        if _live_dt > 0.0 else 0.0)
            _new_det = _stat.get("n_det", 0) - _last_live_det
            _t_live = _now
            _last_live_loop = n_loop
            _last_live_det = _stat.get("n_det", 0)
            _img_stamp = _latest.get("stamp", 0.0) if _latest else 0.0
            with _lock:
                _pose_stamp = _pose_history[-1][0] if _pose_history else None
                _pose_failure = _pose_fault
            _datum = globals().get('_height_datum')
            _pose_age = (_now - _pose_stamp) if _pose_stamp is not None else float('inf')
            if not 0.0 <= _pose_age <= .5 or _pose_failure is not None:
                print('[pr] PERCEPTION_HEALTH pose_age=%.3f image_age=%.3f '
                      'datum=%s datum_valid=%s pose_fault=%s; '
                      'image heartbeat is not localization readiness' %
                      (_pose_age, _now - _img_stamp if _img_stamp else float('inf'),
                       _datum.reason if _datum is not None else 'disabled',
                       _datum.offset is not None if _datum is not None else True,
                       _pose_failure), flush=True)
            with arb_lock:
                _live_tid = arb["tid"]
            with tstate_lock:
                _live_target = tstates.get(_live_tid)
            _live_yaw = _bearing_err = _track_range = _track_age = float('nan')
            if _pose_history:
                _live_pose = _pose_history[-1]
                _live_R = _live_pose[4]
                _live_yaw = math.atan2(_live_R[1, 0], _live_R[0, 0])
                if _live_target is not None:
                    _dx = _live_target[0] - _live_pose[1]
                    _dy = _live_target[1] - _live_pose[2]
                    _bearing_err = math.degrees(wrap_pi(
                        math.atan2(_dy, _dx) - _live_yaw))
                    _track_range = math.hypot(_dx, _dy)
                    _track_age = _now - _live_target[2]
            print("[pr] 心跳 loop=%d actual_hz=%.1f img_ts=%.3f img_age=%.2fs "
                  "n_det=%d det_delta=%d geo_rej=%d assoc=%d new=%d "
                  "z=%.2fm pub=%d coord=%s cam=(%.1f,%.1f,%.1f) "
                  "track=%s cam_yaw=%.1fdeg bearing_err=%.1fdeg "
                  "track_range=%.1fm track_age=%.1fs"
                  % (n_loop, _loop_hz, _img_stamp,
                     (_now - _img_stamp) if _img_stamp else -1.0,
                     _stat.get("n_det", 0), _new_det,
                     _stat.get("n_geo_rej", 0), _stat.get("n_assoc", 0),
                     _stat.get("n_new", 0), annot_z,
                     len(pub_list), "ON" if COORD_ON else "OFF",
                     _pose_history[-1][1] if _pose_history else 0.0,
                     _pose_history[-1][2] if _pose_history else 0.0,
                     _pose_history[-1][3] if _pose_history else 0.0,
                     _live_tid or '-', math.degrees(_live_yaw),
                     _bearing_err, _track_range, _track_age), flush=True)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
