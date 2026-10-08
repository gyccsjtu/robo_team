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

============================ 判据的物理依据 =============================
单目测距已经给出了沿光轴深度 z_cam，于是可以把框的像素高换算成「物体实际高度」：

    H = h_px * z_cam / fy

真人（actor 用 walk 动画，站立身高 ~1.7 m）必然落在 ~1.7 m 附近；
招牌 / 建筑色条要么几米要么几十米。diag 实测：

    真 actor_4 连续 9 帧：H = 1.70 ~ 2.11 m，conf 0.66 ~ 0.91
    同帧误检：          H = 0.92 / 2.63 / 4.62 / 17.8 / 24.4 / 31.2 m

**这个量纲是米，不是拍脑袋的阈值**，且只依赖 z_cam（我们自己测的）和 fy（官方内参）。

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

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetLinkState
from sensor_msgs.msg import Image, CameraInfo
from camera_geometry import calibration, aligned_translation, vertical_extent, image_time_position
from recent_motion import RecentMotion
from motion_identity import MotionIdentity
from stationary_person import allowed as stationary_person_allowed
from fresh_person import FreshPerson
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
from frame_probe import FIELDS as FRAME_FIELDS, frame_row, summarize_raw_boxes
from evidence_capture import EvidenceCapture

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

WEIGHTS = os.environ.get("PR_WEIGHTS") or _resolve_weights()
CONF = float(os.environ.get("PR_CONF", "0.40"))
UAV = os.environ.get("PR_UAV", "typhoon_h480_0")
# ---- 2026-09-19 双目改造 ----
# 相机不再走 cgo3（单目 640x360 云台相机），改成平台上双目相机的**左目**
# （752x480, fx=fy=376, HFOV=90°，本节内参来自 sdf 里的 <Fx>/<Cx> 实测值）。
# 这台相机是直接塞进 base_link 的 <sensor>（没有独立 link 也没有云台），
# 所以取位姿只能取 base_link，再自己补上相机在 base_link 系里的固定偏移。
CAM_LINK = os.environ.get("PR_CAM_LINK", "typhoon_h480_0::base_link")
CAM_OFF_BL = np.array([float(x) for x in
                       os.environ.get("PR_CAM_OFF_BL", "0.12,0.06,-0.05").split(",")])
CAM_TOPIC = os.environ.get("PR_CAM_TOPIC", "/%s/stereo_camera/left/image_raw" % UAV)
CAM_INFO_TOPIC = os.environ.get('PR_CAM_INFO_TOPIC', CAM_TOPIC.rsplit('/', 1)[0]+'/camera_info')
ROI_TOP = int(os.environ.get("PR_ROI_TOP", "60"))      # 忽略画面顶部：无人机自身部件常被误检成 blue
ROI_BOT = int(os.environ.get("PR_ROI_BOT", "478"))
MAX_RANGE = float(os.environ.get("PR_MAX_RANGE", "60"))
# 类别顺序必须是**训练时**的顺序（~/yolo_ds/data.yaml = red0 green1 blue2 white3 brown4）。
# 旧版这里是 ["green","blue","brown","white","red","red"]（单目 v6 模型的顺序 + red 双槽）。
# 顺序错位不会报错，只会让所有颜色系统性认反 —— 换权重时必须同步改这里。
CLASSES = ["red", "green", "blue", "white", "brown"]

# --- 人体几何门限（diag_geom.py 实测标定）---
# 严格档：用于**新建** track（误检从 2.6 m 起步，真人 1.7~2.1 m）
H_MIN = float(os.environ.get("PR_H_MIN", "1.15"))
H_MAX = float(os.environ.get("PR_H_MAX", "2.10"))
AR_MIN = float(os.environ.get("PR_AR_MIN", "0.55"))
AR_MAX = float(os.environ.get("PR_AR_MAX", "3.80"))
# 宽松档：用于**已确认** track 的关联（保链路优先，宁可放进噪声）
H_MIN_L = float(os.environ.get("PR_H_MIN_L", "0.80"))
H_MAX_L = float(os.environ.get("PR_H_MAX_L", "3.20"))
AR_MIN_L = float(os.environ.get("PR_AR_MIN_L", "0.40"))
AR_MAX_L = float(os.environ.get("PR_AR_MAX_L", "4.50"))
CONFIRM_HITS = int(os.environ.get("PR_CONFIRM_HITS", "5"))   # 达到后转宽松档

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

# ---- GPU 显存/CPU 内存瘦身开关（2026-10-03 六机 OOM 后续）----
# 背景：2026-10-03 六机 GPU 轮全局 OOM，gzserver pid 1154 被内核杀（Free swap=0）。
# **原相机与雷达参数一律不动**，只动推理侧。
#
# 【实测结论，勿再重复推断】以下是本机 vision_cuda_20261003 环境下的真实测量，
# 与最初假设有出入，改动按此重写：
#  1. fp16（PR_FP16=1）：**不省内存，反而 +116 MB RSS**（1080->1196 MB），
#     显存两者同为 1174 MB 无差别。YOLO11n 权重太小，fp16 省下的那点权重
#     抵不过半精度算子workspace 的额外分配。**故默认关闭且不推荐开。**
#     保留开关仅为对照实验，正式轮不要开。
#  2. 线程：8 个 pt_main_thread 各烧 ~47s是真在并行计算，wchan=futex_do_wait
#     属于自旋后休眠的正常形态。实测 OMP_WAIT_POLICY=passive + GOMP_SPINCOUNT=0
#     **无效**（24.87s vs 25.00s 噪声内），吞吐反降 24%，故未采用。
#     PR_TORCH_THREADS 保留为可调项，默认 0=不干预。
#  3. 真正省内存的路子是**共享推理服务**（6 份 CUDA context -> 1 份），
#     尚未实现，见 coordination/docs/gpu_memory_optimization_20261003.md。
PR_FP16 = os.environ.get("PR_FP16", "0") == "1"
PR_TORCH_THREADS = int(os.environ.get("PR_TORCH_THREADS", "0") or 0)
PR_MEMORY_FRACTION = float(os.environ.get("PR_MEMORY_FRACTION", "0") or 0.0)

# ---- 共享推理服务（2026-10-03）----
# 六路各自建CUDA context 是 OOM 里唯一可压缩的大头。PR_SHARED_INFER=1 时
# 本进程不再加载权重，改连perception/shared_inference_service.py（单份 context）。
#实测可行性：单流 p50=24.4ms，六路串行 21.4ms/次 ⇒ 每 1000ms 只占 128ms，约 7.8x余量。
# **默认 0**：共享服务本身尚未在六机轮验证过，正式轮先不开。
# 失败语义：服务不可用时**自动回退到进程内推理**，绝不因为服务故障让 15s 判据断流。
PR_SHARED_INFER = os.environ.get("PR_SHARED_INFER", "0") == "1"
PR_SHARED_PORT = int(os.environ.get("PR_SHARED_PORT", "19731"))


class _BoxesShim:
    """Duck-type stand-in for an ultralytics ``Results``.

    perception_real.py consumes exactly ``res.boxes`` (iterable) and, per box,
    ``.cls`` / ``.conf`` / ``.xyxy[0]`` -- verified at the single inference call
    site. Returning this narrow shape is what lets the shared service exist at
    all: only three numbers per detection cross the process boundary instead of
    a pickled Results object. If a future edit starts reading another Results
    attribute, this shim fails loudly (AttributeError) rather than silently
    reporting wrong geometry.
    """

    __slots__ = ("boxes", "device")

    def __init__(self, boxes):
        self.boxes = boxes
        self.device = "shared"





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
RECENT_MOTION_WINDOW = float(os.environ.get('PR_RECENT_MOTION_WINDOW', '0'))
BLUE_MOTION_WINDOW = float(os.environ.get('PR_BLUE_MOTION_WINDOW', str(RECENT_MOTION_WINDOW)))
BLUE_MOTION_MIN_SPAN = float(os.environ.get('PR_BLUE_MOTION_MIN_SPAN', '1'))
BLUE_IDENTITY_GUARD = os.environ.get('PR_BLUE_IDENTITY_GUARD', '0') == '1'
STATIONARY_GREEN_ON = os.environ.get('PR_STATIONARY_GREEN','0') == '1'
FAST_GREEN_WHITE = os.environ.get('PR_FAST_GREEN_WHITE','0') == '1'
FAST_BLUE_PERSON = os.environ.get('PR_FAST_BLUE_PERSON','0') == '1'
FAST_RED_PERSON = os.environ.get('PR_FAST_RED_PERSON','0') == '1'
WHITE_SHORT_MISS_RECOVERY = os.environ.get('PR_WHITE_SHORT_MISS_RECOVERY','1') == '1'
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

# --- 协同上报（接口 v0 / schema v1：robocup_repo/docs/coordination_interface_v0.md）---
# ⚠⚠ 契约形状：/coordination/target_report 上必须是**裸 data 对象**，不是带 envelope
#    的完整消息。队友 coordination_node.py 的 _in() 把 json.loads 的结果**整个**当
#    `data` 交给核心，所以 topic 上只能有这 5 个字段：
#      {"target_id","frame_id":"world_enu","xyz":[x,y,z],"confidence","observation_id"}
#    且**每个目标一条**（target_id 是唯一键），不是"一帧一条、dets 里塞全部目标"。
#    2026-09-20 实测：原发布端发的是 {"stamp","cam","uav","dets":[...]}，
#    target_id / frame_id / observation_id / confidence 四个必填字段**一个都没有**，
#    xyz 还只有 2 维 ⇒ 接上协同核心会**一条都解析不出来**，不是"没联调"而是"接不上"。
# ⚠ target_id 用 tag（green/blue/white/brown/red1/red2）：这是官方目标身份、跨帧稳定；
#    track 的整数 id 会因重建而变，按契约（"非空稳定 ID"）不能用。
# ⚠ 我们算出的 (x,y) 是**地面交点**（射线与 z=0 平面求交），不是目标体心，
#    所以 z 由 TARGET_Z 显式补上（默认取官方 actor 体心高度 1.25m）。
COORD_ON = int(os.environ.get("PR_COORD_ON", "1"))
COORD_HZ = float(os.environ.get("PR_COORD_HZ", "2.0"))   # 对齐队友替身；核心按 tick 消费，灌太快会积压
TARGET_Z = float(os.environ.get("PR_TARGET_Z", "1.25"))
_CSV = logger("perception_%s" % UAV, [
    "ros_time", "event", "target_id", "class_name", "confidence",
    "bbox_u", "bbox_v", "bbox_w", "bbox_h", "target_x", "target_y",
    "target_z", "range_m", "height_m", "strict", "track_id", "hits",
    "image_stamp", "image_age_s", "inference_s", "source",
    "person_frame_verified", "person_hits", "track_miss", "fresh_person_allowed",
    "person_established", "person_current_verified",
    # Gate inputs, added 2026-10-07 so a rejection can be re-evaluated offline
    # under a different setting (e.g. PR_FAST_GREEN_WHITE) instead of guessed at.
    # csv_logger ignores keys that are not declared here, and rotates the file
    # when this list changes, so the extra columns are inert until the next run.
    "recent_speed", "sp", "max_disp", "score_ema", "attached", "confirm_hits",
    "verdict_score", "verdict_sp", "verdict_disp", "stationary_person",
    "person_gate_allowed", "green_frame_proof", "motion_samples", "motion_span_s",
    "original_s", "raw_target_x", "raw_target_y", "blue_identity_qualified", "blue_identity_allowed",
    # N12 (2026-10-08): same-frame person support detail on yolo_detection rows.
    # csv_logger ignores undeclared keys and rotates on header change, so old
    # archives keep their own headers and stay readable.
    "person_iou", "person_conf", "n_person_boxes", "reject_reason"])
# 逐帧分母（N7，2026-10-07）：上面那份 CSV 只在"有检出/有拒发/有上报"时才写行，
# 于是"15s 判据被检出间隙打断"没法归因到"模型没出框"还是"被我们自己的门限丢了"。
# 这一份每处理一帧写一条，只记录、不参与控制。文件名刻意不叫 perception*.csv ——
# audit_fast_city.py / replay_image_motion.py 用 perception*.csv 通配读事件流。
_FRAMES = logger("frame_probe_%s" % UAV, FRAME_FIELDS)


# N12 fix (codex review #2): two bounded capture channels — failures AND a
# sampled stream of proof-passing candidates. The 13 "verified but wrong"
# green candidates live in the second channel; the first alone can never
# capture them.
_EVIDENCE_FAIL = EvidenceCapture(UAV)
_EVIDENCE_PASS = EvidenceCapture(UAV + "_pass", per_minute=2, total=60)
_EVIDENCE_WHITE_FAIL = EvidenceCapture(UAV + "_white_fail", per_minute=2, total=60,
    enabled=os.environ.get('PR_WHITE_FAILURE_EVIDENCE', '0') == '1')


def _person_detail(details, cid, xyxy):
    """N12: find the verifier evidence entry for this exact colour box.

    Matches by same-box overlap (>=0.9), tolerant of float rounding between
    the verifier's stored xyxy and the consumer loop's unpacked values.
    Accepts both evidence rows (best_person_*) and verified rows (person_*).
    Returns a normalised dict or None. Pure lookup; no control impact.
    """
    from person_verifier import overlap as _pv_overlap
    best = None
    best_iou = 0.0
    for d in details or []:
        try:
            if int(d.get('cls', -1)) != int(cid):
                continue
            iou = _pv_overlap(d.get('xyxy') or [], xyxy)
        except Exception:
            continue
        if iou > best_iou:
            best_iou = iou
            best = d
    if best is None or best_iou < 0.9:
        return None
    return dict(person_iou=best.get('person_iou', best.get('best_person_iou')),
                person_conf=best.get('person_conf', best.get('best_person_conf')),
                n_person_boxes=best.get('n_person_boxes'),
                reject_reason=best.get('reject_reason'),
                matched=best.get('matched'),
                shirt_fractions=best.get('shirt_fractions'),
                person_xyxy=best.get('person_xyxy', best.get('best_person_xyxy')))
# 调试快照**另开一条话题**：它原来和契约挤在同一条 topic 上，形状完全不同。
# 调试通道与契约通道必须分开，否则要么队友解析不了、要么我自己的复盘脚本全废。
DEBUG_TOPIC = os.environ.get("PR_DEBUG_TOPIC", "/perception/debug_snapshot")

_lock = threading.Lock()
_latest = {"img": None, "stamp": 0.0, "recv_wall": 0.0}
_pose_history = []  # (ROS time, x, y, z, rotation matrix)
_pose_history_lock = threading.Lock()
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


def on_img(msg):
    try:
        img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except Exception:
        return
    with _lock:
        _latest["img"] = img
        _latest["stamp"] = msg.header.stamp.to_sec()
        _latest["recv_wall"] = time.time()
        _latest["receive_age_s"] = rospy.Time.now().to_sec() - _latest["stamp"]
        _latest["header"] = dict(frame_id=str(msg.header.frame_id),
                                 seq=int(msg.header.seq))


def on_camera_info(msg):
    global FX, FY, CX, CY, IMG_W, IMG_H, _camera_ready
    try:
        values = calibration(msg.width, msg.height, msg.K, msg.D)
    except (ValueError, TypeError):
        return
    with _lock:
        FX, FY, CX, CY, IMG_W, IMG_H = values
        _camera_ready = True


_camera_ready = False


def pose_at_stamp(stamp, fallback, evidence=None):
    """Estimate camera translation at an image ROS timestamp.

    Gazebo's GetLinkState service returns the current pose only. A short history
    lets a delayed image use the camera position from its acquisition time.
    Interpolate camera orientation too: the radar model has a moving gimbal.
    """
    if stamp <= 0.0:
        return None
    with _pose_history_lock:
        history = list(_pose_history)
    if len(history) < 2:
        return None
    first, second = history[-2], history[-1]
    for earlier, later in zip(history, history[1:]):
        if earlier[0] <= stamp <= later[0]:
            first, second = earlier, later
            break
    try:
        xyz = aligned_translation(stamp, first[:4], second[:4])
    except ValueError:
        return None
    alpha = min(1., max(0., (stamp-first[0])/(second[0]-first[0])))
    # Project the small-interval matrix blend back to a proper rotation.
    left, _, right = np.linalg.svd((1.-alpha)*first[4]+alpha*second[4])
    correction = np.diag([1., 1., np.linalg.det(left@right)])
    rotation = left@correction@right
    if evidence is not None:
        # Diagnostic only: copy the exact samples chosen for this image, while
        # keeping the existing alignment and control result unchanged.
        def sample_record(sample):
            row = dict(sample_s=float(sample[0]),
                       camera_xyz=[float(v) for v in sample[1:4]],
                       camera_rotation=sample[4].reshape(-1).tolist())
            if len(sample) > 5:
                row['service'] = dict(sample[5])
            return row
        evidence.update(image_s=float(stamp), first=sample_record(first),
                        second=sample_record(second), rotation_alpha=float(alpha),
                        camera_xyz=[float(v) for v in xyz],
                        camera_rotation=rotation.reshape(-1).tolist())
    return xyz+(rotation,)


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

    def __init__(self, cls, x, y, conf, uv, wh, rng, h, t, uav=(0.0, 0.0, 0.0), appearance_feat=None):
        """appearance_feat: 外观特征向量（颜色直方图/深度特征），用于外观关联"""
        Track._seq += 1
        self.id = Track._seq
        self.cls = cls
        self.x, self.y = x, y
        self.vx, self.vy = 0.0, 0.0
        self.t = t
        self.observed_s = t
        self.raw_xy = (x,y)
        self.green_frame_proof = False
        self.person_support = FreshPerson(resume_after_miss=cls == 'white' and FAST_GREEN_WHITE
                                         and WHITE_SHORT_MISS_RECOVERY)
        motion_window = BLUE_MOTION_WINDOW if cls == 'blue' else RECENT_MOTION_WINDOW
        motion_span = BLUE_MOTION_MIN_SPAN if cls == 'blue' else 1.
        self.recent_motion = RecentMotion(motion_window, motion_span) if motion_window > 0 else None
        if self.recent_motion is not None:
            self.recent_motion.observe(t,x,y)
        self.blue_identity = MotionIdentity() if cls == 'blue' and BLUE_IDENTITY_GUARD else None
        if self.blue_identity is not None:
            self.blue_identity.observe(t,x,y)
        self.hits = 1
        self.miss = 0
        self.conf = conf
        self.uv, self.wh, self.rng = uv, wh, rng
        self.h = h                      # 归一化身高 (m)
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
        self.pub_ema = None             # 对外发布坐标的 EMA（降低抖动，保 15s 连续<1m）
        # v4 新增：外观特征（颜色直方图/深度特征）用于多层次关联
        self.appearance = appearance_feat if appearance_feat is not None else np.zeros(32, dtype=np.float32)
        self.appear_updated = 0         # 外观特征更新帧数

    def predict(self, dt):
        return self.x + self.vx * dt, self.y + self.vy * dt

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
            return self.pub_ema
        return px, py

    def update(self, zx, zy, dt, conf, uv, wh, rng, h, t, uav=None, appearance_feat=None):
        """更新track状态，可选更新外观特征用于多层次关联"""
        if not math.isfinite(t) or t <= self.observed_s or dt <= 0:
            return False
        self.observed_s = t
        self.raw_xy = (zx,zy)
        if self.recent_motion is not None:
            self.recent_motion.observe(t,zx,zy)
        if self.blue_identity is not None:
            self.blue_identity.observe(t,zx,zy)
        px, py = self.predict(dt)
        rx, ry = zx - px, zy - py
        self.x = px + ALPHA * rx
        self.y = py + ALPHA * ry
        if dt > 1e-3:
            self.vx += BETA * rx / dt
            self.vy += BETA * ry / dt
        sp = math.hypot(self.vx, self.vy)
        if sp > V_MAX:
            self.vx *= V_MAX / sp
            self.vy *= V_MAX / sp
        self.conf, self.uv, self.wh, self.rng = conf, uv, wh, rng
        self.h = h
        self.score = conf * person_likelihood(h) * self.motion_factor(t)
        self.score_ema = 0.7 * self.score_ema + 0.3 * self.score
        self.t, self.hits, self.miss = t, self.hits + 1, 0
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

    def verdict(self, now=None, max_coast=None, attach_check=True, stationary_person=False, verified_person=False):
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
        if self.hits < (min(3,CONFIRM_HITS) if verified_person else CONFIRM_HITS):
            return False, "hits"
        if self.miss > max_coast:
            return False, "coast"
        # 视差判据放在几何判据之前：它是"这东西长在机身上"的铁证，
        # 而机身的归一化身高恰好会落在人头区间（实测 1.63~2.32 m），几何拦不住。
        if attach_check and not verified_person and VERDICT_ATTACH_ON and self.attached_to_cam():
            return False, "self"
        if not (VERDICT_H_MIN <= self.h <= VERDICT_H_MAX):
            return False, "height"
        if self.rng > VERDICT_RANGE_MAX:
            return False, "range"
        self.motion_factor(now)                 # 刷新 self.sp
        # 当前在动，或者生命期内动过 —— 见 VERDICT_DISP 的注释（治 actor 卡死）
        recent_speed = self.recent_motion.speed(self.observed_s if now is None else now) if self.recent_motion is not None else None
        blue_qualified = self.blue_identity is not None and self.blue_identity.allowed(self.observed_s if now is None else now)
        # A real, blue-clothed decorative person passes both models. Person
        # proof establishes appearance, not membership of the moving targets.
        # Keep the existing original-image movement qualification mandatory.
        if self.blue_identity is not None and not blue_qualified:
            return False, "blue_identity"
        if not blue_qualified and not stationary_person and not verified_person and ((recent_speed is not None and recent_speed < VERDICT_SP) or (recent_speed is None and self.sp < VERDICT_SP and self.max_disp < VERDICT_DISP)):
            return False, "static"
        if self.score_ema < VERDICT_SCORE:
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


def main():
    global _model
    rospy.init_node("perception_real", anonymous=True)

    # ---- 官方话题发布仲裁状态：本机当前被指派追踪的目标 tid ----
    # 收到 task_type=1/2 且 target_id 非空 -> 持有发布权；
    # 收到搜索任务/其他指派 -> 立即撤销（与 agent 状态机一致）。
    arb = {"tid": None}
    arb_lock = threading.Lock()
    camera_task = None
    if os.environ.get('ROBOCUP_RUN_ID'):
        from camera_task import CameraTask
        camera_task = CameraTask(os.environ['ROBOCUP_RUN_ID'],
            os.environ.get('PR_LOGICAL_UAV_ID',UAV))
    # 目标融合状态缓存：tid -> (x, y, stamp_secs)
    tstates = {}
    tstate_lock = threading.Lock()

    def _arb_cb(msg):
        if str(msg.uav_id) != os.environ.get('PR_LOGICAL_UAV_ID',str(UAV)):
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

    def _authorized_camera_cb(msg):
        try:
            value = json.loads(msg.data)
            with arb_lock:
                camera_task.receive(value,rospy.Time.now().to_sec())
        except (ValueError,TypeError):
            return

    if camera_task is not None:
        rospy.Subscriber('/swarm/authorized_assignment',String,_authorized_camera_cb)
        print('[pr] 相机视差判决使用本运行逐机授权；旧assignment只供诊断',flush=True)
    elif SearchAssignment is not None:
        rospy.Subscriber("/swarm/assignment", SearchAssignment, _arb_cb)
        print("[pr] 官方话题仲裁开启：仅被指派追踪且目标<=%.0fm 时直发 /actor_*_info"
              % ACTOR_PUB_RANGE, flush=True)
    else:
        print("[pr] WARN: 未导入 SearchAssignment（%s），官方话题仅过 %.0fm 距离闸门"
              % (_search_assign_import_err, ACTOR_PUB_RANGE), flush=True)
    if TargetState is not None:
        rospy.Subscriber("/swarm/target_states", TargetState, _tstate_cb)
        print("[pr] 空间身份闸门开启：发布坐标须与融合估计一致(<=%.1fm, 新鲜<=%.0fs)"
              % (IDENTITY_M, TSTATE_FRESH_S), flush=True)

    print("[pr] 加载权重 %s" % WEIGHTS, flush=True)
    print("[pr] v4.0 双目左目适配 | 相机 %s <- %s | 内参 fx=%.1f cx=%.1f cy=%.1f 图 %dx%d"
          % (CAM_LINK, CAM_TOPIC, FX, CX, CY, IMG_W, IMG_H), flush=True)
    print("[pr] 类别顺序 %s" % CLASSES, flush=True)
    infer_device = os.environ.get('PR_DEVICE', '')
    device_reported = False
    _shared = None
    if PR_SHARED_INFER:
        # 共享模式：本进程不加载权重，CUDA context 由服务进程独占一份。
        from shared_inference_client import SharedInferenceClient
        _shared = SharedInferenceClient(port=PR_SHARED_PORT)
        _model = None
        # 等服务就绪再开跑，避免开局全部走回退路径把日志刷满。
        _deadline = time.time() + 30.0
        _ready = False
        while time.time() < _deadline:
            if _shared.available():
                _ready = True
                break
            time.sleep(0.5)
        print('[pr] shared_inference port=%d ready=%s (0=将回退进程内推理)'
              % (PR_SHARED_PORT, _ready), flush=True)
    else:
        from ultralytics import YOLO
        _model = YOLO(WEIGHTS)
    # torch.inference_mode() 不在此处添加：ultralytics 的 predictor.stream_inference
    # 已带 @smart_inference_mode()（predictor.py:214），当前 PyTorch(2.4.1) 下内部
    # 即走 torch.inference_mode()。重复包一层没有收益。
    #
    # PR_FP16 经实测不省内存（RSS 反而 +116 MB，显存持平），故默认关闭。
    # 走ultralytics 原生 half=True 开关（AutoBackend 内 model.half()），
    # 不手工改模型，避免半精度与设备迁移顺序出错。
    if PR_TORCH_THREADS > 0:
        import torch
        torch.set_num_threads(PR_TORCH_THREADS)
        try:
            torch.set_num_interop_threads(PR_TORCH_THREADS)
        except RuntimeError:
            # 并行池若已建立则无法再改；不影响正确性，仅记录实际值。
            pass
        print('[pr] torch_threads intra=%d inter=%d' %
              (torch.get_num_threads(), torch.get_num_interop_threads()), flush=True)
    if PR_MEMORY_FRACTION > 0.0 and infer_device not in ('', 'cpu'):
        import torch
        if torch.cuda.is_available():
            torch.cuda.set_per_process_memory_fraction(PR_MEMORY_FRACTION)
            print('[pr] cuda_memory_fraction=%.3f' % PR_MEMORY_FRACTION, flush=True)


    # pubs[类][槽号]：单流类只有 1 个槽，red 有 2 个（red1/red2）
    pubs = {}
    for c, topics in TOPIC_OF.items():
        pubs[c] = [rospy.Publisher(t, ActorInfo, queue_size=1) for t in topics]
    # 契约通道：一条消息 = 一个目标（形状见文件上方"协同上报"段）
    coord = rospy.Publisher("/coordination/target_report", String, queue_size=50)
    visual_coord = rospy.Publisher('/swarm/visual_observation', String, queue_size=50)
    processed_camera = (rospy.Publisher('/swarm/processed_camera_frame', String, queue_size=30)
                        if camera_task is not None and os.environ.get('SWARM_SEARCH_OBSERVATION', '0') == '1'
                        else None)
    if processed_camera is not None:
        from search_observation import valid_frame
    processed_frame_seq = 0
    # 调试通道：一帧一条、含全部 dets 的快照，供 rec_snap.py / analyze_snap.py 离线复盘
    dbg = rospy.Publisher(DEBUG_TOPIC, String, queue_size=5)
    # YOLO 实时视角（带检测框，调试可视化）
    yolo_view_pub = (rospy.Publisher(YOLO_VIEW_TOPIC, Image, queue_size=1)
                     if YOLO_VIEW else None)

    # A 640x360 RGB frame is larger than rospy's 64 KiB default receive
    # buffer. Read complete frames so queue_size=1 can discard old images
    # instead of leaving seconds of serialized frames in the TCP stream.
    rospy.Subscriber(CAM_TOPIC, Image, on_img, queue_size=1,
                     buff_size=8 * 1024 * 1024, tcp_nodelay=True)
    rospy.Subscriber(CAM_INFO_TOPIC, CameraInfo, on_camera_info, queue_size=1)
    rospy.wait_for_service("/gazebo/get_link_state", timeout=90)
    gls = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    pose_sampler = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

    def _sample_camera_pose(event):
        before = rospy.Time.now().to_sec()
        try:
            response = pose_sampler(CAM_LINK, 'world')
            after = rospy.Time.now().to_sec()
            if not response.success or not 0 <= after-before <= .1:
                return
            pose = response.link_state.pose
            q = pose.orientation
            rotation = quat_to_R(q.x, q.y, q.z, q.w)
            offset = rotation @ CAM_OFF_BL
            sample = ((before+after)*.5, pose.position.x+offset[0],
                      pose.position.y+offset[1], pose.position.z+offset[2], rotation,
                      dict(before_s=float(before), after_s=float(after),
                           link_name=str(response.link_state.link_name),
                           reference_frame=str(response.link_state.reference_frame),
                           link_xyz=[float(pose.position.x), float(pose.position.y),
                                     float(pose.position.z)],
                           link_quaternion_xyzw=[float(q.x), float(q.y), float(q.z), float(q.w)]))
            with _pose_history_lock:
                if _pose_history and sample[0] <= _pose_history[-1][0]:
                    return
                _pose_history.append(sample)
                del _pose_history[:-200]
        except Exception as error:
            rospy.logwarn_throttle(5, 'Camera pose sampler: %s', error)

    # Pose acquisition must not wait for slow CPU inference or delayed images.
    _camera_pose_timer = rospy.Timer(rospy.Duration(.05), _sample_camera_pose)

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
    print("[pr] v4.0(双目) 几何+运动+人判决+red双流 | conf=%.2f  detect=%.0fHz  pub=%.0fHz"
          % (CONF, PUB_HZ / DETECT_EVERY, PUB_HZ), flush=True)
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
    from person_verifier import PersonVerifier, frame_verified
    person_verifier=PersonVerifier(os.environ.get('PR_PERSON_VERIFY_WEIGHTS',''),infer_device,
        os.environ.get('PR_PERSON_VERIFY_CONF','.1'),os.environ.get('PR_PERSON_VERIFY_IOU','.25'),
        classes=(1,),proof_classes=(1,)+((3,) if FAST_GREEN_WHITE else ())
            +((0,) if FAST_RED_PERSON else ())+((2,) if FAST_BLUE_PERSON else ()),
        color_check=os.environ.get('PR_COLOR_VERIFY','0')=='1')
    n_loop = 0
    _t_live = 0.0            # 心跳上次打印墙钟（5s 一次，证明主线程活着）
    _coord_t = 0.0            # 上次发协同上报的墙钟（COORD_HZ 节流用）
    _published_visual_samples = {}  # tag -> last actual image time, not publish time
    _obs_seq = 0              # observation_id 单调序号（契约要求非空且不复用）
    annot_img = None          # 最近一次参与检测的图像，供标注帧使用
    annot_z = 0.0             # 该帧对应的相机高度
    frame_stamp = 0.0         # 当前检测图像的 ROS 采集时间
    frame_age = None
    inference_s = None
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

    _processed_image_stamp = 0.
    while not rospy.is_shutdown():
        n_loop += 1
        now = rospy.Time.now().to_sec()
        dets = []
        new_frame = False
        search_context = None

        # ---------- 1) 检测 + 几何过滤 ----------
        if n_loop % DETECT_EVERY == 0:
            with _lock:
                img = None if _latest["img"] is None else _latest["img"].copy()
                frame_stamp = _latest["stamp"]
                image_header = dict(_latest.get('header', {}))
                image_recv_wall = _latest.get('recv_wall')
                image_receive_age_s = _latest.get('receive_age_s')
            frame_age = max(0.0, now - frame_stamp) if frame_stamp > 0.0 else None
            if (img is not None and _camera_ready and img.shape[:2] == (IMG_H, IMG_W)
                    and frame_stamp > _processed_image_stamp and 0 <= now-frame_stamp <= 1.):
                try:
                    response = gls(CAM_LINK, "world")
                    if not response.success:
                        raise ValueError('Camera link pose unavailable')
                    ls = response.link_state
                    px, py, pz = ls.pose.position.x, ls.pose.position.y, ls.pose.position.z
                    o = ls.pose.orientation
                    R = quat_to_R(o.x, o.y, o.z, o.w)
                    # 双目左目是塞在 base_link 里的 <sensor>（没有独立 link、没有云台），
                    # get_link_state 只能返回 base_link 原点 ⇒ 自己补上相机在机体系里的固定偏移。
                    # 相机 pose rpy=0 ⇒ 轴向与 base_link 完全一致，直接用同一个 R 旋转偏移量。
                    _off = R @ CAM_OFF_BL
                    px, py, pz = px + _off[0], py + _off[1], pz + _off[2]
                    # 相机水平偏航（视差判据要用）：取旋转矩阵第一列的水平分量
                    yaw_cam = math.atan2(R[1, 0], R[0, 0])
                    frame_binding = dict(schema_version=1, image_header=image_header,
                        image_topic=CAM_TOPIC, configured_link=CAM_LINK,
                        camera_info_topic=CAM_INFO_TOPIC,
                        camera_offset_link=CAM_OFF_BL.tolist(),
                        image_recv_wall=image_recv_wall, image_receive_age_s=image_receive_age_s,
                        intrinsics=[float(FX), float(FY), float(CX), float(CY)],
                        size=[int(IMG_W), int(IMG_H)])
                    px, py, pz, R = pose_at_stamp(
                        frame_stamp, (px, py, pz, R), evidence=frame_binding)
                    yaw_cam = math.atan2(R[1, 0], R[0, 0])
                    uav = (px, py, yaw_cam)
                    uav_last = (px, py)          # 供清理段判"离飞机过远"
                    cam_full = [round(px, 3), round(py, 3), round(pz, 3),
                                round(o.x, 5), round(o.y, 5), round(o.z, 5), round(o.w, 5)]
                    # 位姿取到了才更新标注帧：与检测同帧，保证所见即所判
                    annot_img, annot_z = img, pz
                except Exception as e:
                    print("[pr] 取相机位姿失败: %s" % e, flush=True)
                    R, (px, py, pz) = None, (0, 0, 0)
                    uav = (0.0, 0.0, 0.0)
                    cam_full = None

                if R is not None:
                    if processed_camera is not None:
                        with arb_lock:
                            search_context = camera_task.search_context(rospy.Time.now().to_sec(), frame_stamp)
                    _processed_image_stamp = frame_stamp
                    new_frame = True
                    _infer_t0 = time.time()
                    # 推理后端二选一：共享服务（省CUDA context）或本进程。
                    # 共享失败必须回退，不能让 15s 判据因服务故障断流。
                    _shared_meta = None
                    _local_inference = _shared is None
                    if _shared is not None:
                        _boxes, _shared_meta = _shared.infer(img, uav=UAV)
                        if _boxes is None:
                            _local_inference = True
                            # 首次回退时才懒加载本进程模型，之后继续用本地。
                            _shared.note_fallback(_shared.last_error or 'no_reply')
                            if _model is None:
                                from ultralytics import YOLO
                                _model = YOLO(WEIGHTS)
                                print('[pr] 回退进程内推理（共享服务不可用：%s）'
                                      % (_shared.last_error or 'no_reply'), flush=True)
                            res = _model(img, conf=CONF, verbose=False, device=infer_device,
                                         **({'half': True} if PR_FP16 else {}))[0]
                            inference_s = time.time() - _infer_t0
                        else:
                            res = _BoxesShim(_boxes)
                            inference_s = time.time() - _infer_t0
                    else:
                        res = _model(img, conf=CONF, verbose=False, device=infer_device,
                                     **({'half': True} if PR_FP16 else {}))[0]
                        inference_s = time.time() - _infer_t0
                    if _local_inference:
                        res = _BoxesShim(person_verifier.filter_boxes(img,res.boxes))
                        inference_s = time.time() - _infer_t0
                    _green_frame_verified = (person_verifier.green_proof_enabled() if _local_inference
                        else bool(_shared_meta and _shared_meta.get('verified_green_person') is True))
                    _person_colors = (person_verifier.verified_colors() if _local_inference
                        else (_shared_meta or {}).get('verified_person_colors',[]))
                    _person_boxes = (person_verifier.verified_boxes if _local_inference
                        else (_shared_meta or {}).get('verified_person_boxes',[]))
                    # N12: full same-frame support detail for the CSV/evidence log.
                    # Local path: verifier.evidence covers every relevant box.
                    # Shared path: verified boxes (v3: with support keys) plus the
                    # separate rejected list reconstruct the same coverage.
                    _frame_person_details = (list(person_verifier.evidence)
                        if _local_inference else
                        list((_shared_meta or {}).get('verified_person_boxes', []))
                        + list((_shared_meta or {}).get('verified_person_rejected', [])))
                    if not device_reported:
                        print('[pr] inference_device=%s first_inference_s=%.4f' %
                              ((res.device if hasattr(res, 'device') else
                                (_model.predictor.device if _model is not None else 'shared')),
                               inference_s), flush=True)
                        device_reported = True
                    # v4新增：时间戳同步优化 - 推理耗时补偿
                    # 假设推理均匀分布在帧的两端，使用 frame_stamp - inference_s/2 作为更准确的时间戳
                    detection_time = frame_stamp - inference_s * 0.5
                    # v4新增：对同类别的检测结果做IOU去重（解决YOLO同一目标多次检测问题）
                    boxes_by_class = {}
                    for b in res.boxes:
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
                    # 遍历过滤后的boxes
                    _geo_rej_before = _stat["n_geo_rej"]
                    for b in filtered_boxes:
                        cid = int(b.cls)
                        conf = float(b.conf)
                        x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
                        if y2 < ROI_TOP or y1 > ROI_BOT:
                            continue
                        _stat["n_det"] += 1

                        u = (x1 + x2) / 2.0
                        # 用框真实底部 y2，不截断到 ROI_BOT：
                        # 目标在远处时视线接近水平，v_world[2]≈0，t=-pz/v_world[2] 很大，
                        # 框底部 2 像素的截断会被放大成 >1m 的坐标偏差（实测导致官方不认）。
                        # ROI_BOT 仍用于 L794 的过滤（框完全在底部带则跳过）。
                        v = float(y2)
                        v_opt = np.array([(u - CX) / FX, (v - CY) / FY, 1.0])
                        v_link = M_OPT2LINK @ v_opt
                        v_world = R @ v_link
                        if v_world[2] >= -1e-6:
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
                        # Intersect the top ray with the vertical line over
                        # the bottom pixel's ground point. Radial range is
                        # for distance gates, not pixel-to-height projection.
                        top_ray = R @ M_OPT2LINK @ np.array([(u-CX)/FX, (y1-CY)/FY, 1.])
                        try:
                            impl_h = vertical_extent((px,py,pz),
                                (px+t*v_world[0], py+t*v_world[1]), top_ray)
                        except ValueError:
                            continue
                        ar = h_px / max(1e-3, w_px)
                        strict = (H_MIN <= impl_h <= H_MAX) and (AR_MIN <= ar <= AR_MAX)
                        loose = (H_MIN_L <= impl_h <= H_MAX_L) and (AR_MIN_L <= ar <= AR_MAX_L)
                        if not loose:                       # 连宽松档都过不了 -> 纯噪声
                            _stat["n_geo_rej"] += 1
                            continue

                        wx = px + t * v_world[0]
                        wy = py + t * v_world[1]
                        # v4 新增：提取外观特征（HSV颜色直方图）用于多层次关联
                        img_crop = img[int(y1):int(y2), int(x1):int(x2)] if x2 > x1 and y2 > y1 else None
                        appearance_feat = extract_appearance_feat(img_crop) if img_crop is not None else None
                        from jersey_color import torso_fractions, supported
                        _shirt = torso_fractions(img,[x1,y1,x2,y2]) if cid == 1 and _green_frame_verified else None
                        _green_proof = bool(_shirt is not None and supported('green',_shirt))
                        _color = CLASSES[cid] if cid < len(CLASSES) else '?'
                        _person_shirt = torso_fractions(img,[x1,y1,x2,y2]) if _color in _person_colors else None
                        _person_proof = bool(_person_shirt is not None and supported(_color,_person_shirt)
                            and frame_verified(_person_boxes,cid,[x1,y1,x2,y2]))
                        # N12: same-frame support detail for this exact box.
                        try:
                            _pd = _person_detail(_frame_person_details, cid,
                                                 [x1, y1, x2, y2])
                        except Exception:
                            _pd = None
                        dets.append({"cls": CLASSES[cid] if cid < len(CLASSES) else "?",
                                     "green_frame_proof": _green_proof,
                                     "person_frame_proof": _person_proof,
                                     "conf": round(conf, 3),
                                     "uv": [round(u, 1), round(v, 1)],
                                     "wh": [round(w_px, 1), round(h_px, 1)],
                                     "xyz": [round(wx, 2), round(wy, 2)],
                                     "range": round(rng, 2),
                                     "h": round(impl_h, 2),
                                     "strict": strict,
                                     "score": round(conf * person_likelihood(impl_h), 3),
                                     "appearance": appearance_feat})
                        _CSV.write(
                            ros_time=rospy.Time.now().to_sec(), event="yolo_detection",
                            class_name=CLASSES[cid] if cid < len(CLASSES) else "?",
                            confidence=conf, bbox_u=u, bbox_v=v, bbox_w=w_px,
                            bbox_h=h_px, target_x=wx, target_y=wy,
                            target_z=TARGET_Z, range_m=rng, height_m=impl_h,
                            strict=strict, image_stamp=frame_stamp,
                            image_age_s=frame_age, inference_s=inference_s,
                            source="yolo_raw",person_frame_verified=_person_proof,
                            person_iou=(_pd or {}).get('person_iou'),
                            person_conf=(_pd or {}).get('person_conf'),
                            n_person_boxes=(_pd or {}).get('n_person_boxes'),
                            reject_reason=(_pd or {}).get('reject_reason'))
                        # N12 (fixed per codex review): capture BOTH failure
                        # candidates and a bounded sample of proof-passing ones;
                        # every row carries model SHAs/thresholds/run id and the
                        # supporting person box when one exists.
                        try:
                            _common = dict(
                                cls=_color, color_conf=round(float(conf), 3),
                                color_xyxy=[round(float(v), 1) for v in (x1, y1, x2, y2)],
                                person_iou=(_pd or {}).get('person_iou'),
                                person_conf=(_pd or {}).get('person_conf'),
                                n_person_boxes=(_pd or {}).get('n_person_boxes'),
                                reject_reason=(_pd or {}).get('reject_reason'),
                                person_xyxy=(_pd or {}).get('person_xyxy'),
                                shirt_fractions=(_pd or {}).get('shirt_fractions'),
                                range_m=round(float(rng), 2),
                                xyz=[round(wx, 2), round(wy, 2)],
                                image_stamp=frame_stamp,
                                camera_binding=frame_binding,
                                effective_conf=float(CONF),
                                path=('shared' if not _local_inference else 'local'),
                                **_EVIDENCE_FAIL.params())
                            if (not _person_proof) and rng <= 22.0 and conf >= 0.40:
                                _EVIDENCE_FAIL.capture(img, _common)
                                if _color == 'white':
                                    _EVIDENCE_WHITE_FAIL.capture(img, _common)
                            elif _person_proof and rng <= 22.0:
                                _EVIDENCE_PASS.capture(img, _common)
                        except Exception as _ev_err:
                            print("[pr] evidence capture error: %s" % _ev_err, flush=True)

                    # ---- 逐帧分母（N7）：这一帧模型到底出没出框 ----
                    # 放在几何门限之后、关联之前：此刻 res.boxes 是模型的原始输出，
                    # dets 是本帧过了宽松几何档的框，n_geo_rej 的差值是本帧被几何丢掉的。
                    # 记录本身不得影响主循环，所以整个写入用 try 兜住。
                    try:
                        _raw_xyxy = []
                        for _rb in res.boxes:
                            _rcid = int(_rb.cls)
                            _rx1, _ry1, _rx2, _ry2 = [float(v) for v in _rb.xyxy[0]]
                            _raw_xyxy.append((CLASSES[_rcid] if _rcid < len(CLASSES) else "?",
                                              float(_rb.conf), _rx1, _ry1, _rx2, _ry2))
                        _FRAMES.write(**frame_row(
                            summarize_raw_boxes(_raw_xyxy, ROI_TOP, ROI_BOT),
                            rospy.Time.now().to_sec(), frame_stamp, frame_age, inference_s,
                            len(dets), _stat["n_geo_rej"] - _geo_rej_before,
                            (px, py, pz), yaw_cam, (img.shape[1], img.shape[0]),
                            not _local_inference))
                    except Exception as _frame_probe_error:
                        print("[pr] frame_probe 写失败: %s" % _frame_probe_error, flush=True)

        # A completed inference with zero boxes is still a real search image.
        # Never report a received-only, failed, repeated or pre-grant image.
        if new_frame and processed_camera is not None and search_context is not None:
            processed_frame_seq += 1
            processed_s = rospy.Time.now().to_sec()
            with arb_lock:
                still_searching = camera_task.search_context(processed_s, frame_stamp)
            if still_searching == search_context:
                processed = dict(schema_version=1, run_id=os.environ['ROBOCUP_RUN_ID'],
                    uav_id=os.environ.get('PR_LOGICAL_UAV_ID', UAV),
                    generation=search_context['generation'], cell=search_context['cell'],
                    seq=processed_frame_seq, image_s=frame_stamp, sample_s=processed_s,
                    inference_complete=True, size=[int(img.shape[1]), int(img.shape[0])],
                    intrinsics=[float(FX), float(FY), float(CX), float(CY)],
                    camera_xyz=[float(px), float(py), float(pz)],
                    camera_rotation=[float(v) for v in R.reshape(-1)])
                if valid_frame(processed, processed_s):
                    processed_camera.publish(String(data=json.dumps(processed, allow_nan=False)))

        # ---------- 2) 关联（位置 + 身高 + 外观；v4新增多层次匹配）----------
        # 原理：先用位置粗关联，再用身高和外观精细筛选，解决密集目标串扰问题
        used = [False] * len(dets)
        # 外观相似度权重（国赛创新点：颜色特征辅助关联）
        APP_W = 0.3  # 外观权重，可根据实际效果调整
        for tk in (tracks if new_frame else []):
            dt = max(1e-3, frame_stamp - tk.t)
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
                if dd < bd:
                    # 外观特征只在已确认track且有有效特征时起作用
                    if confirmed and appear_sim > 0.3:  # 相似度阈值
                        # 融合位置距离与外观相似度
                        combined_score = dd - APP_W * appear_sim * gate
                        if combined_score < bd:
                            best, bd = j, combined_score
                            best_appear_score = appear_sim
                    else:
                        best, bd = j, dd
                elif confirmed and dd >= gate:
                    dyn_rej += 1                      # 被动态门限拒掉：统计用
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
                tk.update(d["xyz"][0], d["xyz"][1], dt, d["conf"],
                          d["uv"], d["wh"], d["range"], d["h"], frame_stamp, uav,
                          appearance_feat=appear_feat)
                tk.green_frame_proof = d.get('green_frame_proof',False)
                tk.person_support.observe(frame_stamp,d.get('person_frame_proof',False))
                tk.camera_xyz, tk.camera_s = [float(px),float(py),float(pz)], frame_stamp
            else:
                tk.coast(dt)
                tk.person_support.missed(frame_stamp)

        # ---------- 2b) 新建（只认严格档，误检建不了轨）----------
        for j, d in enumerate(dets):
            if not used[j]:
                if not d["strict"]:
                    _stat["n_new_rej"] += 1
                    continue
                _stat["n_new"] += 1
                # v4：创建track时保存外观特征
                tracks.append(Track(d["cls"], d["xyz"][0], d["xyz"][1], d["conf"],
                                    d["uv"], d["wh"], d["range"], d["h"], frame_stamp, uav,
                                    appearance_feat=d.get("appearance")))
                tracks[-1].green_frame_proof = d.get('green_frame_proof',False)
                tracks[-1].person_support.observe(frame_stamp,d.get('person_frame_proof',False))
                tracks[-1].camera_xyz, tracks[-1].camera_s = [float(px),float(py),float(pz)], frame_stamp

        # ---------- 3) 清理 ----------
        # 原判据 math.hypot(tk.x, tk.y) 是"距世界原点"的距离，与飞机在哪无关：
        # 飞机飞到 (-57,25) 后，靠近原点的合法目标会被误删，而正前方 100 m 外的
        # 误检反被留下（"MAX_RANGE*2=120" 也就是这么来的）。改成"距本帧飞机位置"，
        # 并用最后一次有效位姿，避免位姿取失败那帧拿 (0,0) 当飞机位置清空全部 track。
        if uav_last is None:
            tracks = [tk for tk in tracks if (now - tk.t) < TRACK_TIMEOUT]
        else:
            _ux, _uy = uav_last
            tracks = [tk for tk in tracks
                      if (now - tk.t) < TRACK_TIMEOUT
                      and math.hypot(tk.x - _ux, tk.y - _uy) < MAX_RANGE]

        # ---------- 4) 发布 ----------
        # 单流类：每类选"最像人"的 1 个 track。
        # red（双流）：走槽位持久绑定，两条流各锁一个红衣人（理由见 TOPIC_OF 注释）。
        best_of = {}
        red_cands = []
        with arb_lock:
            _cur_tid_for_verdict = camera_task.target(rospy.Time.now().to_sec()) if camera_task is not None else arb["tid"]
        for tk in tracks:
            if tk.hits < MIN_HITS:
                continue
            if tk.miss > MAX_COAST_PUB:   # 跟丢太久，坐标已是纯外推，不让它代表本类发布
                continue
            # v3.5：发布侧的人判决默认开启：静止建筑/招牌误检不能占用追踪机。
            # 真机若确实允许静止目标，可显式设置 PR_PUB_VERDICT=0 做对照实验；
            # 关闭后会放大假目标占机风险。
            _fresh_person=(FAST_RED_PERSON if tk.cls == 'red' else
                FAST_BLUE_PERSON if tk.cls == 'blue' else FAST_GREEN_WHITE) and tk.person_support.allowed(
                tk.cls,tk.miss,now,allow_red=FAST_RED_PERSON,allow_blue=FAST_BLUE_PERSON)
            if PUB_VERDICT:
                # A following camera keeps its assigned person centered at any
                # valid detection range. Only a live per-aircraft task skips
                # attachment; STOP/expiry and unrelated people keep this gate.
                _need_tid = TID_OF_COLOR.get(tk.cls)
                _skip_attach = (_cur_tid_for_verdict is not None and
                    (_cur_tid_for_verdict == _need_tid or
                     (tk.cls == 'red' and _cur_tid_for_verdict in ('t4','t5'))))
                _stationary = stationary_person_allowed(
                    _cur_tid_for_verdict,tk.cls,tk.green_frame_proof and STATIONARY_GREEN_ON,
                    tk.miss,tk.observed_s,now)
                _person_ok, _reject_reason = tk.verdict(
                    now, max_coast=MAX_COAST_PUB,
                    attach_check=not _skip_attach,
                    verified_person=_fresh_person,
                    stationary_person=_stationary)
                if not _person_ok:
                    # Record every input of the seven gates. RecentMotion.speed and
                    # FreshPerson.allowed are side-effect free, so an offline replay
                    # can re-decide this exact rejection under another setting.
                    try:
                        _recent_speed = (tk.recent_motion.speed(tk.observed_s if now is None else now)
                                         if tk.recent_motion is not None else None)
                    except Exception:
                        _recent_speed = None
                    # Measurability, recorded separately: RecentMotion.speed returns
                    # 0.0 both for "measured as stationary" and for "not enough
                    # samples to measure", and only the second case should fall back
                    # to sp/max_disp. Without this the two are indistinguishable
                    # offline (2026-10-07: white was rejected as static at 17 m with
                    # hits>=5, i.e. sparse sampling, not a stationary actor).
                    _motion_samples, _motion_span = None, None
                    try:
                        if tk.recent_motion is not None:
                            _now_s = tk.observed_s if now is None else now
                            _window = tk.recent_motion.window_s
                            _pts = [p for p in tk.recent_motion.samples
                                    if 0 <= _now_s - p[0] <= _window]
                            _motion_samples = len(_pts)
                            _motion_span = (_pts[-1][0] - _pts[0][0]) if len(_pts) >= 2 else 0.
                    except Exception:
                        _motion_samples, _motion_span = None, None
                    try:
                        _attached = bool(tk.attached_to_cam())
                    except Exception:
                        _attached = None
                    _CSV.write(
                        ros_time=rospy.Time.now().to_sec(), event="track_reject",
                        target_id=("%s%d" % (tk.cls, 0) if tk.cls != "red" else "red"),
                        class_name=tk.cls, confidence=tk.conf,
                        bbox_u=tk.uv[0], bbox_v=tk.uv[1], bbox_w=tk.wh[0],
                        bbox_h=tk.wh[1], target_x=tk.x, target_y=tk.y,
                        target_z=TARGET_Z, range_m=tk.rng, height_m=tk.h,
                        strict=True, track_id=tk.id, hits=tk.hits,
                        image_stamp=frame_stamp, image_age_s=frame_age,
                        inference_s=inference_s, source="verdict_%s" % _reject_reason,
                        person_hits=tk.person_support.hits,track_miss=tk.miss,fresh_person_allowed=_fresh_person,
                        person_established=tk.person_support.established,
                        person_current_verified=tk.person_support.current_verified,
                        recent_speed=_recent_speed, sp=tk.sp, max_disp=tk.max_disp,
                        score_ema=tk.score_ema, attached=_attached,
                        confirm_hits=CONFIRM_HITS, verdict_score=VERDICT_SCORE,
                        verdict_sp=VERDICT_SP, verdict_disp=VERDICT_DISP,
                        stationary_person=_stationary,
                        person_gate_allowed=tk.person_support.allowed(
                            tk.cls,tk.miss,now,allow_red=FAST_RED_PERSON),
                        green_frame_proof=bool(tk.green_frame_proof),
                        motion_samples=_motion_samples, motion_span_s=_motion_span,
                        original_s=tk.observed_s,raw_target_x=tk.raw_xy[0],raw_target_y=tk.raw_xy[1],
                        blue_identity_qualified=tk.blue_identity.qualified if tk.blue_identity is not None else None,
                        blue_identity_allowed=tk.blue_identity.allowed(now) if tk.blue_identity is not None else None)
                    continue
            # 运动性是随时间累积的，coast 期间净位移也在涨，这里按当前时刻重算
            tk.score = tk.conf * person_likelihood(tk.h) * tk.motion_factor(now)
            if tk.cls == "red":
                red_cands.append(tk)      # red 不在这里决出，交给下面的槽位绑定
                continue
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
        # Discovery precedes assignment; retain every person check above.
        # The manager needs these observations before it can assign a tracker.
        visual_pub_list = tuple(pub_list)

        # ---- 官方话题闸门：指派仲裁 + 近距离 + 空间身份一致 ----
        # 未过闸不发布 ActorInfo（在进入内部调试快照链路前剔除）。
        with arb_lock:
            cur_tid = camera_task.target(rospy.Time.now().to_sec()) if camera_task is not None else arb["tid"]
        now_secs = rospy.Time.now().to_sec()
        gated_list = []
        for cls, si, tk in pub_list:
            need_tid = TID_OF_RED_SLOT[si] if cls == "red" else TID_OF_COLOR.get(cls)
            if tk.rng > ACTOR_PUB_RANGE:
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
        if len(gated_list) != len(pub_list):
            rospy.loginfo_throttle(
                5, "[pr] 官方话题闸门外挂 %d/%d 条（指派=%s, rng<=%.0fm）",
                len(pub_list) - len(gated_list), len(pub_list), cur_tid, ACTOR_PUB_RANGE)
        pub_list = gated_list

        snap = []
        for cls, si, tk in pub_list:
            px, py = tk.pub_xy()          # 补偿后的对外坐标（见 LAG_COMP / Track.pub_xy）
            # 2026-10-01: 本机不再直发 /actor_<color>_info，改由 yolo_target_bridge
            # 用多机融合坐标统一发布（单一仲裁者，避免 6 机同色位置冲突让裁判反复重置）。
            # m = ActorInfo(cls=cls, x=round(px, 2), y=round(py, 2))
            # pubs[cls][si].publish(m)
            tag = ("%s%d" % (cls, si + 1)) if len(pubs[cls]) > 1 else cls
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
                source="perception_track",person_hits=tk.person_support.hits,
                track_miss=tk.miss,fresh_person_allowed=(FAST_RED_PERSON if cls == 'red' else
                    FAST_BLUE_PERSON if cls == 'blue' else FAST_GREEN_WHITE)
                    and tk.person_support.allowed(cls,tk.miss,now,allow_red=FAST_RED_PERSON,allow_blue=FAST_BLUE_PERSON),
                person_established=tk.person_support.established,
                person_current_verified=tk.person_support.current_verified,
                original_s=tk.observed_s,raw_target_x=tk.raw_xy[0],raw_target_y=tk.raw_xy[1],
                blue_identity_qualified=tk.blue_identity.qualified if tk.blue_identity is not None else None,
                blue_identity_allowed=tk.blue_identity.allowed(now) if tk.blue_identity is not None else None)

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

        # ---- 协同上报（契约 v0/schema v1）：每目标一条，按 COORD_HZ 节流 ----
        # 位置必须在上面那段诊断打印**之后**：那段的缩进属于 `if snap:`，
        # 若把本块插在中间，Python 会把诊断打印并进本块的 if 里 ——
        # 于是 `PR_COORD_ON=0` 时连日志一起消失，静默丢掉排查手段。
        # 为什么要节流：核心按自己 tick_hz 从单写者队列里取，我按检测频率灌
        # 6 条/帧会把它堆成积压（queue_size=50 + 无界 deque），既不加分也拖延迟。
        # 队友替身用的就是 2 Hz。
        if COORD_ON and visual_pub_list and (now - _coord_t) >= 1.0 / max(0.1, COORD_HZ):
            _coord_t = now
            for _cls, _si, _tk in visual_pub_list:
                _tag = ("%s%d" % (_cls, _si + 1)) if len(pubs[_cls]) > 1 else _cls
                _obs_seq += 1
                _cx, _cy = _tk.pub_xy()      # 与裁判侧同一套补偿，两边坐标必须一致
                if (_cls, _si, _tk) in pub_list:
                    coord.publish(String(data=json.dumps({
                    "target_id": _tag,
                    "frame_id": "world_enu",
                    "xyz": [round(_cx, 2), round(_cy, 2), TARGET_Z],
                    "confidence": 1.0,
                        "observation_id": "obs-%s-%d" % (_tag, _obs_seq)})))
                # Only an actual match in this camera frame creates new evidence.
                # Track prediction/coast may support UI, but cannot extend confirmation.
                if (_tk.miss == 0 and 0 < _tk.observed_s <= now <= _tk.observed_s+1.
                        and getattr(_tk,'camera_s',None) == _tk.observed_s
                        and _tk.observed_s > _published_visual_samples.get(_tag, 0.)):
                    _logical_uid = os.environ.get('PR_LOGICAL_UAV_ID', UAV)
                    _visual_run = os.environ.get('ROBOCUP_RUN_ID', '')
                    # The bridge compensates from the original image time.
                    # Do not label a lagging prefiltered point as a raw sample.
                    _original_colors = os.environ.get('PR_ORIGINAL_REPORT_COLORS', '').split(',')
                    _original_xy = image_time_position(_cls, _tk.raw_xy, (_tk.x, _tk.y), _original_colors)
                    visual_coord.publish(String(data=json.dumps(dict(schema_version=3,
                        run_id=_visual_run, uav_id=_logical_uid, seq=_obs_seq, sample_s=_tk.observed_s,
                        target_id=_tag, frame_id='world_enu', xyz=[round(_original_xy[0], 2), round(_original_xy[1], 2), TARGET_Z],
                        confidence=float(_tk.conf), camera_xyz=list(_tk.camera_xyz),
                        person_frame_verified=bool(_tk.person_support.current_verified),
                        observation_id='%s:%s:%d' % (_visual_run, _logical_uid, _obs_seq)),
                        allow_nan=False)))
                    _published_visual_samples[_tag] = _tk.observed_s
                # === 仿真环境日志：YOLO 检测输出 + ROS 时间戳 ===
                # 排查感知延迟/位置滞后：同时打检测框(uv)、世界坐标(xyz)、置信度、时间戳
                _uv = getattr(_tk, 'uv', (None, None))
                rospy.loginfo_throttle(
                    0.5,
                    "[PERC] yolo uav=%s target=%s uv=(%s,%s) world_xyz=(%.2f,%.2f,%.1f) "
                    "conf=%.3f | img_ts=%.3f | obs_id=obs-%s-%d",
                    UAV, _tag,
                    "%.0f" % _uv[0] if _uv[0] is not None else "NA",
                    "%.0f" % _uv[1] if _uv[1] is not None else "NA",
                    _cx, _cy, TARGET_Z, float(_tk.conf), frame_stamp, _tag, _obs_seq)

        # ---------- 5) 标注帧：只画"那个人" ----------
        # v3.4：只画最终进入 best_of 的 track（每类最优），默认再取 score 最高的 1 个。
        # v3.5：再叠一层 **人判决** —— 必须已确认、身高像人、而且**在动**。
        #       这样画出来的框 == 上报给裁判的那个目标 == 真的在走的人；
        #       白墙/招牌/建筑色条/自身起落架 这些静止误检一律不画。
        if ANNOT_ON and annot_img is not None and annot_z >= ANNOT_MIN_Z:
            cands = []                     # [(类, 槽号, track)]
            for _cls, _si, _tk in pub_list:
                if ANNOT_CLS and _cls != ANNOT_CLS:
                    continue
                if ANNOT_VERDICT:
                    ok, why = _tk.verdict(now, max_coast=ANNOT_MAX_COAST)
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
                        if len(pubs[_c]) > 1:
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
                            nm = ("%s%d" % (_cls, _si + 1)) if len(pubs[_cls]) > 1 else _cls
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
            _t_live = _now
            _img_stamp = _latest.get("stamp", 0.0) if _latest else 0.0
            print("[pr] 心跳 loop=%d img_ts=%.3f img_age=%.2fs n_det=%d "
                  "z=%.2fm pub=%d coord=%s cam=(%.1f,%.1f,%.1f)"
                  % (n_loop, _img_stamp,
                     (_now - _img_stamp) if _img_stamp else -1.0,
                     _stat.get("n_det", 0), annot_z,
                     len(pub_list), "ON" if COORD_ON else "OFF",
                     _pose_history[-1][1] if _pose_history else 0.0,
                     _pose_history[-1][2] if _pose_history else 0.0,
                     _pose_history[-1][3] if _pose_history else 0.0), flush=True)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
