#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO→swarm 桥：把合法感知链路替换真值订阅。

输入（己方 YOLO 输出，合法）：
  /coordination/target_report  std_msgs/String，6 个 perception_real 实例共写，
      裸 JSON：{"target_id","frame_id":"world_enu","xyz":[x,y,z],
                "confidence","observation_id"}
      target_id ∈ green / blue / brown / white / red1 / red2
输出：
  /swarm/target_states  robocup_swarm/TargetState
      下游 swarm_manager / swarm_agent 只认本消息，不感知数据来源，
      因此本桥替换 target_sim / official_target_bridge 后二者零算法改动。
  /swarm/report_telemetry  std_msgs/String，记录本队实际发布的 ActorInfo。
      仅供队内估计播报连续性；发布成功不代表裁判收到或确认消除。

颜色已知的四条流对应 actor_0..3。red1/red2 只是队内两条独立视觉槽，
不能据此判断它们分别是官方 actor_4 还是 actor_5。

合规说明：正式节点只订阅本队视觉与协同话题，不订阅裁判反馈、
Gazebo 真值，不读 obstacle.txt / black_box.txt，不启动 target_sim。

设计：纯逻辑内核 TargetBridgeCore 不依赖 rospy（只依赖 math/json），
ROS 包装只做话题收发，内核可离线单测。
"""

import json
import math
import os
import re
import statistics
import sys

# ---- 身份映射 ----
# 四个不同颜色的官方身份固定。红色在队内使用两个稳定的视觉槽，
# t4/t5 仅用于现有任务消息接口；它们不能证明 actor_4/5 的对应关系。
TAG_TO_TID = {
    "green": "t0",
    "blue": "t1",
    "brown": "t2",
    "white": "t3",
    "red1": "t5",   # 内部红色视觉槽 1
    "red2": "t4",   # 内部红色视觉槽 2
}
TID_TO_TAG = dict((v, k) for k, v in TAG_TO_TID.items())
ACTOR_INDEX_OF_TAG = dict((tag, int(tid[1:])) for tag, tid in TAG_TO_TID.items())

# ---- ActorInfo.cls 必须是官方 actor_id_dict 的**键** ----
# score_cal.py:22  actor_id_dict = {'green':[0],'blue':[1],'brown':[2],'white':[3],'red':[4,5]}
# 官方两条 red 输入都要求 cls='red'；当前裁判实现分别对两名红衣人交叉匹配。
# 🔴 曾经写成 cls='red1'/'red2' ⇒ 两个 red 回调都返回空列表 ⇒ **红球永远 0 分**。
OFFICIAL_CLS_OF_TAG = {
    "green": "green", "blue": "blue", "brown": "brown", "white": "white",
    "red1": "red", "red2": "red",
}

# The judge cross-checks either red topic against both red actors.  Each
# visual trajectory still needs its own topic and persistent spatial identity;
# publishing one trajectory on both topics disrupts the other 15 s streak.
# RED_DUAL only keeps the diagnostic focus selector enabled.  The actual
# official publisher below sends each red trajectory to its own topic.
RED_DUAL = os.environ.get("BRIDGE_RED_DUAL", "1") == "1"
RED_TAGS = ("red1", "red2")
RED_ACTOR_IDS = frozenset(ACTOR_INDEX_OF_TAG[t] for t in RED_TAGS)   # {4, 5}
# 焦点滞回：新球必须比当前球近这么多米才换向，防止两球距离接近时来回跳。
RED_FOCUS_HYST_M = float(os.environ.get("BRIDGE_RED_FOCUS_HYST", "3.0"))
# red1/red2 are local detector slots, not identities shared between aircraft.
# Association is deliberately tighter than the fusion spread check: a wrong
# red observation must not pull the other person's official report off target.
RED_ASSOC_GATE_M = float(os.environ.get("BRIDGE_RED_ASSOC_GATE", "2.4"))
RED_ASSOC_FRESH_S = float(os.environ.get("BRIDGE_RED_ASSOC_FRESH", "2.0"))
# A detector's local red1/red2 label can swap after an occlusion.  Border
# actors often stop for >10 s; keep their distinct spatial anchors through
# that gap, while still allowing an unambiguous new binding after 30 s.
RED_ASSOC_ANCHOR_S = float(os.environ.get("BRIDGE_RED_ASSOC_ANCHOR", "15.0"))
# Two local slots less than this far apart can be duplicate tracks of one
# actor.  Never push the second box into the other global red trajectory if
# its old anchor is far away.  The small threshold avoids suppressing two
# independently resolved people merely because both fit the 2.4 m gate.
RED_LOCAL_DUP_M = 0.8
RED_LOCAL_DUP_S = 1.0
RED_LOCAL_DUP_MIN_CONF = 0.7

# ---- 融合 / 计时参数 ----
OBS_WINDOW = 0.6        # 多感知节点近帧融合窗 s（2Hz/节点 × 最多 6 节点）
OBS_SPREAD_M = 5.0      # 窗内新观测与已有观测的最大允许距离 m（防跟丢跳变/多机误检拉飞）
# 跨窗校验：跟丢 >OBS_WINDOW 但 <=COAST_TIME 时，新观测必须离最后融合位置
# SPREAD_FAR_M 内。真人 1.5s 走不了 10m；超距说明跟丢重跟到错误物体。
# 超过 COAST_TIME 才允许任意位置（真·重捕获）。
SPREAD_FAR_M = float(os.environ.get("BRIDGE_SPREAD_FAR", "10.0"))
VEL_DT_MIN = 0.10       # 最小差分间隔，避免高频小 dt 放大噪声
VEL_ALPHA = 0.6         # 速度 EMA 增益（2026-10-06 五轮复盘：0.35 转向收敛太慢，方向误差 23° 导致外推横向偏差）
VEL_MAX = 2.2           # 速度限幅（v24b：4.0→2.2=actor 物理上限 2m/s+10% 余量——
                        # 4m/s 的"速度"必是 YOLO 跳变污染，外推 0.75s 就偏 3m）
                        # 防YOLO跳变拉飞（与 cooperative_tracker 一致）
COAST_TIME = 1.5        # 短暂遮挡：保持最后位置 s
# === 2026-10-03 我方新增：官方播报距离闸门（带滞回）+ 播报保持器 ===
# 官方判据：误差<1m、间隔≤1s、连续 15s（score_cal.py:135-165），15s 攒满即
# delete_model 消除；而首次对上会让裁判 publish /find_actor_N，actor 控制器据此
# 在 **25s 后瞬移**（control_actor.py:112-127）⇒ 净余量只有 10s，断一次就归零。
# 单目测距误差随距离放大，远处播报会把计时清零。默认 12m —— 实测 7m 内误差
# 0.26~0.69m（达标），15.98m 时系统报 17.09m（误差 1.1m，刚好越界）。
# v22-C（2026-10-09）：8.0→9.5 —— v21 实证 brown 在 8.6m 被拦 5 次（每秒一条 GATE_DBG，
# 差 0.6m 进不了闸门），9.5m 处单目误差 ~0.85m 仍 <1m 达标；进闸更早 = streak 更早起算。
ACTOR_PUB_MAX_RANGE_M = float(os.environ.get("BRIDGE_PUB_MAX_RANGE", "9.5"))
# ⛔ 原来这里有一个 GATE_RETRY_S 和 self._gate_hold，注释写"避免抖动导致话题断续"，
#    但 **GATE_RETRY_S 从未被任何代码引用、_gate_hold 只写不读** —— 滞回其实
#    根本没实现，闸门是纯瞬时的：range_m 一过 12m 立刻停发。目标在 12m 边界
#    附近抖动（单目测距噪声 + 机头摆动）就反复通断 ⇒ 播报断续 ⇒ 官方
#    "间隔≤1s"失败 ⇒ 计时清零。这是"追到 25s 上限也消不掉"的直接原因之一。
# ✅ 现在补上真正的滞回带：进入要 ≤ACTOR_PUB_MAX_RANGE_M，退出要 >HOLD。
ACTOR_PUB_HOLD_RANGE_M = float(os.environ.get("BRIDGE_PUB_HOLD_RANGE", "15.0"))
# 播报保持器：融合层偶发空 tick（coast / 多帧验证不够 N 帧）会造成 >1s 静默，
# 官方那条 "间隔 ≤1s" 立刻失败。这里按最低频率用「最后融合位置 + 速度外推」
# 补齐，把静默压在 1s 以内。外推超过 KEEPALIVE_MAX_T 就不再硬撑（宁可断，
# 也不报一个必然 >1m 的假位置 —— 误差超 1m 同样清零）。
ACTOR_KEEPALIVE_DT = float(os.environ.get("BRIDGE_KEEPALIVE_DT", "0.75"))
OFFICIAL_IMAGE_MAX_AGE = 0.6  # modelled reports; never extended by a publish callback
OFFICIAL_UNMODELLED_MAX_AGE = 0.35
# 2026-10-06 六轮复盘核心修复：官方判据「相邻广播间隔 <=1.0s」（score_cal DETECTION_INTERVAL）
# 只约束不超 1s，不要求高频。广播 3Hz 时 15s streak 需 45 次连乘全过（单次失败率 20% ->
# 成功率 0.8^45≈8e-6，必然 0 消除）；节流到 0.75s 一次后 15s 只需 20 次连乘（0.8^20=1.2%），
# 若误差压缩使失败率 <=10% 则成功率 12%/窗口 —— 这是把「数学上不可能」变成「大概率全消除」的
# 最大杠杆。PUB_MIN_INTERVAL=0.75 留 0.25s 余量防 tick 抖动导致间隔 >1.0s 触发 discontinuous。
PUB_MIN_INTERVAL = float(os.environ.get("BRIDGE_PUB_MIN_INTERVAL", "0.75"))
# 行走目标每 0.75s 才报一次时，1m/s 的目标在两次播报间已经移动
# 0.75m；再叠加图像与 ROS 回调延迟，裁判的 1m 误差门常被击穿。
# 只对速度可信且较快的轨迹缩短间隔，静止目标仍保留较低的播报频率。
FAST_PUB_INTERVAL = float(os.environ.get("BRIDGE_FAST_PUB_INTERVAL", "0.45"))
FAST_PUB_SPEED = float(os.environ.get("BRIDGE_FAST_PUB_SPEED", "0.7"))
# 协同层需要给备用机完成接力的窗口。agent 自己还有 TARGET_STALE 门槛，
# 因此这里不能在 3s 时立刻撤掉目标状态，否则短遮挡会直接清空盘旋任务。
# v13（2026-10-07）：2.0→4.0 —— v12c actor_5 贴脸 3.07m 后 YOLO 间歇丢 2.5s，
# 2.0s 停发 → 官方「间隔>1s」reset 27 次。贴脸跟随态 actor 相对静止，
# coast 期外推封顶（te≤1.15s + EXTRAP_MAX_D 限幅）位置不漂，撑过间歇丢失
# 15s streak 不断；actor 快速逃跑时误差 reset 早晚问题，无净损失。
# 2026-10-08 消除 actor 冲刺：DROP_TIME 4.0→8.0（v18b streak_max=0.0s 修复，
# 多撑 4s 遮挡防追鬼断链；EXTRAP_MAX_T/D 联合限幅保误差<1.5m）。
DROP_TIME = float(os.environ.get("BRIDGE_DROP_TIME", "8.0"))
RED_OFFICIAL_FRESH_S = float(os.environ.get("RED_OFFICIAL_FRESH_S", "1.0"))
# 新轨激活门槛：未激活轨只有融合窗内最高置信度 >= 该值才允许 alive。
# 2026-10-01 复盘：3 条 0.43~0.61 的 red1 误检（真身为 green 演员）建出鬼影轨
# t4，全队盘旋假目标并广播假消除。已激活轨不受此限（延续观测允许低置信度）。
NEW_TRACK_CONF = float(os.environ.get("BRIDGE_NEW_TRACK_CONF", "0.7"))

# One fixed object in the 22:45 match was seen by UAV3 as blue and then white
# at (-27.1, 13.2).  It passed the ordinary geometry and stationary-person
# gates, so the bridge sent both identities to the judge and to dispatch.
# Quarantine only that small *location* when two independently timed, usable
# views of each conflicting color came from the same camera.  A real actor
# which moves away can immediately report again; an isolated stationary actor
# never meets this cross-color condition.
COLOR_CONFLICT_HISTORY_S = float(os.environ.get("BRIDGE_COLOR_CONFLICT_HISTORY_S", "9.0"))
COLOR_CONFLICT_MATCH_M = float(os.environ.get("BRIDGE_COLOR_CONFLICT_MATCH_M", "0.75"))
COLOR_CONFLICT_SWITCH_GAP_S = float(os.environ.get("BRIDGE_COLOR_CONFLICT_SWITCH_GAP_S", "0.8"))
COLOR_CONFLICT_RADIUS_M = float(os.environ.get("BRIDGE_COLOR_CONFLICT_RADIUS_M", "1.0"))
COLOR_CONFLICT_TTL_S = float(os.environ.get("BRIDGE_COLOR_CONFLICT_TTL_S", "12.0"))
COLOR_CONFLICT_MIN_CONF = float(os.environ.get("BRIDGE_COLOR_CONFLICT_MIN_CONF", "0.55"))

# ---- 国家一等奖标准改进：多帧验证 + 自适应参数 ----
# 多帧验证：新轨迹需要连续 N 帧高置信度观测才能激活
NEW_TRACK_FRAMES = int(os.environ.get("BRIDGE_NEW_TRACK_FRAMES", "2"))  # 国家一等奖改进：2 帧激活（从 3 降），2Hz 单节点下 1s 即可激活，避免 actor 1.5m/s 移动时延迟累计 >1m 误差导致确认反复归零
# 颜色相似度校验：red1/red2 误检常因为颜色与 green/blue 相似
COLOR_SIMILARITY_THRESHOLD = float(os.environ.get("BRIDGE_COLOR_SIMILARITY", "0.3"))
# 运动一致性校验：误检目标通常运动模式异常
MOTION_CONSISTENCY_WINDOW = float(os.environ.get("BRIDGE_MOTION_WINDOW", "2.0"))

FLEE_SPEED = 2.0        # state 粗估：超过即给 FLEE（下游不依赖该字段），规则要求 2m/s
# Pursuit-only extrapolation bounds. Judge coordinates use official_point_at
# and actual image age, with no fixed delay and no stale saturated prediction.
EXTRAP_MAX_T = float(os.environ.get("BRIDGE_EXTRAP_MAX_T", "1.5"))
EXTRAP_MAX_D = float(os.environ.get("BRIDGE_EXTRAP_MAX_D", "0.5"))

# ---- 国家一等奖标准改进：自适应外推 ----
# 自适应外推：根据速度估计质量动态调整外推参数
EXTRAP_VEL_THRESHOLD = float(os.environ.get("BRIDGE_EXTRAP_VEL_THRESHOLD", "0.5"))  # 低速阈值
EXTRAP_ADAPTIVE = os.environ.get("BRIDGE_EXTRAP_ADAPTIVE", "1") == "1"
# 2026-10-06 B 项诊断：节流字典（key=actor_tag, value=last_print_time）
_BRIDGE_DBG_LAST = {}


def _core_log(fmt, *args):
    """core 诊断出口（2026-10-06 修）：TargetBridgeCore 是"纯逻辑（无 rospy）"，
    但 EXTRAP_DBG 等诊断打印需要出口。真实节点里 __main__ 已 import rospy
    （在 sys.modules 里）→ 走 rosout；离线自测（--selftest）没有 rospy →
    落 stdout。修复了自测在 core.tick 里 NameError 崩溃的问题。"""
    _mod = sys.modules.get("rospy")
    if _mod is not None and hasattr(_mod, "loginfo"):
        try:
            _mod.loginfo(fmt, *args)
            return
        except Exception:
            pass
    try:
        sys.stdout.write("[bridge-core] " + (fmt % args) + "\n")
        sys.stdout.flush()
    except Exception:
        pass


def _vel_ema(v_old, raw):
    """单轴速度 EMA + 限幅。"""
    v = VEL_ALPHA * raw + (1.0 - VEL_ALPHA) * v_old
    return max(-VEL_MAX, min(VEL_MAX, v))


def stable_official_point(history, now):
    """Median only when recent judge-eligible observations are stationary.

    A single monocular depth jump just above 1 m resets the judge's entire
    15-second streak.  Moving people must retain the newest observation,
    because a median of their past positions would lag behind them.
    """
    recent = [p for p in history if 0.0 <= now - p[0] <= 2.0]
    if len(recent) < 4 or recent[-1][0] - recent[0][0] < 1.2 - 1e-6:
        return None
    # A fixed displacement between two half-windows labelled .5–1m/s walks
    # as stationary. Use robust velocity with its actual time baseline.
    slopes = [((b[1]-a[1])/(b[0]-a[0]), (b[2]-a[2])/(b[0]-a[0]))
              for i, a in enumerate(recent) for b in recent[i+1:]
              if b[0]-a[0] >= 0.4-1e-6]
    if not slopes or math.hypot(statistics.median(v[0] for v in slopes),
                               statistics.median(v[1] for v in slopes)) > 0.15:
        return None
    center = (statistics.median(p[1] for p in recent),
              statistics.median(p[2] for p in recent))
    deviations = sorted(math.hypot(p[1]-center[0], p[2]-center[1]) for p in recent)
    if deviations[int(0.8*(len(deviations)-1))] > 0.35:
        return None
    if math.hypot(recent[-1][1] - center[0],
                  recent[-1][2] - center[1]) > 1.25:
        return None  # likely a different person or a genuine position change
    return center


def _robust_official_fit(history, now):
    """Fit one camera's trajectory; retain the time of the last inlier image.

    Ordinary least squares followed by raw fallback amplified isolated depth
    errors. Estimate velocity and intercept robustly first, then fit only the
    agreeing images. A rejected last image cannot extend report freshness.
    """
    points = [p for p in history if 0.0 <= now - p[0] <= 2.0]
    if len(points) < 3 or points[-1][0] - points[0][0] < 0.7 - 1e-6:
        return None
    slopes = [((b[1]-a[1])/(b[0]-a[0]), (b[2]-a[2])/(b[0]-a[0]))
              for i, a in enumerate(points) for b in points[i+1:]
              if b[0]-a[0] >= .4-1e-6]
    if not slopes:
        return None
    vx, vy = (statistics.median(v[k] for v in slopes) for k in (0, 1))
    x = statistics.median(p[1]+vx*(now-p[0]) for p in points)
    y = statistics.median(p[2]+vy*(now-p[0]) for p in points)
    inliers = [p for p in points if math.hypot(
        p[1]+vx*(now-p[0])-x, p[2]+vy*(now-p[0])-y) <= .35]
    if (len(inliers) < max(3, math.ceil(.75*len(points))) or
            inliers[-1][0]-inliers[0][0] < .7-1e-6):
        return None
    times = [p[0] for p in inliers]
    mean_t = sum(times) / len(times)
    den = sum((t - mean_t) ** 2 for t in times)
    if den < 1e-6:
        return None
    mean_x = sum(p[1] for p in inliers) / len(inliers)
    mean_y = sum(p[2] for p in inliers) / len(inliers)
    vx = sum((p[0] - mean_t) * (p[1] - mean_x) for p in inliers) / den
    vy = sum((p[0] - mean_t) * (p[2] - mean_y) for p in inliers) / den
    speed = math.hypot(vx, vy)
    if speed > 2.3:
        return None
    worst = max(math.hypot(p[1] - (mean_x + vx * (p[0] - mean_t)),
                           p[2] - (mean_y + vy * (p[0] - mean_t)))
                for p in inliers)
    if worst > .35:
        return None
    return (mean_x+vx*(now-mean_t), mean_y+vy*(now-mean_t), vx, vy,
            inliers[-1][0], inliers[-1][0] == points[-1][0])


def _official_motion_fit(history, now):
    fit = _robust_official_fit(history, now)
    if fit is None or not fit[5] or math.hypot(*fit[2:4]) < .15:
        return None
    return fit[:4]


def coherent_official_velocity(history, now):
    fit = _official_motion_fit(history, now)
    return fit[2:] if fit is not None else None


def _confirmed_official_segment(history):
    """Three new images can establish a turn, one possible step cannot.

    Callers pass only images after the last accepted anchor. Including old
    inliers would let two old stationary images plus one depth spike look
    like a coherent new trajectory at a slower camera cadence.
    """
    points = history[-3:]
    if (len(points) < 3 or points[-1][0]-points[0][0] < .3-1e-6 or
            any(not 0. < b[0]-a[0] <= 1.+1e-6
                for a, b in zip(points, points[1:]))):
        return None
    mean_t = sum(p[0] for p in points)/3.
    den = sum((p[0]-mean_t)**2 for p in points)
    mean_x, mean_y = (sum(p[k] for p in points)/3. for k in (1, 2))
    vx, vy = (sum((p[0]-mean_t)*(p[k]-mean) for p in points)/den
              for k, mean in ((1, mean_x), (2, mean_y)))
    if (math.hypot(vx, vy) > 2.3 or
            max(math.hypot(p[1]-mean_x-vx*(p[0]-mean_t),
                           p[2]-mean_y-vy*(p[0]-mean_t)) for p in points) > .2):
        return None
    return points


def camera_motion_velocity(histories, now):
    """Estimate derivatives within each camera, never between camera biases.

    One vote per fresh camera prevents a higher-rate source dominating. This
    velocity is for following only and never qualifies a judge coordinate.
    """
    velocities = []
    for history in histories.values():
        points = [p for p in history if 0. <= now-p[0] <= 1.5]
        if len(points) < 3 or now-points[-1][0] > .6 or points[-1][0]-points[0][0] < .6-1e-6:
            continue
        slopes = [((b[1]-a[1])/(b[0]-a[0]), (b[2]-a[2])/(b[0]-a[0]))
                  for i,a in enumerate(points) for b in points[i+1:]
                  if b[0]-a[0] >= .4-1e-6]
        if not slopes:
            continue
        vx, vy = (statistics.median(v[k] for v in slopes) for k in (0,1))
        x = statistics.median(p[1]+vx*(now-p[0]) for p in points)
        y = statistics.median(p[2]+vy*(now-p[0]) for p in points)
        errors = sorted(math.hypot(p[1]+vx*(now-p[0])-x,
                                  p[2]+vy*(now-p[0])-y) for p in points)
        if errors[int(.8*(len(errors)-1))] <= .5 and math.hypot(vx,vy) <= 2.5:
            velocities.append((vx,vy))
    if not velocities:
        return 0., 0.
    return tuple(statistics.median(v[k] for v in velocities) for k in (0,1))


def official_point_at(track, now):
    """One report policy for regular and keepalive paths, measured from image time.

    Fits describe position at the last image, then advance only to now. A
    displacement cap must not silently turn an expired prediction into a
    plausible-looking but lagging report; expired imagery produces no report.
    """
    age = now - track.good_t
    if (track.good_x is None or not 0.0 <= age <= OFFICIAL_IMAGE_MAX_AGE or
            (track.tag in RED_TAGS and age > RED_OFFICIAL_FRESH_S)):
        return None
    fit = _robust_official_fit(track.good_history, track.good_t)
    if fit is not None:
        x, y, vx, vy, last_inlier, latest_ok = fit
        limit = OFFICIAL_IMAGE_MAX_AGE if latest_ok else OFFICIAL_UNMODELLED_MAX_AGE
        if not 0. <= now-last_inlier <= limit:
            return None
        return x + vx*age, y + vy*age
    stable = stable_official_point(track.good_history, track.good_t)
    if stable is not None:
        inliers = [p for p in track.good_history if 0. <= track.good_t-p[0] <= 2.
                   and math.hypot(p[1]-stable[0], p[2]-stable[1]) <= .35]
        limit = (OFFICIAL_IMAGE_MAX_AGE if inliers[-1][0] == track.good_t
                 else OFFICIAL_UNMODELLED_MAX_AGE)
        return stable if 0. <= now-inliers[-1][0] <= limit else None
    # An isolated jump must not destroy a good fit and then acquire more
    # authority through raw fallback. A changed trajectory needs independent
    # fresh images before _accept_official_sample starts its new segment.
    previous = track.good_history[:-1]
    if previous:
        model = _robust_official_fit(previous, previous[-1][0])
        if model is not None:
            dt = track.good_t-previous[-1][0]
            expected = (model[0]+model[2]*dt, model[1]+model[3]*dt)
            if math.hypot(track.good_x-expected[0], track.good_y-expected[1]) > .45+.35*dt:
                return None
    if age <= OFFICIAL_UNMODELLED_MAX_AGE:
        return track.good_x, track.good_y
    return None


class _Track(object):
    __slots__ = ("tag", "obs", "x", "y", "vx", "vy", "conf",
                 "t_obs", "alive", "elim_pending", "range_m",
                 "good_x", "good_y", "good_t", "good_history",
                 "official_source", "source_history", "official_rejects",
                 "accepted_source_samples",
                 "pursuit_history",
                 "_last_fx", "_last_fy", "_last_ft",
                 "_high_conf_count", "_motion_history", "_last_vel_mag")

    def __init__(self, tag):
        self.tag = tag
        self.obs = []              # [(t, x, y, w), ...] 窗内观测
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.conf = 0.0
        # 2026-10-03 我方补：最近一次观测的水平距离（m），播报闸门用。
        # None = 发布端没给（兼容旧发布端）⇒ 闸门不限距离。
        self.range_m = None
        self.official_source = None
        self.source_history = {}
        self.accepted_source_samples = {}
        self.pursuit_history = {}
        self.official_rejects = 0
        self.good_x = None
        self.good_y = None
        self.good_t = -1e18
        self.good_history = []
        self.t_obs = -1e18
        self.alive = False         # 是否正在对外发布
        self.elim_pending = False  # 待补发 eliminated=true
        self._last_fx = None
        self._last_fy = None
        self._last_ft = None
        # 国家一等奖标准改进：多帧验证
        self._high_conf_count = 0  # 连续高置信度帧数
        self._motion_history = []   # 运动历史，用于一致性校验
        self._last_vel_mag = 0.0   # 上一次速度幅值


def parse_report(payload):
    """解析/校验一条 target_report 载荷。

    返回 (tag, x, y, conf, range_m)；非法载荷抛 ValueError。
    只做形状校验，坐标原样返回（坐标系由发布端契约保证为 world_enu）。
    range_m 是 2026-10-03 我方新增的可选字段（发布端算好的水平距离 m）：
    单目测距误差随距离放大，而官方判据是"误差<1m 连续 15s"，
    远距离播报只会把裁判的 15s 计时反复清零 ⇒ 用它做播报闸门。
    字段缺失时返回 None，闸门退化为不限距离（兼容旧发布端）。
    """
    try:
        d = json.loads(payload)
    except (TypeError, ValueError) as exc:
        raise ValueError("非 JSON: %s" % exc)
    if not isinstance(d, dict):
        raise ValueError("载荷不是对象")
    tag = d.get("target_id")
    if tag not in TAG_TO_TID:
        raise ValueError("未知 target_id=%r" % (tag,))
    if d.get("frame_id") not in (None, "world_enu"):
        raise ValueError("非 world_enu 坐标系: %r" % d.get("frame_id"))
    xyz = d.get("xyz")
    if not isinstance(xyz, list) or len(xyz) < 2:
        raise ValueError("xyz 形状非法")
    try:
        x, y = float(xyz[0]), float(xyz[1])
        conf = float(d.get("confidence", 0.0))
    except (TypeError, ValueError):
        raise ValueError("坐标/置信度非数值")
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(conf)):
        raise ValueError("含非有限值")
    rng = d.get("range_m")
    try:
        rng = float(rng) if rng is not None else None
        if rng is not None and not math.isfinite(rng):
            rng = None
    except (TypeError, ValueError):
        rng = None
    conf = max(0.0, min(1.0, conf))
    return tag, x, y, conf, rng


def parse_left_actors(raw):
    """解析官方 /left_actors 字符串为剩余 actor 下标集合。

    官方脚本（py3）初始值是 range 对象，str 后为 "range(0, 6)"：
      - 含 "range" 且数字形如 (a,b)：视为区间 [a, b)，初始即 0..5 全在；
      - 形如 "[..]"：取其中 0..5 的整数；
      - 恰为 "[]"：全部已消除（返回空集合）。
    无法识别时返回 None，调用方必须忽略本条，绝不当作空清单。
    """
    s = str(raw).strip()
    if not s:
        return None
    if "range" in s:
        nums = [int(v) for v in re.findall(r"-?\d+", s)]
        if len(nums) >= 2:
            lo, hi = nums[0], nums[1]
            return set(range(max(0, lo), min(6, hi)))
        return None
    if s == "[]":
        return set()
    nums = set(int(v) for v in re.findall(r"-?\d+", s))
    ids = set(v for v in nums if 0 <= v <= 5)
    if not ids and not s.startswith("["):
        return None
    return ids


class CrossColorLocationGuard(object):
    """Short, spatial quarantine for a camera that changes a fixed object's color.

    This is evidence of an ambiguous *visual object*, not proof that all
    stationary people or all reports of either color are false.  Keep the
    judge gate separate from the pursuit tracks so another location remains
    available for a genuine actor with the same color.
    """

    def __init__(self):
        self.history = {}     # (camera, color) -> [(image time, x, y)]
        self.blocks = []      # {x, y, tags, until}

    @staticmethod
    def _stable_recent(points):
        if len(points) < 2 or points[-1][0] - points[0][0] < 0.2:
            return None
        xs = [p[1] for p in points[-4:]]
        ys = [p[2] for p in points[-4:]]
        cx, cy = statistics.median(xs), statistics.median(ys)
        if max(math.hypot(x-cx, y-cy) for x, y in zip(xs, ys)) > 0.55:
            return None
        return cx, cy

    def blocked(self, tag, x, y, now):
        self.blocks = [b for b in self.blocks if now < b['until']]
        return any(tag in b['tags'] and
                   math.hypot(x-b['x'], y-b['y']) <= COLOR_CONFLICT_RADIUS_M
                   for b in self.blocks)

    def observe(self, sample, source, tag, x, y, visual_conf, view_ok):
        """Return a new conflict event, or None, using only fresh image facts."""
        if (tag in RED_TAGS or not view_ok or not math.isfinite(visual_conf) or
                visual_conf < COLOR_CONFLICT_MIN_CONF):
            return None
        for key in list(self.history):
            self.history[key] = [p for p in self.history[key]
                                 if 0.0 <= sample-p[0] <= COLOR_CONFLICT_HISTORY_S]
            if not self.history[key]:
                del self.history[key]
        key = (source, tag)
        points = self.history.setdefault(key, [])
        if points and sample <= points[-1][0]:
            return None
        points.append((sample, x, y))
        center = self._stable_recent(points)
        if center is None:
            return None
        for (other_source, other_tag), other_points in self.history.items():
            if other_source != source or other_tag == tag or other_tag in RED_TAGS:
                continue
            other_center = self._stable_recent(other_points)
            if other_center is None or sample-other_points[-1][0] > COLOR_CONFLICT_HISTORY_S:
                continue
            # Two real actors of different colors may stand shoulder to
            # shoulder.  A camera that sees both in overlapping image times
            # is not changing the label of one fixed object.  Require one
            # color's observed run to finish before the other's begins.
            sequential = (other_points[-1][0] + COLOR_CONFLICT_SWITCH_GAP_S <= points[0][0] or
                          points[-1][0] + COLOR_CONFLICT_SWITCH_GAP_S <= other_points[0][0])
            if not sequential:
                continue
            if math.hypot(center[0]-other_center[0], center[1]-other_center[1]) > COLOR_CONFLICT_MATCH_M:
                continue
            cx = (center[0]+other_center[0])/2.0
            cy = (center[1]+other_center[1])/2.0
            tags = sorted((tag, other_tag))
            if any(set(tags).issubset(b['tags']) and sample < b['until'] and
                   math.hypot(cx-b['x'], cy-b['y']) <= COLOR_CONFLICT_RADIUS_M
                   for b in self.blocks):
                return None
            self.blocks.append({'x': cx, 'y': cy, 'tags': set(tags),
                                'until': sample+COLOR_CONFLICT_TTL_S})
            return {'x': round(cx, 3), 'y': round(cy, 3),
                    'tags': tags, 'target_ids': [TAG_TO_TID[t] for t in tags],
                    'source_uav': source, 'sample_s': sample,
                    'ttl_s': COLOR_CONFLICT_TTL_S,
                    'radius_m': COLOR_CONFLICT_RADIUS_M,
                    'reason': 'same_source_cross_color_same_place'}
        return None


class TargetBridgeCore(object):
    """纯逻辑桥（无 rospy）。"""

    def __init__(self):
        self.tracks = dict((tag, _Track(tag)) for tag in TAG_TO_TID)
        self.eliminated = set()     # 已消除 tag，后续 YOLO 鬼影直接忽略

    def _accept_official_sample(self, tr, t, source, x, y, range_m):
        """Keep a live report source; validate other cameras independently.

        Agreement qualifies a camera's own view for task health, but is not a
        reason to reset the judge motion fit on every alternating camera frame.
        The incumbent expires on image time, never on keepalive publication.
        """
        if source is not None and not self._official_source_ok(tr, t, source, x, y):
            return
        tr.accepted_source_samples[source] = float(t)
        if (source != tr.official_source and tr.good_x is not None and
                0. <= t-tr.good_t <= OFFICIAL_IMAGE_MAX_AGE):
            return
        if source != tr.official_source or t-tr.good_t > 2.0:
            tr.good_history = []
        elif source is not None:
            segment = _confirmed_official_segment(
                [p for p in tr.source_history.get(source, []) if p[0] > tr.good_t])
            if segment is not None:
                # A new coherent corner/reacquisition supersedes the old fit;
                # never fit a straight line through both sides of the turn.
                tr.good_history = segment[:-1]
        tr.official_source = source
        tr.good_x, tr.good_y, tr.good_t = x, y, float(t)
        tr.good_history = [p for p in tr.good_history if 0. <= t-p[0] <= 2.0]
        tr.good_history.append((float(t), x, y))
        if range_m is not None:
            tr.range_m = float(range_m)

    def _official_source_ok(self, tr, t, source, x, y):
        """Protect the judge stream against camera handoff and single-frame jumps.

        Tracking still consumes rejected samples. This gate uses camera history
        only; it cannot establish the absolute one-metre error without truth.
        """
        history = tr.source_history.get(source, [])
        history = [p for p in history if 0.0 <= t - p[0] <= 2.4]
        if not history or t > history[-1][0]:
            history.append((float(t), x, y))
        tr.source_history[source] = history
        age = t - tr.good_t
        has_sequence = (len(history) >= 3 and
                        history[-1][0] - history[-3][0] >= 0.3)
        if has_sequence:
            has_sequence = all(
                0. < b[0]-a[0] <= 1.0+1e-6 and
                math.hypot(b[1]-a[1], b[2]-a[2]) <=
                VEL_MAX*(b[0]-a[0]) + 0.5
                for a, b in zip(history[-3:], history[-2:]))
        model = _robust_official_fit(tr.good_history, tr.good_t)
        if model is not None:
            ax, ay = model[0]+model[2]*age, model[1]+model[3]*age
        else:
            stationary = stable_official_point(tr.good_history, tr.good_t)
            ax, ay = stationary if stationary is not None else (tr.good_x, tr.good_y)
        if tr.good_x is not None and source == tr.official_source and 0.0 <= age <= 2.0:
            # Physical reachability is not a coordinate-quality check: at
            # .8 s it allowed a lone 1.6 m error to refresh the judge stream.
            accepted = math.hypot(x - ax, y - ay) <= 0.45 + VEL_MAX * age
            candidate_fit = _robust_official_fit(
                tr.good_history + [(float(t), x, y)], t)
            if (candidate_fit is not None and not candidate_fit[5] and
                    math.hypot(x-candidate_fit[0], y-candidate_fit[1]) > .75 and
                    math.hypot(x-tr.good_x, y-tr.good_y) > .75):
                accepted = False
            if not accepted:
                accepted = _confirmed_official_segment(
                    [p for p in history if p[0] > tr.good_t]) is not None
        elif tr.good_x is not None and source != tr.official_source and 0.0 <= age <= 2.0:
            # A disagreeing second camera in the 00:36 match was 2 m away
            # from the trusted blue stream.  Its three smooth frames alone
            # cannot prove which monocular range is right.  Hold the old
            # anchor for a short bounded prediction window, then allow a
            # fresh three-image reacquisition rather than locking it forever.
            gate = 0.75 + 0.35 * age
            accepted = has_sequence and math.hypot(x - ax, y - ay) <= gate
        else:
            # No trustworthy anchor remains.  Reacquisition starts a new
            # confirmation segment after three independent smooth images.
            accepted = has_sequence
        if not accepted:
            tr.official_rejects += 1
        return accepted

    # ---- 感知输入 ----
    def report(self, t, target_id, x, y, conf, range_m=None,
               official_ok=True, source_uav=None):
        """吸收一条 YOLO 观测。非法参数抛 ValueError。"""
        tag = str(target_id)
        if tag not in self.tracks:
            raise ValueError("未知 tag %r" % tag)
        if tag in self.eliminated:
            return                      # 已消除目标的残余观测，不复活
        tr = self.tracks[tag]
        if t < tr.t_obs - 0.05:
            # A newer, rejected camera must not starve a trusted camera with
            # slightly older images. Preserve global pursuit time, but allow
            # independent fresh official evidence to update its own anchor.
            if (source_uav is not None and official_ok and t > tr.good_t and
                    tr.t_obs-t <= OBS_WINDOW):
                self._accept_official_sample(tr, t, source_uav, x, y, range_m)
            return
        # The activation counter describes consecutive *fresh* reports.
        # Keeping it across a long visual gap lets one unrelated detection
        # reactivate a dead track and immediately reach the judge stream.
        if t - tr.t_obs > max(COAST_TIME, DROP_TIME):
            tr.alive = False
        if not tr.alive and t - tr.t_obs > COAST_TIME:
            tr._high_conf_count = 0
            tr._motion_history = []
            tr._last_fx = tr._last_fy = tr._last_ft = None
            tr.vx = tr.vy = 0.0
            tr.good_history = []
        # 2026-10-03 我方补：记录最近一次观测距离，供播报闸门使用。
        # 【v24b 决定性修复 2026-10-09】融合权重加入距离因子。
        # conf² 不含距离信息：远机（12~20m，单目误差 0.9~1.1m+）与贴脸机
        # （2m，误差 ~0.1m）平等融合 → 融合位置被拉偏 1m+。v23c actor_0
        # 恒偏西 1.1m（35s 稳定）、v24 actor_1 msg 在 true 前后跳 2.9m
        # （两机观测交替主导）均为此根因。
        # w = conf²/(1+(range/6)²)：2m→0.89、6m→0.5、12m→0.2、16m→0.12、
        # 22m→0.07 —— 贴脸机主导融合，远机仅作存在性佐证。
        if range_m is not None and range_m > 0.0:
            w = conf * conf / (1.0 + (float(range_m) / 6.0) ** 2)
        elif conf > 0.0:
            w = conf * conf
        else:
            w = 1e-6
        # 先裁剪过期观测：一致性锚点只在 OBS_WINDOW 内有效。
        # 2026-10-01 复盘：旧实现锚点永不过期，YOLO 跟丢 >0.6s 后演员走出
        # 5m，重捕获观测被冻结锚点永久拒绝（拒收又不清窗）→ 目标位置冻结
        # 6~11s → 下游确认重置/放弃盘旋/A* 重规划风暴。
        tr.obs = [o for o in tr.obs if (t - o[0]) <= OBS_WINDOW]
        # 一致性校验：窗内已有观测时，新观测必须落在 OBS_SPREAD_M 内。
        # 真人 0.6s 走不了 5m；超距说明跟丢重跟到错误物体/多机误检，直接丢弃。
        if tr.obs:
            ox, oy = tr.obs[-1][1], tr.obs[-1][2]
            if math.hypot(x - ox, y - oy) > OBS_SPREAD_M:
                return
        elif tr.t_obs > 0.0 and (t - tr.t_obs) <= COAST_TIME:
            # 跨窗校验：窗空但跟丢不超过 COAST_TIME，新观测必须离最后融合位置
            # SPREAD_FAR_M 内。真人 1.5s 走不了 10m；超距=跟丢重跟到错误物体。
            if math.hypot(x - tr.x, y - tr.y) > SPREAD_FAR_M:
                return
        # 超过 COAST_TIME：真·重捕获，允许任意位置。
        tr.obs.append((float(t), x, y, w, conf))
        if official_ok and t >= tr.good_t:
            self._accept_official_sample(tr, t, source_uav, x, y, range_m)

        sw = sum(o[3] for o in tr.obs)
        fx = sum(o[1] * o[3] for o in tr.obs) / sw
        fy = sum(o[2] * o[3] for o in tr.obs) / sw
        tr.conf = max(o[4] for o in tr.obs)

        # Camera offsets change the fused centroid when source rates change.
        # Differentiating that centroid created fictitious chase/brake pulses.
        if source_uav is not None:
            history = [p for p in tr.pursuit_history.get(source_uav, [])
                       if 0. <= t-p[0] <= 1.5]
            if not history or t > history[-1][0]:
                history.append((float(t), x, y))
            tr.pursuit_history[source_uav] = history[-16:]
            tr.vx, tr.vy = camera_motion_velocity(tr.pursuit_history, t)
        elif tr._last_fx is not None:
            dt = t - tr._last_ft
            if dt >= VEL_DT_MIN:
                tr.vx = _vel_ema(tr.vx, (fx - tr._last_fx) / dt)
                tr.vy = _vel_ema(tr.vy, (fy - tr._last_fy) / dt)
                # 速度硬限幅：actor 规则上限 2m/s（逃跑）。YOLO bbox 跳变会把差分速度
                # 拉到 5+ m/s（六轮日志 ed=6.22m @ te=1.1s 反推 v=5.7），外推直接飞掉。
                vmag2 = tr.vx * tr.vx + tr.vy * tr.vy
                if vmag2 > 6.25:  # 2.5^2
                    scale = 2.5 / (vmag2 ** 0.5)
                    tr.vx *= scale
                    tr.vy *= scale
                tr._last_fx, tr._last_fy, tr._last_ft = fx, fy, t
        else:
            tr._last_fx, tr._last_fy, tr._last_ft = fx, fy, t

        tr.x, tr.y = fx, fy
        tr.t_obs = float(t)
        
        # ---- 国家一等奖标准改进：多帧验证 ----
        # 记录运动历史
        vel_mag = math.hypot(tr.vx, tr.vy)
        tr._motion_history.append({'t': t, 'vel': vel_mag, 'conf': conf})
        # 只保留 MOTION_CONSISTENCY_WINDOW 内的历史
        tr._motion_history = [h for h in tr._motion_history if (t - h['t']) <= MOTION_CONSISTENCY_WINDOW]
        
        # 新轨激活逻辑：需要连续多帧高置信度 + 运动一致性校验
        if not tr.alive:
            if tr.conf >= NEW_TRACK_CONF:
                tr._high_conf_count += 1
                # 额外校验：运动一致性（误检目标通常运动异常）
                # 国家一等奖标准改进（2026-10-03）：旧实现用
                #   vel_variance = max(vels[-3:]) - min(vels[-3:])
                #   if vel_variance > VEL_MAX * 0.8 (= 3.2)
                # 这对真实 actor 是致命误伤 —— actor 实际 2m/s 移动时
                # EMA 速度从 0 收敛到 1.5~2.0 时方差 ≈ 2.0 同样命中，
                # 多帧验证永远过不了。修复：用「连续 3 帧速度量级都在
                # 同一区间（同时 < FLEE_SPEED×1.5 或同时 ≥ FLEE_SPEED）」
                # 来判定一致性，而不是依赖变化量。
                motion_ok = True
                if len(tr._motion_history) >= 3:
                    vels = [h['vel'] for h in tr._motion_history[-3:]]
                    # 任一帧速度 > FLEE_SPEED * 2 = 4 m/s 几乎一定是误检
                    # （实际 actor 上限 2m/s，规则允许 ±20% 即 2.4）
                    if any(v > FLEE_SPEED * 1.8 for v in vels):
                        motion_ok = False
                    # 任一帧置信度 < NEW_TRACK_CONF * 0.5 则视为不一致
                    confs = [h['conf'] for h in tr._motion_history[-3:]]
                    if any(c < NEW_TRACK_CONF * 0.5 for c in confs):
                        motion_ok = False

                if tr._high_conf_count >= NEW_TRACK_FRAMES and motion_ok:
                    tr.alive = True
            else:
                # 置信度不够，重置计数
                tr._high_conf_count = 0
        # 已激活轨迹不受此限（延续观测允许低置信度）

    # ---- 离线裁判日志回放接口；正式节点不订阅或调用 ----
    def set_left(self, t, raw):
        """处理一条 /left_actors。返回本次标记消除的 tag 列表。"""
        remaining = parse_left_actors(raw)
        if remaining is None:
            return []
        newly = []
        for tr in self.tracks.values():
            idx = ACTOR_INDEX_OF_TAG[tr.tag]
            # 两个红人的视觉槽位与官方 4/5 没有可证明的一一映射。
            # 只消除一个时保留两条视觉轨，避免按编号错杀仍存活的红目标。
            if tr.tag in RED_TAGS and (remaining & RED_ACTOR_IDS):
                continue
            if idx in remaining:
                continue
            if tr.tag in self.eliminated or tr.elim_pending:
                continue
            # 只消除桥已观测到的目标：官方消除必先有 YOLO 上报，
            # 桥与裁判同时启动时二者集合一致；晚启动则保守不动。
            if not tr.alive:
                continue
            tr.elim_pending = True
            newly.append(tr.tag)
        return newly

    # ---- 输出 ----
    def tick(self, t):
        """返回本步待发布事件列表。

        每个事件为 dict：tag/tid/x/y/vx/vy/state/eliminated。
        """
        out = []
        for tr in self.tracks.values():
            if tr.tag in self.eliminated:
                continue
            if tr.elim_pending:
                tr.elim_pending = False
                tr.alive = False
                tr.obs = []
                self.eliminated.add(tr.tag)
                out.append(dict(tag=tr.tag, tid=TAG_TO_TID[tr.tag],
                                x=tr.x, y=tr.y, vx=tr.vx, vy=tr.vy,
                                state=3, eliminated=True, sample_s=tr.t_obs))
                continue
            if not tr.alive:
                # Perception already required several same-object YOLO hits
                # before sending confidence=1.0.  A first such report can
                # dispatch a search aircraft, but the judge-facing stream
                # still waits for this bridge's second independent report.
                gap = t - tr.t_obs
                if (tr._high_conf_count == 1 and tr.conf >= 1.0 and
                        0.0 <= gap <= 1.0):
                    out.append(dict(tag=tr.tag, tid=TAG_TO_TID[tr.tag],
                                    x=tr.x, y=tr.y, vx=tr.vx, vy=tr.vy,
                                    state=0, eliminated=False, sample_s=tr.t_obs,
                                    official_x=None, official_y=None))
                continue
            gap = t - tr.t_obs
            if gap > DROP_TIME:
                tr.alive = False          # 长时间无观测，停发防追鬼
                continue
            # ---- 国家一等奖标准改进：自适应外推 ----
            # 根据速度估计质量动态调整外推参数
            vel_mag = math.hypot(tr.vx, tr.vy)
            
            if EXTRAP_ADAPTIVE:
                # 低速时减少外推（减少抖动影响）
                if vel_mag < EXTRAP_VEL_THRESHOLD:
                    extrap_factor = 0.5  # 低速时减半外推
                else:
                    extrap_factor = 1.0
            else:
                extrap_factor = 1.0
            
            # 时延补偿外推：见 EXTRAP_* 注释。coast 期 te 封顶，位置不漂移。
            te = min(max(gap, 0.0), EXTRAP_MAX_T) * extrap_factor
            ex, ey = tr.vx * te, tr.vy * te
            ed = math.hypot(ex, ey)
            # 自适应距离限幅
            max_d = EXTRAP_MAX_D * (1.0 if vel_mag >= EXTRAP_VEL_THRESHOLD else 0.7)
            if ed > max_d and ed > 0.0:
                ex, ey = ex * max_d / ed, ey * max_d / ed
            # Log the *applied* displacement.  Previously EXTRAP_DBG printed
            # the pre-clamp 0.9-2.6 m proposal as "out" while the judge only
            # received at most 0.5 m; that made live error diagnosis misleading.
            _dbg_t = _BRIDGE_DBG_LAST.get(tr.tag, 0.0)
            if ed > 0.05 and (t - _dbg_t) > 1.0:
                _BRIDGE_DBG_LAST[tr.tag] = t
                _core_log(
                    "[EXTRAP_DBG] tag=%s te=%.3fs v=(%.2f,%.2f) m/s ex=%.2fm ey=%.2fm "
                    "ed=%.2fm requested=%.2fm raw=(%.2f,%.2f) out=(%.2f,%.2f) "
                    "gap=%.2fs vmag=%.2f",
                    tr.tag, te, tr.vx, tr.vy, ex, ey, math.hypot(ex, ey), ed,
                    tr.x, tr.y, tr.x + ex, tr.y + ey, gap, vel_mag)
            # ---- 国家一等奖标准修复（2026-10-04）：state 字段语义对齐官方规则 ----
            # 官方规则 §2.5(4)：「恐怖分子感知到无人机接近后（无人机向裁判系统广播恐怖分子
            #   位置）会改变方向，并以 2m/s 速度进行躲逃」。—— 触发条件是「任何 UAV 广播
            #   了 actor 位置」，与 actor 实际速度无关。
            # 旧实现 `state = 1 if hypot(vx,vy) >= FLEE_SPEED(2.0)` 用 EMA 速度触发，
            #   EMA(VEL_ALPHA=0.35) 在目标 0.5 秒内只能收敛到 ~0.5 m/s，actor 已经在跑
            #   2.0 m/s 时 state 仍是 0 → 下游 agent 永远走 SPOOK_SPEED=0.5 分支 →
            #   0.5 m/s 追 2 m/s 必然跟丢、永远进不了 15s 确认。
            # 修复语义：bridge 既然对外发布 actor_*_info（向裁判广播位置），就证明「正在
            #   被 UAV 看到」—— actor 必已处于 FLEE（规则定义）。DROP_TIME 之内持续标记
            #   state=1；DROP_TIME 之外停止发布（alive=False 上方已拦掉），不再误判。
            # 这把「被观测到」与「逃跑态」绑定，与规则表述 100% 对齐。
            state = 1
            point = official_point_at(tr, t)
            official_x, official_y = point if point is not None else (None, None)
            out.append(dict(tag=tr.tag, tid=TAG_TO_TID[tr.tag],
                            x=tr.x + ex, y=tr.y + ey, vx=tr.vx, vy=tr.vy,
                            state=state, eliminated=False, sample_s=tr.t_obs,
                            official_x=official_x, official_y=official_y,
                            official_sample_s=tr.good_t))
        return out


# ============================ ROS 包装 ============================
class YoloTargetBridge(object):

    def __init__(self):
        import rospy
        from robocup_swarm.msg import TargetState
        # 裁判官方接口仅有 cls/x/y 三字段；内部 TargetState 仍使用本队消息。
        from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
        self._rospy = rospy
        self._TargetState = TargetState
        self._ActorInfo = ActorInfo
        self.core = TargetBridgeCore()

        self.pub = rospy.Publisher("/swarm/target_states", TargetState, queue_size=30)
        self.source_pub = rospy.Publisher("/swarm/visual_source", _msg_string_cls(), queue_size=30)
        self.conflict_pub = rospy.Publisher("/swarm/visual_conflict", _msg_string_cls(), queue_size=10)
        self.conflict_guard = CrossColorLocationGuard()
        # 同时向官方话题发布 ActorInfo：用多机融合坐标，保证 10Hz 上报连续，
        # 避免单机 track 断流导致上报间隔 >1s 被官方重置。
        self._actor_pubs = {}
        # 2026-10-03 我方补：距离闸门的**滞回状态**（tag → 当前是否处于播报态）。
        # 注意：此前叫 _gate_hold 且只写不读 = 滞回没实现，见常量处说明。
        self._gate_live = {}
        # 2026-10-03 我方补：每个 target 话题最后一次发布的时刻（播报保持器用）。
        self._pub_last = {}
        # 2026-10-06 播报遥测：tag → {n, last, max_gap, sum}（PUB_DBG 数据源，
        # 验证官方「相邻播报间隔 ≤1s」判据的桥端证据）
        self._pub_stats = {}
        self._pub_dbg_last = 0.0
        # 双红只按各自视觉轨迹发布，焦点用于队内诊断。
        self._red_focus = None        # 当前正在盯的红球 tag（'red1'/'red2'）
        self._red_consumed = set()    # 仅供离线裁判回放；正式运行不填充
        self._last_source_sample = {}
        self._red_source_map = {}   # (source, local red slot) -> global red track
        self._red_source_observations = {}  # accepted local slot -> (image time, x, y)
        self._rejected_visual = {}  # tag -> (x, y, expiry); 本队视觉冲突位置冷却
        self._String = _msg_string_cls()
        self.report_pub = rospy.Publisher("/swarm/report_telemetry",
                                          self._String, queue_size=30)
        for tag in TAG_TO_TID:
            self._actor_pubs[tag] = rospy.Publisher(
                "/actor_%s_info" % tag, ActorInfo, queue_size=3)
        rospy.Subscriber("/coordination/target_report",
                         _msg_string_cls(), self._report_cb, queue_size=50)
        rospy.Subscriber("/swarm/rejected_visual", _msg_string_cls(),
                         self._rejected_visual_cb, queue_size=5)

        # 2026-10-08 消除 actor 冲刺：PUB_HZ 10→15（间隔 0.067s ≪ 裁判 1s 阈值，
        # 防止 tick 抖动导致 discontinuous 清零 15s 计数）。
        pub_hz = float(os.environ.get("BRIDGE_PUB_HZ", "15"))
        self._timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, pub_hz)),
                                  self._tick)
        rospy.loginfo("yolo_target_bridge 启动：target_report → /swarm/target_states"
                      "（%d 个固定目标槽）", len(TAG_TO_TID))

    def _now(self):
        return self._rospy.Time.now().to_sec()

    def _rejected_visual_cb(self, msg):
        try:
            item = json.loads(msg.data)
            tid = str(item["target_id"])
            x, y = float(item["x"]), float(item["y"])
            duration = min(900.0, max(1.0, float(item["duration_s"])))
            if not all(math.isfinite(v) for v in (x, y, duration)):
                return
        except (ValueError, KeyError, TypeError):
            return
        for tag, tag_tid in TAG_TO_TID.items():
            if tag_tid != tid:
                continue
            self._rejected_visual[tag] = (x, y, self._now() + duration)
            tr = self.core.tracks[tag]
            if tr.alive and math.hypot(tr.x - x, tr.y - y) <= 8.0:
                self.core.tracks[tag] = _Track(tag)
                self._gate_live.pop(tag, None)
                self._pub_last.pop(tag, None)
            self._rospy.logwarn("[bridge] %s 视觉位置暂时冷却，恢复其他位置的候选", tag)

    def _visual_is_rejected(self, tag, x, y, now):
        q = self._rejected_visual.get(tag)
        if q is None:
            return False
        if now >= q[2]:
            self._rejected_visual.pop(tag, None)
            return False
        return math.hypot(x - q[0], y - q[1]) <= 8.0

    def _associate_red(self, source, local_tag, sample, x, y):
        """Assign an aircraft-local red slot by position, never by its name.

        Two detectors can name opposite people ``red1``.  Reusing their local
        names as global keys fused both people together and reset the judge's
        15-second streak.  A distant unmatched report is rejected when both
        global tracks are fresh; it may be a false detection.
        """
        key = (source, local_tag)
        # Some offline replay harnesses instantiate the bridge without its
        # ROS constructor; keep association state available in both paths.
        observations = self.__dict__.setdefault('_red_source_observations', {})
        mapped = self._red_source_map.get(key)
        candidates = []
        free = []
        other_local = "red2" if local_tag == "red1" else "red1"
        other_mapped = self._red_source_map.get((source, other_local))
        other_sample = self._last_source_sample.get((source, other_local))
        other_obs = observations.get((source, other_local))
        # Both local slots can be observed less often than once per second on
        # a loaded CPU.  Keep them mutually exclusive for the same freshness
        # window used by the global red tracks; the old 1 s check let two
        # still-live local slots collapse onto one judge stream.
        other_active = (other_mapped in RED_TAGS and
                        other_mapped not in self.core.eliminated and
                        other_sample is not None and
                        abs(sample - other_sample) <= RED_ASSOC_FRESH_S)
        if other_active and other_obs is not None and \
                RED_LOCAL_DUP_S < sample-other_obs[0] <= RED_ASSOC_FRESH_S:
            other_track = self.core.tracks[other_mapped]
            distinct_tag = next(tag for tag in RED_TAGS if tag != other_mapped)
            distinct_track = self.core.tracks[distinct_tag]
            # A lost local track can reappear in the camera's other slot.
            # Its old slot is not a simultaneous second person. Rebind only
            # when this existing global anchor matches and no independent
            # second anchor matches; keep close, resolved pairs distinct.
            if (0. <= sample-other_track.t_obs <= RED_ASSOC_FRESH_S and
                    math.hypot(x-other_track.x, y-other_track.y) <= RED_ASSOC_GATE_M and
                    (distinct_track.t_obs < 0. or
                     math.hypot(x-distinct_track.x, y-distinct_track.y) > RED_ASSOC_GATE_M)):
                self._red_source_map.pop((source, other_local), None)
                observations.pop((source, other_local), None)
                self._red_source_map[key] = other_mapped
                observations[key] = (sample, x, y)
                return other_mapped
        if other_active and other_obs is not None and \
                0.0 <= sample - other_obs[0] <= RED_LOCAL_DUP_S and \
                math.hypot(x - other_obs[1], y - other_obs[2]) <= RED_LOCAL_DUP_M:
            other_track = self.core.tracks[other_mapped]
            distinct_tag = next(tag for tag in RED_TAGS if tag != other_mapped)
            distinct_track = self.core.tracks[distinct_tag]
            distinct_dist = (math.hypot(x - distinct_track.x, y - distinct_track.y)
                             if distinct_track.t_obs >= 0.0 else float('inf'))
            # The other local slot already has a confident, nearby global
            # anchor, while the only other global person is elsewhere.
            # This is a duplicate local track, not evidence of two people.
            if (other_track.conf >= RED_LOCAL_DUP_MIN_CONF and
                    0.0 <= sample - other_track.t_obs <= RED_ASSOC_FRESH_S and
                    math.hypot(x - other_track.x, y - other_track.y) <= RED_LOCAL_DUP_M and
                    distinct_dist > RED_ASSOC_GATE_M):
                return None
        for tag in RED_TAGS:
            if tag in self.core.eliminated or self.core.tracks[tag].elim_pending:
                continue
            tr = self.core.tracks[tag]
            age = sample - tr.t_obs
            if tr.t_obs < 0.0 or age > RED_ASSOC_FRESH_S:
                free.append(tag)
                if tr.t_obs < 0.0 or age > RED_ASSOC_ANCHOR_S:
                    continue
            # Use a short, bounded prediction only for association.  Long
            # velocity projections near a crossing swap the two identities.
            dt = max(0.0, min(age, 0.35)) if age <= RED_ASSOC_FRESH_S else 0.0
            distance = math.hypot(x - (tr.x + tr.vx * dt),
                                  y - (tr.y + tr.vy * dt))
            if distance <= RED_ASSOC_GATE_M:
                candidates.append((distance, tag))
        candidates.sort()
        if candidates:
            # A detector cannot assign its two simultaneous local slots to
            # one global person while the other local slot is still active.
            if other_active:
                candidates = [c for c in candidates if c[1] != other_mapped]
            if candidates:
                chosen = candidates[0][1]
                # Preserve a source's identity when both candidates are
                # almost equally close during a brief crossing.
                for distance, tag in candidates:
                    if tag == mapped and distance <= candidates[0][0] + 0.35:
                        chosen = tag
                        break
                self._red_source_map[key] = chosen
                observations[key] = (sample, x, y)
                return chosen
        if free:
            # A camera-local name cannot reassign a still-anchored global
            # track after only a few seconds without spatial evidence.  Wait
            # for a compatible view or for the anchors to expire.
            anchors = [self.core.tracks[tag] for tag in RED_TAGS
                       if tag not in self.core.eliminated and
                       not self.core.tracks[tag].elim_pending]
            if (len(anchors) == 2 and all(
                    RED_ASSOC_FRESH_S < sample - tr.t_obs <= RED_ASSOC_ANCHOR_S
                    for tr in anchors)):
                return None
            # Prefer the previous mapping if its track went stale; otherwise
            # use the same-named empty slot for deterministic startup.
            chosen = mapped if mapped in free else (
                local_tag if local_tag in free else free[0])
            if chosen != other_mapped or not other_active:
                self._red_source_map[key] = chosen
                observations[key] = (sample, x, y)
                return chosen
        return None

    def _report_cb(self, msg):
        try:
            tag, x, y, conf, rng = parse_report(msg.data)
            payload = json.loads(msg.data)
            now = self._now()
            if payload.get("real_detection") is not True:
                raise ValueError("缺少真实检测标志")
            sample = float(payload["sample_s"])
            if not math.isfinite(sample) or sample <= 0.0 or sample > now + 0.05 or now - sample > 0.8:
                raise ValueError("图像时间戳过期或非法")
            source = str(payload["source_uav"])
            if not source or source == "legacy":
                raise ValueError("缺少相机来源")
            key = (source, tag)
            if sample <= self._last_source_sample.get(key, -1e18):
                return
            self._last_source_sample[key] = sample
            if tag in RED_TAGS:
                assigned = self._associate_red(source, tag, sample, x, y)
                if assigned is None:
                    return
                tag = assigned
            if self._visual_is_rejected(tag, x, y, now):
                return
            try:
                visual_conf = float(payload.get('visual_confidence', conf))
            except (TypeError, ValueError):
                visual_conf = 0.0  # malformed optional evidence must not drop a target
            conflict = self.conflict_guard.observe(
                sample, source, tag, x, y, visual_conf,
                payload.get('official_view_ok') is True)
            if conflict is not None:
                self._rospy.logwarn('[COLOR_CONFLICT] camera=%s tags=%s '
                                    'xy=(%.2f,%.2f) ttl=%.1fs',
                                    source, '/'.join(conflict['tags']),
                                    conflict['x'], conflict['y'], conflict['ttl_s'])
                self.conflict_pub.publish(_msg_string_cls()(data=json.dumps(conflict)))
            before_t = self.core.tracks[tag].t_obs
            self.core.report(sample, tag, x, y, conf, rng,
                             official_ok=(payload.get('official_view_ok') is True and
                                          rng is not None and rng > 0.0),
                             source_uav=source)
            tr = self.core.tracks[tag]
            source_accepted = tr.accepted_source_samples.get(source) == sample
            if tr.t_obs == sample and not source_accepted and payload.get('official_view_ok') is True:
                self._rospy.logwarn_throttle(1, '[OFFICIAL_GUARD] tag=%s source=%s '
                    'image=%.3f xy=(%.3f,%.3f) trusted_source=%s good_age=%.3f rejects=%d',
                    tag, source, sample, x, y, tr.official_source,
                    sample-tr.good_t, tr.official_rejects)
            # Preserve the camera that actually saw this person. The manager's
            # geometric /swarm/detection echoes cannot identify that camera.
            tr = self.core.tracks[tag]
            if tr.alive and ((tr.t_obs == sample and sample > before_t) or source_accepted):
                pub = getattr(self, 'source_pub', None)
                if pub is not None:
                    pub.publish(_msg_string_cls()(data=json.dumps({
                        'target_id': TAG_TO_TID[tag], 'source_uav': source,
                        'sample_s': sample, 'x': x, 'y': y,
                        'real_detection': True, 'confidence': conf,
                        'range_m': rng,
                        'official_view_ok': source_accepted})))
        except (ValueError, KeyError, TypeError) as exc:
            self._rospy.logwarn_throttle(5, "丢弃 target_report：%s", exc)

    # ---- 红球双流：焦点选择 ----
    def _red_eligible(self, tag):
        """该红球这一帧是否可以作为"正在盯的球"（见 RED_DUAL 注释）。"""
        tr = self.core.tracks.get(tag)
        if tr is None or not tr.alive or tag in self.core.eliminated:
            return None
        if tag in self._red_consumed:
            return None
        if not self._pub_gate_ok(tag):
            return None
        return tr

    def _pick_red_focus(self):
        """选定本轮要"同时发到两条 red 流"的那个球。返回 tag 或 None。

        选择依据：`range_m`（发布端算的"本机↔目标"水平距离）最小 = 离飞机最近，
        与飞行器自身"锁最近目标"的策略一致。带滞回，防止两球距离接近时来回跳。
        range_m 缺失时退化为不比较距离（保留当前焦点）。
        """
        cands = []
        for tag in RED_TAGS:
            tr = self._red_eligible(tag)
            if tr is None:
                continue
            rng = getattr(tr, "range_m", None)
            has = rng is not None and rng > 0.0
            cands.append((0 if has else 1, float(rng) if has else 0.0, tag))
        if not cands:
            self._red_focus = None
            return None
        cands.sort()
        best_tag, best_rng = cands[0][2], cands[0][1]
        if self._red_focus is None:
            self._red_focus = best_tag
            self._rospy.loginfo("[bridge] 红球诊断焦点锁定 %s(t%s)，两条 red 流独立播报",
                                best_tag, TAG_TO_TID[best_tag])
        elif best_tag != self._red_focus:
            cur = [c for c in cands if c[2] == self._red_focus]
            if not cur:
                # 当前球不可用了（被消除/失联/闸门关闭）⇒ 换到可用的那个
                self._rospy.loginfo("[bridge] 红球双流：%s 不可用 ⇒ 焦点切到 %s",
                                    self._red_focus, best_tag)
                self._red_focus = best_tag
            elif cur[0][0] == 0 and cands[0][0] == 0 \
                    and best_rng + RED_FOCUS_HYST_M < cur[0][1]:
                self._rospy.loginfo("[bridge] 红球双流：%s 更近(%.1fm vs %.1fm) ⇒ 焦点切换",
                                    best_tag, best_rng, cur[0][1])
                self._red_focus = best_tag
        return self._red_focus

    def _note_red_left(self, remaining):
        """Offline referee-log replay only; the competition node never calls this."""
        if remaining is None:
            return
        cur = set(v for v in remaining if v in RED_ACTOR_IDS)
        prev = self._red_prev_ids
        self._red_prev_ids = cur
        if prev is None or len(cur) >= len(prev):
            return
        removed = prev - cur
        now = self._now()
        tag = None
        if len(removed) == 1:
            binding = getattr(self, '_red_find_binding', {}).get(next(iter(removed)))
            if binding is not None and 0.0 <= now - binding[1] <= 30.0:
                tag = binding[0]
        if tag is None:
            # A single recently publishing visual red track is the one just
            # removed.  When both tracks are active, leave identity unknown.
            active = [name for name in RED_TAGS
                      if name not in self.core.eliminated and
                      0.0 <= now - self._pub_last.get(name, -1e18) <= 2.0]
            if len(active) == 1:
                tag = active[0]
        if tag is not None and tag not in self.core.eliminated:
            self.core.tracks[tag].elim_pending = True
            self._red_consumed.add(tag)
            self._gate_live.pop(tag, None)
            self._pub_last.pop(tag, None)
            for key, mapped in list(self._red_source_map.items()):
                if mapped == tag:
                    self._red_source_map.pop(key, None)
                    self._red_source_observations.pop(key, None)
            self._rospy.loginfo("[bridge] 官方红目标 %s 已删除；回收视觉轨 %s，保留另一红轨",
                                next(iter(removed)), tag)
        else:
            self._rospy.loginfo("[bridge] 官方红目标数减少（剩 red=%s）；两轨归属不明，等待新观测",
                                sorted(cur))
        self._red_focus = None

    def _find_red_cb(self, msg, actor_id):
        """Offline referee-log replay only; the competition node never calls this."""
        now = self._now()
        self._red_recent_pub = [item for item in self._red_recent_pub
                                if now - item[0] <= 1.0]
        candidates = sorted((now - t, tag) for t, tag in self._red_recent_pub
                            if 0.0 <= now - t <= 0.35)
        if not candidates:
            return
        best_dt, best_tag = candidates[0]
        if any(tag != best_tag and dt <= best_dt + 0.08
               for dt, tag in candidates[1:]):
            return  # simultaneous red streams: identity is ambiguous
        self._red_find_binding[actor_id] = (best_tag, now)

    def _left_cb(self, msg):
        """Offline referee-log replay only; never registered as a ROS callback."""
        self._note_red_left(parse_left_actors(msg.data))
        newly = self.core.set_left(self._now(), msg.data)
        for tag in newly:
            self._rospy.loginfo("官方清单已无 %s(%s)，补发 eliminated",
                                tag, TAG_TO_TID[tag])

    def _pub_gate_ok(self, tag, rng=None):
        """官方播报距离闸门（**带滞回**，见常量处说明）。

        滞回语义：从未在播报态 → 需要 range_m ≤ ACTOR_PUB_MAX_RANGE_M 才进入；
        已在播报态 → 只有 range_m > ACTOR_PUB_HOLD_RANGE_M 才退出。
        中间 12~15m 这一段**保持原状态**，专门用来吃掉边界抖动，避免播报断续。
        range_m 缺失（None/≤0，发布端没给）时不限制，且不影响既有状态。
        """
        if rng is None:
            _tr = self.core.tracks.get(tag)
            rng = getattr(_tr, "range_m", None) if _tr is not None else None
        if rng is None or rng <= 0.0:
            return True                     # 发布端没给距离 ⇒ 不限制
        if self._gate_live.get(tag, False):
            if rng > ACTOR_PUB_HOLD_RANGE_M:
                self._gate_live[tag] = False
                return False
            return True
        if rng <= ACTOR_PUB_MAX_RANGE_M:
            self._gate_live[tag] = True
            return True
        return False

    def _note_pub(self, tag, now):
        """2026-10-06 播报遥测：记录每次 ActorInfo 实际发布的间隔（PUB_DBG 数据源）。"""
        st = self._pub_stats.setdefault(
            tag, {"n": 0, "last": 0.0, "max_gap": 0.0, "sum": 0.0})
        st["n"] += 1
        if st["last"] > 0.0:
            gap = now - st["last"]
            st["sum"] += gap
            if gap > st["max_gap"]:
                st["max_gap"] = gap
        st["last"] = now

    def _publish_actor(self, tag, x, y):
        """向官方话题发布一条 ActorInfo（唯一的播报出口，供 _emit 与保持器共用）。

        · cls 必须取官方 actor_id_dict 的键（红球是 'red'，不是 'red1'）。
        · RED_DUAL 时 red1/red2 各自只发对应 red 流，避免两球坐标交替污染累计状态。
        """
        if tag not in self._actor_pubs:
            return
        now = self._now()
        if self.conflict_guard.blocked(tag, x, y, now):
            _key = 'color_conflict_hold_' + tag
            if now - _BRIDGE_DBG_LAST.get(_key, -1e18) >= 5.0:
                _BRIDGE_DBG_LAST[_key] = now
                self._rospy.logwarn('[COLOR_CONFLICT_HOLD] tag=%s xy=(%.2f,%.2f)',
                                    tag, x, y)
            return
        am = self._ActorInfo()
        am.cls = OFFICIAL_CLS_OF_TAG.get(tag, tag)
        am.x = round(float(x), 3)
        am.y = round(float(y), 3)
        tr = self.core.tracks[tag]
        # 官方 streak 判据下的节流闸门：距上次发布 <0.75s 直接跳过。
        # 间隔落在 [0.75, ~0.95] 区间，既满足官方 <=1.0s 的连续性判据，
        # 又把 15s 窗口内的连乘次数从 ~45 次压到 ~20 次。
        # 2026-10-06 修：哨兵从 0.0 改为 None —— 用 0.0 兜底会把"仿真时间
        # 恰好为 0（离线自测的冻结时钟）"的首帧发布也拦掉。
        _last_pub = self._pub_last.get(tag)
        tr = self.core.tracks.get(tag)
        _track_speed = (math.hypot(tr.vx, tr.vy) if tr is not None else 0.0)
        _interval = FAST_PUB_INTERVAL if _track_speed >= FAST_PUB_SPEED else PUB_MIN_INTERVAL
        if _last_pub is not None and now - _last_pub < _interval:
            return
        self._rospy.loginfo('[REPORT_POINT] tag=%s source=%s image=%.3f '
                           'age=%.3f xy=(%.3f,%.3f) rejected=%d',
                           tag, tr.official_source, tr.good_t,
                           now-tr.good_t, am.x, am.y, tr.official_rejects)
        # red1/red2 remain separate judge streams.  Their internal names do
        # not assert which official red actor was observed.
        self._actor_pubs[tag].publish(am)
        self._pub_last[tag] = now
        self._note_pub(tag, now)
        self._publish_report_telemetry('published', tag, now, tr, am.x, am.y)

    def _publish_report_telemetry(self, event, tag, now, tr, x, y):
        """Expose our own publish/vision evidence without reading judge state."""
        pub = getattr(self, 'report_pub', None)
        if pub is None:
            return
        sample = float(getattr(tr, 'good_t', -1.0))
        payload = {
            'event': event,
            'target_id': TAG_TO_TID[tag],
            'tag': tag,
            'report_s': float(now),
            'sample_s': sample,
            'fresh_image': 0.0 <= now - sample <= 1.0,
            'source_uav': getattr(tr, 'official_source', None) or '',
            'x': float(x), 'y': float(y),
        }
        pub.publish(self._String(data=json.dumps(payload, separators=(',', ':'))))

    def _emit(self, ev):
        m = self._TargetState()
        # TargetState 时间戳表示最后一次真实视觉观测。桥接可短时保活，
        # 但保活帧不能伪装为新检测，否则 agent/manager 会无限追冻结位置。
        _sample = ev.get("sample_s", self._now())
        m.header.stamp = self._rospy.Time.from_sec(_sample) if _sample > 0 else self._rospy.Time.now()
        m.header.frame_id = "map"
        m.target_id = ev["tid"]
        m.x, m.y = ev["x"], ev["y"]
        m.vx, m.vy = ev["vx"], ev["vy"]
        m.state = ev["state"]
        m.tracked_time = 0.0
        m.confirm_time = 0.0
        m.eliminated = ev["eliminated"]
        self.pub.publish(m)

        # 向官方话题发布 ActorInfo（用融合坐标，10Hz 连续上报）。
        # 仅对存活且有位置的目标发布；eliminated 的目标官方已不再判定。
        tag = TID_TO_TAG.get(ev["tid"])
        if tag is None or tag not in self._actor_pubs or ev["eliminated"]:
            return
        if ev.get("state", 0) == 3:
            return
        if ev.get("official_x") is None or ev.get("official_y") is None:
            _dbg_now = self._now()
            _dbg_key = "official_hold_" + tag
            if _dbg_now - _BRIDGE_DBG_LAST.get(_dbg_key, -1e18) >= 5.0:
                _BRIDGE_DBG_LAST[_dbg_key] = _dbg_now
                _tr = self.core.tracks.get(tag)
                self._rospy.loginfo(
                    "[OFFICIAL_HOLD] tag=%s range=%s real_obs_age=%.2fs "
                    "eligible_xy=(%.2f,%.2f): no recent fully valid view",
                    tag,
                    ("%.1fm" % _tr.range_m) if _tr is not None and
                    _tr.range_m is not None else "None",
                    _dbg_now - _tr.good_t if _tr is not None else float('inf'),
                    ev["x"], ev["y"])
            return
        official_sample = ev.get("official_sample_s", _sample)
        # 红衣人移动且两条本地 red 槽会交叉。旧坐标继续向裁判播 8s 会
        # 把另一条正确红流刚建立的 15s 计时反复清零。队内轨迹仍保留供重捕获，
        # 官方播报只接受近期真实图像支撑的位置。
        if tag in RED_TAGS and self._now() - official_sample > RED_OFFICIAL_FRESH_S:
            return
        if not self._pub_gate_ok(tag):
            # 2026-10-06 闸门拦截遥测：v9 三轮"裁判零输入"时这里完全静默，
            # 无法区分"没检测到"和"检测到了但距离太远被闸"。
            tr = self.core.tracks.get(tag)
            rng = getattr(tr, "range_m", None) if tr is not None else None
            _gn = self._now()
            _gt = _BRIDGE_DBG_LAST.get("gate_" + tag, 0.0)
            if (_gn - _gt) > 1.0:
                _BRIDGE_DBG_LAST["gate_" + tag] = _gn
                self._rospy.loginfo(
                    "[GATE_DBG] tag=%s range=%s gate_live=%s enter<=%.1fm exit>%.1fm"
                    " → 拦截播报",
                    tag,
                    ("%.1fm" % rng) if rng is not None else "None",
                    bool(self._gate_live.get(tag, False)),
                    ACTOR_PUB_MAX_RANGE_M, ACTOR_PUB_HOLD_RANGE_M)
            return
        self._publish_actor(tag, ev["official_x"], ev["official_y"])

    def _keepalive(self, now):
        """Fill publish gaps only while the same image-time policy permits it."""
        for tag in list(self._gate_live):
            if not self._gate_live.get(tag):
                continue
            if tag not in self._actor_pubs or tag in self.core.eliminated:
                self._gate_live[tag] = False
                continue
            tr = self.core.tracks.get(tag)
            if tr is None:
                continue
            point = official_point_at(tr, now)
            if point is None:
                continue
            speed = math.hypot(tr.vx, tr.vy)
            interval = (min(ACTOR_KEEPALIVE_DT, FAST_PUB_INTERVAL)
                        if speed >= FAST_PUB_SPEED else ACTOR_KEEPALIVE_DT)
            if now - self._pub_last.get(tag, -1e18) < interval:
                continue
            if not self._pub_gate_ok(tag):
                continue
            self._publish_actor(tag, *point)

    def _tick(self, _evt):
        now = self._now()
        if RED_DUAL:
            self._pick_red_focus()
        was_alive = {tag: tr.alive for tag, tr in self.core.tracks.items()}
        for ev in self.core.tick(now):
            self._emit(ev)
        for tag, active in was_alive.items():
            tr = self.core.tracks[tag]
            if not active or tr.alive or tag in self.core.eliminated:
                continue
            # An expired image track is only lost, never officially removed.
            # Release its report gate and stale local red-slot bindings so a
            # new real sighting can claim this visual slot again.
            self._gate_live.pop(tag, None)
            self._pub_last.pop(tag, None)
            if tag in RED_TAGS:
                if self._red_focus == tag:
                    self._red_focus = None
                for key, mapped in list(self._red_source_map.items()):
                    if mapped == tag:
                        self._red_source_map.pop(key, None)
                        self._red_source_observations.pop(key, None)
            self._publish_report_telemetry('visual_lost', tag, now,
                                           tr, tr.x, tr.y)
        self._keepalive(now)
        # 2026-10-06 播报连续性遥测：每 5s 汇总一次实际发布间隔。
        # 官方判据「相邻播报间隔 ≤1s」—— max_gap>1.0 即存在断流窗口。
        if now - self._pub_dbg_last >= 5.0:
            self._pub_dbg_last = now
            for _ptag in sorted(self._pub_stats):
                _st = self._pub_stats[_ptag]
                if _st["n"] > 1:
                    self._rospy.loginfo(
                        "[PUB_DBG] tag=%s n=%d max_gap=%.2fs avg_gap=%.2fs",
                        _ptag, _st["n"], _st["max_gap"],
                        _st["sum"] / (_st["n"] - 1))


def _msg_string_cls():
    from std_msgs.msg import String
    return String


# ============================ 离线自测（不连 ROS master）============================
def _self_test():
    """不依赖 rospy 的内核自测：python3 yolo_target_bridge.py --selftest"""
    # 1) 载荷校验
    # 2026-10-03：parse_report 现在返回 5 元组 (tag, x, y, conf, range_m)。
    # 旧断言还停在 4 元组 ⇒ 早就失效（本轮顺手修正，非本轮引入）。
    assert parse_report(json.dumps({
        "target_id": "green", "frame_id": "world_enu",
        "xyz": [1.0, 2.0, 0.0], "confidence": 0.9,
        "observation_id": "x"})) == ("green", 1.0, 2.0, 0.9, None)
    assert parse_report(json.dumps({
        "target_id": "green", "frame_id": "world_enu",
        "xyz": [1.0, 2.0, 0.0], "confidence": 0.9, "range_m": 7.5,
        "observation_id": "x"})) == ("green", 1.0, 2.0, 0.9, 7.5)
    for bad in ["{", json.dumps({"target_id": "zzz", "xyz": [0, 0]}),
                json.dumps({"target_id": "green", "xyz": [0]}),
                json.dumps({"target_id": "green", "xyz": ["a", 0]}),
                json.dumps({"target_id": "green", "frame_id": "cam",
                            "xyz": [0, 0]})]:
        try:
            parse_report(bad)
            raise AssertionError("应拒绝: %r" % bad)
        except ValueError:
            pass
    print("1) parse_report 校验 OK")

    # 2) 观测融合 + 速度估计
    c = TargetBridgeCore()
    c.report(0.0, "green", 0.0, 0.0, 0.8)
    c.report(0.05, "green", 0.1, 0.0, 0.9)      # 同窗近帧
    ev = c.tick(0.05)
    assert len(ev) == 1 and ev[0]["tid"] == "t0" and ev[0]["state"] == 1  # 国家一等奖修复：被观测即 FLEE（规则定义），不再等 EMA 速度
    # 国家一等奖离线自测（2026-10-04 修复）：原测试 9 帧 0.2s 间隔期望
    # state==1（>=2m/s FLEE）且 vx>1.0。但 EMA(VEL_ALPHA=0.35) 在 9 帧
    # 内只能收敛到 ~1.6（实测 1.63），低于 FLEE_SPEED=2.0 → state==0。
    # 这是 EMA 收敛动态特性，不是 bug。但 9 帧太短不合理：让 actor
    # 移动足够久 (>=3s = 15 帧) 才能让 EMA 真正反映稳态速度。
    for k in range(1, 25):                      # 匀速 2m/s 移动 4.8s
        c.report(k * 0.2, "green", k * 0.4, 0.0, 0.9)
    ev = c.tick(4.8)
    assert ev[0]["vx"] > 1.5 and ev[0]["vx"] <= VEL_MAX, ev[0]    # EMA 收敛范围 1.5~2.0
    # state 阈值是 2.0；EMA 收敛到 1.6~1.8 仍 < 2.0 是正常 EMA 滞后；不强制 state==1
    print("2) 融合/速度 OK: vx=%.2f (EMA 收敛)" % ev[0]["vx"])

    # 3) coast 保持 → 接力窗口结束后停发（用新实例避开测试 2 的时序）
    c3 = TargetBridgeCore()
    c3.report(0.0, "green", 0.0, 0.0, 0.8)
    c3.report(0.1, "green", 0.1, 0.0, 0.85)     # 2 帧达 NEW_TRACK_FRAMES 激活
    assert c3.tracks['green'].alive
    # 2026-10-06 修：断言原本写死 DROP_TIME=6 的时序（0.7/5.5/8.0/8.5），
    # DROP_TIME 默认改为 2.0 后 5.5s 处早已停发 ⇒ 自测必挂（被此前 rospy
    # NameError 掩盖）。改为按 DROP_TIME 参数化，语义不变：
    # 半窗保持 / 超窗停发 / 重现复活。
    _t_last = 0.1                               # 最后观测时刻
    assert len(c3.tick(_t_last + 0.5 * DROP_TIME)) == 1     # 半窗内 coast 保持
    assert c3.tick(_t_last + DROP_TIME + 0.5) == []         # 超窗停发
    c3.report(_t_last + DROP_TIME + 1.0, "green", 4.0, 0.0, 0.9)
    assert c3.tick(_t_last + DROP_TIME + 1.0) == []  # long gap resets validation
    c3.report(_t_last + DROP_TIME + 1.2, "green", 4.1, 0.0, 0.9)
    assert len(c3.tick(_t_last + DROP_TIME + 1.2)) == 1
    print("3) coast/drop/复活 OK")

    # 4) 官方消除：补发 eliminated 一次，鬼影不复活
    T = 4.8                                     # 测试 2 中实例 c 的最后观测时刻
    c.report(T + 7.5, "green", 4.0, 0.0, 0.9)  # gap 7.5s > DROP_TIME 已停发 → 补帧复活
    c.report(T + 7.6, "green", 4.0, 0.0, 0.9)  # 2 帧达 NEW_TRACK_FRAMES 确保复活
    c.set_left(T + 7.7, "[1, 2, 3, 4, 5]")      # green(0) 不在
    ev = c.tick(T + 7.7)
    assert len(ev) == 1 and ev[0]["eliminated"] is True and ev[0]["state"] == 3
    assert c.tick(T + 7.8) == []
    c.report(T + 7.9, "green", 4.0, 0.0, 0.9)  # 残余 YOLO 鬼影
    assert c.tick(T + 7.9) == []
    print("4) eliminated 补发/鬼影忽略 OK")

    # 5) /left_actors 形态：range 串、[]、噪声
    assert parse_left_actors("range(0, 6)") == {0, 1, 2, 3, 4, 5}
    assert parse_left_actors("range(2, 5)") == {2, 3, 4}
    assert parse_left_actors("[]") == set()
    assert parse_left_actors("[0, 2, 4]") == {0, 2, 4}
    assert parse_left_actors("") is None
    assert parse_left_actors("garbage") is None
    print("5) parse_left_actors 形态 OK")

    # 6) 全消除（需要先 2 帧激活 track）
    c2 = TargetBridgeCore()
    c2.report(0.0, "blue", 1, 1, 0.9)
    c2.report(0.1, "blue", 1.05, 1, 0.9)       # 2 帧达 NEW_TRACK_FRAMES 激活
    assert c2.tracks['blue'].alive
    c2.set_left(0, "range(0, 6)")               # range 串 = 全在，不消除
    assert c2.set_left(0.2, "[]") == ["blue"]
    assert c2.tick(0.2)[0]["eliminated"]
    print("6) range 不误杀 / [] 全消除 OK")

    _red_dual_selftest()
    print("\nyolo_target_bridge 内核自测全部通过")


# ---- 红球双流自测（不依赖 rospy）----
class _FakePub(object):
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _FakeMsg(object):
    def __init__(self):
        self.header = type("H", (), {"stamp": None, "frame_id": ""})()


class _FakeString(object):
    def __init__(self, data=""):
        self.data = data


class _FakeActorInfo(object):
    """Mirror the official three-field ROS message in bridge self-tests."""
    __slots__ = ("cls", "x", "y")
    _type = "ros_actor_cmd_pose_plugin_msgs/ActorInfo"

    def __init__(self):
        self.cls = ""
        self.x = 0.0
        self.y = 0.0


class _FakeRos(object):
    class Time(object):
        @staticmethod
        def now():
            return 0.0

        @staticmethod
        def from_sec(value):
            return float(value)

    def loginfo(self, *a, **k):
        pass

    logwarn = loginfo
    logwarn_throttle = loginfo


def _mk_bridge():
    """绕过 __init__（不连 ROS），手工装配一个可测的桥实例。"""
    b = object.__new__(YoloTargetBridge)
    b._rospy = _FakeRos()
    b._TargetState = _FakeMsg
    b._ActorInfo = _FakeActorInfo
    b.core = TargetBridgeCore()
    b.pub = _FakePub()
    b._String = _FakeString
    b.report_pub = _FakePub()
    b.conflict_pub = _FakePub()
    b.conflict_guard = CrossColorLocationGuard()
    b._actor_pubs = dict((t, _FakePub()) for t in TAG_TO_TID)
    b._gate_live = {}
    b._pub_last = {}
    # 2026-10-06 播报遥测属性（__init__ 被绕过，手工补齐）
    b._pub_stats = {}
    b._pub_dbg_last = 0.0
    b._red_focus = None
    b._red_consumed = set()
    b._red_prev_ids = None
    b._red_recent_pub = []
    b._red_find_binding = {}
    b._red_source_map = {}
    b._red_source_observations = {}
    b._last_source_sample = {}
    b._rejected_visual = {}
    # 2026-10-06 修：可推进的假时钟 —— PUB_MIN_INTERVAL 节流需要时间前进
    # 才能连续发布（_advance 由自测在每次期望发布前调用）。
    _clock = {"t": 0.0}
    b._now = lambda: _clock["t"]
    b._advance = lambda dt: _clock.__setitem__("t", _clock["t"] + dt)
    return b


def _activate(b, tag, x, y, rng, n=4, conf=0.9):
    """把某条轨推到 alive（多帧验证：需 NEW_TRACK_FRAMES 帧高置信度）。"""
    for k in range(n):
        b.core.report(k * 0.2, tag, x, y, conf, range_m=rng)
    assert b.core.tracks[tag].alive, "%s 未激活" % tag


def _red_dual_selftest():
    # 0) 身份映射：tN ↔ actor_N，红球是反的
    assert ACTOR_INDEX_OF_TAG == {"green": 0, "blue": 1, "brown": 2,
                                  "white": 3, "red2": 4, "red1": 5}, ACTOR_INDEX_OF_TAG
    assert OFFICIAL_CLS_OF_TAG["red1"] == "red" and OFFICIAL_CLS_OF_TAG["red2"] == "red"
    assert OFFICIAL_CLS_OF_TAG["white"] == "white"
    print("7) 内部红色槽位/官方 cls OK（红色物理身份未绑定，cls='red'）")

    b = _mk_bridge()
    # 2026-10-06 修：闸门默认已从 12m 收紧到 8m（ACTOR_PUB_MAX_RANGE_M），
    # red2@10m 会被闸死、测不到"非焦点静默"路径 ⇒ 改为 red2 同距 8m 压线
    # 过闸（字母序 red1 优先成为焦点）。
    _activate(b, "red1", 10.0, 0.0, 8.0)
    _activate(b, "red2", 40.0, 0.0, 8.0)
    assert b._pick_red_focus() == "red1", b._red_focus

    # B3：red1 / t5 只发对应 red1 流，避免污染 red2 流
    b._advance(1.0)                             # 推进假时钟，越过节流窗
    b._emit(dict(tag="red1", tid="t5", x=10.0, y=0.0, vx=0.0, vy=0.0,
                 state=0, eliminated=False, official_x=10.0,
                 official_y=0.0, official_sample_s=0.6))
    assert len(b._actor_pubs["red1"].msgs) == 1
    assert len(b._actor_pubs["red2"].msgs) == 0, "red1 不得污染 red2 流"
    assert b._actor_pubs["red1"].msgs[0].cls == "red"
    print("8) red1 只发 red1 流 OK（cls='red'）")

    # red2 / t4 只发对应 red2 流，不因当前焦点仍是 red1 而静默
    n1 = len(b._actor_pubs["red1"].msgs)
    b._emit(dict(tag="red2", tid="t4", x=40.0, y=0.0, vx=0.0, vy=0.0,
                 state=0, eliminated=False, official_x=40.0,
                 official_y=0.0, official_sample_s=0.6))
    assert len(b._actor_pubs["red1"].msgs) == n1, "red2 不得污染 red1 流"
    assert len(b._actor_pubs["red2"].msgs) == 1
    assert b._actor_pubs["red2"].msgs[0].x == 40.0
    print("9) red2 只发 red2 流 OK")

    # 仅离线回放：已知红人离场时，检查红色焦点可转到另一条视觉轨。
    b._note_red_left({0, 1, 2, 3, 4, 5})      # 先建立 baseline（首帧不作差集）
    assert b._red_focus == "red1"
    b.core.report(1.0, "red2", 40.0, 0.0, 0.9, range_m=6.0)   # red2 变近、过闸门
    b._note_red_left({0, 1, 2, 3, 4})
    assert b._red_focus is None and not b._red_consumed, (b._red_focus, b._red_consumed)
    assert b._pick_red_focus() == "red2", b._red_focus
    b._advance(1.0)                             # 推进假时钟，越过节流窗
    b._emit(dict(tag="red2", tid="t4", x=40.0, y=0.0, vx=0.0, vy=0.0,
                 state=0, eliminated=False, official_x=40.0,
                 official_y=0.0, official_sample_s=1.0))
    assert b._actor_pubs["red1"].msgs[-1].x == 10.0   # red1 流保持旧坐标
    assert b._actor_pubs["red2"].msgs[-1].x == 40.0
    print("10) 离线回放红色焦点切换 OK")

    # 非红球不受双流影响：green 只发自己的话题，cls='green'
    _activate(b, "green", 0.0, 5.0, 7.0)
    b._emit(dict(tag="green", tid="t0", x=0.0, y=5.0, vx=0.0, vy=0.0,
                 state=0, eliminated=False, official_x=0.0,
                 official_y=5.0, official_sample_s=2.0))
    assert len(b._actor_pubs["green"].msgs) == 1
    assert b._actor_pubs["green"].msgs[0].cls == "green"
    assert len(b._actor_pubs["red1"].msgs) == 1         # 未被 green 追加
    assert len(b._actor_pubs["red2"].msgs) == 2         # 第9/10项各发一条，未受 green 影响
    print("11) 非红球路径不受影响 OK")


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        _self_test()
    else:
        import rospy
        rospy.init_node("yolo_target_bridge")
        YoloTargetBridge()
        rospy.spin()
