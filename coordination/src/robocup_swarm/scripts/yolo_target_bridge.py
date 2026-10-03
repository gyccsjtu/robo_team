#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO→swarm 桥：把合法感知链路替换真值订阅。

输入（己方 YOLO 输出，合法）：
  /coordination/target_report  std_msgs/String，6 个 perception_real 实例共写，
      裸 JSON：{"target_id","frame_id":"world_enu","xyz":[x,y,z],
                "confidence","observation_id"}
      target_id ∈ green / blue / brown / white / red1 / red2
  /left_actors  std_msgs/String，官方裁判发布的剩余 actor 清单（权威消除信号）。

输出：
  /swarm/target_states  robocup_swarm/TargetState
      下游 swarm_manager / swarm_agent 只认本消息，不感知数据来源，
      因此本桥替换 target_sim / official_target_bridge 后二者零算法改动。

身份映射与官方裁判一致（score_cal.py:16）：
  green→t0(actor_0) blue→t1 brown→t2 white→t3 red1→t4 red2→t5

合规说明：本节点不订阅 /gazebo/model_states、/gazebo/link_states，
不读 obstacle.txt / black_box.txt，不启动 target_sim。

设计：纯逻辑内核 TargetBridgeCore 不依赖 rospy（只依赖 math/json），
ROS 包装只做话题收发，内核可离线单测。
"""

import json
import math
import os
import re

# ---- 身份映射（对齐 ~/XTDrone/robocup/score_cal.py 的 actor_id_dict）----
# 官方约定（score_cal.py:22 + 275-276 行话题绑定）：
#   /actor_green_info->actor_0  blue->1  brown->2  white->3
#   /actor_red2_info ->actor_4  /actor_red1_info->actor_5   ← red 是反的！
# 团队约定（swarm_viz.py:17、detection_to_official.py:78-83）：tN ↔ actor_N。
# ⚠ 2026-10-03 修正：原表把 red1→t4 / red2→t5，与团队约定相反，
#   连带 ACTOR_INDEX_OF_TAG 反了 ⇒ set_left() 会把"红球消除"记到错的那条流上，
#   导致桥向飞行器补发错误的 eliminated（活着的球被摘掉）。
TAG_TO_TID = {
    "green": "t0",
    "blue": "t1",
    "brown": "t2",
    "white": "t3",
    "red1": "t5",   # red1 = actor_5
    "red2": "t4",   # red2 = actor_4
}
TID_TO_TAG = dict((v, k) for k, v in TAG_TO_TID.items())
ACTOR_INDEX_OF_TAG = dict((tag, int(tid[1:])) for tag, tid in TAG_TO_TID.items())

# ---- ActorInfo.cls 必须是官方 actor_id_dict 的**键** ----
# score_cal.py:22  actor_id_dict = {'green':[0],'blue':[1],'brown':[2],'white':[3],'red':[4,5]}
# score_cal.py:175-179
#   /actor_red1_info -> actor_info1_callback：cls=='red' 才处理，只判 actor_5
#   /actor_red2_info -> actor_info2_callback：cls=='red' 才处理，只判 actor_4
# 🔴 曾经写成 cls='red1'/'red2' ⇒ 两个 red 回调都返回空列表 ⇒ **红球永远 0 分**。
OFFICIAL_CLS_OF_TAG = {
    "green": "green", "blue": "blue", "brown": "brown", "white": "white",
    "red1": "red", "red2": "red",
}

# ---- 红球双流（2026-10-03，按用户指示）----
# 用户原话：官方说红色报回来的数据同时和两个真值（actor_4/actor_5）比，
# 有一个对得上就有分。官方源码正是如此：两条 red 流各自独立做
# 「误差<1m 连续 15s」，不匹配只 _reset_detection（清计时，**不扣分、不清已得 find 分**）。
# 而两个红衣人**外观完全相同**，槽位绑定（slot0→红1、slot1→红2）只能靠空间连续性猜，
# 猜错就 50% 概率两条流一起拿不到分。
# ⇒ 改为：**锁定一个红球，把同一坐标同时发到 /actor_red1_info 与 /actor_red2_info**。
#   哪条流对上了由官方自己判断（我们不需要知道这个球是 actor_4 还是 actor_5）；
#   该球被消除后 /left_actors 会少一个红号，据此切到另一个球 ⇒ 两个红球都能确定性拿分。
RED_DUAL = os.environ.get("BRIDGE_RED_DUAL", "1") == "1"
RED_TAGS = ("red1", "red2")
RED_ACTOR_IDS = frozenset(ACTOR_INDEX_OF_TAG[t] for t in RED_TAGS)   # {4, 5}
# 焦点滞回：新球必须比当前球近这么多米才换向，防止两球距离接近时来回跳。
RED_FOCUS_HYST_M = float(os.environ.get("BRIDGE_RED_FOCUS_HYST", "3.0"))

# ---- 融合 / 计时参数 ----
OBS_WINDOW = 0.6        # 多感知节点近帧融合窗 s（2Hz/节点 × 最多 6 节点）
OBS_SPREAD_M = 5.0      # 窗内新观测与已有观测的最大允许距离 m（防跟丢跳变/多机误检拉飞）
# 跨窗校验：跟丢 >OBS_WINDOW 但 <=COAST_TIME 时，新观测必须离最后融合位置
# SPREAD_FAR_M 内。真人 1.5s 走不了 10m；超距说明跟丢重跟到错误物体。
# 超过 COAST_TIME 才允许任意位置（真·重捕获）。
SPREAD_FAR_M = float(os.environ.get("BRIDGE_SPREAD_FAR", "10.0"))
VEL_DT_MIN = 0.10       # 最小差分间隔，避免高频小 dt 放大噪声
VEL_ALPHA = 0.35        # 速度 EMA 增益
VEL_MAX = 4.0           # 速度限幅，防 YOLO 跳变拉飞（与 cooperative_tracker 一致）
COAST_TIME = 1.5        # 短暂遮挡：保持最后位置 s
# === 2026-10-03 我方新增：官方播报距离闸门（带滞回）+ 播报保持器 ===
# 官方判据：误差<1m、间隔≤1s、连续 15s（score_cal.py:135-165），15s 攒满即
# delete_model 消除；而首次对上会让裁判 publish /find_actor_N，actor 控制器据此
# 在 **25s 后瞬移**（control_actor.py:112-127）⇒ 净余量只有 10s，断一次就归零。
# 单目测距误差随距离放大，远处播报会把计时清零。默认 12m —— 实测 7m 内误差
# 0.26~0.69m（达标），15.98m 时系统报 17.09m（误差 1.1m，刚好越界）。
ACTOR_PUB_MAX_RANGE_M = float(os.environ.get("BRIDGE_PUB_MAX_RANGE", "12.0"))
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
ACTOR_KEEPALIVE_DT = float(os.environ.get("BRIDGE_KEEPALIVE_DT", "0.35"))
ACTOR_KEEPALIVE_MAX_T = float(os.environ.get("BRIDGE_KEEPALIVE_MAX_T", "0.9"))
# 协同层需要给备用机完成接力的窗口。agent 自己还有 TARGET_STALE 门槛，
# 因此这里不能在 3s 时立刻撤掉目标状态，否则短遮挡会直接清空盘旋任务。
DROP_TIME = float(os.environ.get("BRIDGE_DROP_TIME", "6.0"))
# 新轨激活门槛：未激活轨只有融合窗内最高置信度 >= 该值才允许 alive。
# 2026-10-01 复盘：3 条 0.43~0.61 的 red1 误检（真身为 green 演员）建出鬼影轨
# t4，全队盘旋假目标并广播假消除。已激活轨不受此限（延续观测允许低置信度）。
NEW_TRACK_CONF = float(os.environ.get("BRIDGE_NEW_TRACK_CONF", "0.7"))

# ---- 国家一等奖标准改进：多帧验证 + 自适应参数 ----
# 多帧验证：新轨迹需要连续 N 帧高置信度观测才能激活
NEW_TRACK_FRAMES = int(os.environ.get("BRIDGE_NEW_TRACK_FRAMES", "3"))
# 颜色相似度校验：red1/red2 误检常因为颜色与 green/blue 相似
COLOR_SIMILARITY_THRESHOLD = float(os.environ.get("BRIDGE_COLOR_SIMILARITY", "0.3"))
# 运动一致性校验：误检目标通常运动模式异常
MOTION_CONSISTENCY_WINDOW = float(os.environ.get("BRIDGE_MOTION_WINDOW", "2.0"))

FLEE_SPEED = 2.0        # state 粗估：超过即给 FLEE（下游不依赖该字段），规则要求 2m/s
# ---- 上报时延补偿（误差压制）----
# 裁判判定要求上报坐标与 actor 真值误差 <1m 连续 15s。感知链路固有滞后：
# 图像采集+YOLO 推理+传输 (~0.35s) + 感知端内部 EMA + 桥接 0.6s 融合窗加权
# ≈ 0.65s。演员 1.3m/s 跑动时滞后 ≈0.85m，叠加投影噪声即超 1m → 裁判计数
# 反复归零（03_judge.log 大量重复 find actor_N）。上报前按估计速度外推：
#   te = min(gap, EXTRAP_MAX_T) + EXTRAP_DELAY
# 距离限幅 EXTRAP_MAX_D 防速度估计被 YOLO 跳变污染时外推飞掉。
EXTRAP_DELAY  = float(os.environ.get("BRIDGE_EXTRAP_DELAY", "0.35"))
EXTRAP_MAX_T  = float(os.environ.get("BRIDGE_EXTRAP_MAX_T", "1.2"))
EXTRAP_MAX_D  = float(os.environ.get("BRIDGE_EXTRAP_MAX_D", "1.5"))

# ---- 国家一等奖标准改进：自适应外推 ----
# 自适应外推：根据速度估计质量动态调整外推参数
EXTRAP_VEL_THRESHOLD = float(os.environ.get("BRIDGE_EXTRAP_VEL_THRESHOLD", "0.5"))  # 低速阈值
EXTRAP_ADAPTIVE = os.environ.get("BRIDGE_EXTRAP_ADAPTIVE", "1") == "1"


def _vel_ema(v_old, raw):
    """单轴速度 EMA + 限幅。"""
    v = VEL_ALPHA * raw + (1.0 - VEL_ALPHA) * v_old
    return max(-VEL_MAX, min(VEL_MAX, v))


class _Track(object):
    __slots__ = ("tag", "obs", "x", "y", "vx", "vy", "conf",
                 "t_obs", "alive", "elim_pending", "range_m",
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
        self.t_obs = -1e18
        self.alive = False         # 是否正在对外发布
        self.elim_pending = False  # 待补发 eliminated=true
        self._last_fx = None
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


class TargetBridgeCore(object):
    """纯逻辑桥（无 rospy）。"""

    def __init__(self):
        self.tracks = dict((tag, _Track(tag)) for tag in TAG_TO_TID)
        self.eliminated = set()     # 已消除 tag，后续 YOLO 鬼影直接忽略

    # ---- 感知输入 ----
    def report(self, t, target_id, x, y, conf, range_m=None):
        """吸收一条 YOLO 观测。非法参数抛 ValueError。"""
        tag = str(target_id)
        if tag not in self.tracks:
            raise ValueError("未知 tag %r" % tag)
        if tag in self.eliminated:
            return                      # 已消除目标的残余观测，不复活
        tr = self.tracks[tag]
        # 2026-10-03 我方补：记录最近一次观测距离，供播报闸门使用。
        if range_m is not None:
            tr.range_m = float(range_m)
        w = conf * conf if conf > 0.0 else 1e-6
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

        sw = sum(o[3] for o in tr.obs)
        fx = sum(o[1] * o[3] for o in tr.obs) / sw
        fy = sum(o[2] * o[3] for o in tr.obs) / sw
        tr.conf = max(o[4] for o in tr.obs)

        # 速度差分 + EMA（用融合位置，保证平滑）
        if tr._last_fx is not None:
            dt = t - tr._last_ft
            if dt >= VEL_DT_MIN:
                tr.vx = _vel_ema(tr.vx, (fx - tr._last_fx) / dt)
                tr.vy = _vel_ema(tr.vy, (fy - tr._last_fy) / dt)
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
                motion_ok = True
                if len(tr._motion_history) >= 3:
                    # 检查速度变化是否合理
                    vels = [h['vel'] for h in tr._motion_history[-3:]]
                    vel_variance = max(vels) - min(vels)
                    # 速度变化过大可能是误检
                    if vel_variance > VEL_MAX * 0.8:
                        motion_ok = False
                
                if tr._high_conf_count >= NEW_TRACK_FRAMES and motion_ok:
                    tr.alive = True
            else:
                # 置信度不够，重置计数
                tr._high_conf_count = 0
        # 已激活轨迹不受此限（延续观测允许低置信度）

    # ---- 裁判输入 ----
    def set_left(self, t, raw):
        """处理一条 /left_actors。返回本次标记消除的 tag 列表。"""
        remaining = parse_left_actors(raw)
        if remaining is None:
            return []
        newly = []
        for tr in self.tracks.values():
            idx = ACTOR_INDEX_OF_TAG[tr.tag]
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
                                state=3, eliminated=True))
                continue
            if not tr.alive:
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
            te = (min(gap, EXTRAP_MAX_T) + EXTRAP_DELAY) * extrap_factor
            ex, ey = tr.vx * te, tr.vy * te
            ed = math.hypot(ex, ey)
            # 自适应距离限幅
            max_d = EXTRAP_MAX_D * (1.0 if vel_mag >= EXTRAP_VEL_THRESHOLD else 0.7)
            if ed > max_d and ed > 0.0:
                ex, ey = ex * max_d / ed, ey * max_d / ed
            state = 1 if math.hypot(tr.vx, tr.vy) >= FLEE_SPEED else 0
            out.append(dict(tag=tr.tag, tid=TAG_TO_TID[tr.tag],
                            x=tr.x + ex, y=tr.y + ey, vx=tr.vx, vy=tr.vy,
                            state=state, eliminated=False))
        return out


# ============================ ROS 包装 ============================
class YoloTargetBridge(object):

    def __init__(self):
        import rospy
        from robocup_swarm.msg import TargetState
        from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
        self._rospy = rospy
        self._TargetState = TargetState
        self._ActorInfo = ActorInfo
        self.core = TargetBridgeCore()

        self.pub = rospy.Publisher("/swarm/target_states", TargetState, queue_size=30)
        # 同时向官方话题发布 ActorInfo：用多机融合坐标，保证 10Hz 上报连续，
        # 避免单机 track 断流导致上报间隔 >1s 被官方重置。
        self._actor_pubs = {}
        # 2026-10-03 我方补：距离闸门的**滞回状态**（tag → 当前是否处于播报态）。
        # 注意：此前叫 _gate_hold 且只写不读 = 滞回没实现，见常量处说明。
        self._gate_live = {}
        # 2026-10-03 我方补：每个 target 话题最后一次发布的时刻（播报保持器用）。
        self._pub_last = {}
        # 2026-10-03 我方补：红球双流焦点（见 RED_DUAL 注释）
        self._red_focus = None        # 当前正在盯的红球 tag（'red1'/'red2'）
        self._red_consumed = set()    # 已被官方消除的红球 tag，不再选为焦点
        self._red_prev_ids = None     # 上一帧 /left_actors 里的红球 actor 号集合
        for tag in TAG_TO_TID:
            self._actor_pubs[tag] = rospy.Publisher(
                "/actor_%s_info" % tag, ActorInfo, queue_size=3)
        rospy.Subscriber("/coordination/target_report",
                         _msg_string_cls(), self._report_cb, queue_size=50)
        rospy.Subscriber("/left_actors", _msg_string_cls(),
                         self._left_cb, queue_size=5)

        pub_hz = float(os.environ.get("BRIDGE_PUB_HZ", "10"))
        self._timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, pub_hz)),
                                  self._tick)
        rospy.loginfo("yolo_target_bridge 启动：target_report → /swarm/target_states"
                      "（%d 个固定目标槽）", len(TAG_TO_TID))

    def _now(self):
        return self._rospy.Time.now().to_sec()

    def _report_cb(self, msg):
        try:
            tag, x, y, conf, rng = parse_report(msg.data)
            self.core.report(self._now(), tag, x, y, conf, rng)
        except ValueError as exc:
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
            self._rospy.loginfo("[bridge] 红球双流：焦点锁定 %s(t%s)，两条 red 流同发",
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
        """官方 /left_actors 少了一个红球号 ⇒ 我们正盯的那个球已被消除 ⇒ 切到另一个球。

        不需要知道被删的是 actor_4 还是 actor_5：双流同发时，被删的那个必是
        "我们正在盯的球"（另一条流一直不匹配，不会触发消除）。
        """
        if remaining is None:
            return
        cur = set(v for v in remaining if v in RED_ACTOR_IDS)
        prev = self._red_prev_ids
        self._red_prev_ids = cur
        if prev is None or len(cur) >= len(prev):
            return
        if self._red_focus is not None:
            self._red_consumed.add(self._red_focus)
            self._rospy.loginfo("[bridge] 官方已消除一个红衣人（剩 red=%s）⇒ 红球焦点 %s 作废",
                                sorted(cur), self._red_focus)
            self._red_focus = None

    def _left_cb(self, msg):
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

    def _publish_actor(self, tag, x, y):
        """向官方话题发布一条 ActorInfo（唯一的播报出口，供 _emit 与保持器共用）。

        · cls 必须取官方 actor_id_dict 的键（红球是 'red'，不是 'red1'）。
        · RED_DUAL 时只有"当前焦点红球"会真正发出，且同时发到两条 red 流。
        """
        if tag not in self._actor_pubs:
            return
        am = self._ActorInfo()
        am.cls = OFFICIAL_CLS_OF_TAG.get(tag, tag)
        am.x = round(x, 3)
        am.y = round(y, 3)
        now = self._now()
        if RED_DUAL and tag in RED_TAGS:
            # 双流同发：只发"当前焦点"那个球，避免两个球的坐标交替刷同一条流
            # （交替 ⇒ 每条流都被对方的坐标不断 _reset_detection，永远累不满 15s）。
            if tag != self._red_focus:
                return
            for t in RED_TAGS:
                self._actor_pubs[t].publish(am)
                self._pub_last[t] = now
            return
        self._actor_pubs[tag].publish(am)
        self._pub_last[tag] = now

    def _emit(self, ev):
        m = self._TargetState()
        m.header.stamp = self._rospy.Time.now()
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
        if (tag is None or tag not in self._actor_pubs
                or ev["eliminated"] or ev.get("state", 0) == 3
                or not self._pub_gate_ok(tag)):
            return
        self._publish_actor(tag, ev["x"], ev["y"])

    def _keepalive(self, now):
        """播报保持器（2026-10-03 新增）。

        官方那条 "相邻播报间隔 ≤1.0s" 是硬判据（score_cal.py:137），断一次就
        _reset_detection 清零；而清零后重新对上又会重发一次 /find_actor_N，
        等于把 actor 的 25s 瞬移倒计时再往未来叠一份 —— 代价是双重的。
        融合层偶发空 tick（coast 中 / 多帧验证未达 NEW_TRACK_FRAMES / 窗内无新观测）
        会造成 >1s 静默，这里按最低频率补齐，把静默压在 1s 以内。
        位置用「最后融合位置 + 速度 × dt」外推；外推超过 KEEPALIVE_MAX_T 就放弃
        （宁可断流，也不报一个必然 >1m 的假坐标 —— 误差超 1m 照样清零）。
        """
        for tag in list(self._gate_live):
            if not self._gate_live.get(tag):
                continue
            if tag not in self._actor_pubs or tag in self.core.eliminated:
                self._gate_live[tag] = False
                continue
            if RED_DUAL and tag in RED_TAGS and tag != self._red_focus:
                continue                    # 非焦点红球：本来就不发，保持器也别空转
            tr = self.core.tracks.get(tag)
            if tr is None or not tr.alive:
                continue
            last = self._pub_last.get(tag, 0.0)
            dt = now - last
            if dt < ACTOR_KEEPALIVE_DT:
                continue
            if dt > ACTOR_KEEPALIVE_MAX_T:
                continue                    # 太久没观测：不硬撑，等真实观测回来
            x = tr.x + (getattr(tr, "vx", 0.0) or 0.0) * dt
            y = tr.y + (getattr(tr, "vy", 0.0) or 0.0) * dt
            self._publish_actor(tag, x, y)

    def _tick(self, _evt):
        now = self._now()
        if RED_DUAL:
            self._pick_red_focus()
        for ev in self.core.tick(now):
            self._emit(ev)
        self._keepalive(now)


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
    # 2/3/4 段 2026-10-03 重写：多帧验证（NEW_TRACK_FRAMES=3）与 range_m 加入后
    # 旧断言的时间线已失效（旧版只喂 2 帧、且 tick 时刻与最后观测时刻脱节）。
    c.report(0.00, "green", 0.00, 0.0, 0.9)
    c.report(0.05, "green", 0.02, 0.0, 0.9)     # 同窗近帧
    c.report(0.10, "green", 0.04, 0.0, 0.9)     # 第 3 帧高置信 ⇒ 激活
    ev = c.tick(0.10)
    assert len(ev) == 1 and ev[0]["tid"] == "t0" and ev[0]["state"] == 0
    T = 0.10
    for k in range(1, 20):                      # 匀速 2m/s 移动
        T = 0.10 + k * 0.2
        c.report(T, "green", 0.04 + k * 0.4, 0.0, 0.9)
    ev = c.tick(T)
    assert ev and ev[0]["state"] == 1 and ev[0]["vx"] > 1.5, ev
    print("2) 融合/速度 OK: vx=%.2f" % ev[0]["vx"])

    # 3) coast 保持 → DROP_TIME(6s) 后停发 → 再观测需重新满足多帧验证
    assert len(c.tick(T + 0.7)) == 1            # gap 0.7s 仍保持
    assert len(c.tick(T + 5.0)) == 1            # 6s 窗口内仍保持
    assert c.tick(T + 7.0) == []                # gap>6s 停发
    for k in range(3):                          # 重新出现 → 需再满 3 帧才复活
        c.report(T + 7.5 + k * 0.05, "green", 4.0, 0.0, 0.9)
    assert len(c.tick(T + 7.6)) == 1
    print("3) coast/drop/复活 OK")

    # 4) 官方消除：补发 eliminated 一次，鬼影不复活
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

    # 6) 全消除
    c2 = TargetBridgeCore()
    for k in range(3):                          # 需先满 3 帧高置信才激活
        c2.report(k * 0.05, "blue", 1.0, 1.0, 0.9)
    assert c2.tracks["blue"].alive
    c2.set_left(0.2, "range(0, 6)")             # range 串 = 全在，不消除
    assert c2.set_left(0.3, "[]") == ["blue"]
    assert c2.tick(0.3)[0]["eliminated"]
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


class _FakeRos(object):
    class Time(object):
        @staticmethod
        def now():
            return 0.0

    def loginfo(self, *a, **k):
        pass

    logwarn_throttle = loginfo


def _mk_bridge():
    """绕过 __init__（不连 ROS），手工装配一个可测的桥实例。"""
    b = object.__new__(YoloTargetBridge)
    b._rospy = _FakeRos()
    b._TargetState = _FakeMsg
    b._ActorInfo = _FakeMsg
    b.core = TargetBridgeCore()
    b.pub = _FakePub()
    b._actor_pubs = dict((t, _FakePub()) for t in TAG_TO_TID)
    b._gate_live = {}
    b._pub_last = {}
    b._red_focus = None
    b._red_consumed = set()
    b._red_prev_ids = None
    b._now = lambda: 0.0
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
    print("7) 身份映射/官方 cls OK（red1→actor_5，cls 统一为 'red'）")

    b = _mk_bridge()
    # red1 近(8m)、red2 稍远(10m，仍过 12m 闸门但非最近) ⇒ 焦点 = red1
    _activate(b, "red1", 10.0, 0.0, 8.0)
    _activate(b, "red2", 40.0, 0.0, 10.0)
    assert b._pick_red_focus() == "red1", b._red_focus

    # 焦点球（red1 / t5）⇒ 两条 red 流同发，且 cls 都是 'red'
    b._emit(dict(tag="red1", tid="t5", x=10.0, y=0.0, vx=0.0, vy=0.0,
                 state=0, eliminated=False))
    assert len(b._actor_pubs["red1"].msgs) == 1
    assert len(b._actor_pubs["red2"].msgs) == 1, "焦点球必须双流同发"
    assert b._actor_pubs["red1"].msgs[0].cls == "red"
    assert b._actor_pubs["red2"].msgs[0].cls == "red"
    assert b._actor_pubs["red2"].msgs[0].x == b._actor_pubs["red1"].msgs[0].x
    print("8) 焦点红球双流同发 OK（cls='red'，坐标一致）")

    # 非焦点球（red2 / t4）⇒ 一条都不发（避免两球坐标交替刷同一流）
    n1, n2 = len(b._actor_pubs["red1"].msgs), len(b._actor_pubs["red2"].msgs)
    b._emit(dict(tag="red2", tid="t4", x=40.0, y=0.0, vx=0.0, vy=0.0,
                 state=0, eliminated=False))
    assert len(b._actor_pubs["red1"].msgs) == n1
    assert len(b._actor_pubs["red2"].msgs) == n2, "非焦点红球不得发布"
    print("9) 非焦点红球静默 OK")

    # 官方删掉一个红衣人（actor_5）⇒ red1 作废，焦点切到可用的 red2
    b._note_red_left({0, 1, 2, 3, 4, 5})      # 先建立 baseline（首帧不作差集）
    assert b._red_focus == "red1"
    b.core.report(1.0, "red2", 40.0, 0.0, 0.9, range_m=6.0)   # red2 变近、过闸门
    b._note_red_left({0, 1, 2, 3, 4})
    assert b._red_focus is None and b._red_consumed == {"red1"}, (b._red_focus, b._red_consumed)
    assert b._pick_red_focus() == "red2", b._red_focus
    b._emit(dict(tag="red2", tid="t4", x=40.0, y=0.0, vx=0.0, vy=0.0,
                 state=0, eliminated=False))
    assert b._actor_pubs["red1"].msgs[-1].x == 40.0
    assert b._actor_pubs["red2"].msgs[-1].x == 40.0
    print("10) 消除后红球焦点自动切换 OK（两个红球可依次拿分）")

    # 非红球不受双流影响：green 只发自己的话题，cls='green'
    _activate(b, "green", 0.0, 5.0, 7.0)
    b._emit(dict(tag="green", tid="t0", x=0.0, y=5.0, vx=0.0, vy=0.0,
                 state=0, eliminated=False))
    assert len(b._actor_pubs["green"].msgs) == 1
    assert b._actor_pubs["green"].msgs[0].cls == "green"
    assert len(b._actor_pubs["red1"].msgs) == 1 + 1     # 未被 green 追加
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
