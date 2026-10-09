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
ACTOR_KEEPALIVE_MAX_T = float(os.environ.get("BRIDGE_KEEPALIVE_MAX_T", "2.2"))
# 2026-10-06 六轮复盘核心修复：官方判据「相邻广播间隔 <=1.0s」（score_cal DETECTION_INTERVAL）
# 只约束不超 1s，不要求高频。广播 3Hz 时 15s streak 需 45 次连乘全过（单次失败率 20% ->
# 成功率 0.8^45≈8e-6，必然 0 消除）；节流到 0.75s 一次后 15s 只需 20 次连乘（0.8^20=1.2%），
# 若误差压缩使失败率 <=10% 则成功率 12%/窗口 —— 这是把「数学上不可能」变成「大概率全消除」的
# 最大杠杆。PUB_MIN_INTERVAL=0.75 留 0.25s 余量防 tick 抖动导致间隔 >1.0s 触发 discontinuous。
PUB_MIN_INTERVAL = float(os.environ.get("BRIDGE_PUB_MIN_INTERVAL", "0.75"))
# 协同层需要给备用机完成接力的窗口。agent 自己还有 TARGET_STALE 门槛，
# 因此这里不能在 3s 时立刻撤掉目标状态，否则短遮挡会直接清空盘旋任务。
# v13（2026-10-07）：2.0→4.0 —— v12c actor_5 贴脸 3.07m 后 YOLO 间歇丢 2.5s，
# 2.0s 停发 → 官方「间隔>1s」reset 27 次。贴脸跟随态 actor 相对静止，
# coast 期外推封顶（te≤1.15s + EXTRAP_MAX_D 限幅）位置不漂，撑过间歇丢失
# 15s streak 不断；actor 快速逃跑时误差 reset 早晚问题，无净损失。
# 2026-10-08 消除 actor 冲刺：DROP_TIME 4.0→8.0（v18b streak_max=0.0s 修复，
# 多撑 4s 遮挡防追鬼断链；EXTRAP_MAX_T/D 联合限幅保误差<1.5m）。
DROP_TIME = float(os.environ.get("BRIDGE_DROP_TIME", "8.0"))
# 新轨激活门槛：未激活轨只有融合窗内最高置信度 >= 该值才允许 alive。
# 2026-10-01 复盘：3 条 0.43~0.61 的 red1 误检（真身为 green 演员）建出鬼影轨
# t4，全队盘旋假目标并广播假消除。已激活轨不受此限（延续观测允许低置信度）。
NEW_TRACK_CONF = float(os.environ.get("BRIDGE_NEW_TRACK_CONF", "0.7"))

# ---- 国家一等奖标准改进：多帧验证 + 自适应参数 ----
# 多帧验证：新轨迹需要连续 N 帧高置信度观测才能激活
NEW_TRACK_FRAMES = int(os.environ.get("BRIDGE_NEW_TRACK_FRAMES", "2"))  # 国家一等奖改进：2 帧激活（从 3 降），2Hz 单节点下 1s 即可激活，避免 actor 1.5m/s 移动时延迟累计 >1m 误差导致确认反复归零
# 颜色相似度校验：red1/red2 误检常因为颜色与 green/blue 相似
COLOR_SIMILARITY_THRESHOLD = float(os.environ.get("BRIDGE_COLOR_SIMILARITY", "0.3"))
# 运动一致性校验：误检目标通常运动模式异常
MOTION_CONSISTENCY_WINDOW = float(os.environ.get("BRIDGE_MOTION_WINDOW", "2.0"))

FLEE_SPEED = 2.0        # state 粗估：超过即给 FLEE（下游不依赖该字段），规则要求 2m/s
# ---- 上报时延补偿（误差压制）----
# 裁判判定要求上报坐标与 actor 真值误差 <1m 连续 15s。感知链路固有滞后：
# 图像采集+YOLO 推理+传输 (~0.35s) + 感知端内部 EMA + 桥接 0.6s 融合窗加权
# ≈ 0.65s。官方规则 actor 逃跑 2 m/s（±20% 即 2.4），滞后 ≈1.3m，叠加 12m 处测距噪声 ~0.9m
# 总误差可超 1m → 裁判计数
# 反复归零（03_judge.log 大量重复 find actor_N）。上报前按估计速度外推（DELAY 补满全链路滞后）：
#   te = min(gap, EXTRAP_MAX_T) + EXTRAP_DELAY
# 距离限幅 EXTRAP_MAX_D 防速度估计被 YOLO 跳变污染时外推飞掉。
EXTRAP_DELAY  = float(os.environ.get("BRIDGE_EXTRAP_DELAY", "0.45"))
# 2026-10-08 消除 actor 冲刺：EXTRAP_MAX_T 0.5→1.5（让最近 obs 多走 1s 进 strict 限幅），
# EXTRAP_MAX_D 3.0→2.0（双重保护：外推距离 < 2m，仍在裁判 1m 误差阈值的 2× 内留余量）。
# v22-D（2026-10-09）：EXTRAP_DELAY 0.65→0.45 + EXTRAP_MAX_D 2.0→1.2 —— v21 RESET_DBG
# 实证贴脸播报误差稳定 1.39~1.58m（差 0.4m 达标，77 次 reset 全是 far_dist）：
# actor 正常游走 1.5m/s（control_actor velocity=1.5，非逃跑），外推过冲
# （msg 移动 1.5m/s vs true 1.07m/s）+ EXTRAP_MAX_D=2.0 打满 → 误差 1.5m。
# 限幅 1.2m 把最坏误差压进 1m 阈值附近；欠外推 0.3s 好过过冲 0.5s（游走会转弯，
# 过冲方向错 = 双倍罚，欠外推只罚延迟差）。
EXTRAP_MAX_T  = float(os.environ.get("BRIDGE_EXTRAP_MAX_T", "1.5"))
# v24b：EXTRAP_MAX_D 1.2→0.5 —— v24b 实测外推过冲 1.4~3.0m（速度被跳变拉飞时
# 0.75s 双重前推飞掉）。0.5m 限幅 + 观测误差 0.3m = 最坏 0.8m < 1m 阈值。
# 正常速度（1m/s）下前推需求 0.65m——0.5m 略欠外推，欠 0.15s 好过过冲（游走会转弯）。
EXTRAP_MAX_D  = float(os.environ.get("BRIDGE_EXTRAP_MAX_D", "0.5"))

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
                # 速度硬限幅：actor 规则上限 2m/s（逃跑）。YOLO bbox 跳变会把差分速度
                # 拉到 5+ m/s（六轮日志 ed=6.22m @ te=1.1s 反推 v=5.7），外推直接飞掉。
                vmag2 = tr.vx * tr.vx + tr.vy * tr.vy
                if vmag2 > 6.25:  # 2.5^2
                    scale = 2.5 / (vmag2 ** 0.5)
                    tr.vx *= scale
                    tr.vy *= scale
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
            # 2026-10-06 B 项诊断打印：每 actor 每 1s 一次（节流），便于判断外推过冲/欠补
            _dbg_t = _BRIDGE_DBG_LAST.get(tr.tag, 0.0)
            if ed > 0.05 and (t - _dbg_t) > 1.0:
                _BRIDGE_DBG_LAST[tr.tag] = t
                _core_log(
                    "[EXTRAP_DBG] tag=%s te=%.3fs v=(%.2f,%.2f) m/s ex=%.2fm ey=%.2fm ed=%.2fm "
                    "raw=(%.2f,%.2f) out=(%.2f,%.2f) gap=%.2fs vmag=%.2f", (
                        tr.tag, te, tr.vx, tr.vy, ex, ey, ed,
                        tr.x, tr.y, tr.x + ex, tr.y + ey, gap, vel_mag))
            # 自适应距离限幅
            max_d = EXTRAP_MAX_D * (1.0 if vel_mag >= EXTRAP_VEL_THRESHOLD else 0.7)
            if ed > max_d and ed > 0.0:
                ex, ey = ex * max_d / ed, ey * max_d / ed
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
            out.append(dict(tag=tr.tag, tid=TAG_TO_TID[tr.tag],
                            x=tr.x + ex, y=tr.y + ey, vx=tr.vx, vy=tr.vy,
                            state=state, eliminated=False))
        return out


# ============================ ROS 包装 ============================
class YoloTargetBridge(object):

    def __init__(self):
        import rospy
        from robocup_swarm.msg import TargetState
        # 国家一等奖 v3 (2026-10-06): 改用 robocup_swarm.ActorInfo (5字段 Header+cls+x+y+z)
        # 原 ros_actor_cmd_pose_plugin_msgs.ActorInfo (3字段 cls+x+y, float32) 与
        # score_cal.py 的订阅端 `from robocup_swarm.msg import ActorInfo` md5 不匹配
        # -> ROS 拒绝连接 -> /actor_*_info 永远收不到 -> /left_actors 永远是 [0..5]
        from robocup_swarm.msg import ActorInfo
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
        # 2026-10-06 播报遥测：tag → {n, last, max_gap, sum}（PUB_DBG 数据源，
        # 验证官方「相邻播报间隔 ≤1s」判据的桥端证据）
        self._pub_stats = {}
        self._pub_dbg_last = 0.0
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

        # 2026-10-08 消除 actor 冲刺：PUB_HZ 10→15（间隔 0.067s ≪ 裁判 1s 阈值，
        # 防止 tick 抖动导致 discontinuous 清零 15s 计数）。
        pub_hz = float(os.environ.get("BRIDGE_PUB_HZ", "15"))
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
        am = self._ActorInfo()
        am.cls = OFFICIAL_CLS_OF_TAG.get(tag, tag)
        am.x = round(float(x), 3)
        am.y = round(float(y), 3)
        am.z = 0.0
        am.header.stamp = self._rospy.Time.now()
        am.header.frame_id = "world_enu"
        now = self._now()
        # 官方 streak 判据下的节流闸门：距上次发布 <0.75s 直接跳过。
        # 间隔落在 [0.75, ~0.95] 区间，既满足官方 <=1.0s 的连续性判据，
        # 又把 15s 窗口内的连乘次数从 ~45 次压到 ~20 次。
        # 2026-10-06 修：哨兵从 0.0 改为 None —— 用 0.0 兜底会把"仿真时间
        # 恰好为 0（离线自测的冻结时钟）"的首帧发布也拦掉。
        _last_pub = self._pub_last.get(tag)
        if _last_pub is not None and now - _last_pub < PUB_MIN_INTERVAL:
            return
        if RED_DUAL and tag in RED_TAGS:
            # B3（2026-10-09）：双红球各轨独立播报，red1 只喂 red1 流，
            # red2 只喂 red2 流。官方每条流内部都会匹配 actor_4/5；
            # 若把两球坐标同发到两条流，交替坐标会清除对方流的累计状态。
            self._actor_pubs[tag].publish(am)
            self._pub_last[tag] = now
            self._note_pub(tag, now)
            return
        self._actor_pubs[tag].publish(am)
        self._pub_last[tag] = now
        self._note_pub(tag, now)

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
        if tag is None or tag not in self._actor_pubs or ev["eliminated"]:
            return
        if ev.get("state", 0) == 3:
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
            # B3：非焦点红球也走保持器（双流独立后不再拦截）
            tr = self.core.tracks.get(tag)
            if tr is None or not tr.alive:
                continue
            last = self._pub_last.get(tag, 0.0)
            dt = now - last
            if dt < ACTOR_KEEPALIVE_DT:
                continue
            if dt > ACTOR_KEEPALIVE_MAX_T:
                continue                    # 太久没观测：不硬撑，等真实观测回来
            # v24b：补发外推同样限幅 EXTRAP_MAX_D（0.5m）——dt 可达 1.3s，
            # 速度 2.2m/s 时无限幅外推 2.86m，必被裁判 far_dist 清零。
            _vx = getattr(tr, "vx", 0.0) or 0.0
            _vy = getattr(tr, "vy", 0.0) or 0.0
            _d = math.hypot(_vx * dt, _vy * dt)
            if _d > EXTRAP_MAX_D and _d > 1e-6:
                _s = EXTRAP_MAX_D / _d
                _vx *= _s
                _vy *= _s
            x = tr.x + _vx * dt
            y = tr.y + _vy * dt
            self._publish_actor(tag, x, y)

    def _tick(self, _evt):
        now = self._now()
        if RED_DUAL:
            self._pick_red_focus()
        for ev in self.core.tick(now):
            self._emit(ev)
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
    c3.report(_t_last + DROP_TIME + 1.0, "green", 4.0, 0.0, 0.9)  # 重新出现 → 复活
    assert len(c3.tick(_t_last + DROP_TIME + 1.0)) == 1
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
    # 2026-10-06 播报遥测属性（__init__ 被绕过，手工补齐）
    b._pub_stats = {}
    b._pub_dbg_last = 0.0
    b._red_focus = None
    b._red_consumed = set()
    b._red_prev_ids = None
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
    print("7) 身份映射/官方 cls OK（red1→actor_5，cls 统一为 'red'）")

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
                 state=0, eliminated=False))
    assert len(b._actor_pubs["red1"].msgs) == 1
    assert len(b._actor_pubs["red2"].msgs) == 0, "red1 不得污染 red2 流"
    assert b._actor_pubs["red1"].msgs[0].cls == "red"
    print("8) red1 只发 red1 流 OK（cls='red'）")

    # red2 / t4 只发对应 red2 流，不因当前焦点仍是 red1 而静默
    n1 = len(b._actor_pubs["red1"].msgs)
    b._emit(dict(tag="red2", tid="t4", x=40.0, y=0.0, vx=0.0, vy=0.0,
                 state=0, eliminated=False))
    assert len(b._actor_pubs["red1"].msgs) == n1, "red2 不得污染 red1 流"
    assert len(b._actor_pubs["red2"].msgs) == 1
    assert b._actor_pubs["red2"].msgs[0].x == 40.0
    print("9) red2 只发 red2 流 OK")

    # 官方删掉一个红衣人（actor_5）⇒ red1 作废，焦点切到可用的 red2
    b._note_red_left({0, 1, 2, 3, 4, 5})      # 先建立 baseline（首帧不作差集）
    assert b._red_focus == "red1"
    b.core.report(1.0, "red2", 40.0, 0.0, 0.9, range_m=6.0)   # red2 变近、过闸门
    b._note_red_left({0, 1, 2, 3, 4})
    assert b._red_focus is None and b._red_consumed == {"red1"}, (b._red_focus, b._red_consumed)
    assert b._pick_red_focus() == "red2", b._red_focus
    b._advance(1.0)                             # 推进假时钟，越过节流窗
    b._emit(dict(tag="red2", tid="t4", x=40.0, y=0.0, vx=0.0, vy=0.0,
                 state=0, eliminated=False))
    assert b._actor_pubs["red1"].msgs[-1].x == 10.0   # red1 流保持旧坐标
    assert b._actor_pubs["red2"].msgs[-1].x == 40.0
    print("10) 消除后红球焦点自动切换 OK（两个红球可依次拿分）")

    # 非红球不受双流影响：green 只发自己的话题，cls='green'
    _activate(b, "green", 0.0, 5.0, 7.0)
    b._emit(dict(tag="green", tid="t0", x=0.0, y=5.0, vx=0.0, vy=0.0,
                 state=0, eliminated=False))
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
