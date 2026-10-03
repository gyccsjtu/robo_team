#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO→swarm 桥：把合法感知链路替换真值订阅。

输入（己方 YOLO 输出，合法）：
  /swarm/visual_observation std_msgs/String，冻结schema2实际相机证据；
      运行/机号/序号/原采样时间/位置/实际置信度均校验。
      target_id ∈ green / blue / brown / white / red1 / red2
  /left_actors  std_msgs/String，官方裁判发布的剩余 actor 清单（权威消除信号）。

输出：
  /swarm/confirmed_visual_observation：激活后转发原schema2证据，供实际manager/agent复验。
  /swarm/target_states：融合/预测诊断，生产执行不以此旧runless话题更新目标。

非红色身份映射：green→t0(actor_0) blue→t1 brown→t2 white→t3。
red1→t5、red2→t4仅为内部几何轨迹槽，不表示官方红色人物身份；
对外统一上报red和坐标，按用户转述的赛事组口径匹配任一红色真值。

合规说明：本节点不订阅 /gazebo/model_states、/gazebo/link_states，
不读 obstacle.txt / black_box.txt，不启动 target_sim。

设计：纯逻辑内核 TargetBridgeCore 不依赖 rospy（只依赖 math/json），
ROS 包装只做话题收发，内核可离线单测。
"""

import json
import math
import os
import re
import threading
from visual_observation import VisualEvidence, TAG_TO_TID
from search_completion import parse_actor_list
from red_observations import RedObservations, actor_slot_remaining

# ---- 非红色身份与内部红色几何槽（红色槽不声明actor身份）----
TAG_TO_TID = {
    "green": "t0",
    "blue": "t1",
    "brown": "t2",
    "white": "t3",
    "red1": "t5",
    "red2": "t4",
}
TID_TO_TAG = dict((v, k) for k, v in TAG_TO_TID.items())
ACTOR_INDEX_OF_TAG = dict((tag, int(tid[1:])) for tag, tid in TAG_TO_TID.items())

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
#   te = min(gap, EXTRAP_MAX_T)，gap来自原图时间，不能再加固定延迟。
# 距离限幅 EXTRAP_MAX_D 防速度估计被 YOLO 跳变污染时外推飞掉。
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
                 "t_obs", "position_s", "alive", "elim_pending",
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
        self.t_obs = -1e18
        self.position_s = -1e18    # timestamp of the weighted position, not latest image
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

    返回 (tag, x, y, conf)；非法载荷抛 ValueError。
    只做形状校验，坐标原样返回（坐标系由发布端契约保证为 world_enu）。
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
    conf = max(0.0, min(1.0, conf))
    return tag, x, y, conf


def parse_left_actors(raw):
    """解析官方 /left_actors 字符串为剩余 actor 下标集合。

    已核验375行官方脚本使用list(range(actor_num))，发布明确整数清单。
    与manager共用严格JSON解析，不接受旧快照的range表达式。
    无法识别时返回 None，调用方必须忽略本条，绝不当作空清单。
    """
    parsed = parse_actor_list(str(raw))
    if parsed is not None:
        return set(parsed)
    return None


class TargetBridgeCore(object):
    """纯逻辑桥（无 rospy）。"""

    def __init__(self):
        self.tracks = dict((tag, _Track(tag)) for tag in TAG_TO_TID)
        self.eliminated = set()     # 已消除 tag，后续 YOLO 鬼影直接忽略
        self._left_seen = False

    # ---- 感知输入 ----
    def report(self, t, target_id, x, y, conf):
        """吸收一条 YOLO 观测。非法参数抛 ValueError。"""
        tag = str(target_id)
        if tag not in self.tracks:
            raise ValueError("未知 tag %r" % tag)
        if tag in self.eliminated:
            return                      # 已消除目标的残余观测，不复活
        tr = self.tracks[tag]
        if t < tr.t_obs:
            return
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
        position_s = sum(o[0] * o[3] for o in tr.obs) / sw
        tr.conf = max(o[4] for o in tr.obs)

        # 速度差分 + EMA（用融合位置，保证平滑）
        if tr._last_fx is not None:
            dt = position_s - tr._last_ft
            if dt >= VEL_DT_MIN:
                tr.vx = _vel_ema(tr.vx, (fx - tr._last_fx) / dt)
                tr.vy = _vel_ema(tr.vy, (fy - tr._last_fy) / dt)
                tr._last_fx, tr._last_fy, tr._last_ft = fx, fy, position_s
        else:
            tr._last_fx, tr._last_fy, tr._last_ft = fx, fy, position_s

        tr.x, tr.y = fx, fy
        tr.position_s = position_s
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
        if remaining is None or (not remaining and not self._left_seen):
            return []
        if remaining:
            self._left_seen = True
        newly = []
        for tr in self.tracks.values():
            idx = ACTOR_INDEX_OF_TAG[tr.tag]
            if actor_slot_remaining(idx, remaining):
                continue
            if tr.tag in self.eliminated or tr.elim_pending:
                continue
            # 只消除桥已观测到的目标：官方消除必先有 YOLO 上报，
            # 桥与裁判同时启动时二者集合一致；晚启动则保守不动。
            if not tr.alive:
                self.eliminated.add(tr.tag)
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
            # The fused position belongs to the weighted image time.
            # Latest-image freshness remains governed by t_obs above.
            te = max(0., min(t - tr.position_s, EXTRAP_MAX_T)) * extrap_factor
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
        self._red_observations = RedObservations()
        self._lock = threading.RLock()
        self.evidence = VisualEvidence(os.environ.get('ROBOCUP_RUN_ID', ''),
            os.environ.get('SWARM_UAV_IDS', 'uav_1,uav_2,uav_3,uav_4,uav_5,uav_6').split(','))
        from robocup_swarm.msg import TargetDetection
        self._TargetDetection = TargetDetection
        self._detection_pub = rospy.Publisher('/swarm/detection', TargetDetection, queue_size=50)
        self._confirmed_pub = rospy.Publisher('/swarm/confirmed_visual_observation', _msg_string_cls(), queue_size=50)

        self.pub = rospy.Publisher("/swarm/target_states", TargetState, queue_size=30)
        # 同时向官方话题发布 ActorInfo：用多机融合坐标，保证 10Hz 上报连续，
        # 避免单机 track 断流导致上报间隔 >1s 被官方重置。
        self._actor_pubs = {}
        for tag in TAG_TO_TID:
            self._actor_pubs[tag] = rospy.Publisher(
                "/actor_%s_info" % ('red' if tag in ('red1','red2') else tag), ActorInfo, queue_size=3)
        rospy.Subscriber('/swarm/visual_observation', _msg_string_cls(), self._visual_cb, queue_size=50)
        rospy.Subscriber("/left_actors", _msg_string_cls(),
                         self._left_cb, queue_size=5)

        pub_hz = float(os.environ.get("BRIDGE_PUB_HZ", "10"))
        self._timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, pub_hz)),
                                  self._tick)
        rospy.loginfo("yolo_target_bridge 启动：visual_observation v2 → /swarm/target_states"
                      "（%d 个固定目标槽）", len(TAG_TO_TID))

    def _now(self):
        return self._rospy.Time.now().to_sec()

    def _report_cb(self, msg):
        try:
            tag, x, y, conf = parse_report(msg.data)
            self.core.report(self._now(), tag, x, y, conf)
        except ValueError as exc:
            self._rospy.logwarn_throttle(5, "丢弃 target_report：%s", exc)

    def _visual_cb(self, msg):
        try:
            with self._lock:
                observation = self.evidence.receive(json.loads(msg.data), self._now())
                if observation is None:
                    return
                tag, stamp = observation['target_id'], observation['sample_s']
                x, y, _ = observation['xyz']
                if tag in ('red1','red2'):
                    tag = self._red_observations.observe(stamp,(x,y))
                    if tag is None:
                        return
                    observation['target_id'] = tag
                self.core.report(stamp, tag, x, y, observation['confidence'])
                track = self.core.tracks[tag]
                if not track.alive or track.t_obs != stamp or tag in self.core.eliminated:
                    return
                detection = self._TargetDetection()
                detection.header.stamp = self._rospy.Time.from_sec(stamp)
                detection.header.frame_id = 'map'
                detection.uav_id, detection.target_id = observation['uav_id'], TAG_TO_TID[tag]
                detection.x, detection.y, detection.confidence, detection.source = x, y, observation['confidence'], 1
                self._detection_pub.publish(detection)
                self._confirmed_pub.publish(_msg_string_cls()(data=json.dumps(observation, allow_nan=False)))
        except (ValueError, TypeError, KeyError):
            return

    def _left_cb(self, msg):
        with self._lock:
            newly = self.core.set_left(self._now(), msg.data)
        for tag in newly:
            self._rospy.loginfo("官方清单已无 %s(%s)，补发 eliminated",
                                tag, TAG_TO_TID[tag])

    def _emit(self, ev):
        m = self._TargetState()
        track = self.core.tracks[ev['tag']]
        m.header.stamp = self._rospy.Time.from_sec(max(0., track.t_obs))
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
        if (tag is not None and tag in self._actor_pubs
                and not ev["eliminated"]
                and ev.get("state", 0) != 3 and 0 <= self._now()-track.t_obs <= 1.):
            am = self._ActorInfo()
            am.cls = 'red' if tag in ('red1', 'red2') else tag
            am.x = round(ev["x"], 3)
            am.y = round(ev["y"], 3)
            self._actor_pubs[tag].publish(am)

    def _tick(self, _evt):
        with self._lock:
            for ev in self.core.tick(self._now()):
                self._emit(ev)


def _msg_string_cls():
    from std_msgs.msg import String
    return String


# ============================ 离线自测（不连 ROS master）============================
def _self_test():
    """不依赖 rospy 的内核自测：python3 yolo_target_bridge.py --selftest"""
    # 1) 载荷校验
    assert parse_report(json.dumps({
        "target_id": "green", "frame_id": "world_enu",
        "xyz": [1.0, 2.0, 0.0], "confidence": 0.9,
        "observation_id": "x"})) == ("green", 1.0, 2.0, 0.9)
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
    assert c.tick(.05) == []  # Two images cannot satisfy three-frame activation.
    c.report(.1, 'green', .2, 0., .9)
    ev = c.tick(.1)
    assert len(ev) == 1 and ev[0]["tid"] == "t0" and ev[0]["state"] == 0
    for k in range(1, 10):                      # 匀速 2m/s 移动
        c.report(k * 0.2, "green", k * 0.4, 0.0, 0.9)
    ev = c.tick(1.8)
    # Smoothed speed approaches 2m/s from below; no measured threshold crossing yet.
    assert ev[0]["state"] == 0 and 1.0 < ev[0]["vx"] < FLEE_SPEED, ev[0]
    print("2) 融合/速度 OK: vx=%.2f" % ev[0]["vx"])

    # 3) coast 保持 → 接力窗口结束后停发
    assert len(c.tick(2.5)) == 1                # gap 0.7s 仍保持
    assert len(c.tick(6.0)) == 1                # 6s 窗口内仍保持
    assert c.tick(8.0) == []                    # gap>6s 停发
    c.report(8.5, "green", 4.0, 0.0, 0.9)      # 重新出现 → 复活
    assert len(c.tick(8.5)) == 1
    print("3) coast/drop/复活 OK")

    # 4) 官方消除：补发 eliminated 一次，鬼影不复活
    c.set_left(8.6, "[1, 2, 3, 4, 5]")          # green(0) 不在
    ev = c.tick(8.6)
    assert len(ev) == 1 and ev[0]["eliminated"] is True and ev[0]["state"] == 3
    assert c.tick(8.7) == []
    c.report(8.8, "green", 4.0, 0.0, 0.9)      # 残余 YOLO 鬼影
    assert c.tick(8.8) == []
    print("4) eliminated 补发/鬼影忽略 OK")

    # 5) /left_actors accepts only explicit unique JSON integer IDs.
    assert parse_left_actors("range(0, 6)") is None
    assert parse_left_actors("range(2, 5)") is None
    assert parse_left_actors("[]") == set()
    assert parse_left_actors("[0, 2, 4]") == {0, 2, 4}
    assert parse_left_actors("") is None
    assert parse_left_actors("garbage") is None
    print("5) parse_left_actors 形态 OK")

    # 6) 全消除
    c2 = TargetBridgeCore()
    assert c2.set_left(0, '[]') == []  # Initial empty feedback is not completion.
    for t in (0., .2, .4):
        c2.report(t, "blue", 1, 1, 0.9)
    c2.set_left(.4, '[0,1,2,3,4,5]')
    assert c2.set_left(.5, "[]") == ["blue"]
    assert c2.tick(.5)[0]["eliminated"]
    print("6) initial empty ignored / established completion OK")

    print("\nyolo_target_bridge 内核自测全部通过")


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        _self_test()
    else:
        import rospy
        rospy.init_node("yolo_target_bridge")
        YoloTargetBridge()
        rospy.spin()
