#!/usr/bin/env python3
# coding: utf-8
"""YOLO 链路版飞行器（比赛合规路径）—— 单机找人 → 消除 → 转下一个。

与 ~/robocup_real/fly_to_actor.py 的关系
────────────────────────────────────────
飞控骨架（已在本机验证可靠：稳定飞行 / 分段小步逼近 / 跟随悬停 / 姿态不翻转）
沿用它的 6 条设计要点，本文件只做两件事：
  1) 把「目标位置来源」从官方真值话题 /actor_N/pose 换成我们自己的合法链路
         yolo_target_bridge → /swarm/target_states (robocup_swarm/TargetState)
  2) 加「消除一个转下一个」的循环（用户 10-03 明确要求）

合规性
──────
规则 §2.5.11：裁判实时监控节点订阅，防止违规订阅。本脚本**不订阅**
/actor_N/pose、/gazebo/model_states|link_states、/gazebo/get_model_state。
只订阅：
    /swarm/target_states        我们自己的桥输出（来源 YOLO 合法检测）
    /<uav>/mavros/*             本机飞控
    /left_actors                裁判自己发布的剩余清单（官方公开话题）
    /<uav>/stereo_camera/left/image_raw   本机相机

────────────────────────────────────────────────────────────
官方源码给出的三条硬约束（2026-10-03 逐行核对，改动必须据此）
────────────────────────────────────────────────────────────
A. **场地**：control_actor.py:43-46
       self.x_max = 130.0 ; self.x_min = -50.0
       self.y_max =  60.0 ; self.y_min = -60.0
   actor 只在这个框里生成/移动。飞机也必须在这个框里 ⇒ 所有设定点
   **在这个唯一出口处硬夹**，物理上不可能飞出（上一版把框写成 ±48m，
   既挡不住出界、还把场内 x=60 误判为出界）。

B. **逃逸判据**：control_actor.py:211-219
       if catching_flag == 0 and uav_speed_squared > 1.0 and dis < 20.0:
           tracking_flag[i] += 1
           if tracking_flag[i] > 20:        # ≈2 秒
               catching_flag = 1            # actor 以 tracked_speed=2.0 m/s 逃
   ⇒ **无人机在 20m 内速度 >1 m/s 持续 2 秒，人就开始逃**，且只有触发的那架
   机离开 20m 才解除（:226-230）。所以全程水平速度压在 ESCAPE_SAFE_SPEED
   (0.9 m/s) 以下 —— 既不触发逃逸，也天然消除"方向一错冲出几百米"。

C. **判分**：score_cal.py:19,26,31,32
       err_threshold = 1            # 广播误差 < 1m
       DETECTION_INTERVAL = 1.0     # 广播间隔 ≤ 1s
       DETECTION_DURATION = 15.0    # 连续 15s 才删机
       TRACKING_DISTANCE = 10.0 ; TRACKING_DURATION = 20.0
   ⇒ 只要有一条广播不达标，_reset_detection 就把计时清零 ⇒ **宁可不说，
   也不能说错**（距离闸门在 yolo_target_bridge 里，阈值 12m）。
   另：/find_actor_N 由**第一次播准**触发（score_cal.py:147-148），
   而 control_actor.py:112-127 的瞬移回调是忙等 25s 的死循环 ⇒
   **播准第一条就把人冻住 25 秒**，足够稳稳播满 15s 删机。

沿用的飞控要点（来自 fly_to_actor.py 头注，10-03 亲测）
────────────────────────────────────────────────────
1. setpoint 由**独立线程 20Hz 持续发布**；主线程阻塞也不断流。
   断流 >COM_OF_LOSS_T(1.0s) ⇒ PX4 失效保护 ⇒ 降落锁死。
2. **分段小步飞**：设定点每帧只推进 step 米。
3. 全程用 rospy.get_time()（仿真时间）计时。
4. yaw 对准目标（水平前视相机必须朝人）。
5. OFFBOARD 切换前必须先有 setpoint 流。
6. 高度硬护栏：官方 >6m 直接 score=0（硬夹 ALT_MAX_M）。
"""
import math
import os
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ParamValue
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, ParamSet, SetMode
from robocup_swarm.msg import TargetState
from sensor_msgs.msg import Image
from std_msgs.msg import String

UAV = os.environ.get("FLY_UAV", "typhoon_h480_0")
MS = "/%s/mavros" % UAV

# ---- 官方写死的 6 个起飞点（control_actor.py:55-57，原文照抄，不得臆造）----
#     self.uav_takeoff_points = [(0.0, -3.0), (3.0, -3.0),
#                                (0.0,  0.0), (3.0,  0.0),
#                                (0.0,  3.0), (3.0,  3.0)]
# 机号 typhoon_h480_N 的起飞点即表中第 N 项 ⇒ 起飞点无需"标定"，是常量。
UAV_TAKEOFF_POINTS = [(0.0, -3.0), (3.0, -3.0),
                      (0.0, 0.0), (3.0, 0.0),
                      (0.0, 3.0), (3.0, 3.0)]
try:
    UAV_ID = int(os.environ.get("FLY_UAV_ID", UAV.rsplit("_", 1)[1]))
except (IndexError, ValueError):
    UAV_ID = 0
SPAWN_XY = UAV_TAKEOFF_POINTS[UAV_ID % len(UAV_TAKEOFF_POINTS)]
# actor 与起飞点的最小间隔（control_actor.py:54 uav_spawn_distance=8.0）
ACTOR_MIN_FROM_TAKEOFF = 8.0

# ---- 官方场地框（control_actor.py:43-46，一字不改）----
FIELD_XMIN, FIELD_XMAX = -50.0, 130.0
FIELD_YMIN, FIELD_YMAX = -60.0, 60.0
SAFE = 6.0                     # 内缩余量：飞机的设定点永远离边界 ≥6m

# ---- 飞行参数（默认值来自 fly_to_actor.py 的实测可用值）----
SP_HZ = 20.0            # setpoint 发布频率（铁律 1）
ALT_DEFAULT = 2.5       # 巡航高度 m
ALT_MAX_M = 5.0         # 硬顶：官方 6m 判 0，留 1m 余量
KEEP_M_DEFAULT = 7.0    # 观察距离 m（<10m 才有跟踪分；实测 18m 时误差 2.75m）

# ---- 官方逃逸线（control_actor.py:211）----
ESCAPE_RANGE = 20.0     # m：这个距离内速度 >1m/s 会触发逃逸
ESCAPE_SAFE_SPEED = 0.9  # m/s：全程速度上限（<1.0 才不会触发逃逸）

# ---- 目标锁定（"消除一个再换下一个"的落地方式，见 nearest_target）----
LOCK_SWITCH_S = 6.0      # s：锁定目标失联超过这么久才允许换人（防止来回翻转）

# ---- 观察点锚定 / 到位静止（2026-10-03 加，直指"25s 内消不掉"）----
# 官方时间线（control_actor.py:112-127 + score_cal.py:130-165）：
#   首次播报误差<1m ⇒ 裁判 publish /find_actor_N ⇒ actor 控制器开始 25s 倒计时，
#   到点把 actor 瞬移到新的随机点；同时裁判要攒"误差<1m 且间隔≤1s 连续 15s"
#   才会 delete_model 把它消掉。⇒ **窗口只有 25s，需求 15s，净余量 10s**，
#   而且中途断一次就清零重来、清零后重新对上又会再发一次 /find_actor_N（等于
#   在瞬移队列里再叠一次）——代价是双重的。
# 破坏连续性的最大来源不是感知精度，而是**飞机自己一直在动**：
#   原版 run() 每帧用「目标 + keep_m × (本机−目标)方向」重算观察点 ⇒ 本机动一点
#   观察点就跟着动 ⇒ 永远在追一个移动的点 ⇒ 相机姿态持续变化 ⇒ YOLO 输出抖 ⇒
#   误差在 1m 线两侧反复穿越。所以到位后必须**真正静止**。
OBSP_REFRESH_M = 4.0     # m：目标走出这么远（或本机被推离锚这么远）才重算观察点
YAW_DEADBAND_RAD = 0.26  # rad(≈15°)：机头死区，小幅位移不重算 yaw，抑制画面旋转抖动

# ---- 目标可信度（单目测距远距离误差大，离谱的估计不追）----
TARGET_MAX_RANGE = 60.0  # m：比这更远的目标位置估计不可信，不追
TARGET_STALE = 8.0       # s：这么久没更新就丢弃

# ---- 安全看门狗 ----
POS_SLACK = 25.0         # m：位置估计允许超出场地框的余量，超过即判"估计坏了"
POS_BAD_Z = -1.0         # m：高度低于此值判估计坏了（地面静止时 local z 实测约 -0.5，
                         #    故不能用 -0.5，否则飞机还在地面就被误判）
POS_BAD_LIMIT = 8.0      # s：连续这么久估计不可信就停止控制
MISSION_LIMIT = 600.0    # s：官方 MISSION_TIMEOUT=600

# ---- 起飞前自检（2026-10-03 加，两条都是事故后补的硬门槛）----
# (1) 标定守卫：坐标标定隐含前提"此刻飞机就在官方起飞点"。若飞机已被别的控制器
#     带走 δ，则 off = O − δ，flyer 算出的世界坐标 = 真值 − δ，而它把设定点转回
#     局部时又减 off ⇒ PX4 执行后飞机落在「意图目标 + δ」。δ 可达 60m ⇒ 每次必
#     出界。注意：这种情况**看门狗抓不到**（flyer 自认为在场地中心附近）。
#     所以必须在 ARM 前证明飞机仍在起飞点。
SPAWN_TOL_XY = 2.0       # m：local 水平模长小于此值才算"仍在起飞点"
SPAWN_TOL_Z = 1.0        # m：local 高度绝对值小于此值才算"还在地面"
# (2) 独占守卫：铁律 4 —— 同一时刻只能有一个 setpoint 发布者。
#     XTDrone 的 multirotor_communication.py 会**无条件 30Hz** 广播
#     setpoint_raw/local=(0,0,0)（其 target_motion 初值全零、type_mask=0），
#     与我们 20Hz 的 setpoint_position/local 互抢 ⇒ 飞机被两个吸引子来回撕扯，
#     实测表现为坐标在两点间反复跳变并被拖出场地。必须独占。
SP_COMPETITOR_TOPIC_SUFFIX = "/setpoint_raw/local"

# ---- 搜索（找不到人时用；全部点都在场地框内）----
SCAN_DWELL = 8.0         # s：每个搜索点原地旋转扫视时长（原 12s 太费里程）
SCAN_YAW_STEP = 0.039    # rad/帧：20Hz × 8s ≈ 2π，即每停留点转一整圈
SEARCH_SPEED = 0.9       # m/s：转场速度（同样 <1 以免触发逃逸）
SEARCH_PTS = [
    (-40.0, -45.0), (-40.0, 0.0), (-40.0, 45.0),
    (0.0, 45.0), (0.0, 0.0), (0.0, -45.0),
    (40.0, -45.0), (40.0, 0.0), (40.0, 45.0),
    (80.0, 45.0), (80.0, 0.0), (80.0, -45.0),
    (120.0, -45.0), (120.0, 0.0), (120.0, 45.0),
]
SEARCH_ARRIVE = 4.0      # m：到达搜索点的判定距离


def order_search_pts(spawn):
    """按"最近邻"给搜索点重排（从起飞点出发的贪心路径）。

    原版是固定蛇形顺序，第一个点是 (-40,-45) —— 从起飞点 (0,-3) 过去要飞 50m，
    而人可能就在 20m 外的另一个方向。贪心重排把总里程压下来，同样 600s 能多扫
    几个点。只改顺序，不改点集（点集本身覆盖整个场地框）。
    """
    pts, cur = list(SEARCH_PTS), (spawn[0], spawn[1])
    out = []
    while pts:
        i = min(range(len(pts)),
                key=lambda k: (pts[k][0] - cur[0]) ** 2 + (pts[k][1] - cur[1]) ** 2)
        cur = pts.pop(i)
        out.append(cur)
    SEARCH_PTS[:] = out
    return out

# 官方 score_cal.py 的 actor_id_dict（身份映射以官方为准）
#   green->0 blue->1 brown->2 white->3 /actor_red2_info->4 /actor_red1_info->5
TAG_ACTOR = {'green': 0, 'blue': 1, 'brown': 2, 'white': 3, 'red1': 5, 'red2': 4}
# /swarm/target_states 的 target_id 形如 t0..t5（桥内部 tid）。
# 2026-10-03 已与桥的 TAG_TO_TID 对齐为 **tN ↔ actor_N**（团队约定：
# swarm_viz.py:17、detection_to_official.py:78-83），红球不再反着写。
TID_ACTOR = {0: 'green', 1: 'blue', 2: 'brown', 3: 'white', 4: 'red2', 5: 'red1'}
# 注意"桥内部 tid"与"官方 actor 号"现在是同一个数，但**发到官方红球话题的坐标
# 不按槽位走**：桥已改为"同一红球坐标同发 /actor_red1_info + /actor_red2_info"
# （见 yolo_target_bridge.py 的 RED_DUAL 注释）——两条 red 流官方各自比对
# actor_5 / actor_4，有一个对上就计分。


class YoloFlyer(object):
    def __init__(self):
        self.state = None
        self.local = None            # PoseStamped 的 position（MAVROS 局部坐标）
        self.targets = {}            # tid('tN') -> (x, y)  世界坐标，来自 YOLO 桥
        self.t_seen = {}             # tid -> 最后更新时间
        self.left_now = set()        # 官方剩余 actor 清单
        self.left_prev = set()
        self.eliminated = set()      # 已被官方删掉的 actor id
        self._off = (0.0, 0.0)       # local→world 平移（起飞时标定）
        self._yaw_now = 0.0          # 当前机头 yaw（ENU，从 local_position 四元数取）

        # --- 雷达避险层（RADAR_GUARD=1 启用）---
        # 只把 (wx_t, wy_t) 换成避障子目标，**不发布任何 setpoint**
        # ⇒ 仍是单一 setpoint 发布者（铁律 4）。加载失败按无避障运行。
        self.radar = None
        if os.environ.get("RADAR_GUARD", "0") == "1":
            try:
                from radar_guard import RadarGuard
                self.radar = RadarGuard("/%s/scan" % UAV)
            except Exception as _e:
                print("[yf] ⚠ 雷达层加载失败，按无避障运行：%s" % _e, flush=True)

        # 搜索状态机
        self._scan_mode = 'scan'     # 'scan' | 'travel'
        self._scan_until = None
        self._wp_i = 0
        self._yaw = 0.0
        self._sp_w = None            # 内部限速参考点（世界坐标），见 step_toward
        self._anchor = None          # 扫视/悬停锚点：必须钉死一个固定点，
                                     # 否则设定点跟着当前位漂 ⇒ 无修正量、漂移累积
        self._lock = None            # 已锁定的目标 tid（追到被消除为止）
        self._lock_lost_t = 0.0      # 锁定目标最后一次有效的时刻
        self._obsp = None            # 观察点锚 (ax, ay, tx0, ty0)：到位后钉死，
                                     # 只有目标走远/本机被推离才重算（见 OBSP_REFRESH_M）

        self._sp_lock = threading.Lock()
        self.sp = None
        self._stop = False
        self._t_log = 0.0
        self._t_guided = 0.0
        self._n_guided_warn = 0

        # --- 订阅（全部合法）---
        rospy.Subscriber(MS + "/state", State, self._state_cb, queue_size=1)
        rospy.Subscriber(MS + "/local_position/pose", PoseStamped,
                         self._local_cb, queue_size=1)
        rospy.Subscriber("/swarm/target_states", TargetState,
                         self._target_cb, queue_size=10)
        rospy.Subscriber("/left_actors", String, self._left_cb, queue_size=1)
        rospy.Subscriber("/%s/stereo_camera/left/image_raw" % UAV, Image,
                         self._img_cb, queue_size=1)

        # --- 发布 ---
        self.pub_pose = rospy.Publisher(MS + "/setpoint_position/local",
                                        PoseStamped, queue_size=1)
        self.srv_arm = rospy.ServiceProxy(MS + "/cmd/arming", CommandBool)
        self.srv_mode = rospy.ServiceProxy(MS + "/set_mode", SetMode)
        self.srv_param = rospy.ServiceProxy(MS + "/param/set", ParamSet)

        threading.Thread(target=self._sp_loop, daemon=True).start()

    # ---------------- 回调 ----------------
    def _state_cb(self, m):
        self.state = m

    def _local_cb(self, m):
        self.local = m.pose.position
        # 机头 yaw（ENU）：雷达层要把 scan 从机体系转到世界系，必须用真机头角
        q = m.pose.orientation
        self._yaw_now = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _img_cb(self, m):
        pass

    def _target_cb(self, m):
        """来自 yolo_target_bridge 的目标（合法检测链路，世界坐标）。"""
        if m.eliminated or m.state == 3:
            self.targets.pop(m.target_id, None)
            return
        self.targets[m.target_id] = (m.x, m.y)
        self.t_seen[m.target_id] = rospy.get_time()

    def _left_cb(self, m):
        """官方剩余 actor 清单 → 判断谁被消除了，自动转下一个。

        用户 10-03 要求：**消除一个接着找下一个**。这里比对前后两帧差集，
        就知道刚被官方删掉的是哪个，把它从目标表摘掉；若它正是我们锁定的
        目标，立刻解锁 ⇒ nearest_target() 下一帧就会锁上另一个还活着的人，
        无需外部干预。
        """
        try:
            left = set(int(x) for x in eval(m.data))
        except Exception:
            return
        self.left_now = left
        if self.left_prev:
            for aid in (self.left_prev - left):
                self.eliminated.add(aid)
                tid = 't%d' % aid
                self.targets.pop(tid, None)
                print("[yf] ★ actor_%d 已被官方消除（剩 %d 个）→ 转下一个目标"
                      % (aid, len(left)), flush=True)
        self.left_prev = set(left)
        # 锁定的目标若已从目标表消失（被消除/被桥停发），立刻解锁换人
        if self._lock is not None and self._lock not in self.targets:
            print("[yf] 锁定目标 %s 已消失 → 解锁，换下一个" % self._lock, flush=True)
            self._lock = None

    def _sp_loop(self):
        r = rospy.Rate(SP_HZ)
        while not rospy.is_shutdown() and not self._stop:
            with self._sp_lock:
                p = self.sp
            if p is not None:
                p.header.stamp = rospy.Time.now()
                try:
                    self.pub_pose.publish(p)
                except Exception:
                    pass
            r.sleep()

    # ---------------- 坐标系 ----------------
    def world(self):
        """本机**世界坐标** = MAVROS 局部坐标 + 起飞点平移。

        /swarm/target_states 的 x/y 是世界 ENU（与官方 score_cal 同一套）；
        MAVROS /local_position/pose 是以起飞点为原点的局部 ENU，二者只差
        常量平移。基准脚本 fly_to_actor.py 直接读 get_model_state 真值所以
        不用换算；我们不能订阅真值（规则 §2.5.11），故在起飞时用官方写死的
        起飞点 (0,-3) 与当时的 local 做差，得到偏移。全程只用 MAVROS，合法。
        """
        if self.local is None:
            return None
        return (self.local.x + self._off[0],
                self.local.y + self._off[1],
                self.local.z)

    def set_offset_from(self, spawn_wx, spawn_wy, local_x, local_y):
        self._off = (float(spawn_wx) - float(local_x),
                     float(spawn_wy) - float(local_y))
        print("[yf] 坐标标定：起飞点 world=(%.2f,%.2f) local=(%.2f,%.2f) "
              "⇒ offset=(%.2f,%.2f)"
              % (spawn_wx, spawn_wy, local_x, local_y, self._off[0], self._off[1]),
              flush=True)

    def clamp_world(self, wx, wy):
        """把世界坐标夹进场地**安全框**（官方框再内缩 SAFE）。

        这是唯一出口：任何路径（追人/巡逻/返场）都经过这里，所以飞机
        物理上不可能飞出场地。
        """
        return (max(FIELD_XMIN + SAFE, min(FIELD_XMAX - SAFE, wx)),
                max(FIELD_YMIN + SAFE, min(FIELD_YMAX - SAFE, wy)))

    # ---------------- 设定点 ----------------
    @staticmethod
    def pose_msg(lx, ly, lz, yaw_rad):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = lx
        p.pose.position.y = ly
        p.pose.position.z = lz
        p.pose.orientation.z = math.sin(yaw_rad / 2.0)
        p.pose.orientation.w = math.cos(yaw_rad / 2.0)
        return p

    def set_sp(self, lx, ly, lz, yaw_rad=0.0):
        msg = self.pose_msg(lx, ly, min(lz, ALT_MAX_M), yaw_rad)
        with self._sp_lock:
            self.sp = msg

    def set_sp_world(self, wx, wy, wz, yaw_rad=0.0):
        """世界坐标设定点（内部夹框 + 转局部）。"""
        wx, wy = self.clamp_world(wx, wy)
        with self._sp_lock:
            self.sp = self.pose_msg(wx - self._off[0], wy - self._off[1],
                                    min(wz, ALT_MAX_M), yaw_rad)

    def step_toward(self, wx_t, wy_t, alt, yaw, speed, leash=None):
        """世界坐标**限速推进 + 皮带**逼近（2026-10-03 重写，原版有 20 倍速度 bug）。

        ⛔ 原版错在哪（实测复现）：
            原版每帧把设定点设为「**当前位置** + speed/SP_HZ」= 当前位置 + 0.045m。
            位置环是 P 控制 v = Kp·e（PX4 MPC_XY_P≈0.95），而设定点永远只领先
            飞机 0.045m ⇒ 稳态误差 e = s/(1+Kp·dt)，实际速度
                v = Kp·s/(1+Kp·dt) ≈ 0.91·s = 0.041 m/s
            实测真值 0.07~0.1 m/s，与理论吻合。600 秒只能飞 ~25m，永远找不到人。
            根子在这句话：注释写"step = speed/SP_HZ 所以实际速度 ≈ speed" ——
            **错的**。位置环增益 Kp 在分母上，步长必须按 Kp 反算。

        ✅ 现在的做法：内部维护一个**自主限速的参考点** `self._sp_w`，
            它每帧按 speed/SP_HZ 自走（与飞机当前位置无关）⇒ 飞机的稳态速度
            = 参考点速度 = speed（这才是"实际速度≈speed"成立的条件）。
            再加一条**皮带**：参考点离飞机不得超过 leash 米；否则拉回到
            leash 处。作用是①起飞/被拦住时不让误差（也就是瞬时速度）爆炸，
            ②使瞬时速度上限 ≈ Kp·leash，取 leash=speed/Kp 就恰好等于 speed，
            保证不会越过官方逃逸线（20m 内 >1.0 m/s 持续 2s 人就跑）。
        """
        w = self.world()
        if w is None:
            return
        if leash is None:
            # 瞬时速度上限 Kp·leash 要等于 speed ⇒ leash = speed/Kp。
            # Kp 取 PX4 MPC_XY_P 默认 0.95；再硬夹到 0.85 留安全余量。
            leash = min(0.85, speed / 0.95)
        # ---- 雷达避险：把期望目标点换成"沿可通通道外推"的子目标 ----
        # 单机带雷达机型（sdf=typhoon_h480_lidar）才有 scan；无数据则原样透传。
        if self.radar is not None and self.radar.has_scan:
            wx_t, wy_t = self.radar.guard(wx_t, wy_t, self._yaw_now,
                                          (w[0], w[1]))
        wx_t, wy_t = self.clamp_world(wx_t, wy_t)
        if self._sp_w is None:
            self._sp_w = (w[0], w[1])
        sx, sy = self.clamp_world(self._sp_w[0], self._sp_w[1])
        # 1) 参考点自主前进（限速）
        dx, dy = wx_t - sx, wy_t - sy
        gap = math.hypot(dx, dy)
        adv = speed / SP_HZ
        if gap <= adv or gap < 1e-9:
            sx, sy = wx_t, wy_t
        else:
            sx, sy = sx + dx / gap * adv, sy + dy / gap * adv
        # 2) 皮带：参考点离飞机不能超过 leash
        ex, ey = sx - w[0], sy - w[1]
        e = math.hypot(ex, ey)
        if e > leash:
            sx, sy = w[0] + ex / e * leash, w[1] + ey / e * leash
        self._sp_w = (sx, sy)
        self.set_sp_world(sx, sy, alt, yaw)

    # ---------------- 解锁与起飞 ----------------
    def send(self, s):
        try:
            if s == "ARM":
                # mavros CommandBool 只有 value 一个字段（传两参会
                # "Invalid number of arguments"）
                try:
                    r = self.srv_arm(value=True)
                except TypeError:
                    r = self.srv_arm(0, True)
                print("[yf] ARM -> %s" % r.success, flush=True)
            elif s == "OFFBOARD":
                r = self.srv_mode(custom_mode="OFFBOARD")
                print("[yf] OFFBOARD -> %s" % r.mode_sent, flush=True)
        except Exception as e:
            print("[yf] %s 失败: %s" % (s, e), flush=True)

    def set_param(self, pid, val):
        """设一个 PX4 参数（易失，每次飞前都要设）。"""
        try:
            pv = ParamValue()
            pv.integer = int(val)
            pv.real = float(val)
            r = self.srv_param(param_id=pid, value=pv)
            print("[yf] param %s=%s -> success=%s" % (pid, val,
                                                      getattr(r, 'success', '?')),
                  flush=True)
            return bool(getattr(r, 'success', False))
        except Exception as e:
            print("[yf] param %s 设置失败: %s" % (pid, e), flush=True)
            return False

    def set_params_flight(self):
        """起飞前必设的两个**易失**参数（来自已验证 3h 的 fly_to_actor.py:358）。

        无头 SITL 没有遥控器、也没有地面站数据链。若不关掉这两条失效保护，
        PX4 会同时挂着"遥控丢失(NAV_RCL_ACT)"与"数据链丢失(NAV_DLL_ACT)"告警，
        **模式串虽然显示 OFFBOARD，控制权实际被失效保护压制**：
          · mavros 报 guided=False
          · PX4 下令爬升 +1.96m/s，机体却下沉（真值实测）
          · 水平设定点几乎不被执行（原地不动 / 随机漂）
        这两个参数不写进 ROM，重启即失效 ⇒ 每次飞前都设。
        """
        for pid in ("NAV_RCL_ACT", "NAV_DLL_ACT"):
            self.set_param(pid, 0.0)

    def wait_armed(self, timeout=25.0):
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t0 < timeout:
            if self.state and self.state.armed:
                return True
            rospy.sleep(0.2)
        return False

    def wait_mode(self, mode, timeout=20.0):
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t0 < timeout:
            if self.state and self.state.mode == mode:
                return True
            rospy.sleep(0.2)
        return False

    @staticmethod
    def setpoint_competitors():
        """列出 `/<uav>/mavros/setpoint_raw/local` 上除 mavros 之外的发布者。

        铁律 4：同一时刻只能有一个 setpoint 发布者。XTDrone 的
        multirotor_communication.py 启动即无条件 30Hz 广播 (0,0,0)，
        实测会把飞机拖向原点并冲出场地。返回 None 表示查询失败（不阻断）。
        """
        topic = MS + SP_COMPETITOR_TOPIC_SUFFIX
        try:
            import rosgraph
            master = rosgraph.Master('/yolo_flyer_guard')
            pubs, _subs, _srvs = master.getSystemState()
            bad = []
            for t, nodes in pubs:
                if t != topic:
                    continue
                for n in nodes:
                    # mavros 自身的 setpoint 插件订阅该话题，不是竞争者
                    if 'mavros' in n:
                        continue
                    bad.append(n)
            return bad
        except Exception as e:
            print("[yf] 查询 setpoint 发布者失败（跳过该检查）：%s" % e, flush=True)
            return None

    def live_setpoint_competitor(self, window=3.0):
        """行为法判竞争流：我们**自己不发布**的前提下，监听 setpoint_raw/local。

        返回 (消息条数, 速率Hz)。>0 说明真有活着的节点在往 PX4 灌设定点。
        比查 ROS master 注册表可靠得多 —— 注册表会被死进程残留污染（实测
        SIGKILL 后 >20s 仍列出），消息流不会。
        """
        got = {'n': 0}
        sub = rospy.Subscriber(MS + SP_COMPETITOR_TOPIC_SUFFIX, PositionTarget,
                               lambda _m: got.__setitem__('n', got['n'] + 1),
                               queue_size=50)
        rospy.sleep(window)
        try:
            sub.unregister()
        except Exception:
            pass
        return got['n'], got['n'] / max(1e-6, window)

    def check_setpoint_exclusive(self, timeout=20.0):
        """起飞前确认我们是唯一的 setpoint 发布者（铁律 4）。

        判据走**行为法**（live_setpoint_competitor），不用 ROS master 注册表：
        实测 `run_match_single.sh yolo` 已 SIGKILL 掉通信节点，但 master 注册表
        残留 >20s，若按注册表判就会白白拒飞。
        """
        t0 = rospy.get_time()
        n = 0
        while not rospy.is_shutdown():
            print("[yf] 独占检查：监听 %s 共 3s（这段时间我们不发任何设定点）…"
                  % (MS + SP_COMPETITOR_TOPIC_SUFFIX), flush=True)
            n, hz = self.live_setpoint_competitor(window=3.0)
            if n == 0:
                break
            if rospy.get_time() - t0 > timeout:
                break
            print("[yf] 检测到活的竞争流：3s 内 %d 条（≈%.0f Hz），继续等…"
                  % (n, hz), flush=True)
        if n > 0:
            print("[yf] ✖ 拒绝起飞：%s 上有活着的竞争流（3s 内 %d 条）。\n"
                  "      说明有别的节点在给我们这架 PX4 灌设定点，最常见是\n"
                  "      XTDrone 的 multirotor_communication.py —— 它无条件 30Hz\n"
                  "      广播局部原点 (0,0,0)，会把飞机拖走 ⇒ 坐标反复跳、最终出界。\n"
                  "      处理：跑 ./run_match_single.sh yolo（会先停掉竞争者），"
                  "或手工 pkill -f multirotor_communication.py"
                  % (MS + SP_COMPETITOR_TOPIC_SUFFIX, n), flush=True)
            return False
        bad = self.setpoint_competitors()
        if bad:
            print("[yf] 注意：ROS master 注册表仍列着 %s，但实测 3s 内零消息 ⇒ "
                  "判定为残留注册（进程已死），不阻断。" % (bad,), flush=True)
        print("[yf] 独占检查通过：%s 上无活的竞争流"
              % (MS + SP_COMPETITOR_TOPIC_SUFFIX), flush=True)
        return True

    def takeoff(self, alt, spawn_xy=None):
        """起飞爬升到 alt（水平位置全程锁死，绝不横向漂）。

        前置条件（硬门槛，2026-10-03 加）：飞机必须仍停在官方起飞点。
        不满足就拒绝起飞 —— 否则坐标标定会整体偏移 δ，飞机必然飞出场地。
        """
        if spawn_xy is None:
            spawn_xy = SPAWN_XY
        while not rospy.is_shutdown() and self.local is None:
            rospy.sleep(0.1)

        # --- 门槛 1：飞机必须还在起飞点（否则拒绝标定）---
        lxy = math.hypot(self.local.x, self.local.y)
        print("[yf] 起飞前自检：local=(%.2f,%.2f,%.2f) 水平模长=%.2fm 限值=%.2fm"
              % (self.local.x, self.local.y, self.local.z, lxy, SPAWN_TOL_XY),
              flush=True)
        if lxy > SPAWN_TOL_XY or abs(self.local.z) > SPAWN_TOL_Z:
            print("[yf] ✖ 拒绝起飞：飞机已不在官方起飞点%s。\n"
                  "      原因：坐标标定建立在「此刻飞机就在起飞点」之上。若飞机已被\n"
                  "      其它控制器带走 δ，则所有目标坐标会整体偏移 δ（可达 60m），\n"
                  "      飞机会飞到「目标+δ」处 ⇒ 每次必出界。\n"
                  "      处理：重启场景（./run_match_single.sh stop && NO_AGENT=1 "
                  "./run_match_single.sh start），让飞机回到起飞点再飞。"
                  % (spawn_xy,), flush=True)
            return False
        if self.state is not None and self.state.armed:
            print("[yf] ✖ 拒绝起飞：飞机已被解锁（arm）—— 说明它不是干净的地面状态，"
                  "可能正被别的控制器控制。请先重启场景。", flush=True)
            return False

        self.set_offset_from(spawn_xy[0], spawn_xy[1],
                             self.local.x, self.local.y)
        # 与"固定起飞点常量"对照：两者相差过大说明 EKF 原点漂了，打印出来备查
        print("[yf] 标定交叉核对：标定得 off=(%.2f,%.2f)；"
              "按官方固定起飞点应为 (%.2f,%.2f)。差值=(%.2f,%.2f)m"
              % (self._off[0], self._off[1], spawn_xy[0], spawn_xy[1],
                 self._off[0] - spawn_xy[0], self._off[1] - spawn_xy[1]),
              flush=True)
        # ⚠ 关键（2026-10-03 实测）：这里**不能**用"每帧只抬 0.15m"的慢斜坡。
        # 慢斜坡在 45s 里一直命令近乎零的爬升，PX4 的高度环/悬停推力估计会
        # 把这个"贴地"状态当基准，之后再命令 2.5m 它也不爬了（真值实测 125s
        # 停在 z≈0.4m；而直接给目标高度的手工探针 5s 就爬到 2.2m 并稳住）。
        # 所以从第一帧起就把设定点 z 直接给到目标高度。
        self.set_sp_world(spawn_xy[0], spawn_xy[1], alt, 0.0)
        rospy.sleep(1.0)
        # 先关掉失效保护（易失参数），否则 ARM/OFFBOARD 都是假动作
        self.set_params_flight()
        self.send("ARM")
        if not self.wait_armed():
            print("[yf] ARM 超时", flush=True)
            return False
        self.send("OFFBOARD")
        if not self.wait_mode("OFFBOARD"):
            print("[yf] OFFBOARD 超时", flush=True)
            return False
        print("[yf] OFFBOARD+解锁完成，爬升到 %.1fm" % alt, flush=True)
        t0 = rospy.get_time()
        while not rospy.is_shutdown():
            z = self.local.z
            # 始终命令目标高度（不随当前高度回退）
            self.set_sp_world(spawn_xy[0], spawn_xy[1], alt, 0.0)
            if z >= alt - 0.35:
                print("[yf] 已到 %.2fm（目标 %.1fm）" % (z, alt), flush=True)
                return True
            if rospy.get_time() - t0 > 60:
                print("[yf] ⚠ 爬升 60s 只到 %.2fm（目标 %.1fm），仍继续任务——"
                      "若长期贴地请检查 PX4 悬停推力估计/机体响应" % (z, alt),
                      flush=True)
                return True
            rospy.sleep(1.0 / SP_HZ)
        return False

    # ---------------- 目标选择 ----------------
    def nearest_target(self):
        """当前该追的目标。返回 (tid, 距离, 目标世界坐标) 或 (None, 1e9, None)。

        ⚠ 必须**锁定**，不能每帧重选最近的（2026-10-03 实测教训）：
            原来每帧调 nearest_target() 取最近目标，当两个目标距离相近时它会
            在两者之间反复翻转 ⇒ 飞机在两处之间来回移动，两边都待不满官方要求的
            「误差<1m 且间隔≤1s 连续 15s」，谁都消除不了（实测裁判日志里
            `find actor_3` 被重复打印 20+ 次却永远累不满 15s，分数卡在 95）。
        现在的语义：**锁定一个目标，直到它被官方消除 / 失联超时 / 数据不可信，
        才换下一个** —— 这同时天然满足用户要求的「消除一个后接着找下一个」。

        过滤规则（10-03 事故后加）：单目测距远距离误差可达 10m+，离谱的目标位置
        （出场地框 / 距机 >TARGET_MAX_RANGE / 数据过期）一律不追。
        """
        w = self.world()
        if w is None or not self.targets:
            self._lock = None
            return None, 1e9, None
        now = rospy.get_time()

        def _ok(tid):
            if tid not in self.targets:
                return None
            tx, ty = self.targets[tid]
            if now - self.t_seen.get(tid, 0.0) > TARGET_STALE:
                return None
            if not (FIELD_XMIN <= tx <= FIELD_XMAX and
                    FIELD_YMIN <= ty <= FIELD_YMAX):
                return None
            d = math.hypot(tx - w[0], ty - w[1])
            if d > TARGET_MAX_RANGE:
                return None
            return d

        # 1) 已锁定的目标还活着就一直追它
        if self._lock is not None:
            d = _ok(self._lock)
            if d is not None:
                self._lock_lost_t = now
                return self._lock, d, self.targets[self._lock]
            if now - self._lock_lost_t < LOCK_SWITCH_S:
                # 还没到换人的时限：原地等，别乱跑（乱跑会毁掉 15s 连续）
                return None, 1e9, None
            print("[yf] 锁定目标 %s 失联超 %.0fs → 换下一个" % (self._lock, LOCK_SWITCH_S),
                  flush=True)
            self._lock = None

        # 2) 没锁定 → 选最近的并锁上
        best, bd = None, 1e9
        for tid in list(self.targets):
            d = _ok(tid)
            if d is not None and d < bd:
                best, bd = tid, d
        if best is None:
            return None, 1e9, None
        if best != self._lock:
            actor = TID_ACTOR.get(int(best[1:]), '?') if best[1:].isdigit() else '?'
            print("[yf] 锁定目标 %s(actor_%s) 距离 %.1fm（追到它被消除为止）"
                  % (best, actor, bd), flush=True)
        self._lock = best
        self._lock_lost_t = now
        return best, bd, self.targets[best]

    # ---------------- 搜索（找不到人时）----------------
    def _scan_step(self, alt):
        """原地悬停 + 缓慢自转扫视；待够 SCAN_DWELL 秒后转下一个搜索点。

        全部搜索点都在场地安全框内，且转场速度 SEARCH_SPEED < 1 m/s
        （官方逃逸线），所以既不会飞出场地，也不会惊动路上的人。
        """
        w = self.world()
        if w is None:
            return
        now = rospy.get_time()
        if self._scan_mode == 'scan':
            # ⚠ 用**固定锚点**，不是当前位置。原版写 set_sp_world(w[0], w[1], ...)，
            # 设定点=当前位 ⇒ 位置环没有修正量，任何漂移都会累积（实测 40s 漂 6m）。
            if self._anchor is None:
                self._anchor = (w[0], w[1])
                print("[yf] 扫视锚点钉在 (%.1f,%.1f)" % self._anchor, flush=True)
            self._yaw += SCAN_YAW_STEP   # 缓慢自转，让前视相机扫一圈
            self.set_sp_world(self._anchor[0], self._anchor[1], alt, self._yaw)
            if self._scan_until is None:
                self._scan_until = now + SCAN_DWELL
            elif now >= self._scan_until:
                self._scan_mode = 'travel'
                self._scan_until = None
                self._anchor = None
                self._wp_i = (self._wp_i + 1) % len(SEARCH_PTS)
                gx, gy = SEARCH_PTS[self._wp_i]
                print("[yf] 扫完 → 转场到搜索点 %d (%.0f,%.0f)"
                      % (self._wp_i, gx, gy), flush=True)
            return
        # travel
        gx, gy = SEARCH_PTS[self._wp_i]
        gap = math.hypot(gx - w[0], gy - w[1])
        yaw = math.atan2(gy - w[1], gx - w[0])
        self.step_toward(gx, gy, alt, yaw, SEARCH_SPEED)
        if gap <= SEARCH_ARRIVE:
            self._scan_mode = 'scan'
            self._yaw = yaw
            print("[yf] 到达搜索点 %d，开始扫视" % self._wp_i, flush=True)

    # ---------------- 位置可信性看门狗 ----------------
    def _hold_here(self, alt):
        """原地悬停：钉住当前位置（锚点），机头保持当前朝向。

        与 _scan_step 的扫视分支共用 _anchor，但**不旋转**：
        追踪状态下旋转会丢掉目标，也会让前视相机离开人。
        """
        w = self.world()
        if w is None:
            return
        if self._anchor is None:
            self._anchor = (w[0], w[1])
        self.set_sp_world(self._anchor[0], self._anchor[1], alt, self._yaw)

    def pos_trusted(self, w):
        """位置估计是否可信。

        2026-10-03 血泪：PX4 的 spawn 失败、或 offboard 失联触发估计重置时，
        /mavros/local_position/pose 会在 ±150m 间乱跳（实测）。若不去检查，
        每一步方向都是随机的 ⇒ 飞机必然冲出场地。这里做两道判断：
          · 高度出现负值（钻到地面以下）⇒ 估计坏了
          · 位置离场地框超过 POS_SLACK，而飞机物理上不可能自己跑到那里
        """
        if w is None or w[2] < POS_BAD_Z:
            return False
        if not (FIELD_XMIN - POS_SLACK <= w[0] <= FIELD_XMAX + POS_SLACK and
                FIELD_YMIN - POS_SLACK <= w[1] <= FIELD_YMAX + POS_SLACK):
            return False
        return True

    # ---------------- 主循环：找人 → 靠近 → 悬停 → 下一个 ----------------
    def run(self, keep_m, alt, speed):
        t0 = rospy.get_time()
        bad_since = None
        while not rospy.is_shutdown():
            w = self.world()
            if w is None:
                rospy.sleep(0.1)
                continue
            # --- 看门狗：位置估计不可信 → 立刻停止控制，交给 PX4 失效保护降落 ---
            if not self.pos_trusted(w):
                if bad_since is None:
                    bad_since = rospy.get_time()
                    print("[yf] ⚠ 位置估计异常 world=(%.1f,%.1f,%.2f)，开始计时"
                          % w, flush=True)
                elif rospy.get_time() - bad_since > POS_BAD_LIMIT:
                    print("[yf] ✖ 位置估计连续 %.0fs 不可信 ⇒ 停止控制（飞机不可能"
                          "跑到那里）——检查 PX4 spawn 是否成功 / 是否有 offboard 失联"
                          % POS_BAD_LIMIT, flush=True)
                    self._stop = True
                    return
                rospy.sleep(1.0 / SP_HZ)
                continue
            bad_since = None
            if rospy.get_time() - t0 > MISSION_LIMIT:
                print("[yf] 到达任务时限 %.0fs，退出（PX4 失效保护会降落）"
                      % MISSION_LIMIT, flush=True)
                return
            # --- 位置指令被接受了吗（guided）---
            if self.state is not None and not self.state.guided:
                if rospy.get_time() - self._t_guided > 10.0:
                    self._t_guided = rospy.get_time()
                    self._n_guided_warn += 1
                    if self._n_guided_warn <= 2:
                        # 注：实测这一条在 PX4 上会常驻为 False（PX4 不下发
                        # guided 语义），但飞机确实在跟我们的位置设定点 ——
                        # 所以只提示两次，不当故障处理。
                        print("[yf] 提示：mavros 报 guided=False（PX4 不置该位）。"
                              "以真值/设定点跟随情况为准（mode=%s armed=%s）"
                              % (self.state.mode, self.state.armed), flush=True)

            tid, d, tgt = self.nearest_target()
            if tid is None:
                if self._lock is not None:
                    # 锁定目标短暂丢失（YOLO 遮挡/跟丢）：**原地悬停等它回来**。
                    # 绝不能转去搜索别处 —— 一飞走官方那条「误差<1m 连续 15s」
                    # 的计时立刻清零，前面追的距离全白费。
                    self._hold_here(alt)
                    if rospy.get_time() - self._t_log > 3.0:
                        self._t_log = rospy.get_time()
                        print("[yf] 锁定目标 %s 暂丢 → 原地悬停等待（不飞走）"
                              " 位置(%.1f,%.1f,%.2f)" % (self._lock, w[0], w[1], w[2]),
                              flush=True)
                else:
                    self._scan_step(alt)
                    if rospy.get_time() - self._t_log > 5.0:
                        self._t_log = rospy.get_time()
                        print("[yf] 搜索中 位置(%.1f,%.1f,%.2f) 已消除%d个 存活目标%d"
                              % (w[0], w[1], w[2], len(self.eliminated),
                                 len(self.targets)), flush=True)
                rospy.sleep(1.0 / SP_HZ)
                continue

            tx, ty = tgt
            self._anchor = None      # 进入追踪态，清掉扫视/悬停锚点
            # ---- 观察点锚定（2026-10-03 重写，见 OBSP_REFRESH_M 处注释）----
            # ⛔ 原版每帧用「目标 + keep_m × (本机−目标)方向」重算观察点，
            #    而方向里含本机当前位置 ⇒ 本机动一点观察点就跟着动 ⇒ 飞机永远
            #    在追一个移动的点、永远停不下来 ⇒ 相机姿态持续变化、YOLO 输出抖
            #    ⇒ 误差在 1m 线两侧反复穿越 ⇒ 官方"连续 15s"计时被反复清零。
            # ✅ 现在：观察点一旦确定就**钉死**，只有两种情况才重算 ——
            #    ① 目标自身走远超过 OBSP_REFRESH_M（不挪就跟不上/会被 12m 闸门挡）
            #    ② 本机被外力推离锚超过 OBSP_REFRESH_M（异常保护）
            #    其余时间飞机原地静止，只让机头（yaw）对住目标。
            obs = self._obsp
            if obs is not None and obs[4] != tid:
                obs = None                      # 换了目标 ⇒ 锚作废
            need_new = obs is None
            if obs is not None:
                ax, ay, atx, aty, _tid0 = obs
                if math.hypot(tx - atx, ty - aty) > OBSP_REFRESH_M:
                    need_new = True             # 目标走远
                elif math.hypot(w[0] - ax, w[1] - ay) > OBSP_REFRESH_M:
                    need_new = True             # 本机被推离
            if need_new:
                ux, uy = w[0] - tx, w[1] - ty
                n = math.hypot(ux, uy) or 1e-3
                ax, ay = tx + keep_m * ux / n, ty + keep_m * uy / n
                self._obsp = (ax, ay, tx, ty, tid)
            # 机头死区：目标小幅移动不重算 yaw，抑制画面无谓旋转
            yaw_want = math.atan2(ty - w[1], tx - w[0])
            dyaw = yaw_want - self._yaw
            while dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            while dyaw < -math.pi:
                dyaw += 2.0 * math.pi
            if abs(dyaw) > YAW_DEADBAND_RAD:
                self._yaw = yaw_want
            # 20m 内必须 <1m/s（官方逃逸线）
            spd = ESCAPE_SAFE_SPEED if d < ESCAPE_RANGE else min(speed, 2.5)
            self.step_toward(ax, ay, alt, self._yaw, spd)

            if rospy.get_time() - self._t_log > 3.0:
                self._t_log = rospy.get_time()
                actor = TID_ACTOR.get(int(tid[1:]), '?') if tid[1:].isdigit() else '?'
                tag = "到达观察位" if d <= keep_m + 1.0 else "接近中"
                print("[yf] %s 目标=%s(actor_%s) 距离%.1fm 期望%.0fm "
                      "速度%.1fm/s 位置(%.1f,%.1f,%.2f) 存活%d"
                      % (tag, tid, actor, d, keep_m, spd, w[0], w[1], w[2],
                         len(self.targets)), flush=True)
            rospy.sleep(1.0 / SP_HZ)


def main():
    argv = sys.argv[1:]
    keep_m = KEEP_M_DEFAULT
    alt = ALT_DEFAULT
    speed = ESCAPE_SAFE_SPEED
    for a in argv:
        if a.startswith('--keep='):
            keep_m = float(a.split('=', 1)[1])
        elif a.startswith('--alt='):
            alt = float(a.split('=', 1)[1])
        elif a.startswith('--speed='):
            speed = float(a.split('=', 1)[1])
    alt = min(alt, ALT_MAX_M)
    rospy.init_node('yolo_flyer', anonymous=True)
    f = YoloFlyer()
    print("[yf] 启动：目标来源=/swarm/target_states（YOLO 合法链路）", flush=True)
    print("[yf] 机号 %s(=%d) 起飞点 world=(%.1f,%.1f)  "
          "【官方 control_actor.py:55-57 uav_takeoff_points】"
          % (UAV, UAV_ID, SPAWN_XY[0], SPAWN_XY[1]), flush=True)
    print("[yf] 场地框 x[%.0f,%.0f] y[%.0f,%.0f] 速度上限 %.1fm/s "
          "(官方逃逸线 1.0m/s@20m)"
          % (FIELD_XMIN, FIELD_XMAX, FIELD_YMIN, FIELD_YMAX, ESCAPE_SAFE_SPEED),
          flush=True)
    # 铁律 4：独占 setpoint 通道。竞争者存在就拒飞，不冒险。
    if not f.check_setpoint_exclusive():
        f._stop = True
        return
    _pts = order_search_pts(SPAWN_XY)
    print("[yf] 搜索点最近邻重排（从起飞点出发）：%s"
          % ' '.join('(%.0f,%.0f)' % p for p in _pts[:5]), flush=True)
    if not f.takeoff(alt):
        return
    f.run(keep_m, alt, speed)
    print("[yf] 结束", flush=True)


if __name__ == '__main__':
    main()
