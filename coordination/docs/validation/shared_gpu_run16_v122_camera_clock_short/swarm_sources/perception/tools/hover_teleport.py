#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瞬移悬停观测台 —— 把「飞行」从感知验证里剥离出去。

为什么需要它：
  真实飞行追 actor 有三次必栽的坑（EKF 漂移 / 撞楼掉地 / actor 逃跑追不上），
  每一次都要整套重启 ~2 分钟。而我们要量化的是**感知准确率**，
  飞行能力是 M4 的课题。两件事混在一起，感知参数就永远调不快。

做法：每 100 ms 用 set_model_state 把无人机钉在「目标 12 m 外、高 5.5 m、
机头对准目标」的观测点上（twist 归零）。actor 仍在以 ~1 m/s 真实游走，
所以这是**动态目标**验证，不是静态替身。相机位姿取 Gazebo 真值，与飞控无关。

观测方位不是随便选的：读官方 black_box.txt 的 43 个建筑矩形，
在 16 个候选方位里挑净空最大的，避免把观测点塞进楼里。

用法:  hover_teleport.py <actor_id|auto> [keep=12] [height=5.5] [dur=60]
"""
import json
import math
import os
import sys

import rospy
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped

UAV_MODEL = os.environ.get("HT_MODEL", "typhoon_h480_0")
BLACK_BOX = os.environ.get("HT_BLACK_BOX",
                           os.path.expanduser("~/XTDrone/robocup/black_box.txt"))
HT_STATUS = os.environ.get("HT_STATUS", "/tmp/ht_status.json")
N_ACTOR = 6


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_of(q):
    """四元数 -> 偏航角（与 perception_real 的 yaw_cam 同一算法，保证可比）"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_boxes(path):
    """官方 black_box.txt。

    坑：这个文件是**单行**的（wc -l 得 0，末尾没有换行），
    整行是一个列表，含 43 个 [[x0,x1],[y0,y1]]。
    按行解析 + 直接取 v[0]/v[1] 会只拿到 1 个矩形且类型全错。
    """
    boxes = []
    try:
        with open(path) as f:
            raw = f.read()
        data = eval(raw.strip())
        # 两种可能：单个 box [[a,b],[c,d]]，或 box 列表
        items = data if isinstance(data[0][0], list) else [data]
        for b in items:
            (x0, x1), (y0, y1) = b[0], b[1]
            boxes.append((min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)))
    except Exception as e:
        print("[ht] 读 black_box 失败(%s)，退化为固定方位" % e)
    return boxes


def clearance(x, y, boxes):
    """到最近建筑的距离（在建筑内返回负值）"""
    best = 1e9
    for (x0, x1, y0, y1) in boxes:
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        d = math.hypot(dx, dy)
        if dx == 0.0 and dy == 0.0:                 # 内部
            d = -min(x - x0, x1 - x, y - y0, y1 - y)
        best = min(best, d)
    return best if boxes else 1e9


def pick_bearing(ax, ay, keep, boxes):
    """在 16 个方位里选观测点净空最大的"""
    best, bestc = 0.0, -1e9
    for k in range(16):
        th = 2.0 * math.pi * k / 16.0
        ox, oy = ax + keep * math.cos(th), ay + keep * math.sin(th)
        c = clearance(ox, oy, boxes)
        if c > bestc:
            bestc, best = c, th
    return best, bestc


# ---------------------------------------------------------------------------
# 观测方位不能"开局选一次就锁死"（2026-09-15 修）
#
# 病因（blue 轮实测）：`pick_bearing` 只按**观测点净空**选方位，而且只在开局选一次。
# actor 以 ~1 m/s 游走后：① 观测点可能移进楼里；② 更常见的是**连线被楼挡住**
# ——人走到房子后面，整条流静默 21 s（blue 命中仅 56.1%，误差中位 0.77 m 却 FAIL）。
# 这和 pair_walk 踩过的"坑 4"是同一个病：只看目标点净空、不看视线。
#
# 修法：每 0.5 s 评估当前方位是否"视线通 + 净空够"，不可用才换到最优方位；
# 换位带**冷却**（免得抖动打断 track）与**迟滞**（新方位必须明显更好才动）。
# ⚠ 这是验证台的改动，不是感知的改动 —— 它只影响"无人机站哪"。
# ---------------------------------------------------------------------------
LOS_STEP = 0.5        # 视线采样步长（m）
LOS_MIN_CLEAR = 1.0   # 观测点自身最小净空（m）
SWITCH_COOL = 4.0     # 换位冷却（s）
SWITCH_MARGIN = 1.0   # 迟滞：新方位要比当前好这么多才换（m）


def los_ok(x0, y0, x1, y1, boxes, step=LOS_STEP):
    """观测点 -> 目标 的连线是否全程不穿建筑。"""
    if not boxes:
        return True
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
    for k in range(n + 1):
        t = float(k) / n
        if clearance(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, boxes) < 0.0:
            return False
    return True


def bearing_state(ax, ay, keep, boxes, th):
    """某方位是否可用，返回 (可用, 打分, 观测点ox, 观测点oy, 净空)。

    可用 = 观测点净空 >= LOS_MIN_CLEAR **且** 连线不穿建筑。
    """
    ox, oy = ax + keep * math.cos(th), ay + keep * math.sin(th)
    c = clearance(ox, oy, boxes)
    if c < LOS_MIN_CLEAR:
        return False, -1e9, ox, oy, c
    if not los_ok(ox, oy, ax, ay, boxes):
        return False, -1e9, ox, oy, c
    return True, c, ox, oy, c


def best_bearing(ax, ay, keep, boxes, prefer=None, margin=0.0):
    """选方位：可用者中打分最高。

    prefer 给了就带迟滞 —— 除非新方位比 prefer 好出 margin，否则保持 prefer。
    """
    best, bs = None, -1e9
    for k in range(16):
        th = 2.0 * math.pi * k / 16.0
        ok, s, _, _, _ = bearing_state(ax, ay, keep, boxes, th)
        if ok and s > bs:
            bs, best = s, th
    if best is None:
        return None, -1e9
    if prefer is not None:
        ok, ps, _, _, _ = bearing_state(ax, ay, keep, boxes, prefer)
        if ok and ps + margin >= bs:
            return prefer, ps
    return best, bs


def main():
    args = [a for a in sys.argv[1:]]
    target = args[0] if args else "auto"
    keep = float(args[1]) if len(args) > 1 else 12.0
    height = float(args[2]) if len(args) > 2 else 5.5
    dur = float(args[3]) if len(args) > 3 else 60.0

    rospy.init_node("hover_teleport", anonymous=True)
    boxes = load_boxes(BLACK_BOX)
    print("[ht] 建筑矩形 %d 个 (来自 %s)" % (len(boxes), BLACK_BOX), flush=True)

    pos = {}
    for i in range(N_ACTOR):
        pos[i] = None

    def mk(i):
        def cb(m):
            pos[i] = (m.pose.position.x, m.pose.position.y)
        return cb

    for i in range(N_ACTOR):
        rospy.Subscriber("/actor_%d/pose" % i, PoseStamped, mk(i), queue_size=1)

    rospy.sleep(3.0)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=30)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=30)
    setms = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    getms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    # ------------------------------------------------------------------
    # 实测回读（2026-09-15 修）——本轮最大的坑就在这
    #
    # 旧版只在日志里打印 `实距 = |目标 - 观测点|`，那是**纯算术恒等式**，
    # 恒等于 keep（13.00m），跟无人机在哪毫无关系。于是"瞬移台工作正常"
    # 是一句没有证据的断言：brown 轮实测无人机前 55 s 一直趴在起飞点
    # （53.9, 2.2），离目标 75 m，而日志全程打印"实距=13.00m 视线=通"。
    #
    # 修法：每 1 s 用 get_model_state 读回**无人机真实位置**，与观测点比较；
    # 并在首次真正到位（<2m）时打一行 `到位`，供外层脚本 grep 门控。
    # ------------------------------------------------------------------
    meas = [None, None, None]
    last_meas = -1e9
    arrived = None
    n_set_reject = 0
    # 自己累计"偏离观测点"的统计 —— 比外部 1 Hz 采样可靠：
    # 采样会漏掉短时的抢位（实测 t=95s 有一次 23.5 m 的短暂脱位，
    # 看门狗按 1 s 采样恰好错过，而瞬移台自己每秒都在量，不会漏）。
    meas_n = 0
    bad_cnt = 0
    off_max = 0.0
    cur_bad = 0.0
    long_bad = 0.0
    # ------------------------------------------------------------------
    # 姿态回读（2026-09-15 加）—— 和"位置回读"同一课：验证台必须回读自己的效果
    #
    # 踩到的坑：位置回读显示偏差 0.01 m（位置钉得非常牢），但**相机世界偏航在漂**。
    # D 轮实测 t=0→27 s：指令偏航恒定（-111.8°），相机实际偏航却从 -117.0°
    # 单调漂到 -141.4°，即目标离轴角 +5.5° → +29.9°，约 **0.9°/s 匀速漂移**；
    # 再看 t=43~73 s，离轴角出现 ±170° 的失控（相机基本没对着目标）。
    # 后果：目标被推到画面边缘甚至推出画面 -> 检测断流 -> 就是那些"长静默段"。
    # 所以每 1 s 同时回读**无人机本体偏航**，与"本 tick 期望的偏航"比，
    # 得到瞄准误差 aim_deg（度），并把最大值写进状态文件供外层门控。
    # ------------------------------------------------------------------
    aim_deg = float("nan")
    aim_max = 0.0
    aim_bad_cnt = 0
    AIM_BAD = 15.0            # 离轴超过这么多度就算"没瞄准"（半视场 57°）

    try:
        cur = getms(UAV_MODEL, "world").pose.position
        ux, uy = cur.x, cur.y
    except Exception:
        ux, uy = 0.0, 0.0

    # 选目标
    if target == "auto":
        best, bd = 0, 1e9
        for i in range(N_ACTOR):
            if pos[i] is None:
                continue
            d = math.hypot(pos[i][0] - ux, pos[i][1] - uy)
            if d < bd:
                best, bd = i, d
        target = best
        print("[ht] auto -> actor_%d (距无人机 %.1f m)" % (target, bd), flush=True)
    aid = int(target)

    if aid not in pos or pos[aid] is None:
        print("[ht] actor_%d 无位姿" % aid)
        return
    ax, ay = pos[aid]
    bearing, bscore = best_bearing(ax, ay, keep, boxes)
    if bearing is None:
        bearing, _ = pick_bearing(ax, ay, keep, boxes)     # 兜底：16 个方位全不可用
        print("[ht] ⚠ 16 个方位全不可用（视线全被挡 / 净空不足），退回纯净空选择", flush=True)
    _ok, _bs, _ox, _oy, clr = bearing_state(ax, ay, keep, boxes, bearing)
    print("[ht] 目标 actor_%d @ (%.1f,%.1f)  观测距离=%.1fm 高度=%.1fm "
          "方位=%.0f° 净空=%.1fm 视线=%s"
          % (aid, ax, ay, keep, height, math.degrees(bearing), clr,
             "通" if _ok else "挡"), flush=True)
    try:
        _p0 = getms(UAV_MODEL, "world").pose.position
        print("[ht] 起飞前无人机实测 = (%.2f, %.2f, %.2f)  离目标 %.1f m"
              % (_p0.x, _p0.y, _p0.z, math.hypot(_p0.x - ax, _p0.y - ay)), flush=True)
    except Exception as e:
        print("[ht] 读不到无人机位置: %s" % e, flush=True)

    t0 = rospy.get_time()
    last = 0.0
    last_eval = -1e9
    last_switch = -1e9
    n_switch = 0
    ox = oy = 0.0
    n = 0
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        el = rospy.get_time() - t0
        if el > dur:
            break
        if pos[aid] is not None:
            ax, ay = pos[aid]

            # --- 每 0.5 s 评估一次方位：视线被挡 / 净空不足就换位（带冷却，免得打断 track）---
            if el - last_eval >= 0.5:
                last_eval = el
                cur_ok = bearing_state(ax, ay, keep, boxes, bearing)[0]
                if not cur_ok:
                    nb, ns = best_bearing(ax, ay, keep, boxes,
                                          prefer=bearing, margin=SWITCH_MARGIN)
                    if nb is not None and nb != bearing and el - last_switch >= SWITCH_COOL:
                        print("[ht] 换位 %.0f° -> %.0f°  t=%.1fs  原因=当前方位视线被挡/净空不足"
                              % (math.degrees(bearing), math.degrees(nb), el), flush=True)
                        bearing, last_switch, n_switch = nb, el, n_switch + 1
                    elif nb is None and el - last_switch >= SWITCH_COOL:
                        last_switch = el
                        print("[ht] ⚠ t=%.1fs 无可用方位（16 个全被挡），暂不换位" % el,
                              flush=True)

            ox = ax + keep * math.cos(bearing)
            oy = ay + keep * math.sin(bearing)
            yaw = math.atan2(ay - oy, ax - ox)

            st = ModelState()
            st.model_name = UAV_MODEL
            st.pose.position.x = ox
            st.pose.position.y = oy
            st.pose.position.z = height
            st.pose.orientation.x = 0.0
            st.pose.orientation.y = 0.0
            st.pose.orientation.z = math.sin(yaw / 2.0)
            st.pose.orientation.w = math.cos(yaw / 2.0)
            st.twist.linear.x = 0.0
            st.twist.linear.y = 0.0
            st.twist.linear.z = 0.0
            st.twist.angular.x = 0.0
            st.twist.angular.y = 0.0
            st.twist.angular.z = 0.0
            st.reference_frame = "world"
            try:
                rp = setms(st)
                # 服务调用成功 ≠ 生效：模型名写错时 Gazebo 只回 success=False
                if not getattr(rp, "success", True):
                    n_set_reject += 1
                    if n_set_reject <= 3:
                        print("[ht] ⚠ set_model_state 被拒: %s"
                              % getattr(rp, "status_message", ""), flush=True)
            except Exception as e:
                print("[ht] set_model_state 失败: %s" % e, flush=True)
            n += 1

            # --- 读回真实位置（每秒一次）---
            if el - last_meas >= 1.0:
                last_meas = el
                try:
                    mp = getms(UAV_MODEL, "world").pose
                    meas[0], meas[1], meas[2] = (mp.position.x, mp.position.y,
                                                 mp.position.z)
                    aim_deg = math.degrees(abs(wrap_pi(yaw - yaw_of(mp.orientation))))
                except Exception:
                    pass
                if meas[0] is not None:
                    off = math.hypot(meas[0] - ox, meas[1] - oy)
                    meas_n += 1
                    off_max = max(off_max, off)
                    if aim_deg == aim_deg:                    # 不是 nan
                        aim_max = max(aim_max, aim_deg)
                        if aim_deg > AIM_BAD:
                            aim_bad_cnt += 1
                    if off > 2.0:
                        bad_cnt += 1
                        cur_bad += 1.0
                        long_bad = max(long_bad, cur_bad)
                    else:
                        cur_bad = 0.0
                    if arrived is None and off <= 2.0:
                        arrived = el
                        print("[ht] 到位  t=%.1fs  实测=(%.2f,%.2f,%.2f)  偏差=%.2fm"
                              % (el, meas[0], meas[1], meas[2], off), flush=True)
                    # 机器可读状态：外层脚本据此**门控**，不再靠日志里的算术恒等式
                    try:
                        with open(HT_STATUS, "w") as sf:
                            json.dump({"t": round(el, 2), "target": aid,
                                       "intended": [round(ox, 2), round(oy, 2)],
                                       "measured": [round(meas[0], 2), round(meas[1], 2),
                                                    round(meas[2], 2)],
                                       "off": round(off, 2),
                                       "arrived": arrived is not None,
                                       "n_switch": n_switch,
                                       "n_set_reject": n_set_reject,
                                       "meas_n": meas_n,
                                       "bad_cnt": bad_cnt,
                                       "off_max": round(off_max, 2),
                                       "long_bad_s": round(long_bad, 1),
                                       "aim_deg": (None if aim_deg != aim_deg
                                                   else round(aim_deg, 1)),
                                       "aim_max": round(aim_max, 1),
                                       "aim_bad_cnt": aim_bad_cnt}, sf)
                    except Exception:
                        pass

        if el - last >= 5.0:
            last = el
            d = math.hypot(ax - ox, ay - oy) if pos[aid] else 0.0
            los = los_ok(ox, oy, ax, ay, boxes)
            if meas[0] is None:
                ms, off = "(读不到)", float("nan")
            else:
                ms = "(%.1f,%.1f,%.1f)" % (meas[0], meas[1], meas[2])
                off = math.hypot(meas[0] - ox, meas[1] - oy)
            print("[ht] t=%5.1f  actor=(%7.2f,%7.2f)  观测点=(%7.2f,%7.2f)  实距=%.2fm  "
                  "方位=%3.0f° 视线=%s 换位=%d | 无人机实测=%s 偏差=%.1fm 瞄准误差=%.1f° %s"
                  % (el, ax, ay, ox, oy, d, math.degrees(bearing),
                     "通" if los else "挡", n_switch, ms, off,
                     aim_deg if aim_deg == aim_deg else -1.0,
                     "" if off <= 2.0 or off != off else "⚠ 不在观测点"),
                  flush=True)
        rate.sleep()

    print("[ht] TELEPORT_DONE  共下发 %d 次  换位 %d 次" % (n, n_switch), flush=True)


if __name__ == "__main__":
    main()
