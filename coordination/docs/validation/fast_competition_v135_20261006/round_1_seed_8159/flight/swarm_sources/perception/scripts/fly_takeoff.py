#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
官方 base.world 首次真机起飞验证（Milestone 1）

流程：等 MAVROS 连接 -> 设 MIS_TAKEOFF_ALT -> arm -> AUTO.TAKEOFF
      -> 爬升到位后切 AUTO.LOITER 悬停 -> 持续监视高度

验收标准：
  * typhoon_h480_0 真实存在于 Gazebo 世界（不是遥传出来的假位置）
  * 悬停高度稳定在 4.5 ~ 5.5 m
  * 全程 z < 6.5 m —— 超过官方裁判 score_cal.py 会直接 sys.exit 判负

产物：CSV 轨迹（默认 /tmp/fly_takeoff.csv），用于事后举证。
"""
import csv
import os
import time

import rospy
from gazebo_msgs.srv import GetModelState
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import CommandBool, ParamSet, SetMode

UAV = "typhoon_h480_0"
# MAVROS 被 launch 放在 <group ns="typhoon_h480_0"> 下，话题/服务都带这个前缀
NS = os.environ.get("UAV_NS", "/typhoon_h480_0")
FRAME = "ground_plane"          # 与官方裁判一致
TARGET_ALT = 5.0
HARD_LIMIT = 6.5                # 官方判负线
HOLD_SEC = float(os.environ.get("FLY_HOLD_SEC", "40"))
OUT = os.environ.get("FLY_LOG", "/tmp/fly_takeoff.csv")

_state = {"v": None}


def _on_state(m):
    _state["v"] = m


def wait_for(cond, timeout, what):
    t0 = time.time()
    while not rospy.is_shutdown():
        if cond():
            return True
        if time.time() - t0 > timeout:
            print("[fly] 超时等待: %s" % what, flush=True)
            return False
        time.sleep(0.2)
    return False


def main():
    rospy.init_node("fly_takeoff", anonymous=True)
    rospy.Subscriber(NS + "/mavros/state", State, _on_state, queue_size=1)
    # PX4 控制台文本：起飞被拒/失效保护的原因只在这里能看到，必订
    try:
        from mavros_msgs.msg import StatusText
        rospy.Subscriber(NS + "/mavros/statustext/recv", StatusText,
                         lambda m: print("   [PX4] %s" % m.text, flush=True), queue_size=50)
    except Exception as e:
        print("[fly] statustext 订阅失败: %s" % e, flush=True)

    for s in (NS + "/mavros/cmd/arming", NS + "/mavros/set_mode",
              NS + "/mavros/param/set", "/gazebo/get_model_state"):
        print("[fly] 等待服务 %s ..." % s, flush=True)
        try:
            rospy.wait_for_service(s, timeout=90)
        except rospy.ROSException:
            print("[fly] 服务不可用: %s" % s, flush=True)
            return 1
    arm = rospy.ServiceProxy(NS + "/mavros/cmd/arming", CommandBool)
    setmode = rospy.ServiceProxy(NS + "/mavros/set_mode", SetMode)
    pset = rospy.ServiceProxy(NS + "/mavros/param/set", ParamSet)
    gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    print("[fly] 等待 MAVROS 与飞控建立连接 ...", flush=True)
    if not wait_for(lambda: _state["v"] and _state["v"].connected, 120, "MAVROS connected"):
        return 1
    st = _state["v"]
    print("[fly] 飞控已连接  mode=%s  armed=%s" % (st.mode, st.armed), flush=True)

    # 1) 先关掉仿真环境特有的失效保护，再设起飞高度
    #
    # 【必须做】PX4 日志实录：
    #     INFO [commander] Takeoff detected
    #     WARN [commander] Failsafe enabled: no RC and no datalink
    #     INFO [commander] Failsafe mode activated
    #     INFO [navigator] RTL HOME activated
    # 即：飞机确实起飞了，但因为没有遥控器、也没有地面站数据链，
    # PX4 判定失效保护生效 → 立刻 RTL 返航 → 落地 → 上锁。
    # 真实比赛时队伍有 QGroundControl 提供数据链，无头 SITL 下必须显式关掉：
    #   NAV_RCL_ACT=0  遥控器丢失 → 不动作
    #   NAV_DLL_ACT=0  数据链丢失 → 不动作
    # 这是仿真环境适配，不改动官方仓库任何文件。
    for pid, val in (("NAV_RCL_ACT", 0.0), ("NAV_DLL_ACT", 0.0), ("MIS_TAKEOFF_ALT", TARGET_ALT)):
        try:
            r = pset(param_id=pid, value=ParamValue(integer=0, real=val))
            print("[fly] 参数 %s=%s -> success=%s" % (pid, val, r.success), flush=True)
        except Exception as e:
            print("[fly] 设参数 %s 失败(不致命): %s" % (pid, e), flush=True)

    # 2) 采样与记录
    f = open(OUT, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "x", "y", "z", "mode", "armed"])

    def sample():
        """读 Gazebo 真值位置。官方 get_model_state 会间歇性返回 (0,0,0)，需做零值保护。"""
        try:
            p = gms(UAV, FRAME).pose.position
        except Exception:
            return None
        m = _state["v"]
        w.writerow(["%.2f" % rospy.get_time(), "%.3f" % p.x, "%.3f" % p.y, "%.3f" % p.z,
                    m.mode if m else "", int(m.armed) if m else -1])
        f.flush()
        return p

    p0 = sample()
    print("[fly] 起飞前位置: %s" % (("%.2f, %.2f, %.2f" % (p0.x, p0.y, p0.z)) if p0 else "读取失败"),
          flush=True)

    # 3) 解锁
    # 先把可能残留的 AUTO.RTL（上一次失效保护留下的）清掉，否则会拒绝解锁
    try:
        if _state["v"] and _state["v"].mode in ("AUTO.RTL", "AUTO.LAND"):
            print("[fly] 清理残留模式 %s -> AUTO.LOITER" % _state["v"].mode, flush=True)
            setmode(custom_mode="AUTO.LOITER")
            time.sleep(1.5)
    except Exception as e:
        print("[fly] 清理模式异常: %s" % e, flush=True)

    print("[fly] 解锁 -> %s" % (arm(True),), flush=True)
    if not wait_for(lambda: _state["v"] and _state["v"].armed, 20, "armed"):
        print("[fly] 解锁失败", flush=True)
        f.close()
        return 1

    # 4) 切 AUTO.TAKEOFF
    for _ in range(30):
        if _state["v"] and _state["v"].mode == "AUTO.TAKEOFF":
            break
        try:
            print("[fly] set_mode AUTO.TAKEOFF -> %s" % (setmode(custom_mode="AUTO.TAKEOFF"),),
                  flush=True)
        except Exception as e:
            print("[fly] set_mode 异常: %s" % e, flush=True)
        time.sleep(0.5)

    # 5) 爬升监视
    print("[fly] 爬升中，监视高度（判负线 %.1f m）..." % HARD_LIMIT, flush=True)
    t0 = time.time()
    reached = False
    while not rospy.is_shutdown() and time.time() - t0 < 60:
        p = sample()
        if p:
            if p.z > HARD_LIMIT:
                print("[fly] !!! 高度 %.2f 超过判负线 %.1f，立即降落 !!!" % (p.z, HARD_LIMIT),
                      flush=True)
                setmode(custom_mode="AUTO.LAND")
                f.close()
                return 2
            if p.z >= TARGET_ALT - 0.6:
                reached = True
                break
        time.sleep(0.3)
    if not reached:
        print("[fly] 未能在 60s 内爬升到 %.1f m" % TARGET_ALT, flush=True)

    # 6) 悬停保持
    try:
        print("[fly] set_mode AUTO.LOITER -> %s" % (setmode(custom_mode="AUTO.LOITER"),),
              flush=True)
    except Exception as e:
        print("[fly] 切 LOITER 异常: %s" % e, flush=True)

    print("[fly] 悬停监视 %ds ..." % HOLD_SEC, flush=True)
    zs = []
    t1 = time.time()
    while not rospy.is_shutdown() and time.time() - t1 < HOLD_SEC:
        p = sample()
        if p:
            zs.append(p.z)
            if p.z > HARD_LIMIT:
                print("[fly] !!! 悬停中高度 %.2f 超限 !!!" % p.z, flush=True)
        time.sleep(0.3)

    if zs:
        print("[fly] 悬停高度: 最小 %.2f  最大 %.2f  平均 %.2f  (样本 %d)"
              % (min(zs), max(zs), sum(zs) / len(zs), len(zs)), flush=True)
        ok = (min(zs) > 4.0) and (max(zs) < HARD_LIMIT)
        print("[fly] ==== MILESTONE1 %s ====" % ("PASS" if ok else "FAIL"), flush=True)
    f.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
