#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
M2 真机找人 —— 飞行 + 感知闭环验证。

流程：起飞 -> 分段逼近 actor 的观测位 -> 悬停跟随 N 秒（期间采集 perception_real
的上报并与 actor 真值比对误差）-> 输出统计 -> 继续驻留（**永不停止发布 setpoint**）。

要点：
  1. actor 位置来自 /actor_N/pose（ros_actor_cmd_pose_plugin 发布的仿真真值）。
     这是当前唯一可靠来源：get_model_state 对 Actor 恒返回 (0,0,1.02)，
     get_link_state('::Hips') 读到的是陈旧骨骼值。
  2. 分段小步飞行。此前一次性发远距离 setpoint 会得到 roll≈177°/pitch≈49° 的姿态翻转，
     相机朝天。每段 ≤ SEG 米、段间重读 actor 位置并重算，机体保持接近水平，
     水平前视相机才能持续看到地面目标。
  3. 全程用 rospy.get_time()（仿真时间）计时。本机 RTF≈0.38，用墙钟计时会误判"飞了一半"。
  4. 观察点 = actor + keep_m * unit(uav - actor)：沿当前方位退到指定距离，
     机头 yaw 对准 actor，使光轴（机头 +X，水平前视）正对目标。
  5. **setpoint 由独立线程以 20Hz 持续发布**（本脚本的核心改动）。
     此前一次 FLY_DONE 后主线程退出 -> setpoint 停发 -> 超过 COM_OF_LOSS_T=1.0s
     -> PX4 OFFBOARD 失效保护 -> 降落并锁死成 FLIGHT_TERMINATION(system_status=8)，
     必须重启 PX4 才能恢复。改用独立线程后，主线程无论阻塞多久都不会断流。
  6. OFFBOARD 切换前必须先有 setpoint 流，否则 PX4 拒绝切模式。

用法：
  python3 fly_to_actor.py <actor_id|auto> [keep_m] [alt_m] [--hold SEC] [--stay SEC]
                          [--conf F] [--rmin M] [--rmax M]
  例：python3 fly_to_actor.py auto 20 5.0 --hold 30 --stay 60
"""
import json
import math
import os
import sys
import threading

import rospy
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import CommandBool, ParamSet, SetMode
from sensor_msgs.msg import Image
from std_msgs.msg import String

NS = "/xtdrone/typhoon_h480_0"      # ⚠ 本机 uav_only.launch 里没有 XTDrone 桥接节点
MS = "/typhoon_h480_0/mavros"       # mavros 真实命名空间 —— 指令/参数/setpoint 都走这里
UAV = "typhoon_h480_0"
SEG = 9.0          # 每段最大位移（米）
SEG_DUR = 2.4      # 每段保持的仿真秒数（约 3.7 m/s，需快于 actor 的 1 m/s 才能追上）
SP_HZ = 20.0

# actor id -> 官方 score_cal.py 颜色映射
ID2CLS = {0: "green", 1: "blue", 2: "brown", 3: "white", 4: "red", 5: "red"}
FRAME_DIR = os.path.expanduser("~/robocup_real/_m2_frames")


class FlyToActor(object):
    def __init__(self):
        rospy.init_node("fly_to_actor", anonymous=True)
        self.actor = {}
        self.local = None
        self.dets = []
        self.det_lock = threading.Lock()
        self.frames = []
        self.sp = None                 # 当前 setpoint 目标
        self.sp_lock = threading.Lock()
        self.last_img = None
        self.bridge = CvBridge()

        # ⚠ 2026-09-20 根因修复：原实现把 "ARM"/"OFFBOARD" 发到 NS+"/cmd"、setpoint 发到
        # NS+"/cmd_pose_enu"，但本机 ROS 图里**没有任何节点订阅这两条话题**
        # （rostopic info 直接报 Unknown topic）⇒ 指令全部落空 ⇒ PX4 静默不动。
        # 这正是 09-19「逼近失败、无人机悬停不动」的真因，**与 RTF 无关**。
        # 改为直接使用 mavros 原生接口（与已 PASS 的 fly_takeoff.py 同一套）。
        self.pub_pose = rospy.Publisher(MS + "/setpoint_position/local",
                                        PoseStamped, queue_size=1)
        self.state = None
        rospy.Subscriber(MS + "/state", State, self._state, queue_size=1)
        for i in range(6):
            rospy.Subscriber("/actor_%d/pose" % i, PoseStamped,
                             lambda m, k=i: self._actor(k, m), queue_size=1)
        rospy.Subscriber("/%s/mavros/local_position/pose" % UAV, PoseStamped,
                         self._local, queue_size=1)
        # 调试快照话题（含 dets 列表）。⚠ /coordination/target_report 自 2026-09-20
        # 起改为协同契约（每目标一条、无 dets 字段），本回调的解析方式只适用于快照话题。
        rospy.Subscriber(os.environ.get("PR_DEBUG_TOPIC", "/perception/debug_snapshot"),
                         String, self._report, queue_size=1)
        rospy.Subscriber("/%s/cgo3_camera/image_raw" % UAV, Image, self._img, queue_size=1)
        rospy.wait_for_service("/gazebo/get_model_state", timeout=90)
        self.gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        for _sn, _st in (("cmd/arming", CommandBool), ("set_mode", SetMode),
                         ("param/set", ParamSet)):
            rospy.wait_for_service(MS + "/" + _sn, timeout=60)
        self.srv_arm = rospy.ServiceProxy(MS + "/cmd/arming", CommandBool)
        self.srv_mode = rospy.ServiceProxy(MS + "/set_mode", SetMode)
        self.srv_param = rospy.ServiceProxy(MS + "/param/set", ParamSet)
        print("[m2fly] mavros 服务就绪: %s" % MS, flush=True)
        t0 = rospy.get_time()
        while self.local is None and rospy.get_time() - t0 < 30:
            rospy.sleep(0.2)

        # setpoint 心跳线程：只要设过目标就一直发，主线程阻塞也不会断流
        self._stop = False
        threading.Thread(target=self._sp_loop, daemon=True).start()

    # ---------------- 回调 ----------------
    def _state(self, m):
        self.state = m

    def _actor(self, i, m):
        self.actor[i] = (m.pose.position.x, m.pose.position.y)

    def _local(self, m):
        self.local = m.pose.position

    def _report(self, m):
        try:
            d = json.loads(m.data)
        except Exception:
            return
        with self.det_lock:
            self.dets.append((rospy.get_time(), d.get("uav"), d.get("dets") or []))

    def _img(self, msg):
        self.last_img = (msg, rospy.get_time())

    def _sp_loop(self):
        r = rospy.Rate(SP_HZ)
        while not rospy.is_shutdown() and not self._stop:
            with self.sp_lock:
                p = self.sp
            if p is not None:
                p.header.stamp = rospy.Time.now()   # mavros 需要新鲜时间戳
                self.pub_pose.publish(p)
            r.sleep()

    # ---------------- 基本动作 ----------------
    @staticmethod
    def pose_msg(lx, ly, lz, yaw_rad):
        p = PoseStamped()
        p.header.frame_id = "map"
        p.pose.position.x, p.pose.position.y, p.pose.position.z = lx, ly, lz
        p.pose.orientation.z = math.sin(yaw_rad / 2.0)
        p.pose.orientation.w = math.cos(yaw_rad / 2.0)
        return p

    def set_sp(self, lx, ly, lz, yaw_rad):
        with self.sp_lock:
            self.sp = self.pose_msg(lx, ly, lz, yaw_rad)

    def world(self):
        p = self.gms(UAV, "ground_plane").pose.position
        return (p.x, p.y, p.z)

    def send(self, s):
        """ARM / OFFBOARD 走 mavros 原生服务。返回值只代表"指令已发出"，须回读状态确认。"""
        try:
            if s == "ARM":
                r = self.srv_arm(True)
                print("[m2fly] 指令 ARM -> success=%s" % r.success, flush=True)
            elif s == "OFFBOARD":
                r = self.srv_mode(custom_mode="OFFBOARD")
                print("[m2fly] 指令 OFFBOARD -> mode_sent=%s" % r.mode_sent, flush=True)
            else:
                print("[m2fly] 未知指令 %s（忽略）" % s, flush=True)
        except Exception as e:
            print("[m2fly] 指令 %s 异常: %s" % (s, e), flush=True)
        rospy.sleep(1.5)

    def set_params(self, pairs):
        """无头 SITL 必需：NAV_RCL_ACT / NAV_DLL_ACT = 0。
        否则 PX4 判「无遥控器且无数据链」→ 失效保护 → 拒绝解锁。
        （与 fly_takeoff.py 同款处理；这两个参数是**易失**的，每次飞前都要设。）"""
        for pid, val in pairs:
            try:
                r = self.srv_param(param_id=pid,
                                   value=ParamValue(integer=0, real=float(val)))
                print("[m2fly] 参数 %s=%s -> success=%s" % (pid, val, r.success), flush=True)
            except Exception as e:
                print("[m2fly] 设参数 %s 失败(不致命): %s" % (pid, e), flush=True)

    def _armed(self):
        return bool(self.state and self.state.armed)

    def _mode(self):
        return self.state.mode if self.state else "?"

    def wait_armed(self, timeout=20.0):
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t0 < timeout:
            if self._armed():
                return True
            rospy.sleep(0.2)
        return False

    def wait_mode(self, mode, timeout=15.0):
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t0 < timeout:
            if self._mode() == mode:
                return True
            rospy.sleep(0.2)
        return False

    def wait(self, dur, tag, every=2.0):
        """主线程等待 dur 仿真秒，期间 setpoint 线程照发"""
        t0 = rospy.get_time()
        nx = 0.0
        while not rospy.is_shutdown() and rospy.get_time() - t0 < dur:
            if rospy.get_time() - t0 >= nx:
                w = self.world()
                print("[m2fly] %s t=%4.1fs 世界=(%7.2f,%7.2f,%5.2f)"
                      % (tag, rospy.get_time() - t0, w[0], w[1], w[2]), flush=True)
                nx = rospy.get_time() - t0 + every
            rospy.sleep(0.1)

    def follow(self, aid, keep, alt, dur, tag="跟随", sample=False, step=1.2):
        """跟随 actor 保持 keep 米，机头始终对准，持续 dur 仿真秒"""
        t0 = rospy.get_time()
        nx = 0.0
        next_shot = 0.0
        while not rospy.is_shutdown() and rospy.get_time() - t0 < dur:
            w = self.world()
            a = self.actor.get(aid)
            if a is None:
                rospy.sleep(0.1)
                continue
            dx, dy = w[0] - a[0], w[1] - a[1]
            d = math.hypot(dx, dy) or 1e-3
            ux, uy = dx / d, dy / d
            tx, ty = a[0] + keep * ux, a[1] + keep * uy
            yaw = math.atan2(a[1] - w[1], a[0] - w[0])
            # 朝观察点小步移动（每 tick 最多 step 米，避免远距离 setpoint 引起姿态翻转）
            gap = math.hypot(tx - w[0], ty - w[1])
            if gap > step:
                lx = self.local.x + (tx - w[0]) / gap * step
                ly = self.local.y + (ty - w[1]) / gap * step
            else:
                lx, ly = self.local.x, self.local.y
            self.set_sp(lx, ly, alt, yaw)
            if rospy.get_time() - t0 >= nx:
                print("[m2fly] %s t=%4.1fs 距actor=%5.2fm (目标%.1f) 世界=(%7.2f,%7.2f,%5.2f)"
                      % (tag, rospy.get_time() - t0, d, keep, w[0], w[1], w[2]), flush=True)
                nx = rospy.get_time() - t0 + 2.0
            if sample and rospy.get_time() - t0 >= next_shot:
                self._save_frame()
                next_shot = rospy.get_time() - t0 + 3.0
            rospy.sleep(0.1)

    def _save_frame(self):
        if self.last_img is None:
            return
        try:
            import cv2
            img = self.bridge.imgmsg_to_cv2(self.last_img[0], "bgr8")
            os.makedirs(FRAME_DIR, exist_ok=True)
            p = os.path.join(FRAME_DIR, "f_%08.2f.jpg" % rospy.get_time())
            cv2.imwrite(p, img)
            self.frames.append(p)
        except Exception:
            pass


def stats(dets, aid, gt_hist, color, conf_min, rmin, rmax):
    print("\n[m2fly] ============ 感知 vs 真值 统计 (actor_%d / %s) ============"
          % (aid, color), flush=True)
    hit = []
    tried = 0
    cls_cnt = {}
    for t, uav, ds in dets:
        gt, best = None, 1e9
        for gt_t, gx, gy in gt_hist:
            if abs(gt_t - t) < best:
                best, gt = abs(gt_t - t), (gx, gy)
        if gt is None or best > 1.5:
            continue
        for d in ds:
            if d.get("conf", 0) < conf_min:
                continue
            r = d.get("range", 0)
            if r < rmin or r > rmax:
                continue
            cls = d.get("cls", "?")
            cls_cnt[cls] = cls_cnt.get(cls, 0) + 1
            tried += 1
            xyz = d["xyz"]
            err = math.hypot(xyz[0] - gt[0], xyz[1] - gt[1])
            hit.append((err, r, d["conf"], cls, xyz[0], xyz[1]))
    print("[m2fly] HOLD 期间有效检测 %d 条（conf>=%.2f, 距离 %.0f~%.0fm），类别分布: %s"
          % (tried, conf_min, rmin, rmax, cls_cnt), flush=True)
    if not hit:
        print("[m2fly] !! 没有任何有效检测落入距离窗口 —— 目标可能不在视野内", flush=True)
        return
    hit.sort(key=lambda z: z[0])
    print("[m2fly] 与 actor_%d 真值最近的 10 条：" % aid, flush=True)
    for e, r, c, cls, x, y in hit[:10]:
        print("      err=%5.2fm  range=%5.1fm  conf=%.2f  cls=%-6s  解算=(%7.2f,%7.2f)"
              % (e, r, c, cls, x, y), flush=True)
    matched = [h for h in hit if h[0] < 3.0]
    correct = [h for h in hit if h[0] < 3.0 and h[3] == color]
    print("[m2fly] 命中真值(<3m): %d/%d = %.0f%%   其中类别也正确(%s): %d"
          % (len(matched), len(hit), 100.0 * len(matched) / len(hit), color, len(correct)), flush=True)
    if matched:
        errs = [h[0] for h in matched]
        print("[m2fly] 命中样本误差: min=%.2fm  中位=%.2fm  均值=%.2fm  max=%.2fm"
              % (errs[0], errs[len(errs) // 2], sum(errs) / len(errs), errs[-1]), flush=True)
    if correct:
        errs = [h[0] for h in correct]
        print("[m2fly] 【类别正确样本】误差: min=%.2fm  中位=%.2fm  均值=%.2fm  n=%d"
              % (errs[0], errs[len(errs) // 2], sum(errs) / len(errs), len(errs)), flush=True)
    print("[m2fly] ============================================================\n", flush=True)


def main():
    argv = sys.argv[1:]
    args = [a for a in argv if not a.startswith("--")]

    def opt(name, default):
        for a in argv:
            if a.startswith("--" + name + "="):
                return a.split("=", 1)[1]
        return default

    aid_arg = args[0] if args else "auto"
    keep = float(args[1]) if len(args) > 1 else 18.0
    alt = float(args[2]) if len(args) > 2 else 5.0
    hold_sec = float(opt("hold", "30"))
    stay_sec = float(opt("stay", "60"))
    conf_min = float(opt("conf", "0.50"))
    rmin = float(opt("rmin", "8"))
    rmax = float(opt("rmax", "40"))

    f = FlyToActor()
    w0 = f.world()
    print("[m2fly] 当前 世界=(%.2f, %.2f, %.2f)  高度 %.2f m" % (w0[0], w0[1], w0[2], w0[2]), flush=True)
    print("[m2fly] 已知 actor 位置: %s" % {k: (round(v[0], 1), round(v[1], 1))
                                            for k, v in sorted(f.actor.items())}, flush=True)
    if not f.actor:
        print("[m2fly] 未收到任何 /actor_N/pose，退出")
        return

    if aid_arg == "auto":
        aid = min(f.actor, key=lambda k: (f.actor[k][0] - w0[0]) ** 2 + (f.actor[k][1] - w0[1]) ** 2)
    else:
        aid = int(aid_arg)
    color = ID2CLS.get(aid, "?")
    print("[m2fly] 目标 actor_%d (%s) @ (%.2f, %.2f)  保持 %.1f m  高度 %.1f m  HOLD %.0fs"
          % (aid, color, f.actor[aid][0], f.actor[aid][1], keep, alt, hold_sec), flush=True)

    # ---- 先建 setpoint 流（PX4 要求 OFFBOARD 之前已有流），再 ARM/OFFBOARD ----
    l0 = f.local
    yaw0 = math.atan2(f.actor[aid][1] - w0[1], f.actor[aid][0] - w0[0])
    f.set_sp(l0.x, l0.y, alt, yaw0)
    print("[m2fly] setpoint 流已建立(20Hz)，等待 3s 后切 OFFBOARD", flush=True)
    rospy.sleep(3.0)
    # ⚠ 无头 SITL 下 PX4 默认「无遥控器 + 无数据链 ⇒ 失效保护」，ARM 会被静默拒绝。
    #   必须先设这两个参数（fly_takeoff.py 已验证）。它们是**易失**的：PX4 重启或
    #   VM 挂起恢复后可能丢，所以每次飞之前都设一遍，不能指望上一次留下的值。
    f.set_params([("NAV_RCL_ACT", 0.0), ("NAV_DLL_ACT", 0.0)])
    f.send("ARM")
    if not f.wait_armed(20.0):
        print("[m2fly] !! ARM 未生效 (armed=%s mode=%s) —— 中止"
              % (f._armed(), f._mode()), flush=True)
        return
    print("[m2fly] 已解锁 armed=True mode=%s" % f._mode(), flush=True)
    f.send("OFFBOARD")
    if not f.wait_mode("OFFBOARD", 15.0):
        print("[m2fly] !! OFFBOARD 未生效 (mode=%s) —— 中止" % f._mode(), flush=True)
        return
    print("[m2fly] 已切 OFFBOARD", flush=True)
    f.wait(8.0, tag="升空")
    print("[m2fly] 已到工作高度，世界=(%.2f, %.2f, %.2f)" % f.world(), flush=True)

    # ---- 分段逼近观察点 ----
    for step in range(26):
        w = f.world()
        a = f.actor.get(aid)
        if a is None:
            print("[m2fly] actor_%d 位置丢失，停止" % aid)
            break
        dx, dy = w[0] - a[0], w[1] - a[1]
        d = math.hypot(dx, dy)
        if d < 1e-3:
            dx, dy, d = 1.0, 0.0, 1.0
        tx = a[0] + keep * dx / d
        ty = a[1] + keep * dy / d
        yaw = math.atan2(a[1] - w[1], a[0] - w[0])
        gap = math.hypot(tx - w[0], ty - w[1])
        print("[m2fly] --- 第 %d 步: 距 actor %.1f m (目标 %.1f m)  到观察点还差 %.1f m ---"
              % (step + 1, d, keep, gap), flush=True)
        if gap < 1.5 and abs(d - keep) < 3.0:
            print("[m2fly] 已在观测位", flush=True)
            break
        if gap > SEG:
            gx, gy = (tx - w[0]) / gap, (ty - w[1]) / gap
            tx, ty = w[0] + gx * SEG, w[1] + gy * SEG
        f.set_sp(f.local.x + (tx - w[0]), f.local.y + (ty - w[1]), alt, yaw)
        f.wait(SEG_DUR, tag="逼近%d" % (step + 1))

    w = f.world()
    a = f.actor.get(aid, (0, 0))
    print("[m2fly] 到位 无人机世界=(%.2f, %.2f, %.2f)  actor_%d=(%.2f, %.2f) 距离=%.2fm 方位=%.1f°"
          % (w[0], w[1], w[2], aid, a[0], a[1], math.hypot(a[0] - w[0], a[1] - w[1]),
             math.degrees(math.atan2(a[1] - w[1], a[0] - w[0]))), flush=True)

    # ---- HOLD：跟随悬停 + 采集感知 + 记录真值 ----
    with f.det_lock:
        f.dets = []
    gt_hist = []

    def _gt():
        while not rospy.is_shutdown():
            a = f.actor.get(aid)
            if a:
                gt_hist.append((rospy.get_time(), a[0], a[1]))
            rospy.sleep(0.1)
    threading.Thread(target=_gt, daemon=True).start()

    print("[m2fly] >>> 进入 HOLD %.0f 仿真秒（跟随 actor_%d 保持 %.1f m，持续对准）"
          % (hold_sec, aid, keep), flush=True)
    f.follow(aid, keep, alt, hold_sec, tag="HOLD", sample=True)
    with f.det_lock:
        dets = list(f.dets)

    stats(dets, aid, gt_hist, color, conf_min, rmin, rmax)
    print("[m2fly] 存帧 %d 张 -> %s" % (len(f.frames), FRAME_DIR), flush=True)
    print("[m2fly] VERIFY_DONE", flush=True)

    # ---- 继续驻留：绝不停止 setpoint，否则 1s 后触发 OFFBOARD 失效保护 ----
    if stay_sec > 0:
        print("[m2fly] 继续驻留 %.0f 仿真秒（持续发 setpoint，可随时 Ctrl-C）" % stay_sec, flush=True)
        f.follow(aid, keep, alt, stay_sec, tag="驻留")
    print("[m2fly] 进入无限驻留（跟随 actor_%d，Ctrl-C 退出）" % aid, flush=True)
    f.follow(aid, keep, alt, 1e9, tag="驻留")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
