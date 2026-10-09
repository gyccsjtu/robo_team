#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自机体（self-body）遮蔽标定 —— 自动提取「无人机自己」在画面里占的位置。

============================ 为什么必须做这件事 ============================
真机找人时 YOLO 会把无人机**自己的身体部件**当成目标上报。实测证据帧：

    white 0.66  H=2.32m  D=15.8m   -> 框在起落架白色斜撑上
    white 0.75  H=1.96m  D=14.9m   -> 框在同一根撑杆上

两个框的归一化身高 H 都落在「像人」的区间（真人 1.70~2.11 m），
单帧几何判据（H 门限 / 宽高比 / conf）**怎么调都分不开**。
病根不在几何，而在这句话：**它根本不是地面物体**。

  相机与机体是刚性连接 -> 框底边不在地面上 -> `t = -pz/vz` 解出来的
  15 m「距离」是假的 -> 于是 H=h_px*t/fy 也被算成一个像人的数。

============================ 唯一可靠的判据 ============================
**它跟着相机一起动。**
地面/建筑/天空随视点变化；自身机体与相机刚性连接，在**画面坐标里不动**。

做法：把无人机瞬移到 N 个不同视点，每个视点拍 M 帧，
逐像素求时间维标准差：
    场景   -> 视点一变，像素值大变 -> 方差大
    自身机体 -> 永远在同一个像素上     -> 方差≈0
方差图二值化就是机体掩膜。

============================ 两个必须防的坑 ============================
1) **必须持续瞬移**：PX4 在跑，瞬移过去后若不管它，1.5 s 内自由落体十几米，
   抓到的帧会一路下滑。所以用 10 Hz 后台线程把它钉住（同 hover_teleport）。
2) **天空也是低方差**：相机水平时地平线固定在 v≈180.5（cy），
   天空梯度只随俯仰变，而俯仰被我们固定 -> 天空方差也≈0，会被误收进掩膜。
   -> 只用 v > MASK_MIN_V(默认195) 的下半幅做掩膜，天空天然被排除。

用法:  calib_selfmask.py [sites=8] [frames=3] [radius=40] [height=7]
输出:  ~/robocup_real/selfmask.png / selfmask_std.png / selfmask_preview.jpg
"""
import math
import os
import random
import sys
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from sensor_msgs.msg import Image

UAV = os.environ.get("CS_UAV", "typhoon_h480_0")
OUT_DIR = os.path.expanduser(os.environ.get("CS_OUT", "~/robocup_real"))
CAM_TOPIC = "/%s/cgo3_camera/image_raw" % UAV
BLACK_BOX = os.path.expanduser("~/XTDrone/robocup/black_box.txt")

# 只有下半个画面做掩膜：地平线固定在 cy=180.5，其上方是天空（也低方差）
MASK_MIN_V = int(os.environ.get("CS_MIN_V", "195"))
STD_THRESH = float(os.environ.get("CS_STD", "8.0"))     # 逐像素时域标准差门限（0-255）
DILATE = int(os.environ.get("CS_DILATE", "5"))          # 掩膜外扩像素，吃掉边缘抖动

IMG_W, IMG_H = 640, 360

_lock = threading.Lock()
_latest = [None]
_pose = {"yaw": 0.0, "x": 0.0, "y": 0.0, "z": 0.0}


def setup_topic():
    bridge = CvBridge()

    def cb(m):
        try:
            im = bridge.imgmsg_to_cv2(m, "bgr8")
        except Exception:
            return
        with _lock:
            _latest[0] = im

    rospy.Subscriber(CAM_TOPIC, Image, cb, queue_size=1)


def grab(timeout=6.0):
    """取最近一帧（丢弃积压，保证拿到的是瞬移之后的画面）"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        with _lock:
            im = _latest[0]
            _latest[0] = None
        if im is not None:
            return im
        rospy.sleep(0.05)
    return None


def load_boxes(path):
    """官方 black_box.txt 是**单行**的（43 个矩形），必须整行 eval"""
    try:
        with open(path) as f:
            data = eval(f.read().strip())
        items = data if isinstance(data[0][0], list) else [data]
        return [(min(b[0][0], b[0][1]), max(b[0][0], b[0][1]),
                 min(b[1][0], b[1][1]), max(b[1][0], b[1][1])) for b in items]
    except Exception as e:
        print("[cs] black_box 读取失败: %s" % e, flush=True)
        return []


def clearance(x, y, boxes):
    best = 1e9
    for (x0, x1, y0, y1) in boxes:
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        d = math.hypot(dx, dy)
        if dx == 0.0 and dy == 0.0:
            d = -min(x - x0, x1 - x, y - y0, y1 - y)
        best = min(best, d)
    return best if boxes else 1e9


def main():
    args = sys.argv[1:]
    n_sites = int(args[0]) if len(args) > 0 else 8
    n_frames = int(args[1]) if len(args) > 1 else 3
    radius = float(args[2]) if len(args) > 2 else 40.0
    height = float(args[3]) if len(args) > 3 else 7.0

    rospy.init_node("calib_selfmask", anonymous=True)
    setup_topic()
    rospy.wait_for_service("/gazebo/set_model_state", timeout=30)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=30)
    setms = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    getms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    boxes = load_boxes(BLACK_BOX)
    try:
        p = getms(UAV, "world").pose.position
        cx, cy = p.x, p.y
    except Exception:
        cx, cy = 50.0, 0.0
    print("[cs] 中心=(%.1f,%.1f) 建筑=%d 视点=%d 每视点=%d帧 半径=%.0fm 高度=%.0fm"
          % (cx, cy, len(boxes), n_sites, n_frames, radius, height), flush=True)

    # ---- 选视点 ----
    # 三种模式，前两种都失败过，记录在此免得重踩：
    #   pos（失败）：环绕多点但**机头朝着圆心** -> 航向跟着变。
    #               相机与机体刚性连接，航向一变机身就在画面里跟着转，
    #               实测下半幅最小 std 只有 9.4（干净时应 ≈0），分不出来。
    #   yaw（半失败）：定点扫航向。航向变、位置不变，但**姿态在抖**
    #               （10 Hz 瞬移之间物理步进会俯仰），机臂伸得长，
    #               几度姿态抖动就能让杆端在画面里挪几十像素 -> 仍然不是低方差。
    #   trans（默认，正解）：**只平移，航向与姿态全锁死**。
    #               机身与相机的相对位姿完全不变 -> 像素位置严格不变；
    #               场景因为换了地方而整个换一遍 -> 方差极大。区分度最高。
    mode = os.environ.get("CS_MODE", "trans")
    sites = []
    if mode == "yaw":
        fx, fy = cx, cy
        if clearance(fx, fy, boxes) < 5.0:
            for k in range(16):
                th = 2.0 * math.pi * k / 16.0
                tx, ty = cx + radius * math.cos(th), cy + radius * math.sin(th)
                if clearance(tx, ty, boxes) > 8.0:
                    fx, fy = tx, ty
                    break
        sites = [(fx, fy, 2.0 * math.pi * k / float(n_sites)) for k in range(n_sites)]
        print("[cs] 模式=yaw 定点(%.0f,%.0f) 净空=%.1fm" % (fx, fy, clearance(fx, fy, boxes)),
              flush=True)
    elif mode == "pos":
        for k in range(n_sites * 2):
            if len(sites) >= n_sites:
                break
            th = 2.0 * math.pi * k / (n_sites * 2.0)
            sx = cx + radius * math.cos(th)
            sy = cy + radius * math.sin(th)
            if clearance(sx, sy, boxes) < 5.0:
                continue
            sites.append((sx, sy, th + math.pi))       # 机头朝向圆心
        print("[cs] 模式=pos 环绕（航向随之变，已知效果差）", flush=True)
    else:
        fixed_yaw = float(os.environ.get("CS_YAW", "0.0")) * math.pi / 180.0
        for k in range(n_sites * 2):
            if len(sites) >= n_sites:
                break
            th = 2.0 * math.pi * k / (n_sites * 2.0)
            sx = cx + radius * math.cos(th)
            sy = cy + radius * math.sin(th)
            if clearance(sx, sy, boxes) < 5.0:
                continue
            sites.append((sx, sy, fixed_yaw))          # 航向锁死，只换位置
        print("[cs] 模式=trans 环绕平移 半径=%.0fm 航向锁 %.0f°"
              % (radius, math.degrees(fixed_yaw)), flush=True)
    print("[cs] 采用视点 %d 个" % len(sites), flush=True)

    # ---- 10 Hz 后台瞬移线程：不钉住的话无人机会自由落体 ----
    stop = threading.Event()

    def keeper(site):
        rate = rospy.Rate(10.0)
        while not stop.is_set():
            sx, sy, yaw = site[0]
            st = ModelState()
            st.model_name = UAV
            st.pose.position.x = sx
            st.pose.position.y = sy
            st.pose.position.z = height
            st.pose.orientation.z = math.sin(yaw / 2.0)
            st.pose.orientation.w = math.cos(yaw / 2.0)
            st.reference_frame = "world"
            try:
                setms(st)
            except Exception:
                pass
            rate.sleep()

    site_holder = [(cx, cy, 0.0)]
    th_keep = threading.Thread(target=keeper, args=(site_holder,))
    th_keep.daemon = True
    th_keep.start()

    # ---- 采帧 ----
    stack = []
    preview = None
    dump = os.environ.get("CS_DUMP", "")
    if dump:
        os.makedirs(dump, exist_ok=True)
    for i, (sx, sy, yaw) in enumerate(sites):
        site_holder[0] = (sx, sy, yaw)
        rospy.sleep(1.6)                            # 等瞬移生效 + 画面稳定
        got = 0
        for j in range(n_frames):
            im = grab()
            if im is None:
                continue
            if im.shape[1] != IMG_W or im.shape[0] != IMG_H:
                im = cv2.resize(im, (IMG_W, IMG_H))
            stack.append(im.astype(np.float32))
            preview = im.copy()
            got += 1
            if dump:
                cv2.imwrite(os.path.join(dump, "s%02d_f%02d_yaw%03d.jpg"
                                         % (i, j, int(round(math.degrees(yaw))) % 360)), im)
            rospy.sleep(0.30)
        print("[cs] 视点 %d/%d (%.0f,%.0f) yaw=%.0f° 取到 %d 帧"
              % (i + 1, len(sites), sx, sy, math.degrees(yaw) % 360, got), flush=True)

    stop.set()
    th_keep.join(timeout=2.0)

    if len(stack) < 4:
        print("[cs] 帧数不足 (%d)，标定失败" % len(stack), flush=True)
        return

    # ---- 逐像素时域标准差 -> 机体掩膜 ----
    arr = np.stack(stack, axis=0)                   # (N,H,W,3)
    std = arr.std(axis=0).max(axis=2)               # (H,W) 取通道最大

    # 诊断：下半幅方差的分位数（用来选门限，别拍脑袋）
    low = std[MASK_MIN_V:, :]
    qs = [0.1, 0.5, 1, 2, 5, 10, 25, 50]
    print("[cs] 下半幅 std 分位数: " +
          " ".join("p%g=%.1f" % (q * 100, np.percentile(low, q)) for q in qs), flush=True)
    print("[cs] 下半幅 std 最小=%.1f 均值=%.1f" % (low.min(), low.mean()), flush=True)
    # 8x4 网格均值，直接看出低方差区落在画面哪一块
    gh, gw = IMG_H // 8, IMG_W // 4
    grid = [[std[r * gh:(r + 1) * gh, c * gw:(c + 1) * gw].mean() for c in range(4)]
            for r in range(8)]
    for r in range(8):
        print("[cs]   y%3d-%3d | " % (r * gh, (r + 1) * gh) +
              " ".join("%5.1f" % v for v in grid[r]), flush=True)

    raw = np.zeros((IMG_H, IMG_W), np.uint8)
    sel = std < STD_THRESH
    sel[:MASK_MIN_V, :] = False                     # 天空区不参与（见文件头坑 2）
    raw[sel] = 255

    k = np.ones((3, 3), np.uint8)
    m = cv2.morphologyEx(raw, cv2.MORPH_OPEN, k, iterations=2)   # 去椒盐
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=3)    # 补内部空洞
    if DILATE > 0:
        m = cv2.dilate(m, np.ones((DILATE, DILATE), np.uint8), iterations=1)

    lower = float(m[MASK_MIN_V:, :].mean()) / 255.0
    print("[cs] 掩膜在下半幅占比 %.1f%%   掩膜像素总数 %d"
          % (lower * 100.0, int((m > 0).sum())), flush=True)
    if lower > 0.30:
        print("[cs] ⚠ 占比过高，可能视点太少/场景太相似 —— 掩膜可能误伤真实目标", flush=True)

    os.makedirs(OUT_DIR, exist_ok=True)
    p_mask = os.path.join(OUT_DIR, "selfmask.png")
    cv2.imwrite(p_mask, m)
    cv2.imwrite(os.path.join(OUT_DIR, "selfmask_std.png"),
                cv2.applyColorMap(np.clip(std * 4, 0, 255).astype(np.uint8), cv2.COLORMAP_JET))
    if preview is not None:
        vis = preview.copy()
        vis[m > 0] = (0, 0, 255)                    # 红色 = 被判为自身机体
        cv2.imwrite(os.path.join(OUT_DIR, "selfmask_preview.jpg"), vis)
        cv2.imwrite(os.path.join(OUT_DIR, "selfmask_lastframe.jpg"), preview)
    print("[cs] 已写出 %s" % p_mask, flush=True)
    print("[cs] SELFMASK_DONE", flush=True)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
