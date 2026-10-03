#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓一帧相机原图（不做任何判定）—— 用于直接目视核对"画面里到底有没有红衣人"。

为什么需要它：感知的标注帧只画通过"人判决"的目标，验证期间 actor 若被判据
拦住（静止/量程），标注帧就是空的，容易误判成"看不到人"。
原图不带任何滤镜，是最硬的证据。

用法:  python3 snap_frame.py [输出路径=/tmp/snap_frame.jpg] [等待秒=10]
"""
import sys
import time

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/snap_frame.jpg"
WAIT = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
TOPIC = "/typhoon_h480_0/cgo3_camera/image_raw"

box = {"img": None}


def cb(msg):
    try:
        box["img"] = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except Exception:
        pass


rospy.init_node("snap_frame", anonymous=True)
rospy.Subscriber(TOPIC, Image, cb, queue_size=1)
t0 = time.time()
while box["img"] is None and time.time() - t0 < WAIT:
    time.sleep(0.2)
if box["img"] is None:
    print("NO_IMAGE（相机不出图 -> 大概率是 DISPLAY/GL 问题，见 MEMORY 的三个必记坑）")
    sys.exit(1)
cv2.imwrite(OUT, box["img"])
print("saved %s %s" % (OUT, box["img"].shape))
