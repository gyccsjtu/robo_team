#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓一帧官方 cgo3 相机图并存盘，用于目视核验「无人机眼里的世界」。
用法: CAM_NS=/typhoon_h480_0 OUT=/tmp/snap.png python3 snapshot_cam.py
"""
import os
import rospy
import cv2
from sensor_msgs.msg import Image

NS = os.environ.get("CAM_NS", "/typhoon_h480_0")
OUT = os.environ.get("OUT", "/tmp/snap.png")


def cb(msg):
    try:
        from cv_bridge import CvBridge
        img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        print("[snap] cv_bridge 失败: %s" % e, flush=True)
        return
    cv2.imwrite(OUT, img)
    print("[snap] 已保存 %s  尺寸=%dx%d" % (OUT, img.shape[1], img.shape[0]), flush=True)
    rospy.signal_shutdown("done")


def main():
    rospy.init_node("snapshot_cam", anonymous=True)
    rospy.Subscriber(NS + "/cgo3_camera/image_raw", Image, cb, queue_size=1)
    t0 = rospy.get_time()
    while not rospy.is_shutdown() and rospy.get_time() - t0 < 25:
        rospy.sleep(0.2)
    print("[snap] SNAP_DONE", flush=True)


if __name__ == "__main__":
    main()
