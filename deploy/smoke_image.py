#!/usr/bin/env python3
"""Offline dependency and model smoke test; never connects to the ROS graph."""
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(root / 'perception'), str(root / 'coordination/src/robocup_swarm/scripts'),
                str(root / 'coordination/src/robocup_navigation/src')]
import cv2
import numpy as np
from cv_bridge import CvBridge
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from robocup_swarm.msg import SearchAssignment, UavStatus
from ultralytics import YOLO

assert ActorInfo._type == 'ros_actor_cmd_pose_plugin_msgs/ActorInfo'
assert ActorInfo._md5sum == 'd57df6c0908e158da5273b4646f729dd'
image = np.zeros((480, 752, 3), dtype=np.uint8)
bridge = CvBridge()
assert np.array_equal(bridge.imgmsg_to_cv2(bridge.cv2_to_imgmsg(image, 'bgr8'), 'bgr8'), image)
assert SearchAssignment._type == 'robocup_swarm/SearchAssignment'
assert UavStatus._type == 'robocup_swarm/UavStatus'
for name in ('best_yolo11n_bino_v3_clean.pt', 'best_yolo11n_bino_sculpture_v3.pt'):
    path = root / 'weights' / name
    assert path.is_file(), path
    model = YOLO(str(path))
    assert {'red', 'green', 'blue', 'white', 'brown'} <= set(model.names.values()), path
print('[player smoke] PASS: messages, CV bridge and both weights load')
