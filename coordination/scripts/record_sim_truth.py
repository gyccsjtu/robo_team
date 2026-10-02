#!/usr/bin/env python3
"""Independent Gazebo truth recorder. Never publishes control or planner input."""
import argparse
import json
from pathlib import Path
import rospy
from gazebo_msgs.msg import ModelStates

ap = argparse.ArgumentParser()
ap.add_argument('--output', required=True)
args = ap.parse_args()
path = Path(args.output)
path.parent.mkdir(parents=True, exist_ok=True)
log = path.open('w', buffering=1)
rospy.init_node('radar_independent_truth_audit', disable_signals=True)
last = [-1.]


def callback(msg):
    stamp = rospy.get_time()
    if stamp-last[0] < .05:
        return
    try:
        i = msg.name.index('typhoon_h480_0')
    except ValueError:
        return
    p, v = msg.pose[i].position, msg.twist[i].linear
    log.write(json.dumps(dict(sim_s=stamp, position=[p.x,p.y,p.z],
                              velocity=[v.x,v.y,v.z]))+'\n')
    last[0] = stamp


rospy.Subscriber('/gazebo/model_states', ModelStates, callback, queue_size=1)
try:
    rospy.spin()
finally:
    log.close()
