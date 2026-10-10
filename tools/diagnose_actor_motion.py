#!/usr/bin/env python3
"""Read-only snapshot of referee actors' Gazebo motion and commands.

Run on the host while a match is active after sourcing ROS and catkin setups.
This node publishes nothing and does not call Gazebo services.
"""
import math
import time

import rospy
from gazebo_msgs.msg import ModelStates
from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion


SAMPLE_SECONDS = 5.0
ACTOR_IDS = range(6)
ACTOR_NAMES = {'actor_%d' % aid for aid in ACTOR_IDS}
first = {}
last = {}
commands = {}


def model_states_cb(msg):
    for name, pose in zip(msg.name, msg.pose):
        if name not in ACTOR_NAMES:
            continue
        point = (pose.position.x, pose.position.y)
        first.setdefault(name, point)
        last[name] = point


def command_cb(msg, aid):
    commands[aid] = (msg.x, msg.y, msg.v)


def main():
    rospy.init_node('actor_motion_diagnostic', anonymous=True)
    subscribers = [rospy.Subscriber('/gazebo/model_states', ModelStates,
                                    model_states_cb, queue_size=1)]
    for aid in ACTOR_IDS:
        subscribers.append(rospy.Subscriber(
            '/actor_%d/cmd_motion' % aid, ActorMotion,
            command_cb, callback_args=aid, queue_size=1))
    end = time.monotonic() + SAMPLE_SECONDS
    while not rospy.is_shutdown() and time.monotonic() < end:
        time.sleep(0.1)
    for aid in ACTOR_IDS:
        name = 'actor_%d' % aid
        if name not in last:
            print('%s: absent from /gazebo/model_states' % name)
            continue
        x, y = last[name]
        moved = math.hypot(x - first[name][0], y - first[name][1])
        command = commands.get(aid)
        if command is None:
            print('%s: pos=(%.2f,%.2f) moved=%.3fm; no cmd_motion frame'
                  % (name, x, y, moved))
            continue
        tx, ty, speed = command
        to_target = math.hypot(tx - x, ty - y)
        print('%s: pos=(%.2f,%.2f) moved=%.3fm/%gs; '
              'cmd=(%.2f,%.2f) v=%.2fm/s target_dist=%.2fm'
              % (name, x, y, moved, SAMPLE_SECONDS, tx, ty, speed, to_target))


if __name__ == '__main__':
    main()
