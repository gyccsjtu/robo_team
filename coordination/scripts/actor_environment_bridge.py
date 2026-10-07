#!/usr/bin/env python3
"""Gazebo world odometry for platform actors only; never a flight input.

ModelStates has no acquisition header. Odometry uses callback receipt ROS time,
not a fabricated sensor timestamp. Twist is rotated into the aircraft frame.
No MAVROS local pose, estimator offset, actor truth or position cache is used.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import time


def body_vector(vector, quaternion):
    norm = math.sqrt(sum(v*v for v in quaternion))
    if norm < 1e-8:
        raise ValueError('INVALID_WORLD_ORIENTATION')
    x, y, z, w = [v/norm for v in quaternion]
    x, y, z = -x, -y, -z
    vx, vy, vz = vector
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return (vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz,
            vz+w*tz+x*ty-y*tx)


def world_odometry(pose, twist, stamp, model, factory):
    p, q = pose.position, pose.orientation
    linear = (twist.linear.x, twist.linear.y, twist.linear.z)
    angular = (twist.angular.x, twist.angular.y, twist.angular.z)
    orientation = (q.x, q.y, q.z, q.w)
    if stamp <= 0 or not all(math.isfinite(v) for v in
                            (stamp, p.x, p.y, p.z)+orientation+linear+angular):
        raise ValueError('INVALID_WORLD_ODOMETRY')
    result = factory()
    result.header.frame_id = 'world'
    result.child_frame_id = model
    result.pose.pose = copy.deepcopy(pose)
    norm = math.sqrt(sum(v*v for v in orientation))
    body_linear = body_vector(linear, orientation)
    body_angular = body_vector(angular, orientation)
    for name, value in zip(('x', 'y', 'z', 'w'), orientation):
        setattr(result.pose.pose.orientation, name, value/norm)
    for target, values in ((result.twist.twist.linear, body_linear),
                           (result.twist.twist.angular, body_angular)):
        target.x, target.y, target.z = values
    return result


def fleet_odometry(message, stamp, models, factory):
    if len(message.name) != len(message.pose) or len(message.name) != len(message.twist):
        raise ValueError('MODEL_STATES_LENGTH_MISMATCH')
    if len(set(message.name)) != len(message.name):
        raise ValueError('MODEL_STATES_DUPLICATE_NAME')
    indices = {name: i for i, name in enumerate(message.name)}
    output = {}
    for model in models:
        if model in indices:
            i = indices[model]
            output[model] = world_odometry(message.pose[i], message.twist[i],
                                           stamp, model, factory)
    return output


def fleet_mapping(wiring):
    rows = wiring['uavs']
    expected = {'uav_%d' % (i+1): 'typhoon_h480_%d' % i for i in range(6)}
    actual = {row['uav_id']: row['model_name'] for row in rows}
    if len(rows) != 6 or actual != expected:
        raise ValueError('ACTOR_ENVIRONMENT_FLEET_MAPPING_MISMATCH')
    return {model: '/xtdrone/'+model+'/ground_truth/odom'
            for model in expected.values()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wiring', required=True)
    parser.add_argument('--status', required=True)
    args = parser.parse_args()
    topics = fleet_mapping(json.loads(Path(args.wiring).read_text()))
    import rospy
    import rosgraph
    from gazebo_msgs.msg import ModelStates
    from nav_msgs.msg import Odometry
    rospy.init_node('actor_environment_odometry')
    publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
    duplicates = [topic for topic, nodes in publishers if topic in topics.values() and nodes]
    if duplicates:
        raise RuntimeError('ACTOR_ODOMETRY_ALREADY_PUBLISHED '+repr(duplicates))
    outputs = {model: rospy.Publisher(topic, Odometry, queue_size=1)
               for model, topic in topics.items()}
    ready = {'done': False, 'last_log_s': None, 'samples': 0}

    def receive(message):
        now = rospy.get_time()
        try:
            values = fleet_odometry(message, now, topics, Odometry)
        except ValueError as error:
            rospy.logwarn_throttle(2., 'actor environment odometry: %s', error)
            return
        for model, value in values.items():
            value.header.stamp = rospy.Time.from_sec(now)
            outputs[model].publish(value)
        if len(values) != 6:
            return
        ready['samples'] += 1
        if not ready['done']:
            status = dict(schema_version=1, ready=True, control_input=False,
                          source='/gazebo/model_states', pose_frame='world',
                          twist_frame='aircraft_body', stamp_kind='callback_receipt_ros',
                          models=topics, run_id=os.environ.get('ROBOCUP_RUN_ID'),
                          first_receipt_s=now)
            target = Path(args.status)
            temporary = target.with_suffix('.tmp')
            temporary.write_text(json.dumps(status, indent=2))
            temporary.replace(target)
            ready['done'] = True
        if ready['last_log_s'] is None or now-ready['last_log_s'] >= 1.:
            print(json.dumps(dict(receipt_s=now, samples=ready['samples'],
                aircraft={model: dict(x=value.pose.pose.position.x,
                    y=value.pose.pose.position.y,
                    speed_mps=math.sqrt(sum(v*v for v in
                        (value.twist.twist.linear.x, value.twist.twist.linear.y,
                         value.twist.twist.linear.z))))
                    for model, value in values.items()})), flush=True)
            ready['last_log_s'] = now

    subscription = rospy.Subscriber('/gazebo/model_states', ModelStates, receive, queue_size=1)
    deadline = time.monotonic()+20.
    while not ready['done'] and not rospy.is_shutdown():
        if time.monotonic() >= deadline:
            raise RuntimeError('ACTOR_ENVIRONMENT_ODOMETRY_NOT_READY')
        time.sleep(.05)
    rospy.spin()


if __name__ == '__main__':
    main()
