"""Read-only MAVROS/lidar/output sampler. No model states or flight commands."""
import argparse
import gzip
import json
import math
from pathlib import Path
import sys
import threading
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'coordination/src/robocup_swarm/scripts'))
import team_time_ros as rospy
from geometry_msgs.msg import PoseStamped,TwistStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State

parser=argparse.ArgumentParser()
parser.add_argument('output')
parser.add_argument('--seconds',type=float,default=300.)
args=parser.parse_args()
rospy.init_node('motion_safety_read_only',anonymous=True,
                clock_sources=tuple('typhoon_h480_%d' % i for i in range(6)),
                clock_timeout=30.)
lock=threading.Lock();data={}

def receive(msg,uid,kind):
    if kind=='state':
        item=dict(mode=msg.mode,armed=msg.armed,connected=msg.connected)
    else:
        item=dict(sample_s=msg.header.stamp.to_sec(),frame=msg.header.frame_id)
        if kind in ('pose','position_command'):
            p,q=msg.pose.position,msg.pose.orientation
            item.update(xyz=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w])
        elif kind in ('velocity','velocity_command'):
            v=msg.twist.linear
            item.update(xyz=[v.x,v.y,v.z],yaw_rate=msg.twist.angular.z)
        else:
            item.update(angle_min=msg.angle_min,angle_increment=msg.angle_increment,
                range_min=msg.range_min,range_max=msg.range_max,
                ranges=[float(v) if math.isfinite(v) else None for v in msg.ranges])
    with lock:
        data.setdefault(uid,{})[kind]=item

subscriptions=[]
for uid in range(6):
    ns='/typhoon_h480_%d/'%uid
    for suffix,cls,kind in [('mavros/state',State,'state'),
            ('mavros/local_position/pose',PoseStamped,'pose'),
            ('mavros/local_position/velocity_local',TwistStamped,'velocity'),
            ('mavros/setpoint_position/local',PoseStamped,'position_command'),
            ('mavros/setpoint_velocity/cmd_vel',TwistStamped,'velocity_command'),
            ('scan',LaserScan,'scan')]:
        subscriptions.append(rospy.Subscriber(ns+suffix,cls,
            lambda msg,u=uid,k=kind:receive(msg,u,k),queue_size=1))
end=time.monotonic()+args.seconds
try:
    with gzip.open(args.output,'wt') as out:
        while not rospy.is_shutdown() and time.monotonic()<end:
            with lock:
                out.write(json.dumps(dict(ros=rospy.get_time(),wall=time.time(),aircraft=data),
                                     separators=(',',':'))+'\n')
            time.sleep(.1)
finally:
    for sub in subscriptions:sub.unregister()
