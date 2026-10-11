"""Read-only search trial recorder. No truth queries, commands or task feedback."""
import argparse
import gzip
import json
import threading
import time
import rospy
from std_msgs.msg import String
from robocup_swarm.msg import SearchAssignment, UavStatus

parser=argparse.ArgumentParser()
parser.add_argument('output')
parser.add_argument('--seconds',type=float,default=800.)
args=parser.parse_args()
rospy.init_node('search_segments_read_only',anonymous=True)
lock=threading.Lock()
output=gzip.open(args.output,'wt',encoding='utf-8')
def write(event,data):
    with lock:
        output.write(json.dumps(dict(event=event,ros=rospy.get_time(),wall=time.time(),data=data),separators=(',',':'))+'\n')
def status(m):
    write('status',dict(uav=m.uav_id,stamp=m.header.stamp.to_sec(),connected=m.connected,
        x=m.x,y=m.y,z=m.z,task=m.search_task_id,version=m.search_revision,index=m.search_index,
        state=m.search_state,path_version=m.search_path_revision,remaining=m.search_path_remaining,
        blocked=m.search_blocked_waypoint,blocked_reason=m.search_blocked_reason,
        cell=[m.assigned_cell_ix,m.assigned_cell_iy],point=[m.assigned_target_x,m.assigned_target_y]))
def assignment(m):
    write('assignment',dict(uav=m.uav_id,stamp=m.header.stamp.to_sec(),type=m.task_type,target=m.target_id,
        task=m.search_task_id,version=m.search_revision,until=m.search_valid_until.to_sec(),
        cells=list(zip(m.search_cell_ix,m.search_cell_iy)),points=[[p.x,p.y] for p in m.search_waypoints]))
def evidence(m,event):
    try:write(event,json.loads(m.data))
    except (ValueError,TypeError):pass
subs=[rospy.Subscriber('/swarm/uav_status',UavStatus,status,queue_size=100),
      rospy.Subscriber('/swarm/assignment',SearchAssignment,assignment,queue_size=50)]
subs += [rospy.Subscriber(topic,String,evidence,event,queue_size=60) for topic,event in
         [('/swarm/processed_camera_frame','processed_frame'),('/swarm/discovery_observation','discovery'),
          ('/swarm/visual_candidate','candidate'),('/swarm/visual_candidate_missed','candidate_missed')]]
try:
    end=time.monotonic()+args.seconds
    while not rospy.is_shutdown() and time.monotonic()<end:time.sleep(.2)
finally:
    for sub in subs:sub.unregister()
    with lock:output.close()
