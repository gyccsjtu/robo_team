#!/usr/bin/env python3
"""Optional native CGO3 search aid. Publishes bearings, never actor positions.

One shared low-priority detector, latest frame per selected aircraft. Mount
commands are independent of inference; the existing swarm agent alone flies.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'coordination/src/robocup_swarm/scripts'))
from gimbal_attention import (AttitudeHistory, AuxiliaryTracks, ScanAndHandoff,
    measured_packet, camera_rotation, image_bearing, quat_matrix, selected_uavs, wrap)


class GimbalAuxiliary:
    def __init__(self,args):
        import rospy
        import cv2
        import torch
        from ultralytics import YOLO
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image,CameraInfo
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import Mavlink,MountControl,State
        from std_msgs.msg import String
        from robocup_swarm.msg import SearchAssignment
        self.rospy,self.cv2,self.String,self.MountControl=rospy,cv2,String,MountControl
        self.args,self.lock,self.bridge=args,threading.RLock(),CvBridge()
        torch.set_num_threads(1);cv2.setNumThreads(1)
        os.nice(8)
        self.model=YOLO(args.weights)
        # Warm before subscribing; no fake inference-ready based on process life.
        import numpy as np
        self.model.predict(np.zeros((360,640,3),np.uint8),imgsz=args.imgsz,
                           conf=.45,device='cpu',verbose=False)
        self.session=uuid.uuid4().hex[:12]
        self.aircraft={u:dict(body=AttitudeHistory(),gimbal=AttitudeHistory(),image=None,
            info=None,pose=None,state=None,body_yaw=0.,last_image_s=-1.,next_wall=0.,
            bino_s=-1e18,assignment_s=-1.,assigned=False,configured=False,
            tracks=AuxiliaryTracks(self.session+':'+u),action=ScanAndHandoff(),
            inference=[],bino_delay=deque(maxlen=12),last_command=None,log_s=-1e18)
            for u in selected_uavs(args.uavs)}
        self.observation=rospy.Publisher('/swarm/gimbal_observation',String,queue_size=6)
        self.handoff=rospy.Publisher('/swarm/gimbal_candidate',String,queue_size=6)
        self.subs=[]
        for uid,d in self.aircraft.items():
            pre='/'+uid
            d['pub']=rospy.Publisher(pre+'/mavros/mount_control/command',MountControl,queue_size=1)
            for suffix,ty,kind in [('cgo3_camera/image_raw',Image,'image'),
                ('cgo3_camera/camera_info',CameraInfo,'info'),
                ('mavros/local_position/pose',PoseStamped,'body'),
                ('mavros/state',State,'state'),('mavlink/from',Mavlink,'gimbal')]:
                self.subs.append(rospy.Subscriber(pre+'/'+suffix,ty,self.receive,
                    (uid,kind),queue_size=1 if kind=='image' else 10,buff_size=4*1024*1024))
        self.subs += [rospy.Subscriber('/swarm/processed_camera_frame',String,self.bino_frame,queue_size=30),
            rospy.Subscriber('/swarm/assignment',SearchAssignment,self.assignment,queue_size=30),
            rospy.Subscriber('/swarm/gimbal_feedback',String,self.feedback,queue_size=12)]
        self.timer=rospy.Timer(rospy.Duration(.05),self.control)
        print('[GIMBAL_READY] mode=%s uavs=%s imgsz=%d hz_per_uav=%.1f; bearing only' %
              (args.mode,args.uavs,args.imgsz,args.hz),flush=True)

    def configure(self,uid):
        from mavros_msgs.srv import MountConfigure,CommandLong
        pre='/'+uid+'/mavros/'
        try:
            self.rospy.wait_for_service(pre+'mount_control/configure',timeout=3.)
            self.rospy.ServiceProxy(pre+'mount_control/configure',MountConfigure)(
                mode=2,stabilize_roll=False,stabilize_pitch=False,stabilize_yaw=False)
            self.rospy.wait_for_service(pre+'cmd/command',timeout=3.)
            self.rospy.ServiceProxy(pre+'cmd/command',CommandLong)(broadcast=False,
                command=511,confirmation=0,param1=285,param2=50000)
        except Exception as exc:
            self.rospy.logwarn('[GIMBAL_FALLBACK] %s configure %s; no measured attitude means no handoff',uid,type(exc).__name__)

    def receive(self,msg,key):
        uid,kind=key
        now=self.rospy.get_time()
        with self.lock:
            d=self.aircraft[uid]
            try:
                if kind=='image':
                    if d['image'] is None or msg.header.stamp > d['image'].header.stamp:d['image']=msg
                elif kind=='info':
                    d['info']=(msg.width,msg.height,list(msg.K),list(msg.D))
                elif kind=='body':
                    q=msg.pose.orientation;q=(q.x,q.y,q.z,q.w)
                    stamp=msg.header.stamp.to_sec()
                    if not 0<=now-stamp<=.3:return
                    d['body'].add(stamp,q)
                    r=quat_matrix(q);d['body_yaw']=math.atan2(r[1,0],r[0,0])
                    p=msg.pose.position;d['pose']=(stamp,p.x,p.y,p.z)
                elif kind=='gimbal':
                    packet=measured_packet(msg,now)
                    if packet:d['gimbal'].add(*packet)
                else:
                    d['state']=msg
                    if msg.connected and not d['configured']:
                        d['configured']=True
                        threading.Thread(target=self.configure,args=(uid,),daemon=True).start()
            except (ValueError,TypeError,OverflowError):pass

    def bino_frame(self,msg):
        try:
            row=json.loads(msg.data);sample=float(row['sample_s']);now=self.rospy.get_time()
            with self.lock:
                d=self.aircraft.get(row.get('source_uav'))
                if (d is not None and math.isfinite(sample) and
                        0.<=now-sample<=1. and sample>d['bino_s']):
                    d['bino_s']=sample;d['bino_delay'].append(max(0.,now-sample))
        except (ValueError,KeyError,TypeError):pass

    def assignment(self,msg):
        with self.lock:
            d=self.aircraft.get(msg.uav_id);sample=msg.header.stamp.to_sec()
            if d is None or sample<d['assignment_s']:return
            d['assignment_s']=sample
            d['assigned']=msg.task_type not in (0,255)
            if d['assigned']:d['action'].finish(self.rospy.get_time())

    def feedback(self,msg):
        try:
            row=json.loads(msg.data)
            with self.lock:
                d=self.aircraft.get(row['source_uav'])
                a=d['action'].active if d else None
                if (a and row.get('candidate_id')==a['candidate_id'] and
                        row.get('result') in ('bino_acquired','expired','preempted') and
                        0<=self.rospy.get_time()-float(row['sample_s'])<=.5):
                    d['action'].finish(self.rospy.get_time())
        except (ValueError,KeyError,TypeError):pass

    def available(self,d,now):
        state,pose=d['state'],d['pose']
        return bool(state and state.connected and state.armed and state.mode=='OFFBOARD' and
            pose and 0<=now-pose[0]<=.2 and pose[3]>=2. and not d['assigned'] and
            d['gimbal'].rows and 0<=now-d['gimbal'].rows[-1][0]<=.2 and
            0<=now-d['bino_s']<=.6 and
            (not d['bino_delay'] or sum(d['bino_delay'])/len(d['bino_delay'])<=.25))

    def control(self,event):
        now=self.rospy.get_time()
        with self.lock:
            for uid,d in self.aircraft.items():
                command=d['action'].command(now,d['body_yaw'],self.available(d,now))
                if command is None:continue
                # Slew commands including lock acquisition, never jump a large angle.
                previous=d['last_command']
                if previous and 0<=now-previous[0]<=.2:
                    step=30.*(now-previous[0])
                    command=tuple(max(old-step,min(old+step,new)) for old,new in zip(previous[1],command))
                d['last_command']=(now,command)
                msg=self.MountControl();msg.header.stamp=self.rospy.Time.from_sec(now)
                msg.mode=2;msg.pitch,msg.yaw=command;msg.roll=0.
                d['pub'].publish(msg)

    def step(self,uid):
        now=self.rospy.get_time();wall=time.monotonic()
        with self.lock:
            d=self.aircraft[uid]
            if wall<d['next_wall']:return False
            if not self.available(d,now):
                if now-d['log_s']>=5.:
                    d['log_s']=now
                    print('[GIMBAL_PAUSED] uav=%s assigned=%s bino_age=%.3f; primary chain priority' %
                          (uid,d['assigned'],now-d['bino_s']),flush=True)
                return False
            msg,info=d['image'],d['info']
            if msg is None or info is None:return False
            sample=msg.header.stamp.to_sec()
            if not 0<=now-sample<=.25 or sample<=d['last_image_s']:return False
            g,b=d['gimbal'].at(sample),d['body'].at(sample)
            if g is None or b is None:return False
            rotation=camera_rotation(g,b)
            origin=d['pose'][1:3]
            d['last_image_s']=sample;d['next_wall']=wall+1./self.args.hz
        im=self.bridge.imgmsg_to_cv2(msg,'bgr8')
        if im.shape[:2]!=(info[1],info[0]):return False
        started=time.monotonic()
        result=self.model.predict(im,imgsz=self.args.imgsz,conf=.45,device='cpu',verbose=False)[0]
        elapsed=time.monotonic()-started;rows=[]
        for box in result.boxes:
            color=str(result.names[int(box.cls.item())]).lower()
            if color not in ('red','green','blue','white','brown'):continue
            xyxy=box.xyxy.cpu().numpy()[0].tolist()
            try:bearing,elevation=image_bearing(xyxy,info,rotation)
            except ValueError:continue
            rows.append(dict(color=color,confidence=float(box.conf.item()),
                             bearing=bearing,elevation=elevation,box=xyxy))
        now=self.rospy.get_time()
        with self.lock:
            if not self.available(d,now) or not 0<=now-sample<=.35:return True
            stable=d['tracks'].update(sample,rows)
            d['action'].offer(stable,now,origin)
            a=d['action'].active
            self.observation.publish(self.String(data=json.dumps(dict(
                source_uav=uid,camera='cgo3',sample_s=sample,mode=self.args.mode,
                inference_wall_s=elapsed,image_age_s=now-sample,candidates=rows,
                stable_candidates=len(stable),attitude_source='mavlink_285'))))
            if a and a['sample_s']==sample and self.args.mode=='handoff':
                payload={k:a[k] for k in ('candidate_id','color','bearing','confidence',
                                         'sample_s','first_s','hits','started_s','until_s')}
                payload.update(source_uav=uid,camera='cgo3',purpose='gimbal_handoff',
                    mode='handoff',real_detection=True,official_view_ok=False,
                    attitude_source='mavlink_285',attitude_sample_s=sample)
                self.handoff.publish(self.String(data=json.dumps(payload)))
                print('[GIMBAL_HANDOFF] '+json.dumps(payload),flush=True)
        return True

    def run(self):
        while not self.rospy.is_shutdown():
            for uid in self.aircraft:
                try:self.step(uid)
                except Exception as exc:
                    self.rospy.logwarn_throttle(2.,'[GIMBAL_FALLBACK] %s %s; no auxiliary flight command',uid,type(exc).__name__)
            time.sleep(.01)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode',choices=('shadow','handoff'),required=True)
    ap.add_argument('--uavs',default='5')
    ap.add_argument('--weights',required=True)
    ap.add_argument('--imgsz',type=int,default=960)
    ap.add_argument('--hz',type=float,default=3.)
    args=ap.parse_args();selected_uavs(args.uavs)
    if not .5<=args.hz<=4 or args.imgsz not in (640,960) or not Path(args.weights).is_file():
        ap.error('invalid auxiliary rate, image size or weights')
    import rospy
    rospy.init_node('gimbal_auxiliary')
    GimbalAuxiliary(args).run()


if __name__=='__main__':main()
