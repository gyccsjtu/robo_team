"""Read-only dual-camera detector; raw frames and detections only, no target reports."""
import collections
import hashlib
import json
import time
from pathlib import Path
import cv2
import rospy
import torch
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

ROOT=Path(__file__).resolve().parent
WEIGHT=ROOT.parent/'yolo_remote_audit_20261011/weights/best_yolo11n_bino_v3_clean.pt'
torch.set_num_threads(2)
model=YOLO(str(WEIGHT))
rospy.init_node('gimbal_recognition_bench_detector')
bridge=CvBridge()
latest={};info={};phase={};last_stamp={};counts=collections.Counter()
(ROOT/'frames').mkdir(exist_ok=True)
def image_cb(msg,key):latest[key]=msg
def info_cb(msg,key):info[key]=dict(width=msg.width,height=msg.height,K=list(msg.K),D=list(msg.D))
def phase_cb(msg):
    global phase
    phase=json.loads(msg.data)
rospy.Subscriber('/recognition_bench/phase',String,phase_cb,queue_size=1)
subs=[]
for key,suffix in [('gimbal','cgo3_camera'),('stereo','stereo_camera/left')]:
    topic='/typhoon_h480_0/'+suffix
    subs.append(rospy.Subscriber(topic+'/image_raw',Image,image_cb,key,queue_size=1,buff_size=4*1024*1024))
    subs.append(rospy.Subscriber(topic+'/camera_info',CameraInfo,info_cb,key,queue_size=1))
out=(ROOT/'detections.jsonl').open('w',buffering=1)
started=time.monotonic();seq=0
try:
    while not rospy.is_shutdown() and time.monotonic()-started < 480:
        if phase.get('phase')=='done':break
        current=dict(phase)
        label=current.get('phase','init')
        if label in ('init','transit','settle','landing') or not label:
            time.sleep(.1);continue
        messages=dict(latest)
        if len(messages)!=2 or abs(messages['stereo'].header.stamp.to_sec()-messages['gimbal'].header.stamp.to_sec())>.12:
            time.sleep(.02);continue
        for key,msg in messages.items():
            stamp=msg.header.stamp.to_sec()
            if stamp<=last_stamp.get(key,0) or stamp<current['stamp']+.2:continue
            last_stamp[key]=stamp
            im=bridge.imgmsg_to_cv2(msg,'bgr8')
            wall=time.monotonic()
            result=model.predict(im,imgsz=640,conf=.25,device='cpu',verbose=False)[0]
            latency=time.monotonic()-wall
            boxes=[]
            for box in result.boxes:
                cid=int(box.cls.item())
                boxes.append(dict(cls=cid,color=result.names[cid],conf=float(box.conf.item()),
                                  xyxy=box.xyxy.cpu().numpy()[0].tolist()))
            filename='frames/%05d_%s_%s.png'%(seq,label,key)
            if not cv2.imwrite(str(ROOT/filename),im):raise RuntimeError('image save failed')
            out.write(json.dumps(dict(camera=key,phase=label,stamp=stamp,receipt_age=rospy.get_time()-stamp,
                phase_command=current,inference_wall_s=latency,frame=filename,boxes=boxes))+'\n')
            counts[key+'/'+label]+=1
            seq+=1
        time.sleep(.25)
finally:
    out.close()
    (ROOT/'detector_summary.json').write_text(json.dumps(dict(counts=dict(counts),camera_info=info,
        weight=str(WEIGHT),weight_sha256=hashlib.sha256(WEIGHT.read_bytes()).hexdigest(),
        conf=.25,imgsz=640,scope='Raw network outputs; no production geometric gates or judge reports'),indent=2)+'\n')
    print(dict(counts),flush=True)
