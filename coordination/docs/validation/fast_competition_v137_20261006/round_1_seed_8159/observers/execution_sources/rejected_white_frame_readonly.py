import csv,json,time,threading,signal
from collections import deque
from pathlib import Path
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
out=Path(__file__).resolve().parents[1];flight=out.parent/'flight'
folder=out/'white_detected_originals';folder.mkdir(exist_ok=False)
rospy.init_node('rejected_white_original_observer',anonymous=True)
lock=threading.Lock();buffers={'typhoon_h480_%d'%i:deque(maxlen=20) for i in range(6)}
bridge=CvBridge();running=True;offsets={};seen=set();saved=0
log=(folder/'frames.jsonl').open('x')
def stop(*args):
 global running
 running=False
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
def frame(message,key):
 with lock:buffers[key].append(message)
subs=[rospy.Subscriber('/uav_%d/cgo3_camera/image_raw'%(i+1),Image,frame,callback_args='typhoon_h480_%d'%i,queue_size=2,buff_size=2**22) for i in range(6)]
try:
 deadline=time.monotonic()+6000
 while running and not rospy.is_shutdown() and time.monotonic()<deadline and not (flight/'result.json').exists():
  for path in (flight/'algorithm').glob('perception*.csv'):
   key=path.stem.removeprefix('perception_') if hasattr(str,'removeprefix') else path.stem[len('perception_'):]
   if key not in buffers:continue
   with path.open() as stream:
    fields=stream.readline().strip().split(',');stream.seek(offsets.get(str(path),stream.tell()))
    while True:
     before=stream.tell();line=stream.readline()
     if not line or not line.endswith('\n'):stream.seek(before);break
     row=next(csv.DictReader([line],fieldnames=fields))
     if row.get('class_name')!='white' or row.get('event')!='yolo_detection':continue
     stamp=float(row['image_stamp']);identity=(key,stamp)
     if identity in seen or saved>=96:continue
     seen.add(identity)
     with lock:images=list(buffers[key])
     image=min(images,key=lambda im:abs(im.header.stamp.to_sec()-stamp)) if images else None
     rec=dict(control_input=False,pixels_modified=False,source_csv=str(path),raw_detection=row,expected_original_stamp_s=stamp,uav_id='uav_%d'%(int(key.rsplit('_',1)[1])+1))
     if image is None or abs(image.header.stamp.to_sec()-stamp)>.00001:
      rec['error']='EXACT_ORIGINAL_NOT_IN_BUFFER'
     else:
      saved+=1;name='%s_white_%03d.png'%(rec['uav_id'],saved)
      try:
       pixels=bridge.imgmsg_to_cv2(image,'bgr8')
       if not cv2.imwrite(str(folder/name),pixels):raise RuntimeError('PNG_WRITE_FAILED')
       rec.update(image=name,image_sample_s=image.header.stamp.to_sec(),width=image.width,height=image.height)
      except Exception as error:rec['error']=str(error)
     log.write(json.dumps(rec)+'\n');log.flush()
    offsets[str(path)]=stream.tell()
  time.sleep(.3)
finally:
 for sub in subs:sub.unregister()
 rospy.signal_shutdown('Read-only original image evidence complete')
 log.close()
