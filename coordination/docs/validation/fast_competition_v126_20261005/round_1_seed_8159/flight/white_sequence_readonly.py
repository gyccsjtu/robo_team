"""One bounded read-only image sequence within the existing physical round."""
import json,time,threading
from pathlib import Path
import rospy,cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
out=Path(__file__).parent/'white_sequence_readonly'
out.mkdir(exist_ok=False)
rospy.init_node('read_only_white_miss_sequence',anonymous=True)
bridge=CvBridge();lock=threading.Lock();selected=None;start=None;last=-1.;count=0
log=(out/'frames.jsonl').open('x')
def visual_cb(msg):
 global selected,start
 try:r=json.loads(msg.data)
 except ValueError:return
 if r.get('target_id')!='white':return
 with lock:
  if selected is None:
   selected=r['uav_id'];start=rospy.Time.now().to_sec()
   log.write(json.dumps(dict(kind='trigger',observation=r,received_s=start,control_input=False))+'\n');log.flush()
def image_cb(msg,uid):
 global count,last
 with lock:
  stamp=msg.header.stamp.to_sec()
  if uid!=selected or count>=80 or stamp-last<.25:return
  if not start<=stamp<=start+20.:return
  name='%s_%03d_%.3f.png'%(uid,count,stamp)
  pixels=bridge.imgmsg_to_cv2(msg,'bgr8')
  if not cv2.imwrite(str(out/name),pixels):raise RuntimeError('PNG_WRITE_FAILED')
  log.write(json.dumps(dict(kind='image',image=name,image_sample_s=stamp,received_s=rospy.Time.now().to_sec(),control_input=False))+'\n');log.flush()
  count+=1;last=stamp
subs=[rospy.Subscriber('/uav_%d/cgo3_camera/image_raw'%i,Image,image_cb,callback_args='uav_%d'%i,queue_size=1,buff_size=2**22) for i in range(1,7)]
subs.append(rospy.Subscriber('/swarm/visual_observation',String,visual_cb,queue_size=10))
deadline=time.monotonic()+1200.
try:
 while time.monotonic()<deadline and not (Path(__file__).parent/'result.json').exists() and not rospy.is_shutdown():
  with lock:done=start is not None and (rospy.Time.now().to_sec()>start+20. or count>=80)
  if done:break
  time.sleep(.2)
finally:
 for sub in subs:sub.unregister()
 rospy.signal_shutdown('Read-only sequence complete')
 with lock:log.close()
 print(json.dumps(dict(count=count,uav=selected,start_s=start,control_input=False)),flush=True)
