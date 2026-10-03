"""Read-only capture of the actual images behind published visual evidence."""
import argparse
from collections import deque
import json
from pathlib import Path
import threading
import time


def main():
    import cv2
    from cv_bridge import CvBridge
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--wall-seconds',type=float,default=600.)
    parser.add_argument('--limit-per-tag',type=int,default=24)
    parser.add_argument('--minimum-period',type=float,default=2.)
    args=parser.parse_args()
    output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    rospy.init_node('read_only_visual_frame_capture',anonymous=True)
    buffers={'uav_%d'%i:deque(maxlen=20) for i in range(1,7)}
    lock=threading.Lock();counts={};last_saved={};bridge=CvBridge()
    log=(output/'frames.jsonl').open('x')
    def image_cb(message,uid):
        with lock:buffers[uid].append(message)
    def visual_cb(message):
        try:observation=json.loads(message.data)
        except ValueError:return
        uid=observation.get('uav_id');tag=observation.get('target_id')
        if uid not in buffers or tag not in ('green','blue','brown','white','red1','red2'):return
        with lock:
            if counts.get(tag,0)>=args.limit_per_tag:return
            frames=list(buffers[uid])
            if not frames:return
            stamp=float(observation['sample_s'])
            if stamp-last_saved.get(tag,float('-inf'))<args.minimum_period:return
            image=min(frames,key=lambda m:abs(m.header.stamp.to_sec()-stamp))
            delta=abs(image.header.stamp.to_sec()-stamp)
            if delta>.01:return  # Never substitute a different image as evidence.
            counts[tag]=counts.get(tag,0)+1
            last_saved[tag]=stamp
            name='%s_%s_%03d.png'%(uid,tag,counts[tag])
            try:
                pixels=bridge.imgmsg_to_cv2(image,'bgr8')
                if not cv2.imwrite(str(output/name),pixels):raise RuntimeError('PNG_WRITE_FAILED')
                record=dict(image=name,image_sample_s=image.header.stamp.to_sec(),
                            observation=observation,control_input=False)
            except Exception as error:
                record=dict(error=str(error),observation=observation,control_input=False)
            log.write(json.dumps(record)+'\n');log.flush()
    subscribers=[rospy.Subscriber('/'+uid+'/cgo3_camera/image_raw',Image,image_cb,
                     callback_args=uid,queue_size=2,buff_size=2**22) for uid in buffers]
    subscribers.append(rospy.Subscriber('/swarm/visual_observation',String,visual_cb,queue_size=50))
    deadline=time.monotonic()+args.wall_seconds
    try:
        while time.monotonic()<deadline and not rospy.is_shutdown():time.sleep(.2)
    finally:
        for subscriber in subscribers:subscriber.unregister()
        rospy.signal_shutdown('Read-only frame capture complete')
        with lock:log.close()


if __name__=='__main__':main()
