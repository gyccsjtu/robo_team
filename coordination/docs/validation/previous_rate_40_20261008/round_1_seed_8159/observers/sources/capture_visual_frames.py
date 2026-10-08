"""Read-only capture of the actual images behind published visual evidence."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import threading
import time


class GapCapture:
    """Bounded diagnostic frames after an accepted stream pauses, never evidence."""
    def __init__(self, color, total_limit=24, per_gap_limit=8):
        self.color, self.total_limit, self.per_gap_limit = color, total_limit, per_gap_limit
        self.latest, self.gap_counts, self.attempted = {}, {}, set()

    def observe(self, observation, now_s):
        uid, stamp = observation.get('uav_id'), observation.get('sample_s')
        if (observation.get('target_id') != self.color or observation.get('schema_version') not in (2,3)
                or uid not in ('uav_%d'%i for i in range(1, 7))
                or not isinstance(stamp, (int, float)) or not math.isfinite(stamp)
                or not math.isfinite(now_s) or not 0. <= now_s-stamp <= 1.):
            return
        previous = self.latest.get(uid)
        if previous is None or stamp > previous['sample_s']:
            self.latest[uid] = dict(observation)
            self.gap_counts[uid] = 0

    def select(self, uid, image_s, now_s):
        last = self.latest.get(uid)
        key = (uid, image_s)
        if (last is None or not all(math.isfinite(v) for v in (image_s, now_s))
                or not 1. <= now_s-image_s <= 1.5
                or not .8 <= image_s-last['sample_s'] <= 3.
                or key in self.attempted or len(self.attempted) >= self.total_limit
                or self.gap_counts.get(uid, 0) >= self.per_gap_limit):
            return None
        # Bound attempted writes too, so a broken image encoder cannot loop forever.
        self.attempted.add(key)
        self.gap_counts[uid] = self.gap_counts.get(uid, 0)+1
        return dict(kind='gap_image', uav_id=uid, image_sample_s=image_s,
                    last_observation=dict(last), control_input=False,
                    meaning='No later accepted observation received when this frame was selected')


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
    parser.add_argument('--loss-color', choices=('white',), default=None)
    parser.add_argument('--loss-limit', type=int, default=24)
    parser.add_argument('--loss-limit-per-gap', type=int, default=8)
    args=parser.parse_args()
    if args.loss_limit < 1 or args.loss_limit_per_gap < 1:
        parser.error('Loss capture limits must be positive')
    output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    rospy.init_node('read_only_visual_frame_capture',anonymous=True)
    buffers={'uav_%d'%i:deque(maxlen=20) for i in range(1,7)}
    lock=threading.Lock();counts={};last_saved={};bridge=CvBridge()
    log=(output/'frames.jsonl').open('x')
    gaps = GapCapture(args.loss_color, args.loss_limit, args.loss_limit_per_gap) if args.loss_color else None
    def image_cb(message,uid):
        with lock:buffers[uid].append(message)
    def visual_cb(message):
        try:observation=json.loads(message.data)
        except ValueError:return
        uid=observation.get('uav_id');tag=observation.get('target_id')
        if uid not in buffers or tag not in ('green','blue','brown','white','red1','red2'):return
        with lock:
            if gaps is not None:gaps.observe(observation, rospy.Time.now().to_sec())
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
        while time.monotonic()<deadline and not rospy.is_shutdown():
            if gaps is not None:
                with lock:
                    now_s = rospy.Time.now().to_sec()
                    for uid, frames in buffers.items():
                        for frame in frames:
                            record = gaps.select(uid, frame.header.stamp.to_sec(), now_s)
                            if record is None:continue
                            name = '%s_%s_gap_%03d.png'%(uid, args.loss_color, len(gaps.attempted))
                            try:
                                pixels = bridge.imgmsg_to_cv2(frame, 'bgr8')
                                if not cv2.imwrite(str(output/name), pixels):raise RuntimeError('PNG_WRITE_FAILED')
                                record['image'] = name
                            except Exception as error:
                                record['error'] = str(error)
                            log.write(json.dumps(record)+'\n');log.flush()
            time.sleep(.2)
    finally:
        for subscriber in subscribers:subscriber.unregister()
        rospy.signal_shutdown('Read-only frame capture complete')
        with lock:log.close()


if __name__=='__main__':main()
