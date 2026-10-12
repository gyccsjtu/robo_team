"""Bounded, read-only image evidence matched to actual tracking-view stamps."""
import argparse
from collections import deque
import json
from pathlib import Path
import threading
import time

import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--seconds', type=float, default=650.)
    parser.add_argument('--limit', type=int, default=180)
    parser.add_argument('--candidates', action='store_true',
                        help='Capture pixel discovery candidates instead of established tracks.')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rospy.init_node('tracking_images_read_only', anonymous=True)
    bridge, lock = CvBridge(), threading.Lock()
    cache, saved_at = {}, {}
    count = 0
    metadata = (args.output/'images.jsonl').open('x')

    def receive_image(msg, uid):
        with lock:
            cache.setdefault(uid, deque(maxlen=16)).append(msg)

    def receive_view(msg):
        nonlocal count
        try:
            view = json.loads(msg.data)
            uid, stamp = view['source_uav'], float(view['sample_s'])
            tid = (view['candidate_id'].split(':', 1)[0] if args.candidates
                   else view['target_id'])
            key = (uid, tid)
            with lock:
                if count >= args.limit or stamp-saved_at.get(key, -1e9) < 2.:
                    return
                image = next((m for m in reversed(cache.get(uid, ()))
                              if abs(m.header.stamp.to_sec()-stamp) < 1e-6), None)
                if image is None:
                    return
                index = count
                count += 1
                saved_at[key] = stamp
            # Numerical filenames: externally supplied IDs never become paths.
            name = '%04d.jpg' % index
            bgr = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            if not cv2.imwrite(str(args.output/name), bgr, [cv2.IMWRITE_JPEG_QUALITY, 85]):
                raise OSError('image write failed')
            with lock:
                metadata.write(json.dumps(dict(image=name, capture_s=stamp,
                    wall=time.time(), view=view), separators=(',', ':'))+'\n')
                metadata.flush()
        except (ValueError, KeyError, TypeError, OSError) as exc:
            rospy.logwarn_throttle(10., 'image evidence skipped: %s', exc)

    subs = [rospy.Subscriber('/typhoon_h480_%d/stereo_camera/left/image_raw'%i,
                Image, receive_image, 'typhoon_h480_%d'%i, queue_size=1,
                buff_size=2**22) for i in range(6)]
    topic = '/swarm/visual_candidate' if args.candidates else '/swarm/tracking_view'
    subs.append(rospy.Subscriber(topic, String,
                                receive_view, queue_size=4))
    try:
        end = time.monotonic()+args.seconds
        while not rospy.is_shutdown() and time.monotonic() < end:
            time.sleep(.2)
    finally:
        for sub in subs:
            sub.unregister()
        with lock:
            metadata.close()


if __name__ == '__main__':
    main()
