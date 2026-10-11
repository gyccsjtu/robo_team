#!/usr/bin/env python3
"""Read-only image audit, launched separately from competition nodes.

Save the native camera image corresponding to a team visual observation.
No simulator truth, command publisher, generated label or target decision.
All saved frames are unreviewed; a detector proposal is not ground truth.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import queue
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--max-images', type=int, default=600)
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--duration', type=float, default=1200.0)
    args = parser.parse_args()
    if args.max_images <= 0 or args.interval <= 0 or args.duration <= 0:
        parser.error('max-images, interval and duration must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
    manifest_path = args.output / 'frames.jsonl'
    if manifest_path.exists():
        parser.error('Use a new output directory; never overwrite an earlier audit')

    import cv2
    from cv_bridge import CvBridge
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rospy.init_node('visual_frame_audit_read_only', anonymous=True)
    bridge = CvBridge()
    names = ['typhoon_h480_%d' % i for i in range(6)]
    frames = {name: deque(maxlen=24) for name in names}
    last = {}
    lock = threading.Lock()
    pending = queue.Queue(maxsize=12)
    stats = dict(saved=0, unmatched=0, dropped=0, invalid=0)
    last_camera_wall = [None]
    deadline = time.monotonic() + args.duration

    def camera(msg, source):
        stamp = msg.header.stamp.to_sec()
        if math.isfinite(stamp) and stamp > 0.:
            with lock:
                frames[source].append((stamp, msg))
                last_camera_wall[0] = time.monotonic()

    def observation(msg):
        try:
            data = json.loads(msg.data)
            source = data['source_uav']
            sample = float(data['sample_s'])
            if (source not in frames or data.get('real_detection') is not True or
                    not math.isfinite(sample)):
                return
            with lock:
                if sample - last.get(source, -float('inf')) < args.interval:
                    return
                nearby = min(frames[source], key=lambda f: abs(f[0]-sample), default=None)
                if nearby is None or abs(nearby[0]-sample) > .05:
                    stats['unmatched'] += 1
                    return
                try:
                    pending.put_nowait((source, sample, nearby, data))
                    last[source] = sample
                except queue.Full:
                    stats['dropped'] += 1
        except (ValueError, TypeError, KeyError):
            stats['invalid'] += 1

    subscribers = [rospy.Subscriber('/%s/stereo_camera/left/image_raw' % name,
                    Image, camera, name, queue_size=1, buff_size=2**22) for name in names]
    subscribers.append(rospy.Subscriber('/swarm/visual_source', String,
                                       observation, queue_size=30))
    try:
        with manifest_path.open('x') as manifest:
            while not rospy.is_shutdown() and stats['saved'] < args.max_images:
                if time.monotonic() >= deadline:
                    stats['stop_reason'] = 'duration'
                    break
                if (last_camera_wall[0] is not None and
                        time.monotonic()-last_camera_wall[0] > 15.):
                    stats['stop_reason'] = 'all_cameras_stopped'
                    break
                try:
                    source, sample, (stamp, msg), data = pending.get(timeout=.25)
                except queue.Empty:
                    continue
                path = args.output / ('%04d_%s.png' % (stats['saved'], source))
                image = bridge.imgmsg_to_cv2(msg, 'bgr8')
                if not cv2.imwrite(str(path), image, [cv2.IMWRITE_PNG_COMPRESSION, 3]):
                    raise OSError('Image save failed: %s' % path)
                row = dict(image=str(path.resolve()), source_uav=source,
                           image_stamp=stamp, observation_stamp=sample,
                           saved_wall_s=time.time(), reviewed=False,
                           observation=data, annotation=None)
                manifest.write(json.dumps(row, ensure_ascii=False)+'\n')
                manifest.flush()
                stats['saved'] += 1
                if stats['saved'] % 25 == 0:
                    print(json.dumps(stats), flush=True)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        (args.output/'summary.json').write_text(json.dumps(stats, indent=2)+'\n')
        print(json.dumps(stats), flush=True)


if __name__ == '__main__':
    main()
