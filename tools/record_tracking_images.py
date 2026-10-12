"""Bounded, read-only image evidence matched to actual tracking-view stamps."""
import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import threading
import time

COLORS = ('green', 'blue', 'brown', 'white', 'red')
SOURCES = tuple('typhoon_h480_%d' % i for i in range(6))
TARGET_COLORS = dict(zip(('t0', 't1', 't2', 't3', 't4', 't5'),
                        ('green', 'blue', 'brown', 'white', 'red', 'red')))


def evidence_color(target):
    """Only local colour/target labels, never simulator actor identities."""
    if not isinstance(target, str):
        raise ValueError('invalid target label')
    label = target.split(':', 1)[0]
    if label in ('red1', 'red2'):
        return 'red'
    return TARGET_COLORS.get(label, label)


class ImageQuota:
    """Bound total images; optional hard reservations protect colours and late time."""

    def __init__(self, limit, seconds, fair=False, windows=3):
        self.limit, self.seconds = limit, seconds
        self.fair, self.windows = fair, windows
        self.count = 0
        self.saved_at, self.latest_source_stamp = {}, {}
        self.used, self.rejected = Counter(), Counter()
        self.caps = {}
        if fair:
            slots = [(window, color, source) for window in range(windows)
                     for color in COLORS for source in SOURCES]
            base, extra = divmod(limit, len(slots))
            self.caps = {slot: base + (i < extra) for i, slot in enumerate(slots)}

    def reserve(self, source, target, stamp, elapsed):
        """Call only after finding the exact camera frame, under the recorder lock."""
        reason, slot = None, None
        color = evidence_color(target)
        if self.fair:
            if source not in SOURCES or color not in COLORS:
                reason = 'unselected_label_or_source'
            elif not math.isfinite(stamp) or stamp <= 0:
                reason = 'invalid_stamp'
            elif not math.isfinite(elapsed) or elapsed < 0 or elapsed >= self.seconds:
                reason = 'outside_duration'
            elif stamp < self.latest_source_stamp.get(source, -math.inf) - 1e-6:
                reason = 'source_stamp_regressed'
            else:
                # Even a full quota must not hide a later source-clock reset.
                self.latest_source_stamp[source] = max(
                    stamp, self.latest_source_stamp.get(source, -math.inf))
                window = min(self.windows - 1, int(elapsed * self.windows / self.seconds))
                slot = (window, color, source)
                if self.used[slot] >= self.caps[slot]:
                    reason = 'fair_slot_full'
        key = (source, color if self.fair else target)
        if reason is None and self.count >= self.limit:
            reason = 'total_limit'
        if reason is None and stamp - self.saved_at.get(key, -1e9) < 2.:
            reason = 'sample_interval'
        if reason is not None:
            self.rejected[reason] += 1
            return None
        index = self.count
        self.count += 1
        self.saved_at[key] = stamp
        if self.fair:
            self.used[slot] += 1
            self.latest_source_stamp[source] = max(
                stamp, self.latest_source_stamp.get(source, -math.inf))
        return index


def matching_image(cache, stamp):
    return next((m for m in reversed(cache)
                 if abs(m.header.stamp.to_sec() - stamp) < 1e-6), None)


def message_callerid(message):
    """Transport identity is separate from, and cannot be supplied by, JSON payload."""
    header = getattr(message, '_connection_header', None)
    callerid = header.get('callerid') if isinstance(header, dict) else None
    return callerid if isinstance(callerid, str) else None


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--seconds', type=float, default=650.)
    parser.add_argument('--limit', type=int, default=180)
    parser.add_argument('--candidates', action='store_true',
                        help='Capture pixel discovery candidates instead of established tracks.')
    parser.add_argument('--fair-quota', action='store_true',
                        help='Reserve the image limit across five colours, six cameras and time windows.')
    parser.add_argument('--quota-windows', type=int, default=3,
                        help='Wall-time quota windows in fair mode; unused early quotas stay reserved.')
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds <= 0 or args.limit < 1:
        parser.error('--seconds must be positive and finite; --limit must be positive')
    if not 1 <= args.quota_windows <= 10:
        parser.error('--quota-windows must be between 1 and 10')
    if args.fair_quota and args.limit < len(COLORS) * len(SOURCES) * args.quota_windows:
        parser.error('fair quota requires at least one image per colour, camera and time window')
    return args


def main():
    args = parse_args()
    # Imports stay local so quota/time matching tests do not require a ROS master.
    import cv2
    from cv_bridge import CvBridge
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    args.output.mkdir(parents=True, exist_ok=True)
    rospy.init_node('tracking_images_read_only', anonymous=True)
    bridge, lock = CvBridge(), threading.Lock()
    cache, stats = {}, Counter()
    quota = ImageQuota(args.limit, args.seconds, args.fair_quota, args.quota_windows)
    started = time.monotonic()
    (args.output/'config.json').write_text(json.dumps(dict(
        limit=args.limit, seconds=args.seconds, candidates=args.candidates,
        fair_quota=args.fair_quota, quota_windows=args.quota_windows,
        colors=COLORS, sources=SOURCES, cache_frames=16,
        quota_clock='monotonic_wall_elapsed', exact_image_stamp_tolerance_s=1e-6,
        subscriptions_only=True), indent=2)+'\n')
    metadata = (args.output/'images.jsonl').open('x')

    def receive_image(msg, uid):
        with lock:
            cache.setdefault(uid, deque(maxlen=16)).append(msg)

    def receive_view(msg):
        try:
            view = json.loads(msg.data)
            uid, stamp = view['source_uav'], float(view['sample_s'])
            tid = (view['candidate_id'].split(':', 1)[0] if args.candidates
                   else view['target_id'])
            with lock:
                image = matching_image(cache.get(uid, ()), stamp)
                if image is None:
                    stats['exact_image_missing'] += 1
                    return
                index = quota.reserve(uid, tid, stamp, time.monotonic()-started)
                if index is None:
                    return
            # Numerical filenames: externally supplied IDs never become paths.
            name = '%04d.jpg' % index
            bgr = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            if not cv2.imwrite(str(args.output/name), bgr, [cv2.IMWRITE_JPEG_QUALITY, 85]):
                raise OSError('image write failed')
            with lock:
                metadata.write(json.dumps(dict(image=name, capture_s=stamp,
                    wall=time.time(), publisher_callerid=message_callerid(msg),
                    view=view), separators=(',', ':'))+'\n')
                metadata.flush()
                stats['images_written'] += 1
        except (ValueError, KeyError, TypeError, OSError) as exc:
            rospy.logwarn_throttle(10., 'image evidence skipped: %s', exc)

    subs = [rospy.Subscriber('/typhoon_h480_%d/stereo_camera/left/image_raw'%i,
                Image, receive_image, 'typhoon_h480_%d'%i, queue_size=1,
                buff_size=2**22) for i in range(6)]
    topic = '/swarm/visual_candidate' if args.candidates else '/swarm/tracking_view'
    subs.append(rospy.Subscriber(topic, String,
                                receive_view, queue_size=4))
    try:
        end = started+args.seconds
        while not rospy.is_shutdown() and time.monotonic() < end:
            time.sleep(.2)
    finally:
        for sub in subs:
            sub.unregister()
        with lock:
            metadata.close()
            summary = dict(stats, images_reserved=quota.count,
                rejected=dict(quota.rejected), used_slots=[dict(
                    window=slot[0], color=slot[1], source=slot[2],
                    used=quota.used[slot], limit=cap) for slot, cap in quota.caps.items()])
            (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')


if __name__ == '__main__':
    main()
