#!/usr/bin/env python3
"""Read-only fast-tracker audit. No flight commands, world truth, or report output.

Example (ROS environment sourced):
  python3 tools/fast_visual_shadow.py --uavs 5 --output /tmp/fast-shadow-01

All results are unreviewed image-space hypotheses. Even a successful track is
ineligible for official coordinate reporting. --help does not require ROS.
"""
import argparse
from collections import Counter, deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import queue
import sys
import threading
import time


@dataclass(frozen=True)
class Frame:
    stamp: float
    payload: object
    receive_wall_s: float = 0.0
    receive_ros_s: float = 0.0


def stamp_key(stamp):
    """Match serialized ROS timestamps, without accepting a neighbouring image."""
    value = float(stamp)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('invalid_image_stamp')
    return int(round(value * 1e9))


class FrameBuffer:
    """Caller provides locking; payloads are retained references, not copies."""
    def __init__(self, max_frames=16, max_age_s=0.8):
        self.max_frames = max_frames
        self.max_age_s = max_age_s
        self.frames = deque(maxlen=max_frames)

    def add(self, frame):
        key = stamp_key(frame.stamp)
        if self.frames and key <= stamp_key(self.frames[-1].stamp):
            return 'duplicate_image' if key == stamp_key(self.frames[-1].stamp) else 'out_of_order_image'
        self.frames.append(frame)
        while self.frames and frame.stamp - self.frames[0].stamp > self.max_age_s + 1e-9:
            self.frames.popleft()
        return None

    def snapshot(self):
        return list(self.frames)


def catchup_window(frames, seed_stamp, max_frames=12, max_age_s=0.6):
    """A seed can only initialize its exact image; never use nearest-frame seeding."""
    key = stamp_key(seed_stamp)
    index = next((i for i, frame in enumerate(frames) if stamp_key(frame.stamp) == key), None)
    if index is None:
        return [], 'seed_image_not_cached'
    result = frames[index:]
    if result[-1].stamp - seed_stamp > max_age_s + 1e-9:
        return [], 'catchup_age_exceeded'
    if len(result) - 1 > max_frames:
        return [], 'catchup_frames_exceeded'
    return result, None


def validate_seed(data, sources):
    if not isinstance(data, dict) or data.get('schema') != 1:
        raise ValueError('unsupported_seed_schema')
    source = data.get('source_uav')
    if source not in sources:
        raise ValueError('unselected_source')
    sample = float(data['image_stamp'])
    stamp_key(sample)
    detections = data.get('detections')
    if not isinstance(detections, list) or len(detections) > 64:
        raise ValueError('invalid_detections')
    selected, seen = [], set()
    for detection in detections:
        if not isinstance(detection, dict):
            raise ValueError('invalid_detection')
        ident = detection.get('id')
        if not isinstance(ident, (str, int)) or isinstance(ident, bool):
            raise ValueError('invalid_detection_id')
        if str(ident) in seen:
            raise ValueError('duplicate_detection_id')
        seen.add(str(ident))
        label = detection.get('label')
        box = detection.get('box')
        conf = float(detection.get('confidence', float('nan')))
        if (not isinstance(label, str) or not isinstance(box, (list, tuple)) or len(box) != 4
                or not all(math.isfinite(float(x)) for x in box)
                or not math.isfinite(conf) or not 0 <= conf <= 1):
            raise ValueError('invalid_detection_geometry')
        x1, y1, x2, y2 = map(float, box)
        if x1 < 0 or y1 < 0 or x2 <= x1 or y2 <= y1:
            raise ValueError('invalid_detection_geometry')
        selected.append(dict(id=ident, label=label, box=[x1, y1, x2, y2], confidence=conf))
    selected.sort(key=lambda detection: detection['confidence'], reverse=True)
    return source, sample, selected[:6], len(selected) - min(len(selected), 6)


class ShadowState:
    """Per-camera tracker lifecycle, independent of ROS and filesystem access."""
    def __init__(self, factory, decode):
        self.factory = factory
        self.decode = decode
        self.tracker = None
        self.last_seed_stamp = -math.inf
        self.last_step_stamp = -math.inf
        self.last_output_stamp = -math.inf
        self.last_seed_results = []
        self.seeded_ids = set()
        self.not_reanchored_ids = []

    def seed(self, frames, seed_stamp, detections):
        self.last_seed_results = []
        self.not_reanchored_ids = []
        if not detections:
            return None, [], 'empty_detector_result_retained_until_ttl'
        if seed_stamp <= self.last_seed_stamp + 1e-9:
            return None, [], 'out_of_order_seed'
        window, reason = catchup_window(frames, seed_stamp)
        if reason:
            return None, [], reason
        if window[-1].stamp < self.last_step_stamp - 1e-9:
            return None, [], 'seed_would_rewind_tracker'
        # Build separately so a delayed detector result cannot rewind active state.
        candidate = self.factory()
        self.last_seed_results = candidate.seed(self.decode(window[0]), seed_stamp, detections)
        new_ids = {str(item["id"]) for item in detections}
        self.not_reanchored_ids = sorted(self.seeded_ids - new_ids)
        self.seeded_ids = new_ids
        results = []
        for frame in window[1:]:
            results = candidate.step(self.decode(frame), frame.stamp)
        self.tracker = candidate
        self.last_seed_stamp = seed_stamp
        self.last_step_stamp = window[-1].stamp
        latest = window[-1]
        if len(window) <= 1 or latest.stamp <= self.last_output_stamp + 1e-9:
            return None, window, None
        self.last_output_stamp = latest.stamp
        return dict(frame=latest, tracks=results, catchup_frames=len(window)-1), window, None

    def step(self, frames):
        if self.tracker is None or not frames:
            return None
        frame = frames[-1]
        if frame.stamp <= self.last_step_stamp + 1e-9:
            return None
        skipped = sum(self.last_step_stamp < item.stamp < frame.stamp for item in frames)
        results = self.tracker.step(self.decode(frame), frame.stamp)
        self.last_step_stamp = frame.stamp
        self.last_output_stamp = frame.stamp
        return dict(frame=frame, tracks=results, skipped_cached_frames=skipped)


class ArchiveWriter:
    """Optional bounded image archive; disk work never runs in camera callbacks."""
    def __init__(self, output, max_frames, decode):
        self.output, self.max_frames, self.decode = output, max_frames, decode
        self.queue = queue.Queue(maxsize=24)
        self.seen = set()
        self.stats = Counter()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, name='shadow_image_archive', daemon=True)
        self.thread.start()

    def submit(self, source, frames, reason):
        for frame in frames:
            key = (source, stamp_key(frame.stamp))
            if key in self.seen or len(self.seen) >= self.max_frames:
                continue
            try:
                self.queue.put_nowait((source, frame, reason))
                self.seen.add(key)
            except queue.Full:
                self.stats['queue_full'] += 1

    def run(self):
        import cv2
        with (self.output / 'frames.jsonl').open('x') as manifest:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    source, frame, reason = self.queue.get(timeout=.1)
                except queue.Empty:
                    continue
                try:
                    if os.statvfs(str(self.output)).f_bavail * os.statvfs(str(self.output)).f_frsize < 256 * 1024**2:
                        self.stats['disk_reserve_rejections'] += 1
                        continue
                    name = '%s_%d.png' % (source, stamp_key(frame.stamp))
                    path = self.output / name
                    if not cv2.imwrite(str(path), self.decode(frame), [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                        raise OSError('imwrite_failed')
                    manifest.write(json.dumps(dict(source_uav=source, image_stamp=frame.stamp,
                        image=name, receive_wall_s=frame.receive_wall_s,
                        receive_ros_s=frame.receive_ros_s, archive_reason=reason,
                        reviewed=False, annotation=None)) + '\n')
                    manifest.flush()
                    self.stats['saved'] += 1
                except Exception as exc:
                    self.stats['write_errors'] += 1
                    manifest.write(json.dumps(dict(event='archive_error', source_uav=source,
                        image_stamp=frame.stamp, reason=str(exc))) + '\n')
                    manifest.flush()
                finally:
                    self.queue.task_done()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=10)
        self.stats['pending_at_shutdown'] = self.queue.qsize()
        self.stats['writer_alive_at_shutdown'] = int(self.thread.is_alive())


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uavs', default='5', help='Comma-separated UAV indices, default: 5')
    parser.add_argument('--output', type=Path, required=True, help='New audit directory')
    parser.add_argument('--duration', type=float, default=180., help='Wall-clock seconds')
    parser.add_argument('--max-hz', type=float, default=20., help='Maximum processing rate per UAV')
    parser.add_argument('--max-recorded-frames', type=int, default=600, help='Total PNG limit, 0 disables images')
    args = parser.parse_args(argv)
    try:
        indices = [int(item) for item in args.uavs.split(',')]
        if not indices or len(indices) != len(set(indices)) or any(item not in range(6) for item in indices):
            raise ValueError()
    except ValueError:
        parser.error('--uavs must contain distinct indices from 0 to 5')
    if (not math.isfinite(args.duration) or args.duration <= 0 or args.max_recorded_frames < 0
            or not math.isfinite(args.max_hz) or not 0 < args.max_hz <= 60):
        parser.error('duration positive; max-recorded-frames nonnegative; max-hz in (0,60]')
    args.sources = ['typhoon_h480_%d' % item for item in indices]
    return args


def main(argv=None):
    args = parse_args(argv)
    for variable in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
        os.environ[variable] = '1'
    import cv2
    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from perception.fast_visual_tracker import FastVisualTracker

    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'config.json').write_text(json.dumps(dict(
        mode='shadow_only', sources=args.sources, duration_wall_s=args.duration, max_hz=args.max_hz,
        max_recorded_frames=args.max_recorded_frames, buffer_frames=16,
        buffer_age_s=.8, catchup_frames=12, catchup_age_s=.6,
        subscribed_topics=['/%s/stereo_camera/left/image_raw' % x for x in args.sources]
            + ['/perception/fast_tracker_seeds'],
        result_meaning='Unreviewed pixel tracks; no world coordinates or official eligibility',
        created_wall_s=time.time()), indent=2) + '\n')
    rospy.init_node('fast_visual_shadow_read_only', anonymous=True)
    buffers = {source: FrameBuffer() for source in args.sources}
    pending_seeds = {source: deque(maxlen=8) for source in args.sources}
    events = deque(maxlen=256)
    stats = Counter()
    lock = threading.Lock()
    bridge = CvBridge()
    archive_bridge = CvBridge()
    decode = lambda frame: bridge.imgmsg_to_cv2(frame.payload, 'bgr8')
    archive_decode = lambda frame: archive_bridge.imgmsg_to_cv2(frame.payload, 'bgr8')
    states = {source: ShadowState(lambda: FastVisualTracker(max_tracks=6), decode) for source in args.sources}
    archive = ArchiveWriter(args.output, args.max_recorded_frames, archive_decode)
    last_camera_wall = [None]
    next_processing = {source: 0. for source in args.sources}

    def event_locked(row):
        if len(events) == events.maxlen:
            stats['event_queue_overflow'] += 1
        stats["event_sequence"] += 1
        row["arrival_sequence"] = stats["event_sequence"]
        events.append(row)

    def camera(msg, source):
        wall = time.time()
        ros = rospy.get_time()
        try:
            frame = Frame(msg.header.stamp.to_sec(), msg, wall, ros)
            with lock:
                reason = buffers[source].add(frame)
                stats[reason or 'received_images'] += 1
                if reason:
                    event_locked(dict(event='image_rejected', source_uav=source,
                        image_stamp=frame.stamp, reason=reason, receive_wall_s=wall))
                else:
                    last_camera_wall[0] = time.monotonic()
                    event_locked(dict(event='image_arrival', source_uav=source,
                        image_stamp=frame.stamp, receive_wall_s=wall, receive_ros_s=ros))
        except (TypeError, ValueError, AttributeError) as exc:
            with lock:
                stats['invalid_images'] += 1
                event_locked(dict(event='image_rejected', source_uav=source, reason=str(exc)))

    def seeds(msg):
        arrival = time.time()
        arrival_ros = rospy.get_time()
        try:
            if len(msg.data) > 65536:
                raise ValueError('oversized_seed_message')
            data = json.loads(msg.data)
            if isinstance(data, dict) and data.get('source_uav') not in buffers:
                with lock:
                    stats['unselected_source_ignored'] += 1
                return
            source, stamp, detections, omitted = validate_seed(data, buffers)
            with lock:
                if len(pending_seeds[source]) == pending_seeds[source].maxlen:
                    stats['seed_queue_overflow'] += 1
                pending_seeds[source].append((stamp, detections, arrival, arrival_ros))
                event_locked(dict(event='seed_arrival', source_uav=source, image_stamp=stamp,
                    detections=detections, omitted_detections=omitted,
                    arrival_wall_s=arrival, arrival_ros_s=arrival_ros))
        except (TypeError, ValueError, KeyError) as exc:
            with lock:
                stats['invalid_seeds'] += 1
                event_locked(dict(event='seed_rejected', reason=str(exc), arrival_wall_s=arrival))

    subscribers = [rospy.Subscriber('/%s/stereo_camera/left/image_raw' % source,
        Image, camera, callback_args=source, queue_size=1, buff_size=2**22) for source in args.sources]
    subscribers.append(rospy.Subscriber('/perception/fast_tracker_seeds', String, seeds, queue_size=16))
    deadline = time.monotonic() + args.duration
    stop_reason = 'duration'
    try:
        with (args.output / 'events.jsonl').open('x') as output:
            def write(row):
                output.write(json.dumps(row, allow_nan=False, default=lambda x: x.item() if hasattr(x, 'item') else str(x)) + '\n')
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                with lock:
                    queued_events = list(events)
                    events.clear()
                    snapshots = {source: buffers[source].snapshot() for source in args.sources}
                    due = [source for source in args.sources if time.monotonic() >= next_processing[source]]
                    queued_seeds = {source: list(pending_seeds[source]) for source in due}
                    for source in due:
                        pending_seeds[source].clear()
                for row in queued_events:
                    write(row)
                did_work = False
                for source in due:
                    next_processing[source] = time.monotonic() + 1. / args.max_hz
                    state, frames = states[source], snapshots[source]
                    started = time.perf_counter()
                    result = None
                    try:
                        # Latest seed supersedes older queued results; all arrivals remain in the audit.
                        candidates = queued_seeds[source]
                        if candidates:
                            candidates.sort(key=lambda row: row[0])
                            stamp, detections, arrival, arrival_ros = candidates[-1]
                            stats['superseded_seeds'] += len(candidates) - 1
                            result, window, reason = state.seed(frames, stamp, detections)
                            write(dict(event='seed_rejected' if reason else 'seed_applied',
                                source_uav=source, image_stamp=stamp, reason=reason,
                                arrival_wall_s=arrival, arrival_ros_s=arrival_ros,
                                processing_wall_s=time.perf_counter()-started,
                                catchup_frames=max(0, len(window)-1),
                                seed_results=state.last_seed_results,
                                dropped_ids=state.not_reanchored_ids,
                                dropped_reason="not_reanchored" if state.not_reanchored_ids else None))
                            stats['seed_rejected' if reason else 'seed_applied'] += 1
                            if not reason and detections:
                                archive.submit(source, window, 'seed_and_catchup')
                            did_work = True
                        if result is None:
                            result = state.step(frames)
                        if result is not None:
                            frame = result.pop('frame')
                            if state.tracker is not None:
                                archive.submit(source, [f for f in frames if state.last_seed_stamp <= f.stamp <= state.last_seed_stamp + .6 + 1e-9], 'active_interval')
                            row = dict(event='tracking_result', source_uav=source,
                                image_stamp=frame.stamp, receive_wall_s=frame.receive_wall_s,
                                processed_ros_s=rospy.get_time(), processed_wall_s=time.time(),
                                processing_wall_s=time.perf_counter()-started,
                                shadow_only=True, official_eligible=False, **result)
                            row['image_age_s'] = row['processed_ros_s'] - frame.stamp
                            write(row)
                            stats['tracking_frames'] += 1
                            did_work = True
                    except Exception as exc:
                        write(dict(event='processing_error', source_uav=source,
                            reason='%s: %s' % (type(exc).__name__, exc)))
                        stats['processing_errors'] += 1
                output.flush()
                if last_camera_wall[0] is not None and time.monotonic()-last_camera_wall[0] > 15:
                    stop_reason = 'all_cameras_stopped'
                    break
                if not did_work:
                    time.sleep(.005)
            if rospy.is_shutdown():
                stop_reason = 'ros_shutdown'
    except KeyboardInterrupt:
        stop_reason = 'interrupted'
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        archive.close()
        (args.output / 'summary.json').write_text(json.dumps(dict(
            stop_reason=stop_reason, stats=dict(stats), archive=dict(archive.stats)), indent=2) + '\n')
        print(json.dumps(dict(stop_reason=stop_reason, stats=dict(stats), archive=dict(archive.stats))), flush=True)


if __name__ == '__main__':
    main()
