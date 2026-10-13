#!/usr/bin/env python3
"""Bounded fast visual tracking, optionally publishing image-only yaw attention.

Default: record only. ``--publish-yaw`` permits an independently gated bearing
message, never a world position, flight command or official target report.
Only current camera images, genuine detector seeds and own MAVROS poses are read.
"""
import argparse
import ast
from collections import Counter, OrderedDict, deque
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tools.fast_visual_shadow import (ArchiveWriter, Frame, FrameBuffer,
                                      ShadowState, stamp_key, validate_seed)

OUTPUT_TOPIC = '/perception/fast_tracking_view'
SEED_TOPIC = '/perception/fast_tracker_seeds'


@dataclass(frozen=True)
class Calibration:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    optical_to_body: tuple


def load_calibration(path=None):
    """Read existing literal intrinsics/extrinsics without executing ROS/YOLO.

    Fail closed if perception changes these into dynamic expressions: the two
    paths must then be deliberately reconciled, not silently use old defaults.
    """
    tree = ast.parse(Path(path or ROOT / 'perception/perception_real.py').read_text())
    names = {'FX', 'FY', 'CX', 'CY', 'IMG_W', 'IMG_H', 'M_OPT2LINK'}
    values = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        targets = [name.id for target in node.targets for name in ast.walk(target)
                   if isinstance(name, ast.Name)]
        if not names.intersection(targets):
            continue
        expr = node.value
        if targets == ['M_OPT2LINK']:
            if not (isinstance(expr, ast.Call) and isinstance(expr.func, ast.Attribute)
                    and isinstance(expr.func.value, ast.Name)
                    and expr.func.value.id == 'np' and expr.func.attr == 'array'
                    and len(expr.args) == 1 and not expr.keywords):
                raise ValueError('unsupported_camera_extrinsics')
            values['M_OPT2LINK'] = ast.literal_eval(expr.args[0])
        elif len(node.targets) == 1 and isinstance(node.targets[0], (ast.Tuple, ast.List)):
            values.update(zip(targets, ast.literal_eval(expr)))
        else:
            value = ast.literal_eval(expr)
            values.update((name, value) for name in targets)
    if not names.issubset(values):
        raise ValueError('missing_camera_calibration')
    matrix = values['M_OPT2LINK']
    if (len(matrix) != 3 or any(len(row) != 3 for row in matrix)
            or not all(math.isfinite(float(x)) for row in matrix for x in row)):
        raise ValueError('invalid_camera_extrinsics')
    if any(not math.isfinite(float(values[name])) for name in names - {'M_OPT2LINK'}):
        raise ValueError('invalid_camera_intrinsics')
    if min(values['FX'], values['FY'], values['IMG_W'], values['IMG_H']) <= 0:
        raise ValueError('invalid_camera_intrinsics')
    return Calibration(int(values['IMG_W']), int(values['IMG_H']), float(values['FX']),
                       float(values['FY']), float(values['CX']), float(values['CY']),
                       tuple(tuple(float(x) for x in row) for row in matrix))


def normalized_quaternion(values):
    q = tuple(float(x) for x in values)
    if len(q) != 4 or not all(math.isfinite(x) for x in q):
        raise ValueError('invalid_orientation')
    norm = math.sqrt(sum(x*x for x in q))
    if norm < 1e-6:
        raise ValueError('invalid_orientation')
    return tuple(x/norm for x in q)


def slerp(first, second, alpha):
    q0, q1 = normalized_quaternion(first), normalized_quaternion(second)
    dot = sum(a*b for a, b in zip(q0, q1))
    if dot < 0:
        q1, dot = tuple(-x for x in q1), -dot
    dot = max(-1., min(1., dot))
    if dot > .9995:
        return normalized_quaternion([(1-alpha)*a + alpha*b for a, b in zip(q0, q1)])
    theta = math.acos(dot)
    return normalized_quaternion([(math.sin((1-alpha)*theta)*a + math.sin(alpha*theta)*b)
                                  / math.sin(theta) for a, b in zip(q0, q1)])


class PoseBuffer:
    """Own-aircraft orientations only; caller provides locking."""
    def __init__(self):
        self.samples = deque(maxlen=180)

    def add(self, stamp, quaternion):
        stamp_key(stamp)
        quat = normalized_quaternion(quaternion)
        if self.samples and stamp <= self.samples[-1][0]:
            return 'non_increasing_pose'
        self.samples.append((float(stamp), quat))
        while self.samples and stamp - self.samples[0][0] > 3.:
            self.samples.popleft()
        return None

    def at(self, stamp):
        quaternion, reason, _, _ = self.at_with_diagnostics(stamp)
        return quaternion, reason

    def at_with_diagnostics(self, stamp):
        """Also return selection mode and maximum supporting sample distance.

        For interpolation the distance is the farther bracketing timestamp;
        for endpoint reuse it is the absolute image-to-endpoint distance.
        Rejected selections expose the same distance when samples exist.
        """
        if not self.samples or not math.isfinite(stamp):
            return None, 'pose_unavailable', 'unavailable', None
        if stamp < self.samples[0][0] or stamp > self.samples[-1][0]:
            endpoint = self.samples[0] if stamp < self.samples[0][0] else self.samples[-1]
            delta = abs(stamp-endpoint[0])
            mode = 'endpoint_before' if stamp < self.samples[0][0] else 'endpoint_after'
            if delta > .05 + 1e-9:
                return None, 'pose_outside_history', 'rejected_'+mode, delta
            return endpoint[1], None, mode, delta
        for t, q in self.samples:
            if abs(t-stamp) < 1e-9:
                return q, None, 'exact', abs(t-stamp)
        for first, second in zip(self.samples, list(self.samples)[1:]):
            if first[0] <= stamp <= second[0]:
                dt = second[0]-first[0]
                delta = max(stamp-first[0], second[0]-stamp)
                if dt > .15 + 1e-9:
                    return None, 'pose_sample_gap', 'rejected_gap', delta
                return slerp(first[1], second[1], (stamp-first[0])/dt), None, 'interpolated', delta
        return None, 'pose_unavailable', 'unavailable', None


def world_bearing(box, quaternion, calibration):
    """Same optical -> body -> ENU ray as perception.image_world_bearing."""
    u, v = (box[0]+box[2])*.5, (box[1]+box[3])*.5
    optical = ((u-calibration.cx)/calibration.fx, (v-calibration.cy)/calibration.fy, 1.)
    body = tuple(sum(a*b for a, b in zip(row, optical)) for row in calibration.optical_to_body)
    x, y, z, w = normalized_quaternion(quaternion)
    rotation = ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))
    ray = tuple(sum(a*b for a, b in zip(row, body)) for row in rotation)
    if math.hypot(ray[0], ray[1]) < .1 * math.sqrt(sum(x*x for x in ray)):
        raise ValueError('near_vertical_bearing')
    return math.atan2(ray[1], ray[0])


class YawGate:
    """Fresh image-space hypotheses can steer only an independently bound owner."""
    def __init__(self, calibration):
        self.calibration = calibration
        self.last = OrderedDict()

    def build(self, source, track, frame_stamp, now_s, quaternion, width, height):
        try:
            if track.get('status') != 'tracked':
                raise ValueError('not_tracked')
            if (track.get('real_detection') is not False or track.get('report_eligible') is not False
                    or track.get('footpoint_valid') is not False):
                raise ValueError('invalid_eligibility')
            sample, seed, quality = (float(track[name]) for name in ('sample_s', 'seed_s', 'quality'))
            if not all(math.isfinite(x) for x in (sample, seed, quality, now_s)):
                raise ValueError('invalid_timing_or_quality')
            stamp_key(sample)
            stamp_key(seed)
            if stamp_key(sample) != stamp_key(frame_stamp) or sample <= seed:
                raise ValueError('not_a_new_tracked_image')
            if not -.001 <= now_s-sample <= .15 + 1e-9:
                raise ValueError('image_age')
            if sample-seed > .6 + 1e-9 or now_s-seed > .6 + 1e-9:
                raise ValueError('seed_age')
            if not .35 <= quality <= 1.:
                raise ValueError('tracking_quality')
            if width != self.calibration.width or height != self.calibration.height:
                raise ValueError('image_size_changed')
            box = list(map(float, track['box']))
            if (len(box) != 4 or not all(math.isfinite(x) for x in box)
                    or not 0 <= box[0] < box[2] <= width
                    or not 0 <= box[1] < box[3] <= height):
                raise ValueError('invalid_box')
            identity = str(track['id'])
            fields = identity.rsplit(':', 2)
            if (len(fields) != 3 or not fields[0] or ':' in fields[0]
                    or fields[1] not in ('red', 'green', 'blue', 'white', 'brown')
                    or not fields[2].isdigit() or track.get('label') != fields[1]):
                raise ValueError('invalid_identity')
            key = (source, identity)
            if sample <= self.last.get(key, -math.inf):
                raise ValueError('duplicate_or_old_yaw_image')
            if quaternion is None:
                raise ValueError('pose_unavailable')
            bearing = world_bearing(box, quaternion, self.calibration)
            message = dict(schema=1, source_uav=source, sample_s=sample, seed_s=seed,
                           seed_id=identity, seed_candidate_id=':'.join(fields[1:]),
                           candidate_session=fields[0], box=box, width=width, height=height,
                           fx=self.calibration.fx, bearing=bearing, quality=quality,
                           status='tracked', real_detection=False, report_eligible=False,
                           footpoint_valid=False, yaw_only=True)
            self.last[key] = sample
            self.last.move_to_end(key)
            while len(self.last) > 128:
                self.last.popitem(last=False)
            return message, None
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            return None, str(exc)


def allowed_sensor_topics(sources):
    valid = {'typhoon_h480_%d' % i for i in range(6)}
    if not sources or len(set(sources)) != len(sources) or not set(sources) <= valid:
        raise ValueError('invalid_sources')
    return ([('/%s/stereo_camera/left/image_raw' % source) for source in sources]
            + [('/%s/mavros/local_position/pose' % source) for source in sources]
            + [SEED_TOPIC])


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uavs', default='0,1,2,3,4,5')
    parser.add_argument('--output', type=Path, required=True, help='New result directory')
    parser.add_argument('--duration', type=float, default=900., help='Wall seconds')
    parser.add_argument('--max-hz', type=float, default=20.)
    parser.add_argument('--max-recorded-frames', type=int, default=600)
    parser.add_argument('--publish-yaw', action='store_true', help='Enable image-only yaw messages')
    args = parser.parse_args(argv)
    try:
        args.sources = ['typhoon_h480_%d' % int(item) for item in args.uavs.split(',')]
        allowed_sensor_topics(args.sources)
        if (not math.isfinite(args.duration) or args.duration <= 0
                or not math.isfinite(args.max_hz) or not 0 < args.max_hz <= 20
                or args.max_recorded_frames < 0):
            raise ValueError('duration>0; 0<max-hz<=20; max-recorded-frames>=0')
    except ValueError as exc:
        parser.error(str(exc))
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
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from perception.fast_visual_tracker import FastVisualTracker

    calibration = load_calibration()
    args.output.mkdir(parents=True, exist_ok=False)
    paths = ('perception/fast_visual_node.py', 'perception/fast_visual_tracker.py',
             'tools/fast_visual_shadow.py', 'perception/perception_real.py')
    config = dict(mode='yaw_only' if args.publish_yaw else 'record_only',
                  sources=args.sources, duration_wall_s=args.duration, max_hz=args.max_hz,
                  max_recorded_frames=args.max_recorded_frames,
                  image_age_limit_s=.15, seed_age_limit_s=.6, minimum_quality=.35,
                  pose_max_samples=180, pose_history_s=3., pose_endpoint_slack_s=.05,
                  pose_max_gap_s=.15, buffer_frames=16, buffer_age_s=.8,
                  pose_time_delta_meaning='seconds: maximum absolute image-to-supporting-pose sample distance',
                  subscribed_topics=allowed_sensor_topics(args.sources),
                  published_topics=[OUTPUT_TOPIC] if args.publish_yaw else [],
                  allowed_sensor_check='PASS: own camera, own MAVROS orientation, pixel detector seeds only',
                  calibration=calibration.__dict__, created_wall_s=time.time(),
                  source_sha256={name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in paths})
    (args.output/'config.json').write_text(json.dumps(config, indent=2)+'\n')
    rospy.init_node('fast_visual_yaw', anonymous=True)
    publisher = rospy.Publisher(OUTPUT_TOPIC, String, queue_size=12) if args.publish_yaw else None
    buffers = {source: FrameBuffer() for source in args.sources}
    poses = {source: PoseBuffer() for source in args.sources}
    pending = {source: deque(maxlen=8) for source in args.sources}
    stats, events = Counter(), deque(maxlen=512)
    lock = threading.Lock()
    bridge, archive_bridge = CvBridge(), CvBridge()
    decode = lambda frame: bridge.imgmsg_to_cv2(frame.payload, 'bgr8')
    archive_decode = lambda frame: archive_bridge.imgmsg_to_cv2(frame.payload, 'bgr8')
    states = {source: ShadowState(lambda: FastVisualTracker(max_tracks=6), decode) for source in args.sources}
    gate = YawGate(calibration)
    archive = ArchiveWriter(args.output, args.max_recorded_frames, archive_decode)
    last_camera_wall = [None]
    next_processing = {source: 0. for source in args.sources}

    def event_locked(row):
        if len(events) == events.maxlen:
            stats['event_queue_overflow'] += 1
        events.append(row)

    def camera(msg, source):
        frame = Frame(msg.header.stamp.to_sec(), msg, time.time(), rospy.get_time())
        with lock:
            try:
                reason = buffers[source].add(frame)
                stats[reason or 'received_images'] += 1
                if reason:
                    event_locked(dict(event='image_rejected', source_uav=source, reason=reason))
                else:
                    last_camera_wall[0] = time.monotonic()
            except (ValueError, TypeError) as exc:
                event_locked(dict(event='image_rejected', source_uav=source, reason=str(exc)))

    def pose(msg, source):
        q = msg.pose.orientation
        with lock:
            try:
                reason = poses[source].add(msg.header.stamp.to_sec(), (q.x, q.y, q.z, q.w))
                stats[reason or 'received_poses'] += 1
            except (ValueError, TypeError) as exc:
                event_locked(dict(event='pose_rejected', source_uav=source, reason=str(exc)))

    def seeds(msg):
        try:
            if len(msg.data) > 65536:
                raise ValueError('oversized_seed_message')
            data = json.loads(msg.data)
            if isinstance(data, dict) and data.get('source_uav') not in buffers:
                return
            source, stamp, detections, omitted = validate_seed(data, buffers)
            if data.get('real_detection') is not True:
                raise ValueError('seed_not_real_detection')
            with lock:
                if len(pending[source]) == pending[source].maxlen:
                    stats['seed_queue_overflow'] += 1
                pending[source].append((stamp, detections))
                event_locked(dict(event='seed_arrival', source_uav=source, image_stamp=stamp,
                                  receive_ros_s=rospy.get_time(), receive_wall_s=time.time(),
                                  detections=detections, omitted_detections=omitted))
        except (TypeError, ValueError, KeyError) as exc:
            with lock:
                event_locked(dict(event='seed_rejected', reason=str(exc)))

    subscribers = []
    for source in args.sources:
        subscribers.append(rospy.Subscriber('/%s/stereo_camera/left/image_raw' % source,
                           Image, camera, callback_args=source, queue_size=1, buff_size=2**22))
        subscribers.append(rospy.Subscriber('/%s/mavros/local_position/pose' % source,
                           PoseStamped, pose, callback_args=source, queue_size=100))
    subscribers.append(rospy.Subscriber(SEED_TOPIC, String, seeds, queue_size=16))
    deadline, stop_reason = time.monotonic()+args.duration, 'duration'
    try:
        with (args.output/'events.jsonl').open('x') as output:
            def write(row):
                output.write(json.dumps(row, allow_nan=False, default=lambda x: x.item() if hasattr(x, 'item') else str(x))+'\n')
            write(dict(event='sensor_contract', **config))
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                with lock:
                    queued_events = list(events)
                    events.clear()
                    due = [source for source in args.sources if time.monotonic() >= next_processing[source]]
                    snapshots = {source: buffers[source].snapshot() for source in due}
                    seeds_due = {source: list(pending[source]) for source in due}
                    for source in due:
                        pending[source].clear()
                for row in queued_events:
                    write(row)
                did_work = False
                for source in due:
                    next_processing[source] = time.monotonic()+1./args.max_hz
                    state, frames = states[source], snapshots[source]
                    started, result = time.perf_counter(), None
                    try:
                        candidates = seeds_due[source]
                        if candidates:
                            stamp, detections = max(candidates, key=lambda row: row[0])
                            stats['superseded_seeds'] += len(candidates)-1
                            result, window, reason = state.seed(frames, stamp, detections)
                            write(dict(event='seed_rejected' if reason else 'seed_applied', source_uav=source,
                                       image_stamp=stamp, reason=reason, seed_results=state.last_seed_results,
                                       dropped_ids=state.not_reanchored_ids, processing_wall_s=time.perf_counter()-started))
                            if not reason and detections:
                                archive.submit(source, window, 'seed_and_catchup')
                            did_work = True
                        if result is None:
                            result = state.step(frames)
                        if result is None:
                            continue
                        frame = result.pop('frame')
                        now_s = rospy.get_time()
                        with lock:
                            quaternion, pose_reason, pose_mode, pose_time_delta = poses[source].at_with_diagnostics(frame.stamp)
                        if state.tracker is not None:
                            archive.submit(source, [f for f in frames if state.last_seed_stamp <= f.stamp <= state.last_seed_stamp+.6+1e-9], 'active_interval')
                        for track in result['tracks']:
                            payload, reason = gate.build(source, track, frame.stamp, now_s, quaternion,
                                                         frame.payload.width, frame.payload.height)
                            if reason == 'pose_unavailable':
                                reason = pose_reason or reason
                            published = bool(payload is not None and publisher is not None)
                            if published:
                                publisher.publish(String(data=json.dumps(payload, allow_nan=False)))
                            stats['yaw_published' if published else 'yaw_eligible_shadow' if payload else 'yaw_rejected'] += 1
                            write(dict(event='yaw_published' if published else 'yaw_eligible_shadow' if payload else 'yaw_rejected',
                                       source_uav=source, sample_s=frame.stamp, reason=reason,
                                       processed_ros_s=now_s, image_age_s=now_s-frame.stamp,
                                       pose_mode=pose_mode, pose_time_delta=pose_time_delta,
                                       payload=payload, track=track))
                        write(dict(event='tracking_result', source_uav=source, image_stamp=frame.stamp,
                                   receive_ros_s=frame.receive_ros_s, receive_wall_s=frame.receive_wall_s,
                                   processed_ros_s=now_s, processed_wall_s=time.time(), image_age_s=now_s-frame.stamp,
                                   processing_wall_s=time.perf_counter()-started, orientation=quaternion,
                                   pose_reason=pose_reason, pose_mode=pose_mode, pose_time_delta=pose_time_delta,
                                   official_eligible=False, **result))
                        stats['tracking_frames'] += 1
                        did_work = True
                    except Exception as exc:
                        # A processing failure cannot leave a track silently alive.
                        states[source] = ShadowState(lambda: FastVisualTracker(max_tracks=6), decode)
                        write(dict(event='processing_error', source_uav=source, reason='%s: %s' % (type(exc).__name__, exc)))
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
        summary = dict(stop_reason=stop_reason, stats=dict(stats), archive=dict(archive.stats))
        (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
