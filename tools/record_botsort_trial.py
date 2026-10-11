#!/usr/bin/env python3
"""Read-only development-trial observer. Never publishes algorithm/control data.

Run after sourcing ROS/catkin; captures original timestamps and optional raw
camera JPEGs. Gazebo truth, when enabled, is saved ONLY for offline diagnostics.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
from urllib.parse import urlparse


TOPICS = (
    '/coordination/target_report', '/swarm/target_states', '/swarm/visual_source',
    '/swarm/tracking_view', '/swarm/visual_candidate', '/swarm/discovery_observation',
    '/swarm/discovery_states', '/swarm/processed_camera_frame', '/swarm/assignment',
    '/swarm/uav_status', '/swarm/report_telemetry', '/swarm/visual_conflict',
    '/swarm/visual_streak', '/swarm/reobserve', '/perception/debug_snapshot',
    '/score', '/time_usage', '/left_actors',
) + tuple('/actor_%s_info' % color for color in ('green', 'blue', 'brown', 'white', 'red1', 'red2')) \
  + tuple('/find_actor_%d' % index for index in range(6))


def recording_topics(algorithm_only=False):
    if not algorithm_only:
        return set(TOPICS)
    return {topic for topic in TOPICS
            if topic not in ('/score', '/time_usage', '/left_actors')
            and not topic.startswith('/find_actor_')}


def convert(value):
    if hasattr(value, '__slots__'):
        return {key: convert(getattr(value, key)) for key in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [convert(item) for item in value]
    if isinstance(value, bytes):
        return {'bytes': len(value)}
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def image_sample(msg):
    try:
        value = msg.header.stamp.to_sec()
        return value if math.isfinite(value) and value > 0. else None
    except (AttributeError, TypeError):
        return None


def source_and_sample(topic, payload, msg):
    source, sample = None, None
    if isinstance(payload, dict):
        source = payload.get('source_uav', payload.get('uav'))
        for field in ('sample_s', 'image_stamp', 'image_sample_s'):
            try:
                value = float(payload[field])
                if math.isfinite(value) and value > 0.:
                    sample = value
                    break
            except (KeyError, TypeError, ValueError):
                pass
    if topic.startswith('/typhoon_h480_'):
        source = topic.split('/')[1]
    if sample is None:
        sample = image_sample(msg)
    return source, sample


def original_target_report(payload):
    if not isinstance(payload, dict) or payload.get('real_detection') is not True:
        return False
    candidates = payload.get('candidates')
    if isinstance(candidates, list):
        return any(isinstance(row, dict) and row.get('target_id') for row in candidates)
    return bool(payload.get('target_id'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--master-port', type=int, default=11345)
    parser.add_argument('--duration', type=float, default=900., help='wall seconds; 0 until stopped')
    parser.add_argument('--algorithm-only', action='store_true',
                        help='record onboard/algorithm streams and JPEG; exclude judge feedback and truth')
    parser.add_argument('--truth', action='store_true', help='record Gazebo truth ONLY to disk')
    parser.add_argument('--truth-hz', type=float, default=5.)
    parser.add_argument('--truth-service-hz', type=float, default=0., help='optional read-only fallback')
    parser.add_argument('--image-period', type=float, default=0., help='raw JPEG interval per camera; 0 disabled')
    parser.add_argument('--burst-image-period', type=float, default=0., help='one-camera short replay burst; e.g. 0.2')
    parser.add_argument('--burst-duration', type=float, default=60.)
    parser.add_argument('--burst-start-delay', type=float, default=0., help='wall seconds after observer starts')
    parser.add_argument('--burst-uav', default='', help='one selected camera; otherwise first reporting source')
    parser.add_argument('--burst-on-report', action='store_true', help='wait for selected source original target report')
    parser.add_argument('--max-burst-frames', type=int, default=350)
    parser.add_argument('--max-image-mb', type=float, default=512., help='total saved JPEG byte limit')
    parser.add_argument('--uavs', default=','.join('typhoon_h480_%d' % i for i in range(6)))
    args = parser.parse_args()
    uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    if urlparse(uri).port != args.master_port:
        parser.error('ROS_MASTER_URI does not match requested isolated master port')
    if args.algorithm_only and (args.truth or args.truth_service_hz > 0.):
        parser.error('--algorithm-only conflicts with --truth and --truth-service-hz')
    if args.truth_service_hz and not args.truth:
        parser.error('--truth-service-hz requires --truth')
    for field in ('duration', 'truth_hz', 'truth_service_hz', 'image_period',
                  'burst_image_period', 'burst_duration', 'burst_start_delay', 'max_image_mb'):
        if not math.isfinite(getattr(args, field)) or getattr(args, field) < 0.:
            parser.error('--%s must be finite and nonnegative' % field.replace('_', '-'))
    if args.max_burst_frames < 1:
        parser.error('--max-burst-frames must be positive')
    uavs = tuple(name.strip() for name in args.uavs.split(',') if name.strip())
    if args.burst_uav and args.burst_uav not in uavs:
        parser.error('--burst-uav must be one of --uavs')
    import rospy
    import roslib.message
    rospy.init_node('botsort_trial_observer', anonymous=True)
    args.out.mkdir(parents=True, exist_ok=True)
    event_path = args.out / 'events.jsonl'
    stream = event_path.open('x', encoding='utf8')  # never overwrite prior evidence
    if args.image_period > 0 or args.burst_image_period > 0:
        (args.out / 'images').mkdir(exist_ok=True)
    counters, failures, subscribed = Counter(), Counter(), {}
    pending = queue.Queue(maxsize=4000)
    image_slots = threading.BoundedSemaphore(16)  # bounded raw-frame memory, never block ROS callbacks
    lock = threading.Lock()
    last_images, last_truth = {}, [-float('inf')]
    wall_start = time.time()
    burst = dict(source_uav=args.burst_uav or None, start_wall_s=None, frames=0)
    if args.burst_uav and not args.burst_on_report:
        burst['start_wall_s'] = wall_start+args.burst_start_delay
    root = Path(__file__).resolve().parents[1]
    from write_run_manifest import git_head
    commit = git_head(root)
    hashes = {}
    for relative in ('perception/perception_real.py', 'perception/image_mot.py',
                     'coordination/src/robocup_swarm/scripts/swarm_agent.py',
                     'coordination/src/robocup_swarm/scripts/yolo_target_bridge.py', 'run_match.sh'):
        path = root / relative
        if path.is_file():
            hashes[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
    metadata = dict(schema_version=1, mode='development_simulation', formal_competition=False,
        observer_only=True, algorithm_only=args.algorithm_only,
        master_uri=uri, wall_start_s=wall_start, ros_start_s=rospy.get_time(),
        commit=commit, source_sha256=hashes, arguments=vars(args).copy(),
        truth_time_semantics='Gazebo ModelStates has no header: receive ROS time, not capture time',
        images='original raw-camera frames encoded as JPEG; sparse sampling is not a complete MOT sequence',
        image_burst_time_semantics='duration/start-delay use wall seconds; cadence uses original camera ROS sample; '
                                   'image_burst.frames counts selected attempts, saved events are authoritative')
    metadata['arguments']['out'] = str(args.out)
    (args.out / 'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf8')

    def enqueue(event, msg=None):
        if msg is not None and not image_slots.acquire(blocking=False):
            with lock:
                failures['image_pending_limit'] += 1
            return
        try:
            pending.put_nowait((event, msg))
        except queue.Full:
            if msg is not None:
                image_slots.release()
            with lock:
                failures['writer_queue_full'] += 1

    def writer():
        bridge = None
        while True:
            job = pending.get()
            if job is None:
                break
            event, image = job
            try:
                if image is not None:
                    import cv2
                    if bridge is None:
                        from cv_bridge import CvBridge
                        bridge = CvBridge()
                    array = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
                    ok, encoded = cv2.imencode('.jpg', array, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if not ok:
                        raise OSError('JPEG encoding failed')
                    if counters['saved_image_bytes']+len(encoded) > args.max_image_mb*1024*1024:
                        event.update(image_saved=False, image_error='image_byte_budget', image_file=None)
                        failures['image_byte_budget'] += 1
                    else:
                        (args.out / event['image_file']).write_bytes(encoded.tobytes())
                        event['image_saved'] = True
                        event['jpeg_sha256'] = hashlib.sha256(encoded.tobytes()).hexdigest()
                        counters['saved_image_bytes'] += len(encoded)
                        counters['saved_images'] += 1
                stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
                counters['written_events'] += 1
            except Exception as exc:
                with lock:
                    failures[type(exc).__name__ + ':' + str(exc)[:160]] += 1
            finally:
                if image is not None:
                    image_slots.release()
            pending.task_done()
        stream.flush()
    thread = threading.Thread(target=writer, name='trial-disk-writer', daemon=True)
    thread.start()

    wanted = recording_topics(args.algorithm_only)
    wanted.update('/%s/mavros/local_position/%s' % (uid, part)
                  for uid in uavs for part in ('pose', 'velocity_local'))
    if args.truth:
        wanted.add('/gazebo/model_states')
    if args.image_period > 0 or args.burst_image_period > 0:
        wanted.update('/%s/stereo_camera/left/image_raw' % uid for uid in uavs)

    def callback(msg, topic):
        received, wall = rospy.get_time(), time.time()
        try:
            if topic.endswith('/image_raw'):
                uid = topic.split('/')[1]
                sample = image_sample(msg)
                with lock:
                    in_burst = (args.burst_image_period > 0 and uid == burst['source_uav'] and
                                burst['start_wall_s'] is not None and
                                0. <= wall-burst['start_wall_s'] < args.burst_duration and
                                burst['frames'] < args.max_burst_frames)
                    period = args.burst_image_period if in_burst else args.image_period
                    if period <= 0 or sample is None or sample-last_images.get(uid, -float('inf')) < period:
                        return
                    last_images[uid] = sample
                    if in_burst:
                        burst['frames'] += 1
                filename = 'images/%s_%.9f_%s.jpg' % (uid, sample, getattr(msg.header, 'seq', 0))
                enqueue(dict(topic=topic, message_type=msg._type, source_uav=uid,
                             sample_s=sample, receive_ros_s=received, receive_wall_s=wall,
                             image_kind='burst' if in_burst else 'sparse',
                             image_file=filename, image_header=convert(msg.header),
                             width=msg.width, height=msg.height, encoding=msg.encoding), msg)
            else:
                if topic == '/gazebo/model_states':
                    if wall-last_truth[0] < 1./max(.1, args.truth_hz):
                        return
                    last_truth[0] = wall
                    payload = {'models': {name: dict(pose=convert(pose), twist=convert(twist))
                        for name, pose, twist in zip(msg.name, msg.pose, msg.twist)
                        if name.startswith('actor_') or name in uavs}}
                elif hasattr(msg, 'data') and isinstance(msg.data, str):
                    try:
                        payload = json.loads(msg.data)
                    except ValueError:
                        payload = {'data': msg.data}
                else:
                    payload = convert(msg)
                source, sample = source_and_sample(topic, payload, msg)
                with lock:
                    if (args.burst_image_period > 0 and burst['start_wall_s'] is None and
                            topic == '/coordination/target_report' and source in uavs and
                            sample is not None and original_target_report(payload) and
                            (not burst['source_uav'] or source == burst['source_uav'])):
                        burst.update(source_uav=source, trigger_sample_s=sample,
                                     start_wall_s=max(wall, wall_start+args.burst_start_delay))
                enqueue(dict(topic=topic, message_type=msg._type, source_uav=source,
                             sample_s=sample, receive_ros_s=received,
                             receive_wall_s=wall, payload=payload))
            with lock:
                counters[topic] += 1
        except Exception as exc:
            with lock:
                failures['callback:' + str(exc)[:160]] += 1

    truth_proxy = None
    if args.truth_service_hz > 0:
        from gazebo_msgs.srv import GetModelState
        truth_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    next_truth, next_discovery = 0., 0.
    deadline = time.monotonic()+args.duration if args.duration > 0 else float('inf')
    print('OBSERVER_READY out=%s master=%s algorithm_only=%s truth=%s images_period=%s' %
          (args.out, uri, args.algorithm_only, args.truth, args.image_period), flush=True)
    try:
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if time.monotonic() >= next_discovery:
                next_discovery = time.monotonic()+.5
                try:
                    for topic, type_name in rospy.get_published_topics('/'):
                        if topic not in wanted or topic in subscribed:
                            continue
                        message_class = roslib.message.get_message_class(type_name)
                        if message_class is None:
                            failures['missing_message_type:' + type_name] += 1
                            continue
                        subscribed[topic] = rospy.Subscriber(topic, message_class, callback,
                            callback_args=topic, queue_size=30 if not topic.endswith('/image_raw') else 1,
                            buff_size=8*1024*1024)
                except Exception as exc:
                    failures['discovery:' + str(exc)[:160]] += 1
            if truth_proxy is not None and time.monotonic() >= next_truth:
                next_truth = time.monotonic()+1./max(.1, args.truth_service_hz)
                for i in range(6):
                    try:
                        before = rospy.get_time()
                        result = truth_proxy('actor_%d' % i, 'world')
                        after = rospy.get_time()
                        enqueue(dict(topic='observer/get_model_state', source_uav=None,
                            sample_s=None, receive_ros_s=after, receive_wall_s=time.time(),
                            payload=dict(name='actor_%d' % i, success=result.success,
                                         pose=convert(result.pose), twist=convert(result.twist),
                                         request_ros_s=before, response_ros_s=after)))
                    except Exception as exc:
                        failures['truth_service:' + str(exc)[:160]] += 1
            time.sleep(min(.5, .5/args.truth_service_hz) if args.truth_service_hz > 0 else .5)
    except KeyboardInterrupt:
        pass
    finally:
        for sub in subscribed.values():
            sub.unregister()
        pending.put(None)
        thread.join(timeout=30.)
        if thread.is_alive():
            failures['writer_drain_incomplete'] += 1
        else:
            stream.close()
        metadata.update(wall_end_s=time.time(), ros_end_s=rospy.get_time(),
                        subscribed_topics=sorted(subscribed), missing_topics=sorted(wanted-set(subscribed)),
                        event_counts=dict(counters), observer_failures=dict(failures),
                        image_burst=burst,
                        writer_drained=not thread.is_alive())
        (args.out / 'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf8')
        print('OBSERVER_DONE events=%s failures=%s' % (counters['written_events'], dict(failures)), flush=True)


if __name__ == '__main__':
    main()
