#!/usr/bin/env python3
"""Actual bridge/manager ROS wiring with synthetic camera samples, no aircraft."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[2]
    scripts = repo/'coordination/src/robocup_swarm/scripts'
    sys.path.insert(0, str(scripts))
    run_id = str(uuid.uuid4())
    fleet = ['uav_%d' % i for i in range(1, 7)]
    metadata = output/'metadata.json'
    metadata.write_text(json.dumps(dict(schema='robocup_training_worlds/metadata/v1',
        frame=dict(frame_id='map', convention='ENU', units='m'),
        bounds=dict(x_min=-10., x_max=10., y_min=-10., y_max=10.),
        grid=dict(width=40, height=40, resolution_m=.5, origin=[-10., -10.], frame_id='map',
                  encoding='rle_pairs', data=[[0, 1600]]), obstacles=[], spawn=dict(center=[0.,0.]),
        goal_candidates=[dict(id='synthetic', center=[2.,3.], outdoors=True, inside_obstacle=None)],
        generator=dict(name='SYNTHETIC_VISUAL_WIRING_ONLY'))))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11385', ROBOCUP_RUN_ID=run_id,
        SWARM_UAV_IDS=','.join(fleet), ROBOCUP_METADATA=str(metadata), ROBOCUP_LOG_DIR=str(output/'algorithm'),
        SEED_TRUTH='0', VIS_ENABLE='0')
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 11385))
    log = (output/'roscore.log').open('w')
    master = subprocess.Popen(['roscore', '-p', '11385'], stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True)
    result = dict(visual_ros_wiring_verified=False, synthetic_observations=True,
                  physical_vision_verified=False, formal_competition_pass=False, run_id=run_id)
    try:
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            with socket.socket() as probe:
                if probe.connect_ex(('127.0.0.1', 11385)) == 0:
                    break
            if master.poll() is not None:
                raise RuntimeError('Owned ROS master exited')
            time.sleep(.1)
        import rospy
        from std_msgs.msg import String
        from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
        from robocup_swarm.msg import TargetDetection, TargetState
        from swarm_manager import SwarmManager
        from yolo_target_bridge import YoloTargetBridge
        rospy.init_node('visual_evidence_smoke', disable_signals=True)
        manager, bridge = SwarmManager(fleet), YoloTargetBridge()
        manager._left_cb(String(data='[]'))
        assert not manager._mission_finished
        detections, official, states = [], [], []
        subscribers = [rospy.Subscriber('/swarm/detection', TargetDetection, detections.append),
                       rospy.Subscriber('/actor_red1_info', ActorInfo, official.append),
                       rospy.Subscriber('/swarm/target_states', TargetState, states.append)]
        publisher = rospy.Publisher('/swarm/visual_observation', String, queue_size=10)
        left = rospy.Publisher('/left_actors', String, queue_size=10)
        time.sleep(.6)
        left.publish(String(data='[0, 1, 2, 3, 4, 5]'))
        time.sleep(.2)
        message = None
        for seq in range(1, 4):
            stamp = rospy.Time.now().to_sec()
            message = dict(schema_version=2, run_id=run_id, uav_id='uav_1', seq=seq,
                sample_s=stamp, target_id='red1', frame_id='world_enu', xyz=[2., 3., 0.], confidence=.9,
                observation_id='%s:uav_1:%d' % (run_id, seq))
            publisher.publish(String(data=json.dumps(message)))
            time.sleep(.2)
        assert detections and all(m.target_id == 't5' and m.uav_id == 'uav_1' and m.source == 1 for m in detections)
        assert official and all(m.cls == 'red' for m in official)
        assert 't5' in manager.tracker.targets
        before = len(detections)
        publisher.publish(String(data=json.dumps(message)))
        publisher.publish(String(data=json.dumps(dict(message, run_id='old'))))
        time.sleep(.3)
        assert len(detections) == before
        time.sleep(1.0)
        stopped_count = len(official)
        time.sleep(.4)
        assert len(official) == stopped_count
        assert states and states[-1].header.stamp.to_sec() == message['sample_s']
        # Completion must fence busy aircraft without waiting for allocation.
        from task_authority import TaskGate
        task = dict(cell_ix=0, cell_iy=0, target_x=2., target_y=3., task_type=1, target_id='t5')
        with manager._authority_lock:
            manager._authority.offer('uav_1', task, rospy.Time.now().to_sec())
        manager._left_cb(String(data='[5]'))
        manager._left_cb(String(data='[]'))
        assert manager._mission_finished and manager._authority.closed
        assert manager._authority.active['uav_1']['stopping'] and not manager._authority.pending
        manager._left_cb(String(data='[]'))
        result.update(visual_ros_wiring_verified=True, replay_and_old_run_rejected=True,
            coast_does_not_extend_official_reports=True, official_red_class='red', red1_target_id='t5',
            detections=len(detections), official_messages=len(official), state_messages=len(states))
        result['busy_fleet_official_completion_fenced'] = True
        result['source_sha256'] = {str(path.relative_to(repo)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [scripts/'visual_observation.py', scripts/'yolo_target_bridge.py', scripts/'swarm_manager.py']}
        for sub in subscribers:
            sub.unregister()
        bridge._timer.shutdown()
        bridge._timer.join(timeout=2)
        print(json.dumps(result))
    finally:
        (output/'result.json').write_text(json.dumps(result, indent=2))
        if master.poll() is None:
            os.killpg(master.pid, signal.SIGTERM)
            try:
                master.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(master.pid, signal.SIGKILL)
                master.wait()
        log.close()


if __name__ == '__main__':
    main()
