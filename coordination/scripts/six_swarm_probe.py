"""Observe the real manager/agent search chain in the empty six-aircraft fixture."""
import json
import hashlib
import math
from pathlib import Path
import shutil
import time
import uuid


def run(out, wiring, spawn, env, seconds):
    import rospy
    from gazebo_msgs.msg import ModelStates
    from mavros_msgs.msg import State
    from robocup_swarm.msg import UavStatus
    from std_msgs.msg import String
    repo = Path(__file__).resolve().parents[2]
    snapshot = out / 'swarm_sources'
    for relative in ('coordination/src/robocup_swarm/scripts', 'coordination/src/robocup_navigation/src'):
        shutil.copytree(repo / relative, snapshot / relative,
                        ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '*.bak*'))
    (out / 'swarm_source_manifest.json').write_text(json.dumps({str(p.relative_to(snapshot)):
        hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.rglob('*') if p.is_file()}, indent=2))
    # Explicit empty-world fixture, never the metadata for a random competition world.
    metadata = dict(schema='robocup_training_worlds/metadata/v1',
        frame=dict(frame_id='map', convention='ENU', units='m'),
        bounds=dict(x_min=-28., x_max=28., y_min=-28., y_max=28.),
        grid=dict(width=112, height=112, resolution_m=.5, origin=[-28., -28.],
                  frame_id='map', encoding='rle_pairs', data=[[0, 12544]]),
        obstacles=[], spawn=dict(center=[0., 0.]),
        goal_candidates=[dict(id='fixture', center=[0., 10.], outdoors=True, inside_obstacle=None)],
        generator=dict(name='EMPTY_WORLD_DEVELOPMENT_FIXTURE'))
    path = out / 'fixture_metadata.json'
    path.write_text(json.dumps(metadata))
    flight_env = dict(env, ROBOCUP_RUN_ID=str(uuid.uuid4()), ROBOCUP_LOG_DIR=str(out / 'algorithm'),
        ROBOCUP_METADATA=str(path), ROBOCUP_WS=str(snapshot / 'coordination'), SEED_TRUTH='0',
        VIS_ENABLE='0', RADAR_GUARD='1', SWARM_MAX_SPEED='1.0', ALT_BASE='4.5',
        SWARM_UAV_IDS=','.join(r['uav_id'] for r in wiring['uavs']),
        ALT_NLAYER='1', ALT_TARGET_CAP='4.5', SWARM_ARRIVE_TOL='0.15')
    flight_env.pop('SWARM_CALIB', None)
    flight_env['PYTHONPATH'] = str(snapshot / 'coordination/src/robocup_navigation/src') + ':' + env.get('PYTHONPATH', '')
    scripts = snapshot / 'coordination/src/robocup_swarm/scripts'
    ids = [r['uav_id'] for r in wiring['uavs']]
    truth = []
    grants = set()
    status = {}
    armed_uavs = set()
    completion_messages = []
    record = (out / 'six_truth.jsonl').open('w')

    def truth_cb(message):
        now = rospy.Time.now().to_sec()
        if truth and now - truth[-1]['sim_s'] < .05:
            return
        sample = dict(sim_s=now, positions={})
        for uid in ids:
            if uid in message.name:
                pose = message.pose[message.name.index(uid)].position
                sample['positions'][uid] = [pose.x, pose.y, pose.z]
        if len(sample['positions']) == 6:
            truth.append(sample)
            record.write(json.dumps(sample) + '\n')

    def assignment_cb(message):
        command = json.loads(message.data)
        if command.get('action') == 'GRANT':
            grants.add(command['uav_id'])

    subscriptions = [rospy.Subscriber('/gazebo/model_states', ModelStates, truth_cb),
                     rospy.Subscriber('/swarm/authorized_assignment', String, assignment_cb),
                     rospy.Subscriber('/swarm/finish', String, lambda msg: completion_messages.append(msg.data)),
                     rospy.Subscriber('/swarm/uav_status', UavStatus, lambda msg: status.update({msg.uav_id: msg}))]
    for row in wiring['uavs']:
        def arm_cb(message, uid=row['uav_id']):
            if message.armed:
                armed_uavs.add(uid)
        subscriptions.append(rospy.Subscriber(row['mavros_namespace'] + '/state', State, arm_cb))
    processes = []
    try:
        processes.append(spawn(['python3', str(scripts / 'swarm_manager.py'), '_uav_ids:=' + ','.join(ids)], 'swarm_manager', flight_env))
        for index, row in enumerate(wiring['uavs']):
            uid = row['uav_id']
            processes.append(spawn(['python3', str(scripts / 'swarm_agent.py'), '__name:=swarm_agent_' + uid,
                '_uav_id:=' + uid, '_model_name:=' + uid, '_mavros_namespace:=' + row['mavros_namespace'],
                '_scan_topic:=' + row['scan_topic'], '_world_offset_x:=' + str(-20 + index * 8),
                '_world_offset_y:=0'], 'agent_' + uid, flight_env))
        start = rospy.Time.now().to_sec()
        wall_deadline = time.monotonic() + seconds * 5 + 60
        while rospy.Time.now().to_sec() - start < seconds:
            if time.monotonic() > wall_deadline:
                raise RuntimeError('SIX_SEARCH_SIM_TIME_TIMEOUT')
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('SIX_SEARCH_CONTROLLER_EXIT')
            time.sleep(.2)
        if not truth:
            raise RuntimeError('SIX_SEARCH_TRUTH_MISSING')
        maximum_z = max(p[2] for s in truth for p in s['positions'].values())
        min_separation = min(math.dist(s['positions'][a], s['positions'][b])
            for s in truth for i, a in enumerate(ids) for b in ids[i+1:])
        displacement = {uid: max(math.dist(s['positions'][uid][:2], truth[0]['positions'][uid][:2]) for s in truth) for uid in ids}
        climbed = {uid: max(s['positions'][uid][2] for s in truth) > 4. for uid in ids}
        observed = set(status) == set(ids) and grants == set(ids)
        reasons = []
        if not observed:
            reasons.append('MISSING_FLEET_STATUS_OR_AUTHORIZATION')
        if not all(climbed.values()):
            reasons.append('TAKEOFF_INCOMPLETE')
        if not all(v > 1. for v in displacement.values()):
            reasons.append('TASK_MOTION_INCOMPLETE')
        if maximum_z >= 6:
            reasons.append('ALTITUDE_LIMIT_VIOLATION')
        if min_separation <= 3:
            reasons.append('SPACING_VIOLATION')
        sample_gap = max((b['sim_s'] - a['sim_s'] for a, b in zip(truth, truth[1:])), default=float('inf'))
        if sample_gap > .2:
            reasons.append('TRAJECTORY_EVIDENCE_GAP')
        if 'MISSION_FINISHED' in completion_messages:
            reasons.append('UNVERIFIED_MISSION_FINISHED_IN_FIXTURE')
        verified = not reasons
        return dict(status='SIX_SEARCH_FLIGHT_OBSERVED' if verified else 'SIX_SEARCH_FLIGHT_INCOMPLETE',
                    prototype_search_verified=verified, simulated_seconds=rospy.Time.now().to_sec()-start,
                    maximum_true_altitude_m=maximum_z, minimum_sampled_separation_m=min_separation,
                    maximum_displacement_m=displacement, climbed=climbed, granted_uavs=sorted(grants),
                    truth_samples=len(truth), collision_result='ABSTAIN_NO_CONTACT_EVIDENCE',
                    failure_reasons=reasons, maximum_truth_gap_s=sample_gap,
                    armed_during_run_uavs=sorted(armed_uavs),
                    mission_completion_messages=completion_messages,
                    formal_competition_pass=False, fixture_only=True)
    finally:
        for subscriber in subscriptions:
            subscriber.unregister()
        record.close()
