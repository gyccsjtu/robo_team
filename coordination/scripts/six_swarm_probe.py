"""Observe the real manager/agent search chain in the empty six-aircraft fixture."""
import json
import hashlib
import math
from pathlib import Path
import shutil
import time
import uuid
from fixture_contacts import classify, sonar_virtual_collisions
from evidence_writer import JsonlEvidence


def run(out, wiring, spawn, env, seconds, actor_probe=False):
    import rospy
    from gazebo_msgs.msg import ModelStates, ContactsState
    from mavros_msgs.msg import State, PositionTarget
    from nav_msgs.msg import OccupancyGrid
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
    clearance = out / 'startup_clearance.json'
    world = out / 'connectivity.world'
    obstacle_fixture = 'fixture_box_' in world.read_text()
    clearance.write_text(json.dumps(dict(schema_version=2 if obstacle_fixture else 1, run_id=flight_env['ROBOCUP_RUN_ID'],
        world_path=str(world), world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
        positions={row['uav_id']: [-20 + index*8, 0.] for index, row in enumerate(wiring['uavs'])})))
    flight_env.update(ONLINE_RADAR_PLANNING='1', RADAR_START_CLEARANCE_FILE=str(clearance))
    flight_env.pop('SWARM_CALIB', None)
    flight_env['PYTHONPATH'] = str(snapshot / 'coordination/src/robocup_navigation/src') + ':' + env.get('PYTHONPATH', '')
    scripts = snapshot / 'coordination/src/robocup_swarm/scripts'
    ids = [r['uav_id'] for r in wiring['uavs']]
    truth = []
    grants = set()
    status = {}
    armed_uavs = set()
    completion_messages = []
    observed_maps = {}
    contact_samples, box_contacts = {}, []
    virtual_contacts = set()
    sonar_intersections = 0
    contact_record = JsonlEvidence(out / 'fixture_contacts.jsonl') if obstacle_fixture else None
    virtual = set()
    if obstacle_fixture:
        model_root = next(Path(p) for p in env['GAZEBO_MODEL_PATH'].split(':') if p and (Path(p)/'sonar/model.sdf').is_file())
        virtual = sonar_virtual_collisions(model_root, ids)
        shutil.copy2(model_root/'sonar/model.sdf', out/'sonar_source.sdf')
    record = JsonlEvidence(out / 'six_truth.jsonl')
    execution_record = JsonlEvidence(out/'executor_telemetry.jsonl')
    command_counts = {uid: 0 for uid in ids}
    actor_observer = None

    def execution_cb(msg, uid):
        if actor_observer is not None:
            actor_observer.observe_command(uid, msg)
        command_counts[uid] += 1
        execution_record.write(dict(kind='raw_setpoint', uav_id=uid,
            sample_s=msg.header.stamp.to_sec(), frame=msg.coordinate_frame, mask=msg.type_mask,
            local_position=[msg.position.x,msg.position.y,msg.position.z],
            velocity=[msg.velocity.x,msg.velocity.y,msg.velocity.z], yaw_rate=msg.yaw_rate))

    def motion_cb(msg):
        execution_record.write(dict(kind='motion_state', received_s=rospy.Time.now().to_sec(),
                                               message=json.loads(msg.data)))

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
            record.write(sample)

    def assignment_cb(message):
        command = json.loads(message.data)
        if command.get('action') == 'GRANT':
            grants.add(command['uav_id'])

    subscriptions = [rospy.Subscriber('/gazebo/model_states', ModelStates, truth_cb),
                     rospy.Subscriber('/swarm/motion_state', String, motion_cb, queue_size=100),
                     rospy.Subscriber('/swarm/authorized_assignment', String, assignment_cb),
                     rospy.Subscriber('/swarm/finish', String, lambda msg: completion_messages.append(msg.data)),
                     rospy.Subscriber('/swarm/uav_status', UavStatus, lambda msg: status.update({msg.uav_id: msg}))]
    if obstacle_fixture:
        def contact_cb(msg, index):
            nonlocal sonar_intersections
            contact_samples[index] = msg.header.stamp.to_sec()
            for state in msg.states:
                kind = classify(state.collision1_name, state.collision2_name, ids, virtual)
                item = dict(sample_s=msg.header.stamp.to_sec(), box=index, classification=kind,
                            collision1=state.collision1_name, collision2=state.collision2_name)
                contact_record.write(item)
                if kind == 'UAV_BODY_CONTACT':
                    box_contacts.append(item)
                elif kind == 'SONAR_SENSOR_INTERSECTION':
                    sonar_intersections += 1
                    virtual_contacts.add(state.collision1_name if state.collision1_name in virtual else state.collision2_name)
        for index in range(6):
            subscriptions.append(rospy.Subscriber('/fixture/contacts/fixture_box_%d' % index,
                                                  ContactsState, contact_cb, callback_args=index))
    for row in wiring['uavs']:
        subscriptions.append(rospy.Subscriber(row['mavros_namespace']+'/setpoint_raw/local', PositionTarget,
                                              execution_cb, callback_args=row['uav_id'], queue_size=10))
        def arm_cb(message, uid=row['uav_id']):
            if message.armed:
                armed_uavs.add(uid)
        subscriptions.append(rospy.Subscriber(row['mavros_namespace'] + '/state', State, arm_cb))
        subscriptions.append(rospy.Subscriber('/' + row['uav_id'] + '/radar_observed_map', OccupancyGrid,
                             lambda msg, uid=row['uav_id']: observed_maps.update({uid: msg})))
    processes = []
    try:
        if actor_probe:
            from flight_actor_probe import FlightActorProbe
            actor_observer = FlightActorProbe(out, snapshot, flight_env, spawn)
            processes.extend(actor_observer.processes)
            (out / 'swarm_source_manifest.json').write_text(json.dumps({str(p.relative_to(snapshot)):
                hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.rglob('*') if p.is_file()}, indent=2))
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
            if actor_observer is not None and truth and 0 <= rospy.Time.now().to_sec()-truth[-1]['sim_s'] <= .2:
                actor_observer.tick(truth[-1]['positions'])
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
        if min_separation <= 4.5:
            reasons.append('SPACING_VIOLATION')
        sample_gap = max((b['sim_s'] - a['sim_s'] for a, b in zip(truth, truth[1:])), default=float('inf'))
        if sample_gap > .2:
            reasons.append('TRAJECTORY_EVIDENCE_GAP')
        if 'MISSION_FINISHED' in completion_messages:
            reasons.append('UNVERIFIED_MISSION_FINISHED_IN_FIXTURE')
        map_counts = {uid: dict(unknown=msg.data.count(-1), free=msg.data.count(0), occupied=msg.data.count(100),
            frame=msg.header.frame_id, sample_s=msg.header.stamp.to_sec(), version=msg.header.seq)
                      for uid, msg in observed_maps.items()}
        map_verified = (set(observed_maps) == set(ids)
                        and all(c['unknown'] > 0 and c['free'] > 0 and c['frame'] == 'map'
                                and 0 <= rospy.Time.now().to_sec()-c['sample_s'] <= 1. for c in map_counts.values()))
        if not map_verified:
            reasons.append('ONLINE_OBSERVATION_MAP_MISSING_OR_STALE')
        route_commits = {uid: (out / ('agent_'+uid+'.log')).read_text(errors='replace').count('ROUTE_COMMITTED') for uid in ids}
        online_plans = {uid: (out / ('agent_'+uid+'.log')).read_text(errors='replace').count('ONLINE_PLAN') for uid in ids}
        if not all(route_commits.values()):
            reasons.append('ACTUAL_ROUTE_COMMIT_EVIDENCE_MISSING')
        if not all(online_plans.values()):
            reasons.append('ONLINE_PLANNING_EVIDENCE_MISSING')
        if not all(command_counts.values()):
            reasons.append('ACTUAL_RAW_SETPOINT_EVIDENCE_MISSING')
        contact_verified = (obstacle_fixture and len(contact_samples) == 6
            and all(0 <= rospy.Time.now().to_sec()-stamp <= .5 for stamp in contact_samples.values()))
        min_box_clearance = None
        if obstacle_fixture:
            min_box_clearance = min(math.hypot(max(abs(p[0]-(-20+index*8))-1.5, 0),
                                              max(abs(p[1]+9)-1.5, 0))
                for sample in truth for p in sample['positions'].values() for index in range(6))
            if min_box_clearance <= 1.2:
                reasons.append('BOX_PROTECTIVE_CLEARANCE_VIOLATION')
            if box_contacts:
                reasons.append('BOX_CONTACT_OBSERVED')
            if not contact_verified:
                reasons.append('BOX_CONTACT_EVIDENCE_MISSING_OR_STALE')
            if not any(count['occupied'] > 0 for count in map_counts.values()):
                reasons.append('RADAR_OBSTACLE_OBSERVATION_MISSING')
        actor_result = actor_observer.summary(truth) if actor_observer is not None else {}
        if actor_probe and not actor_result['physical_visual_tracking_verified']:
            reasons.append('PHYSICAL_VISUAL_TRACKING_EVIDENCE_INCOMPLETE')
        verified = not reasons
        return dict(status='SIX_SEARCH_FLIGHT_OBSERVED' if verified else 'SIX_SEARCH_FLIGHT_INCOMPLETE',
                    prototype_search_verified=verified, simulated_seconds=rospy.Time.now().to_sec()-start,
                    maximum_true_altitude_m=maximum_z, minimum_sampled_separation_m=min_separation,
                    maximum_displacement_m=displacement, climbed=climbed, granted_uavs=sorted(grants),
                    truth_samples=len(truth), collision_result='ABSTAIN_NO_CONTACT_EVIDENCE',
                    failure_reasons=reasons, maximum_truth_gap_s=sample_gap,
                    armed_during_run_uavs=sorted(armed_uavs),
                    mission_completion_messages=completion_messages,
                    online_observation_maps_verified=map_verified, online_map_counts=map_counts,
                    route_commits=route_commits, online_plans=online_plans,
                    raw_setpoint_counts=command_counts,
                    obstacle_fixture=obstacle_fixture, minimum_box_clearance_m=min_box_clearance,
                    box_contact_observation_verified=contact_verified, box_contacts=box_contacts,
                    sonar_sensor_intersections=sonar_intersections, verified_virtual_collision_names=sorted(virtual_contacts),
                    formal_competition_pass=False, fixture_only=True, **actor_result)
    finally:
        if actor_observer is not None:
            actor_observer.close()
        for subscriber in subscriptions:
            subscriber.unregister()
        record.close()
        execution_record.close()
        if contact_record is not None:
            contact_record.close()
