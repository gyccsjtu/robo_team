"""Six real aircraft in an isolated platform city, with platform target/judge copies."""
import hashlib
import json
import math
from pathlib import Path
import shutil
import time
import uuid
import queue
import threading


def bounded_query(function, timeout=5.):
    """A dead Gazebo service must not freeze the run's failure detector."""
    result = queue.Queue(maxsize=1)
    def worker():
        try:
            result.put((True, function()))
        except Exception as error:
            result.put((False, error))
    threading.Thread(target=worker, daemon=True).start()
    try:
        success, value = result.get(timeout=timeout)
    except queue.Empty:
        raise TimeoutError('CITY_MODEL_STATE_TIMEOUT') from None
    if not success:
        raise value
    return value
from evidence_writer import JsonlEvidence, audit_jsonl


def run(out, wiring, spawn, env, seconds, city, owned_health_check=None):
    import rospy
    from gazebo_msgs.srv import GetModelState, GetPhysicsProperties, SetPhysicsProperties
    from mavros_msgs.msg import State, PositionTarget
    from std_msgs.msg import String, Int16
    repo=Path(__file__).resolve().parents[2]
    snapshot=out/'swarm_sources'
    for relative in ('coordination/src/robocup_swarm/scripts', 'coordination/src/robocup_navigation/src', 'perception'):
        shutil.copytree(repo/relative,snapshot/relative,
                        ignore=shutil.ignore_patterns('__pycache__','*.pyc','*.bak*'))
    (out/'swarm_source_manifest.json').write_text(json.dumps({str(p.relative_to(snapshot)):
        hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.rglob('*') if p.is_file()},indent=2))
    scene=json.loads((city/'scene_manifest.json').read_text())
    run_id=str(uuid.uuid4())
    metadata=out/'city_bounds.json'
    width,height=720,480
    metadata.write_text(json.dumps(dict(schema='robocup_training_worlds/metadata/v1',
        frame=dict(frame_id='map',convention='ENU',units='m'),
        bounds=dict(x_min=-50.,x_max=130.,y_min=-60.,y_max=60.),
        grid=dict(width=width,height=height,resolution_m=.25,origin=[-50.125,-60.125],
                  frame_id='map',encoding='rle_pairs',data=[[0,width*height]]),
        obstacles=[],spawn=dict(center=[0.,0.]),goal_candidates=[],
        generator=dict(name='BOUNDS_ONLY_NOT_OBSTACLE_PRIOR'))))
    world=out/'connectivity.world'
    clearance=out/'startup_clearance.json'
    clearance.write_text(json.dumps(dict(schema_version=3,purpose='DEVELOPMENT_CITY_STARTUP',
        run_id=run_id,world_path=str(world),world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
        positions=scene['positions'],startup_checks=scene['startup_checks'])))
    flight_env=dict(env,ROBOCUP_RUN_ID=run_id,ROBOCUP_LOG_DIR=str(out/'algorithm'),
        ROBOCUP_METADATA=str(metadata),ROBOCUP_WS=str(snapshot/'coordination'),
        SWARM_UAV_IDS=','.join(r['uav_id'] for r in wiring['uavs']),
        ONLINE_RADAR_PLANNING='1',RADAR_START_CLEARANCE_FILE=str(clearance),
        SEED_TRUTH='0',VIS_ENABLE='0',RADAR_GUARD='1',SWARM_MAX_SPEED='3.0',PR_RECENT_MOTION_WINDOW='4.0',FLEE_CHASE_SPEED='2.6',
        # The last real flight reached world z=6.019 while MAVROS reported
        # z=4.747. Reserve height for that observed estimator discrepancy;
        # world truth remains an independent audit input, not a controller input.
        # v1.15 hit a fast-food protrusion below 3m with its leg while
        # the body was at world 3.15m and the lidar above that protrusion.
        # Local 2.8m produced world ~3.0-3.4m. Trial local 2.2m, aiming
        # for the teammate's *actual* 2.5-3m suggestion; audit world height
        # independently. No Gazebo truth feeds this height controller.
        ALT_BASE='2.2',ALT_NLAYER='1',ALT_TARGET_CAP='2.2',CLIMB_DONE_ALT='2.0',
        ALT_HARD_CEIL='4.1',ALT_PANIC='4.3',ALT_EMERG_CEIL='4.5',
        SWARM_ROUTE_CLEARANCE_M='2.5',SWARM_FLEET_SEPARATION_M='2.5',
        PR_PERSON_VERIFY_WEIGHTS=str(repo/'weights/yolo11n_person.pt'),
        PR_PERSON_VERIFY_CONF='.1',PR_PERSON_VERIFY_IOU='.25',
        SAFE_3D='2.8',DANGER_3D='2.0',SLOWDOWN_DIST='4.0',
        PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    flight_env.pop('SWARM_CALIB',None)
    flight_env['PYTHONPATH']=str(snapshot/'coordination/src/robocup_navigation/src')+':'+env.get('PYTHONPATH','')
    (out/'city_control_config.json').write_text(json.dumps(dict(run_id=run_id,
        control_truth_input=False, own_camera_pose_source='DEVELOPMENT_GAZEBO_LINK_POSE',
        clearance_configuration_revision='v1.2',route_clearance_m=2.5,fleet_separation_m=2.5,
        observed_grid_revision='v1.14',observed_grid_resolution_m=.25,
        observed_body_proof_revision='v1.16_scan_carry',
        maximum_speed_mps=3.,visual_fusion_revision='v2.7',tracker_selection_revision='v1.3',
        white_track_activation_confidence=.4,other_track_activation_confidence=.7,
        tracking_guidance_revision='v1.18',tracking_guidance_coast_s=1.5,
        visual_evidence_incoming_max_age_s=1.,
        person_motion_revision='v1.4',recent_motion_window_s=4.,
        target_motion_revision='v1.5',flee_chase_speed_mps=2.6,
        route_continuity_revision='v1.7',route_corner_guidance_revision='v1.8',
        visual_rendezvous_revision='v1.9',official_short_coast_revision='v1.10',
        green_person_verification_revision='v1.11',
        estimator_configuration_revision='v1.12',
        estimator_jump_handling_revision='v1.15_quarantine_without_reanchor',
        teammate_radar_source_commit='f45671b04bb8d7a6d2dcad5f891879e02ba0f6d5',
        person_verifier_weights_sha256=hashlib.sha256((repo/'weights/yolo11n_person.pt').read_bytes()).hexdigest(),
        red_matching_revision='organizer_clarification_20261004_v2_agent_remaining',
        red_report_topic='/actor_red_info',red_coordination_identity='geometric_track_slot_not_actor_id',
        altitude_configuration_revision='v1.17',altitude_reference='MAVROS_LOCAL',
        cruise_altitude_m=2.2,horizontal_takeoff_gate_local_m=2.,
        motion_evidence_minimum_world_height_m=1.,
        actual_height_trial_goal_m=[2.5,3.],altitude_hard_m=4.1,altitude_panic_m=4.3,
        altitude_emergency_m=4.5,
        formal_competition_pass=False),indent=2))
    # Let PX4 initialize at 250Hz, then slow physics without changing sensors.
    physics=rospy.ServiceProxy('/gazebo/get_physics_properties',GetPhysicsProperties)()
    update_rate=float(env.get('CITY_PHYSICS_RATE','20'))
    if not math.isfinite(update_rate) or not 1<=update_rate<=250:
        raise ValueError('Invalid development physics rate')
    changed=rospy.ServiceProxy('/gazebo/set_physics_properties',SetPhysicsProperties)(
        physics.time_step,update_rate,physics.gravity,physics.ode_config)
    (out/'development_physics.json').write_text(json.dumps(dict(update_rate=update_rate,
        time_step=physics.time_step,sensor_parameters_changed=False,
        reason='City software rendering produced image age 1.3-1.8s at 50Hz')))
    if not changed.success:
        raise RuntimeError('CITY_PHYSICS_SLOWDOWN_FAILED')
    scripts=snapshot/'coordination/src/robocup_swarm/scripts'
    processes=[spawn(['python3','-u',str(scripts/'yolo_target_bridge.py')],'yolo_target_bridge',flight_env)]
    # Observe native physics contacts before arming. Sonar sensing volumes are
    # verified separately; logical uav IDs must never substitute model names.
    contact_binary=Path(flight_env.get('CITY_CONTACT_OBSERVER_BINARY',
        '/root/robo_team_build/contact_audit/city_contact_observer'))
    if contact_binary.is_file():
        from fixture_contacts import sonar_virtual_collisions
        model_root=next((Path(path) for path in env.get('GAZEBO_MODEL_PATH','').split(':')
                         if path and (Path(path)/'sonar/model.sdf').is_file()),None)
        if model_root is None:
            raise RuntimeError('CONTACT_AUDIT_SONAR_MODEL_MISSING')
        models=[row['model_name'] for row in wiring['uavs']]
        whitelist=out/'contact_sonar_whitelist_models.txt'
        whitelist.write_text('\n'.join(sorted(sonar_virtual_collisions(model_root,models)))+'\n')
        source=repo/'coordination/scripts/city_contact_observer.cc'
        shutil.copy2(source,out/source.name)
        (out/'contact_observer_manifest.json').write_text(json.dumps(dict(
            fleet_models=models,control_input=False,
            binary_sha256=hashlib.sha256(contact_binary.read_bytes()).hexdigest(),
            source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            sonar_model_sha256=hashlib.sha256((model_root/'sonar/model.sdf').read_bytes()).hexdigest()),indent=2))
        processes.append(spawn([str(contact_binary),str(whitelist)],'city_contacts',flight_env))
    python=flight_env.get('VISION_PYTHON','/root/robo_team_build/vision_env/bin/python')
    # Shared inference (2026-10-03): one CUDA context for all six cameras instead of
    # six. Off unless PR_SHARED_INFER=1, and the service refuses device=cpu because on
    # CPU it would only add a hop. Started BEFORE the perception nodes so they can
    # connect on their first frame instead of falling back.
    shared_infer = flight_env.get('PR_SHARED_INFER', '0') == '1'
    if shared_infer:
        shared_port = flight_env.get('PR_SHARED_PORT', '19731')
        # Must match perception_real.py's CONF default, otherwise the shared path
        # would detect at a different threshold than the local path.
        shared_conf = float(flight_env.get('PR_CONF', '0.40'))
        service_python = flight_env.get('SHARED_VISION_PYTHON', python)
        processes.append(spawn([service_python,'-u',
            str(snapshot/'perception/shared_inference_service.py'),
            '--port', shared_port,
            '--device', flight_env.get('VISION_DEVICE', '0'),
            '--conf', str(shared_conf)],
            'shared_inference', dict(flight_env,
                PR_WEIGHTS=str(repo/'weights/best_yolo11n_bino_v1.pt'))))
        # Warm-up must complete before clients start; spawning alone is not readiness.
        import socket
        ready_deadline = time.monotonic() + 120
        while True:
            if owned_health_check is not None:
                owned_health_check()
            if processes[-1].poll() is not None:
                raise RuntimeError('SHARED_INFERENCE_STARTUP_EXIT')
            try:
                with socket.create_connection(('127.0.0.1', int(shared_port)), timeout=2) as connection:
                    connection.settimeout(2)
                    with connection.makefile('rwb') as stream:
                        stream.write(b'{"op":"ping"}\n'); stream.flush()
                        reply = json.loads(stream.readline())
                        if reply.get('ok') and reply.get('ready'):
                            break
            except (OSError, ValueError):
                pass
            if time.monotonic() >= ready_deadline:
                raise RuntimeError('SHARED_INFERENCE_STARTUP_TIMEOUT')
            time.sleep(.5)
        (out/'shared_inference.json').write_text(json.dumps(dict(
            enabled=True, port=int(shared_port),
            device=flight_env.get('VISION_DEVICE',''),
            conf=shared_conf,
            reason='six independent CUDA contexts were the largest reclaimable '
                   'memory in the 2026-10-03 OOM; sensors unchanged'), indent=2))
        print('shared inference service on port %s conf=%.2f' % (shared_port, shared_conf), flush=True)
    for row in wiring['uavs']:
        uid,model=row['uav_id'],row['model_name']
        vision_env=dict(flight_env,PR_UAV=model,PR_LOGICAL_UAV_ID=uid,
            PR_DEVICE=flight_env.get('PR_CLIENT_DEVICE',flight_env.get('VISION_DEVICE','')),
            PR_CAM_LINK=model+'::cgo3_camera_link',PR_CAM_OFF_BL='0,0,-0.162',
            PR_CAM_TOPIC='/'+uid+'/cgo3_camera/image_raw',
            PR_CAM_INFO_TOPIC='/'+uid+'/cgo3_camera/camera_info',
            PR_OFFICIAL_ARBITRATED='0',PR_IDENTITY_GATE='0',
            PR_SHARED_INFER='1' if shared_infer else '0',
            PR_SHARED_PORT=flight_env.get('PR_SHARED_PORT','19731'),
            PR_WEIGHTS=str(repo/'weights/best_yolo11n_bino_v1.pt'))
        processes.append(spawn([python,'-u',str(snapshot/'perception/perception_real.py')],
                               'perception_'+uid,vision_env))
    processes.append(spawn(['python3','-u',str(scripts/'swarm_manager.py'),
                            '_uav_ids:='+flight_env['SWARM_UAV_IDS']],'swarm_manager',flight_env))
    for row in wiring['uavs']:
        uid=row['uav_id']
        x,y=row['spawn_xy']
        processes.append(spawn(['python3','-u',str(scripts/'swarm_agent.py'),
            '__name:=swarm_agent_'+uid,'_uav_id:='+uid,'_model_name:='+row['model_name'],
            '_mavros_namespace:='+row['mavros_namespace'],'_scan_topic:='+row['scan_topic'],
            '_world_offset_x:='+str(x),'_world_offset_y:='+str(y)],'agent_'+uid,flight_env))
    # The platform programs use relative resource files: launch from their own isolated cwd.
    for index in range(6):
        processes.append(spawn(['bash','-c','cd "$1" && exec python3 -u control_actor.py "$2"',
            'city-actor',str(city),str(index)],'actor_'+str(index),flight_env))
    processes.append(spawn(['bash','-c','cd "$1" && exec python3 -u "$2" "$1/score_cal.py" typhoon_h480',
        'city-judge',str(city),str(out/'execution_sources/start_city_judge.py')],'judge',flight_env))
    records=JsonlEvidence(out/'city_events.jsonl')
    trajectory=JsonlEvidence(out/'city_trajectory.jsonl')
    commands=JsonlEvidence(out/'city_commands.jsonl')
    armed=set()
    latest=dict(left_actors=None,score=None,visual_count=0,target_grants=0)
    def left(msg):
        import ast
        latest['left_actors']=ast.literal_eval(msg.data)
        records.write(dict(kind='left_actors',sample_s=rospy.Time.now().to_sec(),value=latest['left_actors']))
    def score(msg):
        latest['score']=msg.data
    def visual(msg):
        latest['visual_count']+=1
        records.write(dict(kind='visual',sample_s=rospy.Time.now().to_sec(),value=json.loads(msg.data)))
    def assignment(msg):
        value=json.loads(msg.data)
        if value.get('action')=='GRANT' and value['task']['task_type']==1:
            latest['target_grants']+=1
    subscriptions=[rospy.Subscriber('/left_actors',String,left),rospy.Subscriber('/score',Int16,score),
        rospy.Subscriber('/swarm/visual_observation',String,visual,queue_size=100),
        rospy.Subscriber('/swarm/authorized_assignment',String,assignment,queue_size=100)]
    def command(message,uid):
        commands.write(dict(sample_s=message.header.stamp.to_sec(),uav_id=uid,
            mask=message.type_mask,velocity=[message.velocity.x,message.velocity.y,message.velocity.z],
            position=[message.position.x,message.position.y],yaw_rate=message.yaw_rate))
    def arm(message,uid):
        if message.armed:
            armed.add(uid)
    for row in wiring['uavs']:
        subscriptions.extend([
            rospy.Subscriber(row['mavros_namespace']+'/state',State,arm,callback_args=row['uav_id']),
            rospy.Subscriber(row['mavros_namespace']+'/setpoint_raw/local',PositionTarget,command,
                             callback_args=row['uav_id'],queue_size=20)])
    samples=[]
    model_state=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
    start=rospy.Time.now().to_sec()
    deadline=time.monotonic()+seconds*250/update_rate*1.5+90
    error=None
    last_sample=-1.
    try:
        while rospy.Time.now().to_sec()-start < seconds:
            if owned_health_check is not None:
                owned_health_check()
            if time.monotonic()>deadline:
                error='CITY_SIM_TIME_TIMEOUT'; break
            def expected_actor_exit(process):
                return (process.returncode == 0 and 'city-actor' in process.args
                    and latest['left_actors'] is not None
                    and int(process.args[-1]) not in latest['left_actors'])
            dead=[dict(pid=p.pid,returncode=p.returncode,command=p.args)
                  for p in processes if p.poll() is not None and not expected_actor_exit(p)
                  and latest['left_actors'] != []]
            if dead:
                error='CITY_COMPONENT_EXIT:'+str(dead); break
            now=rospy.Time.now().to_sec()
            if now-last_sample>=.2:
                def collect_positions():
                    positions={}
                    for row in wiring['uavs']:
                        response=model_state(row['model_name'],'world')
                        if response.success:
                            p=response.pose.position
                            positions[row['uav_id']]=[p.x,p.y,p.z]
                    return positions
                try:
                    positions=bounded_query(collect_positions)
                except Exception as failure:
                    error='CITY_MODEL_STATE_FAILURE:'+str(failure)
                    break
                if len(positions)==6:
                    sample=dict(sample_s=now,positions=positions)
                    samples.append(sample); trajectory.write(sample)
                last_sample=now
            if latest['left_actors']==[]:
                break
            time.sleep(.05)
    finally:
        for subscription in subscriptions:
            subscription.unregister()
        records.close(); trajectory.close(); commands.close()
    heights={r['uav_id']:max((s['positions'][r['uav_id']][2] for s in samples),default=0.) for r in wiring['uavs']}
    movements={r['uav_id']:max((math.dist(s['positions'][r['uav_id']][:2],r['spawn_xy']) for s in samples),default=0.)
               for r in wiring['uavs']}
    audit=audit_jsonl([out/'city_events.jsonl',out/'city_trajectory.jsonl',out/'city_commands.jsonl'])
    # This is only evidence of airborne motion, not competition completion.
    # The former 2.5m minimum was a trial-height assumption, not an official
    # minimum altitude. Low-altitude development flights still count as motion.
    moving=(not error and audit['valid'] and len(armed)==6
            and all(1.<=z<6. for z in heights.values()) and all(d>1. for d in movements.values()))
    return dict(status='CITY_SIX_AIRCRAFT_MOVING' if moving else 'CITY_MOTION_BLOCKED',
        six_aircraft_motion_observed=moving,official_targets_eliminated=(6-len(latest['left_actors']))
            if latest['left_actors'] is not None else None,
        all_targets_eliminated=latest['left_actors']==[],maximum_altitudes_m=heights,
        maximum_displacements_m=movements,simulated_seconds=rospy.Time.now().to_sec()-start,
        minimum_sampled_separation_m=min((math.dist(s['positions'][a['uav_id']],s['positions'][b['uav_id']])
            for s in samples for i,a in enumerate(wiring['uavs']) for b in wiring['uavs'][i+1:]),default=None),
        component_error=error,evidence_streams_verified=audit['valid'],fixture_only=False,
        formal_competition_pass=False,armed_during_run_uavs=sorted(armed),**latest)
