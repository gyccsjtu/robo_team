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


class OwnedRunInterrupted(RuntimeError):
    """A signal to the owning launcher, distinct from a Gazebo RPC failure."""
    def __init__(self, evidence):
        self.evidence = dict(evidence)
        reason = ('OWNED_WATCHDOG_TIMEOUT' if evidence['signal_name']=='SIGUSR1'
                  and evidence['owned_watchdog_fired'] else 'OWNED_INTERRUPT_'+evidence['signal_name'])
        super().__init__(reason)


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


def query_positions(function, health_check=None, on_retry=None, retryable=(OSError,)):
    """One short retry for a failed RPC, never for a hung query or dead child."""
    try:
        return bounded_query(function)
    except TimeoutError:
        raise
    except retryable as failure:
        if health_check is not None:
            health_check()
        if on_retry is not None:
            on_retry(str(failure))
        return bounded_query(function, timeout=1.)
from evidence_writer import JsonlEvidence, audit_jsonl
from judge_terminal import completed as judge_completed


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
    experimental=env.get('CITY_EXPERIMENTAL_V123','0')=='1'
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
        ROBOCUP_JUDGE_TERMINAL=str(out/'judge_terminal.json'),
        ROBOCUP_METADATA=str(metadata),ROBOCUP_WS=str(snapshot/'coordination'),
        SWARM_UAV_IDS=','.join(r['uav_id'] for r in wiring['uavs']),
        ONLINE_RADAR_PLANNING='1',RADAR_START_CLEARANCE_FILE=str(clearance),SWARM_LOOKAHEAD_M='4.0',
        SWARM_ORBIT_PLAN_CHORD_M='9.0',
        SWARM_WHITE_REACQUIRE='1',BRIDGE_BROWN_ALIGNED='1',BRIDGE_RED_MOTION_LIMITS='1',
        BRIDGE_BLUE_ALIGNED=env.get('CITY_BLUE_ALIGNED','1'),
        PR_ORIGINAL_REPORT_COLORS=env.get('CITY_ORIGINAL_REPORT_COLORS','blue'),
        PR_WHITE_EARLY_DISCOVERY=env.get('CITY_WHITE_EARLY_DISCOVERY','1'),
        BRIDGE_GREEN_ORIGINAL=env.get('CITY_GREEN_ORIGINAL','1'),
        PR_WHITE_FAILURE_EVIDENCE='1',
        PR_DISTANT_CANDIDATES='1',
        SWARM_CONTINUOUS_TRACK_TASK='1',
        SWARM_TARGET_HOLD_S='8',SWARM_TRACK_NAVIGATION_S='8',
        SWARM_COMPANION_TRACKING='1',
        SWARM_OWN_CAMERA_GUIDANCE=env.get('CITY_OWN_CAMERA_GUIDANCE','1'),
        SWARM_BOUNDED_ESCAPE=env.get('CITY_BOUNDED_ESCAPE','1'),
        BRIDGE_BROWN_ACTIVATION=env.get('CITY_BROWN_ACTIVATION','1'),
        BRIDGE_BROWN_NEW_TRACK_CONF='0.6',
        BRIDGE_APPROACH_REPORTING=env.get('CITY_APPROACH_REPORTING','1'),
        BRIDGE_SOURCE_REPORTING=env.get('CITY_SOURCE_REPORTING','1'),
        SWARM_CURRENT_SCAN_CORRIDOR=env.get('CITY_CURRENT_SCAN_CORRIDOR','1'),
        SWARM_SEARCH_ZONE_LAYOUT=env.get('CITY_SEARCH_ZONE_LAYOUT', 'sectors_3x2'),
        SEED_TRUTH='0',PRIORITY_CORNER='0',VIS_ENABLE='0',SWARM_SEARCH_OBSERVATION='1',SWARM_POSE_RATE_GUARD='1',RADAR_GUARD='1',SWARM_MAX_SPEED='3.0',PR_RECENT_MOTION_WINDOW='4.0',FLEE_CHASE_SPEED='2.6',
        PR_COORD_HZ='10',
        PR_FAST_GREEN_WHITE='1',
        PR_FAST_BLUE_PERSON=env.get('CITY_FAST_BLUE_PERSON','1'),
        PR_FAST_RED_PERSON='0',PR_WHITE_SHORT_MISS_RECOVERY='0',
        SWARM_BEHAVIOR_BASELINE='c77063e',
        SWARM_HANDOFF_REACQUIRE='1',
        PR_COLOR_VERIFY='1',
        PR_BLUE_IDENTITY_GUARD='1',
        PR_BLUE_MOTION_WINDOW='10' if experimental else '4',
        PR_BLUE_MOTION_MIN_SPAN='3' if experimental else '1',
        PR_STATIONARY_GREEN='1' if experimental else '0',
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
        PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',
        OPENBLAS_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1')
    flight_env.pop('SWARM_CALIB',None)
    if (flight_env['BRIDGE_SOURCE_REPORTING'] != '1'
            or flight_env['BRIDGE_APPROACH_REPORTING'] != '1'):
        flight_env['BRIDGE_GREEN_ORIGINAL'] = '0'
    if flight_env['BRIDGE_GREEN_ORIGINAL'] == '1':
        colors = flight_env['PR_ORIGINAL_REPORT_COLORS'].split(',')
        flight_env['PR_ORIGINAL_REPORT_COLORS'] = ','.join(dict.fromkeys(colors+['green']))
    flight_env['PYTHONPATH']=str(snapshot/'coordination/src/robocup_navigation/src')+':'+env.get('PYTHONPATH','')
    (out/'city_control_config.json').write_text(json.dumps(dict(run_id=run_id,
        control_truth_input=False, own_camera_pose_source='DEVELOPMENT_GAZEBO_LINK_POSE',
        clearance_configuration_revision='v1.2',route_clearance_m=2.5,fleet_separation_m=2.5,
        search_priority_revision='no_implicit_actor_spawn_prior',priority_corner=flight_env['PRIORITY_CORNER'],
        red_geometry_revision='association_matches_6s_retention_without_timestamp_refresh',
        red_motion_limits_revision='red_only_1p5_seconds_3m_no_freshness_extension',
        blue_identity_revision='motion_identity_not_exempted_by_person_proof',
        image_time_reporting_revision='blue_raw_projection_v3_compat_3p1',
        image_time_reporting_colors=flight_env['PR_ORIGINAL_REPORT_COLORS'],
        navigation_candidate_revision='v1.45_schema6_white_two_proofs_navigation_only_0_to_22m',
        white_early_discovery_enabled=flight_env['PR_WHITE_EARLY_DISCOVERY'],
        green_original_motion_enabled=flight_env['BRIDGE_GREEN_ORIGINAL'],
        blue_source_alignment_enabled=flight_env['BRIDGE_BLUE_ALIGNED'],
        blue_person_enabled=flight_env['PR_FAST_BLUE_PERSON'],
        brown_fusion_revision='v1.39_source_aligned_original_time',
        brown_activation_revision='v1_three_actual_originals_at_0p6',
        brown_activation_enabled=flight_env['BRIDGE_BROWN_ACTIVATION'],
        brown_activation_threshold=float(flight_env['BRIDGE_BROWN_NEW_TRACK_CONF']),
        approach_reporting_revision='v1_same_camera_12m_three_originals_0p6s',
        approach_reporting_enabled=flight_env['BRIDGE_APPROACH_REPORTING'],
        source_reporting_enabled=flight_env['BRIDGE_SOURCE_REPORTING'],
        official_report_source_revision='v1.42_single_ready_camera',
        tracking_camera_revision='v1.44_own_fresh_guidance_per_source_coverage',
        own_camera_guidance_enabled=flight_env['SWARM_OWN_CAMERA_GUIDANCE'],
        visual_observation_schema=3,
        current_scan_corridor_revision='v1_local_0p2s_full_0p5s_original_ttl',
        current_scan_corridor_enabled=flight_env['SWARM_CURRENT_SCAN_CORRIDOR'],
        white_reacquisition_revision='v1.39_own_image_once_per_generation_3s',
        orbit_planning_revision='v1.38_braking_length_ring_chord',
        orbit_radius_m=8.,orbit_plan_chord_m=9.,
        orbit_direction_selection='fresh_measured_directions_then_fresh_visual_target_velocity',
        observed_grid_revision='v1.35_recent_hit_original_ttl',observed_grid_resolution_m=.25,
        observed_recent_hit_hold_s=3.,
        measured_motion_revision='v1.35_velocity_local_and_accepted_pose_rate',
        pose_rate_guard_enabled=True,pose_rate_window_s=.3,pose_rate_minimum_span_s=.2,
        observed_body_proof_revision='v1.16_scan_carry',
        local_execution_clearance_revision='v1.24_current_scan_corridor',
        bounded_escape_revision='v1_search_measured_rest_same_generation_grant',
        bounded_escape_enabled=flight_env['SWARM_BOUNDED_ESCAPE'],
        bounded_escape_configuration=dict(normal_radius_m=1.2,recovery_radius_m=.9,
            maximum_path_m=2.,maximum_measured_travel_m=2.,speed_limit_mps=.3,
            measured_rest_s=1.,rest_speed_mps=.15,maximum_execution_s=15.),
        position_brake_revision='v2_xyz_after_takeoff',navigation_feedback_revision='v1.1_visual_lost_tracking_only',
        behavior_baseline='c77063e',behavior_rollback_revision='v1.29',
        tracking_planner_handoff_revision='v1.30_same_generation_finishes_before_camera_replacement',
        tracking_reacquisition_revision='v1.31_new_grant_bounded_yaw_only',
        target_speed_revision='v1.32_chase_without_flee_classification_search_without_spook',
        task_elimination_revision='v1.33_official_target_fence',
        search_intent_revision='v1.33_cancel_unexecuted_claim_only',
        search_observation_revision='v1.34_inferred_frames_bounded_views',
        search_progress_revision='v1.3_bounded_20s_4m_0p5_progress_6s_rest',
        search_feedback_schema=1,processed_camera_frame_schema=1,
        search_revisit_sim_s=30.,search_observe_min_s=2.5,search_observe_timeout_s=3.,
        search_max_views=3,search_min_distinct_inferred_images=3,
        search_min_new_sample_points=5,search_feedback_receipt_schema=1,
        navigation_retry_revision='v1.24_no_position_rejection_filter',
        green_white_confirm_revision='v1.24_existing_confirmation_counter',
        confirmation_attempt_revision='v1.24_existing_confirmation_counter',
        tracking_retry_revision='v1.24_no_new_grant_camera_retry',
        backoff_observation_revision='v1.29_stop_without_search_fallthrough',
        pending_camera_reacquisition_revision='disabled',
        stop_command_revision='v1.28_zero_request_bypasses_slew',
        tracking_refresh_maximum_deferral_s=20.,
        maximum_speed_mps=3.,visual_fusion_revision='v2.15_v124_behavior_metadata2',tracker_selection_revision='v1.20_camera_takeover',
        guide_lookahead_m=4.,guide_lookahead_revision='v1.37_city_4m_existing_speed_caps',
        measured_route_envelope_revision='v1.37_current_and_fresh_owned_same_generation_retained',
        retained_route_snapshot_maximum_receipt_age_s=1.5,
        perception_csv_revision='v1.26_established_current_person',red_person_proof_revision='disabled_v124_behavior',
        fresh_person_recovery_revision='disabled_v124_behavior',
        fast_red_person_enabled=False,white_short_miss_recovery_enabled=False,
        model_state_query_revision='v1.26_single_bounded_rpc_retry',
        experimental_v123_enabled=experimental,
        stationary_green_revision='v1.24_three_original_person_frames',
        fresh_green_white_person_enabled=True,
        blue_motion_window_s=10. if experimental else 4.,blue_motion_minimum_span_s=3. if experimental else 1.,
        shirt_color_veto_revision='v1.22_same_image_green_white',
        gimbal_wiring_revision='v1.21_per_aircraft_follow_body',fleet_wiring_schema_version=2,
        visual_report_throttle_hz=10.,original_image_report_deduplication=True,
        observer_dispatch_range_m=20.,camera_takeover_debounce_original_s=.75,
        white_track_activation_confidence=.4,other_track_activation_confidence=.7,
        tracking_guidance_revision='v1.18',tracking_guidance_coast_s=1.5,
        tracking_navigation_revision='v1_original_image_bounded_approach',
        tracking_navigation_window_s=8.,tracking_navigation_standoff_m=10.,
        tracking_navigation_scan_s=3.,tracking_navigation_timestamp='original_image',
        companion_tracking_revision='v1_visual_motion_standoff',
        companion_tracking_standoff_m=10.,companion_plan_forward_m=9.,
        tracking_execution_mode='companion',legacy_orbit_parameters_active=False,
        search_zone_revision='v1_public_bounds_contiguous_3x2_home_first',
        search_zone_layout=flight_env['SWARM_SEARCH_ZONE_LAYOUT'],
        stop_observation_revision='v1_own_accepted_original_bearing_during_stop',
        stop_observation_maximum_image_age_s=8.,task_authority_compatibility='v2.6',
        confirmed_visual_acceptance_revision='v1_explicit_core_acceptance',
        bridge_trace_revision='v1_exact_input_and_upload_order',
        actor_environment_revision='v1_gazebo_world_pose_body_twist',
        actor_environment_control_input=False,
        visual_evidence_incoming_max_age_s=1.,
        person_motion_revision='v1.4',recent_motion_window_s=4.,
        target_motion_revision='v1.5',flee_chase_speed_mps=2.6,
        route_continuity_revision='v1.36_current_offer_renewal_during_pending',
        route_corner_guidance_revision='v1.36_map_and_committed_envelope',
        route_speed_budget_revision='v1.36_request_scaling_dual_measured_final_gate',
        visual_rendezvous_revision='v1.9',official_short_coast_revision='v1.10',
        green_person_verification_revision='v1.11',
        estimator_configuration_revision='v1.12',
        estimator_jump_handling_revision='v1.15_quarantine_without_reanchor',
        teammate_radar_source_commit='f45671b04bb8d7a6d2dcad5f891879e02ba0f6d5',
        person_verifier_weights_sha256=hashlib.sha256((repo/'weights/yolo11n_person.pt').read_bytes()).hexdigest(),
        red_matching_revision='organizer_clarification_20261004_v2_agent_remaining',
        red_report_topics=['/actor_red1_info','/actor_red2_info'],red_report_class='red',
        red_coordination_identity='geometric_track_slot_not_actor_id',
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
    processes=[spawn(['python3','-u',str(scripts/'yolo_target_bridge.py')],'yolo_target_bridge',
        dict(flight_env,BRIDGE_TRACE_JSONL=str(out/'algorithm/bridge_trace.jsonl')))]
    # Platform-only input: new actors require actual world pose AND velocity.
    # The legacy MAVROS pose bridge has zero twist and cannot trigger escape.
    actor_wiring=out/'actor_environment_wiring.json'
    actor_wiring.write_text(json.dumps(wiring))
    actor_status=out/'actor_environment_odometry.json'
    actor_bridge=spawn(['python3','-u',str(out/'execution_sources/actor_environment_bridge.py'),
        '--wiring',str(actor_wiring),'--status',str(actor_status)],
        'actor_environment_odometry',flight_env)
    processes.append(actor_bridge)
    actor_deadline=time.monotonic()+25.
    while not actor_status.exists():
        if owned_health_check is not None:
            owned_health_check()
        if actor_bridge.poll() is not None or time.monotonic() >= actor_deadline:
            raise RuntimeError('ACTOR_ENVIRONMENT_ODOMETRY_STARTUP_FAILED')
        time.sleep(.05)
    actor_ready=json.loads(actor_status.read_text())
    if not actor_ready.get('ready') or actor_ready.get('run_id') != run_id:
        raise RuntimeError('ACTOR_ENVIRONMENT_ODOMETRY_RUN_MISMATCH')
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
    judge_process=spawn(['bash','-c','cd "$1" && exec python3 -u "$2" "$1/score_cal.py" typhoon_h480',
        'city-judge',str(city),str(out/'execution_sources/start_city_judge.py')],'judge',flight_env)
    processes.append(judge_process)
    judge_sha=hashlib.sha256((city/'score_cal.py').read_bytes()).hexdigest()
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
            position=[message.position.x,message.position.y],position_z=message.position.z,
            yaw_rate=message.yaw_rate))
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
    terminal=None
    topic_last=None
    last_sample=-1.
    query_retries=[]
    try:
        while rospy.Time.now().to_sec()-start < seconds:
            if owned_health_check is not None:
                owned_health_check()
            if time.monotonic()>deadline:
                error='CITY_SIM_TIME_TIMEOUT'; break
            terminal=judge_completed(out/'judge_terminal.json',run_id,judge_sha,judge_process.poll())
            if terminal is not None:
                topic_last=latest['left_actors']
                latest['left_actors']=[]
                records.write(dict(kind='judge_terminal',sample_s=rospy.Time.now().to_sec(),value=terminal))
                break
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
                    prior_retries=len(query_retries)
                    def record_retry(failure):
                        query_retries.append(failure)
                        records.write(dict(kind='model_state_retry',sample_s=rospy.Time.now().to_sec(),
                                           error=failure,maximum_retry_wall_s=1.))
                    positions=query_positions(collect_positions,owned_health_check,record_retry,
                                              retryable=(rospy.ServiceException,OSError))
                    if len(query_retries)>prior_retries:
                        now=rospy.Time.now().to_sec()
                except Exception as failure:
                    error=('CITY_OWNED_INTERRUPT:' if isinstance(failure,OwnedRunInterrupted)
                           else 'CITY_MODEL_STATE_FAILURE:')+str(failure)
                    break
                if len(positions)==6:
                    sample=dict(sample_s=now,positions=positions)
                    samples.append(sample); trajectory.write(sample)
                last_sample=now
            if latest['left_actors']==[]:
                break
            time.sleep(.05)
    except OwnedRunInterrupted as failure:
        error='CITY_OWNED_INTERRUPT:'+str(failure)
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
        component_error=error,model_state_retry_count=len(query_retries),
        evidence_streams_verified=audit['valid'],fixture_only=False,
        judge_terminal_evidence=terminal,left_actors_topic_last=topic_last,
        formal_competition_pass=False,armed_during_run_uavs=sorted(armed),**latest)
