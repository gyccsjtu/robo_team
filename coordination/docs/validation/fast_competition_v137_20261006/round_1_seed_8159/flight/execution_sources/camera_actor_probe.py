"""Observe untouched radar-model camera pixels and run the existing YOLO weights."""
import hashlib
import json
from pathlib import Path
import subprocess
import time


def run(out, spawn, env):
    import rospy
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import String
    from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion
    from gazebo_msgs.srv import GetLinkState, GetModelState
    from robocup_swarm.msg import TargetDetection
    from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
    uid = 'uav_1'
    images, infos, visual, forwarded, official = [], [], [], [], []
    subs = [rospy.Subscriber('/'+uid+'/cgo3_camera/image_raw', Image, images.append, queue_size=1),
            rospy.Subscriber('/'+uid+'/cgo3_camera/camera_info', CameraInfo, infos.append, queue_size=1),
            rospy.Subscriber('/swarm/visual_observation', String, lambda message: visual.append(json.loads(message.data)), queue_size=100)]
    subs += [rospy.Subscriber('/swarm/detection', TargetDetection, lambda m: forwarded.append(dict(
        target_id=m.target_id, uav_id=m.uav_id, sample_s=m.header.stamp.to_sec(), x=m.x,y=m.y,source=m.source)), queue_size=100),
        rospy.Subscriber('/actor_green_info', ActorInfo, lambda m: official.append(dict(
            received_s=rospy.Time.now().to_sec(), cls=m.cls,x=m.x,y=m.y)), queue_size=100)]
    try:
        spawn(['timeout', '8', 'stdbuf', '-oL', 'gz', 'topic', '-e', '/gazebo/default/skeleton_pose/info'], 'skeleton_pose')
        subprocess.run(['gz', 'topic', '-l'], env=env, stdout=(out/'gazebo_topics.log').open('w'), timeout=10)
        subprocess.run(['gz', 'topic', '-i', '/gazebo/default/skeleton_pose/info'], env=env,
                       stdout=(out/'skeleton_connections.log').open('w'), timeout=10)
        # The original actor plugin advances skeletal animation by distance.
        # A completely stationary uncommanded actor may never render a frame.
        actor_command = rospy.Publisher('/actor_0/cmd_motion', ActorMotion, queue_size=1)
        deadline = time.monotonic()+10
        while time.monotonic() < deadline and actor_command.get_num_connections() == 0:
            time.sleep(.1)
        if actor_command.get_num_connections() == 0:
            raise RuntimeError('ORIGINAL_ACTOR_COMMAND_PLUGIN_MISSING')
        actor_command.publish(ActorMotion(x=-14., y=3., v=.8))
        images.clear()
        deadline = time.monotonic()+25
        while time.monotonic() < deadline and (len(images) < 5 or not infos):
            time.sleep(.1)
        if not images or not infos:
            raise RuntimeError('REAL_CAMERA_FRAME_OR_CALIBRATION_MISSING')
        image, info = images[-1], infos[-1]
        camera_pose = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)(uid+'::cgo3_camera_link', 'world')
        actor_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)('actor_0', 'world')
        pose_fields = lambda p: dict(xyz=[p.position.x,p.position.y,p.position.z],
            quaternion_xyzw=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
        actor_link = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)('actor_0::actor_0_pose', 'world')
        (out/'camera_actor_observer.json').write_text(json.dumps(dict(
            camera_success=camera_pose.success, actor_success=actor_pose.success,
            camera=pose_fields(camera_pose.link_state.pose), actor=pose_fields(actor_pose.pose),
            actor_link_success=actor_link.success, actor_link=pose_fields(actor_link.link_state.pose),
            note='Diagnostic observer only; not an input to perception or flight control.')))
        if (image.width, image.height) != (info.width, info.height) or image.header.stamp.to_sec() <= 0:
            raise RuntimeError('REAL_CAMERA_CALIBRATION_MISMATCH')
        frame = out/'camera_frame.png'
        cv2.imwrite(str(frame), CvBridge().imgmsg_to_cv2(image, 'bgr8'))
        (out/'camera_info.json').write_text(json.dumps(dict(width=info.width, height=info.height,
            K=list(info.K), D=list(info.D), sample_s=image.header.stamp.to_sec(), frame_id=image.header.frame_id)))
        repo = Path(__file__).resolve().parents[2]
        weights = repo/'weights/best_yolo11n_bino_v1.pt'
        program = out/'yolo_pixels_probe.py'
        program.write_text('''import json, sys, cv2, torch
from ultralytics import YOLO
torch.set_num_threads(1)
model=YOLO(sys.argv[1])
result=model(sys.argv[2], conf=.2, verbose=False)[0]
detections=[dict(class_name=model.names[int(b.cls)], confidence=float(b.conf), xyxy=b.xyxy[0].tolist()) for b in result.boxes]
cv2.imwrite(sys.argv[3],result.plot())
open(sys.argv[4],'w').write(json.dumps(dict(detections=detections,torch=torch.__version__)))
''')
        python = env.get('VISION_PYTHON', '/root/robo_team_build/vision_env/bin/python')
        command = [python, str(program), str(weights), str(frame), str(out/'camera_yolo.png'), str(out/'pixel_detections.json')]
        with (out/'yolo_pixels.log').open('w') as log:
            subprocess.run(command, env=dict(env, OMP_NUM_THREADS='1', MKL_NUM_THREADS='1'), stdout=log,
                           stderr=subprocess.STDOUT, timeout=60, check=True)
        detections = json.loads((out/'pixel_detections.json').read_text())
        verified = any(item['class_name'] == 'green' for item in detections['detections'])
        # Execute the actual perception node, not a separate pixel-only substitute.
        sources = out/'perception_sources'
        sources.mkdir()
        for name in ('perception_real.py', 'camera_geometry.py'):
            import shutil
            shutil.copy2(repo/'perception'/name, sources/name)
        (out/'perception_sources.json').write_text(json.dumps({p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sources.iterdir()}, indent=2))
        run_id = 'camera-probe-'+out.name
        node_env = dict(env, ROBOCUP_RUN_ID=run_id, SWARM_UAV_IDS=','.join('uav_'+str(i) for i in range(1,7)),
            PR_UAV=uid, PR_CAM_LINK=uid+'::cgo3_camera_link', PR_CAM_OFF_BL='0,0,-0.162',
            PR_CAM_TOPIC='/'+uid+'/cgo3_camera/image_raw', PR_WEIGHTS=str(weights),
            ROBOCUP_LOG_DIR=str(out/'algorithm'),
            PYTHONPATH=str(repo/'coordination/src/robocup_swarm/scripts')+':'+env.get('PYTHONPATH',''),
            OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
        bridge_sources = out/'swarm_sources'
        bridge_sources.mkdir()
        for path in (repo/'coordination/src/robocup_swarm/scripts').glob('*.py'):
            shutil.copy2(path, bridge_sources/path.name)
        (out/'swarm_source_manifest.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest()
            for p in bridge_sources.iterdir()}, indent=2))
        metadata = out/'metadata.json'
        metadata.write_text(json.dumps(dict(schema='robocup_training_worlds/metadata/v1',
            frame=dict(frame_id='map', convention='ENU', units='m'),
            bounds=dict(x_min=-30.,x_max=30.,y_min=-20.,y_max=20.),
            grid=dict(width=120,height=80,resolution_m=.5,origin=[-30.,-20.],frame_id='map',encoding='rle_pairs',data=[[0,9600]]),
            obstacles=[],spawn=dict(center=[0.,0.]),goal_candidates=[],generator=dict(name='CAMERA_ONLY_DEVELOPMENT'))))
        import os, sys
        os.environ.update(node_env, ROBOCUP_METADATA=str(metadata), ONLINE_RADAR_PLANNING='1', SEED_TRUTH='0', VIS_ENABLE='0')
        sys.path.insert(0,str(bridge_sources))
        from swarm_manager import SwarmManager
        manager = SwarmManager(node_env['SWARM_UAV_IDS'].split(','))
        bridge = spawn(['python3','-u',str(bridge_sources/'yolo_target_bridge.py')], 'yolo_target_bridge', node_env)
        process = spawn([python, '-u', str(sources/'perception_real.py')], 'perception_real', node_env)
        actor_command.publish(ActorMotion(x=-8., y=3., v=.8))
        deadline = time.monotonic()+45
        while time.monotonic()<deadline and (len(visual)<5 or len(forwarded)<3 or len(official)<3) and process.poll() is None and bridge.poll() is None:
            time.sleep(.1)
        (out/'actual_visual_observations.json').write_text(json.dumps(visual, indent=2))
        (out/'forwarded_detections.json').write_text(json.dumps(forwarded, indent=2))
        (out/'official_coordinate_reports.json').write_text(json.dumps(official, indent=2))
        actual_verified = len([v for v in visual if v.get('target_id')=='green' and v.get('run_id')==run_id])>=5
        bridge_verified = len(forwarded)>=3 and len(official)>=3 and all(m['source']==1 and m['uav_id']==uid
            and m['target_id']=='t0' for m in forwarded) and all(m['cls']=='green' for m in official)
        manager_verified = ('t0' in manager.tracker.targets and uid in manager.tracker.targets['t0'].observers
            and manager._last_detect.get('t0') in [m['sample_s'] for m in forwarded])
        (out/'manager_visual_state.json').write_text(json.dumps(dict(last_detect=manager._last_detect,
            target_ids=list(manager.tracker.targets), no_flight_agents=True, execution_authority_granted=False), indent=2))
        return dict(camera_actor_probe_verified=verified and actual_verified and bridge_verified and manager_verified, physical_rendered_yolo_verified=verified,
            actual_perception_observations_verified=actual_verified,
            actual_bridge_forwarding_verified=bridge_verified,
            actual_manager_real_observation_verified=manager_verified,
            camera_topic='/'+uid+'/cgo3_camera/image_raw', fixed_camera_mount=True, armed=False,
            synthetic_observations=False, physical_coordinate_accuracy_verified=False,
            detections=detections['detections'], weights_sha256=hashlib.sha256(weights.read_bytes()).hexdigest(),
            formal_competition_pass=False)
    finally:
        for sub in subs:
            sub.unregister()
