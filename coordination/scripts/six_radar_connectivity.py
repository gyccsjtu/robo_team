#!/usr/bin/env python3
"""Bounded six-PX4 connectivity check in an empty development world; no arming."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import threading
import time

from prepare_radar_fleet import derive
from radar_fleet_models import generate


def owned_process_snapshot(children):
    """Poll only owned children before shutdown changes their exit evidence."""
    return [dict(pid=process.pid, command=process.args,
                 log_file=str(log.name), returncode=process.poll())
            for process, log in children]


def check_endpoints(tcp_ports, udp_ports):
    for kind, ports in ((socket.SOCK_STREAM, tcp_ports), (socket.SOCK_DGRAM, udp_ports)):
        for port in sorted(ports):
            with socket.socket(socket.AF_INET, kind) as sock:
                if kind == socket.SOCK_STREAM:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    sock.bind(('0.0.0.0', port))
                except OSError as error:
                    raise RuntimeError('PORT_UNAVAILABLE_%d_%s: %s' %
                                       (port, 'TCP' if kind == socket.SOCK_STREAM else 'UDP', error))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--px4', default='/root/third_party_official/PX4-Autopilot')
    parser.add_argument('--runtime', default='/root/robocup_runtime/stereo_20261002T140417Z')
    parser.add_argument('--source-sdf', default='/root/vendor_eval/d43bac6/models/typhoon_h480_lidar/typhoon_h480_lidar.sdf')
    parser.add_argument('--master-port', type=int, default=11375)
    parser.add_argument('--gazebo-port', type=int, default=11376)
    parser.add_argument('--flight-seconds', type=float, default=0,
                        help='Run real swarm search after connectivity in this empty development fixture')
    parser.add_argument('--obstacle-fixture', action='store_true', help='Add six static box obstacles with contact observations')
    parser.add_argument('--camera-actor-probe', action='store_true', help='Grounded camera/YOLO probe with one actor; no flight')
    parser.add_argument('--flight-actor-probe', action='store_true', help='Observe real camera-to-target execution during development flight')
    parser.add_argument('--city-scene', help='Prepared isolated platform city directory; six targets and judge')
    args = parser.parse_args()
    if args.camera_actor_probe and args.flight_seconds:
        parser.error('The grounded camera actor probe cannot arm or fly')
    if args.flight_actor_probe and (not args.flight_seconds or args.camera_actor_probe):
        parser.error('--flight-actor-probe requires flight and cannot use the grounded probe')
    if args.city_scene and (args.obstacle_fixture or args.camera_actor_probe or args.flight_actor_probe):
        parser.error('City scene cannot be combined with development fixtures')
    if not math.isfinite(args.flight_seconds) or not 0 <= args.flight_seconds <= (600 if args.city_scene else 120):
        parser.error('Invalid duration; city limit is 600s, fixture limit is 120s')
    px4 = Path(args.px4)
    build = px4 / 'build/px4_sitl_default'
    wiring = derive(build)
    city = Path(args.city_scene).resolve() if args.city_scene else None
    if city:
        scene = json.loads((city/'scene_manifest.json').read_text())
        for index, row in enumerate(wiring['uavs']):
            row['model_name'] = 'typhoon_h480_%d' % index
            row['spawn_xy'] = scene['positions'][row['uav_id']]
    runtime = Path(args.runtime)
    children = []
    locks = []
    report = dict(status='FAILED', formal_competition_pass=False, fixture_only=True, armed=False)
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    script_directory = Path(__file__).resolve().parent
    snapshot = out / 'execution_sources'
    snapshot.mkdir()
    source_names = ('six_radar_connectivity.py', 'prepare_radar_fleet.py', 'radar_fleet_models.py',
                    'six_swarm_probe.py', 'fixture_contacts.py', 'camera_actor_probe.py',
                    'flight_actor_probe.py', 'evidence_writer.py', 'city_swarm_run.py', 'start_city_judge.py', 'judge_terminal.py',
                    'build_namespaced_gimbal_overlay.py', 'actor_environment_bridge.py')
    for name in source_names:
        shutil.copyfile(script_directory / name, snapshot / name)
    (out / 'execution_sources.json').write_text(json.dumps({name:
        hashlib.sha256((script_directory / name).read_bytes()).hexdigest()
        for name in source_names}, indent=2))
    def interrupted(signum, frame):
        raise RuntimeError('CHECK_INTERRUPTED_OR_WALL_TIMEOUT')
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGUSR1, interrupted)
    city_rate = float(os.environ.get('CITY_PHYSICS_RATE', '20'))
    if not math.isfinite(city_rate) or not 1 <= city_rate <= 250:
        parser.error('CITY_PHYSICS_RATE must be finite and between 1 and 250')
    watchdog = threading.Timer(args.flight_seconds*250/city_rate*1.5+360 if city else (780 if args.flight_actor_probe else 360),
                               lambda: os.kill(os.getpid(), signal.SIGUSR1))
    watchdog.daemon = True
    watchdog.start()
    try:
        # Cooperates with the single-flight owning runner and other six checks.
        for instance in range(7):
            lock = open('/tmp/robo_team_px4_instance%d.lock' % instance, 'a+')
            locks.append(lock)
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        tcp_ports = {args.master_port, args.gazebo_port}
        udp_ports = set()
        for row in wiring['uavs']:
            i = row['px4_instance']
            tcp_ports.add(row['simulator_tcp_port'])
            udp_ports.update(row[k] for k in ('simulator_udp_port', 'px4_local_port', 'mavros_local_port'))
            udp_ports.update((18570+i, 14280+i, row['px4_gimbal_port'],row['gimbal_local_port']))
        if args.master_port == args.gazebo_port:
            raise ValueError('ROS and Gazebo ports must differ')
        # Check each endpoint's actual protocol; unrelated TCP reservations are irrelevant to UDP.
        check_endpoints(tcp_ports, udp_ports)
        if not Path('/tmp/.X11-unix/X0').exists():
            raise RuntimeError('Camera-preserving model requires the verified X0 display')
        (out / 'wiring.json').write_text(json.dumps(wiring, indent=2))
        gimbal_overlay = Path(os.environ.get('ROBOCUP_GIMBAL_RUNTIME',
            '/root/robocup_runtime/gimbal_namespaced_20261004')) if city else None
        generate(args.source_sdf, wiring, runtime / 'gps', out / 'models',
                 hide_ray_visuals=bool(city),gimbal_overlay=gimbal_overlay)
        if gimbal_overlay is not None:
            shutil.copyfile(gimbal_overlay/'manifest.json',out/'gimbal_runtime_manifest.json')
        world = out / 'connectivity.world'
        world.write_text('''<sdf version="1.6"><world name="default">
<include><uri>model://ground_plane</uri></include><include><uri>model://sun</uri></include>
<physics name="default_physics" type="ode"><max_step_size>0.004</max_step_size><real_time_update_rate>250</real_time_update_rate></physics>
</world></sdf>''')
        if city:
            shutil.copyfile(city/'robocup.world', world)
        if args.camera_actor_probe:
            import xml.etree.ElementTree as ET
            repo = script_directory.parents[1]
            source_world = repo/'perception/worlds/actor_min.world'
            actor = ET.parse(source_world).find("world/actor[@name='actor_0']")
            actor.find("plugin[@filename='libros_actor_cmd_pose_plugin.so']/init_pose").text = '-15 3 1.25 1.57 0 0'
            world.write_text(world.read_text().replace('</world>', ET.tostring(actor, encoding='unicode')+'</world>'))
            # Camera-only fixed mount, never a flight/clearance certificate.
            camera_model = out/'models/uav_1.sdf'
            model_tree = ET.parse(camera_model)
            model = model_tree.getroot().find('model')
            mount = ET.SubElement(model, 'joint', name='camera_fixture_mount', type='fixed')
            ET.SubElement(mount, 'parent').text = 'world'
            ET.SubElement(mount, 'child').text = 'base_link'
            model_tree.write(camera_model, encoding='unicode')
            shutil.copy2(source_world, out/'actor_source.world')
        if args.obstacle_fixture:
            boxes = []
            for index in range(6):
                name, x = 'fixture_box_%d' % index, -20+index*8
                boxes.append('''<model name="%s"><static>true</static><pose>%s -9 5 0 0 0</pose>
<link name="box"><collision name="collision"><geometry><box><size>3 3 10</size></box></geometry></collision>
<visual name="visual"><geometry><box><size>3 3 10</size></box></geometry></visual>
<sensor name="contact" type="contact"><always_on>true</always_on><update_rate>50</update_rate>
<contact><collision>collision</collision></contact><plugin name="contacts" filename="libgazebo_ros_bumper.so">
<bumperTopicName>/fixture/contacts/%s</bumperTopicName><frameName>map</frameName></plugin></sensor>
</link></model>''' % (name, x, name))
            world.write_text(world.read_text().replace('</world>', ''.join(boxes)+'</world>'))
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % args.master_port,
                   GAZEBO_MASTER_URI='http://127.0.0.1:%d' % args.gazebo_port,
                   ROBOCUP_LEGACY_GPS_MODEL='1', GAZEBO_MODEL_DATABASE_URI='', PYTHONUNBUFFERED='1')
        # WSL's Mesa D3D12 backend rendered animated people invisible in the
        # original camera. Keep stock sensors; select a working renderer.
        env.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
        libraries = ':'.join(str(runtime / name) for name in ('gazebo', 'gps', 'actor_lib'))
        env['LD_LIBRARY_PATH'] = libraries + ':/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:' + env.get('LD_LIBRARY_PATH', '')
        env['GAZEBO_PLUGIN_PATH'] = env['LD_LIBRARY_PATH'] + ':' + env.get('GAZEBO_PLUGIN_PATH', '')
        env['GAZEBO_MODEL_PATH'] = ':'.join(('/root/vendor_eval/d43bac6/models',
            '/root/third_party_official/XTDrone_official/sitl_config/models',
            str(px4 / 'Tools/sitl_gazebo/models'), '/root/robocup_resources/vm_gazebo_cache_20261002/models',
            env.get('GAZEBO_MODEL_PATH', '')))
        os.environ.update(env)

        def spawn(command, name, child_env=None):
            log = (out / (name + '.log')).open('w')
            process = subprocess.Popen(command, env=child_env or env, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            children.append((process, log))
            return process

        spawn(['roscore', '-p', str(args.master_port)], 'roscore')
        deadline = time.monotonic() + 30
        import xmlrpc.client
        while time.monotonic() < deadline:
            try:
                socket.setdefaulttimeout(2)
                if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/six_radar_check')[0] == 1:
                    break
            except (OSError, xmlrpc.client.Error):
                time.sleep(.5)
        else:
            raise RuntimeError('ROS_MASTER_TIMEOUT')
        socket.setdefaulttimeout(None)
        import rospy
        from gazebo_msgs.srv import SpawnModel
        from geometry_msgs.msg import Pose, PoseStamped
        from mavros_msgs.msg import State
        from sensor_msgs.msg import LaserScan
        subprocess.run(['rosparam', 'set', '/use_sim_time', 'true'], env=env, check=True, timeout=10)
        rospy.init_node('six_radar_connectivity', anonymous=True, disable_signals=True)
        server = ['gzserver', '--verbose', '-s', 'libgazebo_ros_api_plugin.so']
        if args.camera_actor_probe and env.get('RENDER_OBSERVER_PLUGIN'):
            server += ['-s', env['RENDER_OBSERVER_PLUGIN']]
            env['RENDER_OBSERVER_OUTPUT'] = str(out/'render_observer.log')
        spawn(server + [str(world)], 'gzserver')
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=90)
        # API registration precedes completion of large city-world loading.
        # Require physical simulation ticks before submitting model insertion.
        from rosgraph_msgs.msg import Clock
        clock_start = rospy.wait_for_message('/clock', Clock, timeout=90).clock.to_sec()
        ready_deadline = time.monotonic() + 90
        while time.monotonic() < ready_deadline:
            tick = rospy.wait_for_message('/clock', Clock, timeout=10).clock.to_sec()
            if tick > clock_start + .1:
                (out/'world_ready.json').write_text(json.dumps(dict(
                    first_clock_s=clock_start, advancing_clock_s=tick)))
                break
        else:
            raise RuntimeError('WORLD_NOT_ADVANCING_BEFORE_SPAWN')
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        observations = {row['uav_id']: {} for row in wiring['uavs']}
        subscriptions = []

        def record(uid, kind):
            def callback(message):
                observations[uid][kind] = (time.monotonic(), message)
            return callback

        launch = __import__('xml.etree.ElementTree', fromlist=['ElementTree'])
        root = launch.Element('launch')
        for row in wiring['uavs']:
            uid = row['uav_id']
            group = launch.SubElement(root, 'group', ns=uid)
            include = launch.SubElement(group, 'include', file='$(find mavros)/launch/px4.launch')
            for name, value in [('fcu_url', row['fcu_url']), ('gcs_url', ''),
                                ('tgt_system', str(row['mavlink_system_id'])), ('tgt_component', '1')]:
                launch.SubElement(include, 'arg', name=name, value=value)
            for name, value in [('local_position/frame_id', uid + '/map'),
                                ('local_position/tf/frame_id', uid + '/map'),
                                ('local_position/tf/child_frame_id', uid + '/base_link'),
                                ('global_position/frame_id', uid + '/map'),
                                ('global_position/child_frame_id', uid + '/base_link'),
                                ('global_position/tf/frame_id', uid + '/map'),
                                ('global_position/tf/child_frame_id', uid + '/base_link'),
                                ('global_position/tf/global_frame_id', uid + '/earth'),
                                ('imu/frame_id', uid + '/base_link')]:
                launch.SubElement(group, 'param', name='mavros/' + name, value=value)
            subscriptions += [rospy.Subscriber(row['mavros_namespace'] + '/state', State, record(uid, 'state')),
                              rospy.Subscriber(row['mavros_namespace'] + '/local_position/pose', PoseStamped, record(uid, 'pose')),
                              rospy.Subscriber(row['scan_topic'], LaserScan, record(uid, 'scan'))]
        launch_path = out / 'mavros.launch'
        launch_path.write_text(launch.tostring(root, encoding='unicode'))
        flight_etc = out / 'px4_etc'
        shutil.copytree(build / 'etc', flight_etc)
        if city:
            from estimator_bootstrap import single_estimator,SETTINGS
            startup=flight_etc/'init.d-posix/rcS'
            original_startup=startup.read_text()
            startup.write_text(single_estimator(original_startup))
            env['SWARM_EXPECT_SINGLE_EKF']='1'
            (out/'px4_estimator_config.json').write_text(json.dumps(dict(
                revision='v1.12',scope='isolated city SITL startup before EKF initialization',
                source_sha256=hashlib.sha256(original_startup.encode()).hexdigest(),
                adapted_sha256=hashlib.sha256(startup.read_bytes()).hexdigest(),
                parameters={name:value[1] for name,value in SETTINGS.items()},
                reason='Observed Multi-EKF filter switching, invalid setpoints and large local-coordinate excursions'),indent=2))
        post = flight_etc / 'init.d-posix/airframes/6011_typhoon_h480.post'
        post_text = post.read_text()
        if '-u 14558 ' not in post_text or '-o 14530 ' not in post_text:
            raise RuntimeError('Unsupported typhoon camera MAVLink ports')
        post.write_text(post_text.replace('-u 14558 ', '-u $((14600+px4_instance)) ')
                       .replace('-o 14530 ', '-o $((14630+px4_instance)) '))
        for index, row in enumerate(wiring['uavs']):
            uid = row['uav_id']
            pose = Pose()
            pose.position.x, pose.position.y = row.get('spawn_xy', [-20+index*8, 0.])
            pose.position.z = 2.2 if args.camera_actor_probe and index == 0 else .2
            pose.orientation.w = 1
            response = spawn_model(row['model_name'], (out / 'models' / (row['model_name'] + '.sdf')).read_text(), uid, pose, 'world')
            if not response.success:
                raise RuntimeError('SPAWN_FAILED_' + uid + ':' + response.status_message)
            work = out / ('px4_' + uid)
            work.mkdir()
            flight_env = dict(env, PX4_SIM_MODEL='typhoon_h480', PATH=str(build / 'bin') + ':' + env['PATH'])
            spawn([str(build / 'bin/px4'), '-d', str(flight_etc), '-s', 'etc/init.d-posix/rcS',
                   '-i', str(row['px4_instance']), '-w', str(work)], 'px4_' + uid, flight_env)
            print('Spawned ' + uid, flush=True)
        spawn(['roslaunch', str(launch_path)], 'mavros')
        deadline = time.monotonic() + 150
        while time.monotonic() < deadline:
            if any(p.poll() is not None for p, _ in children):
                raise RuntimeError('OWNED_CHILD_EXITED')
            healthy = []
            now = time.monotonic()
            for uid, values in observations.items():
                if all(k in values and now-values[k][0] < 3 for k in ('state', 'pose', 'scan')):
                    state, scan = values['state'][1], values['scan'][1]
                    if (state.connected and not state.armed and scan.header.frame_id == uid + '/laser_2d'
                            and values['pose'][1].header.frame_id == uid + '/map'):
                        healthy.append(uid)
            if len(healthy) == 6:
                report.update(status='SIX_RADAR_CONNECTED', connected_uavs=healthy,
                              scan_beams={uid: len(values['scan'][1].ranges) for uid, values in observations.items()},
                              telemetry={uid: dict(local_pose_frame=values['pose'][1].header.frame_id,
                                  local_position=[values['pose'][1].pose.position.x, values['pose'][1].pose.position.y,
                                                  values['pose'][1].pose.position.z],
                                  scan_frame=values['scan'][1].header.frame_id,
                                  scan_sample_s=values['scan'][1].header.stamp.to_sec(),
                                  range_min=values['scan'][1].range_min, range_max=values['scan'][1].range_max)
                                         for uid, values in observations.items()})
                print(json.dumps(report), flush=True)
                if args.camera_actor_probe:
                    from camera_actor_probe import run as camera_run
                    report.update(camera_run(out, spawn, env))
                    return 0 if report['camera_actor_probe_verified'] else 1
                if args.flight_seconds:
                    if city:
                        from city_swarm_run import run as run_city
                        report['connectivity_armed'] = report.pop('armed')
                        infrastructure = tuple(children)
                        def owned_health_check():
                            dead = [dict(pid=p.pid, returncode=p.returncode, command=p.args)
                                    for p, _ in infrastructure if p.poll() is not None]
                            if dead:
                                raise RuntimeError('OWNED_CHILD_EXITED:' + str(dead))
                        report.update(run_city(out, wiring, spawn, env, args.flight_seconds, city,
                                               owned_health_check=owned_health_check))
                        print(json.dumps(report), flush=True)
                        return 0 if report['six_aircraft_motion_observed'] else 1
                    from six_swarm_probe import run
                    report['connectivity_armed'] = report.pop('armed')
                    report.update(run(out, wiring, spawn, env, args.flight_seconds, actor_probe=args.flight_actor_probe))
                    from evidence_writer import audit_jsonl
                    required = [out/'six_truth.jsonl', out/'executor_telemetry.jsonl',
                        out/'algorithm/authority_events.jsonl', out/'algorithm/route_events.jsonl']
                    if args.flight_actor_probe:
                        required.append(out/'target_execution_events.jsonl')
                    if args.obstacle_fixture:
                        required.append(out/'fixture_contacts.jsonl')
                    audit = audit_jsonl(list(dict.fromkeys(required + list(out.glob('*.jsonl'))
                        + list((out/'algorithm').glob('*.jsonl')))))
                    (out/'evidence_stream_audit.json').write_text(json.dumps(audit, indent=2))
                    report['evidence_streams_verified'] = audit['valid']
                    if not audit['valid']:
                        report['failure_reasons'].append('EVIDENCE_STREAM_INVALID')
                        report['prototype_search_verified'] = False
                        report['physical_visual_tracking_verified'] = False
                        report['status'] = 'SIX_SEARCH_FLIGHT_INCOMPLETE'
                    print(json.dumps({k: v for k, v in report.items() if k != 'box_contacts'},
                                     allow_nan=False), flush=True)
                    return 0 if report['prototype_search_verified'] else 1
                return 0
            time.sleep(.5)
        raise RuntimeError('SIX_RADAR_HEALTH_TIMEOUT; samples=' + str({uid: list(v) for uid, v in observations.items()}))
    except Exception as error:
        report['error'] = str(error)
        print(json.dumps(report), flush=True)
        return 1
    finally:
        watchdog.cancel()
        # A service can disappear between the health check and the RPC. Keep
        # pre-cleanup codes: later SIGTERM/SIGKILL codes do not explain failure.
        report['owned_children_before_cleanup'] = dict(
            schema_version=1, wall_time_s=time.time(),
            processes=owned_process_snapshot(children))
        if 'rospy' in locals():
            rospy.signal_shutdown('Owned fixture completed')
        for process, log in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        deadline = time.monotonic() + 5
        for process, log in reversed(children):
            try:
                process.wait(timeout=max(.1, deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            log.close()
        (out / 'result.json').write_text(json.dumps(report, indent=2))
        for lock in locks:
            lock.close()


if __name__ == '__main__':
    raise SystemExit(main())
