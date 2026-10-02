#!/usr/bin/env python3
"""Bounded six-PX4 connectivity check in an empty development world; no arming."""
import argparse
import fcntl
import hashlib
import json
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
    args = parser.parse_args()
    px4 = Path(args.px4)
    build = px4 / 'build/px4_sitl_default'
    wiring = derive(build)
    runtime = Path(args.runtime)
    children = []
    locks = []
    report = dict(status='FAILED', formal_competition_pass=False, fixture_only=True, armed=False)
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    script_directory = Path(__file__).resolve().parent
    snapshot = out / 'execution_sources'
    snapshot.mkdir()
    for name in ('six_radar_connectivity.py', 'prepare_radar_fleet.py', 'radar_fleet_models.py'):
        shutil.copyfile(script_directory / name, snapshot / name)
    (out / 'execution_sources.json').write_text(json.dumps({name:
        hashlib.sha256((script_directory / name).read_bytes()).hexdigest()
        for name in ('six_radar_connectivity.py', 'prepare_radar_fleet.py', 'radar_fleet_models.py')}, indent=2))
    def interrupted(signum, frame):
        raise RuntimeError('CHECK_INTERRUPTED_OR_WALL_TIMEOUT')
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGUSR1, interrupted)
    watchdog = threading.Timer(360, lambda: os.kill(os.getpid(), signal.SIGUSR1))
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
            udp_ports.update((18570+i, 14280+i, 13030+i))
        if args.master_port == args.gazebo_port:
            raise ValueError('ROS and Gazebo ports must differ')
        # Check each endpoint's actual protocol; unrelated TCP reservations are irrelevant to UDP.
        check_endpoints(tcp_ports, udp_ports)
        if not Path('/tmp/.X11-unix/X0').exists():
            raise RuntimeError('Camera-preserving model requires the verified X0 display')
        (out / 'wiring.json').write_text(json.dumps(wiring, indent=2))
        generate(args.source_sdf, wiring, runtime / 'gps', out / 'models')
        world = out / 'connectivity.world'
        world.write_text('''<sdf version="1.6"><world name="default">
<include><uri>model://ground_plane</uri></include><include><uri>model://sun</uri></include>
<physics name="default_physics" type="ode"><max_step_size>0.004</max_step_size><real_time_update_rate>250</real_time_update_rate></physics>
</world></sdf>''')
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % args.master_port,
                   GAZEBO_MASTER_URI='http://127.0.0.1:%d' % args.gazebo_port,
                   ROBOCUP_LEGACY_GPS_MODEL='1', GAZEBO_MODEL_DATABASE_URI='')
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
        rospy.init_node('six_radar_connectivity', anonymous=True, disable_signals=True)
        rospy.set_param('/use_sim_time', True)
        spawn(['gzserver', '--verbose', '-s', 'libgazebo_ros_api_plugin.so', str(world)], 'gzserver')
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=90)
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
        for index, row in enumerate(wiring['uavs']):
            uid = row['uav_id']
            pose = Pose()
            pose.position.x = -20 + index * 8
            pose.position.z = .2
            pose.orientation.w = 1
            response = spawn_model(uid, (out / 'models' / (uid + '.sdf')).read_text(), uid, pose, 'world')
            if not response.success:
                raise RuntimeError('SPAWN_FAILED_' + uid + ':' + response.status_message)
            work = out / ('px4_' + uid)
            work.mkdir()
            flight_env = dict(env, PX4_SIM_MODEL='typhoon_h480', PATH=str(build / 'bin') + ':' + env['PATH'])
            spawn([str(build / 'bin/px4'), '-d', str(build / 'etc'), '-s', 'etc/init.d-posix/rcS',
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
                return 0
            time.sleep(.5)
        raise RuntimeError('SIX_RADAR_HEALTH_TIMEOUT; samples=' + str({uid: list(v) for uid, v in observations.items()}))
    except Exception as error:
        report['error'] = str(error)
        print(json.dumps(report), flush=True)
        return 1
    finally:
        watchdog.cancel()
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
