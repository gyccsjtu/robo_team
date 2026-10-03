#!/usr/bin/env python3
"""Six real agent objects and manager on isolated ROS, synthetic sensor samples.

No Gazebo or FCU is started. This validates wiring, not physical flight safety.
Source this repository's isolated catkin devel before invoking.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid
import hashlib
import traceback

ROOT = Path(__file__).resolve().parents[2]
ap = argparse.ArgumentParser()
ap.add_argument('--output', required=True)
ap.add_argument('--master-port', type=int, default=11365)
args = ap.parse_args()
out = Path(args.output).resolve()
out.mkdir(parents=True, exist_ok=False)
probe = socket.socket()
try:
    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    probe.bind(('127.0.0.1', args.master_port))
except OSError as error:
    (out/'result.json').write_text(json.dumps(dict(ros_wiring_verified=False,
        physical_flight_verified=False, formal_competition_pass=False,
        error='MASTER_PORT_UNAVAILABLE: '+str(error)),indent=2))
    raise
finally:
    probe.close()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % args.master_port,
                  ROBOCUP_RUN_ID=str(uuid.uuid4()), ROBOCUP_LOG_DIR=str(out),
                  ROBOCUP_METADATA=str(ROOT / 'coordination/src/robocup_training_worlds/worlds/generated/training_city_unit_single_wall_s42.json'),
                  ROBOCUP_WS=str(ROOT / 'coordination'), VIS_ENABLE='0', SEED_TRUTH='0')
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
log = (out / 'roscore.log').open('w')
core = subprocess.Popen(['roscore', '-p', str(args.master_port)], stdout=log,
                        stderr=subprocess.STDOUT, start_new_session=True)
try:
    import rosgraph
    deadline = time.monotonic() + 20.
    while not rosgraph.is_master_online():
        if core.poll() is not None or time.monotonic() > deadline:
            raise RuntimeError('ISOLATED_MASTER_NOT_READY')
        time.sleep(.1)
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from std_msgs.msg import String
    from robocup_swarm.msg import SearchAssignment
    from swarm_agent import SwarmAgent
    from swarm_manager import SwarmManager
    rospy.init_node('authority_ros_smoke', anonymous=False, disable_signals=True)
    fleet = ['uav_%d' % i for i in range(1, 7)]
    manager = SwarmManager(fleet)
    agents = {u: SwarmAgent(u, 'test_' + u) for u in fleet}
    for agent in agents.values():
        agent.offset = (0., 0.)
    time.sleep(.5)

    def assignment(uid, typ=0, target='', x=-4., cell=0):
        msg = SearchAssignment()
        msg.uav_id, msg.task_type, msg.target_id = uid, typ, target
        msg.cell_ix, msg.cell_iy = cell, 0
        msg.target_x, msg.target_y = x, -4.
        return msg

    def pump(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            for agent in agents.values():
                pose, velocity = PoseStamped(), TwistStamped()
                pose.header.stamp = velocity.header.stamp = rospy.Time.now()
                pose.pose.position.x = pose.pose.position.y = -4.
                pose.pose.position.z = 4.
                pose.pose.orientation.w = 1.
                agent._local_cb(pose)
                agent._velocity_cb(velocity)
                with agent._authority_lock:
                    agent._gate.can_move(rospy.Time.now().to_sec())
                    agent._publish_authority_state()
            with manager._authority_lock:
                manager._emit_authority(manager._authority.tick(rospy.Time.now().to_sec()))
            time.sleep(.05)

    for i, uid in enumerate(fleet):
        manager._authorized_publish(assignment(uid, cell=i))
    pump(.4)
    assert all(a._gate.task is not None and a._gate.task['task_type'] == 0 for a in agents.values())
    manager._authorized_publish(assignment('uav_1', 1, 't0'))
    manager._authorized_publish(assignment('uav_2', 1, 't0'))
    pump(1.5)
    assert agents['uav_1']._gate.generation == 2
    assert agents['uav_1']._gate.task['target_id'] == 't0'
    assert not agents['uav_2']._gate.stopping
    assert agents['uav_2']._gate.generation == 1
    assert agents['uav_2']._gate.task['task_type'] == 0
    assert manager._authority.locks[('target', 't0')]['owner'] == 'uav_1'
    # Inject an old-run command over the real authorized topic.
    before = agents['uav_1']._gate.seq
    stale = dict(schema_version=2, run_id='old-run', uav_id='uav_1', seq=99999,
                 generation=999, action='GRANT', expires_s=rospy.Time.now().to_sec() + 100.,
                 task=dict(cell_ix=0, cell_iy=0, target_x=3., target_y=3., task_type=1, target_id='old'))
    manager._authority_pub.publish(String(data=json.dumps(stale)))
    time.sleep(.15)
    assert agents['uav_1']._gate.seq == before
    assert agents['uav_1']._gate.task['target_id'] == 't0'
    report = dict(ros_wiring_verified=True, aircraft_objects=6,
                  stopped_handoff_verified=True, unique_target_owner='uav_1',
                  rejected_backup_keeps_search_authorization=True,
                  old_run_rejected=True, physical_flight_verified=False,
                  formal_competition_pass=False, run_id=os.environ['ROBOCUP_RUN_ID'])
    report['source_sha256'] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (ROOT / 'coordination/src/robocup_swarm/scripts' / name
                  for name in ('swarm_agent.py', 'swarm_manager.py', 'task_authority.py', 'radar_velocity_guard.py'))}
    (out / 'result.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)
except Exception as error:
    (out/'failure.log').write_text(traceback.format_exc())
    (out/'result.json').write_text(json.dumps(dict(ros_wiring_verified=False,
        physical_flight_verified=False, formal_competition_pass=False,
        error=str(error), run_id=os.environ['ROBOCUP_RUN_ID']),indent=2))
    raise
finally:
    if 'rospy' in sys.modules:
        sys.modules['rospy'].signal_shutdown('owned smoke complete')
    if core.poll() is None:
        os.killpg(core.pid, signal.SIGTERM)
        try:
            core.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            os.killpg(core.pid, signal.SIGKILL)
            core.wait()
    log.close()
