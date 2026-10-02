#!/usr/bin/env python3
"""Bounded mission fixture using the repository's real SwarmManager authority.

Consumes MAVROS/agent estimates only. Separate recorder handles simulator truth.
Two search tasks exercise physical stop/handoff, not full search/perception.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
import rospy
from geometry_msgs.msg import TwistStamped
from robocup_swarm.msg import SearchAssignment, UavStatus
from swarm_manager import SwarmManager

ap = argparse.ArgumentParser()
ap.add_argument('--output', required=True)
ap.add_argument('--timeout', type=float, default=180.)
ap.add_argument('--goal', nargs=2, type=float, default=[8., -10.])
args = ap.parse_args()
out = Path(args.output)
out.mkdir(parents=True, exist_ok=False)
rospy.init_node('single_swarm_mission_fixture', disable_signals=True)
manager = SwarmManager(['uav_1'])
latest, velocity = [None], [None]
rospy.Subscriber('/swarm/uav_status', UavStatus,
                 lambda m: latest.__setitem__(0, m) if m.uav_id == 'uav_1' else None)
rospy.Subscriber('/typhoon_h480_0/mavros/local_position/velocity_local', TwistStamped,
                 lambda m: velocity.__setitem__(0, m))
goals = [(8., -3.), tuple(args.goal)]
started = rospy.get_time()
wall_deadline = time.monotonic() + args.timeout * 3 + 120.
phase, stable_since, reached, final = -1, None, [], None
telemetry = (out / 'telemetry.jsonl').open('w', buffering=1)
rate = rospy.Rate(10)


def offer(index):
    msg = SearchAssignment()
    msg.uav_id, msg.task_type, msg.target_id = 'uav_1', 0, ''
    key = manager.grid.world_to_cell(*goals[index])
    if key is None:
        raise RuntimeError('FIXTURE_GOAL_OUT_OF_BOUNDS')
    msg.cell_ix, msg.cell_iy = key
    msg.target_x, msg.target_y = goals[index]
    manager._authorized_publish(msg)


try:
    while not rospy.is_shutdown():
        now = rospy.get_time()
        if now - started > args.timeout or time.monotonic() > wall_deadline:
            raise RuntimeError('MISSION_TIMEOUT')
        st, vel = latest[0], velocity[0]
        with manager._authority_lock:
            manager._emit_authority(manager._authority.tick(now))
        if (st is not None and vel is not None and 0 <= now - st.header.stamp.to_sec() <= .5
                and 0 <= now - vel.header.stamp.to_sec() <= .5):
            speed = math.sqrt(sum(v * v for v in (vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z)))
            position = [st.x, st.y, st.z]
            telemetry.write(json.dumps(dict(sim_s=now, phase=phase, position=position,
                                           speed_mps=speed, generation=manager._authority.generation.get('uav_1', 0))) + '\n')
            error = math.hypot(st.x - goals[phase][0], st.y - goals[phase][1]) if phase >= 0 else None
            good = abs(st.z - 4.5) <= .3 and speed <= .15 and (phase < 0 or error <= .25)
            if good:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= 1.:
                    if phase >= 0:
                        reached.append(dict(phase=phase, sim_s=now, position=position, error_xy_m=error))
                    phase += 1
                    stable_since = None
                    if phase == len(goals):
                        final = dict(status='SINGLE_SWARM_TASKS_REACHED', phases=reached,
                                     final_position=position, speed_mps=speed,
                                     physical_aircraft=1, fixture_only=True,
                                     formal_competition_pass=False, collision_verdict='ABSTAIN')
                        break
                    offer(phase)
            else:
                stable_since = None
        rate.sleep()
except Exception as exc:
    final = dict(status='FAILED', reason=str(exc), phases=reached,
                 physical_aircraft=1, fixture_only=True, formal_competition_pass=False)
finally:
    (out / 'result.json').write_text(json.dumps(final or dict(status='FAILED', reason='ROS_SHUTDOWN'), indent=2))
    telemetry.close()
    # Keep the coordinator heartbeat alive until the owning runner stops the
    # entire simulator. The final task's controller continues a terminal hold.
    while not rospy.is_shutdown():
        with manager._authority_lock:
            manager._emit_authority(manager._authority.tick(rospy.get_time()))
        rate.sleep()
