#!/usr/bin/env python3
"""Offline reproductions of CURRENT defects, not passing safety regressions.

No ROS nodes, network requests, simulator or model loading. Read source only.
Run from any directory: python3 -B this_file.py [repository]
After repairs, these defect assertions must be replaced by desired invariants.
"""
import ast
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace

sys.dont_write_bytecode = True
ROOT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'coordination/src/robocup_swarm/scripts'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


view = load('audit_view', SCRIPTS / 'tracking_view.py')
height_results = {}
for label, tilts in [('level', [0]), ('intermittent_tilt', [0, 9, 0, 0, 9])]:
    state = None
    for i in range(25):
        sample = dict(target_id='t3', sample_s=1 + i * .2,
                      height=480., fy=376., box=(340., 240., 380., 400.),
                      camera_tilt=math.radians(tilts[i % len(tilts)]))
        state = view.update_altitude_hold(state, sample, 2.6, 8., 2., 4.2)
    height_results[label] = state['goal']
assert height_results['level'] > 2.7
assert height_results['intermittent_tilt'] == 2.6

# Compile the actual final scan method, with only its runtime constants and
# a local clock/log stub. This does not construct or import the ROS agent.
agent_path = SCRIPTS / 'swarm_agent.py'
tree = ast.parse(agent_path.read_text())
constants = {'RADAR_GUARD', 'RADAR_FRESH_S', 'RADAR_SIDE_PANIC_R',
             'RADAR_SIDE_PANIC_SPEED', 'RADAR_STOP_R', 'MAX_SPEED', 'MAX_ACC'}
ns = dict(math=math, os=os, rospy=SimpleNamespace(
    Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: 100.)),
    logwarn_throttle=lambda *args: None))
for node in tree.body:
    if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id in constants for t in node.targets):
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(agent_path), 'exec'), ns)
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SwarmAgent')
method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_final_scan_cap')
exec(compile(ast.Module(body=[method], type_ignores=[]), str(agent_path), 'exec'), ns)
scan_results = {}
for label, invalid in [('nan', float('nan')), ('negative_infinity', -float('inf'))]:
    scan = SimpleNamespace(angle_min=math.radians(-135), angle_increment=math.radians(1),
                           ranges=[invalid] * 271, range_min=.1, range_max=30.)
    aircraft = SimpleNamespace(_scan=scan, _scan_t=100., yaw=0., uav_id='audit')
    scan_results[label] = ns['_final_scan_cap'](aircraft, 2., 0.)
assert scan_results['nan'] == (0., 0.)
assert scan_results['negative_infinity'] == (2., 0.)

audit = load('audit_graph_offline', ROOT / 'tools/audit_match_graph.py')
topics = ['/actor_%s_info' % tag for tag in audit.ACTOR_TAGS]
publishers = [[topic, [audit.BRIDGE_NODE]] for topic in topics]
publishers += [[audit.TARGET_TOPIC, [audit.BRIDGE_NODE]],
               ['/actor_0/cmd_motion', ['/team_bad']]]
subscribers = [[topic, [audit.JUDGE_NODE]] for topic in topics]
subscribers += [['/typhoon_h480_0/odom', ['/team_bad']]]
types = [[topic, audit.ACTOR_INFO_TYPE] for topic in topics]
graph_failures = audit.audit_graph([publishers, subscribers, []], types)
assert graph_failures == []

print(json.dumps(dict(
    status='current_failure_signatures_reproduced',
    altitude_hold_goal=height_results,
    final_scan_invalid_input_output=scan_results,
    graph_audit_failures_for_environment_command_publisher=graph_failures,
    limits=[
        'Synthetic inputs demonstrate code behavior, not a recorded collision.',
        'ROS graph results do not prove that any formal team node uses forbidden data.',
        'Negative-infinity scan input has not been established in the match logs.'
    ]), ensure_ascii=False, indent=2))
