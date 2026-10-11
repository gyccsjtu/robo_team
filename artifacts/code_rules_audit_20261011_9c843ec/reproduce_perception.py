#!/usr/bin/env python3
"""Read-only offline audit. No ROS nodes, simulator, model loading, or repository writes.

Run: python3 -B this_file.py [repository]
Tests assert the presently observed failure signatures, NOT desired behavior.
After fixes, failures of these assertions should be converted into regression tests.
"""
import ast
import importlib.util
import json
import math
from pathlib import Path
import sys

sys.dont_write_bytecode = True
repo = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2]
perception = repo / 'perception/perception_real.py'
bridge_path = repo / 'coordination/src/robocup_swarm/scripts/yolo_target_bridge.py'
source = perception.read_text()
module = ast.parse(source)
# Extract only the real pure class. Do not import perception's ROS/model setup.
cls = next(n for n in module.body if isinstance(n, ast.ClassDef) and n.name == 'PixelCueTracker')
ns = {'math': math, 'FY': 376.0}
exec(compile(ast.Module(body=[cls], type_ignores=[]), str(perception), 'exec'), ns)

rates = {}
for hz in (5, 10, 12, 20):
    tr = ns['PixelCueTracker']()
    count = 0
    for k in range(100):
        count += len(tr.update(1.0 + k / hz, [dict(
            cls='white', conf=.95, u=376.0, v=260.0, h=60.0,
            bearing=0.0, cam_x=0.0, cam_y=0.0)]))
    rates[hz] = count
assert rates[5] > 0 and rates[12] == rates[20] == 0, rates

# Extract the real association selection statement, preserving the exact code.
# The final elif logs rejected geometry; these in-gate examples never enter it.
selection = next(n for n in ast.walk(module) if isinstance(n, ast.If)
    and isinstance(n.test, ast.Compare)
    and isinstance(n.test.left, ast.Name) and n.test.left.id == 'dd'
    and len(n.test.comparators) == 1 and isinstance(n.test.comparators[0], ast.Name)
    and n.test.comparators[0].id == 'bd')
selector = compile(ast.Module(body=[selection], type_ignores=[]), str(perception), 'exec')
associations = []
for rows in [[(.20, .60), (.15, .95)], [(.15, .95), (.20, .60)]]:
    state = dict(bd=1.0, gate=1.0, best=-1, confirmed=True, APP_W=.3,
                 best_appear_score=0.0, dyn_rej=0)
    for j, (distance, appearance) in enumerate(rows):
        state.update(j=j, dd=distance, appear_sim=appearance)
        exec(selector, state)
    associations.append(dict(order=rows, selected=rows[state['best']], score=state['bd']))
assert associations[0]['selected'] != associations[1]['selected'], associations

# This module is deliberately pure at import; ROS imports occur only in node construction.
spec = importlib.util.spec_from_file_location('readonly_bridge_audit', str(bridge_path))
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)

guard = bridge.CrossColorLocationGuard()
for stamp in (1.0, 1.25, 1.5):
    guard.observe(stamp, 'camera', 'blue', 0.0, 0.0, .9, True)
conflicts = []
for stamp in (3.0, 3.25, 3.5):
    event = guard.observe(stamp, 'camera', 'white', .5, 0.0, .9, True)
    if event:
        conflicts.append(event)
assert len(conflicts) == 1 and conflicts[0]['tags'] == ['blue', 'white'], conflicts
assert guard.blocked('blue', 0.0, 0.0, 4.0)
assert guard.blocked('white', .5, 0.0, 4.0)

core = bridge.TargetBridgeCore()
for i in range(81):
    stamp = 1.0 + i * .25
    core.report(stamp, 'white', 0.0, 0.0, 1.0, 8.0, True, 'A', 'actor-A')
    if i >= 8:
        core.report(stamp + .05, 'white', 1.2, 0.0, 1.0, 6.0, True, 'B', 'actor-B')
track = core.tracks['white']
lock_result = dict(good_source=track.official_source,
    good_xy=[track.good_x, track.good_y],
    accepted_sources=list(track.accepted_source_samples),
    rejected_samples=track.official_rejects,
    camera_B_retained_samples=len(track.source_history.get(('B', 'actor-B'), [])))
assert track.official_source == 'A' and ('B', 'actor-B') not in track.accepted_source_samples, lock_result
# No claim that B is physically correct: this proves permanent incumbent dependence,
# not which camera has the accurate absolute coordinate in a real scene.
print(json.dumps(dict(repository=str(repo),
    status='four_current_failure_signatures_reproduced',
    candidate_rate_outputs=rates, association_order_dependence=associations,
    sequential_distinct_neighbor_quarantine=conflicts, incumbent_source_lock=lock_result),
    ensure_ascii=False, indent=2))
