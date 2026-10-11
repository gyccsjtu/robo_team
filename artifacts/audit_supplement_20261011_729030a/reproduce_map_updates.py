#!/usr/bin/env python3
"""Read-only current-source method extraction: no ROS import or simulator.
Assertions document current defects, not desired safety behavior."""
import ast
import hashlib
import json
import math
from pathlib import Path
import threading
import types

ROOT = Path(__file__).resolve().parents[2] / 'coordination/src/robocup_swarm/scripts'


def extract(name, filename, namespace):
    source = ROOT / filename
    module = ast.parse(source.read_text())
    method = next(node for node in ast.walk(module)
                  if isinstance(node, ast.FunctionDef) and node.name == name)
    method.decorator_list = []
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])),
                 str(source), 'exec'), namespace)
    return namespace[name]


def main():
    results = []
    callback = extract('_occ_grid_cb', 'swarm_manager.py', {'json': json, 'math': math})
    manager = types.SimpleNamespace(_slam_lock=threading.Lock(), _slam_cells=None,
                                    _slam_sources={}, _slam_dirty=False)

    def packet(uid, occupied):
        return types.SimpleNamespace(data=json.dumps(dict(
            uav_id=uid, width=1, height=1, resolution=.5, origin=[0, 0],
            rle=[[int(occupied), 1]])))

    callback(manager, packet('old_source', 1))
    callback(manager, packet('new_source', 0))
    results.append(dict(case='old_source_retained_vs_new_zero',
                        merged=list(manager._slam_cells),
                        source_ids=list(manager._slam_sources),
                        timestamp_fields=[key for key in vars(manager)
                                          if 'time' in key or 'stamp' in key],
                        explanation='Wire zero conflates unknown and observed free; cannot safely override occupied.'))
    assert list(manager._slam_cells) == [1]
    callback(manager, packet('old_source', 0))
    results.append(dict(case='same_source_new_full_snapshot_clears',
                        merged=list(manager._slam_cells)))
    assert list(manager._slam_cells) == [0]

    update = extract('_slam_update', 'swarm_agent.py', dict(
        SLAM_PERIOD=.2, SLAM_TILT_RAD=math.radians(5), SLAM_MAX_RANGE=12,
        SLAM_DEPTH_CELLS=4, SLAM_REPLAN_SEC=2, SLAM_PUB_SEC=2,
        json=json, String=lambda **kwargs: kwargs))
    changes = iter([1, 0, 0])
    published = []
    grid = types.SimpleNamespace(mark_scan=lambda *args: next(changes), width=1,
                                 height=1, resolution=.5, origin=(0, 0),
                                 to_rle=lambda: [[0, 1]])
    agent = types.SimpleNamespace(
        _scan=object(), world_xy=(0, 0), _scan_t=101., _slam_last_t=0.,
        slam=grid, yaw=0., _slam_dirty=False, path=[], path_target=None,
        _slam_pub_t=100., occ_pub=types.SimpleNamespace(publish=published.append),
        uav_id='u0')
    for stamp in (101., 103., 200.):
        agent._scan_t = stamp
        update(agent)
        results.append(dict(case='final_clear_inside_publish_throttle',
                            stamp=stamp, dirty=agent._slam_dirty,
                            published_count=len(published)))
    assert agent._slam_dirty and not published

    print(json.dumps(dict(
        method='AST-extracted unmodified current functions; pure Python mocks; no ROS/simulation',
        source_sha256={name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                       for name in ('slam_grid.py', 'swarm_manager.py', 'swarm_agent.py')},
        results=results), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
