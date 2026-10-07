#!/usr/bin/env python3
"""Offline observed-ray replay. Never launches ROS, moves a UAV, or grants a route."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/robocup_navigation/src'))
from online_radar_planner import OnlinePlanner
from radar_observed_map import ObservedMap


def ranges_from_record(scan):
    result = list(scan['ranges'])
    for index,kind in scan['nonfinite_ranges']:
        result[index] = {'nan':math.nan,'positive_infinity':math.inf,
                         'negative_infinity':-math.inf}[kind]
    if any(v is None for v in result):
        raise ValueError('UNCLASSIFIED_NONFINITE_RANGE')
    return result


def footprint(observed,planner,position,now):
    cell = (math.floor((position[0]-observed.origin[0])/observed.resolution),
            math.floor((position[1]-observed.origin[1])/observed.resolution))
    extent = math.ceil(planner.radius/observed.resolution)
    result = dict(occupied=[],unknown=[],known_free=0,retained_body_unknown=0)
    for dy in range(-extent,extent+1):
        for dx in range(-extent,extent+1):
            if math.hypot(dx,dy)*observed.resolution > planner.radius+observed.resolution/2:
                continue
            ix,iy = cell[0]+dx,cell[1]+dy
            if not (0 <= ix < observed.width and 0 <= iy < observed.height):
                result['unknown'].append(dict(cell=[ix,iy],outside=True))
                continue
            index = iy*observed.width+ix
            stamp = observed.observed_s[index]
            value = observed.cells[index] if stamp is not None and 0 <= now-stamp <= 3. else -1
            if value == 0:
                result['known_free'] += 1
            elif value == -1 and index in planner.body_proof:
                result['retained_body_unknown'] += 1
            else:
                item = dict(cell=[ix,iy],xy=[observed.origin[0]+(ix+.5)*observed.resolution,
                    observed.origin[1]+(iy+.5)*observed.resolution],stamp=stamp)
                result['occupied' if value == 100 else 'unknown'].append(item)
    return result


def replay(flight,uid,query_times):
    metadata = json.loads((flight/'city_bounds.json').read_text())['grid']
    certificate = json.loads((flight/'startup_clearance.json').read_text())
    planner = OnlinePlanner(certificate['positions'][uid])
    observed = ObservedMap(metadata['width'],metadata['height'],metadata['resolution_m'],metadata['origin'])
    nav = flight/'algorithm'/('navigation_'+uid+'.jsonl')
    scans,selected = {},[]
    pending = iter(sorted(query_times))
    target = next(pending,None)
    epoch = None
    with nav.open() as stream:
        for line in stream:
            record = json.loads(line)
            if record['run_id'] != certificate['run_id']:
                raise ValueError('RUN_ID_MISMATCH')
            for scan in record['scan_window']:
                stamp = scan['sample_s']
                if stamp in scans:
                    if scans[stamp] != scan:
                        raise ValueError('CONFLICTING_DUPLICATE_SCAN')
                    continue
                if observed.last_scan_s is not None and stamp <= observed.last_scan_s:
                    raise ValueError('MISSING_OR_UNORDERED_SCAN_HISTORY')
                if epoch is None:
                    epoch = stamp
                ok = observed.feed(ranges_from_record(scan),scan['position_xy'],scan['yaw'],
                    scan['angle_min'],scan['angle_increment'],scan['range_min'],scan['range_max'],
                    stamp,scan['pose_s'],stamp)
                if not ok:
                    raise ValueError('RECORDED_SCAN_REJECTED')
                planner.carry_body_proof(observed,scan['position_xy'],stamp,epoch)
                scans[stamp] = scan
            now,position = record['sample_s'],record['world_xy']
            if position is not None and observed.last_scan_s is not None:
                # Reconstruct proof updates at RECORDED control samples only.
                # Missing control ticks/planner callbacks are not invented.
                planner.local_command_clear(observed,position,(0.,0.),now,epoch)
            if target is None:
                break
            if now < target:
                continue
            proposal = planner.exit_candidate(observed,position,now,epoch,lambda grid:grid)
            narrow = OnlinePlanner(planner.seed_xy,radius=.9)
            narrow.body_proof,narrow.epoch = set(planner.body_proof),planner.epoch
            narrow._body_sample_s = planner._body_sample_s
            narrow.carry_body_proof(observed,position,now,epoch)
            selected.append(dict(requested_s=target,record_s=now,record_sha256=hashlib.sha256(line.encode()).hexdigest(),
                original_reason=record['reason'],generation=record['generation'],position_xy=position,
                last_scan_s=observed.last_scan_s,reconstructed_scan_count=len(scans),
                normal_footprint=footprint(observed,planner,position,now),
                recovery_footprint=footprint(observed,narrow,position,now),
                geometry_exit=None if proposal is None else [list(p) for p in proposal['points']],
                motion_evidence=record.get('motion_evidence')))
            print('record',now,'scans',len(scans),'geometry_exit',selected[-1]['geometry_exit'],flush=True)
            target = next(pending,None)
    stamps = sorted(scans)
    return dict(scope='Observed-ray and recorded-position reconstruction, not full actual callback or physical replay',
        limitations=['No original retained body-proof snapshots or every control/planner callback',
                     'Peer exclusion omitted for geometry diagnosis only; no authorization or execution claim',
                     'Original callback arrival time unavailable; replay feeds each accepted scan at its own stamp'],
        flight=str(flight),uid=uid,startup_seed=planner.seed_xy,run_id=certificate['run_id'],
        navigation_sha256=hashlib.sha256(nav.read_bytes()).hexdigest(),
        recorded_first_scan_s=stamps[0] if stamps else None,recorded_last_scan_s=stamps[-1] if stamps else None,
        maximum_recorded_scan_gap_s=max((b-a for a,b in zip(stamps,stamps[1:])),default=None),
        recorded_scan_gaps_over_055_s=[[a,b] for a,b in zip(stamps,stamps[1:]) if b-a>.55],
        results=selected,unanswered_requested_s=target)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--flight',type=Path,required=True)
    parser.add_argument('--uid',default='uav_2')
    parser.add_argument('--query',type=float,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    result = replay(args.flight,args.uid,args.query)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
