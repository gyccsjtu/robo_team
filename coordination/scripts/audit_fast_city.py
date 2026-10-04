"""Read-only post-run evidence summary and selective archive; never launch ROS."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import shutil


def read(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def rows(path):
    if path.exists():
        with path.open() as stream:
            for line in stream:
                try:
                    yield json.loads(line)
                except ValueError:
                    continue


def audit(out):
    flight = out/'flight'
    trajectory = list(rows(flight/'city_trajectory.jsonl'))
    events = list(rows(flight/'city_events.jsonl'))
    contacts = list(rows(flight/'city_contacts.log'))
    first_s = trajectory[0]['sample_s'] if trajectory else None
    last_s = trajectory[-1]['sample_s'] if trajectory else None
    latest_left = [r['value'] for r in events if r['kind'] == 'left_actors']
    terminal = read(flight/'judge_terminal.json')
    raw = read(flight/'result.json') or {}
    judge_text = (flight/'judge.log').read_text(errors='replace')
    deleted = sorted({int(i) for i in re.findall(r'actor_(\d+) is OK', judge_text)})
    usage = [float(s) for s in re.findall(r'Time usage:\s*([\d.]+)',judge_text)]
    non_ground = [r for r in contacts if r.get('kind') == 'UAV_BODY_CONTACT'
                  and not any(r[n].startswith('ground_plane::') for n in ('collision1','collision2'))]
    static = [r for r in non_ground
              if not any(r[n].startswith('actor_') for n in ('collision1','collision2'))
              and not all(r[n].startswith('typhoon_h480_') for n in ('collision1','collision2'))]
    heartbeats = [r for r in contacts if r.get('kind') == 'CONTACT_OBSERVER_HEARTBEAT']
    movement = {}
    max_height = {}
    minimum_separation = None
    for sample in trajectory:
        positions = sample['positions']
        for uid, xyz in positions.items():
            max_height[uid] = max(max_height.get(uid, -math.inf), xyz[2])
            initial = trajectory[0]['positions'].get(uid)
            if initial:
                movement[uid] = max(movement.get(uid, 0.), math.dist(initial[:2], xyz[:2]))
        uids = sorted(positions)
        for i, uid in enumerate(uids):
            for other in uids[i+1:]:
                distance = math.dist(positions[uid], positions[other])
                minimum_separation = distance if minimum_separation is None else min(minimum_separation, distance)
    masks = Counter()
    hold_z_present = Counter()
    for command in rows(flight/'city_commands.jsonl'):
        masks[command['mask']] += 1
        if command['mask'] == 1528 and 'position_z' in command:
            hold_z_present[command['uav_id']] += 1
    reject = {}
    reject_reasons = {}
    for path in (flight/'algorithm').glob('perception*.csv'):
        with path.open() as stream:
            for row in csv.DictReader(stream):
                color = row.get('class_name')
                if color in ('green', 'white'):
                    reject.setdefault(color, Counter())[row['event']] += 1
                    if row['event'] == 'track_reject':
                        reject_reasons.setdefault(color,Counter())[row['source']] += 1
    source_manifest = read(flight/'swarm_source_manifest.json') or {}
    mismatches = []
    for name, expected in source_manifest.items():
        path = flight/'swarm_sources'/name
        if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            mismatches.append(name)
    eliminated = len(deleted)
    if terminal and raw.get('judge_terminal_evidence') == terminal and terminal.get('mission_finished') is True and terminal.get('left_actors') == [] and terminal.get('target_finish') == 6:
        eliminated = 6
    coverage = bool(heartbeats and first_s is not None and
        heartbeats[-1]['first_sample_s'] <= first_s and
        heartbeats[-1]['last_sample_s'] >= last_s-.25 and
        heartbeats[-1]['maximum_sample_gap_s'] <= .00401)
    return dict(schema_version=1, formal_competition_pass=False, raw_result_unchanged=True,
        observed_sim_s=last_s-first_s if first_s is not None else None,
        trajectory_start_s=first_s, trajectory_end_s=last_s,
        actual_judge_deleted_actor_ids=deleted, judge_deleted_count=eliminated,
        last_left_actors=latest_left[-1] if latest_left else None,
        last_judge_time_usage_s=usage[-1] if usage else None,
        terminal=terminal, raw_status=raw.get('status'), raw_error=raw.get('component_error',raw.get('error')),
        non_ground_contact_rows=len(non_ground), static_contact_rows=len(static),
        actor_contact_rows=sum(any(r[n].startswith('actor_') for n in ('collision1','collision2')) for r in non_ground),
        fleet_contact_rows=sum(all(r[n].startswith('typhoon_h480_') for n in ('collision1','collision2')) for r in non_ground),
        first_static_contact=static[0] if static else None,
        contact_observer_last=heartbeats[-1] if heartbeats else None,
        contact_observer_covers_trajectory=coverage,
        maximum_actual_height_m=max_height, sampled_minimum_3d_separation_m=minimum_separation,
        maximum_horizontal_displacement_m=movement,
        command_mask_counts=dict(masks), xyz_hold_z_recorded=dict(hold_z_present),
        green_white_events={color:dict(count) for color,count in reject.items()},
        green_white_reject_reasons={color:dict(count) for color,count in reject_reasons.items()},
        executed_source_count=len(source_manifest), executed_source_sha_mismatches=mismatches,
        first_round_z_evidence=read(out/'xyz_hold_readonly_sample.json'),
        competition_goal_met=(eliminated == 6 and not static and len(movement) == 6
            and all(d > 1. for d in movement.values()) and not mismatches
            and max(max_height.values(),default=math.inf) < 6. and coverage
            and raw.get('evidence_streams_verified') is True
            and len(raw.get('armed_during_run_uavs',[])) == 6
            and bool(usage) and usage[-1] <= 600.))


def archive(out, destination):
    destination.mkdir(parents=True, exist_ok=True)
    manifest = {}
    for path in out.rglob('*'):
        if not path.is_file() or '__pycache__' in path.parts:
            continue
        relative = path.relative_to(out)
        # Full simulator output stays in WSL; retain its SHA and final 8KB.
        if path.name == 'gzserver.log':
            h = hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda:stream.read(1024*1024), b''):
                    h.update(chunk)
                stream.seek(max(0,path.stat().st_size-8192))
                tail = stream.read()
            target = destination/relative.with_name('gzserver_tail.log')
            target.parent.mkdir(parents=True,exist_ok=True); target.write_bytes(tail)
            manifest[str(relative)] = dict(sha256=h.hexdigest(),bytes=path.stat().st_size,
                full_file_wsl=str(path),archive='tail only')
            continue
        allowed = (relative.parts[0] == 'scene' or len(relative.parts) == 1
            or relative.parts[1] in ('algorithm','execution_sources','swarm_sources','models')
            or len(relative.parts) == 2 and path.suffix in ('.log','.json','.jsonl','.world','.txt'))
        if not allowed:
            continue
        target = destination/relative; target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,target)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if hashlib.sha256(target.read_bytes()).hexdigest() != digest:
            raise RuntimeError('ARCHIVE_SHA_MISMATCH:'+str(relative))
        manifest[str(relative)] = dict(sha256=digest,bytes=path.stat().st_size)
    (destination/'archive_manifest.json').write_text(json.dumps(manifest,indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root',required=True)
    parser.add_argument('--archive',required=True)
    args = parser.parse_args()
    root = Path(args.root)
    budget = read(root/'budget.json')
    summaries = []
    for record in budget['attempts']:
        if record['status'] == 'RUNNING':
            continue
        out = Path(record['output'])
        result = audit(out)
        (out/'validation_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False))
        archive(out,Path(args.archive)/out.name)
        summaries.append(dict(index=record['index'],seed=record['seed'],
            **{k:result[k] for k in ('observed_sim_s','judge_deleted_count','static_contact_rows','competition_goal_met')}))
    destination=Path(args.archive);destination.mkdir(parents=True,exist_ok=True)
    shutil.copy2(root/'budget.json',destination/'budget.json')
    print(json.dumps(summaries,indent=2))


if __name__ == '__main__':
    main()
