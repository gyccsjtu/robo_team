"""Summarize recorded aircraft sensors offline; never queries simulator truth."""
import argparse
import gzip
import json
import math
from pathlib import Path


def summarize(path, end=None):
    aircraft = {}
    interval = [None, None]
    complete = True
    minimum_separation = None
    try:
        with gzip.open(path, 'rt') as stream:
            for line in stream:
                event = json.loads(line)
                now = event['ros']
                if end is not None and now > end:
                    continue
                if interval[0] is None:
                    interval[0] = now
                interval[1] = now
                positions = {}
                for uid, sensor in event['aircraft'].items():
                    pose = sensor.get('pose', {})
                    xyz = pose.get('xyz')
                    if xyz is None or not 0 <= now-pose['sample_s'] <= .3:
                        continue
                    # This sampler contains the six official fixed-spawn UAVs.
                    n = int(uid)
                    world = [xyz[0]+(3. if n % 2 else 0.), xyz[1]+(-3., 0., 3.)[n//2], xyz[2]]
                    if xyz[2] < 2. and uid not in aircraft:
                        continue
                    row = aircraft.setdefault(uid, dict(samples=0, modes={},
                        min_scan_m=None, max_tilt_deg=0., max_local_z=0.,
                        min_xy=world[:2], max_xy=world[:2], max_speed_mps=0.))
                    row['samples'] += 1
                    mode = sensor.get('state', {}).get('mode', '')
                    row['modes'][mode] = row['modes'].get(mode, 0)+1
                    row['max_local_z'] = max(row['max_local_z'], xyz[2])
                    row['min_xy'] = list(map(min, row['min_xy'], world[:2]))
                    row['max_xy'] = list(map(max, row['max_xy'], world[:2]))
                    q = pose['quaternion']
                    row['max_tilt_deg'] = max(row['max_tilt_deg'], math.degrees(
                        math.acos(max(-1., min(1., 1.-2.*(q[0]**2+q[1]**2))))))
                    velocity = sensor.get('velocity')
                    if velocity and 0 <= now-velocity['sample_s'] <= .3:
                        row['max_speed_mps'] = max(row['max_speed_mps'],
                                                   math.hypot(*velocity['xyz'][:2]))
                    scan = sensor.get('scan')
                    if scan and xyz[2] >= 2. and 0 <= now-scan['sample_s'] <= .3:
                        ranges = [v for i, v in enumerate(scan['ranges']) if v is not None
                            and scan['range_min'] <= v <= scan['range_max']
                            and abs(scan['angle_min']+i*scan['angle_increment']) < math.radians(100.)]
                        if ranges:
                            old = row['min_scan_m']
                            row['min_scan_m'] = min(ranges+[old]) if old is not None else min(ranges)
                    if xyz[2] >= 2.:
                        positions[uid] = world
                for a, pa in positions.items():
                    for b, pb in positions.items():
                        if a >= b:
                            continue
                        distance = math.dist(pa, pb)
                        if minimum_separation is None or distance < minimum_separation['distance_m']:
                            minimum_separation = dict(ros=now, uavs=[a, b], distance_m=distance)
    except EOFError:
        complete = False  # a readable prefix of a recorder still writing
    return dict(source=str(path), complete_file=complete, ros_interval=interval,
        aircraft=aircraft, estimated_min_separation=minimum_separation,
        limits='Estimated MAVROS poses plus fixed spawn offsets, not ground truth. '
        'Front/side 2D scan excludes rear airframe echoes and cannot prove absence of contact. '
        'An optional end time excludes test shutdown. Mode counts are sampled records, not durations.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--end', type=float)
    args = parser.parse_args()
    result = summarize(args.record, args.end)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
