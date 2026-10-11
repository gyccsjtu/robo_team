"""Offline comparison of recorder events; never subscribes or controls flight."""
import argparse
import collections
import gzip
import json
import math
from pathlib import Path


def search_key(data):
    if data['task']:
        return data['task']
    if data['cell'][0] >= 0 and data['cell'][1] >= 0:
        return ('legacy',*data['cell'],*data['point'])
    return None


def summarize(path, start=None, end=None):
    counts = collections.Counter()
    states = collections.defaultdict(collections.Counter)
    travel = collections.defaultdict(lambda: [0., 0., 0.])
    last = {}
    first_detection = {}
    tasks = {}
    conflicts = []
    first_s = last_s = None
    complete_file = True
    try:
        with gzip.open(path, 'rt') as stream:
            for line in stream:
                event = json.loads(line)
                kind, now, data = event['event'], event['ros'], event['data']
                if start is not None and now < start or end is not None and now > end:
                    continue
                counts[kind] += 1
                first_s = now if first_s is None else first_s
                last_s = now
                if kind == 'discovery':
                    for candidate in data.get('candidates', []):
                        if candidate.get('real_detection') and candidate.get('geometry_ok'):
                            first_detection.setdefault(candidate['target_id'], dict(
                                sample_s=candidate['sample_s'], source=candidate['source_uav']))
                if kind == 'status':
                    uid = data['uav']
                    previous = last.get(uid)
                    if previous and search_key(data) and search_key(data) == search_key(previous[1]):
                        dt = data['stamp']-previous[1]['stamp']
                        if 0 < dt <= .5:
                            states[uid][str(data['state'])] += dt
                            distance = math.dist((data['x'], data['y']),
                                                 (previous[1]['x'], previous[1]['y']))
                            travel[uid][0] += dt
                            travel[uid][1] += dt if distance/dt < .2 else 0.
                            travel[uid][2] += distance
                    last[uid] = (now, data)
                    task = tasks.get(uid)
                    if task and (task['task'],task['version']) == (data['task'],data['version']):
                        task['index'] = max(task.get('index',0),data['index'])
                elif kind == 'assignment':
                    uid = data['uav']
                    previous = tasks.get(uid)
                    if data['type'] == 0 and data['task']:
                        data['index'] = (previous.get('index',0) if previous and
                            (previous['task'],previous['version']) == (data['task'],data['version']) else 0)
                        tasks[uid] = data
                        reserved = {tuple(k) for k in data['cells'][data['index']:]}
                        for other, task in tasks.items():
                            if other == uid or task['until'] <= now:
                                continue
                            overlap = reserved & {tuple(k) for k in task['cells'][task['index']:]}
                            if overlap:
                                conflicts.append(dict(ros=now,uavs=[uid,other],cells=sorted(overlap)))
                    elif data['type'] != 0:
                        if not data['task'] or (previous and (previous['task'],previous['version']) ==
                                               (data['task'],data['version'])):
                            tasks.pop(uid,None)
    except EOFError:
        complete_file = False  # valid prefix of an active recorder
    return dict(source=str(path),complete_file=complete_file,ros_interval=[first_s,last_s],
        events=dict(counts),first_geometric_visual_detection=first_detection,
        search_motion={uid:dict(sampled_seconds=t[0],stopped_seconds=t[1],
            stopped_fraction=t[1]/t[0] if t[0] else None,distance_m=t[2],
            state_seconds=dict(states[uid])) for uid,t in travel.items()},
        observed_reservation_conflicts=conflicts,
        limits='Only the captured interval is evaluated. Local-colour IDs are not six verified identities. '
               'Stop fraction uses reported pose, '
               'not command or truth. This record alone cannot prove collision absence or coverage.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('events', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--start',type=float)
    parser.add_argument('--end',type=float)
    args = parser.parse_args()
    result = summarize(args.events,args.start,args.end)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
