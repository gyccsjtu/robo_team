"""Read-only reconstruction; associations are not a replay of track callbacks."""
import bisect, csv, json, math, pathlib, sys
sys.path.insert(0, '/mnt/d/a/.robocup/robo_team/perception')
from recent_motion import RecentMotion

def analyze(root):
    root = pathlib.Path(root)
    truth = []
    for line in (root/'observers/independent_visual_accuracy_blue.jsonl').open():
        row = json.loads(line)
        if row.get('kind') == 'actor_truth':
            truth.append(row['sample'])
    truth.sort()
    times = [p[0] for p in truth]
    out = []
    for f in (root/'flight/algorithm').glob('perception*.csv'):
        rows = list(csv.DictReader(f.open()))
        raw = {}
        for r in rows:
            if r['event'] == 'yolo_detection' and r['class_name'] == 'blue':
                raw.setdefault(float(r['image_stamp']), []).append(r)
        tracks, published = {}, set()
        for r in rows:
            if r['class_name'] != 'blue' or not r.get('track_id'): continue
            stamp = float(r['image_stamp'])
            candidates = raw.get(stamp, [])
            if not candidates: continue
            candidate = min(candidates, key=lambda q: math.hypot(
                float(q['bbox_u'])-float(r['bbox_u']), float(q['bbox_v'])-float(r['bbox_v'])))
            if math.hypot(float(candidate['bbox_u'])-float(r['bbox_u']),
                          float(candidate['bbox_v'])-float(r['bbox_v'])) > 1.: continue
            motion = tracks.setdefault(r['track_id'], RecentMotion())
            motion.observe(stamp, float(candidate['target_x']), float(candidate['target_y']))
            if r['event'] != 'track_publish': continue
            key = (r['track_id'], stamp)
            if key in published: continue
            published.add(key)
            idx = bisect.bisect_left(times, stamp)
            error = None
            if 0 < idx < len(truth) and truth[idx][0]-truth[idx-1][0] <= .5:
                a,b = truth[idx-1],truth[idx]
                fraction = (stamp-a[0])/(b[0]-a[0])
                xy = [a[k]+fraction*(b[k]-a[k]) for k in (1,2)]
                error = math.hypot(float(r['target_x'])-xy[0],float(r['target_y'])-xy[1])
            samples = list(motion.samples)
            span = samples[-1][0]-samples[0][0]
            displacement = math.hypot(samples[-1][1]-samples[0][1],samples[-1][2]-samples[0][2])
            out.append(dict(camera=f.stem,track=r['track_id'],stamp=stamp,
                error=error,speed=motion.speed(stamp),span=span,displacement=displacement,
                count=len(samples)))
    groups = {}
    for name,predicate in [('accurate',lambda p:p['error'] is not None and p['error']<1.),
                           ('far_error',lambda p:p['error'] is not None and p['error']>10.),
                           ('other',lambda p:p['error'] is None or 1.<=p['error']<=10.)]:
        group = [p for p in out if predicate(p)]
        groups[name] = dict(count=len(group),
            pass_speed_05=sum(p['speed']>=.5 for p in group),
            pass_displacement_25=sum(p['displacement']>=2.5 for p in group),
            measurable=sum(p['span']>=1. and p['count']>=2 for p in group))
    return dict(root=str(root),limitations=[
        'Raw-to-track association reconstructed by same stamp and bbox center within 1 px.',
        'Only logged matched raw measurements are used; missing update history may change speed.',
        'Far error includes NPC and other errors; accurate labels are offline truth only.',
        'Threshold counts are diagnostics, not production acceptance or continuous judge timing.'],
        groups=groups,published_samples=out)

print(json.dumps([analyze(p) for p in sys.argv[1:]],indent=2))
