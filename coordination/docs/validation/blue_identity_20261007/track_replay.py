"""Actual Track methods with reconstructed raw-to-track association, not ROS replay."""
import bisect,csv,json,math,pathlib,sys
sys.path.insert(0,'/mnt/d/a/.robocup/robo_team/coordination/tests')
from test_blue_motion_identity import actual_track_class

def replay(root):
 root=pathlib.Path(root)
 truth=[json.loads(l)['sample'] for l in (root/'observers/independent_visual_accuracy_blue.jsonl').open() if json.loads(l)['kind']=='actor_truth']
 times=[r[0] for r in truth]
 def error(t,xy):
  i=bisect.bisect_left(times,t)
  if not 0<i<len(truth) or truth[i][0]-truth[i-1][0]>.5:return None
  a,b=truth[i-1],truth[i];f=(t-a[0])/(b[0]-a[0])
  return math.hypot(xy[0]-(a[1]+f*(b[1]-a[1])),xy[1]-(a[2]+f*(b[2]-a[2])))
 samples=[]
 for file in (root/'flight/algorithm').glob('perception*.csv'):
  rows=list(csv.DictReader(file.open()));raw={}
  for r in rows:
   if r['event']=='yolo_detection' and r['class_name']=='blue':raw.setdefault(float(r['image_stamp']),[]).append(r)
  tracks={};published=set();classes=[actual_track_class(False),actual_track_class(True)]
  for r in rows:
   if r['class_name']!='blue' or not r.get('track_id'):continue
   stamp=float(r['image_stamp']);candidates=raw.get(stamp,[])
   if not candidates:continue
   q=min(candidates,key=lambda q:math.hypot(float(q['bbox_u'])-float(r['bbox_u']),float(q['bbox_v'])-float(r['bbox_v'])))
   if math.hypot(float(q['bbox_u'])-float(r['bbox_u']),float(q['bbox_v'])-float(r['bbox_v']))>1.:continue
   xy=(float(q['target_x']),float(q['target_y']));uv=tuple(float(q[k]) for k in ('bbox_u','bbox_v'));wh=tuple(float(q[k]) for k in ('bbox_w','bbox_h'))
   arguments=[float(q['confidence']),uv,wh,float(q['range_m']),float(q['height_m']),stamp]
   key=r['track_id']
   if key not in tracks:tracks[key]=[cls('blue',*xy,*arguments) for cls in classes]
   else:
    for tr in tracks[key]:
     if stamp>tr.observed_s:tr.update(*xy,stamp-tr.observed_s,*arguments)
   if r['event']!='track_publish' or (key,stamp) in published:continue
   published.add((key,stamp))
   old,new=tracks[key];now=float(r['ros_time'])
   samples.append(dict(camera=file.stem,track=key,stamp=stamp,error=error(stamp,(float(r['target_x']),float(r['target_y']))),
      old_verdict=old.verdict(now,attach_check=False),new_verdict=new.verdict(now,attach_check=False),
      identity_allowed=new.blue_identity.allowed(now)))
 groups={}
 for name,predicate in [('accurate',lambda r:r['error'] is not None and r['error']<1.),('far_error',lambda r:r['error'] is not None and r['error']>10.),('other',lambda r:r['error'] is None or 1.<=r['error']<=10.)]:
  rs=[r for r in samples if predicate(r)]
  groups[name]=dict(count=len(rs),old_allowed=sum(r['old_verdict'][0] for r in rs),new_allowed=sum(r['new_verdict'][0] for r in rs),identity_allowed=sum(r['identity_allowed'] for r in rs))
 return dict(root=str(root),groups=groups,samples=samples)

result=[replay('/root/robocup_runs/'+run+'/round_1_seed_8159') for run in ['competition_recovery_r1_20261007','competition_recovery_r2_20261007','heartbeat_v137_20261006T020228Z']]
out=pathlib.Path(__file__).resolve().parent;out.mkdir(exist_ok=True)
(out/'track_replay.json').write_text(json.dumps({'runs':result,'limitations':['Actual Track init/update/verdict methods; raw-to-track identity reconstructed by timestamp and <=1px bbox centre.','Track birth is earliest matched logged raw detection; unseen earlier measurements are unavailable.','No coasting/association/appearance/camera attachment callbacks are reconstructed; attach_check=False in both variants.','Offline truth labels refer to historic published coordinates, not new predicted geometry.','No actual new ROS pipeline or judge continuity is proven.']},indent=2)+'\n')
for r in result:print(r['root'],r['groups'])
