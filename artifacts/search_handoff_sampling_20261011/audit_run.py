"""External file-only validation. Never imported by competition nodes."""
import bisect, collections, gzip, json, math, re, sys
from pathlib import Path
root=Path(__file__).resolve().parent
manifest=json.loads((root/'trial_manifest.json').read_text())
logs=Path(manifest['logdir'])
judge=(logs/'03_judge.log').read_text()
elims={int(a):float(t) for a,t in re.findall(r'actor_(\d+) is OK\s+Time usage:\s*([\d.]+)',judge)}
truth=collections.defaultdict(list);missing={};uavs=collections.defaultdict(list)
reports=[];sources=[];hints=[];states=[];view_count=collections.Counter();events=collections.Counter()
sample_file=root/'read_only_accuracy.jsonl'
sample_stream=sample_file.open() if sample_file.exists() else gzip.open(str(sample_file)+'.gz','rt')
for line in sample_stream:
 try:d=json.loads(line)
 except ValueError:continue
 ev=d['event'];events[ev]+=1
 if ev=='truth':
  a=d['actor']
  if d['success']:truth[a].append(((d['ros']+d['request_ros'])/2,d['x'],d['y']))
  elif truth[a]:missing.setdefault(a,d['ros'])
 elif ev=='uav_truth' and d['success']:uavs[d['uav']].append((d['ros'],d['x'],d['y'],d['z']))
 elif ev=='report':reports.append(d)
 elif ev=='visual_source':sources.append(d['data'])
 elif ev=='search_view_hint':hints.append(d['data'])
 elif ev=='fcu':states.append(d)
 elif ev=='tracking_view':
  v=d['data'];view_count[(v['source_uav'],v['target_id'],'images')]+=1
  if v.get('report_ok'):view_count[(v['source_uav'],v['target_id'],'report_ok')]+=1
  if v['box'][3]>.94*v['height']:view_count[(v['source_uav'],v['target_id'],'near_bottom')]+=1
sample_stream.close()
times={a:[p[0] for p in ps] for a,ps in truth.items()}
def position(a,t):
 ts=times[a];i=bisect.bisect_left(ts,t)
 if not 0<i<len(ts):return None
 p,q=truth[a][i-1:i+1]
 if q[0]-p[0]>.6:return None
 f=(t-p[0])/(q[0]-p[0]);return p[1]+f*(q[1]-p[1]),p[2]+f*(q[2]-p[2])
start_estimates=[(truth[a][-1][0]+missing[a])/2-elims[a] for a in elims if a in missing]
start=sorted(start_estimates)[len(start_estimates)//2] if start_estimates else None
finish=start+max(elims.values()) if start is not None and len(elims)==6 and 'Mission finished' in judge else None
def in_match(t):
 return start is not None and start<=t and (finish is None or t<=finish)
def log_time(line):
 match=re.search(r'\[\d+\.\d+,\s*([\d.]+)\]',line)
 return float(match.group(1)) if match else float('inf')
metrics={};bad=[];ids={'green':[0],'blue':[1],'brown':[2],'white':[3],'red1':[4,5],'red2':[4,5]}
for tag,aa in ids.items():
 rows=[d for d in reports if d['tag']==tag];es=[];excluded=0
 for d in rows:
  if any(a in missing and truth[a][-1][0]<=d['ros']<=missing[a]+1 and math.hypot(d['x']-truth[a][-1][1],d['y']-truth[a][-1][2])<2 for a in aa):excluded+=1;continue
  dist=[]
  for a in aa:
   p=position(a,d['ros'])
   if p is not None:dist.append((math.hypot(d['x']-p[0],d['y']-p[1]),a))
  if not dist:continue
  error,a=min(dist);es.append(error)
  if error>=1:bad.append(dict(tag=tag,actor=a,ros=d['ros'],error=error))
 gaps=[b['ros']-a['ros'] for a,b in zip(rows,rows[1:])]
 metrics[tag]=dict(reports=len(rows),evaluated=len(es),under_1m=sum(e<1 for e in es),max_error=max(es) if es else None,max_gap=max(gaps) if gaps else None,gaps_over_1s=sum(g>1 for g in gaps),deletion_ambiguity=excluded,first_report_elapsed=rows[0]['ros']-start if rows and start else None)
motion={}
for a in elims:
 if not truth[a]:continue
 ps=[p for p in truth[a] if p[0]>=truth[a][-1][0]-15]
 motion[a]=dict(last_15s_displacement=math.hypot(ps[-1][1]-ps[0][1],ps[-1][2]-ps[0][2]),last_15s_path=sum(math.hypot(q[1]-p[1],q[2]-p[2]) for p,q in zip(ps,ps[1:])))
first_source={}
for a in range(6):
 candidates=[d for d in sources if d['target_id']=='t'+str(a)] if a<4 else [d for d in sources if d['target_id'] in ('t4','t5')]
 for d in candidates:
  p=position(a,d['sample_s'])
  if p and math.hypot(p[0]-d['x'],p[1]-d['y'])<3:
   first_source[a]=dict(elapsed=d['sample_s']-start if start else None,uav=d['source_uav'],sample=d['sample_s']);break
manager=(logs/'06_swarm_manager.log').read_text()
support=[l for l in manager.splitlines() if '[SEARCH_PROGRESS_SUPPORT]' in l]
hint_lines=[]
for path in logs.glob('06_swarm_agent_*.log'):
 hint_lines += [dict(file=path.name,line=l) for l in path.read_text().splitlines() if '[SEARCH_VIEW_HINT]' in l or '[SEARCH_VIEW_HINT_END]' in l]
out=dict(commit=manifest['commit'],official=elims,finished='Mission finished' in judge,start_ros_estimate=start,report_metrics=metrics,errors_over_1m=bad,motion=motion,first_spatially_matching_source=first_source,view_counts={':'.join(k):v for k,v in view_count.items()},support=support,hint_assignments=hints,hint_execution=hint_lines,dynamic_reassignments=manager.count('原航点受阻，明确重派安全视点'),blocked_cancellations=manager.count('已无安全视点，取消并回收租约'),failsafe_samples=sum(d.get('armed') and d.get('mode') in ('AUTO.LAND','AUTO.RTL') for d in states),events=events,scope='Truth is external file-only. XY interpolated <=.6s; excludes deletion ambiguity. First-source <3m is diagnostic spatial agreement, not report eligibility; for red both slots are considered. Official elapsed estimate has sampling uncertainty. No blanket no-collision certification.')
active_manager=[line for line in manager.splitlines() if in_match(log_time(line))]
active_hints=[d for d in hint_lines if in_match(log_time(d['line']))]
unsafe=[d for d in states if d.get('armed') and d.get('mode') in ('AUTO.LAND','AUTO.RTL')]
out['match_phase']=dict(
 finish_ros_estimate=finish,
 support_jobs=sum('[SEARCH_PROGRESS_SUPPORT]' in line for line in active_manager),
 direction_observed=sum('result=direction_observed' in d['line'] for d in active_hints),
 direction_timeouts=sum('result=look_timeout' in d['line'] for d in active_hints),
 direction_expired=sum('result=expired_or_untrusted' in d['line'] for d in active_hints),
 dynamic_reassignments=sum('原航点受阻，明确重派安全视点' in line for line in active_manager),
 blocked_cancellations=sum('已无安全视点，取消并回收租约' in line for line in active_manager),
 failsafe_samples=sum(in_match(d['ros']) for d in unsafe))
out['post_finish_failsafe_first']={}
for d in unsafe:
 if finish is not None and d['ros']>finish:
  out['post_finish_failsafe_first'].setdefault(str(d['uav']),dict(mode=d['mode'],elapsed=d['ros']-start))
out['report_totals']=dict(reports=sum(v['reports'] for v in metrics.values()),evaluated=sum(v['evaluated'] for v in metrics.values()),at_least_1m=len(bad))
(root/'validation_result.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({k:out[k] for k in ('official','finished','report_metrics','first_spatially_matching_source','match_phase','report_totals','post_finish_failsafe_first')},indent=2))
