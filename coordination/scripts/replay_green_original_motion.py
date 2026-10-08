"""Archival green publication-time comparison; truth is only a label."""
import argparse
import csv
import json
from pathlib import Path
import sys
import math
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'))
from yolo_target_bridge import TargetBridgeCore
from official_report_sources import OfficialReportSources
from report_readiness import ReportReadiness
from audit_coordinate_and_white_chain import truth_at, load


def replay(root):
    truth=sorted((r['sample'] for r in load(root/'observers/independent_visual_accuracy.jsonl')
                  if r.get('kind')=='actor_truth'),key=lambda r:r[0])
    times=[r[0] for r in truth]; raw={}
    for path in (root/'flight/algorithm').glob('perception_*.csv'):
        uid='uav_'+str(int(path.stem.rsplit('_',1)[1])+1)
        for row in csv.DictReader(path.open()):
            if row.get('event')=='track_publish' and row.get('class_name')=='green':
                raw[(uid,float(row['original_s']))]=[float(row['raw_target_x']),float(row['raw_target_y'])]
    systems={}
    for name in ('filtered_source','raw_source','raw_fit'):
        ready=ReportReadiness()
        systems[name]=(ready,OfficialReportSources(lambda name=name:
            TargetBridgeCore(green_alignment=name=='raw_fit'),ready))
    rows=[]; missing=0
    for record in load(root/'flight/algorithm/bridge_trace.jsonl'):
        if record['kind']=='input' and record['observation']['target_id']=='green':
            original=record['observation']; point=raw.get((original['uav_id'],original['sample_s']))
            if point is None: missing+=1
            for name,(ready,sources) in systems.items():
                obs=dict(original)
                if name!='filtered_source' and point is not None:
                    obs['xyz']=point+[original['xyz'][2]]
                ready.observe(obs,record['receipt_s'],record['accepted'])
                sources.observe(obs,record['accepted'])
        elif record['kind'] in ('upload','skip') and record.get('tag')=='green':
            now=record['receipt_s']; point=truth_at(now,truth,times)
            if point is None: continue
            values={}
            for name,(_,sources) in systems.items():
                events=sources.tick(now)
                if events:
                    uid,event,track=events[0];xy=[round(event['x'],3),round(event['y'],3)]
                    values[name]=dict(xy=xy,error_m=math.dist(xy,point),uid=uid,
                        original_s=track.t_obs,velocity=[track.vx,track.vy])
            rows.append(dict(stamp=now,values=values,actual_error_m=math.dist(record['requested_xy'],point)
                if record['kind']=='upload' else None))
    summary={}
    for name in systems:
        sampled=[r['values'][name]['error_m'] for r in rows if name in r['values']]
        longest=0.;start=None;previous=None
        for row in rows:
            error=row['values'].get(name,{}).get('error_m')
            if error is None or error>=1:
                start=previous=None;continue
            if previous is None or row['stamp']-previous>1.:
                start=row['stamp']
            previous=row['stamp'];longest=max(longest,previous-start)
        summary[name]=dict(samples=len(sampled),under_1m=sum(e<1 for e in sampled),
            maximum_m=max(sampled,default=None),longest_sampled_accurate_span_s=longest)
    common=[r for r in rows if all(name in r['values'] for name in systems)]
    summary['common_samples']=len(common)
    summary['common_under_1m']={name:sum(r['values'][name]['error_m']<1 for r in common) for name in systems}
    return dict(control_input=False,scope='recorded bridge callbacks/tick times; no new images or physics',
        missing_raw=missing,summary=summary,rows=rows)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=replay(a.run);a.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result['summary'],indent=2))
