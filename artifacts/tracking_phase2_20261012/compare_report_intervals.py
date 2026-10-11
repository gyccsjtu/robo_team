"""Offline counterfactual only. Never imports ROS or publishes real messages."""
import json, math, sys
from pathlib import Path
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'tools'))
from test_report_turn_continuity import replay, bridge
bridge._core_log=lambda *a:None
baseline=bridge.official_point_at

def fresh(track, now, cap=.15, stationary=True):
    age=now-track.good_t
    if track.good_x is None or not 0.<=age<=.35:return None
    fit=bridge._robust_official_fit(track.good_history,track.good_t)
    if fit is not None and not fit[5]:return None
    if baseline(track,track.good_t) is None:return None
    if stationary and fit is not None and math.hypot(*fit[2:4])<.15:
        return fit[:2]
    if fit is None:return track.good_x,track.good_y
    vx,vy=fit[2:4];d=math.hypot(vx,vy)*age
    scale=min(1.,cap/d) if d>0 else 1.
    return track.good_x+vx*age*scale,track.good_y+vy*age*scale

results={}
for interval in (.35,.45,.55,.6):
    bridge.PUB_MIN_INTERVAL=bridge.FAST_PUB_INTERVAL=bridge.ACTOR_KEEPALIVE_DT=interval
    bridge.official_point_at=fresh
    rows=[]
    for speed in (1.,2.):
      for angle in (math.pi/2,math.pi):
       for hz in (4.,5.):
        for corner in (8.,8.05,8.1,8.15,8.2):
         for phase in (0.,.025,.05):
          r=replay(speed,angle,hz,.2,corner,phase)
          rows.append(dict(speed=speed,angle_deg=round(math.degrees(angle)),hz=hz,corner=corner,phase=phase,max_error=r['max_error'],max_gap=r['max_gap'],count=r['count']))
    results[str(interval)]=rows
    print(interval,dict(cases=len(rows),error_gt1=sum(r['max_error']>=1 for r in rows),gap_ge1=sum(r['max_gap']>=1 for r in rows),worst_error=max(r['max_error'] for r in rows),worst_gap=max(r['max_gap'] for r in rows)),flush=True)
(root/'artifacts/tracking_phase2_20261012/report_interval_comparison.json').write_text(json.dumps(results,indent=2)+'\n')
