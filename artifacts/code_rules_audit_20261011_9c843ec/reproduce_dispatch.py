#!/usr/bin/env python3
"""Read-only source audit: no ROS imports, simulator, subscriptions or publishers.
The fake publisher below is an in-memory list.append, not ROS.
This records CURRENT DEFECTS, not desired regression behavior.
Fixtures isolate named methods; surrounding control and planning are not simulated.
"""
import ast
import math
import types
import json
import sys
sys.dont_write_bytecode = True
from pathlib import Path
ROOT=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[2]
BASE=ROOT/'coordination/src/robocup_swarm/scripts'
CLOCK=[0.0]
N=types.SimpleNamespace
ROSPY=N(Time=N(now=lambda:N(to_sec=lambda:CLOCK[0])), logwarn_throttle=lambda *a:None, loginfo_throttle=lambda *a:None, logerr_throttle=lambda *a:None)
def functions(file,names,ns):
    tree=ast.parse((BASE/file).read_text())
    nodes={n.name:n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
    for name in names:
        node=nodes[name]
        node.decorator_list=[]
        exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(BASE/file),'exec'),ns)
    return ns
ns=functions('swarm_manager.py',['_predict_target','visual_streak_should_release','_in_processed_view','joint_target_assignment'],dict(math=math,rospy=ROSPY,PREDICT_ACTOR_VMAX=3.,CONFIRM_HOLD_TIMEOUT=30.,VISUAL_STREAK_MAX_HOLD_S=45.))
results={}
for cached in [False,True]:
    owner=N(_target_hist={})
    for k in range(31 if cached else 7):
        t=k*.2 if cached else float(k)
        CLOCK[0]=t
        out=ns['_predict_target'](owner,'t0',float(math.floor(t+1e-6)),0.,1.)
    results['prediction_'+('cached_5Hz' if cached else 'fresh_1Hz')]=dict(last_t=t,target_x=6.,lead_s=1.,output=out)
results['dispatch_seen_vs_blind']=dict(seen_distance_m=9.,blind_distance_m=5.,costs={'seeing':3.,'blind':5/3.+1},chosen=ns['joint_target_assignment']({'t0':{'seeing':3.,'blind':5/3.+1}}))
view=ns['_in_processed_view']
results['coverage']=dict(uav=[0.,0.,2.8],yaw=0.,cell_m=7.,center=[8.,0.],center_visible=view(0.,0.,0.,8.,0.,2.8,0.),same_cell_corner=[11.5,3.5],corner_visible=view(0.,0.,0.,11.5,3.5,2.8,0.))
ctns={'__name__':'audit_import'}
exec(compile((BASE/'cooperative_tracker.py').read_text(),str(BASE/'cooperative_tracker.py'),'exec'),ctns)
target=ctns['CooperativeTarget']('t0',strict_samples=True,official_only=True,gap_tol=1.)
target.set_observers(['u0'])
done=None;events=[];release=[]
for k in range(2001):
    now=k*.1
    if k%170 <= 155: target.report('u0',now,0.,0.)
    ev=target.update(now)
    if ev=='confirmed': done=now;events.append(round(now,1))
    if done is not None and ns['visual_streak_should_release'](done,now,target.covered(now)):
        release.append(round(now,1))
results['tenure']=dict(continuous_s=15.5,blackout_s=1.5,duration_s=200,confirmed_events=events,release_due_times=release,last_done=done,absolute_tenure_branch_skipped=done is not None)
def pose():
    return N(header=N(),pose=N(position=N(x=0.,y=0.,z=0.),orientation=N(z=0.,w=0.)))
gl=dict(math=math,rospy=ROSPY,ALT_TAKEOFF=2.,SAFE_3D=5.,FLIGHT_X_MIN=-50.,FLIGHT_X_MAX=150.,FLIGHT_Y_MIN=-60.,FLIGHT_Y_MAX=60.,FLIGHT_INSET=2.,MAP_GUARD_MARGIN=2.,MAP_GUARD_SOFT=6.,RADAR_GUARD=1,RADAR_FRESH_S=.5,RADAR_STOP_R=1.6,RADAR_SIDE_PANIC_R=1.6,RADAR_SIDE_PANIC_SPEED=.5,MAX_SPEED=6.,MAX_ACC=2.5,POS_SP_LEAD=1.,POS_SP_STEP_MAX=6.,PoseStamped=pose,ALT_RECOVER_CEIL=4.5,YAW_RATE_MAX=1.,POS_SPV=0)
a=functions('swarm_agent.py',['flight_bounds','flight_waypoint','_apply_friend_avoidance','_final_scan_cap','_map_guard_velocity','_publish_pos_output'],gl)
CLOCK[0]=100.
f=N(world_xy=(0.,0.),local_z=3.,altitude_layer=3.,_friend_positions={'u1':(.4,0.,3.)},_friend_seen={'u1':100.},_view_follow_velocity=lambda x,y:(x,y),_adaptive_speed=lambda x,y:(x,y))
results['friend_close_skip']=dict(own=[0.,0.,3.],friend=[.4,0.,3.],desired=[1.,0.],output=a['_apply_friend_avoidance'](f,1.,0.))
# Match the native 270-degree laser: rearward directions outside +/-135 deg lack beams.
scan=N(ranges=[12.]*720,angle_min=-math.radians(135),angle_increment=math.radians(270)/719.,range_min=.1,range_max=12.)
output=[]
p=N(local_xy=(149.,0.),world_xy=(149.,0.),local_z=3.,yaw=0.,offset=(0.,0.),_takeoff_done=True,_landing=False,_scan=scan,_scan_t=100.,pos_pub=N(publish=output.append))
incoming=(0.,.1)
checked=a['_final_scan_cap'](p,*incoming)
bounded=a['_map_guard_velocity'](p,*checked)
a['_publish_pos_output'](p,*bounded,3.,0.,0.)
sp=output[-1].pose.position
actual=(sp.x-p.local_xy[0],sp.y-p.local_xy[1])
results['position_clip_turn']=dict(world=p.world_xy,input=incoming,after_final_scan=checked,after_map=bounded,published_world_sp=[sp.x,sp.y,sp.z],actual_step=actual,scan_verdict_for_actual_step=a['_final_scan_cap'](p,*actual),note='Final setpoint creates an unscanned rearward displacement; this is a deterministic command-chain failure, not a claim of an observed collision.')
# Zero command outside the envelope is converted to a nonzero position move.
p.local_xy=p.world_xy=(150.5,0.)
a['_publish_pos_output'](p,0.,0.,3.,0.,0.)
sp=output[-1].pose.position
results['position_clip_zero_stop']=dict(world=p.world_xy,input=[0.,0.],published_world_sp=[sp.x,sp.y,sp.z],actual_step=[sp.x-p.local_xy[0],sp.y-p.local_xy[1]])
print(json.dumps(results,ensure_ascii=False,indent=2))
