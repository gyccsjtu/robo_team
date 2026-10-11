#!/usr/bin/env python3
"""Synthetic calibrated pinhole projections, not recorded flight measurements.
No ROS, simulator, model-truth access or runtime changes.
Reproduces the current height formula; not a passing correctness test.
"""
import math, json
import numpy as np
FY=376.; C=np.array([0.,0.,3.]); H=1.7
M=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
cases=[]
for tilt,roll,x,y in [(0,0,7,5),(30,0,7,0),(0,20,7,0)]:
    a,b=math.radians(tilt),math.radians(roll)
    Ry=np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])
    Rx=np.array([[1.,0.,0.],[0.,math.cos(b),-math.sin(b)],[0.,math.sin(b),math.cos(b)]])
    A=Ry@Rx@M; G=np.array([x,y,0.]); T=G+np.array([0.,0.,H])
    qf=A.T@(G-C); qt=A.T@(T-C)
    uf,vf=376+FY*qf[0]/qf[2],240+FY*qf[1]/qf[2]
    ut,vt=376+FY*qt[0]/qt[2],240+FY*qt[1]/qt[2]
    hp=vf-vt
    ray=A@np.array([(uf-376)/FY,(vf-240)/FY,1.]); t=-C[2]/ray[2]; rng=t*np.linalg.norm(ray)
    code_ray=A@np.array([(uf-376)/FY,(vf-.05*hp-240)/FY,1.]); code_t=-C[2]/code_ray[2]
    code_rng=code_t*np.linalg.norm(code_ray)
    head_ray=A@np.array([(ut-376)/FY,(vt-240)/FY,1.])
    scale=float(np.dot(head_ray[:2],(G-C)[:2])/np.dot(head_ray[:2],head_ray[:2]))
    recovered=C[2]+scale*head_ray[2]
    cases.append(dict(pitch_deg=tilt,roll_deg=roll,world_xy=[x,y],head_px=[ut,vt],foot_px=[uf,vf],h_px=hp,true_height=H,optical_depth=t,slant=rng,height_by_slant=hp*rng/FY,height_by_depth=hp*t/FY,height_by_correct_head_ray=recovered,current_foot_shrink_sin_down=-code_ray[2]/np.linalg.norm(code_ray),current_foot_shrink_height=hp*code_rng/FY))
assert cases[0]['height_by_slant']>2.1
assert cases[0]['current_foot_shrink_sin_down']>.30
assert cases[0]['current_foot_shrink_height']>2.1
assert abs(cases[0]['height_by_depth']-H)<1e-9
assert abs(cases[1]['height_by_depth']-H)>.05
assert abs(cases[2]['height_by_depth']-H)>.05
assert all(abs(c['height_by_correct_head_ray']-H)<1e-9 for c in cases)
print(json.dumps(cases, indent=2))
