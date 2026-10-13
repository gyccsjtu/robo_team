"""Project a live tracking intention onto all nearby no-closing constraints.

Used before final safety; zero stays zero and speed never increases."""
import math

def project_no_closing(intent,normals):
    if not all(math.isfinite(v) for v in intent):return (0.,0.)
    planes=[]
    for x,y in normals:
        length=math.hypot(x,y)
        if not math.isfinite(length) or length<=1e-9:return (0.,0.)
        planes.append((x/length,y/length))
    def feasible(v):return all(v[0]*x+v[1]*y<=1e-10 for x,y in planes)
    if feasible(intent):return intent
    candidates=[(0.,0.)]
    for x,y in planes:
        closing=intent[0]*x+intent[1]*y
        v=(intent[0]-closing*x,intent[1]-closing*y)
        if feasible(v):candidates.append(v)
    return min(candidates,key=lambda v:(v[0]-intent[0])**2+(v[1]-intent[1])**2)
