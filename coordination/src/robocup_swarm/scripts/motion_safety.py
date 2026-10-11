"""Final horizontal constraints using measured closing velocity, without steering.

Each obstacle is a unit outward normal and its distance from the aircraft.
All constraints apply simultaneously by scaling the intended vector, preserving
already-checked direction. A zero intent can never become a recovery command.
"""
import math


class HeightClearanceHold:
    """A vanished 2D return alone cannot authorize changing scan height.

    Routine height control resumes only after horizontal escape and a fresh,
    sustained clear scan. Clock passage or repeated output calls buy no credit.
    Horizontal escape remains subject to the ordinary flight safety checks.
    """
    def __init__(self):
        self.anchor = None
        self.clear_since = None
        self.last_sample = -float('inf')

    def update(self, sample, xy, near, clear, escape_m, settle_s=.5):
        if sample <= self.last_sample:
            return self.anchor is not None
        self.last_sample = sample
        if near:
            self.anchor = xy
            self.clear_since = None
        elif self.anchor is not None:
            if not clear or math.dist(xy, self.anchor) < escape_m:
                self.clear_since = None
            else:
                if self.clear_since is None:
                    self.clear_since = sample
                if sample-self.clear_since >= settle_s:
                    self.anchor = self.clear_since = None
        return self.anchor is not None


def braking_cap(intent, measured, constraints, margin, delay, decel):
    values = (*intent,*measured,margin,delay,decel)
    if (not all(math.isfinite(v) for v in values) or
            margin < 0 or delay < 0 or decel <= 0):
        return (0.,0.),'invalid_input'
    scale,reason = 1.,''
    for nx,ny,distance,label in constraints:
        if (not all(math.isfinite(v) for v in (nx,ny,distance)) or
                abs(math.hypot(nx,ny)-1.) > 1e-5):
            return (0.,0.),'invalid_constraint'
        toward = intent[0]*nx+intent[1]*ny
        if toward <= 1e-9:
            continue  # allow tangent travel and checked return from the inset
        closing = max(0.,measured[0]*nx+measured[1]*ny)
        room = max(0.,distance-margin)
        required = closing*delay+closing*closing/(2.*decel)
        if room <= required:
            cap = 0.
            cause = 'measured_braking:'+label
        else:
            # Account for sensing/command delay in the allowed next command,
            # not only in the emergency stop threshold.
            cap = max(0.,math.sqrt((decel*delay)**2+2.*decel*room)-decel*delay)
            cause = 'braking_envelope:'+label
        candidate = min(1.,cap/toward)
        if candidate < scale:
            scale,reason = candidate,cause
    return (intent[0]*scale,intent[1]*scale),reason


def boundary_constraints(position,bounds,inset):
    x,y=position;xmin,xmax,ymin,ymax=bounds
    return [(1.,0.,xmax-inset-x,'east'),(-1.,0.,x-xmin-inset,'west'),
            (0.,1.,ymax-inset-y,'north'),(0.,-1.,y-ymin-inset,'south')]


def scan_constraints(scan,yaw,intent):
    """Finite measured beams only; blind directions are gated by the caller."""
    constraints=[]
    for i,raw in enumerate(scan.ranges):
        r=float(raw)
        if not math.isfinite(r) or not scan.range_min <= r <= scan.range_max:
            continue
        angle=scan.angle_min+i*scan.angle_increment
        motion=math.atan2(intent[1],intent[0])-yaw
        delta=math.atan2(math.sin(angle-motion),math.cos(angle-motion))
        # Retain the existing sensor mask for rear airframe echoes; when
        # travelling into this sector the raw beam must still constrain us.
        if abs(angle)>math.radians(100.) and abs(delta)>math.radians(20.):
            continue
        constraints.append((math.cos(yaw+angle),math.sin(yaw+angle),r,'lidar'))
    return constraints
