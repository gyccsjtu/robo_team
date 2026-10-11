"""Horizontal camera intersections from image-time onboard poses only.

Caller owns instance association. A new track must get a new accumulator.
No ROS, actor coordinates, map landmarks or judge feedback are inputs.
"""
import math


def camera_bearing(camera, box, intrinsics=(376., 376., 376., 240.),
                   image_size=(752., 480.)):
    """Return a world ENU azimuth for a complete box and camera-link pose."""
    if camera is None:
        return None
    try:
        px, py, pz, qx, qy, qz, qw = map(float, camera)
        x1, y1, x2, y2 = map(float, box)
        fx, fy, cx, cy = intrinsics
        if not all(math.isfinite(v) for v in (*camera, *box)):
            return None
        if not (6. < x1 < x2 < image_size[0]-6. and
                6. < y1 < y2 < image_size[1]-6.):
            return None
        norm = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
        if norm < .5 or norm > 1.5:
            return None
        qx, qy, qz, qw = (v/norm for v in (qx, qy, qz, qw))
        # optical +Z forward, +X right, +Y down -> camera link FLU
        y, z = -(0.5*(x1+x2)-cx)/fx, -(0.5*(y1+y2)-cy)/fy
        dx = 1-2*(qy*qy+qz*qz)+2*(qx*qy-qz*qw)*y+2*(qx*qz+qy*qw)*z
        dy = 2*(qx*qy+qz*qw)+(1-2*(qx*qx+qz*qz))*y+2*(qy*qz-qx*qw)*z
        length = math.hypot(dx, dy)
        if length < .2:
            return None
        return dict(origin=[px, py], direction=[dx/length, dy/length])
    except (TypeError, ValueError, ZeroDivisionError):
        return None


def validate_bearing(value):
    """An invalid optional extension cannot discard a valid legacy report."""
    try:
        if not isinstance(value, dict):
            return None
        origin, direction = value['origin'], value['direction']
        if len(origin) != 2 or len(direction) != 2:
            return None
        ox, oy, dx, dy = map(float, (*origin, *direction))
        if not all(math.isfinite(v) for v in (ox, oy, dx, dy)):
            return None
        n = math.hypot(dx, dy)
        if not .999 <= n <= 1.001:
            return None
        return dict(origin=(ox, oy), direction=(dx/n, dy/n))
    except (KeyError, TypeError, ValueError):
        return None


def intersection(a, b):
    """Pair nearly simultaneous views of an already associated instance."""
    if (a['source_uav'] == b['source_uav'] or a['tag'] != b['tag'] or
            abs(a['sample_s']-b['sample_s']) > .15+1e-9):
        return None
    if math.hypot(a['x']-b['x'], a['y']-b['y']) > 3.:
        return None
    av, bv = a['bearing'], b['bearing']
    ax, ay = av['origin']; bx, by = bv['origin']
    dx, dy = av['direction']; ex, ey = bv['direction']
    cross = dx*ey-dy*ex
    if abs(cross) < math.sin(math.radians(25.)):
        return None
    da = ((bx-ax)*ey-(by-ay)*ex)/cross
    db = ((bx-ax)*dy-(by-ay)*dx)/cross
    if not (3. <= da <= 20. and 3. <= db <= 20.):
        return None
    x, y = ax+da*dx, ay+da*dy
    if max(math.hypot(x-r['x'], y-r['y']) for r in (a, b)) > 3.:
        return None
    return (.5*(a['sample_s']+b['sample_s']), x, y)


class MultiViewTrack:
    def __init__(self):
        self.views = {}
        self.pairs = {}

    def add(self, row):
        if (not row.get('official_ok') or not row.get('candidate_id') or
                row.get('appearance_status') in ('ambiguous', 'distractor') or
                not row.get('bearing')):
            return
        source, t = row['source_uav'], row['sample_s']
        old = self.views.get(source)
        if old is not None and t <= old['sample_s']:
            return
        self.views[source] = row
        self.views = {s:r for s,r in self.views.items() if t-r['sample_s'] <= 2.}
        self.pairs = {k:v for k,v in self.pairs.items() if t-v[0][-1][0] <= 2.}
        for other in self.views.values():
            point = intersection(row, other)
            if point is None:
                continue
            a, b = sorted((row, other), key=lambda r:r['source_uav'])
            key = (a['source_uav'], a['candidate_id'], b['source_uav'], b['candidate_id'])
            history, stamps = self.pairs.get(key, ([], (-1., -1.)))
            # Both images must be new. One cached image is never three votes.
            new_stamps = (a['sample_s'], b['sample_s'])
            if not all(n > p for n,p in zip(new_stamps, stamps)):
                continue
            history = [p for p in history if 0. <= point[0]-p[0] <= 2.]
            self.pairs[key] = (history+[point], new_stamps)

    def point_at(self, now, fit_function, fallback):
        choices = []
        for key, (history, stamps) in self.pairs.items():
            # Intersection already tolerates 0.15 s between two exposures.
            # Do not add the monocular 0.6 s model horizon on top: an actor
            # can turn after both cameras last saw it. Fresh monocular data
            # is the fallback, not an older extrapolated intersection.
            if not all(0. <= now-s <= .35 for s in stamps):
                continue
            if any(self.views.get(source, {}).get('candidate_id') != candidate
                   for source,candidate in (key[:2], key[2:])):
                continue
            fit = fit_function(history, history[-1][0])
            if fit is None or not fit[5]:
                continue
            age = now-history[-1][0]
            point = (fit[0]+age*fit[2], fit[1]+age*fit[3])
            if math.hypot(point[0]-fallback[0], point[1]-fallback[1]) > 1.5:
                continue
            choices.append((min(stamps), key, point))
        return max(choices)[2] if choices else None
