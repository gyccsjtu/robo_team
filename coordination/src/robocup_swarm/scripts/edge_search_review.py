"""Bounded inward/along-edge looks, using permitted map and camera data only."""
import math

from search_observation import cell_points, in_view, observable_points, next_heading


def review_points(grid, camera, bounds, visible):
    """Legal ground points behind/along the nearest edges, never beyond them."""
    _, x, y, _, altitude, pitch = camera
    xmin, xmax, ymin, ymax = bounds
    margins = (x-xmin, xmax-x, y-ymin, ymax-y)
    if min(margins) < 0. or min(margins) > 9.:
        return {}
    points = []
    for cell in grid.cells.values():
        if math.hypot(cell.cx-x, cell.cy-y) > 9.+grid.cell_m:
            continue
        for px, py in cell_points(grid, cell):
            if not xmin <= px <= xmax or not ymin <= py <= ymax:
                continue
            # At a corner both edge normals apply. Along-edge bearings are
            # allowed; a search look must not continue towards an outer wall.
            if ((margins[0] <= 9. and px < x-.1) or
                    (margins[1] <= 9. and px > x+.1) or
                    (margins[2] <= 9. and py < y-.1) or
                    (margins[3] <= 9. and py > y+.1)):
                continue
            points.append((px, py))
    return observable_points(points, x, y, altitude, pitch, visible)


class EdgeReview:
    """One finite review per nearby place; images cannot renew its deadline.

    Seen points here only finish this look. Real coverage still belongs to the
    existing processed-frame callback, with no task or target writes here.
    """
    def __init__(self, bounds, budget=8., cooldown=60., spacing=10., dwell=.25):
        self.bounds = bounds
        self.budget, self.cooldown, self.spacing, self.dwell = budget, cooldown, spacing, dwell
        self.active = None
        self.visits = []
        self.last_result = None

    def cancel(self, reason='preempted'):
        if self.active is not None:
            self.last_result = dict(reason=reason, started=self.active['started'])
        self.active = None

    def step(self, key, now, own, camera, grid, visible):
        active = self.active
        if active is not None and (active['key'] != key or
                now < active['started'] or math.dist(own, active['origin']) > 1.5):
            self.cancel('changed_visit')
            active = None
        fresh = (camera is not None and 0. <= now-camera[0] <= .5 and
                 math.dist(own, camera[1:3]) <= 1.)
        if active is None:
            self.visits = [v for v in self.visits if 0. <= now-v[0] < self.cooldown][-32:]
            if not fresh or any(math.dist(own, xy) <= self.spacing for _, xy in self.visits):
                return None
            points = review_points(grid, camera, self.bounds, visible)
            if not points:
                return None
            active = self.active = dict(key=key, started=now, origin=own,
                                        points=points, seen=set(), first={})
            self.visits.append((now, own))
        if now-active['started'] >= self.budget:
            self.cancel('timeout')
            return None
        if not fresh:
            return float('nan')  # hold safely; never infer a successful look
        sample, x, y, yaw, altitude, pitch = camera
        if sample >= active['started']:
            for index, point in active['points'].items():
                if index in active['seen']:
                    continue
                if not in_view(x, y, yaw, *point, altitude, pitch) or not visible(x,y,*point):
                    active['first'].pop(index, None)
                    continue
                prior = active['first'].get(index)
                if prior is None or sample < prior[1] or sample-prior[1] > .7:
                    active['first'][index] = (sample, sample)
                elif sample > prior[1]:
                    active['first'][index] = (prior[0], sample)
                    if sample-prior[0] >= self.dwell:
                        active['seen'].add(index)
        missing = {i:p for i,p in active['points'].items() if i not in active['seen']}
        if not missing:
            self.cancel('imaged')
            return None
        return next_heading(missing, x, y, yaw)
