"""Observed-space planning with unknown blocking and connected frontier progress."""
from collections import deque
import math
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import threading
from robocup_navigation.astar import GridMap, plan


def fixture_seed(path, run_id, uid):
    certificate = json.loads(Path(path).read_text())
    world = Path(certificate['world_path']).read_bytes()
    root = ET.fromstring(world)
    node = root.find('world')
    if certificate.get('schema_version') == 3:
        point = certificate['positions'][uid]
        check = certificate['startup_checks'][uid]
        if (certificate.get('purpose') != 'DEVELOPMENT_CITY_STARTUP'
                or certificate['run_id'] != run_id or node is None
                or hashlib.sha256(world).hexdigest() != certificate['world_sha256']
                or len(point) != 2 or not all(math.isfinite(v) for v in point)
                or check.get('free_radius_m') != 2.):
            raise ValueError('Invalid city startup configuration')
        return tuple(point)
    if (certificate['schema_version'] not in (1, 2) or certificate['run_id'] != run_id
            or hashlib.sha256(world).hexdigest() != certificate['world_sha256']
            or node is None or node.findall('actor') or node.findall('plugin')
            or {n.findtext('uri') for n in node.findall('include')} != {'model://ground_plane', 'model://sun'}):
        raise ValueError('Unverified fixture startup clearance')
    if certificate['schema_version'] == 1 and node.findall('model'):
        raise ValueError('Version 1 requires an empty fixture')
    boxes = []
    for model in node.findall('model'):
        links = model.findall('link')
        if model.findtext('static') != 'true' or len(links) != 1 or model.findall('plugin') or model.findall('joint'):
            raise ValueError('Only verified static boxes are supported')
        link = links[0]
        collisions = link.findall('collision')
        pose = [float(v) for v in model.findtext('pose', '').split()]
        if (len(collisions) != 1 or len(pose) != 6 or not all(math.isfinite(v) for v in pose)
                or any(pose[3:]) or link.find('pose') is not None or collisions[0].find('pose') is not None
                or link.findall('plugin')):
            raise ValueError('Unsupported box transform')
        size = [float(v) for v in collisions[0].findtext('geometry/box/size', '').split()]
        if len(size) != 3 or not all(math.isfinite(v) and v > 0 for v in size):
            raise ValueError('Unsupported fixture geometry')
        for sensor in link.findall('sensor'):
            plugins = sensor.findall('plugin')
            if (sensor.get('type') != 'contact' or len(plugins) != 1
                    or plugins[0].get('filename') != 'libgazebo_ros_bumper.so'):
                raise ValueError('Unsupported fixture sensor')
        boxes.append((pose[:3], size))
    point = certificate['positions'][uid]
    if len(point) != 2 or not all(math.isfinite(v) for v in point):
        raise ValueError('Invalid startup point')
    for center, size in boxes:
        distance = math.hypot(max(abs(point[0]-center[0])-size[0]/2, 0),
                              max(abs(point[1]-center[1])-size[1]/2, 0))
        if distance <= 2.:
            raise ValueError('Startup clearance intersects a fixture box')
    return tuple(point)


class OnlinePlanner:
    def __init__(self, seed_xy, seed_radius=2., radius=1.2):
        self.seed_xy, self.seed_radius, self.radius = tuple(seed_xy), seed_radius, radius
        self.body_proof = set()
        self.epoch = None
        self._body_lock = threading.RLock()
        self._body_sample_s = None
        self._local_key = None
        self._local_free = {}
        self._frontier_visits = {}

    def local_command_clear(self, observed, position, velocity, now, epoch,
                            latency=.5, brake=.5):
        """Current observed stopping corridor, independent of A* completion.

        Cache only footprint queries within one scan version. Unknown cells
        need existing body proof; new hits always win. Caller locks the map.
        """
        if (observed.last_scan_s is None or not 0 <= now-observed.last_scan_s <= .5
                or not all(math.isfinite(v) for v in (*position,*velocity,now))):
            return False
        proof = self._retain_body(observed,
            {i: observed.cells[i] if observed.observed_s[i] is not None
                and 0 <= now-observed.observed_s[i] <= 3. else -1
             for i in self._body_cells(observed,position)},
            position,epoch,observed.last_scan_s)
        key = (epoch,observed.version,frozenset(proof))
        if key != self._local_key:
            self._local_key, self._local_free = key, {}
        res = observed.resolution
        radius_cells = math.ceil(self.radius/res)
        offsets = [(dx,dy) for dx in range(-radius_cells,radius_cells+1)
                   for dy in range(-radius_cells,radius_cells+1)
                   if math.hypot(dx,dy)*res <= self.radius+res/2]
        speed = math.hypot(*velocity)
        distance = speed*latency+speed*speed/(2*brake)
        steps = max(1,math.ceil(distance/(res/3)))
        for k in range(steps+1):
            x = position[0]+velocity[0]/max(speed,1e-9)*distance*k/steps
            y = position[1]+velocity[1]/max(speed,1e-9)*distance*k/steps
            cell = (math.floor((x-observed.origin[0])/res),math.floor((y-observed.origin[1])/res))
            if cell not in self._local_free or now > self._local_free[cell][1]:
                free = True
                valid_until = float('inf')
                for dx,dy in offsets:
                    ix,iy = cell[0]+dx,cell[1]+dy
                    if not (0 <= ix < observed.width and 0 <= iy < observed.height):
                        free = False; break
                    i = iy*observed.width+ix
                    stamp = observed.observed_s[i]
                    value = observed.cells[i] if stamp is not None and 0 <= now-stamp <= 3. else -1
                    if value == 0:
                        valid_until = min(valid_until,stamp+3.)
                    if value != 0 and not (value == -1 and i in proof):
                        free = False; break
                self._local_free[cell] = (free,valid_until)
            if not self._local_free[cell][0]:
                return False
        return True

    def _body_cells(self, observed, position):
        resolution = observed.resolution
        ix = math.floor((position[0]-observed.origin[0])/resolution)
        iy = math.floor((position[1]-observed.origin[1])/resolution)
        retained_radius = self.radius+resolution/2
        extent = math.ceil(retained_radius/resolution)
        cells = set()
        for dy in range(-extent, extent+1):
            for dx in range(-extent, extent+1):
                x, y = ix+dx, iy+dy
                if not (0 <= x < observed.width and 0 <= y < observed.height):
                    continue
                point = (observed.origin[0]+(x+.5)*resolution,
                         observed.origin[1]+(y+.5)*resolution)
                if math.dist(point, position) <= retained_radius:
                    cells.add(y*observed.width+x)
        return cells

    def _retain_body(self, observed, values, position, epoch, sample_s):
        body_cells = self._body_cells(observed, position)
        with self._body_lock:
            # A planner's frozen snapshot must not roll back newer scan proof.
            if self._body_sample_s is not None and sample_s < self._body_sample_s:
                return self.body_proof & body_cells if epoch == self.epoch else set()
            if epoch != self.epoch:
                self.body_proof.clear()
                self._frontier_visits.clear()
                self.epoch = epoch
                for i in body_cells:
                    x, y = i % observed.width, i // observed.width
                    point = (observed.origin[0]+(x+.5)*observed.resolution,
                             observed.origin[1]+(y+.5)*observed.resolution)
                    if math.dist(point, self.seed_xy) <= self.seed_radius:
                        self.body_proof.add(i)
            self.body_proof &= body_cells
            self.body_proof |= {i for i in body_cells if values[i] == 0}
            self.body_proof -= {i for i in body_cells if values[i] == 100}
            self._body_sample_s = sample_s
            return set(self.body_proof)

    def carry_body_proof(self, observed, position, now, epoch):
        """Carry previously observed cells under the body on each fresh scan.

        Planning is intermittent; the robot can cross an old free cell before
        that cell enters the scan's 0.5m blind zone. Unknown cells are never
        added, and hits revoke proof. This does not certify any future route.
        """
        if observed.last_scan_s is None or not 0 <= now-observed.last_scan_s <= .5:
            return
        cells = self._body_cells(observed, position)
        values = {i: observed.cells[i] if observed.observed_s[i] is not None
                  and 0 <= now-observed.observed_s[i] <= 3. else -1 for i in cells}
        self._retain_body(observed, values, position, epoch, observed.last_scan_s)

    def grid(self, observed, position, now, epoch):
        values = observed.snapshot(now)
        base = GridMap(observed.width, observed.height, observed.resolution, observed.origin,
                       bytes(0 if value == 0 else 1 for value in values), 'map')
        proof = self._retain_body(observed, values, position, epoch,
                                  observed.last_scan_s if observed.last_scan_s is not None else now)
        raw = bytearray(base.cells)
        for i in proof:
            if values[i] == -1:
                raw[i] = 0
        radius_cells = math.ceil(self.radius/base.resolution)
        safe = bytearray(raw)
        offsets = [(dx, dy) for dx in range(-radius_cells, radius_cells+1)
                   for dy in range(-radius_cells, radius_cells+1)
                   if math.hypot(dx, dy)*base.resolution <= self.radius+base.resolution/2]
        for iy in range(base.height):
            for ix in range(base.width):
                index = iy*base.width+ix
                if raw[index] == 0 and any(not (0 <= ix+dx < base.width and 0 <= iy+dy < base.height)
                    or raw[(iy+dy)*base.width+ix+dx] != 0 for dx, dy in offsets):
                    safe[index] = 1
        return GridMap(base.width, base.height, base.resolution, base.origin, bytes(safe), 'map')

    def route(self, grid, start, goal, observed=None, now=None):
        initial = grid.world_to_cell(start)
        if initial is None or not grid.is_free(initial):
            return dict(ok=False, reason='START_CLEARANCE_UNKNOWN', points=())
        seen = {initial}
        hops = {initial: 0}
        queue = deque([initial])
        while queue:
            cell = queue.popleft()
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                other = cell[0]+dx, cell[1]+dy
                if other not in seen and grid.is_free(other):
                    seen.add(other)
                    hops[other] = hops[cell]+1
                    queue.append(other)
        target = grid.world_to_cell(goal)
        complete = target in seen
        if not complete:
            target = min(seen, key=lambda cell: (math.dist(grid.cell_to_world(cell), goal), cell))
            if math.dist(grid.cell_to_world(target), goal) >= math.dist(start, goal)-.5:
                if observed is None or now is None:
                    return dict(ok=False, reason='NO_REACHABLE_PROGRESS', points=())
                values = observed.snapshot(now)
                self._frontier_visits = {p:s for p,s in self._frontier_visits.items() if 0 <= now-s < 30.}
                extent = math.ceil((self.radius+2*grid.resolution)/grid.resolution)
                ring = [(dx,dy) for dx in range(-extent,extent+1) for dy in range(-extent,extent+1)
                        if self.radius < math.hypot(dx,dy)*grid.resolution <= self.radius+2*grid.resolution]
                candidates = []
                for cell in seen:
                    point = grid.cell_to_world(cell)
                    if math.dist(point,start) < .5 or any(math.dist(point,p) < 1. for p in self._frontier_visits):
                        continue
                    unknown = sum(1 for dx,dy in ring
                        if 0 <= cell[0]+dx < grid.width and 0 <= cell[1]+dy < grid.height
                        and values[(cell[1]+dy)*grid.width+cell[0]+dx] == -1)
                    if unknown:
                        candidates.append((-unknown,hops[cell],math.dist(point,goal),cell))
                if not candidates:
                    return dict(ok=False, reason='NO_REACHABLE_PROGRESS', points=())
                target = min(candidates)[-1]
                self._frontier_visits[grid.cell_to_world(target)] = now
        endpoint = goal if complete else grid.cell_to_world(target)
        result = plan(grid, start, endpoint, connectivity=4)
        if not result.success:
            return dict(ok=False, reason=result.reason, points=())
        points = (tuple(start),) + tuple(result.points) + (tuple(endpoint),)
        return dict(ok=True, reason='GOAL_OBSERVED' if complete else 'OBSERVED_FRONTIER', points=points)

    @staticmethod
    def connector_clear(grid, start, end):
        """Check every crossed grid cell, including cells touched at corners."""
        if grid is None or any(grid.world_to_cell(p) is None for p in (start,end)):
            return False
        a=[(start[i]-grid.origin[i])/grid.resolution for i in (0,1)]
        b=[(end[i]-grid.origin[i])/grid.resolution for i in (0,1)]
        cuts={0.,1.}
        for axis in (0,1):
            delta=b[axis]-a[axis]
            if abs(delta)>1e-12:
                for edge in range(math.floor(min(a[axis],b[axis]))+1,
                                  math.ceil(max(a[axis],b[axis]))):
                    fraction=(edge-a[axis])/delta
                    if 0 < fraction < 1:
                        cuts.add(fraction)
        ordered=sorted(cuts)
        samples=ordered+[(x+y)/2 for x,y in zip(ordered,ordered[1:])]
        for fraction in samples:
            axes=[]
            for axis in (0,1):
                value=a[axis]+fraction*(b[axis]-a[axis])
                edge=round(value)
                axes.append((edge-1,edge) if abs(value-edge)<1e-9 else (math.floor(value),))
            if any(not grid.is_free((ix,iy)) for ix in axes[0] for iy in axes[1]):
                return False
        return True

    @classmethod
    def visible_goal(cls, grid, position, prefix):
        # Prefer the farthest original path point whose connecting segment stays
        # inside known free space; no straight shortcut through an inflated corner.
        for point in reversed(prefix):
            if math.dist(position,point) > .05 and cls.connector_clear(grid,position,point):
                return tuple(point)
        return None

    @staticmethod
    def command_clear(grid, position, velocity, radius=1.2, latency=.5, brake=.5):
        speed = math.hypot(*velocity)
        if speed < 1e-8:
            return True
        distance = speed*latency+speed*speed/(2*brake)
        steps = max(1, math.ceil(distance/(grid.resolution/3)))
        for k in range(steps+1):
            cell = grid.world_to_cell((position[0]+velocity[0]/speed*distance*k/steps,
                                      position[1]+velocity[1]/speed*distance*k/steps))
            if cell is None or not grid.is_free(cell):
                return False
        return True
