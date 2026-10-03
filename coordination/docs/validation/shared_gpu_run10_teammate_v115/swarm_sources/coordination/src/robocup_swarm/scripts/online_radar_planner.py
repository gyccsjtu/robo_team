"""Observed-space planning with unknown blocking and connected frontier progress."""
from collections import deque
import math
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
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

    def grid(self, observed, position, now, epoch):
        values = observed.snapshot(now)
        base = GridMap(observed.width, observed.height, observed.resolution, observed.origin,
                       bytes(0 if value == 0 else 1 for value in values), 'map')
        if epoch != self.epoch:
            self.body_proof.clear()
            self.epoch = epoch
            # Only the externally verified startup patch can initialize the blind region.
            for iy in range(base.height):
                for ix in range(base.width):
                    point = base.cell_to_world((ix, iy))
                    if math.dist(point, self.seed_xy) <= self.seed_radius:
                        self.body_proof.add(iy*base.width+ix)
        body_cells = set()
        center = base.world_to_cell(position)
        if center is not None:
            # Inflation tests cell centers out to radius + half a cell. Retain
            # the same already-certified footprint; the smaller disk erased
            # diagonal startup cells and made our own start become occupied.
            retained_radius = self.radius+base.resolution/2
            extent = math.ceil(retained_radius/base.resolution)
            for dy in range(-extent, extent+1):
                for dx in range(-extent, extent+1):
                    cell = center[0]+dx, center[1]+dy
                    if base.in_bounds(cell) and math.dist(base.cell_to_world(cell), position) <= retained_radius:
                        body_cells.add(cell[1]*base.width+cell[0])
        # Carry only already certified free cells under the current protected footprint.
        # New unknown cells are never added. Fresh hits revoke the carried certificate.
        self.body_proof &= body_cells
        self.body_proof |= {i for i in body_cells if values[i] == 0}
        self.body_proof -= {i for i in body_cells if values[i] == 100}
        raw = bytearray(base.cells)
        for i in self.body_proof:
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

    def route(self, grid, start, goal):
        initial = grid.world_to_cell(start)
        if initial is None or not grid.is_free(initial):
            return dict(ok=False, reason='START_CLEARANCE_UNKNOWN', points=())
        seen = {initial}
        queue = deque([initial])
        while queue:
            cell = queue.popleft()
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                other = cell[0]+dx, cell[1]+dy
                if other not in seen and grid.is_free(other):
                    seen.add(other)
                    queue.append(other)
        target = grid.world_to_cell(goal)
        complete = target in seen
        if not complete:
            target = min(seen, key=lambda cell: (math.dist(grid.cell_to_world(cell), goal), cell))
            if math.dist(grid.cell_to_world(target), goal) >= math.dist(start, goal)-.5:
                return dict(ok=False, reason='NO_REACHABLE_PROGRESS', points=())
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
