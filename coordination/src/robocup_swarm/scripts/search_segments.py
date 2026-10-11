"""Short search reservations and route geometry, independent of ROS and coverage.

Coverage remains an image-evidence property of CoverageGrid. Neither passing a
waypoint nor releasing a reservation grants coverage. All callers share the
manager task lock for selection AND reservation, including cross-zone support.
"""
import math
import threading
import uuid
from dataclasses import dataclass

NONE, TRAVEL, LOOK, ATTEND, YIELD, DONE, FAILED = range(7)


@dataclass
class Segment:
    uid: str
    task_id: str
    revision: int
    cells: tuple
    points: tuple
    created: float
    deadline: float
    expires: float
    index: int = 0
    feedback_s: float = -1.
    progress_s: float = 0.
    progress: object = None
    wait_since: object = None
    yield_replanned: bool = False
    blocked_since: object = None


class SegmentBook:
    def __init__(self, lock=None):
        self.lock = lock if lock is not None else threading.RLock()
        self.tasks = {}
        self.reserved = {}  # cell -> task id; persists even after image coverage

    def allocate(self, uid, choose, now, ttl, max_hold):
        """Call choose(reserved_keys), then reserve all its cells atomically."""
        with self.lock:
            if uid in self.tasks:
                return None
            proposal = choose(set(self.reserved))
            if proposal is None:
                return None
            return self.reserve(uid, *proposal, now, ttl, max_hold)

    @staticmethod
    def validate(cells, points):
        if not 1 <= len(cells) <= 3 or len(cells) != len(points):
            raise ValueError('search segment must contain one to three paired cells/points')
        if len(set(cells)) != len(cells):
            raise ValueError('duplicate cell in segment')
        if any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('nonfinite search point')

    def reserve(self, uid, cells, points, now, ttl, max_hold):
        with self.lock:
            cells, points = tuple(cells), tuple(points)
            self.validate(cells, points)
            if uid in self.tasks or any(k in self.reserved for k in cells):
                return None
            task = Segment(uid, uuid.uuid4().hex, 1, cells, points, now,
                           now+max_hold, min(now+ttl, now+max_hold), progress_s=now)
            self.tasks[uid] = task
            for key in cells:
                self.reserved[key] = task.task_id
            return task

    def matches(self, uid, task_id, revision):
        task = self.tasks.get(uid)
        return task is not None and (task.task_id, task.revision) == (task_id, revision)

    def advance(self, uid, task_id, revision, index):
        with self.lock:
            if not self.matches(uid, task_id, revision):
                return None
            task = self.tasks[uid]
            if not task.index <= index <= len(task.cells):
                return None
            visited = task.cells[task.index:index]
            for key in visited:
                if self.reserved.get(key) == task.task_id:
                    self.reserved.pop(key)
            task.index = index
            return visited

    def revise(self, uid, cells, points, now):
        with self.lock:
            task = self.tasks[uid]
            cells, points = tuple(cells), tuple(points)
            self.validate(cells, points)
            if any(self.reserved.get(k, task.task_id) != task.task_id for k in cells):
                return None
            old_remaining = task.cells[task.index:]
            for key in old_remaining:
                if self.reserved.get(key) == task.task_id:
                    self.reserved.pop(key)
            for key in cells:
                self.reserved[key] = task.task_id
            task.cells, task.points, task.index = cells, points, 0
            task.revision += 1
            task.feedback_s = -1.
            # Replanning does not buy another maximum-hold or wait budget.
            task.progress = None
            return task

    def release(self, uid, task_id=None, revision=None):
        with self.lock:
            task = self.tasks.get(uid)
            if task is None or (task_id is not None and
                    not self.matches(uid, task_id, revision)):
                return None
            self.tasks.pop(uid)
            for key in task.cells[task.index:]:
                if self.reserved.get(key) == task.task_id:
                    self.reserved.pop(key)
            return task


def clear_line(grid, start, end):
    """Conservative sampled corridor; unknown/outside is not a checked link."""
    if grid is None:
        return False
    distance = math.dist(start, end)
    count = max(1, int(math.ceil(distance / max(.05, grid.resolution*.35))))
    previous = None
    for i in range(count+1):
        f = i/float(count)
        cell = grid.world_to_cell((start[0]+f*(end[0]-start[0]),
                                   start[1]+f*(end[1]-start[1])))
        if cell is None or not grid.is_free(cell):
            return False
        if previous is not None and cell[0] != previous[0] and cell[1] != previous[1]:
            if not grid.is_free((cell[0],previous[1])) or not grid.is_free((previous[0],cell[1])):
                return False
        previous = cell
    return True


def aligned(a, b, c, max_turn_deg=35.):
    ab, bc = (b[0]-a[0],b[1]-a[1]), (c[0]-b[0],c[1]-b[1])
    scale = math.hypot(*ab)*math.hypot(*bc)
    return scale > .01 and (ab[0]*bc[0]+ab[1]*bc[1])/scale >= math.cos(math.radians(max_turn_deg))


def path_remaining(path, xy):
    """Project on a local path segment; report arc length, not goal distance."""
    if not path or xy is None:
        return -1.
    if len(path) == 1:
        return math.dist(xy,path[0])
    suffix = [0.]*len(path)
    for i in range(len(path)-2,-1,-1):
        suffix[i] = suffix[i+1]+math.dist(path[i],path[i+1])
    best = None
    for i,(a,b) in enumerate(zip(path,path[1:])):
        dx,dy = b[0]-a[0],b[1]-a[1]
        length2 = dx*dx+dy*dy
        f = max(0.,min(1.,((xy[0]-a[0])*dx+(xy[1]-a[1])*dy)/max(1e-12,length2)))
        distance = math.hypot(xy[0]-a[0]-f*dx,xy[1]-a[1]-f*dy)
        remaining = math.sqrt(length2)*(1-f)+suffix[i+1]
        # At an exact crossing choose the earlier portion conservatively.
        candidate = (distance,-remaining)
        if best is None or candidate < best:
            best = candidate
    return -best[1]
