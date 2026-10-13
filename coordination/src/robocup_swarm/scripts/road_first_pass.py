"""Finite complementary directions on the public city road skeleton.

No map loading, actor positions, control commands or private reservations.
The manager supplies sensed-map eligibility and checked flight/view geometry;
SegmentBook remains the only reservation owner. Roads are search preferences,
not assumptions that actors stay on roads or that roads are obstacle-free.
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class RoadLane:
    name: str
    start: tuple
    end: tuple

    @property
    def heading(self):
        return math.atan2(self.end[1]-self.start[1], self.end[0]-self.start[0])


PUBLIC_FIRST_LANES = (
    RoadLane('middle_east', (45., 0.), (90., 0.)),
    RoadLane('central_east', (12., 0.), (45., 0.)),
    RoadLane('west_north', (-14., 14.), (-14., 48.)),
    RoadLane('west_south', (-14., -14.), (-14., -48.)),
)


class RoadFirstPass:
    """At most four finite jobs after the existing two east jobs allocate.

    Progress remembers executed prefixes; uncovered image samples remain in
    the normal coverage/review allocator. An interruption releases ownership
    without discarding the prefix or renewing the first-pass time budget.
    """
    def __init__(self, budget_s=90., lanes=PUBLIC_FIRST_LANES):
        self.lanes = tuple(lanes)[:4]
        self.budget_s = float(budget_s)
        self.started_s = None
        self.owners = {}
        self.bindings = {}
        self.visited = {lane.name: set() for lane in self.lanes}
        self.retry_after = {}

    def remaining(self, now):
        return self.budget_s if self.started_s is None else max(0., self.started_s+self.budget_s-now)

    def lane_cells(self, lane, coverage):
        dx, dy = lane.end[0]-lane.start[0], lane.end[1]-lane.start[1]
        length = math.hypot(dx, dy)
        if length <= 0.:
            return ()
        result = []
        for key, cell in coverage.cells.items():
            x, y = cell.cx-lane.start[0], cell.cy-lane.start[1]
            along, cross = (x*dx+y*dy)/length, abs(x*dy-y*dx)/length
            if -.5*coverage.cell_m <= along <= length+.5*coverage.cell_m and cross <= .5*coverage.cell_m:
                result.append((along, key))
        return tuple(key for _, key in sorted(result))

    def choose(self, coverage, positions, now, eligible, build):
        """Return (lane, uid, cells, points, looks), without claiming anything.

        build(uid, cells, heading) performs actual sensor-map checks and may
        shorten to <=3 points. False proposals cannot acquire a lane owner.
        """
        if not positions or self.remaining(now) <= 0.:
            return None
        active = set(self.bindings.values())
        peer_owners = set(self.owners.values())
        for lane in self.lanes:
            if lane.name in active:
                continue
            owner = self.owners.get(lane.name)
            if owner is not None and owner not in positions:
                self.owners.pop(lane.name, None)
                peer_owners.discard(owner)
            candidates = [key for key in self.lane_cells(lane, coverage)
                          if key not in self.visited[lane.name] and eligible(key) and
                          now >= self.retry_after.get((lane.name, key), -math.inf)]
            if not candidates:
                continue
            owner = self.owners.get(lane.name)
            uids = ([owner] if owner in positions else [] if owner is not None else
                    sorted((uid for uid in positions if uid not in peer_owners),
                           key=lambda uid: (math.dist(positions[uid], lane.start), uid)))
            for uid in uids:
                # Try a bounded sensed prefix; unknown/unreachable cells may
                # not force a blind route or claim this whole public road.
                for offset in range(min(6, len(candidates))):
                    # Look samples on two adjacent grid edges can be only
                    # .02 m apart. Supply a bounded longer candidate list so
                    # the builder can skip such a cell without reserving it.
                    proposal = build(uid, candidates[offset:offset+6], lane.heading)
                    if proposal is not None:
                        cells, points, looks = proposal
                        return lane.name, uid, cells, points, looks
        return None

    def started(self, task, lane):
        if self.started_s is None:
            self.started_s = task.created
        self.owners[lane] = task.uid
        self.bindings[task.task_id] = lane

    def progress(self, task):
        lane = self.bindings.get(task.task_id)
        if lane is not None:
            self.visited[lane].update(task.cells[:task.index])

    def ended(self, task, reason, now):
        lane = self.bindings.get(task.task_id)
        if lane is None:
            return
        self.progress(task)
        self.bindings.pop(task.task_id, None)
        if reason != 'complete':
            self.owners.pop(lane, None)
            if reason != 'person_task':
                for key in task.cells[task.index:]:
                    self.retry_after[(lane, key)] = now+8.
