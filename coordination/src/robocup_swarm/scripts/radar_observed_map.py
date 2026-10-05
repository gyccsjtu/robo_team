"""ROS-free planar ray observations. Unknown is never inferred free by distance."""
import math


class ObservedMap:
    def __init__(self, width, height, resolution, origin):
        if width <= 0 or height <= 0 or width*height > 2_000_000 or resolution <= 0:
            raise ValueError('Invalid observation grid')
        self.width, self.height, self.resolution, self.origin = width, height, resolution, tuple(origin)
        self.cells = [-1]*(width*height)
        self.observed_s = [None]*(width*height)
        self.last_scan_s = None
        self.range_min = None  # Actual sensor blind range; never invent a free disk.
        self.version = 0

    def index(self, x, y):
        ix, iy = math.floor((x-self.origin[0])/self.resolution), math.floor((y-self.origin[1])/self.resolution)
        return iy*self.width+ix if 0 <= ix < self.width and 0 <= iy < self.height else None

    def feed(self, ranges, position, yaw, angle_min, angle_increment, range_min, range_max,
             scan_s, pose_s, now_s):
        values = (*position, yaw, angle_min, angle_increment, range_min, range_max, scan_s, pose_s, now_s)
        if (not all(math.isfinite(v) for v in values) or not 0 <= now_s-scan_s <= .5
                or abs(scan_s-pose_s) > .1 or range_min < 0 or range_max <= range_min
                or angle_increment <= 0 or len(ranges) < 4
                or (self.last_scan_s is not None and scan_s <= self.last_scan_s)):
            return False
        free, occupied = set(), set()
        step = self.resolution/2
        for i, raw in enumerate(ranges):
            distance = float(raw)
            if math.isnan(distance) or distance == -math.inf or distance < range_min:
                continue  # Invalid beams contribute no evidence.
            hit = math.isfinite(distance) and distance <= range_max
            end = min(distance, range_max)
            angle = yaw+angle_min+i*angle_increment
            ux, uy = math.cos(angle), math.sin(angle)
            # Preserve the sensor's near blind zone. No filled circular free disk.
            count = max(0, int(math.ceil((end-range_min)/step)))
            for k in range(count):
                length = range_min+k*step
                index = self.index(position[0]+length*ux, position[1]+length*uy)
                if index is not None:
                    free.add(index)
            index = self.index(position[0]+end*ux, position[1]+end*uy)
            if index is not None:
                (occupied if hit else free).add(index)
        for index in free-occupied:
            self.cells[index], self.observed_s[index] = 0, scan_s
        for index in occupied:
            self.cells[index], self.observed_s[index] = 100, scan_s
        self.range_min = range_min
        self.last_scan_s, self.version = scan_s, self.version+1
        return True

    def snapshot(self, now_s, max_age=3.):
        return [value if stamp is not None and 0 <= now_s-stamp <= max_age else -1
                for value, stamp in zip(self.cells, self.observed_s)]
