"""Bounded image-only approach. Never supplies an actor/world coordinate."""
import math


def approach_image(data):
    """Only a full, modest-size, upright view permits an exploratory approach."""
    try:
        box = tuple(float(v) for v in data['box'])
        w, h = float(data['width']), float(data['height'])
        tilt = float(data['camera_tilt'])
        if len(box) != 4 or not all(math.isfinite(v) for v in (*box, w, h, tilt)):
            return None
        x1, y1, x2, y2 = box
        if (w <= 0 or h <= 0 or not 8 <= x1 < x2 <= w-8 or
                not 8 <= y1 < y2 <= .83*h or
                not 12 <= y2-y1 <= .32*h or
                abs(tilt) > math.radians(8)):
            return None
        return dict(box=box, width=w, height=h, camera_tilt=tilt)
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def candidate_velocity(cue, now, yaw, xy, scan, scan_stamp):
    """Approach at <=.95m/s for <=6s/3m, with fresh view and clear raw rays.

    The returned request STILL goes through all flight safety guards. Unknown
    range is never converted into a target position. Missing/invalid scan or
    image means stop, not optimistic free space.
    """
    image = cue.get('image')
    origin = cue.get('approach_origin')
    if (image is None or origin is None or cue.get('xy') is not None or
            not 0 <= now-cue['sample_s'] <= .35 or
            not 0 <= now-cue['started'] < 6. or
            math.hypot(xy[0]-origin[0], xy[1]-origin[1]) >= 3. or
            scan is None or not 0 <= now-scan_stamp <= .3):
        return 0., 0.
    error = math.atan2(math.sin(cue['bearing']-yaw), math.cos(cue['bearing']-yaw))
    if abs(error) > math.radians(12):
        return 0., 0.
    clear = 0
    try:
        if not math.isfinite(scan.range_max) or scan.range_max < 4.:
            return 0., 0.
        for i, raw in enumerate(scan.ranges):
            angle = scan.angle_min+i*scan.angle_increment
            delta = math.atan2(math.sin(angle-error), math.cos(angle-error))
            if abs(delta) > math.radians(30):
                continue
            r = float(raw)
            if math.isinf(r) and r > 0:  # valid no-return, not NaN/negative inf
                clear += 1
            elif math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                if r < 4.:
                    return 0., 0.
                clear += 1
            else:
                return 0., 0.
    except (AttributeError, TypeError, ValueError):
        return 0., 0.
    if clear < 3:
        return 0., 0.
    # Slow down before the box grows out of the admissible approach band.
    height = image['box'][3]-image['box'][1]
    speed = min(.95, max(0., (.32-height/image['height'])*5.))
    return speed*math.cos(cue['bearing']), speed*math.sin(cue['bearing'])


class ImagePublishClock:
    """Image-time cadence without phase drift or repeat/coast publications."""
    def __init__(self, hz):
        self.period = 1. / max(.1, float(hz))
        self.next_due = None
        self.last_sample = -float('inf')

    def ready(self, sample):
        if not math.isfinite(sample) or sample <= self.last_sample:
            return False
        self.last_sample = sample
        if self.next_due is None:
            self.next_due = sample
        if sample+1e-7 < self.next_due:
            return False
        steps = max(1, int(math.floor((sample-self.next_due+1e-7)/self.period))+1)
        self.next_due += steps*self.period
        return True
