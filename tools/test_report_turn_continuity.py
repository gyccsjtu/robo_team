"""Actual bridge callbacks/ticks with exact images and known motion."""
import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                     'coordination/src/robocup_swarm/scripts'))
import yolo_target_bridge as bridge


def replay(speed=1., angle=math.pi/2, image_hz=4., latency=.2,
           corner=8., tick_phase=0., noise=None, tag='blue', skip_image=None):
    node = bridge._mk_bridge()
    emitted = []
    node._actor_pubs['blue'].publish = lambda m: emitted.append((node._now(), m.x, m.y))
    def position(t):
        dt = t-100.
        return (30.+speed*min(dt, corner)+speed*max(0., dt-corner)*math.cos(angle),
                10.+speed*max(0., dt-corner)*math.sin(angle))
    events = [(100.+i/image_hz+latency, 0, 100.+i/image_hz)
              for i in range(int(18.*image_hz))]
    events += [(100.+tick_phase+i/15., 1, None) for i in range(270)]
    for now, kind, sample in sorted(events):
        node._advance(now-node._now())
        if kind == 0:
            if skip_image is not None and skip_image(sample):
                continue
            x, y = position(sample)
            node._report_cb(bridge._FakeString(json.dumps(dict(
                target_id='blue', xyz=[x, y, 1.25], confidence=.95,
                range_m=8., sample_s=sample, source_uav='u0',
                candidate_id='camera:person', real_detection=True, official_view_ok=True))))
        else:
            node._tick(None)
    gaps = [b[0]-a[0] for a, b in zip(emitted, emitted[1:])]
    errors = [math.hypot(x-position(t)[0], y-position(t)[1]) for t, x, y in emitted]
    return dict(max_gap=max(gaps), max_error=max(errors), count=len(emitted),
                corner_reports=[r for r in emitted if 106.<r[0]<112.])


class TurnReportContinuityTests(unittest.TestCase):
    def test_accurate_continuous_images_do_not_break_reports_at_a_corner(self):
        for speed in (1., 2.):
            for angle in (0., math.pi/2):
                with self.subTest(speed=speed, angle=angle):
                    result = replay(speed=speed, angle=angle)
                    self.assertLess(result['max_gap'], 1.)
                    self.assertLess(result['max_error'], 1.)

    def test_turn_recovery_does_not_depend_on_image_timer_phase(self):
        # This is a continuity contract only. Some fast corners already have
        # a prediction error above 1 m before a second turn image arrives;
        # those accuracy limits are recorded separately, not hidden here.
        for speed in (1., 2.):
            for hz in (4., 5.):
                for corner in (8., 8.05, 8.1, 8.15, 8.2):
                    for phase in (0., .025, .05):
                        with self.subTest(speed=speed, hz=hz, corner=corner, phase=phase):
                            result = replay(speed=speed, image_hz=hz,
                                            corner=corner, tick_phase=phase)
                            self.assertLess(result['max_gap'], 1.)


if __name__ == '__main__':
    unittest.main()
