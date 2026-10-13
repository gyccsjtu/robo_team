"""A moving person needs full-foot view; old frames cannot enable faster motion."""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'coordination/src/robocup_swarm/scripts'))
from tracking_view import update_cropped_view_pressure, cropped_view_pressure_ready


class CroppedViewPressureTests(unittest.TestCase):
    def view(self, sample=100., **changes):
        result = dict(target_id='t0', world_candidate_id='u0:session:6',
                      track_id=6, sample_s=sample, height=480.,
                      box=(320., 300., 365., 480.), geometry_ok=True,
                      person_verdict_ok=True, report_ok=False,
                      needs_space=True, camera_tilt=0.)
        result.update(changes)
        return result

    def test_three_continuous_real_frames_are_required(self):
        pressure = None
        for i, sample in enumerate((100., 100.2, 100.4)):
            view = self.view(sample)
            pressure = update_cropped_view_pressure(pressure, view)
            self.assertEqual(cropped_view_pressure_ready(pressure, view), i == 2)

    def test_duplicates_never_accumulate_and_new_identity_restarts(self):
        view = self.view()
        state = update_cropped_view_pressure(None, view)
        for _ in range(50):
            state = update_cropped_view_pressure(state, view)
        self.assertEqual(state['count'], 1)
        for sample in (100.2, 100.4):
            state = update_cropped_view_pressure(state, self.view(sample))
        self.assertFalse(cropped_view_pressure_ready(state,
            self.view(100.4, world_candidate_id='other_instance')))
        changed = self.view(100.4, world_candidate_id='u0:session:7')
        state = update_cropped_view_pressure(state, changed)
        self.assertFalse(cropped_view_pressure_ready(state, changed))

    def test_image_gap_and_bad_frames_restart_pressure(self):
        state = None
        for sample in (100., 100.2, 100.4):
            state = update_cropped_view_pressure(state, self.view(sample))
        view = self.view(101.)
        state = update_cropped_view_pressure(state, view)
        self.assertFalse(cropped_view_pressure_ready(state, view))
        cases = (dict(geometry_ok=False), dict(person_verdict_ok=False),
                 dict(report_ok=True), dict(pixel_only=True),
                 dict(needs_space=False), dict(camera_tilt=math.radians(9.)),
                 dict(box=(320., 300., 365., 440.)),
                 dict(box=(320., 0., 365., 480.)))
        for change in cases:
            self.assertIsNone(update_cropped_view_pressure(state,
                              self.view(101.2, **change)), change)


if __name__ == '__main__':
    unittest.main()
