"""Skipping an observation stop requires captured image evidence, not arrival."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'coordination/src/robocup_swarm/scripts'))
from search_transit_view import remember_transit_views, can_pass_imaged_view


class SearchTransitViewTests(unittest.TestCase):
    def task(self):
        return dict(purpose='coverage_view', created=100., index=0,
                    cells=((1, 0), (2, 0)), look_points=((8.5, 0.), (15.5, 0.)))

    def frame(self, task, sample, visible=True, yaw=0., height=4.):
        remember_transit_views(task, (sample, 0., 0., yaw, height, 0.),
                               lambda *_: visible)

    def test_arrival_old_or_repeated_images_do_not_prove_observation(self):
        task = self.task()
        self.assertFalse(can_pass_imaged_view(task, 100.))
        self.frame(task, 99.9)
        self.frame(task, 100.)
        for _ in range(30): self.frame(task, 100.)
        self.assertFalse(can_pass_imaged_view(task, 100.3))
        self.frame(task, 100.2)
        self.assertTrue(can_pass_imaged_view(task, 100.3))
        self.assertFalse(can_pass_imaged_view(task, 102.3))

    def test_invisible_wrong_heading_impossible_height_and_image_gaps(self):
        for changes in (dict(visible=False), dict(yaw=3.14), dict(height=6.)):
            task = self.task()
            self.frame(task, 100., **changes)
            self.frame(task, 100.2, **changes)
            self.assertFalse(can_pass_imaged_view(task, 100.3))
        task = self.task()
        self.frame(task, 100.)
        self.frame(task, 100.8)
        self.assertFalse(can_pass_imaged_view(task, 100.9))
        self.frame(task, 101.)
        self.assertTrue(can_pass_imaged_view(task, 101.1))

    def test_task_revision_person_transition_and_final_point_keep_stop(self):
        task = self.task()
        self.frame(task, 100.)
        self.frame(task, 100.2)
        self.assertTrue(can_pass_imaged_view(task, 100.3))
        self.assertFalse(can_pass_imaged_view(self.task(), 100.3))
        task['purpose'] = 'building_review'
        self.assertFalse(can_pass_imaged_view(task, 100.3))
        task['purpose'] = 'coverage_view'; task['index'] = 1
        self.assertFalse(can_pass_imaged_view(task, 100.3))
        self.assertNotIn('covered', task)


if __name__ == '__main__': unittest.main()
