"""Replay real perception callback with manager's new-task/old-cancel order."""
import ast
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
from perception import perception_real as pr


class PerceptionScopedCancelTests(unittest.TestCase):
    def setUp(self):
        tree = ast.parse(Path(pr.__file__).read_text())
        callback = next(node for node in ast.walk(tree)
                        if isinstance(node, ast.FunctionDef) and node.name == '_arb_cb')
        self.state = {'arb': {'tid': None}, 'arb_lock': threading.Lock(), 'UAV': 'u0'}
        exec(compile(ast.Module(body=[callback], type_ignores=[]), pr.__file__, 'exec'), self.state)

    def offer(self, kind, target, uid='u0'):
        self.state['_arb_cb'](NS(task_type=kind, target_id=target, uav_id=uid))
        return self.state['arb']['tid']

    def test_new_person_survives_late_cancellation_of_old_person(self):
        self.offer(1, 't0')
        self.offer(1, 't3')
        self.assertEqual(self.offer(255, 't0'), 't3')
        self.assertTrue(pr.skip_attached_verdict('white', self.state['arb']['tid'], 8.))

    def test_two_red_instances_are_target_scoped_despite_shared_colour(self):
        self.offer(1, 't4')
        self.offer(1, 't5')
        self.assertEqual(self.offer(255, 't4'), 't5')
        self.assertIsNone(self.offer(255, 't5'))

    def test_matching_global_and_search_cancellation_still_clear_owner(self):
        for kind, target in ((255, 't3'), (255, ''), (0, '')):
            with self.subTest(kind=kind, target=target):
                self.offer(1, 't3')
                self.assertIsNone(self.offer(kind, target))

    def test_other_aircraft_cannot_change_camera_owner(self):
        self.offer(1, 't3')
        self.assertEqual(self.offer(255, 't3', uid='u1'), 't3')
        self.assertEqual(self.offer(1, 't0', uid='u1'), 't3')


if __name__ == '__main__':
    unittest.main()
