"""Regression for real swarm search lease, independent of ROS."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/robocup_swarm/scripts'))
from swarm_task import CoverageGrid, LeaseManager, TaskAllocator, STATE_ASSIGNED


class SearchLeaseTests(unittest.TestCase):
    def setUp(self):
        self.grid = CoverageGrid(0, 20, 0, 10, cell_m=10, num_uavs=2)
        self.lease = LeaseManager(self.grid, duration=3.)
        self.lease.grant('uav_1', (0, 0), 0., duration=3.)

    def test_expiry_keeps_owner_and_excludes_reauction(self):
        self.assertEqual(self.lease.expire(4.), [(0, 0)])
        cell = self.grid.cell((0, 0))
        self.assertEqual((cell.owner, cell.state), ('uav_1', STATE_ASSIGNED))
        self.assertNotIn((0, 0), self.grid.uncovered_cells())
        assignments = TaskAllocator(self.grid).allocate({'uav_2': (5., 5.)})
        self.assertNotIn((0, 0), assignments.values())

    def test_another_owner_cannot_take_expired_task(self):
        self.lease.expire(4.)
        self.assertFalse(self.lease.grant('uav_2', (0, 0), 5., duration=3.))
        self.assertFalse(self.lease.renew('uav_2', (0, 0), 5.))
        self.assertEqual(self.grid.cell((0, 0)).owner, 'uav_1')

    def test_repeated_expiry_is_reported_once_and_same_owner_can_resume(self):
        self.lease.expire(4.)
        self.assertEqual(self.lease.expire(5.), [])
        self.assertTrue(self.lease.renew('uav_1', (0, 0), 5.))
        self.assertEqual(self.lease.expire(7.), [])
        self.assertEqual(self.lease.expire(9.), [(0, 0)])


if __name__ == '__main__':
    unittest.main()
