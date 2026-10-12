"""Eastern entry precedes a long corner trip; west review stays a local task."""
import unittest
from unittest.mock import patch
import test_east_split_patrol as east_fixture
import test_west_corner_review as west_fixture
import swarm_manager as sm


class EastEntryTests(unittest.TestCase):
    setUp = east_fixture.EastSplitPatrolTests.setUp
    refresh = east_fixture.EastSplitPatrolTests.refresh
    allocate = east_fixture.EastSplitPatrolTests.allocate
    def test_first_east_legs_enter_near_the_centre_then_corner_priority_returns(self):
        with patch.dict('os.environ',{'EAST_FIRST_ENTRY':'1'}):
            tasks=self.allocate()
        self.assertEqual(len(tasks),2)
        for task in tasks:
            self.assertGreaterEqual(task.points[0][0],115.)
            self.assertLessEqual(abs(task.points[0][1]),25.)
            self.assertEqual(task.purpose,'south_patrol')
            self.assertTrue(self.m._search_point_free(task.points[0]))
            self.assertNotEqual(self.m.grid.cell(task.cells[0]).covered_t,100.)

    def test_unreachable_central_entry_still_allows_checked_outer_fallback(self):
        east_fixture.EastSplitPatrolTests.test_outer_northeast_and_southeast_cells_have_safe_inward_viewpoints(self)


class LocalWestReviewTests(unittest.TestCase):
    setUp = west_fixture.WestCornerReviewTests.setUp
    refresh = west_fixture.WestCornerReviewTests.refresh
    allocate = west_fixture.WestCornerReviewTests.allocate
    corner_tasks = west_fixture.WestCornerReviewTests.corner_tasks
    uid = west_fixture.WestCornerReviewTests.uid
    def test_eastern_idle_aircraft_is_not_sent_to_a_distant_west_corner(self):
        for uid in self.m.uav_ids:
            self.m.status[uid].x,self.m.status[uid].y=98.4377,-34.9369
        self.m._allocate_west_corner_review(list(self.m.uav_ids),self.now)
        self.assertEqual(self.corner_tasks(),[])
        self.assertEqual(self.m._west_corner_seen,{})

    def test_checked_route_budget_preserves_nearby_west_coverage(self):
        self.allocate()
        task=self.corner_tasks()[0]
        origin=self.m.status[task.uid]
        route=sm.plan(self.m._g_infl,(origin.x,origin.y),task.points[0],
                      connectivity=8,prevent_corner_cutting=True,max_expansions=2500)
        self.assertTrue(route.success)
        self.assertLessEqual(route.path_length_m,60.)


if __name__=='__main__':unittest.main()
