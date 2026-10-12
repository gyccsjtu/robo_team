"""Aircraft clearance margins must not be mistaken for camera occluders."""
import threading
import unittest
from unittest.mock import Mock, patch

import test_search_plan
import test_east_split_patrol as patrol_fixture
import boundary_patrol as bp
import swarm_manager as sm
import swarm_task as st


class BuildingSideObservationTests(unittest.TestCase):
    def fixture(self):
        manager = sm.SwarmManager.__new__(sm.SwarmManager)
        manager.grid = st.CoverageGrid(0, 35, 0, 35, cell_m=7, num_uavs=1)
        key = manager.grid.world_to_cell(17.5, 17.5)
        cells = bytearray(240 * 240)
        raw = sm.GridMap(240, 240, .25, (-10., -10.), cells)
        # Sensed building wall; the observed ground is 1.5m outside it.
        for index in range(40):
            xy = raw.world_to_cell((10. + index * .25, 16.))
            cells[xy[1] * raw.width + xy[0]] = 1
        raw = sm.GridMap(raw.width, raw.height, raw.resolution, raw.origin, cells)
        manager._g_infl = sm.inflate_grid(raw, 2.)
        manager._slam_lock = threading.RLock()
        manager._merged_grid = Mock(return_value=raw)
        manager._cell_waypoint = {key: (17.5, 21.)}
        manager._blind_viewpoints = {}
        manager._camera_point_seen = {}
        return manager, key, raw

    def test_free_ground_inside_flight_margin_is_visible_from_safe_side(self):
        manager, key, raw = self.fixture()
        cell = manager.grid.cell(key)
        self.assertTrue(raw.is_free(raw.world_to_cell((cell.cx, cell.cy))))
        self.assertFalse(manager._g_infl.is_free(
            manager._g_infl.world_to_cell((cell.cx, cell.cy))))
        self.assertTrue(manager._blind_viewpoint_valid(cell, (17.5, 26.)))
        self.assertEqual(manager.grid.cell(key).state, st.STATE_FREE)
        self.assertEqual(manager._camera_point_seen, {})

    def test_physical_wall_still_occludes_the_opposite_side(self):
        manager, key, _ = self.fixture()
        self.assertFalse(manager._blind_viewpoint_valid(
            manager.grid.cell(key), (17.5, 9.)))

    def test_camera_visibility_does_not_grant_aircraft_clearance(self):
        manager, key, _ = self.fixture()
        self.assertFalse(manager._blind_viewpoint_valid(
            manager.grid.cell(key), (9., 17.5)))

    def test_building_alternative_waypoint_does_not_disable_blind_recovery(self):
        manager, key, raw = self.fixture()
        with patch.object(sm.rospy.Time, 'now', return_value=sm.rospy.Time.from_sec(100.)):
            point = manager._choose_blind_viewpoint(key, manager.grid.cell(key), (17.5, 24.))
        self.assertIsNotNone(point)
        self.assertTrue(manager._blind_viewpoint_valid(manager.grid.cell(key), point))
        self.assertTrue(sm.plan(manager._g_infl, (17.5, 24.), point).success)
        self.assertEqual(manager.grid.cell(key).state, st.STATE_FREE)
        self.assertEqual(manager._camera_point_seen, {})

    def test_missing_raw_map_keeps_conservative_camera_check(self):
        manager, key, _ = self.fixture()
        manager._merged_grid.return_value = None
        self.assertFalse(manager._blind_viewpoint_valid(
            manager.grid.cell(key), (17.5, 26.)))


class PatrolBuildingSideTests(unittest.TestCase):
    use_default_flight_bounds = True
    refresh = patrol_fixture.EastSplitPatrolTests.refresh

    def setUp(self):
        patrol_fixture.DefaultEastSplitPatrolTests.setUp(self)
        self.key = self.m.grid.world_to_cell(114.5, 20.5)
        self.m._blocked_cells = set(self.m.grid.cells)-{self.key}
        self.uid = self.m.uav_ids[0]
        self.m.status[self.uid].x, self.m.status[self.uid].y = 100., 20.5
        cells = bytearray(self.m._g_infl.width*self.m._g_infl.height)
        base = self.m._g_infl
        self.raw = sm.GridMap(base.width, base.height, base.resolution, base.origin, cells)
        for y in range(15, 31):
            ix, iy = self.raw.world_to_cell((116., float(y)))
            cells[iy*base.width+ix] = 1
        self.raw = sm.GridMap(base.width, base.height, base.resolution, base.origin, cells)
        self.m._g_infl = sm.inflate_grid(self.raw, 2.)
        self.m._slam_lock = threading.RLock()
        self.m._merged_grid = Mock(return_value=self.raw)

    def test_patrol_looks_at_wall_foot_without_flying_through_its_margin(self):
        patrol = self.m._regional_patrols[0]
        cell = self.m.grid.cell(self.key)
        point = (cell.cx-8.95, cell.cy)
        self.assertFalse(bp.clear_line(self.m._g_infl, point, (cell.cx, cell.cy)))
        self.assertTrue(self.m._blind_viewpoint_valid(cell, point))
        proposal = patrol.choose(self.m, [self.uid], self.now)
        self.assertIsNotNone(proposal)
        self.assertEqual(proposal[1], [self.key])
        self.assertEqual(proposal[2], [point])
        route = sm.plan(self.m._g_infl, (100.,20.5), point)
        self.assertTrue(route.success)
        self.assertTrue(all(bp.clear_line(self.m._g_infl, a,b)
                            for a,b in zip(route.points,route.points[1:])))
        self.assertEqual(cell.state, st.STATE_FREE)

    def test_patrol_still_rejects_an_actual_wall_in_its_view(self):
        base = self.raw
        cells = bytearray(base.cells)
        ix, iy = base.world_to_cell((110., 20.5))
        cells[iy*base.width+ix] = 1
        self.m._merged_grid.return_value = sm.GridMap(
            base.width,base.height,base.resolution,base.origin,cells)
        self.assertIsNone(self.m._regional_patrols[0].choose(self.m,[self.uid],self.now))


if __name__ == '__main__':
    unittest.main()
