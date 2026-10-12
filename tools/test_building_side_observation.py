"""Aircraft clearance margins must not be mistaken for camera occluders."""
import threading
import unittest
from unittest.mock import Mock, patch

import test_search_plan
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


if __name__ == '__main__':
    unittest.main()
