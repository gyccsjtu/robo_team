"""Regress the actual city/actor-envelope mix-up and outside A* detours."""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'coordination/src/robocup_swarm/scripts'))
from flight_boundary import (FlightGrid, flight_bounds, inside_flight,
                             boundary_velocity, validate_environment)
from robocup_navigation.astar import GridMap, plan
from swarm_task import CoverageGrid, STATE_FREE
from slam_grid import SlamGrid


class CityBoundaryTests(unittest.TestCase):
    def test_old_actor_limits_cannot_be_restored_through_environment(self):
        for env in ({'FLIGHT_X_MAX': '150'}, {'FLIGHT_Y_MAX': '60'},
                    {'FLIGHT_Y_MIN': '-60'}, {'FLIGHT_INSET': 'nan'}):
            with self.assertRaises(ValueError):
                validate_environment(env)
        validate_environment({'FLIGHT_X_MAX': '130'})

    def test_partial_rows_have_interior_centers_and_no_padding_cells(self):
        g = CoverageGrid(*flight_bounds(), cell_m=7, num_uavs=6)
        self.assertEqual(len(g.cells), 390)
        for key, cell in g.cells.items():
            self.assertTrue(inside_flight(cell.cx, cell.cy))
            self.assertEqual(g.world_to_cell(cell.cx, cell.cy), key)
            self.assertEqual(cell.state, STATE_FREE)
        for point in ((130.5, 0), (0, 51.5), (0, 57.5), (-51.5, 0)):
            self.assertIsNone(g.world_to_cell(*point))
        self.assertEqual(set().union(*map(set, g.zone_cells.values())), set(g.cells))

    def test_path_cannot_bypass_wall_through_storage_padding(self):
        cells = bytearray(210*130)
        for y in range(-50, 50):
            cells[(y+65)*210+55] = 1  # synthetic wall, no simulator truth
        raw = GridMap(210, 130, 1., (-55., -65.), cells)
        self.assertTrue(plan(raw, (-10., 45.), (10., 45.)).success)
        masked = FlightGrid(raw)
        self.assertFalse(plan(masked, (-10., 45.), (10., 45.)).success)
        # Open an interior passage: routing must work without leaving bounds.
        for y in range(-5, 5):
            cells[(y+65)*210+55] = 0
        route = plan(masked, (-10., 45.), (10., 45.))
        self.assertTrue(route.success)
        self.assertTrue(all(inside_flight(*p, inset=2.) for p in route.points))

    def test_navigation_mask_does_not_fabricate_sensor_obstacles(self):
        raw = SlamGrid(-55, -65, 155, 65, .5, 2.)
        before = raw.to_rle()
        mask = FlightGrid(raw)
        cell = raw.world_to_cell((0., 54.57))  # observed UAV3 excursion
        self.assertTrue(raw.is_free(cell))
        self.assertFalse(mask.is_free(cell))
        self.assertEqual(raw.to_rle(), before)

    def test_braking_budget_and_tangential_motion_on_all_four_edges(self):
        for room in (0., .1, .5, 2., 5., 10., 20.):
            for x, y, vx, vy, axis in (
                    (128-room, 0., 6., 1.5, 0),
                    (-48+room, 0., -6., 1.5, 0),
                    (0., 48-room, 1.5, 6., 1),
                    (0., -48+room, 1.5, -6., 1)):
                out = boundary_velocity(x, y, vx, vy)
                speed = abs(out[axis])
                self.assertLessEqual(speed*.6 + speed*speed/2.4, room+1e-8)
                self.assertEqual(out[1-axis], 1.5)
                self.assertLessEqual(math.hypot(*out), math.hypot(vx, vy))
        self.assertEqual(boundary_velocity(0., 54.57, 2., 1.), (2., 0.))
        self.assertEqual(boundary_velocity(0., 54.57, 2., -1.), (2., -1.))


if __name__ == '__main__':
    unittest.main()
