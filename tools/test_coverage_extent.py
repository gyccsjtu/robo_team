"""Boundary partial cells and shared manager/agent coverage contract."""
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'coordination/src/robocup_swarm/scripts'))
from swarm_task import CoverageGrid,coverage_bounds
from search_observation import cell_points
import swarm_manager as sm
import swarm_agent as sa

class CoverageExtentTests(unittest.TestCase):
    def test_map_margin_does_not_create_outside_search_jobs(self):
        bounds=coverage_bounds((-55.,155.,-65.,65.),sa.flight_bounds())
        g=CoverageGrid(*bounds,cell_m=7.)
        self.assertEqual(bounds,(-50.,150.,-60.,60.))
        self.assertEqual(len(g.cells),522)
        for key,c in g.cells.items():
            self.assertTrue(-50<c.cx<150 and -60<c.cy<60)
            self.assertEqual(g.world_to_cell(c.cx,c.cy),key)
            for x,y in cell_points(g,c):
                self.assertEqual(g.world_to_cell(x,y),key)
        self.assertAlmostEqual(g.cells[(28,17)].cx,148.)
        self.assertAlmostEqual(g.cells[(28,17)].cy,59.5)

    def test_ceil_storage_does_not_accept_outside_world_coordinates(self):
        g=CoverageGrid(-55.,155.,-65.,65.,7.)
        for xy in ((0,65),(0,67),(155,0),(-56,0),(float('nan'),0),(0,float('inf'))):
            self.assertIsNone(g.world_to_cell(*xy))
        self.assertIsNotNone(g.world_to_cell(0,64.999))
        for c in g.cells.values():
            self.assertLess(c.cy,65.)

    def test_manager_uses_same_intersection_as_agent(self):
        path=Path(__file__).resolve().parents[1]/'coordination/src/robocup_training_worlds/worlds/generated/robocup_base.json'
        with patch.object(sm,'METADATA_PATH',str(path)),patch.object(sm,'MAP_BOUNDS_FROM_META',True):
            self.assertEqual(sm._search_bounds(),sa.flight_bounds())

    def test_invalid_or_disjoint_extents_fail_instead_of_assigning_outside(self):
        for b in ((5,4,0,1),(0,float('nan'),0,1),(0,1,0,0)):
            with self.assertRaises(ValueError):CoverageGrid(*b)
        with self.assertRaises(ValueError):coverage_bounds((0,1,0,1),(2,3,0,1))
        with self.assertRaises(ValueError):coverage_bounds((0,1,0,1),(0,float('nan'),0,1))

if __name__=='__main__':unittest.main()
