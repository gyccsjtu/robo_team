import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
CORE=Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'
sys.path[:0]=[str(SCRIPTS),str(CORE)]
from prepare_competition_scene import startup_distances, SPAWNS
from online_radar_planner import fixture_seed, OnlinePlanner
from allocation_geometry import screen_leg
from swarm_task import CoverageGrid
from radar_observed_map import ObservedMap


class CityStartupTests(unittest.TestCase):
    def test_partial_edge_cells_stay_inside_city(self):
        grid=CoverageGrid(-50,130,-60,60,7,6)
        self.assertTrue(all(-50<c.cx<130 and -60<c.cy<60 for c in grid.cells.values()))
        self.assertEqual(grid.cell((0,17)).cy,59.5)

    def test_certified_start_survives_diagonal_inflation(self):
        observed=ObservedMap(20,20,.5,(-5.25,-5.25))
        grid=OnlinePlanner((0.,0.)).grid(observed,(0.,0.),1.,1)
        self.assertTrue(grid.is_free(grid.world_to_cell((0.,0.))))
        self.assertFalse(grid.is_free(grid.world_to_cell((3.,0.))))
    def test_house_contains_platform_start(self):
        d=startup_distances([[[0.,30.],[1.,25.]]])
        self.assertEqual(d[4],0.)
        self.assertEqual(d[5],0.)

    def test_near_wall_is_not_clear_start(self):
        self.assertLess(startup_distances([[[1.,2.],[-4.,-2.]]])[0],2.)

    def test_three_meter_formation_can_leave_outward(self):
        self.assertFalse(screen_leg(SPAWNS[0],(-10.,-3.),SPAWNS[1:],[]))
        self.assertTrue(screen_leg(SPAWNS[0],(-10.,-3.),SPAWNS[1:],[],separation=2.5))
        self.assertFalse(screen_leg(SPAWNS[0],(10.,-3.),SPAWNS[1:],[],separation=2.5))

    def test_city_initialization_bound_to_world_and_run(self):
        with tempfile.TemporaryDirectory() as directory:
            world=Path(directory)/'city.world'
            world.write_text('<sdf><world name="default"><actor name="actor_0"/></world></sdf>')
            path=Path(directory)/'startup.json'
            path.write_text(json.dumps(dict(schema_version=3,purpose='DEVELOPMENT_CITY_STARTUP',
                run_id='city-run',world_path=str(world),world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
                positions={'uav_1':[0.,-3.]},startup_checks={'uav_1':dict(free_radius_m=2.)})))
            self.assertEqual(fixture_seed(path,'city-run','uav_1'),(0.,-3.))
            with self.assertRaises(ValueError):
                fixture_seed(path,'old-run','uav_1')
            world.write_text('<sdf><world name="changed"/></sdf>')
            with self.assertRaises(ValueError):
                fixture_seed(path,'city-run','uav_1')
