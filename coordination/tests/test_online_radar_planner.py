from pathlib import Path
import sys
import unittest
import tempfile
import json
import hashlib

root = Path(__file__).parents[1]
sys.path.insert(0, str(root / 'src/robocup_swarm/scripts'))
sys.path.insert(0, str(root / 'src/robocup_navigation/src'))
from online_radar_planner import OnlinePlanner, fixture_seed
from radar_observed_map import ObservedMap


class OnlinePlanningTests(unittest.TestCase):
    def observed(self):
        observed = ObservedMap(40, 40, .5, (-10., -10.))
        for iy in range(40):
            for ix in range(40):
                x, y = -10+(ix+.5)*.5, -10+(iy+.5)*.5
                if abs(x) < 5 and abs(y) < 5:
                    index = iy*40+ix
                    observed.cells[index] = 0
                    observed.observed_s[index] = 10.
        observed.last_scan_s = 10.
        return observed

    def test_unknown_goal_uses_only_connected_observed_frontier(self):
        planner = OnlinePlanner((0.,0.))
        grid = planner.grid(self.observed(), (0.,0.), 10., 'epoch')
        result = planner.route(grid, (0.,0.), (9.,9.))
        self.assertTrue(result['ok'])
        self.assertEqual(result['reason'], 'OBSERVED_FRONTIER')
        self.assertNotEqual(result['points'][-1], (9.,9.))
        self.assertTrue(all(grid.is_free(grid.world_to_cell(p)) for p in result['points']))

    def test_goal_in_observed_clearance_is_reached_exactly(self):
        planner = OnlinePlanner((0.,0.))
        grid = planner.grid(self.observed(), (0.,0.), 10., 'epoch')
        result = planner.route(grid, (0.,0.), (2.1,1.1))
        self.assertEqual(result['reason'], 'GOAL_OBSERVED')
        self.assertEqual(result['points'][-1], (2.1,1.1))

    def test_expired_observations_cannot_create_a_route(self):
        planner = OnlinePlanner((0.,0.))
        grid = planner.grid(self.observed(), (0.,0.), 14., 'epoch')
        self.assertFalse(planner.route(grid, (0.,0.), (9.,9.))['ok'])

    def test_final_velocity_cannot_cross_unknown_or_bounds(self):
        planner = OnlinePlanner((0.,0.))
        grid = planner.grid(self.observed(), (0.,0.), 10., 'epoch')
        self.assertTrue(planner.command_clear(grid, (0.,0.), (1.,0.)))
        self.assertFalse(planner.command_clear(grid, (3.,0.), (2.,0.)))
        self.assertFalse(planner.command_clear(grid, (9.5,0.), (1.,0.)))

    def test_fixture_clearance_rejects_near_box_hash_change_and_wrong_run(self):
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory)/'fixture.world'
            world.write_text('<sdf><world><include><uri>model://ground_plane</uri></include><include><uri>model://sun</uri></include><model name="box"><static>true</static><pose>0 -9 5 0 0 0</pose><link name="box"><collision><geometry><box><size>3 3 10</size></box></geometry></collision></link></model></world></sdf>')
            certificate = dict(schema_version=2, run_id='run', world_path=str(world),
                world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(), positions={'a': [0., 0.], 'b': [0., -8.]})
            path = Path(directory)/'certificate.json'
            path.write_text(json.dumps(certificate))
            self.assertEqual(fixture_seed(path, 'run', 'a'), (0., 0.))
            with self.assertRaises(ValueError):
                fixture_seed(path, 'run', 'b')
            with self.assertRaises(ValueError):
                fixture_seed(path, 'old', 'a')
            world.write_text(world.read_text().replace('-9', '-1'))
            with self.assertRaises(ValueError):
                fixture_seed(path, 'run', 'a')
