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

    def test_scan_carries_observed_footprint_between_infrequent_planning_calls(self):
        observed = self.observed()
        planner = OnlinePlanner((0.,0.))
        old = OnlinePlanner((0.,0.))
        planner.grid(observed, (0.,0.), 10., 'epoch')
        old.grid(observed, (0.,0.), 10., 'epoch')
        # Traversed free cells enter the blind region while no route replan runs.
        for stamp, position in ((11.,(1.,0.)), (12.,(2.,0.)), (13.,(3.,0.))):
            observed.last_scan_s = stamp
            planner.carry_body_proof(observed, position, stamp, 'epoch')
        observed.observed_s = [14. if v == 0 else None for v in observed.cells]
        observed.last_scan_s = 14.
        hole = observed.index(3.,0.)
        observed.cells[hole], observed.observed_s[hole] = -1, None
        position = (3.,0.)
        carried = planner.grid(observed, position, 14., 'epoch')
        missed = old.grid(observed, position, 14., 'epoch')
        self.assertTrue(carried.is_free(carried.world_to_cell(position)))
        self.assertFalse(missed.is_free(missed.world_to_cell(position)))
        # A fresh physical obstacle revokes the retained cell immediately.
        observed.cells[hole], observed.observed_s[hole] = 100, 14.1
        observed.last_scan_s = 14.1
        planner.carry_body_proof(observed, position, 14.1, 'epoch')
        hit = planner.grid(observed, position, 14.1, 'epoch')
        self.assertFalse(hit.is_free(hit.world_to_cell(position)))

    def test_body_carry_never_fills_unseen_cells_or_crosses_map_reset(self):
        observed = self.observed()
        planner = OnlinePlanner((0.,0.))
        planner.grid(observed, (0.,0.), 10., 'epoch')
        unseen = observed.index(8.,0.)
        planner.carry_body_proof(observed, (8.,0.), 10., 'epoch')
        self.assertNotIn(unseen, planner.body_proof)
        observed.cells = [-1] * (observed.width*observed.height)
        observed.observed_s = [None] * len(observed.cells)
        observed.last_scan_s = 11.
        planner.carry_body_proof(observed, (8.,0.), 11., 'new_epoch')
        self.assertFalse(planner.body_proof)

    def test_frozen_plan_cannot_roll_back_newer_footprint_proof(self):
        observed = self.observed()
        planner = OnlinePlanner((0.,0.))
        planner.grid(observed, (0.,0.), 10., 'epoch')
        observed.last_scan_s = 11.
        planner.carry_body_proof(observed, (1.,0.), 11., 'epoch')
        latest = set(planner.body_proof)
        frozen = self.observed()  # Older scan, and old aircraft position.
        planner.grid(frozen, (0.,0.), 10., 'epoch')
        self.assertEqual(planner.body_proof, latest)

    def test_fine_grid_retains_radius_but_avoids_diagonal_quantization_stall(self):
        import math
        def scanned_grid(resolution, distance):
            count = int(20 / resolution)
            observed = ObservedMap(count, count, resolution,
                                   (-10-resolution/2, -10-resolution/2))
            ranges = [math.inf] * 512
            ranges[320] = distance  # Southwest wall point, behind escape direction.
            self.assertTrue(observed.feed(ranges, (0.,0.), 0., 0., 2*math.pi/512,
                                          .5, 20., 10., 10., 10.))
            # Only the near blind region is certified; outside it comes from rays.
            planner = OnlinePlanner((0.,0.), seed_radius=.55, radius=1.2)
            return planner, planner.grid(observed, (0.,0.), 10., 'epoch')
        coarse, coarse_grid = scanned_grid(.5, 1.58)
        fine, fine_grid = scanned_grid(.25, 1.58)
        self.assertEqual(coarse.radius, fine.radius)
        self.assertFalse(coarse_grid.is_free(coarse_grid.world_to_cell((0.,0.))))
        self.assertTrue(fine_grid.is_free(fine_grid.world_to_cell((0.,0.))))
        self.assertTrue(fine.command_clear(fine_grid, (0.,0.), (.25,.25)))
        _, close_grid = scanned_grid(.25, 1.1)
        self.assertFalse(close_grid.is_free(close_grid.world_to_cell((0.,0.))))

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
