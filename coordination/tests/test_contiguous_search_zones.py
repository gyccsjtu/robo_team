"""Allocation responsibility is distinct from route or observation evidence."""
import importlib.util
import json
from pathlib import Path
import unittest


def load():
    path = Path(__file__).resolve().parents[1] / 'src/robocup_swarm/scripts/swarm_task.py'
    spec = importlib.util.spec_from_file_location('contiguous_search_subject', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ContiguousSearchTests(unittest.TestCase):
    def setUp(self):
        self.m = load()
        self.grid = self.m.CoverageGrid(-50, 130, -60, 60, 7, 6,
                                       zone_layout='sectors_3x2')

    def test_partition_complete_unique_and_connected_at_actual_bounds(self):
        keys = [k for group in self.grid.zone_cells.values() for k in group]
        self.assertEqual(len(keys), len(set(keys)))
        self.assertEqual(set(keys), set(self.grid.cells))
        for group in self.grid.zone_cells.values():
            remaining = set(group)
            self.assertTrue(remaining)
            pending = [remaining.pop()]
            while pending:
                x, y = pending.pop()
                for neighbor in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        pending.append(neighbor)
            self.assertFalse(remaining)

    def test_six_initial_aircraft_receive_distinct_home_sectors(self):
        positions = {'uav_1': (0,-3), 'uav_2': (3,-3), 'uav_3': (0,0),
                     'uav_4': (3,0), 'uav_5': (0,3), 'uav_6': (3,3)}
        allocator = self.m.TaskAllocator(self.grid)
        result = allocator.allocate(positions)
        self.assertEqual(len(set(result.values())), 6)
        for uid, key in result.items():
            self.assertEqual(self.grid.get_zone_id(key), self.grid.get_uav_zone(uid))
        # This is allocation only; no route, image or completed observation.
        self.assertTrue(all(self.grid.cell(k).state == self.m.STATE_ASSIGNED
                            for k in result.values()))

    def test_rejected_home_candidates_cannot_prevent_admissible_takeover(self):
        outside = self.grid.zone_cells[5][0]
        result = self.m.TaskAllocator(self.grid).allocate(
            {'uav_1': (0,0)}, candidate_filter=lambda uid,key,assigned:key == outside)
        self.assertEqual(result['uav_1'], outside)

    def test_occupied_or_observed_home_cells_are_not_reassigned(self):
        for key in self.grid.zone_cells[0]:
            cell = self.grid.cell(key)
            cell.state = self.m.STATE_COVERED
        occupied = self.grid.zone_cells[1][0]
        self.grid.cell(occupied).state = self.m.STATE_ASSIGNED
        self.grid.cell(occupied).owner = 'uav_2'
        result = self.m.TaskAllocator(self.grid).allocate({'uav_1': (0,0)})
        self.assertNotEqual(result['uav_1'], occupied)
        self.assertNotEqual(self.grid.get_zone_id(result['uav_1']), 0)
        self.assertEqual(self.grid.cell(occupied).owner, 'uav_2')

    def test_no_admissible_candidate_returns_none(self):
        result = self.m.TaskAllocator(self.grid).allocate(
            {'uav_1': (0,0)}, candidate_filter=lambda *args:False)
        self.assertIsNone(result['uav_1'])

    def test_actual_first_auction_positions_pass_existing_departure_screen(self):
        root = Path(__file__).resolve().parents[1]
        fixture = root / 'docs/validation/contiguous_search_candidate_20261007/first_auction_geometry_reconstruction.json'
        data = json.loads(fixture.read_text())
        spec = importlib.util.spec_from_file_location('departure_subject',
            root / 'src/robocup_swarm/scripts/allocation_geometry.py')
        geometry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(geometry)
        positions = data['positions']
        def valid(uid, key, assigned):
            cell = self.grid.cell(key)
            legs = [(positions[u], (self.grid.cell(k).cx,self.grid.cell(k).cy))
                    for u,k in assigned.items() if u != uid and k is not None]
            return geometry.screen_leg(positions[uid], (cell.cx,cell.cy),
                [p for u,p in positions.items() if u != uid], legs,
                separation=data['separation_m'])
        result = self.m.TaskAllocator(self.grid).allocate(positions, candidate_filter=valid)
        for uid, key in result.items():
            self.assertIsNotNone(key)
            self.assertEqual(self.grid.get_zone_id(key), self.grid.get_uav_zone(uid))

    def test_legacy_partition_and_invalid_modes(self):
        grid = self.m.CoverageGrid(0,30,0,20,10,6,zone_layout='interleaved')
        order = sorted(grid.cells, key=lambda k:(k[1],k[0]))
        self.assertEqual([grid.get_zone_id(k) for k in order], list(range(6)))
        with self.assertRaises(ValueError):
            self.m.CoverageGrid(0,30,0,20,10,2,zone_layout='sectors_3x2')
        with self.assertRaises(ValueError):
            self.m.CoverageGrid(0,30,0,20,10,6,zone_layout='unknown')


if __name__ == '__main__':
    unittest.main()
