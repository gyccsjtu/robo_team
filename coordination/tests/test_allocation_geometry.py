from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'src/robocup_swarm/scripts'))
from allocation_geometry import segment_distance, screen_leg
from swarm_task import CoverageGrid, TaskAllocator, STATE_FREE


class GeometryTests(unittest.TestCase):
    def test_crossing_and_collinear_routes_rejected(self):
        self.assertEqual(segment_distance((-10., 0.), (10., 0.), (0., -10.), (0., 10.)), 0.)
        self.assertFalse(screen_leg((-10., 0.), (10., 0.), [], [((0., -10.), (0., 10.))]))
        self.assertFalse(screen_leg((0., 0.), (10., 0.), [], [((5., 0.), (20., 0.))]))

    def test_parallel_safe_routes_and_degenerate_holds(self):
        self.assertTrue(screen_leg((0., 0.), (10., 0.), [], [((0., 8.), (10., 8.))]))
        self.assertFalse(screen_leg((0., 0.), (10., 0.), [(5., 3.)], []))
        self.assertFalse(screen_leg((0., 0.), (10., 0.), [], [((5., 0.), (5., 0.))]))

    def test_invalid_evidence_rejected(self):
        self.assertFalse(screen_leg((0., 0.), (float('nan'), 0.), [], []))

    def test_auction_rejects_highest_bid_crossing_before_committing_owner(self):
        grid = CoverageGrid(-20., 20., -20., 20., 10., 2)
        allocator = TaskAllocator(grid)
        positions = {'a': (-10., 0.), 'b': (10., 0.)}
        first, crossing, alternative = (3, 3), (0, 3), (3, 0)
        allocator.utility = lambda uid, x, y, key, *args: 100. if key == crossing else 10.
        allocator._priority_bonus = lambda *args: 0.
        def point(key):
            cell = grid.cell(key)
            return cell.cx, cell.cy
        def candidate(uid, key, assigned):
            if key not in ([first] if uid == 'a' else [crossing, alternative]):
                return False
            return screen_leg(positions[uid], point(key),
                [p for owner, p in positions.items() if owner != uid],
                [(positions[owner], point(k)) for owner, k in assigned.items() if k is not None])
        result = allocator.allocate(positions, candidate_filter=candidate)
        self.assertEqual(result, {'a': first, 'b': alternative})
        self.assertEqual(grid.cell(crossing).state, STATE_FREE)
        self.assertIsNone(grid.cell(crossing).owner)
