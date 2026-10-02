from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/robocup_swarm/scripts'))
sys.path.insert(0, str(ROOT / 'src/robocup_navigation/src'))
from route_endpoint import connect_exact_goal
from robocup_navigation.astar import GridMap


class EndpointTests(unittest.TestCase):
    def test_free_cell_centre_connects_to_actual_point(self):
        grid = GridMap(4, 4, .5, (0., 0.), [0] * 16)
        self.assertEqual(connect_exact_goal([(0.25, .25), (1.25, 1.25)], (1.01, 1.02), grid)[-1], (1.01, 1.02))

    def test_occupied_or_different_cell_does_not_invent_safe_connector(self):
        grid = GridMap(4, 4, .5, (0., 0.), [0] * 16)
        path = [(.25, .25)]
        self.assertEqual(connect_exact_goal(path, (.75, .25), grid), path)
        grid.cells[0] = 1
        self.assertEqual(connect_exact_goal(path, (.1, .1), grid), path)

    def test_out_of_bounds_or_invalid_goal_is_not_appended(self):
        grid = GridMap(4, 4, .5, (0., 0.), [0] * 16)
        for goal in ((-1., 0.), (float('nan'), .1)):
            self.assertEqual(connect_exact_goal([(.25, .25)], goal, grid), [(.25, .25)])


if __name__ == '__main__':
    unittest.main()
