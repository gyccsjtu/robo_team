"""Exercise the actual agent guide selection without a ROS runtime."""
import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).parents[1]
SCRIPTS = ROOT / 'src/robocup_swarm/scripts'
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(ROOT / 'src/robocup_navigation/src'))
from online_radar_planner import OnlinePlanner
from robocup_navigation.astar import GridMap

tree = ast.parse((SCRIPTS / 'swarm_agent.py').read_text(encoding='utf-8'))
method = next(n for cls in tree.body if isinstance(cls, ast.ClassDef)
              for n in cls.body if isinstance(n, ast.FunctionDef)
              and n.name == '_pick_local_goal')
namespace = dict(math=math, LOOKAHEAD=3.)
exec(compile(ast.Module(body=[method], type_ignores=[]), 'swarm_agent.py', 'exec'), namespace)
pick = namespace['_pick_local_goal']


class CornerFollowingTests(unittest.TestCase):
    def grid(self, free):
        return GridMap(5, 5, 1., (0., 0.),
                       [0 if (x, y) in free else 100 for y in range(5) for x in range(5)])

    def agent(self, grid, path):
        return SimpleNamespace(path=path, world_xy=(.5, .5),
                               _online_planner=OnlinePlanner((.5, .5)), _online_safe_grid=grid)

    def test_l_corridor_guides_to_turn_instead_of_cutting_wall(self):
        grid = self.grid({(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)})
        path = [(.5, .5), (1.5, .5), (2.5, .5), (2.5, 1.5), (2.5, 2.5)]
        self.assertFalse(OnlinePlanner.connector_clear(grid, path[0], (2.5, 1.5)))
        self.assertEqual(pick(self.agent(grid, path)), (2.5, .5))

    def test_straight_known_corridor_preserves_three_meter_lookahead(self):
        grid = self.grid({(x, 0) for x in range(5)})
        path = [(x+.5, .5) for x in range(5)]
        self.assertEqual(pick(self.agent(grid, path)), (3.5, .5))

    def test_no_known_connector_has_no_motion_goal(self):
        grid = self.grid({(0, 0), (2, 0)})
        self.assertIsNone(pick(self.agent(grid, [(.5, .5), (2.5, .5)])))

    def test_corner_touch_checks_both_adjacent_cells(self):
        blocked = self.grid({(0, 0), (1, 1), (0, 1)})
        clear = self.grid({(0, 0), (1, 1), (0, 1), (1, 0)})
        self.assertFalse(OnlinePlanner.connector_clear(blocked, (.5, .5), (1.5, 1.5)))
        self.assertTrue(OnlinePlanner.connector_clear(clear, (.5, .5), (1.5, 1.5)))

    def test_legacy_planner_keeps_original_guide(self):
        agent = self.agent(None, [(.5, .5), (2.5, .5), (2.5, 2.5)])
        agent._online_planner = None
        self.assertEqual(pick(agent), (2.5, 1.5))


if __name__ == '__main__':
    unittest.main()
