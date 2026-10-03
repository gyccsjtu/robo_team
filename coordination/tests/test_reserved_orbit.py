"""Exercise actual agent methods without loading ROS or PX4 dependencies."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


source = Path(__file__).parents[1]/'src/robocup_swarm/scripts/swarm_agent.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
methods = [node for cls in tree.body if isinstance(cls, ast.ClassDef)
           for node in cls.body if isinstance(node, ast.FunctionDef)
           and node.name in ('_fly_orbit', '_orbit_stale')]
scope = dict(math=math, rospy=SimpleNamespace(Time=SimpleNamespace(
    now=lambda: SimpleNamespace(to_sec=lambda: 10.))), ORBIT_RADIUS=8., POS_KP=1.)
exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])), str(source), 'exec'), scope)


class ReservedOrbitTests(unittest.TestCase):
    def agent(self):
        a = SimpleNamespace(world_xy=(8., 0.), _orbit_target='t5',
            assignment=SimpleNamespace(target_id='t5'), _t_seen={'t5': 9.9},
            targets={'t5': (0., 0., 0., 0.)}, _online_planner=object(),
            _send_vel=Mock(), _request_plan=Mock(), _pick_local_goal=Mock(return_value=(8., 1.)),
            _apply_friend_avoidance=lambda x, y: (x, y), _publish_claim=Mock(), path_target=None)
        a._orbit_stale = lambda: scope['_orbit_stale'](a)
        return a

    def test_ungranted_orbit_leg_stops_and_requests_actual_planner(self):
        a = self.agent()
        scope['_fly_orbit'](a)
        a._request_plan.assert_called_once()
        a._send_vel.assert_called_once_with(0., 0.)
        a._pick_local_goal.assert_not_called()

    def test_committed_leg_follows_local_path_and_caps_velocity(self):
        a = self.agent()
        point = (8.*math.cos(.2), 8.*math.sin(.2))
        a._reserved_orbit_goal = ((0., 0.), point)
        a.path_target = point
        a._pick_local_goal.return_value = (8., 4.)
        scope['_fly_orbit'](a)
        self.assertEqual(a._send_vel.call_args.args, (0., 1.5))
        a._publish_claim.assert_called_once()

    def test_target_without_orbit_state_still_requires_fresh_camera(self):
        a = self.agent()
        a._orbit_target = None
        a._t_seen = {}
        self.assertTrue(scope['_orbit_stale'](a))
        for stamp in (8.9, 10.1):
            a._t_seen = {'t5': stamp}
            self.assertTrue(scope['_orbit_stale'](a))
        a._t_seen = {'t5': 9.9}
        self.assertFalse(scope['_orbit_stale'](a))
