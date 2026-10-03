import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


class PlanarAltitudeTests(unittest.TestCase):
    def method(self, name='_compute_desired_altitude'):
        path = Path(__file__).parents[1]/'src/robocup_swarm/scripts/swarm_agent.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        method = next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)
                      and n.name == name)
        namespace = dict(rospy=SimpleNamespace(logwarn=lambda *a: None))
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),namespace)
        return namespace[method.name]

    def test_tracking_cannot_lower_flight_plane_without_vertical_evidence(self):
        node = SimpleNamespace(_online_planner=object(),_orbit_target='t0',world_xy=(0.,0.))
        self.assertEqual(self.method()(node),0.)

    def test_terminal_assignment_cannot_switch_fcu_or_start_descent(self):
        for typ in (2,3):
            node = SimpleNamespace(uav_id='uav_1',_online_planner=object(),_landing=True,
                mode_srv=SimpleNamespace(call=lambda *a: self.fail('Unproved RTL mode switch')))
            self.method('_assign_cb')(node,SimpleNamespace(uav_id='uav_1',task_type=typ))
            self.assertFalse(node._landing)
            sent = []
            node._gate = SimpleNamespace(task={'task_type':typ})
            node._send_vel = lambda *a: sent.append(a)
            self.method('_control')(node)
            self.assertEqual(sent,[(0.,0.)])

    def test_near_obstacle_cannot_authorize_descent(self):
        node = SimpleNamespace(_online_planner=object(),_orbit_target=None,world_xy=(0.,0.),
            los=object(),grid=SimpleNamespace(world_to_cell=lambda _: (1,1),is_free=lambda _: False))
        self.assertEqual(self.method()(node),0.)
