"""Platform escape inputs: real world coordinates and rotated actual velocity."""
import ast
import importlib.util
import math
from pathlib import Path
import types
import unittest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('actor_environment_bridge',
    ROOT/'coordination/scripts/actor_environment_bridge.py')
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)
NS = types.SimpleNamespace


def vector(x=0., y=0., z=0.):
    return NS(x=x, y=y, z=z)


def pose(x=0., y=0., z=0., yaw=0.):
    return NS(position=vector(x, y, z),
              orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))


def twist(x=0., y=0., z=0.):
    return NS(linear=vector(x, y, z), angular=vector())


def odometry():
    return NS(header=NS(frame_id='', stamp=None), child_frame_id='',
              pose=NS(pose=pose()), twist=NS(twist=twist()))


class ActorEnvironmentTests(unittest.TestCase):
    def test_actual_world_position_is_not_local_spawn_position(self):
        original = pose(51., -23., 3.)
        result = bridge.world_odometry(original, twist(2.), 100., 'typhoon_h480_0', odometry)
        self.assertEqual((result.pose.pose.position.x, result.pose.pose.position.y), (51., -23.))
        self.assertEqual(result.header.frame_id, 'world')
        self.assertEqual(result.child_frame_id, 'typhoon_h480_0')
        result.pose.pose.position.x = 0.
        self.assertEqual(original.position.x, 51.)

    def test_world_velocity_rotated_to_body_preserves_escape_speed(self):
        result = bridge.world_odometry(pose(yaw=math.pi/2), twist(0., 2., .5),
                                       100., 'typhoon_h480_0', odometry)
        self.assertAlmostEqual(result.twist.twist.linear.x, 2.)
        self.assertAlmostEqual(result.twist.twist.linear.y, 0.)
        self.assertAlmostEqual(result.twist.twist.linear.z, .5)

    def test_missing_aircraft_never_reuses_cached_position(self):
        message = NS(name=['actor_0'], pose=[pose()], twist=[twist()])
        self.assertEqual(bridge.fleet_odometry(message, 100., ['typhoon_h480_0'], odometry), {})

    def test_name_mapping_does_not_depend_on_gazebo_order(self):
        names = ['typhoon_h480_%d' % i for i in reversed(range(6))]
        message = NS(name=names, pose=[pose(float(i)) for i in reversed(range(6))],
                     twist=[twist(float(i)) for i in reversed(range(6))])
        output = bridge.fleet_odometry(message, 100., names, odometry)
        for i in range(6):
            self.assertEqual(output['typhoon_h480_%d' % i].pose.pose.position.x, i)

    def test_duplicate_names_and_mismatched_arrays_rejected(self):
        for message in (NS(name=['x', 'x'], pose=[pose(), pose()], twist=[twist(), twist()]),
                        NS(name=['x'], pose=[], twist=[twist()])):
            with self.assertRaises(ValueError):
                bridge.fleet_odometry(message, 100., ['x'], odometry)

    def test_invalid_numeric_input_rejected(self):
        for p, t, stamp in ((pose(float('nan')), twist(), 100.),
                            (pose(), twist(float('inf')), 100.),
                            (pose(), twist(), 0.)):
            with self.assertRaises(ValueError):
                bridge.world_odometry(p, t, stamp, 'typhoon_h480_0', odometry)
        p = pose()
        p.orientation.w = 0.
        with self.assertRaises(ValueError):
            bridge.world_odometry(p, twist(), 100., 'typhoon_h480_0', odometry)

    def test_fleet_mapping_matches_official_zero_based_actor_names(self):
        wiring = dict(uavs=[dict(uav_id='uav_%d' % (i+1), model_name='typhoon_h480_%d' % i)
                            for i in range(6)])
        self.assertEqual(len(bridge.fleet_mapping(wiring)), 6)
        wiring['uavs'][0]['model_name'] = 'typhoon_h480_1'
        with self.assertRaises(ValueError):
            bridge.fleet_mapping(wiring)

    def test_actual_official_escape_callback_with_bridge_output(self):
        source = ROOT/'coordination/vendor/official_robocup/20261006/control_actor.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))
        callback = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                        and n.name == 'uav_odom_callback')
        scope = {'math': math}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[callback], type_ignores=[])),
                     str(source), 'exec'), scope)
        for reported, distance, speed, expected in (
                (True, 19., 2., True), (False, 19., 2., False),
                (True, 21., 2., False), (True, 19., 1., False),
                (True, 20., 2., True), (True, 19., 0., False)):
            with self.subTest(reported=reported, distance=distance, speed=speed):
                actor = NS(id='0', reported=reported, escape_triggered=False,
                    escape_active=False, actor_pose_ready=True, uav_odom={},
                    current_pose=vector(50., -20.), escape_distance=20.,
                    escape_trigger_speed=1., escape_requested=False)
                output = bridge.world_odometry(pose(50.+distance, -20., yaw=math.pi/2),
                    twist(speed), 100., 'typhoon_h480_0', odometry)
                scope['uav_odom_callback'](actor, output, 0)
                self.assertEqual(actor.escape_requested, expected)
                self.assertAlmostEqual(actor.uav_odom[0][2], speed)


if __name__ == '__main__':
    unittest.main()
