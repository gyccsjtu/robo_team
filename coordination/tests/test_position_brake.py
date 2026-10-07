import ast
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
sys.path.insert(0, str(Path(__file__).parents[1]/'src/robocup_swarm/scripts'))
from position_brake import PositionBrake


class PositionBrakeTests(unittest.TestCase):
    def test_moving_xy_can_hold_z_without_ignoring_horizontal_velocity(self):
        gate = PositionBrake()
        output = gate.encode((.3,0.,0.),.2,(2.,3.),'frame',moving_altitude=2.4)
        self.assertEqual(output['type_mask'],1507)
        self.assertEqual(output['position_z'],2.4)
        self.assertEqual(output['velocity_xyz'],(.3,0.,0.))
        self.assertEqual(output['yaw_rate'],.2)
        mask = output['type_mask']
        self.assertTrue(mask & 1 and mask & 2)  # Ignore PX/PY.
        self.assertFalse(mask & 4)  # Use PZ.
        self.assertFalse(mask & 8 or mask & 16)  # Use VX/VY.
        self.assertTrue(mask & 32)  # Ignore VZ.

    def test_vertical_emergency_and_missing_pose_override_moving_z_hold(self):
        gate = PositionBrake()
        for vz in (-1.,.8):
            output = gate.encode((.3,0.,vz),0.,(2.,3.),'frame',moving_altitude=2.4)
            self.assertEqual(output['type_mask'],1479)
            self.assertEqual(output['velocity_xyz'][2],vz)
        self.assertEqual(gate.encode((.3,0.,0.),0.,None,'frame',
                                    moving_altitude=2.4)['type_mask'],1479)
        with self.assertRaises(ValueError):
            gate.encode((.3,0.,0.),0.,(2.,3.),'frame',moving_altitude=float('nan'))

    def test_xyz_stop_latches_altitude_and_explicit_descent_keeps_velocity_mode(self):
        gate = PositionBrake()
        first = gate.encode((0.,0.,0.),0.,(2.,3.),'frame',hold_altitude=2.3)
        drift = gate.encode((0.,0.,0.),0.,(2.1,3.1),'frame',hold_altitude=2.4)
        self.assertEqual(first['type_mask'],1528)
        self.assertEqual(drift['position_z'],2.3)
        self.assertEqual(drift['position_xy'],(2.,3.))
        self.assertEqual(drift['velocity_xyz'],(0.,0.,0.))
        descent = gate.encode((0.,0.,-1.),0.,(2.,3.),'frame',hold_altitude=2.4)
        self.assertEqual(descent['type_mask'],1500)
        self.assertEqual(descent['velocity_xyz'][2],-1.)
        gate.encode((1.,0.,0.),0.,(2.,3.),'frame')
        self.assertEqual(gate.encode((0.,0.,0.),0.,(4.,3.),'frame',hold_altitude=2.6)['position_z'],2.6)

    def test_zero_velocity_locks_position_instead_of_following_drift(self):
        gate = PositionBrake()
        first = gate.encode((0.,0.,.2), 0., (10.,5.), 'epoch')
        second = gate.encode((0.,0.,.1), 0., (11.,6.), 'epoch')
        self.assertEqual(first['type_mask'], 1500)
        self.assertEqual(second['position_xy'], (10.,5.))
        self.assertEqual(second['velocity_xyz'][2], .1)

    def test_only_real_motion_or_transform_change_replaces_anchor(self):
        gate = PositionBrake()
        gate.encode((0.,0.,0.), 0., (10.,5.), 1)
        self.assertEqual(gate.encode((1.,0.,0.), 0., (10.,5.), 1)['type_mask'], 1479)
        self.assertEqual(gate.encode((0.,0.,0.), 0., (11.,6.), 1)['position_xy'], (11.,6.))
        self.assertEqual(gate.encode((0.,0.,0.), 0., (2.,3.), 2)['position_xy'], (2.,3.))
        self.assertEqual(gate.encode((0.,0.,0.), 0., None, 3)['type_mask'], 1479)

    def test_missing_pose_never_invents_anchor_and_nan_is_rejected(self):
        gate = PositionBrake()
        self.assertEqual(gate.encode((0.,0.,0.), 0., None, None)['type_mask'], 1479)
        with self.assertRaises(ValueError):
            gate.encode((float('nan'),0.,0.), 0., None, None)

    def test_actual_ros_adapter_encodes_enu_position_with_standard_mask(self):
        source = Path(__file__).parents[1]/'src/robocup_swarm/scripts/swarm_agent.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))
        method = next(n for c in tree.body if isinstance(c, ast.ClassDef)
                      for n in c.body if isinstance(n, ast.FunctionDef) and n.name == '_publish_command')
        vector = lambda: SimpleNamespace(x=0.,y=0.,z=0.)
        scope = dict(rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda:10.))),
            PositionTarget=lambda:SimpleNamespace(header=SimpleNamespace(),position=vector(),velocity=vector()))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),str(source),'exec'),scope)
        output = []
        agent = SimpleNamespace(local_xy=(2.,3.),_pose_sample_s=10.,offset=(8.,0.),
            _position_brake=PositionBrake(),vel_pub=SimpleNamespace(publish=output.append))
        cmd = SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0.,y=0.,z=.4),angular=SimpleNamespace(z=.2)))
        scope['_publish_command'](agent,cmd)
        self.assertEqual(output[0].type_mask,1500)
        self.assertEqual((output[0].position.x,output[0].position.y),(2.,3.))
        self.assertEqual(output[0].velocity.z,.4)
        self.assertEqual(output[0].yaw_rate,.2)
        agent._xyz_stop_requested,agent.local_z = True,2.3
        cmd.twist.linear.z = 0.
        scope['_publish_command'](agent,cmd)
        self.assertEqual(output[-1].type_mask,1528)
        # A late emergency recovery overrides the earlier horizontal-stop flag.
        cmd.twist.linear.z = .8
        scope['_publish_command'](agent,cmd)
        self.assertEqual(output[-1].type_mask,1500)
        self.assertEqual(output[-1].velocity.z,.8)
