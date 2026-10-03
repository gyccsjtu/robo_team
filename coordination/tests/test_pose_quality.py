import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

SCRIPTS = Path(__file__).parents[1] / 'src/robocup_swarm/scripts'
sys.path.insert(0, str(SCRIPTS))
from pose_quality import PoseQuality
from position_brake import PositionBrake


class PoseQualityTests(unittest.TestCase):
    def test_real_motion_and_preflight_origin_initialization(self):
        guard = PoseQuality()
        self.assertTrue(guard.accept((0., 0., 0.), 1.))
        self.assertTrue(guard.accept((40., 0., 0.), 1.02, enforce=False))
        self.assertTrue(guard.accept((40.06, 0., .02), 1.04))
        self.assertTrue(guard.usable(1.3))

    def test_collision_jump_is_latched_even_when_new_frame_stabilizes(self):
        guard = PoseQuality()
        guard.accept((0., 0., 3.), 10.)
        self.assertFalse(guard.accept((80., -30., 3.), 10.02))
        self.assertEqual(guard.fault, 'IMPLAUSIBLE_POSE_JUMP')
        self.assertFalse(guard.accept((80.01, -30., 3.), 11.))
        self.assertFalse(guard.usable(11.))
        self.assertEqual(guard.sample[0], (0., 0., 3.))

    def test_vertical_jump_invalidates_height_control_too(self):
        guard = PoseQuality()
        guard.accept((0., 0., 2.8), 10.)
        self.assertFalse(guard.accept((0., 0., -12.), 10.02))

    def test_repeated_subthreshold_jumps_are_not_mistaken_for_real_motion(self):
        guard = PoseQuality()
        guard.accept((0., 0., 3.), 10.)
        for i in range(1, 16):
            if not guard.accept((i * .4, 0., 3.), 10. + i * .02):
                break
        self.assertEqual(guard.fault, 'IMPLAUSIBLE_POSE_SPEED')

    def test_duplicate_stale_nonfinite_and_clock_reset(self):
        guard = PoseQuality()
        guard.accept((0., 0., 0.), 10.)
        self.assertFalse(guard.accept((1., 0., 0.), 10.))
        self.assertFalse(guard.usable(10.6))
        self.assertTrue(guard.accept((.1, 0., 0.), 10.7))
        self.assertFalse(guard.accept((.1, 0., 0.), 9.))
        self.assertEqual(guard.fault, 'POSE_CLOCK_REVERSED')
        other = PoseQuality()
        self.assertFalse(other.accept((math.nan, 0., 0.), 1.))

    def test_ros_final_publisher_discards_old_position_anchor(self):
        tree = ast.parse((SCRIPTS / 'swarm_agent.py').read_text(encoding='utf-8'))
        node = next(n for c in tree.body if isinstance(c, ast.ClassDef)
                    for n in c.body if isinstance(n, ast.FunctionDef) and n.name == '_publish_command')
        vector = lambda: SimpleNamespace(x=0., y=0., z=0.)
        twist = lambda: SimpleNamespace(twist=SimpleNamespace(linear=vector(), angular=vector()))
        scope = dict(rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda:10.1))),
                     TwistStamped=twist,
                     PositionTarget=lambda: SimpleNamespace(header=SimpleNamespace(), position=vector(), velocity=vector()))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), 'adapter', 'exec'), scope)
        brake = PositionBrake()
        brake.encode((0., 0., 0.), 0., (1., 2.), (10., 20.))
        quality = PoseQuality()
        quality.accept((1., 2., 3.), 10.)
        quality.accept((100., 2., 3.), 10.02)
        output = []
        agent = SimpleNamespace(_pose_quality=quality, _position_brake=brake,
                                local_xy=(1., 2.), _pose_sample_s=10., offset=(10., 20.),
                                vel_pub=SimpleNamespace(publish=output.append))
        cmd = twist()
        cmd.twist.linear.x, cmd.twist.linear.z = 3., 1.
        scope['_publish_command'](agent, cmd)
        self.assertEqual(output[0].type_mask, 1479)
        self.assertEqual((output[0].velocity.x, output[0].velocity.y, output[0].velocity.z), (0., 0., 0.))
        self.assertIsNone(brake.anchor)

    def test_freshness_expiry_cannot_turn_coordinate_read_into_none_mid_tick(self):
        tree = ast.parse((SCRIPTS / 'swarm_agent.py').read_text(encoding='utf-8'))
        names = {'world_xy', '_publish_motion', '_publish_status', '_publish_authority_state'}
        methods = [n for c in tree.body if isinstance(c, ast.ClassDef)
                   for n in c.body if isinstance(n, ast.FunctionDef) and n.name in names]
        now = [10.1]
        scope = dict(rospy=SimpleNamespace(Time=SimpleNamespace(
            now=lambda: SimpleNamespace(to_sec=lambda:now[0]))))
        exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])), 'adapter', 'exec'), scope)
        quality = PoseQuality()
        quality.accept((1., 2., 3.), 10.)
        agent = SimpleNamespace(_pose_quality=quality, local_xy=(1., 2.), offset=(10., 20.),
                                _stopped_since=9.)
        coordinate = scope['world_xy'].fget
        self.assertEqual(coordinate(agent), (11., 22.))
        now[0] = 10.6
        self.assertEqual(coordinate(agent), (11., 22.))
        # None of these methods may access a publisher or refresh an ACK using
        # the stable-but-stale coordinate. No such publishers exist on this stub.
        for name in ('_publish_motion', '_publish_status', '_publish_authority_state'):
            scope[name](agent)
        self.assertIsNone(agent._stopped_since)
        quality.accept((100., 2., 3.), 10.62)
        self.assertIsNone(coordinate(agent))


if __name__ == '__main__':
    unittest.main()
