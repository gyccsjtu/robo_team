"""Directional, sensor-loss and momentum regressions; no ROS needed."""
import ast
import math
from pathlib import Path
import sys
import types
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / 'src/robocup_swarm/scripts'
sys.path.insert(0, str(SCRIPTS))
from radar_velocity_guard import guard_velocity


class RadarGuardTests(unittest.TestCase):
    def check(self, request=(1., 0.), measured=(0., 0.), yaw=0., **changes):
        kw = dict(request=request, measured=measured, yaw=yaw,
                  ranges=[math.inf] * 360, angle_min=-math.pi,
                  angle_increment=math.pi / 180, range_min=.5, range_max=20.,
                  scan_s=10., now_s=10.1)
        kw.update(changes)
        return guard_velocity(**kw)

    def scan_hit(self, deg, distance):
        scan = [math.inf] * 360
        scan[(deg + 180) % 360] = distance
        return scan

    def test_clear_scan_preserves_request(self):
        self.assertEqual(self.check()['velocity_xy'], (1., 0.))

    def test_missing_delayed_and_future_scan_stop(self):
        for change in (dict(ranges=None), dict(scan_s=9.), dict(scan_s=11.),
                       dict(ranges=[]), dict(ranges=[math.inf] * 90)):
            self.assertEqual(self.check(**change)['velocity_xy'], (0., 0.))

    def test_invalid_ranges_and_configuration_stop(self):
        for r in (math.nan, -math.inf, .1):
            self.assertEqual(self.check(ranges=[r] * 360)['velocity_xy'], (0., 0.))
        for change in (dict(brake_accel=0), dict(radius=math.nan),
                       dict(angle_increment=0), dict(measured=(math.inf, 0.))):
            self.assertEqual(self.check(**change)['velocity_xy'], (0., 0.))

    def test_side_and_backward_motion_are_guarded(self):
        for direction, deg in (((0., 2.), 90), ((-2., 0.), -180)):
            result = self.check(request=direction, ranges=self.scan_hit(deg, 1.))
            self.assertEqual(result['velocity_xy'], (0., 0.))

    def test_yaw_rotates_scan_to_enu(self):
        result = self.check(request=(0., 2.), yaw=math.pi / 2,
                            ranges=self.scan_hit(0, 1.))
        self.assertEqual(result['velocity_xy'], (0., 0.))

    def test_measured_momentum_stops_even_when_new_request_turns(self):
        result = self.check(request=(0., 1.), measured=(3., 0.),
                            ranges=self.scan_hit(0, 3.))
        self.assertEqual(result['reason'], 'BRAKING_REQUIRED')
        self.assertEqual(result['velocity_xy'], (0., 0.))

    def test_speed_limit_obeys_stopping_distance_without_side_step(self):
        result = self.check(request=(5., 0.), ranges=self.scan_hit(0, 5.))
        vx, vy = result['velocity_xy']
        self.assertEqual(vy, 0.)
        self.assertGreater(vx, 0.)
        self.assertLess(vx, 5.)
        self.assertLessEqual(vx * .5 + vx * vx, result['clearance_m'] + 1e-12)

    def test_beam_width_protects_between_ray_hits(self):
        # A hit just outside the centreline corridor still intersects the
        # swept radius after accounting for finite angular resolution.
        r = 1.21 / math.sin(math.radians(30))
        result = self.check(request=(4., 0.), ranges=self.scan_hit(30, r))
        self.assertEqual(result['reason'], 'SLOW')

    def test_sensor_horizon_limits_speed(self):
        result = self.check(request=(10., 0.), range_max=5.)
        self.assertLess(result['velocity_xy'][0], 2.)


class AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Execute the actual thin adapter methods without importing ROS or
        # constructing the large map/control node.
        tree = ast.parse((SCRIPTS / 'swarm_agent.py').read_text(encoding='utf-8'))
        agent = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SwarmAgent')
        names = {'_radar_guard_velocity', '_scan_cb', '_velocity_cb', '_send_vel'}
        selected = [n for n in agent.body if isinstance(n, ast.FunctionDef) and n.name in names]
        module = ast.fix_missing_locations(ast.Module(body=selected, type_ignores=[]))
        env = dict(RADAR_GUARD=1, RADAR_FRESH_S=.5, RADAR_STOP_R=1.2,
                   RADAR_LATENCY_S=.5, RADAR_BRAKE_MPS2=.5, guard_velocity=guard_velocity)
        env.update(math=math, MAX_ACC=2.5, CTRL_RATE=20., ALT_PANIC=5.7,
                   ALT_PANIC_HSCALE=.3, ALT_TARGET_CAP=4.5, ALT_HARD_CEIL=5.5,
                   ALT_HARD_MARGIN=.1, MIN_CRUISE_ALT=2., ALT_P=1., ALT_VZ_MIN=.2,
                   ALT_EMERG_CEIL=5.9, CRASH_DETECT=False)
        def twist():
            vector = lambda: types.SimpleNamespace(x=0., y=0., z=0.)
            return types.SimpleNamespace(twist=types.SimpleNamespace(linear=vector(), angular=vector()))
        env['TwistStamped'] = twist
        env['rospy'] = types.SimpleNamespace(
            Time=types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: 10.1)),
            logwarn_throttle=lambda *a: None)
        exec(compile(module, 'swarm_agent_adapter', 'exec'), env)
        cls.methods = env

    def test_missing_measured_velocity_stops_adapter(self):
        obj = types.SimpleNamespace(_scan=object(), _velocity_sample=None,
                                    _local_prev_t=10., uav_id='uav_1')
        self.assertEqual(self.methods['_radar_guard_velocity'](obj, 2., 0.), (0., 0.))

    def test_delayed_scan_is_not_refreshed_by_arrival(self):
        stamp = types.SimpleNamespace(to_sec=lambda: 1.)
        scan = types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp),
                                     ranges=[math.inf] * 360, angle_min=-math.pi,
                                     angle_increment=math.pi / 180, range_min=.5, range_max=20.)
        obj = types.SimpleNamespace(_velocity_sample=(0., 0., 10.),
                                    _local_prev_t=10., yaw=0., uav_id='uav_1')
        self.methods['_scan_cb'](obj, scan)
        self.assertEqual(obj._scan_t, 1.)
        self.assertEqual(self.methods['_radar_guard_velocity'](obj, 2., 0.), (0., 0.))

    def test_final_stop_survives_bounds_recovery_and_slew_limiter(self):
        published = []
        obj = types.SimpleNamespace(
            _scan=None, _velocity_sample=None, _local_prev_t=10., uav_id='uav_1',
            _bounds_recovery_velocity=lambda: (2., 0.), world_xy=(120., 0.),
            _last_cmd_v=(5., 0.), local_z=4., altitude_layer=4.,
            _compute_desired_altitude=lambda: 0., _last_csv_t=10.1, _look_at=None,
            _gate=types.SimpleNamespace(can_move=lambda now: True),
            vel_pub=types.SimpleNamespace(publish=published.append))
        obj._radar_guard_velocity = types.MethodType(self.methods['_radar_guard_velocity'], obj)
        self.methods['rospy'].logerr_throttle = lambda *a: None
        self.methods['_send_vel'](obj, 3., 0.)
        self.assertEqual((published[0].twist.linear.x, published[0].twist.linear.y), (0., 0.))
        self.assertEqual(obj._last_cmd_v, (0., 0.))
        self.assertEqual(obj._last_flight_v, (0., 0.))


if __name__ == '__main__':
    unittest.main()
