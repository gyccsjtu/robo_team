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

    def test_teammate_grazing_wall_hits_on_skipped_indices_still_brake(self):
        # Reproduce f45671b's grazing wall with every even-index echo missing.
        # The six-aircraft final guard must still consume all 512 beams.
        n = 512
        increment = 2 * math.pi / n
        ranges = [math.inf] * n
        for i in range(1, n, 2):
            angle = -math.pi + i * increment
            if math.sin(angle) <= 1e-6:
                continue
            x = .55 / math.tan(angle)
            if 2. <= x <= 8.:
                ranges[i] = .55 / math.sin(angle)
        self.assertTrue(any(math.isfinite(r) for r in ranges))
        self.assertTrue(all(r == math.inf for r in ranges[::2]))
        result = self.check(request=(3., 0.), measured=(3., 0.), ranges=ranges,
                            angle_increment=increment)
        self.assertEqual(result['reason'], 'BRAKING_REQUIRED')
        self.assertEqual(result['velocity_xy'], (0., 0.))

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
        names = {'_radar_guard_velocity', '_grid_guard_velocity', '_online_velocity_clear', '_scan_cb', '_velocity_cb', '_send_vel'}
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

    def test_online_guard_slows_inside_known_corridor_without_unreserved_turn(self):
        import threading
        from online_radar_planner import OnlinePlanner
        from radar_observed_map import ObservedMap
        from robocup_navigation.astar import GridMap
        # A three-cell-wide observed corridor: diagonal cone probes are blocked,
        # but the requested centerline and a reduced stopping distance are clear.
        cells=bytes(0 if 3 <= iy <= 5 and ix < 12 else 1
                    for iy in range(9) for ix in range(20))
        grid=GridMap(20,9,.5,(0.,-2.25),cells,'map')
        observed=ObservedMap(20,9,.5,(0.,-2.25))
        observed.cells=[0 if ix < 12 else -1 for iy in range(9) for ix in range(20)]
        observed.observed_s=[10. if value == 0 else None for value in observed.cells]
        observed.last_scan_s,observed.version=10.,1
        a=types.SimpleNamespace(world_xy=(4.,0.),_online_planner=OnlinePlanner((0.,0.)),
            _online_map=observed,_online_map_lock=threading.RLock(),_velocity_sample=(0.,0.,10.),
            _online_safe_grid=grid,_online_safe_s=8.,_online_safe_epoch=1.,_online_map_epoch_s=1.)
        a._online_velocity_clear=lambda vx,vy:self.methods['_online_velocity_clear'](a,vx,vy)
        self.methods['GRID_GUARD']=1
        vx,vy=self.methods['_grid_guard_velocity'](a,2.,0.)
        self.assertGreater(vx,0.);self.assertLess(vx,2.);self.assertEqual(vy,0.)
        self.assertTrue(a._online_planner.command_clear(grid,a.world_xy,(vx,vy)))
        for stamp in (8.,10.2):
            observed.last_scan_s=stamp
            self.assertEqual(self.methods['_grid_guard_velocity'](a,2.,0.),(0.,0.))

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
            _friend_guard_velocity=lambda x, y: (x, y),
            _publish_command=published.append)
        obj._radar_guard_velocity = types.MethodType(self.methods['_radar_guard_velocity'], obj)
        self.methods['rospy'].logerr_throttle = lambda *a: None
        self.methods['_send_vel'](obj, 3., 0.)
        self.assertEqual((published[0].twist.linear.x, published[0].twist.linear.y), (0., 0.))
        self.assertEqual(obj._last_cmd_v, (0., 0.))
        self.assertEqual(obj._last_flight_v, (0., 0.))

    def test_city_altitude_profile_overrides_even_explicit_climb(self):
        city = SCRIPTS.parents[2] / 'scripts/city_swarm_run.py'
        tree = ast.parse(city.read_text(encoding='utf-8'))
        profile = next(n for n in ast.walk(tree) if isinstance(n, ast.Call)
                       and any(k.arg == 'ALT_BASE' for k in n.keywords))
        values = {k.arg: float(ast.literal_eval(k.value)) for k in profile.keywords
                  if k.arg and k.arg.startswith('ALT_')}
        original = {k: self.methods.get(k) for k in values}
        extra = {'ALT_HARD_DESCENT': -1., 'ALT_PANIC_DESCENT': -1.5,
                 'ALT_EMERG_DESCENT': -2., 'ALT_EMERG_HSCALE': .35,
                 '_alt_guard_hit': lambda *args: None}
        original.update({k: self.methods.get(k) for k in extra})
        self.methods.update(values); self.methods.update(extra)
        try:
            for z, expected in ((4.11, -1.), (4.31, -1.5), (4.51, -2.)):
                with self.subTest(local_z=z):
                    published = []
                    obj = types.SimpleNamespace(
                        _bounds_recovery_velocity=lambda: None, world_xy=(0., 0.),
                        _grid_guard_velocity=lambda x, y: (x, y),
                        _map_guard_velocity=lambda x, y: (x, y),
                        _last_cmd_v=(0., 0.), local_z=z, altitude_layer=values['ALT_BASE'],
                        _compute_desired_altitude=lambda: 0., _last_csv_t=10.1, _look_at=None,
                        _gate=types.SimpleNamespace(can_move=lambda now: True),
                        _radar_guard_velocity=lambda x, y: (x, y),
                        _friend_guard_velocity=lambda x, y: (x, y),
                        _publish_command=published.append, uav_id='uav_5')
                    self.methods['_send_vel'](obj, 0., 0., vz=1.)
                    self.assertEqual(published[0].twist.linear.z, expected)
        finally:
            self.methods.update(original)


if __name__ == '__main__':
    unittest.main()
