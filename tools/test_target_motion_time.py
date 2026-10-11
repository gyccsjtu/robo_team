"""Original-camera-time regressions, running actual agent methods without ROS.

No flight dynamics are simulated: these tests validate observation consumption
and the search controller's request before the unchanged flight safety chain.
"""
import ast
import math
import os
from pathlib import Path
from types import SimpleNamespace
import unittest

SOURCE = Path(__file__).resolve().parents[1] / 'coordination/src/robocup_swarm/scripts/swarm_agent.py'
TREE = ast.parse(SOURCE.read_text(encoding='utf8'))
CLOCK = {'now': 100.}


class Time:
    def __init__(self, value=0.):
        self.value = value

    def to_sec(self):
        return self.value

    @classmethod
    def now(cls):
        return cls(CLOCK['now'])


ROS = SimpleNamespace(Time=Time, loginfo=lambda *a: None,
                      logwarn=lambda *a: None, logwarn_throttle=lambda *a: None)
CONSTANTS = ('TARGET_TTL', 'TARGET_STALE', 'ESCAPE_SPENT_QUIET_S',
             'ESCAPE_SAMPLE_MAX_GAP_S', 'ESCAPE_RUN_SPEED', 'WALK_SPEED_MAX',
             'MAX_SPEED', 'SPOOK_SPEED', 'SPOOK_DIST', 'UNARMED_BRAKE_DIST',
             'SEARCH_DECEL_M', 'SEARCH_CRUISE_SPEED', 'SEARCH_ARRIVE_TOL', 'POS_KP',
             'DWELL_LOOKAHEAD')
GLOBALS = dict(math=math, os=os, rospy=ROS, serialized_visual_task=lambda f: f)
for node in TREE.body:
    if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id in CONSTANTS for t in node.targets):
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), 'exec'), GLOBALS)
CLASS = next(n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name == 'SwarmAgent')
METHODS = ('_target_cb', '_escape_bookkeeping', '_transit_cap', '_is_armed',
           '_apply_visual_hypothesis', '_control')
for name in METHODS:
    node = next(n for n in CLASS.body if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), 'exec'), GLOBALS)


def make_agent():
    agent = SimpleNamespace(uav_id='u0', world_xy=(0., 0.), targets={},
                            _target_state={}, _t_seen={}, _flee_run_t={},
                            _walk_since={}, _escape_spent=set(), _spend_mode=set())
    for name in METHODS:
        setattr(agent, name, GLOBALS[name].__get__(agent))
    return agent


def observe(agent, sample, speed=0., now=None, tid='t0', x=18.):
    CLOCK['now'] = sample if now is None else now
    msg = SimpleNamespace(target_id=tid, x=x, y=0., vx=speed, vy=0., state=0,
                          eliminated=False, header=SimpleNamespace(stamp=Time(sample)))
    agent._target_cb(msg)


class TargetMotionTimeTests(unittest.TestCase):
    def setUp(self):
        CLOCK['now'] = 100.
        self.agent = make_agent()

    def test_lost_person_has_bounded_search_slowdown_and_retained_history(self):
        observe(self.agent, 100.)
        CLOCK['now'] = 100. + GLOBALS['TARGET_STALE']
        self.assertEqual(self.agent._transit_cap(), GLOBALS['SPOOK_SPEED'])
        CLOCK['now'] += .001
        self.assertEqual(self.agent._transit_cap(), GLOBALS['MAX_SPEED'])
        self.assertEqual(self.agent._t_seen['t0'], 100.)
        self.assertIn('t0', self.agent.targets)
        self.assertTrue(self.agent._is_armed('t0'))  # no judge-state dependency

    def test_one_live_person_still_limits_search_despite_another_stale_one(self):
        observe(self.agent, 90., tid='t0', now=100.)
        observe(self.agent, 100., tid='t1')
        self.assertEqual(self.agent._transit_cap(), GLOBALS['SPOOK_SPEED'])

    def test_missing_future_or_nonfinite_image_times_are_not_control_positions(self):
        self.agent.targets = {'t0': (18., 0., 0., 0.)}
        for stamp in (None, 101., float('nan'), float('inf')):
            self.agent._t_seen = {} if stamp is None else {'t0': stamp}
            self.assertEqual(self.agent._transit_cap(), GLOBALS['MAX_SPEED'])

    def test_actual_search_request_recovers_from_old_target_cache(self):
        a = self.agent
        a.assignment = SimpleNamespace(task_type=0, target_x=30., target_y=0.)
        a._landing = False
        a._takeoff_done = True
        a._target_to_orbit = None
        a._orbit_target = None
        a._hold_camera_candidate = lambda: False
        a._update_orbit = lambda: None
        a.path_target = (30., 0.)
        a._pick_local_goal = lambda: (3., 0.)
        a._search_start_cap = lambda vx, vy: (vx, vy)
        a._apply_friend_avoidance = lambda vx, vy: (vx, vy)
        commands = []
        a._send_vel = lambda vx, vy: commands.append((vx, vy))
        old_dwell = GLOBALS['DWELL_LOOKAHEAD']
        GLOBALS['DWELL_LOOKAHEAD'] = False
        # In-bounds fixture; this helper is unrelated to camera-time logic.
        GLOBALS['flight_waypoint'] = lambda x, y: (x, y)
        try:
            observe(a, 100.)
            a._control()
            self.assertAlmostEqual(commands[-1][0], GLOBALS['SPOOK_SPEED'])
            CLOCK['now'] = 100. + GLOBALS['TARGET_STALE'] + .001
            a._control()
            self.assertAlmostEqual(commands[-1][0], GLOBALS['SEARCH_CRUISE_SPEED'])
        finally:
            GLOBALS['DWELL_LOOKAHEAD'] = old_dwell

    def test_run_time_is_original_image_time_even_with_variable_delivery_latency(self):
        observe(self.agent, 100., speed=2., now=100.8)
        self.assertEqual(self.agent._flee_run_t['t0'], 100.)
        observe(self.agent, 100., speed=2., now=101.8)
        self.assertEqual(self.agent._flee_run_t['t0'], 100.)
        self.assertEqual(self.agent._escape_sample_s['t0'], 100.)
        observe(self.agent, 99.5, speed=2., now=101.8)
        self.assertEqual(self.agent._flee_run_t['t0'], 100.)

    def test_old_first_run_image_cannot_arm_motion_inference(self):
        observe(self.agent, 90., speed=2., now=100.)
        self.assertEqual(self.agent._flee_run_t, {})
        self.assertEqual(self.agent._walk_since, {})

    def test_duplicate_low_speed_samples_do_not_complete_a_quiet_streak(self):
        observe(self.agent, 100., speed=2.)
        observe(self.agent, 100.5, speed=1.)
        observe(self.agent, 100.5, speed=1., now=102.)
        self.assertNotIn('t0', self.agent._escape_spent)
        self.assertEqual(self.agent._walk_since['t0'], 100.5)
        observe(self.agent, 100.5, speed=1., now=110.)
        self.assertEqual(self.agent._walk_since, {})
        self.assertNotIn('t0', self.agent._escape_spent)

    def test_sparse_low_speed_samples_restart_instead_of_claiming_eight_seconds(self):
        observe(self.agent, 100., speed=2.)
        observe(self.agent, 104., speed=1.)
        observe(self.agent, 116., speed=1.)
        self.assertEqual(self.agent._walk_since['t0'], 116.)
        self.assertNotIn('t0', self.agent._escape_spent)

    def test_new_low_speed_images_can_complete_a_real_eight_second_streak(self):
        observe(self.agent, 100., speed=2.)
        for k in range(17):
            sample = 100.5 + k*.5
            observe(self.agent, sample, speed=1., now=sample + (k % 3)*.1)
            if k < 16:
                self.assertNotIn('t0', self.agent._escape_spent)
        self.assertIn('t0', self.agent._escape_spent)
        self.assertEqual(self.agent._walk_since['t0'], 100.5)

    def test_intermediate_speed_interrupts_low_speed_streak(self):
        observe(self.agent, 100., speed=2.)
        observe(self.agent, 100.5, speed=1.)
        observe(self.agent, 101., speed=1.5)
        self.assertEqual(self.agent._walk_since, {})
        observe(self.agent, 101.5, speed=1.)
        self.assertEqual(self.agent._walk_since['t0'], 101.5)

    def test_continuity_gap_boundary_is_inclusive_and_larger_gap_restarts(self):
        gap = GLOBALS['ESCAPE_SAMPLE_MAX_GAP_S']
        observe(self.agent, 100., speed=2.)
        observe(self.agent, 100.5, speed=1.)
        observe(self.agent, 100.5+gap, speed=1.)
        self.assertEqual(self.agent._walk_since['t0'], 100.5)
        observe(self.agent, 100.5+2*gap+.001, speed=1.)
        self.assertEqual(self.agent._walk_since['t0'], 100.5+2*gap+.001)

    def test_nonfinite_motion_interrupts_streak(self):
        observe(self.agent, 100., speed=2.)
        observe(self.agent, 100.5, speed=1.)
        observe(self.agent, 101., speed=float('nan'))
        self.assertEqual(self.agent._walk_since, {})
        self.assertNotIn('t0', self.agent._escape_spent)

    def test_new_person_hypothesis_does_not_inherit_consumed_motion_clock(self):
        observe(self.agent, 100., speed=2.)
        observe(self.agent, 100.5, speed=1.)
        self.agent.assignment = None
        self.agent._apply_visual_hypothesis(dict(target_id='t0', sample_s=101.,
            x=30., y=0., hypothesis_switch=dict(switch_s=101.)))
        self.assertEqual(self.agent._escape_sample_s, {})
        self.assertEqual(self.agent._flee_run_t, {})
        self.assertEqual(self.agent._walk_since, {})


if __name__ == '__main__':
    unittest.main()
