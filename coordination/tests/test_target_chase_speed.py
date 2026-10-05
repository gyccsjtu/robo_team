"""Exercise the actual command branch for delayed FLEE and stale red slots."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import test_tracking_backoff as backoff
import test_reserved_orbit as orbit


class TargetChaseSpeedTests(unittest.TestCase):
    def setUp(self):
        fixture = backoff.TrackingBackoffTests()
        fixture.setUp()
        self.fixture = fixture
        fixture.scope.update(MAX_SPEED=3., FLEE_CHASE_SPEED=2.6, ORBIT_RADIUS=8.,
                             POS_KP=1., ARRIVE_TOL=1.)
        fixture.grant()
        self.agent = a = fixture.agent
        a._giveup_until.clear()
        a._resume_tracking_attempt = Mock(return_value=False)
        a._need_replan_track = Mock(return_value=False)
        a._pick_local_goal = Mock(return_value=(0., 8.))
        a._apply_friend_avoidance = lambda x, y: (x, y)
        a._target_fleeing = Mock(return_value=False)

    def test_approach_does_not_wait_for_flee_classification(self):
        self.agent._control()
        self.assertEqual(self.agent._send_vel.call_args.args, (0., 2.6))
        self.agent._target_fleeing.assert_not_called()

    def test_task_stop_still_blocks_chase(self):
        self.fixture.grant(seq=2, action='STOP')
        self.agent._control()
        self.assertEqual(self.agent._send_vel.call_args.args, (0., 0.))
        self.agent._pick_local_goal.assert_not_called()

    def test_chase_respects_lower_configured_maximum(self):
        self.fixture.scope['MAX_SPEED'] = 2.
        self.agent._control()
        self.assertEqual(self.agent._send_vel.call_args.args, (0., 2.))

    def test_search_not_slowed_by_stale_nearby_red_slot(self):
        a = self.agent
        a.assignment = SimpleNamespace(task_type=0, target_x=0., target_y=10.)
        a.path_target = (0., 10.)
        a._orbit_target = None
        a._update_orbit = Mock()
        a.targets = {'t5': (0., 1., 0., 0.)}
        a._t_seen = {'t5': 0.}
        a._nearest_target_dist = Mock(return_value=('t5', 1.))
        a._control()
        self.assertEqual(a._send_vel.call_args.args, (0., 3.))
        a._nearest_target_dist.assert_not_called()

    def test_small_tracking_error_is_not_forced_to_chase_speed(self):
        self.agent._pick_local_goal.return_value = (0., .3)
        self.agent._control()
        self.assertEqual(self.agent._send_vel.call_args.args, (0., .3))

    def test_downstream_orbit_limit_does_not_restore_one_point_five_cap(self):
        a = SimpleNamespace(_orbit_target='t5', _last_heading=None,
                            _target_fleeing=Mock(return_value=False))
        limited = orbit.scope['_adaptive_speed'](a, 0., 2.6)
        self.assertEqual(limited, (0., 2.6))
        self.assertGreater(math.hypot(*limited), 2.)
        a._target_fleeing.assert_not_called()
