"""Actual target-task callbacks: fresh retry, stopped observation and STOP fences."""
import ast
import json
import math
from pathlib import Path
import sys
import threading
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock

scripts = Path(__file__).parents[1]/'src/robocup_swarm/scripts'
sys.path.insert(0, str(scripts))
from target_motion import current_target_point
from task_authority import TaskGate


class TrackingBackoffTests(unittest.TestCase):
    def setUp(self):
        self.now = 20.
        source = scripts/'swarm_agent.py'
        cls = next(n for n in ast.parse(source.read_text()).body
                   if isinstance(n, ast.ClassDef) and n.name == 'SwarmAgent')
        names = ('_authorized_cb', '_assign_cb', '_resume_tracking_attempt', '_control', '_orbit_stale')
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
        self.scope = dict(json=json, math=math, current_target_point=current_target_point,
            TRACK_GUIDANCE_TTL=1.5, SearchAssignment=SimpleNamespace,
            rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:self.now)),
                                  loginfo=Mock(), logwarn_throttle=Mock()))
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'), self.scope)
        self.agent = a = SimpleNamespace(uav_id='uav_4', _gate=TaskGate('run', 'uav_4'),
            _authority_lock=threading.RLock(), _plan_lock=threading.RLock(),
            _route_gate=SimpleNamespace(clear=Mock()), _plan_ticket=0, _plan_pending=None,
            _tracking_retry_pending=None, _track_assigned_id=None, _giveup_until={'t3':60.},
            _reset_n={3:7}, targets={'t3':(12.,6.,1.,0.)}, _t_seen={'t3':19.9},
            _online_planner=None, _landing=False, _takeoff_done=True, local_z=2.2,
            world_xy=(0.,0.), assignment=None, _target_to_orbit=None,
            _look_at=None, _send_vel=Mock(), _pick_local_goal=Mock(side_effect=AssertionError('SEARCH_FALLTHROUGH')))
        for name in names:
            setattr(a, name, MethodType(self.scope[name], a))

    def grant(self, seq=1, generation=1, action='GRANT', tid='t3'):
        task = dict(cell_ix=-1, cell_iy=-1, target_x=99., target_y=99., task_type=1, target_id=tid)
        self.agent._authorized_cb(SimpleNamespace(data=json.dumps(dict(schema_version=2,
            run_id='run', uav_id='uav_4', seq=seq, generation=generation,
            action=action, expires_s=self.now+10., task=task))))

    def test_new_grant_uses_fresh_camera_and_cannot_clear_later_same_attempt_backoff(self):
        self.grant()
        a = self.agent
        self.assertNotIn('t3', a._giveup_until)
        self.assertEqual(a._reset_n[3], 0)
        self.assertEqual(a._target_to_orbit, (12.,6.))  # Not assignment's 99,99.
        a._giveup_until['t3'] = 80.; a._reset_n[3] = 3
        self.grant(seq=2)
        self.assertEqual(a._giveup_until['t3'], 80.)
        self.assertEqual(a._reset_n[3], 3)
        a._control()
        a._send_vel.assert_called_once_with(0.,0.)
        self.assertEqual(a._look_at, (12.,6.))
        a._pick_local_goal.assert_not_called()

    def test_new_generation_after_stop_may_retry_only_with_current_evidence(self):
        self.grant(); self.agent._giveup_until['t3'] = 80.
        self.grant(seq=2, action='STOP')
        self.agent._t_seen['t3'] = 18.
        self.grant(seq=3, generation=2)
        a = self.agent
        self.assertEqual(a._giveup_until['t3'], 80.)
        self.assertEqual(a._tracking_retry_pending, ('t3',2))
        a._control(); a._send_vel.assert_called_once_with(0.,0.)
        self.assertEqual(a._look_at, (12.,6.))  # Pending new grant may only reacquire by yaw.
        a._t_seen['t3'] = 19.95
        self.assertTrue(a._resume_tracking_attempt())
        self.assertNotIn('t3', a._giveup_until)
        self.assertFalse(a._resume_tracking_attempt())
        self.assertIsNone(a._route_pending)  # No fabricated route grant.

    def test_pending_new_attempt_can_aim_but_not_resume_from_old_actual_frame(self):
        self.agent._t_seen['t3'] = 16.428  # Actual white regrant window was 3.572s old.
        self.grant()
        a = self.agent
        a._control()
        a._send_vel.assert_called_once_with(0.,0.)
        self.assertEqual(a._look_at, (12.,6.))
        self.assertEqual(a._tracking_retry_pending, ('t3',1))
        self.assertEqual(a._giveup_until['t3'],60.)
        self.assertEqual(a._reset_n[3],7)
        self.assertEqual(a._t_seen['t3'],16.428)
        a._pick_local_goal.assert_not_called()
        self.assertIsNone(a._route_pending)

    def test_old_frame_cannot_aim_after_retry_marker_consumed_or_generation_changes(self):
        self.grant(); a = self.agent
        a._giveup_until['t3']=80.; a._t_seen['t3']=17.
        a._control(); self.assertIsNone(a._look_at)
        a._send_vel.assert_called_once_with(0.,0.)
        a._tracking_retry_pending=('t3',a._gate.generation+1)
        a._control(); self.assertIsNone(a._look_at)

    def test_pending_reacquisition_age_and_coordinate_bounds(self):
        for stamp,xy in [(14.999,(12.,6.)),(20.001,(12.,6.)),
                         (float('nan'),(12.,6.)),(None,(12.,6.)),
                         (17.,(float('inf'),6.))]:
            with self.subTest(stamp=stamp,xy=xy):
                self.setUp(); self.agent._t_seen['t3']=stamp
                self.agent.targets['t3']=xy; self.grant();self.agent._control()
                self.assertIsNone(self.agent._look_at)
                self.agent._send_vel.assert_called_once_with(0.,0.)
                self.assertEqual(self.agent._giveup_until['t3'],60.)

    def test_missing_orbit_point_stays_in_target_task_and_observes_fresh_person(self):
        self.grant(); a = self.agent
        a._target_to_orbit = None
        a._control()
        a._send_vel.assert_called_once_with(0.,0.)
        self.assertEqual(a._look_at, (12.,6.))
        a._pick_local_goal.assert_not_called()

    def test_stale_future_missing_or_invalid_camera_cannot_aim_or_end_cooldown(self):
        for stamp in (18.999,20.001,float('nan'),None):
            with self.subTest(stamp=stamp):
                self.setUp(); self.agent._t_seen['t3'] = stamp
                self.grant()
                self.assertEqual(self.agent._giveup_until['t3'], 60.)
                self.assertIsNone(current_target_point(self.agent.targets,self.agent._t_seen,'t3',20.))
        self.agent._t_seen.clear()
        self.assertIsNone(current_target_point(self.agent.targets,self.agent._t_seen,'t3',20.))

    def test_stop_and_expired_grant_block_observation_and_pending_retry(self):
        self.agent._t_seen.clear(); self.grant()
        self.grant(seq=2, action='STOP')
        a = self.agent
        self.assertIsNone(a._tracking_retry_pending)
        a._t_seen['t3'] = 19.9
        a._control(); self.assertIsNone(a._look_at)
        a._send_vel.assert_called_once_with(0.,0.)
        self.assertEqual(a._giveup_until['t3'], 60.)
        self.setUp(); self.agent._t_seen.clear(); self.grant()
        self.now = 31.; self.agent._t_seen['t3'] = 30.9
        self.assertFalse(self.agent._resume_tracking_attempt())
        self.assertEqual(self.agent._giveup_until['t3'],60.)

    def test_other_target_evidence_cannot_resume_new_target_attempt(self):
        self.grant(tid='t1')
        self.assertEqual(self.agent._giveup_until['t3'],60.)
        self.assertEqual(self.agent._tracking_retry_pending,('t1',1))
        self.assertFalse(self.agent._resume_tracking_attempt())

    def test_official_elimination_cannot_be_cleared_by_new_grant_or_ghost_image(self):
        self.agent._giveup_until['t3'] = float('inf')
        self.grant()
        self.assertEqual(self.agent._giveup_until['t3'],float('inf'))
        self.assertFalse(self.agent._resume_tracking_attempt())
        self.agent._control()
        self.agent._send_vel.assert_called_once_with(0.,0.)
        self.assertIsNone(self.agent._look_at)

    def test_v124_profile_preserves_cooldown_and_stops_without_search_fallthrough(self):
        self.agent._v124_behavior=True
        self.grant();a=self.agent
        self.assertEqual(a._giveup_until['t3'],60.)
        self.assertEqual(a._reset_n[3],7)
        self.assertFalse(a._resume_tracking_attempt())
        a._target_to_orbit=None;a._control()
        a._send_vel.assert_called_once_with(0.,0.)
        self.assertIsNone(a._look_at)
        a._pick_local_goal.assert_not_called()

    def test_v124_profile_has_no_new_attempt_counter_reset(self):
        self.agent._v124_behavior=True
        self.agent._giveup_until.clear();self.grant()
        a=self.agent
        self.scope['ORBIT_RADIUS']=3.
        a.world_xy=(99.,99.)
        a._fly_orbit=Mock();a._control()
        self.assertEqual(a._reset_n[3],7)
        a._fly_orbit.assert_called_once_with()
