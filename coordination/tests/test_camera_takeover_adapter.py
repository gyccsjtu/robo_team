"""Exercise actual manager dispatch and withdrawal without ROS."""
import ast
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

scripts = Path(__file__).parents[1]/'src/robocup_swarm/scripts'
sys.path.insert(0, str(scripts))
from tracker_selection import tracker_rank, takeover_candidate
from visual_observation import TAG_TO_TID

source = scripts/'swarm_manager.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
methods = [node for cls in tree.body if isinstance(cls, ast.ClassDef)
           for node in cls.body if isinstance(node, ast.FunctionDef)
           and node.name in ('_dispatch_tracker', '_consider_camera_takeover')]
scope = dict(math=math, TAG_TO_TID=TAG_TO_TID, takeover_candidate=takeover_candidate,
             DETECT_RADIUS=20., DISPATCH_MARGIN=5., STATE_ASSIGNED=1, CRUISE_SPEED=3.,
             SearchAssignment=lambda: SimpleNamespace(header=SimpleNamespace(), target_id=''),
             rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda: 10.),
                                   loginfo=Mock(), logwarn_throttle=Mock()))
exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])),
             str(source), 'exec'), scope)


class CameraTakeoverAdapterTests(unittest.TestCase):
    def manager(self):
        evidence = SimpleNamespace(stamps={('live', 'green'): 9.9})
        authority = SimpleNamespace(withdraw=Mock(return_value=['stop']))
        m = SimpleNamespace(uav_ids=['blind', 'live', 'owner'],
            status={'blind': SimpleNamespace(connected=True, x=2., y=0.),
                    'live': SimpleNamespace(connected=True, x=10., y=0.),
                    'owner': SimpleNamespace(connected=False, x=8., y=0.)},
            grid=SimpleNamespace(cells={0: SimpleNamespace(state=1, owner='live')},
                                 x_min=-50., x_max=130., y_min=-60., y_max=60.),
            _tracking={}, _backup={}, _idle_uavs=lambda: ['blind'],
            _target_authority_held=lambda tid: False, _authorized_publish=Mock(),
            _authority=authority, _authority_lock=threading.RLock(), _emit_authority=Mock(),
            _visual_evidence=evidence, _truth_cache={'t0': (0., 0., 9.9)})
        m._tracker_rank = lambda tid, uid, d: tracker_rank(uid, d,
            [(u, t) for (u, tag), t in evidence.stamps.items()], 10., 15.)
        return m

    def test_search_camera_beats_blind_idle_aircraft_in_actual_dispatch(self):
        m = self.manager()
        scope['_dispatch_tracker'](m, 't0', 0., 0.)
        self.assertEqual(m._authorized_publish.call_args.args[0].uav_id, 'live')
        self.assertEqual(m._tracking, {'t0': 'live'})

    def test_without_current_camera_actual_dispatch_keeps_idle_preference(self):
        m = self.manager()
        m._visual_evidence.stamps = {}
        scope['_dispatch_tracker'](m, 't0', 0., 0.)
        self.assertEqual(m._authorized_publish.call_args.args[0].uav_id, 'blind')

    def test_actual_takeover_debounces_original_frames_then_only_requests_old_stop(self):
        m = self.manager()
        m._tracking['t0'] = 'owner'
        m.status['owner'].connected = True
        m._visual_evidence.stamps[('owner', 'green')] = 8.
        for stamp, now in ((9.9, 10.), (10.3, 10.4)):
            m._visual_evidence.stamps[('live', 'green')] = stamp
            scope['_consider_camera_takeover'](m, 't0', now)
            m._authority.withdraw.assert_not_called()
        m._visual_evidence.stamps[('live', 'green')] = 10.7
        scope['_consider_camera_takeover'](m, 't0', 10.8)
        m._authority.withdraw.assert_called_once_with('owner', 10.8)
        m._emit_authority.assert_called_once_with(['stop'])
        m._authorized_publish.assert_not_called()
        self.assertNotIn('t0', m._tracking)

    def test_owner_recovery_and_busy_candidate_prevent_takeover(self):
        m = self.manager()
        m._tracking['t0'] = 'owner'
        m._visual_evidence.stamps[('owner', 'green')] = 9.9
        scope['_consider_camera_takeover'](m, 't0', 10.)
        m._authority.withdraw.assert_not_called()
        m._visual_evidence.stamps[('owner', 'green')] = 8.
        m._tracking['t1'] = 'live'
        scope['_consider_camera_takeover'](m, 't0', 10.)
        m._authority.withdraw.assert_not_called()

    def test_takeover_debounce_remains_possible_after_observer_crosses_old_fifteen_meter_margin(self):
        m = self.manager()
        m._tracking['t0'] = 'owner'
        m._visual_evidence.stamps[('owner', 'green')] = 8.
        for stamp, now, distance in ((9.9, 10., 14.5), (10.3, 10.4, 15.4), (10.7, 10.8, 15.8)):
            m.status['live'].x = distance
            m._visual_evidence.stamps[('live', 'green')] = stamp
            scope['_consider_camera_takeover'](m, 't0', now)
        m._authority.withdraw.assert_called_once_with('owner', 10.8)
