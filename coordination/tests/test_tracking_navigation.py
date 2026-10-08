import ast
import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT/'coordination/src/robocup_swarm/scripts'
sys.path.insert(0, str(SCRIPTS))
from tracking_navigation import TrackingNavigation
NS = types.SimpleNamespace


class TrackingNavigationTests(unittest.TestCase):
    def test_current_image_and_bounded_old_image_have_distinct_phases(self):
        nav = TrackingNavigation()
        self.assertEqual(nav.step(1,'t3',100.2,(30.,0.),(0.,0.),100.)[0], 'CURRENT')
        phase, point = nav.step(1,'t3',102.,(30.,0.),(0.,0.),100.)
        self.assertEqual(phase, 'APPROACH')
        self.assertEqual(point, (10.,0.))
        self.assertEqual(nav.step(1,'t3',108.1,(30.,0.),(0.,0.),100.)[0], 'FAILED')

    def test_same_old_image_never_extends_window(self):
        nav = TrackingNavigation()
        for now in (100., 102., 104., 106., 108.):
            self.assertNotEqual(nav.step(1,'t0',now,(30.,0.),(0.,0.),100.)[0], 'FAILED')
        self.assertEqual(nav.step(1,'t0',108.01,(30.,0.),(0.,0.),100.)[0], 'FAILED')

    def test_scan_cannot_extend_image_window(self):
        nav = TrackingNavigation()
        self.assertEqual(nav.step(1,'t0',107.8,(10.,0.),(0.,0.),100.)[0], 'REACQUIRE')
        self.assertEqual(nav.step(1,'t0',108.1,(10.,0.),(0.,0.),100.)[0], 'FAILED')

    def test_one_scan_and_no_late_recovery_after_failure(self):
        nav = TrackingNavigation()
        self.assertEqual(nav.step(1,'t0',102.,(10.,0.),(0.,0.),100.)[0], 'REACQUIRE')
        self.assertEqual(nav.step(1,'t0',102.2,(10.,0.),(0.,0.),102.1)[0], 'CURRENT')
        self.assertEqual(nav.step(1,'t0',104.,(10.,0.),(0.,0.),102.1)[0], 'FAILED')
        self.assertEqual(nav.step(1,'t0',104.2,(10.,0.),(0.,0.),104.1)[0], 'FAILED')
        self.assertEqual(nav.step(2,'t0',104.2,(10.,0.),(0.,0.),104.1)[0], 'CURRENT')

    def test_wait_for_actual_visual_callback_but_not_forever(self):
        nav = TrackingNavigation()
        self.assertEqual(nav.step(1,'t0',100.,(30.,0.),None,None)[0], 'WAIT')
        self.assertEqual(nav.step(1,'t0',100.2,(30.,0.),(0.,0.),100.1)[0], 'CURRENT')
        nav = TrackingNavigation()
        nav.step(1,'t0',100.,(30.,0.),None,None)
        self.assertEqual(nav.step(1,'t0',108.1,(30.,0.),None,None)[0], 'FAILED')

    def test_stop_or_expired_authority_cannot_move(self):
        nav = TrackingNavigation()
        self.assertEqual(nav.step(1,'t0',102.,(30.,0.),(0.,0.),100.,eligible=False), ('IDLE',None))

    def test_white_used_opportunity_is_not_reopened(self):
        nav = TrackingNavigation()
        self.assertEqual(nav.step(1,'t3',102.,(10.,0.),(0.,0.),100.,allow_scan=False)[0], 'FAILED')

    def test_bad_and_future_coordinates_rejected(self):
        for now, point, stamp in ((100.,(0.,0.),101.),(100.,(float('nan'),0.),99.)):
            self.assertEqual(TrackingNavigation().step(1,'t0',now,(30.,0.),point,stamp)[0], 'FAILED')

    def test_actual_manager_cache_uses_original_timestamp_and_coordinates(self):
        source = (SCRIPTS/'swarm_manager.py').read_text(encoding='utf-8')
        node = next(n for n in ast.walk(ast.parse(source)) if isinstance(n,ast.FunctionDef)
                    and n.name == '_get_target_pos')
        from tracker_selection import held_position
        scope = dict(SWARM_TARGET_HOLD_S=8., TRUTH_TTL=8., held_position=held_position)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),
                     'swarm_manager.py','exec'), scope)
        original = NS(t=100., x=3., y=4., los=True)
        ct = NS(last_obs={'uav_1':original}, obs_ttl=1.,
                fused=lambda now:(9.,9.) if now <= 101. else None)
        manager = NS(_held_pos={}, _truth_cache={}, _truth_pos={}, tracker=NS(targets={'t0':ct}))
        scope['_get_target_pos'](manager,'t0',100.2)
        scope['_get_target_pos'](manager,'t0',100.8)
        self.assertEqual(manager._held_pos['t0'], (3.,4.,100.))
        self.assertEqual(scope['_get_target_pos'](manager,'t0',102.), (3.,4.))
        self.assertEqual(scope['_get_target_pos'](manager,'t0',108.01), (None,None))

    def test_actual_agent_pending_plan_stops_and_expiry_reports(self):
        source = (SCRIPTS/'swarm_agent.py').read_text(encoding='utf-8')
        node = next(n for n in ast.walk(ast.parse(source)) if isinstance(n,ast.FunctionDef)
                    and n.name == '_control_tracking_navigation')
        from own_visual_guidance import tracking_input
        scope = dict(tracking_input=tracking_input,TAG_TO_TID={'white':'t3'}, POS_KP=.8, MAX_SPEED=3., FLEE_CHASE_SPEED=2.6,
                     math=math, rospy=NS(loginfo=lambda *args:None))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),
                     'swarm_agent.py','exec'), scope)
        commands, requests, failures = [], [], []
        agent = NS(_tracking_navigation=TrackingNavigation(), _tracking_navigation_phase=None,
            targets={'t0':(0.,0.,0.,0.)}, _t_seen={'t0':100.}, world_xy=(30.,0.),
            _gate=NS(generation=1,can_move=lambda now:True), _white_reacquire=None,
            _blocked_plan=None, _plan_fail_t=0., uav_id='uav_1',
            _need_replan_track=lambda point:True, _request_plan=lambda point:requests.append(point),
            _pick_local_goal=lambda:None, _send_vel=lambda x,y:commands.append((x,y)),
            _report_blocked_plan=lambda reason,now:failures.append(reason))
        method=scope['_control_tracking_navigation']
        self.assertTrue(method(agent,102.,'t0',False))
        self.assertEqual(requests,[(10.,0.)])
        self.assertEqual(commands,[(0.,0.)])
        agent.path_target = (0.,0.)
        agent._pick_local_goal = lambda:(18.,0.)
        method(agent,102.2,'t0',False)
        self.assertEqual(commands[-1],(0.,0.))  # Old route still ends at the person.
        agent.path_target = (10.,0.)
        agent._apply_friend_avoidance = lambda vx,vy:(vx,vy)
        method(agent,103.,'t0',False)
        self.assertAlmostEqual(commands[-1][0],-2.6)
        self.assertEqual(commands[-1][1],0.)
        method(agent,108.1,'t0',False)
        self.assertEqual(failures,['TARGET_VISUAL_LOST'])
        self.assertEqual(agent._t_seen['t0'],100.)


if __name__ == '__main__':
    unittest.main()
