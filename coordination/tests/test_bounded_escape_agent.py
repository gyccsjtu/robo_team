"""Actual agent computation, offer/grant and control methods on measured fixtures."""
import ast
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace, MethodType
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/robocup_swarm/scripts'))
sys.path.insert(0,str(ROOT/'src/robocup_navigation/src'))
from bounded_escape import RestEvidence
from online_radar_planner import OnlinePlanner
from radar_observed_map import ObservedMap
from route_reservation import RouteGate, exclude_peers
from fleet_motion_guard import MotionCache
from position_brake import PositionBrake


class EscapeAgentTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.commands, self.offers, self.radar_inputs = [], [], []
        vector = lambda: SimpleNamespace(x=0.,y=0.,z=0.)
        twist = lambda: SimpleNamespace(twist=SimpleNamespace(linear=vector(),angular=vector()))
        stamp = lambda: SimpleNamespace(to_sec=lambda:self.now)
        rospy = SimpleNamespace(Time=SimpleNamespace(now=stamp),
            loginfo=lambda *a:None,logwarn_throttle=lambda *a:None,
            logerr=lambda *a:None,logerr_throttle=lambda *a:None)
        scope = dict(math=math,json=json,OnlinePlanner=OnlinePlanner,
            ObservedMap=ObservedMap,MotionCache=MotionCache,exclude_peers=exclude_peers,
            rospy=rospy,TwistStamped=twist,String=lambda **kw:SimpleNamespace(**kw),
            PositionTarget=lambda:SimpleNamespace(header=SimpleNamespace(),position=vector(),velocity=vector()),
            POS_KP=.8,LOOKAHEAD=4.,GRID_GUARD=1,MAX_ACC=2.5,CTRL_RATE=20.,
            ALT_PANIC=5.7,ALT_PANIC_HSCALE=.3,ALT_TARGET_CAP=4.5,ALT_HARD_CEIL=5.5,
            ALT_HARD_MARGIN=.1,MIN_CRUISE_ALT=2.,ALT_P=1.,ALT_VZ_MIN=.2,
            ALT_EMERG_CEIL=5.9,CRASH_DETECT=False)
        tree = ast.parse((ROOT/'src/robocup_swarm/scripts/swarm_agent.py').read_text())
        names = {'_update_escape_rest','_cancel_escape','_current_escape','_escape_path_valid',
            '_escape_limit_velocity','_control_escape','_compute_plan','_planner_loop',
            '_route_grant_cb','_request_plan','_pick_local_goal','_online_velocity_clear',
            '_route_guard_velocity','_route_velocity_clear','_grid_guard_velocity',
            '_send_vel','_publish_command','_authorized_cb'}
        names.add('_body_proof_diagnostic')
        methods = [n for c in tree.body if isinstance(c,ast.ClassDef)
                   for n in c.body if isinstance(n,ast.FunctionDef) and n.name in names]
        exec(compile(ast.fix_missing_locations(ast.Module(body=methods,type_ignores=[])),
                     'actual_escape_agent_methods','exec'),scope)
        self.scope = scope
        observed = ObservedMap(40,40,.25,(0.,0.))
        observed.cells = [0]*1600
        observed.observed_s = [11.]*1600
        observed.last_scan_s = 11.
        observed.version = 3
        observed.cells[20*40+25] = 100
        gate = SimpleNamespace(run_id='run',generation=2,task={'task_type':0},stopping=False)
        gate.can_move = lambda now:not gate.stopping
        motion = MotionCache('run',['uav_1'])
        motion.receive(dict(schema_version=1,run_id='run',uav_id='uav_1',seq=1,
            sample_s=11.,frame='world_enu_xy',position_xy=[5.125,5.125],velocity_xy=[0.,0.]),11.)
        a = self.a = SimpleNamespace(uav_id='uav_1',_gate=gate,_authority_lock=threading.RLock(),
            _plan_lock=threading.RLock(),_online_map_lock=threading.RLock(),
            _plan_event=threading.Event(),_plan_ticket=7,_plan_pending=None,_plan_inflight=None,
            _route_pending=None,_route_pending_s=0.,_route_offer_id=0,_route_gate=RouteGate('run','uav_1'),
            _escape_enabled=True,_escape_rest=RestEvidence(),_escape_ready=False,
            _escape_active=None,_escape_pending=None,_escape_used=set(),_computed_escape=None,
            _pose_quality=SimpleNamespace(usable=lambda now:True),_pose_rate_enabled=True,
            _pose_sample_s=10.,_takeoff_done=True,_landing=False,_last_flight_v=(0.,0.),
            _last_cmd_v=(0.,0.),world_xy=(5.125,5.125),local_xy=(0.,0.),local_z=2.2,
            offset=(5.125,5.125),_offset_param=(5.125,5.125),_online_map_epoch_s=1,
            _online_map=observed,_online_planner=OnlinePlanner((5.125,5.125)),
            _online_safe_grid=None,_peer_routes={},_peer_routes_s=11.,_motion_cache=motion,
            _route_offer_pub=SimpleNamespace(publish=self.offers.append),path=[],path_target=None,
            _report_blocked_plan=lambda *a:None,_position_brake=PositionBrake(),
            vel_pub=SimpleNamespace(publish=self.commands.append),_last_csv_t=11.,
            altitude_layer=2.2,_compute_desired_altitude=lambda:0.,_look_at=None,
            _bounds_recovery_velocity=lambda:None,_map_guard_velocity=lambda x,y:(x,y),
            _friend_guard_velocity=lambda x,y:(x,y),_apply_friend_avoidance=lambda x,y:(x,y))
        for method in methods:
            setattr(a,method.name,MethodType(scope[method.name],a))
        a._measured_motion = lambda now:dict(speed_mps=0.,sample_s=a._pose_sample_s,
            velocity_candidates=[(0.,0.)],velocity_local_sample=[0.,0.,0.,a._pose_sample_s],
            pose_rate_sample=[0.,0.,0.,a._pose_sample_s])
        def radar(x,y):
            self.radar_inputs.append((x,y))
            return x,y
        a._radar_guard_velocity = radar
        for time in (10.,10.25,10.5,10.75,11.):
            self.now = a._pose_sample_s = time
            a._update_escape_rest(time)

    def offer(self):
        a = self.a
        a._request_plan((8.,5.125))
        shutdown = iter((False,True))
        self.scope['rospy'].is_shutdown = lambda:next(shutdown)
        a._planner_loop()
        self.assertEqual(len(self.offers),1)
        return json.loads(self.offers[0].data)

    def commit(self):
        message = self.offer()
        message.update(seq=1,expires_s=12.)
        self.a._route_grant_cb(SimpleNamespace(data=json.dumps(message)))
        self.assertIsNotNone(self.a._escape_active)
        return message

    def test_actual_worker_offers_real_start_but_only_grant_enables_exit(self):
        offer = self.offer()
        a = self.a
        self.assertEqual(offer['points'][0],list(a.world_xy))
        self.assertIsNone(a._escape_active)
        self.assertEqual(a._escape_used,set())
        self.assertFalse(a._online_velocity_clear(-.1,0.))
        offer.update(seq=1,expires_s=12.)
        a._route_grant_cb(SimpleNamespace(data=json.dumps(offer)))
        self.assertIsNotNone(a._escape_active)
        self.assertEqual(a._escape_used,{2})
        self.assertTrue(a._online_velocity_clear(-.1,0.))
        self.assertEqual(a._online_planner.radius,1.2)

    def test_late_or_mismatched_grant_does_not_consume_exit(self):
        for change in ('expired','generation','points','rest_lost','pose_drift'):
            self.setUp()
            offer = self.offer()
            offer.update(seq=1,expires_s=12.)
            if change == 'expired':offer['expires_s'] = 11.
            elif change == 'generation':offer['generation'] = 3
            elif change == 'points':offer['points'][-1][0] -= .25
            elif change == 'rest_lost':self.a._last_flight_v = (.2,0.)
            else:self.a.world_xy = (5.,5.125)  # Connecting back toward the hit is rejected.
            self.a._route_grant_cb(SimpleNamespace(data=json.dumps(offer)))
            self.assertIsNone(self.a._escape_active)
            self.assertEqual(self.a._escape_used,set())

    def test_final_command_caps_peer_change_and_holds_entry_height(self):
        self.commit()
        a = self.a
        a._friend_guard_velocity = lambda x,y:(-3.,0.)
        a.local_z = 2.25
        a._send_vel(-3.,0.)
        output = self.commands[-1]
        self.assertLessEqual(math.hypot(output.velocity.x,output.velocity.y),.3)
        self.assertLess(output.velocity.x,0.)
        self.assertEqual(output.type_mask,1507)
        self.assertEqual(output.position.z,2.2)
        self.assertTrue(self.radar_inputs)
        # Friend steering toward the blocking hit cannot bypass final gates.
        a._friend_guard_velocity = lambda x,y:(3.,0.)
        a._send_vel(-.1,0.)
        self.assertEqual(self.commands[-1].type_mask,1528)
        self.assertEqual(self.commands[-1].velocity.x,0.)
        self.assertEqual(self.commands[-1].position.z,2.2)

    def test_stop_callback_expiry_epoch_and_pose_loss_cancel_actual_exit(self):
        for reason in ('STOP','expiry','epoch','pose','generation','timeout'):
            self.setUp()
            self.commit()
            a = self.a
            if reason == 'STOP':
                def receive(message,now):
                    a._gate.stopping = True
                    return True
                a._gate.receive = receive
                a._authorized_cb(SimpleNamespace(data='{}'))
            elif reason == 'expiry':self.now = 12.
            elif reason == 'epoch':a._online_map_epoch_s = 2
            elif reason == 'pose':a._pose_quality.usable = lambda now:False
            elif reason == 'generation':a._gate.generation = 3
            else:
                self.now = a._pose_sample_s = 27.
                a._route_gate.record['expires_s'] = 30.
            self.assertIsNone(a._current_escape(self.now))
            self.assertEqual(a.path,[])
            self.assertEqual(a._escape_used,{2})

    def test_active_exit_finishes_before_search_replanning_and_only_once(self):
        self.commit()
        a = self.a
        ticket = a._plan_ticket
        a._request_plan((9.,6.))
        self.assertEqual(a._plan_ticket,ticket)
        a.world_xy = tuple(a.path[-1])
        self.assertTrue(a._control_escape(11.))
        self.assertIsNone(a._escape_active)
        self.assertEqual(a.path,[])
        # Put the real pose back inside the obstructed start, update fresh rest:
        # same generation cannot issue another narrow-radius proposal.
        a.world_xy = (5.125,5.125)
        a._compute_plan((8.,5.125))
        self.assertIsNone(a._computed_escape)

    def test_explicit_vertical_action_wins_over_escape_height(self):
        self.commit()
        self.a._send_vel(-.1,0.,vz=-.8)
        output = self.commands[-1]
        self.assertEqual(output.type_mask,1479)
        self.assertEqual(output.velocity.z,-.8)

    def test_measured_travel_budget_cannot_be_reset_by_grant_renewal(self):
        message = self.commit()
        a = self.a
        for time in (11.1,11.2,11.3,11.4):
            self.now = a._pose_sample_s = time
            a.world_xy = (a.world_xy[0]-.5,a.world_xy[1])
            self.assertIsNotNone(a._current_escape(time))
        self.assertEqual(a._escape_active['travel_m'],2.)
        message.update(seq=2,expires_s=13.)
        a._route_grant_cb(SimpleNamespace(data=json.dumps(message)))
        self.assertEqual(a._escape_active['travel_m'],2.)
        self.now = a._pose_sample_s = 11.5
        a.world_xy = (a.world_xy[0]-.01,a.world_xy[1])
        self.assertIsNone(a._current_escape(11.5))
        self.assertEqual(a._escape_used,{2})

    def test_fresh_hit_overrides_committed_narrow_corridor(self):
        self.commit()
        a = self.a
        a._online_map.cells[20*40+20] = 100
        a._online_map.version += 1
        self.assertFalse(a._online_velocity_clear(-.1,0.))
        a._send_vel(-.1,0.)
        self.assertEqual(self.commands[-1].velocity.x,0.)
        self.assertEqual(self.commands[-1].type_mask,1528)

    def test_missing_last_command_cannot_claim_measured_rest(self):
        self.a._last_flight_v = None
        self.assertFalse(self.a._update_escape_rest(11.))

    def test_diagnostic_records_both_actual_proofs_without_mutating_clearance(self):
        self.commit()
        a = self.a
        normal = set(a._online_planner.body_proof)
        recovery = set(a._escape_active['planner'].body_proof)
        output = a._body_proof_diagnostic()
        self.assertEqual(set(output['normal']['indices']),normal)
        self.assertEqual(set(output['escape']['indices']),recovery)
        self.assertEqual(output['normal']['radius_m'],1.2)
        self.assertEqual(output['escape']['radius_m'],.9)
        json.dumps(output,allow_nan=False)
        output['normal']['indices'].clear()
        output['escape']['indices'].append(1599)
        self.assertEqual(a._online_planner.body_proof,normal)
        self.assertEqual(a._escape_active['planner'].body_proof,recovery)


if __name__ == '__main__':
    unittest.main()
