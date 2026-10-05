"""Own-pose drift, opposite velocity, stopped ACK, and actual guard adapters."""
import ast
from collections import deque
import json
import math
from pathlib import Path
import sys
import threading
from types import MethodType, SimpleNamespace
import unittest

SCRIPTS = Path(__file__).parents[1]/'src/robocup_swarm/scripts'
sys.path.insert(0, str(SCRIPTS))
from pose_rate import PoseRate, motion_evidence
from pose_quality import PoseQuality
from radar_observed_map import ObservedMap
from radar_velocity_guard import guard_velocity


def rate(xyz_velocity, end=10., epoch=(0., 0.)):
    result = PoseRate()
    for k in range(16):
        dt = k*.02
        result.add(tuple(v*dt for v in xyz_velocity), end-.3+dt, epoch)
    return result


class RateTests(unittest.TestCase):
    def test_pose_differences_keep_direction_and_vertical_motion(self):
        r = rate((.4, -.2, .3))
        estimate = r.estimate(10.1, (0., 0.))
        for a, b in zip(estimate['velocity_xyz'], (.4, -.2, .3)):
            self.assertAlmostEqual(a, b)
        self.assertAlmostEqual(estimate['sample_s']-estimate['window_start_s'], .3)

    def test_duplicate_headers_do_not_refresh_and_short_span_is_not_speed(self):
        r = PoseRate()
        r.add((0., 0., 0.), 10., 'a')
        r.add((1., 0., 0.), 10.1, 'a')
        self.assertIsNone(r.estimate(10.1, 'a'))
        r = rate((.4, 0., 0.))
        self.assertFalse(r.add((100., 0., 0.), 10., (0., 0.)))
        self.assertIsNone(r.estimate(10.51, (0., 0.)))

    def test_future_old_epoch_and_clock_reversal_cannot_reuse_rate(self):
        r = rate((.4, 0., 0.))
        self.assertIsNone(r.estimate(9.9, (0., 0.)))
        self.assertIsNone(r.estimate(10., (1., 0.)))
        r.add((0., 0., 0.), 10.1, (1., 0.))
        self.assertIsNone(r.estimate(10.1, (1., 0.)))
        self.assertFalse(r.add((0., 0., 0.), 9., (1., 0.)))
        self.assertIsNone(r.estimate(10.1, (1., 0.)))

    def test_invalid_pose_and_gap_never_retime_an_old_estimate(self):
        r = rate((.4, 0., 0.))
        self.assertFalse(r.add((math.nan, 0., 0.), 10.2, (0., 0.)))
        self.assertEqual(r.estimate(10.2, (0., 0.))['sample_s'], 10.)
        r.add((1., 0., 0.), 10.6, (0., 0.))
        self.assertIsNone(r.estimate(10.6, (0., 0.)))

    def test_opposing_sources_never_average_into_a_false_stop(self):
        r = rate((.4, 0., .3))
        e = motion_evidence((-.05, 0., 10.), 0., r.estimate(10.1, (0., 0.)), 10.1)
        self.assertIn((-.05, 0.), e['velocity_candidates'])
        self.assertGreater(e['selected_xy'][0], .39)
        self.assertAlmostEqual(e['speed_mps'], .5)
        self.assertEqual(e['selected_source'], 'pose_rate')
        self.assertAlmostEqual(e['sample_s'], 10.)

    def test_fast_twist_is_retained_even_with_slow_pose_rate(self):
        e = motion_evidence((-2., 0., 10.), 0., rate((.4, 0., 0.)).estimate(10., (0., 0.)), 10.)
        self.assertEqual(e['selected_xy'], (-2., 0.))
        self.assertEqual(e['speed_mps'], 2.)
        self.assertEqual(len(e['velocity_candidates']), 2)

    def test_missing_required_pose_or_either_stale_source_is_unknown(self):
        self.assertIsNone(motion_evidence((0., 0., 10.), 0., None, 10.))
        estimate = rate((0., 0., 0.)).estimate(10., (0., 0.))
        self.assertIsNone(motion_evidence((0., 0., 9.), 0., estimate, 10.))
        self.assertIsNone(motion_evidence((0., 0., 10.6), 0., estimate, 10.6))
        self.assertIsNotNone(motion_evidence((0., 0., 10.), 0., None, 10., require_pose=False))


class ActualAdapterTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.1
        def vector(): return SimpleNamespace(x=0., y=0., z=0.)
        def occupancy():
            return SimpleNamespace(header=SimpleNamespace(),
                info=SimpleNamespace(origin=SimpleNamespace(position=vector(),orientation=SimpleNamespace(w=0.))))
        ros = SimpleNamespace(Time=SimpleNamespace(
            now=lambda: SimpleNamespace(to_sec=lambda:self.now), from_sec=lambda t:t),
            logwarn_throttle=lambda *a:None,logerr_throttle=lambda *a:None)
        scope = dict(math=math,json=json,rospy=ros,String=lambda **kw:SimpleNamespace(**kw),
            motion_evidence=motion_evidence,guard_velocity=guard_velocity,OccupancyGrid=occupancy,
            RADAR_GUARD=1,RADAR_FRESH_S=.5,RADAR_STOP_R=1.2,RADAR_LATENCY_S=.5,RADAR_BRAKE_MPS2=.5)
        tree = ast.parse((SCRIPTS/'swarm_agent.py').read_text(encoding='utf-8'))
        names = {'_measured_motion','_radar_guard_velocity','_online_velocity_clear','_route_velocity_clear',
                 '_publish_authority_state','_publish_motion','_report_blocked_plan','_local_cb','_scan_cb'}
        methods = [n for c in tree.body if isinstance(c,ast.ClassDef) and c.name=='SwarmAgent'
                   for n in c.body if isinstance(n,ast.FunctionDef) and n.name in names]
        exec(compile(ast.fix_missing_locations(ast.Module(body=methods,type_ignores=[])),str(SCRIPTS/'swarm_agent.py'),'exec'),scope)
        self.methods = {name:scope[name] for name in names}

    def agent(self, xyz_velocity=(.4, 0., 0.), twist=(-.05, 0., 10.)):
        a = SimpleNamespace(_pose_rate_enabled=True,_pose_rate=rate(xyz_velocity),
            _velocity_sample=twist,_velocity_z=0.,offset=(0.,0.),world_xy=(0.,0.),
            local_xy=(0.,0.),local_z=2.2,_local_prev_t=10.,_pose_sample_s=10.,yaw=0.,uav_id='uav_6',
            _gate=SimpleNamespace(stopping=True,run_id='r',generation=1,can_move=lambda now:True),
            _stopped_since=None,_ack_seq=0,_authority_lock=threading.RLock(),
            _authority_ack_pub=SimpleNamespace(publish=lambda msg:None),
            _blocked_plan=None,_blocked_feedback_seq=0,_navigation_pub=SimpleNamespace(publish=lambda msg:None))
        for name, f in self.methods.items(): setattr(a,name,MethodType(f,a))
        return a

    def test_actual_radar_checks_pose_direction_when_twist_points_away(self):
        a = self.agent()
        ranges = [math.inf]*360
        ranges[180] = 1.3
        a._scan = SimpleNamespace(ranges=ranges,angle_min=-math.pi,angle_increment=math.pi/180,
            range_min=.5,range_max=20.,header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:10.)))
        old = guard_velocity((-.1,0.),a._velocity_sample[:2],0.,ranges,-math.pi,math.pi/180,.5,20.,10.,10.1)
        self.assertEqual(old['velocity_xy'],(-.1,0.))
        self.assertEqual(a._radar_guard_velocity(-.1,0.),(0.,0.))
        self.assertEqual(a._final_stop_reason,'RADAR_BRAKING_REQUIRED')

    def test_actual_online_and_route_check_both_candidates(self):
        a = self.agent()
        calls = []
        def local(observed,position,velocity,now,epoch):
            calls.append(velocity); return velocity[0] < .2
        a._online_planner=SimpleNamespace(local_command_clear=local)
        a._online_map=None; a._online_map_lock=threading.RLock(); a._online_map_epoch_s=1.
        self.assertFalse(a._online_velocity_clear(-.1,0.))
        self.assertGreater(calls[-1][0],.39)
        calls.clear()
        a._route_gate=SimpleNamespace(command_clear=lambda p,r,v,n,g: calls.append(v) is None and v[0]<.2)
        self.assertFalse(a._route_velocity_clear(-.1,0.,10.1))
        self.assertEqual(len(calls),2)
        self.assertGreater(calls[-1][0],.39)

    def test_vertical_pose_drift_cannot_ack_stop_or_claim_stationary_block(self):
        a = self.agent((0.,0.,.4),(0.,0.,10.))
        messages=[]; a._authority_ack_pub.publish=lambda msg:messages.append(json.loads(msg.data))
        a._stopped_since=8.
        a._publish_authority_state()
        self.assertEqual(messages[-1]['status'],'STATE')
        self.assertAlmostEqual(messages[-1]['speed_mps'],.4)
        self.assertEqual(messages[-1]['stopped_s'],0.)
        a._blocked_plan=(3.,(0.,0.),1)
        a._report_blocked_plan('NO_REACHABLE_PROGRESS',10.1)
        self.assertIsNone(a._blocked_plan)
        self.assertEqual(a._blocked_feedback_seq,0)

    def test_real_ack_requires_continuous_fresh_three_dimensional_stop(self):
        a=self.agent((0.,0.,0.),(0.,0.,10.))
        messages=[]; a._authority_ack_pub.publish=lambda msg:messages.append(json.loads(msg.data))
        for k in range(12):
            self.now=10.1+k*.1
            a._pose_rate=rate((0.,0.,0.),self.now)
            a._velocity_sample=(0.,0.,self.now); a._pose_sample_s=self.now
            a._publish_authority_state()
        self.assertEqual(messages[-1]['status'],'STOPPED')
        self.assertGreaterEqual(messages[-1]['stopped_s'],1.)
        before=len(messages)
        a.offset=(1.,0.)
        a._publish_authority_state()
        self.assertEqual(len(messages),before)
        self.assertIsNone(a._stopped_since)

    def test_motion_transport_keeps_schema_and_larger_speed(self):
        a=self.agent()
        messages=[]
        a._motion_seq=0
        a._motion_pub=SimpleNamespace(publish=lambda msg:messages.append(json.loads(msg.data)))
        a._motion_cache=SimpleNamespace(run_id='r',receive=lambda m,n:True)
        a._publish_motion()
        self.assertEqual(set(messages[-1]),{'schema_version','run_id','uav_id','seq','sample_s','frame','position_xy','velocity_xy'})
        self.assertGreater(messages[-1]['velocity_xy'][0],.39)
        a._pose_rate_enabled=True; a._pose_rate=PoseRate()
        a._publish_motion()
        self.assertEqual(len(messages),1)

    def test_quarantined_pose_never_updates_rate_or_recovers_itself(self):
        a=self.agent()
        a._pose_quality=PoseQuality(); a._pose_quality.accept((0.,0.,2.2),10.)
        a._anchor_done=True
        msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:10.1)),
            pose=SimpleNamespace(position=SimpleNamespace(x=20.,y=0.,z=2.2)))
        a._local_cb(msg)
        self.assertEqual(a._pose_quality.fault,'IMPLAUSIBLE_POSE_JUMP')
        self.assertEqual(a._pose_rate.estimate(10.1,a.offset)['sample_s'],10.)
        self.assertIsNone(a._measured_motion(10.1))

    def test_scan_log_preserves_nonfinite_types_and_pose_pairing(self):
        a=self.agent(); a._online_map=ObservedMap(40,40,.25,(-5.,-5.))
        a._online_map_lock=threading.RLock(); a._online_map_offset=a.offset; a._online_map_epoch_s=1.
        a._online_planner=None; a._scan_window=deque(maxlen=6)
        a._map_pub=SimpleNamespace(publish=lambda msg:None)
        a.grid=SimpleNamespace(width=40,height=40,resolution=.25,origin=(-5.,-5.))
        a._orientation_xyzw=(0.,0.,0.,1.)
        ranges=[math.inf]*360; ranges[0]=math.nan; ranges[1]=-math.inf; ranges[2]=2.
        scan=SimpleNamespace(ranges=ranges,angle_min=-math.pi,angle_increment=math.pi/180,
            range_min=.5,range_max=20.,header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:10.)))
        a._scan_cb(scan)
        record=a._scan_window[-1]
        self.assertEqual(record['pose_s'],10.)
        self.assertEqual(record['ranges'][:3],[None,None,2.])
        self.assertIn([0,'nan'],record['nonfinite_ranges'])
        self.assertIn([1,'negative_infinity'],record['nonfinite_ranges'])
        self.assertIn([3,'positive_infinity'],record['nonfinite_ranges'])
        json.dumps(record,allow_nan=False)


if __name__ == '__main__':
    unittest.main()
