"""Recorded 2073 -> 2080 early-green dispatch must retain one bounded re-look."""
import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                     'coordination/src/robocup_swarm/scripts'))
import swarm_manager as sm
import yolo_target_bridge as bridge
from robocup_swarm.msg import TargetState
from cooperative_tracker import CooperativeTracker


class EarlyBridgeReacquireTests(unittest.TestCase):
    def setUp(self):
        self.now = 2073.436
        clock = patch.object(sm.rospy.Time, 'now',
                             side_effect=lambda:sm.rospy.Time.from_sec(self.now))
        clock.start();self.addCleanup(clock.stop)
        self.m = sm.SwarmManager.__new__(sm.SwarmManager)
        m = self.m
        m._quarantined_visual = lambda *args:False
        m._hypothesis_sample_current = lambda *args:True
        m._truth_cache, m._visual_candidate = {}, {}
        m._reacquire_sites, m._visual_source = {}, {}
        m._eligible_observations, m._visual_observations = {}, {}
        m._last_detect, m._last_detection_sample = {}, {}
        m.tracker = CooperativeTracker(official_only=True, strict_samples=True)
        m.tracker.report = Mock(wraps=m.tracker.report)
        m._blocked_cells = set()
        m.allocator = NS(revisit_jobs={})
        def queue(kind, x, y, now, ttl_s, blocked):
            key = (26,12)
            m.allocator.revisit_jobs[(kind,key)] = (now, now+max(1.,ttl_s))
            return key
        m.allocator.queue_revisit = Mock(side_effect=queue)

    def msg(self, sample=2073.328, **changes):
        m = TargetState()
        m.header.stamp = sm.rospy.Time.from_sec(sample)
        m.header.frame_id = 'map'
        m._connection_header = {'callerid':'/yolo_target_bridge'}
        m.target_id, m.x, m.y = 't0', 129.87, 22.03
        m.state, m.eliminated = 0, False
        for k,v in changes.items():setattr(m,k,v)
        return m

    def test_exact_early_green_cache_expiry_queues_remaining_budget(self):
        m=self.m;m._truth_cb(self.msg())
        self.now=2080.784
        self.assertEqual(m._get_target_pos('t0',self.now),(None,None))
        self.assertNotIn('t0',m._truth_cache)
        m._queue_lost_revisit('t0',self.now)
        call=m.allocator.queue_revisit.call_args
        self.assertEqual(call.args,('lost',129.87,22.03))
        self.assertAlmostEqual(call.kwargs['ttl_s'],17.544,places=6)
        self.assertAlmostEqual(m.allocator.revisit_jobs[('lost',(26,12))][1],2098.328,places=6)
        self.assertEqual(m._get_target_pos('t0',self.now),(None,None))
        m.tracker.report.assert_not_called()
        self.assertEqual(m._eligible_observations,{})
        self.assertEqual(m._visual_observations,{})
        self.assertEqual(m._last_detect,{})

    def test_original_deadline_is_not_extended_by_duplicate_keepalive_or_loss(self):
        m=self.m;m._truth_cb(self.msg())
        expiry=m._reacquire_sites['t0']['expires_s']
        self.now+=.5;m._truth_cb(self.msg(x=131.))
        self.assertEqual(m._reacquire_sites['t0']['expires_s'],expiry)
        self.assertEqual(m._reacquire_sites['t0']['x'],129.87)
        m._queue_lost_revisit('t0',2080.784)
        m._queue_lost_revisit('t0',2088.)
        self.assertEqual(m.allocator.queue_revisit.call_count,1)
        m._queue_lost_revisit('t0',2098.328)
        self.assertNotIn('t0',m._reacquire_sites)
        self.assertEqual(m.allocator.revisit_jobs,{})
        self.now=2098.5;m._truth_cb(self.msg())
        self.assertNotIn('t0',m._reacquire_sites)

    def test_new_real_sample_moves_deadline_only_by_capture_time(self):
        m=self.m;m._truth_cb(self.msg());m._queue_lost_revisit('t0',2073.8)
        self.now=2074.
        m._truth_cb(self.msg(sample=2073.8,x=130.))
        self.assertEqual(m.allocator.revisit_jobs,{})
        self.assertAlmostEqual(m._reacquire_sites['t0']['expires_s'],2098.8)
        self.assertEqual(m._reacquire_sites['t0']['x'],130.)

    def test_prediction_and_official_backends_do_not_create_revisit_evidence(self):
        for changes in (dict(state=1),dict(state=2),dict(state=3),
                        dict(eliminated=True),dict(_connection_header=None),
                        dict(_connection_header={'callerid':'/target_sim_node'}),
                        dict(_connection_header={'callerid':'/score_cal'}),
                        dict(_connection_header={'callerid':'/gazebo'})):
            with self.subTest(changes=changes):
                self.m._truth_cache.clear();self.m._reacquire_sites.clear()
                self.m._truth_cb(self.msg(**changes))
                self.assertEqual(self.m._reacquire_sites,{})

    def test_invalid_geometry_frame_stale_future_and_unknown_target_fail_closed(self):
        messages=[self.msg(sample=2071.),self.msg(sample=2073.45),
                  self.msg(x=float('nan')),self.msg(y=float('inf')),
                  self.msg(target_id='actor_0')]
        wrong=self.msg();wrong.header.frame_id='world_truth';messages.append(wrong)
        for msg in messages:
            with self.subTest(msg=msg):
                self.m._truth_cache.clear();self.m._reacquire_sites.clear()
                self.m._truth_cb(msg)
                self.assertEqual(self.m._reacquire_sites,{})

    def test_nonfinite_and_unknown_inputs_do_not_enter_any_target_cache(self):
        invalid=[self.msg(x=float('nan')),self.msg(y=float('inf')),
                 self.msg(target_id='actor_0')]
        stamp=self.msg()
        stamp=NS(**{k:getattr(stamp,k) for k in ('target_id','x','y','state','eliminated')},
                 header=NS(stamp=NS(to_sec=lambda:float('nan')),frame_id='map'),
                 _connection_header={'callerid':'/yolo_target_bridge'})
        invalid.append(stamp)
        for msg in invalid:
            with self.subTest(msg=msg):
                self.m._truth_cb(msg)
                self.assertEqual(self.m._truth_cache,{})
                self.assertEqual(self.m._visual_candidate,{})
                self.assertEqual(self.m.tracker.targets,{})
                self.assertEqual(self.m._reacquire_sites,{})
        with patch.object(sm.rospy.Time,'now',return_value=NS(to_sec=lambda:float('nan'))):
            self.m._truth_cb(self.msg())
        self.assertEqual(self.m._truth_cache,{})
        self.assertEqual(self.m.tracker.targets,{})

    def test_real_bridge_early_branch_requires_camera_confirmation_and_no_extrapolation(self):
        for confirmed in (False,True):
            node=bridge._mk_bridge();node._advance(2073.412-node._now())
            # Recorded first accepted green input in logs_20261011_201918.
            row=dict(target_id='green',candidate_id='session:2',xyz=[129.87,22.03,1.25],
                     frame_id='world_enu',confidence=.787,range_m=13.18,
                     official_view_ok=True,track_confirmed=confirmed,track_hits=3,
                     appearance_status='actor',position_uncertainty_m=.88)
            node._report_cb(NS(data=json.dumps(dict(schema_version=2,
                source_uav='typhoon_h480_3',sample_s=2073.328,real_detection=True,
                candidates=[row]))))
            events=node.core.tick(2073.436)
            if not confirmed:
                self.assertEqual(events,[])
                continue
            self.assertEqual(len(events),1)
            ev=events[0]
            self.assertEqual(ev['state'],0)
            self.assertEqual((ev['x'],ev['y']),(129.87,22.03))
            self.assertIsNone(ev['official_x'])
            self.m._truth_cb(self.msg(sample=ev['sample_s'],state=ev['state'],x=ev['x'],y=ev['y']))
            self.assertIn('t0',self.m._reacquire_sites)
            self.m.tracker.report.assert_not_called()


if __name__=='__main__':unittest.main()
