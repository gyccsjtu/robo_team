"""Real callback chain: discovery can dispatch but cannot qualify judge evidence."""
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'coordination/src/robocup_swarm/scripts'))
import swarm_manager as sm
import swarm_agent as sa
from cooperative_tracker import CooperativeTracker


class DiscoveryDispatchTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        clock = patch.object(sm.rospy.Time, 'now', side_effect=lambda:sm.rospy.Time.from_sec(self.now))
        clock.start()
        self.addCleanup(clock.stop)
        m = self.m = sm.SwarmManager.__new__(sm.SwarmManager)
        m.uav_ids = ['u0','u1','u2']
        m.status = {u:NS(x=i*10., y=0., z=3., connected=True) for i,u in enumerate(m.uav_ids)}
        m.last_report = {u:sm.rospy.Time(100) for u in m.uav_ids}
        for name in ('_tracking','_backup','_release_cooldown','_visual_observations',
                     '_eligible_observations','_visual_source','_last_detection_sample',
                     '_visual_candidate','_last_detect','_visual_near_seen','_truth_cache',
                     '_cur_targets'):
            setattr(m,name,{})
        m._eliminated = set()
        m._quarantined_visual = lambda *a:False
        m._discovery_state_pub = Mock()
        m.assign_pub = Mock()
        m._free_search_lease = Mock()
        m._sanitize_dispatch_xy = lambda x,y,tid:(x,y)
        m._update_tracker_position = Mock()
        m.tracker = CooperativeTracker(official_only=True,strict_samples=True)
        m.tracker.report = Mock(wraps=m.tracker.report)

    def payload(self, **changes):
        data = dict(purpose='discovery_only', geometry_ok=True, official_view_ok=False,
                    real_detection=True, source_uav='u0', target_id='t1',
                    sample_s=self.now, candidate_id='10', hits=5, range_m=8.,
                    confidence=.8, x=8., y=0.)
        data.update(changes)
        return data

    def send(self, **changes):
        self.m._discovery_cb(NS(data=json.dumps(self.payload(**changes))))

    def test_three_people_reach_real_dispatch_with_distinct_owners(self):
        for i,tid in enumerate(('t0','t1','t2')):
            self.send(target_id=tid, candidate_id=str(i), source_uav='u%d'%i, x=4.+10*i)
        m = self.m
        self.assertEqual(set(m.tracker.targets), {'t0','t1','t2'})
        m._dispatch_target_batch([(tid,*m._get_target_pos(tid,100.)) for tid in m.tracker.targets],100.)
        self.assertEqual(len(set(m._tracking.values())),3)
        self.assertEqual(m._discovery_state_pub.publish.call_count,3)
        m.tracker.report.assert_not_called()
        self.assertEqual(m._eligible_observations,{})
        for call in m._discovery_state_pub.publish.call_args_list:
            self.assertFalse(call.args[0].eliminated)

    def test_same_frame_precise_upgrade_is_accepted_once(self):
        self.send()
        raw = self.payload(official_view_ok=True)
        self.m._visual_source_cb(NS(data=json.dumps(raw)))
        self.assertTrue(self.m._visual_observations[('u0','t1')]['eligible'])
        self.assertEqual(self.m.tracker.report.call_count,1)
        self.m._visual_source_cb(NS(data=json.dumps(raw)))
        self.send()
        self.assertEqual(self.m.tracker.report.call_count,1)
        self.assertTrue(self.m._visual_observations[('u0','t1')]['eligible'])

    def test_same_frame_different_location_cannot_upgrade(self):
        self.send()
        self.m._visual_source_cb(NS(data=json.dumps(self.payload(official_view_ok=True,x=20.))))
        self.m.tracker.report.assert_not_called()

    def test_other_same_color_hypothesis_cannot_steal_active_owner(self):
        self.send()
        self.m._tracking['t1'] = 'u0'
        self.now += .2
        self.send(candidate_id='11',source_uav='u1',x=20.)
        self.assertEqual(len(self.m._discovery_pool),2)
        self.assertEqual(self.m._get_target_pos('t1',self.now),(8.,0.))
        self.assertEqual(self.m._discovery_state_pub.publish.call_count,1)

    def test_failed_place_can_yield_to_other_same_color_hypothesis(self):
        self.send()
        self.m._defer_failed_view('t1',('u0',),100.)
        self.now += .2
        self.send(candidate_id='11',source_uav='u1',x=20.)
        self.assertEqual(self.m._get_target_pos('t1',self.now),(20.,0.))

    def test_stale_unlocated_red_and_report_qualified_inputs_rejected(self):
        for change in (dict(sample_s=98.),dict(geometry_ok=False),dict(target_id='t4'),
                       dict(official_view_ok=True),dict(hits=1),dict(range_m=100.),
                       dict(x=float('nan')),dict(real_detection=False)):
            self.send(**change)
        self.assertEqual(self.m.tracker.targets,{})

    def test_discovery_state_only_updates_agent_approach_cache(self):
        self.send()
        state = self.m._discovery_state_pub.publish.call_args.args[0]
        a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.targets, a._t_seen, a._visual_observations = {},{},{}
        a.assignment = None
        a._confirm_start = 0.
        a._discovery_state_cb(state)
        self.assertEqual(a.targets['t1'],(8.,0.,0.,0.))
        self.assertEqual(a._visual_observations,{})
        self.assertEqual(a._confirm_start,0.)

    def test_discovery_does_not_erase_recent_bridge_velocity(self):
        self.send()
        state = self.m._discovery_state_pub.publish.call_args.args[0]
        a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.targets = {'t1':(7.8,0.,2.,0.)}
        a._t_seen, a._report_state_sample = {'t1':99.8},{'t1':99.8}
        a.assignment = None
        a._discovery_state_cb(state)
        self.assertEqual(a.targets['t1'],(7.8,0.,2.,0.))


if __name__ == '__main__':
    unittest.main()
