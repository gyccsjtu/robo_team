"""A spatial colour rebind must reach dispatch and invalidate old flight state."""
import json
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa
import yolo_target_bridge as bridge
from visual_evidence import parse_visual_evidence


def switch_event(stamp=100.):
    return dict(switch_s=stamp, old_x=0., old_y=0., new_x=100., new_y=20.,
                source_uav='u1', trial_until_s=stamp+25.)


def evidence(**change):
    data = dict(target_id='t1', source_uav='u1', sample_s=100., x=100., y=20.,
                real_detection=True, confidence=.9, official_view_ok=True,
                range_m=8., hypothesis_switch=switch_event())
    data.update(change)
    return NS(data=json.dumps(data))


class HypothesisEventTests(unittest.TestCase):
    def agent(self, target='t1'):
        a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id = 'u0'
        a.assignment = NS(task_type=1, target_id=target, target_x=0., target_y=0.)
        a._track_assigned_id = target
        a._team_tracks = {'t1': {'u0'}, 't0': {'u1'}}
        a._friend_positions = {'u1': (100., 20., 3.)}
        a._team_visual = {('u0', 't1'): dict(sample_s=99.8, hits=10, since_s=90.)}
        a._visual_observations = {'t1': dict(sample_s=99.8, eligible=True)}
        a._eligible_observations = dict(a._visual_observations)
        a._last_eligible_image = {'t1': 99.8}
        a._flee_run_t, a._walk_since = {'t1': 90.}, {'t1': 95.}
        a._escape_spent, a._spend_mode = {'t1'}, {'t1'}
        a.targets = {'t1': (0., 0., 1., 0.), 't0': (105., 20., .2, 0.)}
        a._t_seen, a._target_state = {'t1': 99.8}, {}
        a._tracking_image = dict(target_id=target, sample_s=99.8)
        a._tracking_altitude = dict(target_id=target, goal=3.4)
        a._orbit_target, a._confirm_close = target, True
        a._confirm_start, a._last_confirm_t = 90., 99.8
        a._giveup_until = {'t1': 130.}
        a._plan_lock, a._plan_epoch = threading.RLock(), 2
        a._plan_pending, a.path, a.path_target = object(), [(0., 0.)], (0., 0.)
        a._last_flight_v = (1., 0.)
        a.detect_pub = Mock()
        return a

    def test_schema_rejects_malformed_or_unqualified_switch(self):
        cases = [dict(target_id='t4'), dict(official_view_ok=False),
                 dict(x=-100.), dict(hypothesis_switch=[])]
        for change in (dict(switch_s=101.), dict(old_x=100., old_y=20.),
                       dict(trial_until_s=126.), dict(new_x=float('nan')),
                       dict(source_uav='unknown')):
            cases.append(dict(hypothesis_switch=dict(switch_event(), **change)))
        for change in cases:
            with self.subTest(change=change), self.assertRaises(ValueError):
                parse_visual_evidence(evidence(**change).data, 100.1, {'u0', 'u1'})
        data = parse_visual_evidence(evidence().data, 100.1, {'u0', 'u1'})
        self.assertEqual(data['hypothesis_switch'], switch_event())
        legacy = json.loads(evidence().data)
        legacy.pop('hypothesis_switch')
        self.assertNotIn('hypothesis_switch', parse_visual_evidence(
            json.dumps(legacy), 100.1, {'u0', 'u1'}))

    def test_peer_switch_clears_old_flight_without_fabricating_own_detection(self):
        a = self.agent()
        with patch.object(sa.rospy.Time, 'now', return_value=sa.rospy.Time(100.1)):
            a._visual_source_cb(evidence())
        self.assertEqual(a._plan_epoch, 3)
        self.assertEqual(a.path, [])
        self.assertEqual(a._last_flight_v, (0., 0.))
        self.assertEqual(a.targets['t1'], (100., 20., 0., 0.))
        self.assertEqual(a.assignment.target_id, 't1')
        self.assertEqual(a._target_to_orbit, (100., 20.))
        self.assertIsNone(a._tracking_image)
        self.assertFalse(a._confirm_close)
        self.assertNotIn('t1', a._escape_spent)
        self.assertNotIn('t1', a._visual_observations)
        self.assertNotIn('t1', a._last_eligible_image)
        a.detect_pub.publish.assert_not_called()

    def test_busy_green_observer_keeps_its_path_and_camera(self):
        a = self.agent('t0')
        image = a._tracking_image
        with patch.object(sa.rospy.Time, 'now', return_value=sa.rospy.Time(100.1)):
            a._visual_source_cb(evidence())
        self.assertEqual(a._plan_epoch, 2)
        self.assertEqual(a._orbit_target, 't0')
        self.assertIs(a._tracking_image, image)
        self.assertTrue(a._confirm_close)

    def test_repeated_event_does_not_cancel_new_route_or_reset_new_camera(self):
        a = self.agent()
        with patch.object(sa.rospy.Time, 'now', return_value=sa.rospy.Time(100.5)):
            a._visual_source_cb(evidence())
            a.path = [(95., 20.)]
            a._tracking_image = dict(target_id='t1', sample_s=100.3)
            a._visual_source_cb(evidence(sample_s=100.4))
        self.assertEqual(a._plan_epoch, 3)
        self.assertEqual(a.path, [(95., 20.)])
        self.assertIsNotNone(a._tracking_image)
        self.assertEqual(a._team_visual[('u1','t1')]['hits'], 2)

    def test_older_event_cannot_roll_back_current_binding(self):
        a = self.agent()
        with patch.object(sa.rospy.Time, 'now', return_value=sa.rospy.Time(100.5)):
            a._visual_source_cb(evidence())
            a._visual_source_cb(evidence(sample_s=100.4,
                                        hypothesis_switch=switch_event(99.)))
        self.assertEqual(a._hypothesis_switch_s['t1'], 100.)
        self.assertEqual(a._team_visual[('u1','t1')]['sample_s'], 100.)

    def test_old_target_state_cannot_restore_pre_switch_velocity(self):
        a = self.agent()
        with patch.object(sa.rospy.Time, 'now', return_value=sa.rospy.Time(100.1)):
            a._visual_source_cb(evidence())
            a._target_cb(NS(target_id='t1', eliminated=False,
                header=NS(stamp=sa.rospy.Time(99.9)), x=0., y=0., vx=2., vy=0.,state=1))
        self.assertEqual(a.targets['t1'], (100., 20., 0., 0.))

    def test_metadata_arriving_after_target_state_preserves_newer_position(self):
        a = self.agent()
        a._t_seen['t1'], a.targets['t1'] = 100.1, (100.2, 20., .4, 0.)
        a._report_state_sample = {'t1': 100.1}
        with patch.object(sa.rospy.Time, 'now', return_value=sa.rospy.Time(100.2)):
            a._visual_source_cb(evidence())
            a._discovery_state_cb(NS(target_id='t1', eliminated=False,
                header=NS(stamp=sa.rospy.Time(100.15)), x=100.25, y=20.))
        self.assertEqual(a.targets['t1'], (100.2, 20., .4, 0.))
        self.assertEqual(a._target_to_orbit, (100.2, 20.))

    def test_assignment_cannot_interleave_halfway_through_blue_rebind(self):
        a = self.agent()
        entered, release, assign_started, assigned = [threading.Event() for _ in range(4)]
        original = a._discard_old_plan
        failures = []
        def paused_discard():
            entered.set()
            if not release.wait(2.):
                raise AssertionError('test did not release callback')
            original()
        a._discard_old_plan = paused_discard
        def run_switch():
            try:
                a._visual_source_cb(evidence())
            except Exception as exc:
                failures.append(exc)
        def assign_green():
            assign_started.set()
            try:
                a._assign_cb(NS(uav_id='u0', target_id='t0', task_type=1,
                                target_x=105., target_y=20.))
            except Exception as exc:
                failures.append(exc)
            finally:
                assigned.set()
        with patch.object(sa.rospy.Time, 'now', return_value=sa.rospy.Time(100.1)):
            first = threading.Thread(target=run_switch)
            second = threading.Thread(target=assign_green)
            first.start()
            try:
                self.assertTrue(entered.wait(1.))
                second.start()
                self.assertTrue(assign_started.wait(1.))
                self.assertFalse(assigned.wait(.05))
            finally:
                release.set()
                first.join(2.)
                if second.ident is not None:
                    second.join(2.)
        self.assertFalse(failures)
        self.assertFalse(first.is_alive() or second.is_alive())
        self.assertEqual(a.assignment.target_id, 't0')
        self.assertEqual(a._target_to_orbit, (105., 20.))

    def test_bridge_repeats_real_switch_metadata_only_during_trial(self):
        b = bridge._mk_bridge()
        b.source_pub = bridge._FakePub()
        for k in range(201):
            b.core.report(100.+k*.2, 'blue', 0., 0., .9, 8., source_uav='u0')
        def send(sample, x=100., y=20.):
            b._advance(sample+.05-b._now())
            b._report_cb(NS(data=json.dumps(dict(target_id='blue',source_uav='u1',
                sample_s=sample, xyz=[x,y,1.25], real_detection=True,
                confidence=.9, range_m=8., official_view_ok=True))))
        for k in range(9):
            send(140.2+k*.2)
        events=[json.loads(m.data) for m in b.source_pub.msgs
                if 'hypothesis_switch' in json.loads(m.data)]
        self.assertGreaterEqual(len(events), 2)
        switch = events[0]['hypothesis_switch']
        self.assertTrue(all(r['hypothesis_switch']==switch for r in events))
        for row in events:
            parse_visual_evidence(json.dumps(row),row['sample_s']+.05,{'u0','u1'})
        previous_count = len(b.source_pub.msgs)
        for k in range(6):
            send(switch['trial_until_s']+.1+k*.2)
        self.assertGreater(len(b.source_pub.msgs), previous_count)
        self.assertNotIn('hypothesis_switch',json.loads(b.source_pub.msgs[-1].data))


if __name__ == '__main__':
    unittest.main()
