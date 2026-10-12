"""An appearance veto withdraws one original-camera instance only."""
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
import swarm_manager as sm
from cooperative_tracker import CooperativeTracker


class AppearanceRejectionTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        clock = patch.object(sm.rospy.Time, 'now',
                             side_effect=lambda: sm.rospy.Time.from_sec(self.now))
        clock.start()
        self.addCleanup(clock.stop)
        m = self.m = sm.SwarmManager.__new__(sm.SwarmManager)
        m.uav_ids = ['u0', 'u1', 'u2']
        m.status = {uid: NS(x=0., y=0., z=3., connected=True)
                    for uid in m.uav_ids}
        m.last_report = {uid: sm.rospy.Time(100.) for uid in m.uav_ids}
        m._tracking, m._backup = {'t4': 'u0', 't5': 'u2'}, {'t4': 'u1'}
        m._tracking_since = {'t4': 99.5, 't5': 90.}
        m._release_cooldown, m._approach_leases = {}, {}
        m._backup_approach_leases = {}
        m._appearance_rejections = {}
        m._visual_observations = {
            ('u0', 't4'): self.view('bad', 8., 100.),
            ('u1', 't4'): self.view('peer', 8.2, 99.9),
            ('u0', 't5'): self.view('other_red', 18., 100.),
        }
        m._eligible_observations = dict(m._visual_observations)
        m._discovery_pool = {
            ('t4', 'u0', 'bad'): self.view('bad', 8., 100.),
            ('t5', 'u0', 'other_red'): self.view('other_red', 18., 100.),
        }
        m._visual_source = {'t4': ('u0', 100., 8., 0.),
                            't5': ('u0', 100., 18., 0.)}
        m._truth_cache = {'t4': (8., 0., 100.), 't5': (18., 0., 100.)}
        m._cur_targets = {'t4': (8., 0., 'u0'), 't5': (18., 0., 'u0')}
        m._visual_candidate = {'t4': (100., 8., 0.), 't5': (100., 18., 0.)}
        m._target_hist, m._reacquire_sites = {}, {}
        m._last_detection_sample, m._qualified_detection_sample = {}, {}
        m._discovery_selected_s = {'t4': 100., 't5': 100.}
        m._free_search_lease, m.assign_pub = Mock(), Mock()
        cell = NS(state=sm.STATE_COVERED)
        m.grid = NS(world_to_cell=Mock(return_value=(1, 1)),
                    cell=Mock(return_value=cell), mark_review=Mock())
        m.tracker = CooperativeTracker(official_only=True, strict_samples=True)
        red_a = m.tracker.add_target('t4', 99.)
        red_a.set_observers(['u0', 'u1'])
        red_a.report('u0', 100., 8., 0.)
        red_a.report('u1', 99.9, 8.2, 0.)
        red_b = m.tracker.add_target('t5', 90.)
        red_b.set_observers(['u2'])

    @staticmethod
    def view(candidate, x, sample):
        return dict(candidate_id=candidate, source_uav='u0', sample_s=sample,
                    eligible=True, x=x, y=0., confidence=.9, hits=4,
                    since_s=sample-.5)

    def reject(self, candidate='bad', sample=100., label='red1'):
        self.m._appearance_rejection_cb(NS(data=json.dumps(dict(
            source_uav='u0', target_id=label, candidate_id=candidate,
            sample_s=sample, reason='distractor'))))

    def test_exact_red_instance_withdrawn_other_red_and_peer_preserved(self):
        ct = self.m.tracker.targets['t4']
        ct.fx, ct.fy, ct.t_fuse = 8., 0., 100.
        ct.confirm_since, ct.last_ok_t = 95., 99.9
        self.reject()
        m = self.m
        self.assertNotIn(('u0', 't4'), m._visual_observations)
        self.assertNotIn(('u0', 't4'), m._eligible_observations)
        self.assertNotIn(('t4', 'u0', 'bad'), m._discovery_pool)
        self.assertIn(('u1', 't4'), m._eligible_observations)
        self.assertIn(('u0', 't5'), m._eligible_observations)
        self.assertIn(('t5', 'u0', 'other_red'), m._discovery_pool)
        self.assertEqual(m._tracking, {'t4': 'u1', 't5': 'u2'})
        self.assertEqual(m._backup, {})
        self.assertEqual(m.tracker.targets['t4'].observers, {'u1'})
        self.assertNotIn('u0', m.tracker.targets['t4'].last_obs)
        self.assertEqual((ct.fx, ct.fy, ct.t_fuse), (8.2, 0., 99.9))
        self.assertEqual((ct.confirm_since, ct.last_ok_t), (95., 99.9))
        self.assertEqual((m.assign_pub.publish.call_args.args[0].uav_id,
                          m.assign_pub.publish.call_args.args[0].target_id,
                          m.assign_pub.publish.call_args.args[0].task_type),
                         ('u0', 't4', 255))
        m.grid.mark_review.assert_called_once_with((1, 1), 0.0)
        self.assertTrue(m._appearance_rejected('u0', 'bad', self.now))
        self.assertFalse(m._appearance_rejected('u0', 'other_red', self.now))

    def test_delayed_rejection_cannot_clear_a_newer_image_or_set_tombstone(self):
        self.m._visual_observations[('u0', 't4')]['sample_s'] = 100.2
        self.now = 100.3
        self.reject(sample=100.)
        self.assertIn(('u0', 't4'), self.m._visual_observations)
        self.assertEqual(self.m._tracking['t4'], 'u0')
        self.assertFalse(self.m._appearance_rejected('u0', 'bad', self.now))
        self.m.assign_pub.publish.assert_not_called()

    def test_older_same_colour_decoy_does_not_cancel_new_instance(self):
        m = self.m
        m._eligible_observations[('u0', 't4')] = self.view('bad', 8., 99.9)
        m._visual_observations[('u0', 't4')] = self.view('real', 9., 100.1)
        m._visual_source['t4'] = ('u0', 100.1, 9., 0.)
        self.now = 100.2
        self.reject(sample=100.)
        self.assertEqual(m._visual_observations[('u0', 't4')]['candidate_id'], 'real')
        self.assertNotIn(('u0', 't4'), m._eligible_observations)
        self.assertEqual(m._tracking['t4'], 'u0')
        self.assertEqual(m._visual_source['t4'], ('u0', 100.1, 9., 0.))
        m.assign_pub.publish.assert_not_called()

    def test_sole_rejected_source_clears_untrusted_confirmation(self):
        m = self.m
        m._eligible_observations.pop(('u1', 't4'))
        m._visual_observations.pop(('u1', 't4'))
        ct = m.tracker.targets['t4']
        ct.last_obs.pop('u1')
        ct.fx, ct.fy, ct.t_fuse = 8., 0., 100.
        ct.confirm_since, ct.last_ok_t = 95., 100.
        self.reject()
        self.assertIsNone(ct.fx)
        self.assertIsNone(ct.confirm_since)
        self.assertIsNone(ct.last_ok_t)

    def test_tombstone_blocks_queued_old_frame_but_expires(self):
        self.reject()
        m = self.m
        old = dict(target_id='t4', source_uav='u0', sample_s=100.,
                   x=8., y=0., confidence=.9, real_detection=True,
                   range_m=8., official_view_ok=True, candidate_id='bad')
        m._visual_source_cb(NS(data=json.dumps(old)))
        self.assertNotIn(('u0', 't4'), m._visual_observations)
        self.assertFalse(m._appearance_rejected('u0', 'real', 100.))
        self.assertFalse(m._appearance_rejected('u0', 'bad', 108.))

    def test_repeated_images_cannot_extend_fixed_eight_second_tombstone(self):
        self.reject()
        m = self.m
        self.assertTrue(m._appearance_rejected('u0', 'bad', 104.))
        self.assertEqual(m._appearance_rejections[('u0', 'bad')], (108., 100.))
        self.assertTrue(m._appearance_rejected('u0', 'bad', 107.))
        self.assertEqual(m._appearance_rejections[('u0', 'bad')], (108., 100.))
        self.assertFalse(m._appearance_rejected('u0', 'bad', 108.))


if __name__ == '__main__':
    unittest.main()
