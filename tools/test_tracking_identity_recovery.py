"""Lost camera IDs recover pursuit without promoting unqualified judge data."""
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'coordination/src/robocup_swarm/scripts'))
import yolo_target_bridge as bridge


class TrackingIdentityRecoveryTests(unittest.TestCase):
    def setUp(self):
        transport = patch.object(bridge, '_msg_string_cls', return_value=bridge._FakeString)
        transport.start()
        self.addCleanup(transport.stop)

    def node(self):
        node = bridge._mk_bridge()
        node.source_pub = bridge._FakePub()
        return node

    def send(self, node, sample, candidate='old', x=0., official=True, **change):
        row = dict(target_id='blue', candidate_id=candidate, xyz=[x, 0., 1.25],
                   frame_id='world_enu', confidence=.9, range_m=8.,
                   official_view_ok=official, appearance_status='actor',
                   appearance_confidence=.9)
        row.update(change)
        node._advance(max(0., sample+.01-node._now()))
        node._report_cb(NS(data=json.dumps(dict(schema_version=2, source_uav='u0',
            sample_s=sample, real_detection=True, candidates=[row]))))

    def incumbent(self):
        node = self.node()
        for k in range(6):
            self.send(node, 100.+k*.2)
        return node, 101.

    def test_slow_same_person_rebuild_recovers_with_original_image_time(self):
        for cadence in (.7, 1.):
            for official in (False, True):
                with self.subTest(cadence=cadence, official=official):
                    node, t = self.incumbent()
                    for k in range(30):
                        sample = t+5.+k*cadence
                        self.send(node, sample, 'rebuilt', official=official)
                        node._tick(None)
                        if k < 5:
                            self.assertEqual(node.core.tracks['blue'].tracking_candidate_id, 'old')
                    tr = node.core.tracks['blue']
                    self.assertTrue(tr.alive)
                    self.assertEqual(tr.tracking_candidate_id, 'rebuilt')
                    self.assertEqual(tr.t_obs, sample)
                    evidence = [json.loads(m.data) for m in node.source_pub.msgs]
                    self.assertEqual(evidence[-1]['candidate_id'], 'rebuilt')
                    self.assertEqual(evidence[-1]['sample_s'], sample)
                    self.assertEqual(evidence[-1]['official_view_ok'], official)
                    if official:
                        self.assertEqual(tr.official_candidate_id, 'rebuilt')
                        self.assertIsNone(bridge.official_point_at(tr, sample+.61))
                    else:
                        self.assertEqual(tr.good_history, [])
                        self.assertIsNone(bridge.official_point_at(tr, sample+.01))
                        self.assertEqual(node._actor_pubs['blue'].msgs, [])

    def test_unqualified_moving_images_recover_without_old_fit_or_far_switch(self):
        node, t = self.incumbent()
        for k in range(6):
            self.send(node, t+5.+k, 'rebuilt', x=1.2*k, official=False)
        tr = node.core.tracks['blue']
        self.assertEqual(tr.tracking_candidate_id, 'rebuilt')
        self.assertAlmostEqual(tr.x, 6.)
        self.assertEqual(tr.good_history, [])
        self.assertNotIn('blue', node.core._hypothesis_switches)
        self.assertIsNone(bridge.official_point_at(tr, node._now()))

    def test_recovered_identity_still_needs_three_eligible_official_images(self):
        node, t = self.incumbent()
        for k in range(6):
            self.send(node, t+5.+k, 'rebuilt', official=False)
        self.assertEqual(node.core.tracks['blue'].good_history, [])
        for k in range(3):
            sample = t+11.+k
            self.send(node, sample, 'rebuilt', official=True)
            tr = node.core.tracks['blue']
            if k < 2:
                self.assertEqual(tr.good_history, [])
            else:
                self.assertEqual(tr.good_t, sample)
                self.assertEqual(tr.official_candidate_id, 'rebuilt')
        self.assertTrue(all(p[0] >= t+11. for p in tr.source_history[('u0', 'rebuilt')]))
        self.assertIsNone(bridge.official_point_at(tr, sample+.61))

    def test_replayed_or_timer_predicted_samples_do_not_accumulate_rebind(self):
        node, t = self.incumbent()
        for _ in range(20):
            self.send(node, t+5., 'rebuilt', official=False)
        for k in range(20):
            node._advance(.1)
            node._tick(None)
        tr = node.core.tracks['blue']
        self.assertEqual(tr.tracking_candidate_id, 'old')
        self.assertEqual(tr.t_obs, t)
        self.assertEqual(len(node.core._tracking_rebind_history[('blue', ('u0', 'rebuilt'))]), 1)

    def test_long_gaps_and_distinct_ids_cannot_pool_six_images(self):
        node, t = self.incumbent()
        for k in range(30):
            self.send(node, t+5.+k*1.3, 'rebuilt', official=False)
        self.assertEqual(node.core.tracks['blue'].tracking_candidate_id, 'old')
        node, t = self.incumbent()
        for k in range(10):
            self.send(node, t+5.+k*.2, 'rebuilt'+str(k % 2), official=False)
        self.assertEqual(node.core.tracks['blue'].tracking_candidate_id, 'old')

    def test_fresh_incumbent_is_not_replaced_by_neighbouring_candidate(self):
        node, t = self.incumbent()
        for k in range(30):
            sample = t+.2*(k+1)
            self.send(node, sample, 'old')
            self.send(node, sample, 'neighbour', x=2., official=False)
        self.assertEqual(node.core.tracks['blue'].tracking_candidate_id, 'old')
        self.assertFalse(node.core._tracking_rebind_history)

    def test_far_or_ambiguous_unqualified_candidate_cannot_use_near_recovery(self):
        for change in (dict(x=100.), dict(appearance_status='ambiguous'),
                       dict(confidence=.4), dict(range_m=13.)):
            node, t = self.incumbent()
            for k in range(30):
                self.send(node, t+5.+k*.7, 'other', official=False, **change)
            self.assertEqual(node.core.tracks['blue'].tracking_candidate_id, 'old')

    def test_incoherent_depth_jumps_do_not_establish_new_identity(self):
        node, t = self.incumbent()
        for k in range(30):
            self.send(node, t+5.+k, 'other', x=2.*(k % 2), official=False)
        self.assertEqual(node.core.tracks['blue'].tracking_candidate_id, 'old')

    def test_out_of_order_candidate_image_cannot_rewind_recovery_history(self):
        node, t = self.incumbent()
        for k in range(3):
            self.send(node, t+5.+k, 'rebuilt', official=False)
        # Exercise the pure core as well as the ROS per-candidate replay filter.
        node.core.report(t+6.5, 'blue', 0., 0., .9, 8., official_ok=False,
                         source_uav='u0', candidate_id='rebuilt')
        for k in range(3, 6):
            self.send(node, t+5.+k, 'rebuilt', official=False)
        tr = node.core.tracks['blue']
        self.assertEqual(tr.tracking_candidate_id, 'rebuilt')
        self.assertEqual(tr.t_obs, t+10.)
        self.assertTrue(all(p[0] != t+6.5 for p in tr.pursuit_history[('u0', 'rebuilt')]))


if __name__ == '__main__':
    unittest.main()
