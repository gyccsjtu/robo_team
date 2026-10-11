"""Frame arbitration must preserve real image and candidate identities."""
import json
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
import yolo_target_bridge as bridge
from visual_evidence import parse_visual_evidence


class CandidateBridgeTests(unittest.TestCase):
    def node(self):
        node = bridge._mk_bridge()
        node.source_pub = bridge._FakePub()
        return node

    @staticmethod
    def candidate(identity, x, y=0., tag='blue', **change):
        row = dict(target_id=tag, candidate_id=identity, xyz=[x, y, 1.25],
                   frame_id='world_enu', confidence=.9, range_m=8.,
                   official_view_ok=True, appearance_status='actor',
                   appearance_confidence=.9)
        row.update(change)
        return row

    def send(self, node, sample, candidates, source='u0', **change):
        data = dict(schema_version=2, source_uav=source, sample_s=sample,
                    real_detection=True, candidates=candidates)
        data.update(change)
        node._advance(max(0., sample+.01-node._now()))
        node._report_cb(NS(data=json.dumps(data)))

    def incumbent(self, duration=1., speed=0.):
        node = self.node()
        for k in range(round(duration*5)+1):
            t = 100.+k*.2
            self.send(node, t, [self.candidate('old', speed*(t-100.))])
        return node, t

    def evidence(self, node):
        return [json.loads(msg.data) for msg in node.source_pub.msgs]

    def test_frame_order_does_not_choose_a_different_instance(self):
        snapshots = []
        for reverse in (False, True):
            node = self.node()
            candidates = [self.candidate('b', 90.), self.candidate('a', -20.)]
            for k in range(6):
                self.send(node, 100.+k*.2, candidates[::(-1 if reverse else 1)])
            tr = node.core.tracks['blue']
            snapshots.append((tr.x, tr.good_x, tr.official_candidate_id,
                              self.evidence(node), dict(node.core._alternate_history)))
        self.assertEqual(snapshots[0], snapshots[1])
        self.assertEqual(snapshots[0][:3], (-20., -20., 'a'))

    def test_fresh_incumbent_wins_when_higher_scored_actor_walks_past(self):
        node, t = self.incumbent()
        for k in range(12):
            self.send(node, t+.2*(k+1), [self.candidate('other', 2., confidence=.99),
                                         self.candidate('old', 0., confidence=.8)])
        tr = node.core.tracks['blue']
        self.assertEqual((tr.x, tr.good_x, tr.official_candidate_id), (0., 0., 'old'))
        self.assertTrue(all(row['candidate_id'] == 'old' for row in self.evidence(node)))
        self.assertNotIn(('u0', 'other'), tr.source_history)

    def test_unseen_camera_associates_by_position_before_appearance_score(self):
        for reverse in (False, True):
            node, t = self.incumbent()
            for k in range(6):
                rows = [self.candidate('statue', 4., confidence=.99,
                                       appearance_confidence=.99),
                        self.candidate('person', 0., confidence=.8)]
                self.send(node, t+.2*(k+1), rows[::(-1 if reverse else 1)], source='u1')
            tr = node.core.tracks['blue']
            self.assertEqual((tr.x, tr.good_x, tr.tracking_candidate_id), (0., 0., 'person'))
            self.assertTrue(all(row['candidate_id'] != 'statue' for row in self.evidence(node)))

    def test_one_nearby_unverified_instance_cannot_move_fresh_pursuit_anchor(self):
        node, t = self.incumbent()
        for k in range(6):
            sample = t+.2*(k+1)
            self.send(node, sample, [self.candidate('old', 0.)])
            self.send(node, sample, [self.candidate('statue', 4.)], source='u1')
        tr = node.core.tracks['blue']
        self.assertEqual((tr.x, tr.good_x, tr.tracking_candidate_id), (0., 0., 'old'))
        self.assertTrue(all(row['candidate_id'] != 'statue' for row in self.evidence(node)))

    def test_new_camera_can_associate_with_a_moving_prediction(self):
        node, t = self.incumbent(duration=2., speed=2.)
        for k in range(3):
            sample = t+.5+k*.5
            self.send(node, sample, [self.candidate('person', 2.*(sample-100.))], source='u1')
        tr = node.core.tracks['blue']
        self.assertEqual(tr.tracking_candidate_id, 'person')
        self.assertEqual(tr.official_candidate_id, 'person')

    def test_same_camera_mature_statue_yields_only_after_six_actor_images(self):
        for reverse in (False, True):
            node, t = self.incumbent(duration=40.)
            for k in range(6):
                rows = [self.candidate('old', 0.), self.candidate('actor', 102., 19.)]
                self.send(node, t+.2*(k+1), rows[::(-1 if reverse else 1)])
                if k < 5:
                    self.assertEqual(node.core.tracks['blue'].official_candidate_id, 'old')
            tr = node.core.tracks['blue']
            self.assertEqual((tr.x, tr.good_x, tr.official_candidate_id), (102., 102., 'actor'))
            self.assertTrue(all(p[1] == 102. for p in tr.good_history))
            event = node.core._hypothesis_switches['blue']
            self.assertEqual((event['old_x'], event['new_x'], event['candidate_id']),
                             (0., 102., 'actor'))
            self.assertEqual(self.evidence(node)[-1]['hypothesis_switch'], event)
            self.assertFalse(node.core.eliminated)

    def test_arbitration_considers_all_ready_alternatives_before_switching(self):
        winners = []
        for reverse in (False, True):
            node, t = self.incumbent(duration=40.)
            for k in range(6):
                rows = [self.candidate('old', 0.), self.candidate('a', 40.),
                        self.candidate('z', 100., appearance_confidence=.99)]
                self.send(node, t+.2*(k+1), rows[::(-1 if reverse else 1)])
            winners.append(node.core.tracks['blue'].official_candidate_id)
        self.assertEqual(winners, ['z', 'z'])

    def test_a_long_gap_never_licenses_one_or_five_far_images(self):
        for gap in (1.6, 8.1, 30.):
            node, t = self.incumbent()
            for k in range(5):
                self.send(node, t+gap+k*.2, [self.candidate('actor', 100.)])
                self.assertEqual(node.core.tracks['blue'].x, 0.)
                self.assertNotIn('blue', node.core._hypothesis_switches)
            self.send(node, t+gap+1., [self.candidate('actor', 100.)])
            self.assertEqual(node.core.tracks['blue'].good_x, 100.)
            self.assertIn('blue', node.core._hypothesis_switches)

    def test_lost_incumbent_yields_without_waiting_for_35_seconds(self):
        node, t = self.incumbent()
        for k in range(6):
            self.send(node, t+2.1+k*.2, [self.candidate('actor', 100.)])
        tr = node.core.tracks['blue']
        self.assertLess(tr.good_t-100., 5.)
        self.assertEqual(tr.official_candidate_id, 'actor')
        self.assertEqual(set(tr.source_history), {('u0', 'actor')})
        self.assertEqual(set(tr.pursuit_history), {('u0', 'actor')})

    def test_same_instance_far_reacquisition_still_requires_six_images(self):
        node, t = self.incumbent()
        for k in range(5):
            self.send(node, t+3.+k*.2, [self.candidate('old', 100.)])
            self.assertEqual(node.core.tracks['blue'].good_x, 0.)
        self.send(node, t+4., [self.candidate('old', 100.)])
        self.assertEqual(node.core.tracks['blue'].good_x, 100.)
        self.assertIn('blue', node.core._hypothesis_switches)

    def test_six_images_must_span_a_second_and_have_no_long_gap(self):
        for offsets in ([0., .1, .2, .3, .4, .5], [0., .2, .4, 1.1, 1.3, 1.5]):
            node, t = self.incumbent()
            for offset in offsets:
                self.send(node, t+3.+offset, [self.candidate('actor', 100.)])
            self.assertEqual(node.core.tracks['blue'].good_x, 0.)

    def test_six_qualified_images_at_actual_two_hz_or_allowed_slowest_cadence(self):
        for cadence in (.4, .5, .6):
            node, t = self.incumbent()
            for k in range(6):
                self.send(node, t+3.+k*cadence, [self.candidate('actor', 100.)])
                if k < 5:
                    self.assertEqual(node.core.tracks['blue'].good_x, 0.)
            self.assertEqual(node.core.tracks['blue'].good_x, 100.)

    def test_distinct_instances_cannot_pool_their_qualified_images(self):
        node, t = self.incumbent()
        for k in range(6):
            self.send(node, t+3.+k*.2, [self.candidate('actor'+str(k % 2), 100.)])
        self.assertEqual(node.core.tracks['blue'].good_x, 0.)
        for key in (('blue', ('u0', 'actor0')), ('blue', ('u0', 'actor1'))):
            self.assertEqual(len(node.core._alternate_history[key]), 3)

    def test_repeated_frames_and_repeated_boxes_are_one_sample(self):
        node, t = self.incumbent()
        row = self.candidate('actor', 100.)
        for _ in range(20):
            self.send(node, t+3., [row]*6)
        self.assertEqual(len(node.core._alternate_history[('blue', ('u0', 'actor'))]), 1)
        self.assertEqual(node.core.tracks['blue'].good_x, 0.)

    def test_conflicting_copies_of_same_instance_are_dropped_regardless_of_order(self):
        for reverse in (False, True):
            node = self.node()
            rows = [self.candidate('one', 10.), self.candidate('one', 100.)]
            self.send(node, 100., rows[::(-1 if reverse else 1)])
            self.assertLess(node.core.tracks['blue'].t_obs, 0.)
            self.assertFalse(self.evidence(node))

    def test_unqualified_and_ambiguous_candidates_never_switch(self):
        for change in (dict(official_view_ok=False), dict(appearance_status='ambiguous'),
                       dict(range_m=13.), dict(confidence=.4)):
            node, t = self.incumbent()
            for k in range(10):
                self.send(node, t+3.+k*.2, [self.candidate('actor', 100., **change)])
            self.assertEqual(node.core.tracks['blue'].good_x, 0.)

    def test_ambiguous_appearance_cannot_construct_a_dispatchable_world_track(self):
        node = self.node()
        for k in range(8):
            self.send(node, 100.+k*.2, [self.candidate(
                'ambiguous', 10., appearance_status='ambiguous')])
        self.assertFalse(node.core.tracks['blue'].alive)
        self.assertFalse(self.evidence(node))
        self.assertEqual(node.core.tick(102.), [])

    def test_ambiguous_incumbent_does_not_extend_its_qualified_or_tracking_age(self):
        node, t = self.incumbent()
        count = len(self.evidence(node))
        for k in range(8):
            self.send(node, t+.2*(k+1), [self.candidate(
                'old', 0., appearance_status='ambiguous')])
        self.assertEqual(node.core.tracks['blue'].t_obs, t)
        self.assertEqual(node.core.tracks['blue'].good_t, t)
        self.assertEqual(len(self.evidence(node)), count)

    def test_fresh_moving_incumbent_remains_protected_after_35_seconds(self):
        node, t = self.incumbent(duration=40., speed=1.)
        for k in range(8):
            sample = t+.2*(k+1)
            self.send(node, sample, [self.candidate('old', sample-100.),
                                     self.candidate('actor', 100.)])
        self.assertEqual(node.core.tracks['blue'].official_candidate_id, 'old')

    def test_old_site_is_deferred_for_25_seconds_for_every_camera(self):
        node, t = self.incumbent()
        for k in range(6):
            self.send(node, t+3.+k*.2, [self.candidate('actor', 100.)])
        switch = node.core._hypothesis_switches['blue']
        for k in range(8):
            self.send(node, switch['switch_s']+2.+k*.2,
                      [self.candidate('other-old', 0.)], source='u1')
        self.assertEqual(node.core.tracks['blue'].good_x, 100.)
        for k in range(6):
            self.send(node, switch['trial_until_s']+.1+k*.2,
                      [self.candidate('other-old', 0.)], source='u1')
        self.assertEqual(node.core.tracks['blue'].good_x, 0.)

    def test_near_different_instance_does_not_mix_old_official_fit(self):
        node, t = self.incumbent()
        for k in range(5):
            self.send(node, t+3.+k*.2, [self.candidate('other', 2.)])
            self.assertEqual(node.core.tracks['blue'].good_x, 0.)
        self.send(node, t+4., [self.candidate('other', 2.)])
        tr = node.core.tracks['blue']
        self.assertEqual((tr.x, tr.good_x, tr.official_candidate_id), (2., 2., 'other'))
        self.assertTrue(all(p[1] == 2. for p in tr.good_history))
        self.assertNotIn('blue', node.core._hypothesis_switches)

    def test_legacy_single_row_remains_supported_and_publicly_identified(self):
        node = self.node()
        for k in range(4):
            sample = 100.+k*.2
            node._advance(sample+.01-node._now())
            row = self.candidate('unused', 10.)
            row.pop('candidate_id')
            row.update(source_uav='u0', sample_s=sample, real_detection=True)
            node._report_cb(NS(data=json.dumps(row)))
        self.assertEqual(node.core.tracks['blue'].accepted_source_samples['u0'], sample)
        self.assertEqual(self.evidence(node)[-1]['candidate_id'], 'legacy:blue')

    def test_two_red_slots_in_one_image_do_not_collapse_or_depend_on_order(self):
        outputs = []
        for reverse in (False, True):
            node = self.node()
            for k in range(6):
                rows = [self.candidate('red-a', 10., tag='red1'),
                        self.candidate('red-b', 30., tag='red2')]
                self.send(node, 100.+k*.2, rows[::(-1 if reverse else 1)])
            outputs.append(self.evidence(node))
            self.assertEqual({r['candidate_id'] for r in outputs[-1]}, {'red-a', 'red-b'})
            self.assertEqual({r['target_id'] for r in outputs[-1]}, {'t4', 't5'})
        self.assertEqual(outputs[0], outputs[1])

    def test_bad_frame_is_rejected_atomically_and_empty_frame_is_harmless(self):
        for change in (dict(source_uav='u1'), dict(sample_s=99.9),
                       dict(real_detection=False), dict(candidate_id='')):
            node = self.node()
            self.send(node, 100., [self.candidate('valid', 0.),
                                   self.candidate('invalid', 100., **change)])
            self.assertLess(node.core.tracks['blue'].t_obs, 0.)
            self.assertFalse(getattr(node, '_candidate_sample_s', {}))
        node, t = self.incumbent()
        self.send(node, t+.2, [])
        self.assertEqual(node.core.tracks['blue'].t_obs, t)

    def test_visual_parser_preserves_valid_identity_and_rejects_malformed_identity(self):
        node, t = self.incumbent()
        row = self.evidence(node)[-1]
        parsed = parse_visual_evidence(json.dumps(row), t+.01, {'u0'})
        self.assertEqual(parsed['candidate_id'], 'old')
        for candidate_id in (None, '', [], 7, 'x'*129):
            with self.assertRaises((ValueError, TypeError)):
                parse_visual_evidence(json.dumps(dict(row, candidate_id=candidate_id)),
                                      t+.01, {'u0'})

    def test_timer_cannot_publish_partially_arbitrated_camera_frame(self):
        node, t = self.incumbent()
        entered, release, ticked = threading.Event(), threading.Event(), threading.Event()
        original = node.core.report_frame
        def blocked(rows):
            entered.set()
            if not release.wait(2.):
                raise AssertionError('report was not released')
            return original(rows)
        with patch.object(node.core, 'report_frame', side_effect=blocked), \
                patch.object(node, '_tick_locked', side_effect=lambda _: ticked.set()):
            reporter = threading.Thread(target=self.send,
                args=(node, t+.2, [self.candidate('old', 0.)]))
            timer = threading.Thread(target=node._tick, args=(None,))
            reporter.start()
            self.assertTrue(entered.wait(1.))
            timer.start()
            try:
                self.assertFalse(ticked.wait(.03))
            finally:
                release.set()
                reporter.join(2.)
                timer.join(2.)
        self.assertTrue(ticked.is_set())
        self.assertFalse(reporter.is_alive() or timer.is_alive())

    def test_actual_perception_frame_keeps_same_color_instances(self):
        from perception.perception_real import report_candidate_frame
        node = self.node()
        tracks = []
        for identity, x in ((10, 0.), (11, 100.)):
            tr = NS(id=identity, cls='blue', miss=0, hits=20, conf=.9, rng=8.,
                    observed_s=100., appearance_status='actor',
                    observation_xy=lambda x=x: (x, 0.),
                    world_candidate_ready=lambda: True,
                    verdict=lambda *a, **kw: (True, 'eligible'), can_report_image=lambda: True)
            tracks.append(('blue', 0, tr))
        for k in range(6):
            sample = 100.+k*.2
            for _, _, tr in tracks:
                tr.observed_s = sample
            payload = report_candidate_frame(tracks, 'u0', sample, 'session', (0., 0.))
            node._advance(sample+.01-node._now())
            node._report_cb(NS(data=json.dumps(payload)))
        self.assertEqual(len(payload['candidates']), 2)
        self.assertEqual(node.core.tracks['blue'].tracking_candidate_id, 'session:10')
        self.assertTrue(all(r['candidate_id'] == 'session:10' for r in self.evidence(node)))


if __name__ == '__main__':
    unittest.main()
