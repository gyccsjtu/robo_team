"""A fresh camera that cannot report must not block a valid nearby source."""
import unittest
from unittest.mock import patch
import test_candidate_bridge as fixtures

bridge = fixtures.bridge


class BlockedReportSourceTests(unittest.TestCase):
    node = fixtures.CandidateBridgeTests.node
    candidate = staticmethod(fixtures.CandidateBridgeTests.candidate)
    send = fixtures.CandidateBridgeTests.send
    incumbent = fixtures.CandidateBridgeTests.incumbent

    def setUp(self):
        p = patch.object(bridge, 'REPORT_POLICY', 'fresh')
        p.start(); self.addCleanup(p.stop)

    def scenario(self, blocked=True, challenger_x=0., eligible=True, count=3):
        node, t = self.incumbent()
        if blocked:
            node._gate_live['blue'] = False
        for i in range(count):
            sample = t+.2*(i+1)
            self.send(node, sample, [self.candidate('old', 0., range_m=11.)])
            self.send(node, sample+.02, [self.candidate('new', challenger_x,
                      range_m=7., official_view_ok=eligible)], source='u1')
        return node

    def test_three_coherent_accepted_images_unblock_reporting(self):
        node = self.scenario()
        tr = node.core.tracks['blue']
        self.assertEqual(tr.official_source, 'u1')
        self.assertTrue(node._gate_live['blue'])
        self.assertEqual(tr.good_history, tr.source_history[('u1', 'new')])
        self.assertTrue(all(p[1] == 0. for p in tr.good_history))
        self.assertEqual(node._pub_last['blue'], node._now())

    def test_live_hysteresis_does_not_trigger_needless_switch(self):
        node = self.scenario(blocked=False)
        self.assertEqual(node.core.tracks['blue'].official_source, 'u0')

    def test_single_image_unqualified_and_disagreeing_camera_do_not_switch(self):
        for kwargs in ({'count': 1}, {'eligible': False}, {'challenger_x': 1.2}):
            with self.subTest(**kwargs):
                node = self.scenario(**kwargs)
                self.assertEqual(node.core.tracks['blue'].official_source, 'u0')
                self.assertFalse(node._gate_live['blue'])

    def test_stale_evidence_cannot_reopen_a_gate(self):
        node = self.scenario()
        node._gate_live['blue'] = False
        tr = node.core.tracks['blue']
        old = tr.good_t
        node._advance(2.)
        row = dict(source_uav='u0', candidate_id='old', sample_s=old-.02,
                   range_m=7., x=0., y=0.)
        self.assertFalse(node._handoff_range_blocked_source(tr, row, node._now()))
        self.assertFalse(node._gate_live['blue'])

    def test_expired_image_yields_to_three_fresh_coherent_images(self):
        node, t = self.incumbent()
        for i in range(3):
            self.send(node, t+.2*(i+1), [self.candidate('new', 0., range_m=7.)],
                      source='u1')
        tr = node.core.tracks['blue']
        self.assertEqual(tr.official_source, 'u1')
        self.assertEqual(tr.good_t, t+.6)
        self.assertEqual(tr.good_history, tr.source_history[('u1', 'new')])
        self.assertEqual(len(tr.good_history), 3)
        self.assertEqual(node._pub_last['blue'], node._now())

    def test_valid_incumbent_is_not_replaced_by_secondary_images(self):
        node, t = self.incumbent()
        for i in range(3):
            sample = t+.2*(i+1)
            self.send(node, sample, [self.candidate('old', 0.)])
            self.send(node, sample, [self.candidate('new', 0., range_m=7.)], source='u1')
        self.assertEqual(node.core.tracks['blue'].official_source, 'u0')

    def test_expiry_does_not_qualify_disagreeing_or_invalid_observations(self):
        for kwargs in ({'x': 1.2}, {'official_view_ok': False}, {'range_m': 10.}):
            with self.subTest(**kwargs):
                node, t = self.incumbent()
                for i in range(3):
                    row = self.candidate('new', 0., range_m=7.)
                    if 'x' in kwargs:
                        row['xyz'][0] = kwargs['x']
                    else:
                        row.update(kwargs)
                    self.send(node, t+.2*(i+1), [row], source='u1')
                self.assertEqual(node.core.tracks['blue'].official_source, 'u0')


if __name__ == '__main__':
    unittest.main()
