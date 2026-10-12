"""Same-image sculpture evidence must not break a real actor report by default."""
import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

os.environ.setdefault('ROBOCUP_LOG_DIR', '/tmp/robocup_decoy_tests')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from perception import perception_real as pr
from perception.distractor_verifier import SecondaryAppearance


def box(cls=2, conf=.95, xy=(100., 100., 130., 180.)):
    return NS(cls=cls, conf=conf, xyxy=[list(xy)],
              appearance_status='actor', distractor_confidence=0.)


class SecondaryAppearanceTests(unittest.TestCase):
    def setUp(self):
        self.guard = SecondaryAppearance.__new__(SecondaryAppearance)
        self.guard.mode = 'enforce'
        self.guard.histories = {}
        self.guard.model = Mock()
        self.secondary = [box(cls=5, conf=.95)]
        self.guard.model.predict.return_value = [NS(
            names={5: 'distractor_person'}, boxes=self.secondary)]

    def filter(self, t, primary=None, camera='u0'):
        return self.guard.filter(None, primary if primary is not None else [box()],
                                 pr.CLASSES, t, camera)

    def test_one_strong_frame_blocks_report_but_keeps_image_candidate(self):
        kept, evidence = self.filter(1.)
        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0].appearance_status, 'ambiguous')
        self.assertFalse(kept[0].secondary_decoy_confirmed)
        self.assertEqual(evidence[0]['status'], 'secondary_pending')
        track = NS(miss=0, observed_s=1., hits=5, appearance_status='ambiguous',
                   world_candidate_ready=lambda: True, cls='blue', id=7,
                   observation_xy=lambda: (1., 2.), conf=.95, uv=(120., 140.),
                   wh=(30., 80.), rng=8., can_report_image=lambda: False,
                   verdict=lambda *_args, **_kw: (True, ''))
        report = pr.report_candidate_frame([('blue', 0, track)], 'u0', 1., 'sess', (0., 0.))
        self.assertEqual(len(report['candidates']), 1)
        self.assertFalse(report['candidates'][0]['official_view_ok'])

    def test_three_distinct_frames_confirm_and_emit_instance_rejection(self):
        for sample in (1., 1.15):
            kept, _ = self.filter(sample)
            self.assertEqual(kept[0].appearance_status, 'ambiguous')
        kept, evidence = self.filter(1.30)
        self.assertEqual(kept[0].appearance_status, 'distractor')
        self.assertTrue(kept[0].secondary_decoy_confirmed)
        self.assertEqual(evidence[0]['status'], 'secondary_distractor')
        track = NS(observed_s=1.30, hits=5, appearance_status='distractor',
                   secondary_decoy_confirmed=True, id=7)
        self.assertEqual(pr.appearance_rejection_events(
            [('blue', 0, track)], 'u0', 1.30, 'sess'), [dict(
                source_uav='u0', target_id='blue', candidate_id='sess:7',
                sample_s=1.30, reason='distractor')])

    def test_rejection_event_requires_current_secondary_confirmation(self):
        stale = NS(observed_s=1., hits=8, appearance_status='distractor',
                   secondary_decoy_confirmed=True, id=11)
        primary_only = NS(observed_s=2., hits=8, appearance_status='distractor',
                          secondary_decoy_confirmed=False, id=12)
        pending = NS(observed_s=2., hits=8, appearance_status='ambiguous',
                     secondary_decoy_confirmed=False, id=13)
        self.assertEqual(pr.appearance_rejection_events(
            [('red', 0, stale), ('red', 1, primary_only),
             ('white', 0, pending)], 'u5', 2., 'sess'), [])
        primary_only.secondary_decoy_confirmed = True
        self.assertEqual(pr.appearance_rejection_events(
            [('red', 1, primary_only)], 'u5', 2., 'sess')[0]['target_id'],
            'red2')

    def test_duplicate_and_out_of_order_frames_do_not_confirm(self):
        for sample in (1., 1., .9, 1.15):
            kept, _ = self.filter(sample)
            self.assertEqual(kept[0].appearance_status, 'ambiguous')
        kept, _ = self.filter(1.30)
        self.assertEqual(kept[0].appearance_status, 'distractor')

    def test_conflicting_secondary_actor_preserves_primary(self):
        self.guard.model.predict.return_value = [NS(names={5: 'distractor_person',
                                                       2: 'blue'},
                                                    boxes=[box(cls=5, conf=.95),
                                                           box(cls=2, conf=.8)])]
        for sample in (1., 1.15, 1.30):
            kept, evidence = self.filter(sample)
            self.assertEqual(kept[0].appearance_status, 'actor')
            self.assertEqual(evidence, [])

    def test_one_secondary_error_never_sticks_to_true_white_or_green(self):
        for primary_class in (1, 3):
            with self.subTest(primary_class=primary_class):
                self.guard.histories.clear()
                first, _ = self.filter(1., [box(cls=primary_class)])
                self.assertEqual(first[0].appearance_status, 'ambiguous')
                # The next image has no unique strong decoy evidence. It must
                # restore the original v3 actor immediately, not blacklist a
                # colour or the prior pixel location.
                self.guard.model.predict.return_value = [NS(
                    names={5: 'distractor_person'}, boxes=[])]
                restored, _ = self.filter(1.2, [box(cls=primary_class)])
                self.assertEqual(restored[0].appearance_status, 'actor')
                self.guard.model.predict.return_value = [NS(
                    names={5: 'distractor_person'}, boxes=self.secondary)]

    def test_weak_missing_or_failed_secondary_preserves_actor(self):
        for secondary in ([box(cls=5, conf=.84)], []):
            self.guard.model.predict.return_value = [NS(
                names={5: 'distractor_person'}, boxes=secondary)]
            kept, evidence = self.filter(1.+len(secondary))
            self.assertEqual(kept[0].appearance_status, 'actor')
            self.assertEqual(evidence, [])
        self.guard.model.predict.side_effect = RuntimeError('secondary unavailable')
        kept, evidence = self.filter(3.)
        self.assertEqual(kept[0].appearance_status, 'actor')
        self.assertEqual(evidence, [])

    def test_nearby_independent_actor_and_cameras_keep_separate(self):
        near = box(cls=3, xy=(125., 100., 155., 180.))
        for sample in (1., 1.15, 1.30):
            kept, _ = self.filter(sample, [box(), near])
        self.assertEqual([b.appearance_status for b in kept],
                         ['distractor', 'actor'])
        kept, _ = self.filter(1.30, [box()], camera='u1')
        self.assertEqual(kept[0].appearance_status, 'ambiguous')

    def test_no_primary_boxes_skip_secondary_inference(self):
        self.guard.model.predict.reset_mock()
        self.assertEqual(self.filter(1., []), ([], []))
        self.guard.model.predict.assert_not_called()


if __name__ == '__main__':
    unittest.main()
