"""Shadow seeds are actual image measurements and cannot gain task authority."""
import unittest

from perception.perception_real import fast_tracker_seed_frame


class FastVisualSeedTests(unittest.TestCase):
    def cue(self, **kwargs):
        row = dict(candidate_id='red:2', sample_s=100., confidence=.8,
                   box=[100., 200., 130., 280.], appearance_status='actor')
        row.update(kwargs)
        return row

    def test_same_colour_instances_are_distinct_and_process_scoped(self):
        out = fast_tracker_seed_frame([self.cue(), self.cue(candidate_id='red:3')],
                                      'typhoon_h480_5', 100., 'session')
        self.assertEqual([r['id'] for r in out['detections']],
                         ['session:red:2', 'session:red:3'])
        self.assertFalse(out['control_eligible'])
        self.assertFalse(out['report_eligible'])
        self.assertNotIn('xyz', out)

    def test_old_image_and_ambiguous_appearance_cannot_seed(self):
        out = fast_tracker_seed_frame([self.cue(sample_s=99.9),
                                      self.cue(appearance_status='ambiguous'),
                                      self.cue(appearance_status='distractor')],
                                      'typhoon_h480_5', 100., 'session')
        self.assertEqual(out['detections'], [])

    def test_invalid_boxes_and_confidence_are_excluded(self):
        out = fast_tracker_seed_frame([self.cue(box=[1., 2.]),
                                      self.cue(box=[1., 2., float('nan'), 4.]),
                                      self.cue(confidence=0.)],
                                      'typhoon_h480_5', 100., 'session')
        self.assertEqual(out['detections'], [])


if __name__ == '__main__':
    unittest.main()
