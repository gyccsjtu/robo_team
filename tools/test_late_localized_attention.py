"""A late qualified image must not turn a valid raw look into an expired one.

Replay exact sensor packets from 025212. These tests grant no task, report,
identity or localization credit and do not connect to the live ROS graph.
"""
import copy
import json
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

from tools import test_attention_priority as fixture

FIXTURE = json.loads((Path(__file__).parent / 'fixtures' /
                     'late_localized_attention_025212.json').read_text())


class LateLocalizedAttentionTests(unittest.TestCase):
    def setUp(self):
        fixture.AttentionPriorityTests.setUp(self)
        a = self.a
        a.uav_id = FIXTURE['source_uav']
        a.state = NS(mode='OFFBOARD')
        a._target_to_orbit = None
        a.targets = {}
        a._hypothesis_switch_s = {}
        self.first = next(e['data'] for e in FIXTURE['events']
                          if e['event'] == 'candidate' and
                          e['data']['sample_s'] == 1989.352)
        self.raw = next(e['data'] for e in FIXTURE['events']
                        if e['event'] == 'candidate' and
                        e['data']['sample_s'] == 1995.316)
        self.view = next(e['data'] for e in FIXTURE['events']
                         if e['event'] == 'tracking_view')
        self.now = FIXTURE['started']
        a._visual_candidate_cb(NS(data=json.dumps(self.first)))
        self.assertEqual(a._camera_cue['until'], FIXTURE['raw_until'])
        a.local_xy = tuple(FIXTURE['world_xy'])
        a.local_z = FIXTURE['local_z']

    def promote(self, raw, now):
        self.now = now
        self.a._tracking_view_cb(NS(data=json.dumps(raw)))

    def replay_raw_until(self, now):
        for event in FIXTURE['events']:
            if event['event'] != 'candidate' or event['ros'] <= FIXTURE['started']:
                continue
            # The agent log timestamps the final callback at 1995.488; the
            # independent sampler received its identical image at 1995.492.
            callback_s = (FIXTURE['promotion_agent_ros']
                          if event['data']['sample_s'] == 1995.316 else event['ros'])
            if callback_s > now:
                continue
            self.now = callback_s
            self.a._visual_candidate_cb(NS(data=json.dumps(event['data'])))
        self.now = now

    def test_exact_late_white_packet_preserves_remaining_raw_budget(self):
        self.replay_raw_until(FIXTURE['promotion_agent_ros'])
        before = copy.deepcopy(self.a._camera_cue)
        self.promote(self.view, self.now)
        self.assertEqual(self.a._camera_cue, before)
        self.assertEqual(self.a._camera_cue['candidate_id'], 'white:6')
        self.assertIsNone(self.a._camera_cue['xy'])
        self.assertAlmostEqual(self.a._camera_cue['until'], 1995.5)
        self.assertTrue(self.a._hold_camera_candidate())
        self.assertEqual(self.a._look_at, (
            self.a.world_xy[0] + 10. * fixture.sa.math.cos(self.raw['bearing']),
            self.a.world_xy[1] + 10. * fixture.sa.math.sin(self.raw['bearing'])))
        self.assertEqual(self.detections, [])
        self.assertEqual(self.a.targets, {})
        self.now = FIXTURE['raw_until']
        self.assertFalse(self.a._hold_camera_candidate())
        self.assertIsNone(self.a._camera_cue)

    def test_repeated_and_later_localized_frames_never_renew_raw_deadline(self):
        self.replay_raw_until(FIXTURE['promotion_agent_ros'])
        for sample, now in ((1995.316, 1995.488), (1995.316, 1995.49),
                            (1995.32, 1995.496)):
            data = dict(self.view, sample_s=sample)
            self.promote(data, now)
            self.assertEqual(self.a._camera_cue['candidate_id'], 'white:6')
            self.assertEqual(self.a._camera_cue['until'], 1995.5)
            self.assertIsNone(self.a._camera_cue['xy'])
        self.now = 1995.5
        self.assertFalse(self.a._hold_camera_candidate())

    def test_same_image_localized_callback_before_raw_keeps_original_budget(self):
        self.replay_raw_until(1995.288)
        before = copy.deepcopy(self.a._camera_cue)
        self.promote(self.view, FIXTURE['promotion_agent_ros'])
        self.assertEqual(self.a._camera_cue, before)
        self.a._visual_candidate_cb(NS(data=json.dumps(self.raw)))
        self.assertEqual(self.a._camera_cue['candidate_id'], 'white:6')
        self.assertEqual(self.a._camera_cue['sample_s'], 1995.316)
        self.assertEqual(self.a._camera_cue['until'], 1995.5)
        self.assertTrue(self.a._hold_camera_candidate())
        self.assertEqual(self.detections, [])

    def test_exact_four_second_boundary_rejects_nonpositive_promotion_budget(self):
        self.replay_raw_until(FIXTURE['started'] + fixture.sa.CAMERA_HANDOFF_HOLD_S)
        data = dict(self.view, sample_s=self.now, bearing=self.a._camera_cue['bearing'])
        before = copy.deepcopy(self.a._camera_cue)
        self.promote(data, self.now)
        self.assertEqual(self.a._camera_cue, before)

    def test_existing_early_promotion_still_uses_original_absolute_deadline(self):
        self.replay_raw_until(FIXTURE['started'] + 3.9)
        data = dict(self.view, sample_s=self.now, bearing=self.a._camera_cue['bearing'])
        self.promote(data, self.now)
        self.assertEqual(self.a._camera_cue['xy'], (self.view['x'], self.view['y']))
        self.assertEqual(self.a._camera_cue['started'], FIXTURE['started'])
        self.assertEqual(self.a._camera_cue['until'], FIXTURE['started'] + 4.)
        self.assertEqual(self.detections, [])
        self.assertEqual(self.a.targets, {})

    def test_invalid_or_unbound_late_image_cannot_modify_raw_lease(self):
        self.replay_raw_until(FIXTURE['promotion_agent_ros'])
        before = copy.deepcopy(self.a._camera_cue)
        for change in (dict(person_verdict_ok=False), dict(report_ok=False),
                       dict(geometry_ok=False), dict(appearance_status='ambiguous'),
                       dict(source_uav='typhoon_h480_4'),
                       dict(sample_s=self.now - .6), dict(candidate_id='white:99')):
            with self.subTest(change=change):
                self.promote(dict(self.view, **change), self.now)
                self.assertEqual(self.a._camera_cue, before)
        self.assertEqual(self.detections, [])


if __name__ == '__main__':
    unittest.main()
