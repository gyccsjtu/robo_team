"""Replay actual white19 cancellation-before-pursuit; no ROS graph or flight calls."""
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from tools import test_attention_priority as fixture
from search_view_hint import parse_failed_view

RECORDED = json.loads((Path(__file__).parent/'fixtures/interrupted_white19_044046.json').read_text())


class InterruptedCameraCueTests(unittest.TestCase):
    def setUp(self):
        fixture.AttentionPriorityTests.setUp(self)
        a = self.a
        a.uav_id = RECORDED['source_uav']
        a.state = NS(**RECORDED['safety_state_near_interrupt']['state'])
        a.targets = {}
        a._target_to_orbit = None
        a._tracking_image = None
        a._search_scan_key = None  # coverage is outside this direction-revisit test
        old = RECORDED['initial_search_assignment']
        a.assignment = self.assignment(old)
        a._search_segment = dict(id=old['task'], revision=old['version'],
            cells=tuple(map(tuple, old['cells'])), points=tuple(map(tuple, old['points'])),
            look_points=tuple(map(tuple, old['look_points'])), index=0, until=old['until'],
            purpose=old['purpose'], state=0, created=old['stamp'])
        a._last_task_stamp = old['stamp']
        self.recorded_tasks = [e for e in RECORDED['events'] if e['event']=='assignment']
        self.replay_images()
        self.original = copy.deepcopy(a._camera_cue)
        self.assertEqual(self.original['candidate_id'], 'white:19')
        self.assertAlmostEqual(self.original['sample_s'], 2567.264)
        self.assertAlmostEqual(self.original['until'], 2570.74)

    def assignment(self, data=None, kind=1, target='t1'):
        data = data or dict(stamp=self.now, type=kind, target=target, task='', version=0,
                            cells=[], points=[], purpose='', look_points=[], until=0.)
        m = fixture.sa.SearchAssignment()
        m.header.stamp = fixture.sa.rospy.Time.from_sec(data['stamp'])
        m.uav_id = RECORDED['source_uav']
        m.task_type, m.target_id = data['type'], data['target']
        m.search_task_id, m.search_revision = data['task'], data['version']
        m.search_valid_until = fixture.sa.rospy.Time.from_sec(data['until'])
        if data['cells']:
            m.cell_ix, m.cell_iy = data['cells'][0]
        # Tracking XY is irrelevant to this test. The recorder did not retain it;
        # use the rounded 2567.400 agent-log goal, not invented "exact" coordinates.
        m.target_x, m.target_y = data['points'][0] if data['points'] else (114.5, 42.3)
        return m

    def replay_images(self):
        for e in RECORDED['events']:
            if e['event']=='assignment':
                continue
            self.now = e['ros']
            msg = NS(data=json.dumps(e['data']))
            if e['event']=='processed_frame':
                self.a.local_xy = (e['data']['x'], e['data']['y'])
                self.a.local_z = e['data']['z']
                self.a.yaw = e['data']['yaw']
                self.a._processed_camera_frame_cb(msg)
            else:
                self.a._visual_candidate_cb(msg)

    def cancel(self):
        e = self.recorded_tasks[0]
        self.now = e['ros']
        self.a._assign_cb(self.assignment(e['data']))

    def follow(self, target=None):
        e = self.recorded_tasks[1]
        self.now = e['ros']
        data = dict(e['data'])
        if target is not None:
            data['target'] = target
        self.a._assign_cb(self.assignment(data))

    def test_real_cancel_then_new_tracking_preserves_exact_last_image_once(self):
        self.cancel()
        self.assertEqual(self.revisits, [])  # wait for same-person takeover information
        self.assertIsNone(self.a._camera_cue)
        self.assertIsNone(self.a.assignment)
        self.follow()
        self.assertEqual(len(self.revisits), 1)
        event = json.loads(self.revisits[0].data)
        origin = self.original['view_origin']
        self.assertEqual((event['x'], event['y']), (origin['x'], origin['y']))
        self.assertEqual(event['image_sample_s'], self.original['sample_s'])
        self.assertEqual(event['origin_sample_s'], self.original['sample_s'])
        self.assertEqual(event['bearing'], self.original['bearing'])
        self.assertFalse(event['real_detection'])
        self.assertFalse(event['report_eligible'])
        parsed = parse_failed_view(event, self.now)
        self.assertEqual(parsed['expires_s'], self.original['sample_s']+35.)
        self.assertEqual(self.a.assignment.target_id, 't1')
        self.assertEqual(self.a._target_to_orbit, (114.5, 42.3))
        self.assertEqual((self.commands, self.detections, self.a.targets), ([], [], {}))
        duplicate = self.recorded_tasks[2]
        self.now = duplicate['ros']
        self.a._assign_cb(self.assignment(duplicate['data']))
        self.a._finish_interrupted_camera_cue()
        self.assertEqual(len(self.revisits), 1)

    def test_direct_tracking_assignment_without_cancel_preserves_cue(self):
        self.follow()
        self.assertEqual(len(self.revisits), 1)
        self.assertEqual(self.a.assignment.target_id, 't1')

    def test_cancel_only_control_sweep_is_bounded_and_has_no_new_motion(self):
        self.cancel()
        cancelled_s = self.now
        self.now = cancelled_s+.249
        self.a._control()
        self.assertEqual(self.revisits, [])
        self.now = cancelled_s+.251
        self.a._control()
        self.a._control()
        self.assertEqual(len(self.revisits), 1)
        self.assertTrue(all(v==(0., 0.) for v in self.commands))
        self.assertIsNone(getattr(self.a, '_interrupted_camera_cue', None))

    def test_already_localized_cue_has_no_extra_revisit(self):
        self.a._camera_cue['xy'] = (106.6, 39.2)
        self.cancel(); self.follow('t3')
        self.assertEqual(self.revisits, [])

    def test_fresh_same_ray_qualified_own_view_resolves_cue(self):
        cue = self.a._camera_cue
        origin = cue['view_origin']
        self.a._visual_observations['t3'] = dict(eligible=True, source_uav=self.a.uav_id,
            sample_s=cue['sample_s'], x=origin['x']+10.*math.cos(cue['bearing']),
            y=origin['y']+10.*math.sin(cue['bearing']))
        self.cancel(); self.follow('t3')
        self.assertEqual(self.revisits, [])

    def test_old_same_colour_localization_does_not_erase_new_camera_picture(self):
        cue = self.a._camera_cue
        self.a._visual_observations['t3'] = dict(eligible=True, source_uav=self.a.uav_id,
            sample_s=cue['sample_s']-5., x=106.6, y=39.2)
        self.cancel(); self.follow('t3')
        self.assertEqual(len(self.revisits), 1)

    def test_reliable_spatial_peer_avoids_duplicate_revisit_but_same_colour_is_not_enough(self):
        self.a._team_tracks['t3'] = {'peer'}
        bearing = self.original['bearing']
        x, y = self.a.world_xy
        self.a._team_visual[('peer', 't3')] = dict(hits=4, since_s=2566.,
            sample_s=2567.264, x=x+10.*math.cos(bearing), y=y+10.*math.sin(bearing))
        self.cancel(); self.follow()
        self.assertEqual(self.revisits, [])

    def test_different_instance_or_stale_peer_cannot_erase_real_candidate(self):
        self.a._team_tracks['t3'] = {'peer'}
        x, y = self.a.world_xy
        self.a._team_visual[('peer', 't3')] = dict(hits=4, since_s=2566.,
            sample_s=2567.264, x=x+10., y=y+10.)
        self.cancel(); self.follow('t3')
        self.assertEqual(len(self.revisits), 1)

    def test_red_assignment_does_not_claim_an_unbound_other_red_by_colour(self):
        self.a._camera_cue.update(candidate_id='red:19', color='red', target_id=None)
        self.cancel(); self.follow('t4')
        self.assertEqual(len(self.revisits), 1)
        self.assertEqual(json.loads(self.revisits[0].data)['candidate_id'], 'red:19')

    def test_missing_or_unmatched_origin_has_no_legacy_position_fallback(self):
        for origin in (None, {}, dict(self.original['view_origin'], sample_s=2564.672),
                       dict(self.original['view_origin'], x=float('nan'))):
            self.a._camera_cue = dict(self.original, view_origin=origin)
            self.now = 2567.4
            self.a._stage_interrupted_camera_cue(self.now)
            self.now += .3
            self.a._finish_interrupted_camera_cue()
        self.assertEqual(self.revisits, [])

    def test_expired_or_untrusted_cue_does_not_survive_interrupt(self):
        for changes in (dict(until=2567.), dict(sample_s=2564.)):
            self.a._camera_cue = dict(self.original, **changes)
            self.a._stage_interrupted_camera_cue(2567.4)
        self.assertIsNone(getattr(self.a, '_interrupted_camera_cue', None))
        self.a._camera_cue = copy.deepcopy(self.original)
        self.a._world_trusted = lambda: False
        self.cancel(); self.follow()
        self.assertEqual(self.revisits, [])

    def test_safety_loss_or_missing_publisher_before_flush_discards_once(self):
        self.cancel()
        self.a._landing = True
        self.follow()
        self.assertEqual(self.revisits, [])
        self.assertIsNone(getattr(self.a, '_interrupted_camera_cue', None))

    def test_missing_publisher_after_cancel_consumes_slot_once(self):
        self.cancel()
        self.a._candidate_missed_pub = None
        self.now += .3
        self.a._finish_interrupted_camera_cue()
        self.a._finish_interrupted_camera_cue()
        self.assertIsNone(getattr(self.a, '_interrupted_camera_cue', None))
        self.assertEqual(self.revisits, [])

    def test_unknown_disarmed_manual_or_landing_state_cannot_stage_a_revisit(self):
        for state in (None, NS(), NS(mode='OFFBOARD'), NS(mode='MANUAL', armed=True),
                      NS(mode='AUTO.LAND', armed=True), NS(mode='OFFBOARD', armed=False)):
            self.a._camera_cue = copy.deepcopy(self.original)
            self.a.state = state
            self.now = 2567.4
            self.a._stage_interrupted_camera_cue(self.now)
            self.assertIsNone(getattr(self.a, '_interrupted_camera_cue', None))

    def test_raw_image_ttl_and_future_image_are_not_renewed_by_notification(self):
        for sample in (2520., 2568.):
            self.a._camera_cue = dict(self.original, sample_s=sample,
                approach_started=False, until=2600.,
                view_origin=dict(self.original['view_origin'], sample_s=sample))
            self.a._stage_interrupted_camera_cue(2567.4)
            self.assertIsNone(getattr(self.a, '_interrupted_camera_cue', None))

    def test_ros_clock_reset_drops_waiting_slot_instead_of_waiting_forever(self):
        self.cancel()
        self.now = 2567.3  # before cancellation, but after its bound image sample
        self.a._finish_interrupted_camera_cue()
        self.assertIsNone(getattr(self.a, '_interrupted_camera_cue', None))
        self.follow()
        self.assertEqual(self.revisits, [])

    def test_expired_or_newly_unhealthy_slot_cannot_publish_on_flush(self):
        for state in (NS(), NS(mode='MANUAL', armed=True), NS(mode='OFFBOARD', armed=False)):
            self.a.state = NS(**RECORDED['safety_state_near_interrupt']['state'])
            self.a.assignment = None
            self.a._camera_cue = copy.deepcopy(self.original)
            self.now = 2567.4
            self.a._stage_interrupted_camera_cue(self.now)
            self.assertIsNotNone(getattr(self.a, '_interrupted_camera_cue', None))
            self.a.state = state
            self.now += .3
            self.a._finish_interrupted_camera_cue()
            self.assertIsNone(getattr(self.a, '_interrupted_camera_cue', None))
        self.a.state = NS(**RECORDED['safety_state_near_interrupt']['state'])
        self.a._camera_cue = copy.deepcopy(self.original)
        self.now = 2567.4
        self.a._stage_interrupted_camera_cue(self.now)
        self.now = self.original['until']+.01
        self.a._finish_interrupted_camera_cue()
        self.assertIsNone(getattr(self.a, '_interrupted_camera_cue', None))
        self.assertEqual(self.revisits, [])

    def test_ambiguous_or_nonunique_latest_candidate_is_not_preserved(self):
        packet = next(e['data'] for e in reversed(RECORDED['events']) if e['event']=='candidate')
        for changes in (dict(appearance_status='ambiguous'), dict(association_unique=False)):
            self.a._camera_cue = copy.deepcopy(self.original)
            self.now = 2567.36
            self.a._visual_candidate_cb(NS(data=json.dumps(dict(packet, **changes))))
            self.assertFalse(self.a._camera_cue['interrupt_revisit_ok'])
            self.a._stage_interrupted_camera_cue(2567.4)
        self.assertIsNone(getattr(self.a, '_interrupted_camera_cue', None))


if __name__ == '__main__':
    unittest.main()
