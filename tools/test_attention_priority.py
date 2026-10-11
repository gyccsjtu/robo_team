"""Search attention must survive routing while remaining bounded and sensor-only."""
import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa


class AttentionPriorityTests(unittest.TestCase):
    def setUp(self):
        self.a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a = self.a
        a.uav_id = 'u0'
        a.local_xy, a.offset, a.local_z = (0., 0.), (0., 0.), 3.
        a.yaw = 0.
        a.assignment = NS(task_type=0, cell_ix=0, cell_iy=0,
                          target_x=30., target_y=0., target_id='')
        a._team_tracks, a._team_visual = {}, {}
        a._visual_observations, a._eligible_observations = {}, {}
        a._search_scan_key = (0, 0, 30., 0.)
        a._search_scan_started_t = None
        a._camera_cue = None
        a._camera_cue_retry_after = {}
        a._landing = False
        a._takeoff_done = True
        a._track_assigned_id = None
        a._confirm_close = False
        a._giveup_until, a._t_seen = {}, {}
        a._orbit_target = None
        a._world_trusted = lambda: True
        a._discard_old_plan = lambda: None
        self.commands, self.detections, self.revisits = [], [], []
        a._send_vel = lambda *args: self.commands.append(args)
        a.detect_pub = NS(publish=self.detections.append)
        a._candidate_missed_pub = NS(publish=self.revisits.append)
        self.now = 100.
        self.clock = patch.object(sa.rospy.Time, 'now',
            side_effect=lambda: sa.rospy.Time.from_sec(self.now))
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def cue(self, cid='blue:1', bearing=.2, hits=3, sample=None, confidence=.95):
        self.a._visual_candidate_cb(NS(data=json.dumps(dict(
            source_uav='u0', real_detection=True, purpose='camera_reacquire',
            candidate_id=cid, bearing=bearing, hits=hits,
            sample_s=self.now if sample is None else sample, confidence=confidence))))

    def image(self, tid='t1', source='u0', x=8., y=0., eligible=True):
        self.a._visual_source_cb(NS(data=json.dumps(dict(
            source_uav=source, target_id=tid, real_detection=True,
            sample_s=sa.rospy.Time.from_sec(self.now).to_sec(),
            x=x, y=y, confidence=.95, range_m=8.,
            official_view_ok=eligible))))

    def test_single_frame_cannot_interrupt_search(self):
        self.cue(hits=1)
        self.assertIsNone(self.a._camera_cue)

    def test_full_small_person_triggers_bounded_safe_approach_and_expiry(self):
        def send():
            self.a._visual_candidate_cb(NS(data=json.dumps(dict(
                source_uav='u0',real_detection=True,purpose='camera_reacquire',
                candidate_id='brown:1',hits=4,sample_s=self.now,
                bearing=0.,confidence=.8,box=[340.,200.,380.,300.],
                width=752,height=480,camera_tilt=0.))))
        self.a._scan = NS(angle_min=-math.pi,angle_increment=math.pi/180.,
                          ranges=[float('inf')]*361,range_min=.1,range_max=30.)
        self.a._scan_t = self.now
        send()
        self.a._control()
        self.assertGreater(self.commands[-1][0],0.)
        self.assertEqual(self.a._camera_cue['until'],106.)
        self.assertEqual(self.detections,[])
        self.now = 105.9
        self.a._scan_t = self.now
        send()
        self.assertEqual(self.a._camera_cue['until'],106.)
        self.now = 106.1
        self.assertFalse(self.a._hold_camera_candidate())
        self.assertEqual(self.detections,[])

    def test_image_approach_cannot_move_without_lidar(self):
        self.a._visual_candidate_cb(NS(data=json.dumps(dict(
            source_uav='u0',real_detection=True,purpose='camera_reacquire',
            candidate_id='green:1',hits=4,sample_s=self.now,bearing=0.,confidence=.8,
            box=[340.,200.,380.,300.],width=752,height=480,camera_tilt=0.))))
        self.a._control()
        self.assertEqual(self.commands[-1],(0.,0.))

    def test_multiframe_moderate_confidence_interrupts_search_for_every_color(self):
        for color in ('blue', 'green', 'brown', 'red', 'white'):
            with self.subTest(color=color):
                self.a._camera_cue = None
                self.commands.clear()
                self.cue(cid=color + ':100', confidence=.585)
                self.assertIsNotNone(self.a._camera_cue)
                self.a._control()
                self.assertEqual(self.commands, [(0., 0.)])
                self.assertEqual(self.a.assignment.task_type, 0)
                self.assertEqual(self.detections, [])

    def test_moderate_single_frame_or_stale_cue_cannot_interrupt(self):
        self.cue(confidence=.6, hits=1)
        self.assertIsNone(self.a._camera_cue)
        self.cue(confidence=.6, sample=self.now-1.)
        self.assertIsNone(self.a._camera_cue)

    def test_attention_interrupts_before_waypoint_or_scan(self):
        self.cue()
        self.a._control()
        self.assertEqual(self.commands, [(0., 0.)])
        self.assertAlmostEqual(math.atan2(self.a._look_at[1], self.a._look_at[0]), .2)

    def test_search_reassignment_preserves_person_and_resumes_latest_route(self):
        self.cue()
        self.a._assign_cb(NS(uav_id='u0', task_type=0, target_id='',
                             cell_ix=1, cell_iy=0, target_x=37., target_y=0.))
        self.assertTrue(self.a._hold_camera_candidate())
        self.now = 102.1
        self.assertFalse(self.a._hold_camera_candidate())
        self.assertEqual(self.a.assignment.target_x, 37.)
        self.assertEqual(len(self.revisits), 1)

    def test_new_frames_update_bearing_without_extending_deadline(self):
        self.cue()
        self.now = 101.8
        self.cue(bearing=.5, confidence=.6)
        self.assertEqual(self.a._camera_cue['until'], 102.)
        self.a._hold_camera_candidate()
        self.assertAlmostEqual(math.atan2(self.a._look_at[1], self.a._look_at[0]), .5)
        self.cue(bearing=-1., sample=101.7)  # out of order
        self.assertEqual(self.a._camera_cue['bearing'], .5)

    def test_new_pixel_candidate_cannot_steal_active_view_even_same_color(self):
        self.cue()
        self.now = 100.2
        self.cue(cid='blue:2', bearing=-1.)
        self.assertEqual(self.a._camera_cue['candidate_id'], 'blue:1')

    def test_stale_owner_does_not_suppress_candidate(self):
        self.a._team_tracks = {'t1': {'u1'}}
        self.cue()
        self.assertTrue(self.a._hold_camera_candidate())

    def test_distant_same_color_owner_cannot_suppress_local_candidate(self):
        self.a._team_tracks = {'t1': {'u1'}}
        for stamp in (100., 100.2, 100.4):
            self.now = stamp
            self.image(source='u1', x=140., y=20.)
        self.cue(bearing=.2)
        self.assertTrue(self.a._hold_camera_candidate())

    def test_same_color_owner_on_different_bearing_cannot_suppress_candidate(self):
        self.a._team_tracks = {'t1': {'u1'}}
        for stamp in (100., 100.2, 100.4):
            self.now = stamp
            self.image(source='u1', x=0., y=8.)
        self.cue(bearing=0.)
        self.assertTrue(self.a._hold_camera_candidate())

    def test_detector_id_churn_cannot_bypass_view_retry_budget(self):
        self.cue()
        self.now = 102.1
        self.a._hold_camera_candidate()
        self.cue(cid='blue:99', bearing=.21)
        self.assertIsNone(self.a._camera_cue)
        self.now = 104.1
        self.cue(cid='blue:100', bearing=.21)
        self.assertIsNotNone(self.a._camera_cue)
        self.now = 106.2
        self.a._hold_camera_candidate()
        self.now = 108.2  # repeated failed view gets four seconds, not two
        self.cue(cid='blue:101', bearing=.22)
        self.assertIsNone(self.a._camera_cue)
        self.cue(cid='blue:102', bearing=-1.)  # genuinely different view
        self.assertIsNotNone(self.a._camera_cue)

    def test_localization_can_promote_after_pixel_retry_is_exhausted(self):
        self.cue(bearing=0.)
        self.now = 102.1
        self.a._hold_camera_candidate()
        for stamp in (102.2, 102.4, 102.6):
            self.now = stamp
            self.image()
        self.assertIsNotNone(self.a._camera_cue)
        self.assertEqual(self.a._camera_cue['xy'], (8., 0.))

    def test_new_camera_position_gets_a_fresh_bounded_look(self):
        self.cue()
        self.now = 102.1
        self.a._hold_camera_candidate()
        self.a.local_xy = (4., 0.)
        self.cue(cid='blue:99')
        self.assertIsNotNone(self.a._camera_cue)

    def test_only_continuous_real_peer_images_release_local_attention(self):
        self.a._team_tracks = {'t1': {'u1'}}
        self.cue()
        self.image(source='u1')
        self.assertTrue(self.a._hold_camera_candidate())
        for stamp in (100.2, 100.4):
            self.now = stamp
            self.image(source='u1')
        self.assertFalse(self.a._hold_camera_candidate())
        self.assertEqual(self.detections, [])  # never label peer images as own
        self.assertEqual(self.a._visual_observations, {})
        self.now = 102.
        self.cue(cid='blue:2')  # assigned peer has now lost contact
        self.assertTrue(self.a._hold_camera_candidate())

    def test_localized_track_waits_for_dispatch_and_updates_world_point(self):
        for stamp in (100., 100.2, 100.4):
            self.now = stamp
            self.image()
        self.assertTrue(self.a._hold_camera_candidate())
        end = self.a._camera_cue['until']
        self.now = 101.
        self.image(y=1.)
        self.a._hold_camera_candidate()
        self.assertEqual(self.a._look_at, (8., 1.))
        self.assertEqual(self.a._camera_cue['until'], end)
        self.assertEqual(len(self.detections), 4)
        self.assertEqual(self.a.assignment.task_type, 0)  # no private task claim

    def test_same_color_slot_jump_cannot_turn_an_active_local_view_away(self):
        for stamp in (100., 100.2, 100.4):
            self.now = stamp
            self.image()
        self.assertEqual(self.a._camera_cue['xy'], (8., 0.))
        self.now = 100.6
        self.image(x=140., y=20.)
        self.assertEqual(self.a._camera_cue['xy'], (8., 0.))

    def test_localized_peer_must_match_position_not_just_color(self):
        self.a._team_tracks = {'t1': {'u1'}}
        for stamp in (100., 100.2, 100.4):
            self.now = stamp
            self.image(source='u1', x=15., y=0.)
            self.image(source='u0', x=8., y=0.)
        self.assertTrue(self.a._hold_camera_candidate())
        self.assertEqual(self.a._camera_cue['xy'], (8., 0.))

    def test_pixel_promotes_to_localization_once_without_infinite_lease(self):
        self.cue(bearing=0.)
        for stamp in (100.1, 100.3, 100.5):
            self.now = stamp
            self.image()
        self.assertEqual(self.a._camera_cue['until'], 104.)
        self.now = 103.9
        self.image()
        self.assertEqual(self.a._camera_cue['until'], 104.)
        self.now = 104.1
        self.a._hold_camera_candidate()
        self.assertIsNone(self.a._camera_cue)

    def test_active_tracking_and_reacquisition_cannot_be_preempted(self):
        self.cue()
        self.a._assign_cb(NS(uav_id='u0', task_type=1, target_id='t2',
                             target_x=8., target_y=1.))
        self.assertIsNone(self.a._camera_cue)
        self.now = 100.2
        self.cue(cid='white:1')
        self.assertIsNone(self.a._camera_cue)
        self.assertEqual(self.a.assignment.target_id, 't2')

    def test_failed_candidate_can_retry_without_global_eight_second_lockout(self):
        self.cue()
        self.now = 102.1
        self.a._hold_camera_candidate()
        self.cue(cid='white:2')
        self.assertEqual(self.a._camera_cue['candidate_id'], 'white:2')
        self.now = 104.2
        self.a._hold_camera_candidate()
        self.cue()
        self.assertEqual(self.a._camera_cue['candidate_id'], 'blue:1')

    def test_search_heading_slews_across_wrap_and_replan(self):
        self.a.yaw = math.radians(179.)
        self.a._set_search_heading(math.radians(-179.))
        heading = self.a._search_heading[1]
        self.assertLessEqual(abs(heading-self.a.yaw), sa.SEARCH_HEADING_RATE/sa.CTRL_RATE+1e-9)
        self.now += .05
        self.a._set_search_heading(0.)
        self.assertLessEqual(abs(self.a._search_heading[1]-heading),
                             sa.SEARCH_HEADING_RATE*.05+1e-9)

    def test_search_sweep_is_slow_bounded_and_stops_for_person_or_obstacle(self):
        a = self.a
        a._radar_prev = (20., 20., 20.)
        samples = []
        with patch.object(sa, 'SEARCH_SWEEP_DEG', 20.), \
                patch.object(sa, 'SEARCH_SWEEP_PERIOD_S', 12.):
            for i in range(241):
                self.now = 100. + i*.05
                a._scan_t = sa.rospy.Time.from_sec(self.now).to_sec()
                samples.append(a._search_sweep_bearing(0., 3.))
            self.assertAlmostEqual(max(samples), math.radians(20.), places=5)
            self.assertAlmostEqual(min(samples), -math.radians(20.), places=5)
            self.assertLess(max(abs(b-c)/.05 for b, c in zip(samples, samples[1:])),
                            math.radians(11.))
            self.cue()
            self.assertEqual(a._search_sweep_bearing(0., 3.), 0.)
            a._camera_cue = None
            a._radar_prev = (4., 20., 20.)
            self.assertEqual(a._search_sweep_bearing(0., 3.), 0.)

    def test_sweep_stops_at_turn_stale_scan_and_tracking(self):
        a = self.a
        a._radar_prev = (20., 20., 20.)
        a._scan_t = self.now
        a._search_sweep_state = (self.now-.05, 0., 3.)
        turn = math.radians(45.)
        self.assertEqual(a._search_sweep_bearing(turn, 3.), turn)
        a._scan_t = self.now-2.
        self.assertEqual(a._search_sweep_bearing(0., 3.), 0.)
        a._scan_t = self.now
        a.assignment.task_type = 1
        self.assertEqual(a._search_sweep_bearing(0., 3.), 0.)

    def test_collision_avoidance_still_overrides_attention_stop(self):
        self.cue()
        self.a._friend_positions = {'u1': (1., 0., 3.)}
        self.a._apply_friend_avoidance = lambda vx, vy: (-.4, .1)
        self.a._hold_camera_candidate()
        self.assertEqual(self.commands[-1], (-.4, .1))
        self.assertAlmostEqual(math.atan2(self.a._look_at[1], self.a._look_at[0]), .2)

    def test_missing_sector_scan_begins_near_current_view(self):
        a = self.a
        a.yaw = math.pi
        a._search_scan_yaw0 = None
        a._search_scan_done = a._search_scan_failed = False
        a._search_scan_seen = {1}
        a._search_scan_step()
        self.assertEqual(a._search_scan_phase, 2)
        self.assertAlmostEqual(a._search_heading[1], math.pi)

    def test_unassigned_peer_image_cannot_establish_local_or_owner_evidence(self):
        self.image(source='u1')
        self.assertEqual(self.a._team_visual, {})
        self.assertEqual(self.a._visual_observations, {})
        self.assertEqual(self.detections, [])


if __name__ == '__main__':
    unittest.main()
