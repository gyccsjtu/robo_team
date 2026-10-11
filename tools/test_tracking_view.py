"""Replay the white actor's low-in-frame view and obstacle-stop failure."""
import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa
from tracking_view import (box_from_foot, parse_tracking_view, image_altitude_goal,
                           view_needs_space, update_altitude_hold,
                           bound_pixel_candidate, retain_pixel_view)
from perception import perception_real as pr


class TrackingViewTests(unittest.TestCase):
    def test_repeated_tilt_pauses_but_does_not_erase_height_intent(self):
        state = None
        for i in range(50):
            tilt = math.radians([0., 9., 0., 0., 9.][i % 5])
            view = self.payload(sample_s=100.+i*.2, camera_tilt=tilt,
                                box=[340., 240., 380., 400.])
            state = update_altitude_hold(state, view, 2.6, 8., 2., 4.2)
        self.assertGreater(state['goal'], 2.7)

    def setUp(self):
        clock = patch.object(sa.rospy.Time, 'now',
                             return_value=sa.rospy.Time.from_sec(100.1))
        clock.start()
        self.addCleanup(clock.stop)

    def payload(self, **kw):
        data = dict(source_uav='u0', target_id='t3', real_detection=True,
                    track_id=7, sample_s=100., confidence=.88, hits=10,
                    box=[325., 360., 375., 450.], width=752, height=480,
                    fy=376., bearing=.1, x=8., y=0., camera_tilt=0., geometry_ok=True)
        data.update(kw)
        return data

    def agent(self):
        a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id = 'u0'
        a.assignment = NS(task_type=1, target_id='t3')
        a.local_xy, a.offset, a.local_z = (0., 0.), (0., 0.), 3.3
        a.altitude_layer = 3.5
        a.targets = {'t3': (8., 0., 1., 0.)}
        a._orbit_target = 't3'
        a._scan_t = 100.
        a._radar_prev = (20., 20., 20.)
        a._visual_observations = {}
        a._confirm_start, a._last_confirm_t = 0., 0.
        return a

    def send(self, a, data=None, now=100.1):
        with patch.object(sa.rospy.Time, 'now', return_value=sa.rospy.Time.from_sec(now)):
            a._tracking_view_cb(NS(data=json.dumps(data or self.payload())))

    def sustained_edge(self, a):
        for t in (99.2, 99.4, 99.6, 99.8, 100.):
            self.send(a, self.payload(sample_s=t), now=t+.1)

    def test_reconstructs_actual_centre_not_the_logged_foot_pixel(self):
        box = box_from_foot((350., 445.5), (50., 90.))
        self.assertEqual(box, [325., 360., 375., 450.])
        self.assertEqual((box[1]+box[3])/2., 405.)

    def test_low_white_view_requests_bounded_descent_before_cropping(self):
        a = self.agent()
        self.sustained_edge(a)
        self.assertLess(a._tracking_image['altitude_goal'], a.local_z)
        self.assertGreaterEqual(a._tracking_image['altitude_goal'], sa.MIN_CRUISE_ALT)
        with patch.object(sa.rospy.Time, 'now', return_value=sa.rospy.Time.from_sec(100.2)):
            actual = a.altitude_layer+a._compute_desired_altitude()
        self.assertEqual(actual, a._tracking_image['altitude_goal'])
        self.assertLessEqual(abs(actual-a.local_z), .35+1e-9)
        self.assertEqual(a._visual_observations, {})
        self.assertEqual((a._confirm_start, a._last_confirm_t), (0., 0.))

    def test_repeat_frame_cannot_integrate_altitude_error_again(self):
        a = self.agent()
        self.send(a)
        goal = a._tracking_image['altitude_goal']
        a.local_z -= .3
        self.send(a, now=100.3)
        self.assertEqual(a._tracking_image['altitude_goal'], goal)

    def test_same_color_same_frame_selects_nearest_assigned_location(self):
        a = self.agent()
        self.send(a,self.payload(track_id=1,x=10.,confidence=.95))
        self.send(a,self.payload(track_id=2,x=8.,confidence=.5))
        self.send(a,self.payload(track_id=3,x=9.,confidence=.98))
        self.assertEqual(a._tracking_image['track_id'],2)
        self.assertEqual(a._visual_observations,{})

    def test_bound_instance_blocks_nearby_same_color_distractor_view(self):
        a = self.agent()
        a._camera_instance = {'t3': dict(candidate_id='session:7', sample_s=100.)}
        self.send(a, self.payload(world_candidate_id='session:7', x=8.2))
        self.send(a, self.payload(world_candidate_id='session:8', x=8., confidence=.99))
        self.assertEqual(a._tracking_image['world_candidate_id'], 'session:7')
        self.assertEqual(a._tracking_image['x'], 8.2)

    def test_expired_instance_allows_geometric_reacquisition(self):
        a = self.agent()
        a._camera_instance = {'t3': dict(candidate_id='session:7', sample_s=98.)}
        self.send(a, self.payload(world_candidate_id='session:8'))
        self.assertEqual(a._tracking_image['world_candidate_id'], 'session:8')

    def test_own_qualified_instance_change_clears_old_view_and_streak(self):
        a = self.agent()
        self.send(a)
        a._camera_instance = {'t3': dict(candidate_id='session:old', sample_s=99.9)}
        a._team_visual = {('u0', 't3'): dict(candidate_id='session:old',
            sample_s=99.9, hits=10, since_s=97., eligible=True)}
        data = dict(source_uav='u0', target_id='t3', real_detection=True,
                    candidate_id='session:new', sample_s=100., confidence=.9,
                    x=8., y=0., range_m=8., official_view_ok=True)
        a._visual_source_cb(NS(data=json.dumps(data)))
        self.assertIsNone(a._tracking_image)
        self.assertIsNone(a._tracking_altitude)
        self.assertEqual(a._look_at, (8., 0.))
        self.assertEqual(a._team_visual[('u0', 't3')]['hits'], 1)
        self.assertEqual(a._camera_instance['t3']['candidate_id'], 'session:new')

    def test_unqualified_image_cannot_rebind_the_camera_instance(self):
        a = self.agent()
        self.send(a)
        old_view = a._tracking_image
        a._camera_instance = {'t3': dict(candidate_id='session:old', sample_s=99.9)}
        data = dict(source_uav='u0', target_id='t3', real_detection=True,
                    candidate_id='session:new', sample_s=100., confidence=.9,
                    x=8., y=0., range_m=8., official_view_ok=False)
        a._visual_source_cb(NS(data=json.dumps(data)))
        self.assertIs(a._tracking_image, old_view)
        self.assertEqual(a._camera_instance['t3']['candidate_id'], 'session:old')

    def test_first_binding_clears_earlier_wrong_instance_view(self):
        for expired in (False, True):
            a = self.agent()
            if expired:
                a._camera_instance = {'t3': dict(candidate_id='session:right', sample_s=98.)}
            self.send(a, self.payload(world_candidate_id='session:wrong', candidate_id='white:8'))
            data = dict(source_uav='u0', target_id='t3', real_detection=True,
                        candidate_id='session:right', sample_s=100., confidence=.9,
                        x=8., y=0., range_m=8., official_view_ok=True)
            a._visual_source_cb(NS(data=json.dumps(data)))
            self.assertIsNone(a._tracking_image)
            self.assertIsNone(a._tracking_altitude)

    def test_pixel_continuity_preserves_and_checks_world_instance(self):
        a = self.agent()
        self.send(a, self.payload(world_candidate_id='session:wrong', candidate_id='white:8'))
        raw = self.payload(candidate_id='white:8', sample_s=100.2, purpose='camera_reacquire')
        continued = retain_pixel_view(a._tracking_image, raw, 'u0', 't3', 100.3)
        self.assertEqual(continued['world_candidate_id'], 'session:wrong')
        a._camera_instance = {'t3': dict(candidate_id='session:right', sample_s=100.)}
        a._visual_candidate_cb(NS(data=json.dumps(raw)))
        self.assertIsNone(a._tracking_image)

    def test_bound_pixel_survives_short_bad_geometry_without_refreshing_position(self):
        a = self.agent()
        self.send(a,self.payload(candidate_id='white:11'))
        for t in (100.2,100.4,100.6,100.8,101.,101.2,101.4,101.6,101.8):
            raw = self.payload(candidate_id='white:11',sample_s=t,bearing=.15)
            raw['purpose'] = 'camera_reacquire'
            # Image-only messages contain no actor world coordinate.
            raw.pop('x'); raw.pop('y'); raw.pop('target_id')
            with patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time.from_sec(t+.1)):
                a._visual_candidate_cb(NS(data=json.dumps(raw)))
            self.assertAlmostEqual(a._tracking_image['sample_s'],t)
            self.assertEqual(a._tracking_image['position_sample_s'],100.)
            self.assertEqual((a._tracking_image['x'],a._tracking_image['y']),(8.,0.))
            self.assertFalse(a._tracking_image['geometry_ok'])
        self.assertEqual(a._visual_observations,{})
        raw['sample_s'] = 102.
        self.assertIsNone(retain_pixel_view(a._tracking_image,raw,'u0','t3',102.1))

    def test_pixel_cannot_rebind_by_color_or_renew_after_gap(self):
        previous = parse_tracking_view(self.payload(candidate_id='white:11'),'u0','t3',100.1)
        for change in (dict(candidate_id='white:12'),dict(source_uav='u1'),
                       dict(sample_s=101.),dict(bearing=2.),dict(real_detection=False)):
            raw = self.payload(candidate_id='white:11',sample_s=100.2)
            raw.update(change)
            self.assertIsNone(retain_pixel_view(previous,raw,'u0','t3',100.3))

    def test_geometry_can_replace_same_frame_pixel_fallback(self):
        a = self.agent()
        self.send(a,self.payload(candidate_id='white:11'))
        raw = self.payload(candidate_id='white:11',sample_s=100.2)
        a._tracking_image = retain_pixel_view(a._tracking_image,raw,'u0','t3',100.3)
        self.assertTrue(a._tracking_image['pixel_only'])
        self.send(a,raw,now=100.3)
        self.assertFalse(a._tracking_image.get('pixel_only',False))
        self.assertEqual(a._tracking_image['position_sample_s'],100.2)

    def test_pixel_binding_requires_unique_overlapping_same_frame_box(self):
        cue = dict(candidate_id='white:11',box=[325.,360.,375.,450.])
        box = cue['box']
        self.assertEqual(bound_pixel_candidate([cue],'white',box),'white:11')
        self.assertIsNone(bound_pixel_candidate([cue],'blue',box))
        self.assertIsNone(bound_pixel_candidate([cue,dict(cue,candidate_id='white:12')],
                                                'white',box))

    def test_half_second_gap_cannot_switch_height_back_to_old_range_controller(self):
        a = self.agent()
        self.sustained_edge(a)
        held = a._tracking_altitude['goal']
        with patch.object(sa.rospy.Time, 'now', return_value=sa.rospy.Time.from_sec(100.8)):
            self.assertIsNone(a._fresh_tracking_view(100.8))
            self.assertAlmostEqual(a.altitude_layer+a._compute_desired_altitude(), held)
        a.local_z = 3.05
        with patch.object(sa.rospy.Time, 'now', return_value=sa.rospy.Time.from_sec(103.)):
            self.assertAlmostEqual(a.altitude_layer+a._compute_desired_altitude(), a.local_z)

    def test_interior_person_does_not_trigger_altitude_hunt(self):
        a = self.agent()
        a.local_z = 3.0  # within the 8 m / pitch-reserve height window
        for i in range(20):
            t = 100.+i*.2
            self.send(a, self.payload(sample_s=t, box=[325., 190., 375., 310.]), now=t+.1)
        self.assertAlmostEqual(a._tracking_altitude['goal'], 3.0)

    def test_transient_tilt_and_single_cropped_frame_do_not_change_height(self):
        a = self.agent()
        self.send(a, self.payload(box=[325., 390., 375., 480.]))
        self.assertAlmostEqual(a._tracking_altitude['goal'], a.local_z)
        for i in range(1, 10):
            t = 100.+i*.2
            self.send(a, self.payload(sample_s=t, box=[325., 390., 375., 480.],
                                      camera_tilt=.2), now=t+.1)
        self.assertAlmostEqual(a._tracking_altitude['goal'], a.local_z)

    def test_sustained_level_edge_correction_is_slow_and_keeps_geometry_height(self):
        a = self.agent()
        last = a.local_z
        for i in range(50):
            t = 100.+i*.2
            self.send(a, self.payload(sample_s=t, box=[325., 390., 375., 480.]), now=t+.1)
            goal = a._tracking_altitude['goal']
            self.assertLessEqual(abs(goal-last), .03+1e-8)
            self.assertGreaterEqual(goal, 2.6)
            a.local_z = goal
            last = goal
        self.assertLess(last, 3.3)

    def test_recorded_brown_pixels_and_pitch_do_not_cause_height_sawtooth(self):
        data = json.loads((ROOT/'tools/fixtures/tracking_view_032614_brown.json').read_text())
        state, goals = None, []
        for frame in data['frames']:
            view = dict(frame, target_id='t2', height=480., fy=376.)
            state = update_altitude_hold(state, view, 3.0, frame['range_m'], 2.6, 4.2)
            goals.append(state['goal'])
        self.assertGreater(len(goals), 50)
        # Fixed-altitude input isolates the controller from the recorded bad
        # flight response; this is not a Gazebo dynamics or judge replay.
        self.assertLess(max(goals)-min(goals), .3)

    def test_old_layer_discount_cannot_drive_centred_view_down_each_frame(self):
        a = self.agent()
        a._alt_sag = .5
        self.send(a, self.payload(box=[325., 230., 375., 360.]))
        final_goal = a.altitude_layer+a._compute_desired_altitude()-a._alt_sag
        self.assertAlmostEqual(final_goal, a.local_z)

    def test_pixel_yaw_and_height_reach_position_output_without_undoing_stop(self):
        a = self.agent()
        self.sustained_edge(a)
        a.yaw, a._look_at = 0., (8., 0.)  # coordinate bearing disagrees with image
        a._takeoff_done, a._landing = True, False
        a._est_trustworthy = lambda: True
        a._bounds_recovery_velocity = lambda: None
        a._radar_guard_velocity = lambda vx, vy: (0., 0.)
        a._grid_guard_velocity = lambda vx, vy: (vx, vy)
        a._map_guard_velocity = lambda vx, vy: (vx, vy)
        a._final_scan_cap = lambda vx, vy: (vx, vy)
        a._last_cmd_v = (1., 0.)
        a._last_csv_t = 100.1
        a._climb_latch, a._climb_t0, a._alt_sag = False, None, 0.
        # Height adjustment in this test occurs in fresh, clear lidar space.
        a._scan = NS(angle_min=-math.pi, angle_increment=math.pi/180.,
                     range_min=.1, range_max=20., ranges=[20.]*361)
        a.pos_pub = Mock()
        with patch.object(sa, 'FLIGHT_OUTPUT', 'pos'):
            a._send_vel(1., 0.)
        output = a.pos_pub.publish.call_args.args[0].pose
        self.assertEqual((output.position.x, output.position.y), a.local_xy)
        self.assertLess(output.position.z, a.local_z)
        self.assertGreater(output.orientation.z, 0.)
        self.assertAlmostEqual(a._tracking_blocked_since, 100.1)

    def test_cancellation_discards_old_image_and_observation_move(self):
        a = self.agent()
        self.send(a)
        a._track_assigned_id = 't3'
        a._observation_detour = dict(tid='t3', angle=.3, until=103.)
        a._discard_old_plan = lambda: None
        a._assign_cb(NS(uav_id='u0', target_id='t3', task_type=255))
        self.assertIsNone(a._tracking_image)
        self.assertIsNone(a._tracking_altitude)
        self.assertIsNone(a._observation_detour)
        self.assertIsNone(a._tracking_blocked_since)

    def test_abandonment_discards_old_image_and_observation_move(self):
        a = self.agent()
        self.send(a)
        a._confirm_close = False
        a._claims = {}
        a._observation_detour = dict(tid='t3', angle=.3, until=103.)
        a._abort_orbit('test lost image')
        self.assertIsNone(a._tracking_image)
        self.assertIsNone(a._observation_detour)

    def test_future_stale_peer_and_wrong_identity_views_do_not_steer(self):
        for change in (dict(sample_s=100.2), dict(sample_s=99.),
                       dict(source_uav='u1'), dict(target_id='t1'),
                       dict(x=140., y=20.), dict(real_detection=False),
                       dict(box=[0., 0., 0., 0.]), dict(bearing=float('nan'))):
            with self.subTest(change=change):
                a = self.agent()
                self.send(a, self.payload(**change))
                self.assertIsNone(getattr(a, '_tracking_image', None))

    def test_old_view_expires_and_search_cannot_use_it(self):
        a = self.agent()
        self.send(a)
        self.assertIsNotNone(a._fresh_tracking_view(100.4))
        self.assertIsNone(a._fresh_tracking_view(100.6))
        a.assignment = NS(task_type=0)
        self.assertIsNone(a._fresh_tracking_view(100.2))

    def test_safe_centre_holds_altitude_and_high_person_moves_up(self):
        view = parse_tracking_view(self.payload(box=[325., 230., 375., 360.]), 'u0', 't3', 100.1)
        self.assertEqual(image_altitude_goal(view, 3., 8., 2.6, 4.2), 3.)
        view['box'] = (325., 10., 375., 160.)
        self.assertGreater(image_altitude_goal(view, 3., 8., 2.6, 4.2), 3.)
        # A high box cannot authorize height that loses the reserved foot FOV.
        self.assertLess(image_altitude_goal(view, 4.2, 8., 2.6, 4.2), 4.2)

    def test_large_clipped_box_needs_distance_not_unbounded_height_changes(self):
        view = parse_tracking_view(self.payload(box=[325., 0., 375., 480.]), 'u0', 't3', 100.1)
        self.assertTrue(view_needs_space(view))
        self.assertEqual(image_altitude_goal(view, 2.6, 2., 2.6, 4.2), 2.6)

    def obstacle_agent(self):
        a = self.agent()
        a.local_xy = (58., 20.)
        a.targets = {'t3': (50., 20., 1., 0.)}
        a._radar_prev = (3., 20., 20.)
        a.grid = NS(origin=(-50., -60.), width=200, height=120, resolution=1.,
                    world_to_cell=lambda p: p, is_free=lambda p: p[0] < 60.5)
        a.los = NS(visible=lambda *args: True)
        return a

    def test_obstacle_triggers_side_view_even_while_images_are_fresh(self):
        a = self.obstacle_agent()
        a._visual_observations = {'t3': dict(sample_s=100., eligible=True)}
        goal = a._obstructed_view_goal(50., 20., 1., 0., 100.1)
        self.assertIsNotNone(goal)
        self.assertGreater(abs(goal[1]-20.), 2.)
        self.assertLessEqual(math.hypot(goal[0]-58., goal[1]-20.), 6.)
        self.assertTrue(a._safe_reobserve_segment((58., 20.), goal))

    def test_detour_rejects_blocked_segments_and_stale_lidar(self):
        a = self.obstacle_agent()
        a.grid.is_free = lambda p: abs(p[1]-20.) < .3
        self.assertIsNone(a._obstructed_view_goal(50., 20., 1., 0., 100.1))
        a.grid.is_free = lambda p: True
        self.assertIsNone(a._obstructed_view_goal(50., 20., 1., 0., 102.))

    def test_detour_rejects_occluded_view_and_boundary_exit(self):
        a = self.obstacle_agent()
        a.los.visible = lambda *args: False
        self.assertIsNone(a._obstructed_view_goal(50., 20., 1., 0., 100.1))
        a.los.visible = lambda *args: True
        a._scan_t = 101.
        with patch.object(sa, 'flight_bounds', return_value=(57., 60., 19., 21.)):
            self.assertIsNone(a._obstructed_view_goal(50., 20., 1., 0., 101.))

    def test_open_space_good_view_keeps_original_following_goal(self):
        a = self.obstacle_agent()
        a._radar_prev = (20., 20., 20.)
        a.grid.is_free = lambda p: True
        self.assertIsNone(a._obstructed_view_goal(50., 20., 1., 0., 100.1))

    def test_wall_beside_safe_motion_does_not_start_circling(self):
        a = self.obstacle_agent()
        a.grid.is_free = lambda p: p[1] < 22.
        self.assertIsNone(a._obstructed_view_goal(50., 20., 1., 0., 100.1))

    def test_obstacle_probe_follows_requested_motion_not_aircraft_heading(self):
        a = self.obstacle_agent()
        a.yaw = math.pi
        self.assertIsNotNone(a._obstructed_view_goal(
            50., 20., 0., 0., 100.1, desired_velocity=(1., 0.)))

    def test_safe_stop_requests_alternative_without_any_pixel_loss(self):
        a = self.obstacle_agent()
        a.grid.is_free = lambda p: True
        a._tracking_blocked_since = 99.5
        self.assertIsNotNone(a._obstructed_view_goal(50., 20., 0., 0., 100.1))

    def test_cropped_frame_alone_cannot_command_sideways_orbit(self):
        a = self.obstacle_agent()
        a.grid.is_free = lambda p: True
        self.send(a, self.payload(x=50., y=20., box=[325., 390., 375., 480.]))
        self.assertIsNotNone(a._fresh_tracking_view(100.1))
        self.assertIsNone(a._obstructed_view_goal(50., 20., 0., 0., 100.1))

    def test_full_local_pixels_do_not_veto_explicit_quality_recovery(self):
        a = self.obstacle_agent()
        self.send(a, self.payload(x=50., y=20., box=[325., 230., 375., 360.]))
        a._last_eligible_image = {'t3': 90.}
        a._reobserve_forced = ('t3', 106.)
        a._safe_reobserve_segment = lambda *args: True
        a.los = NS(visible=lambda *args: True)
        a._t_seen = {'t3': 100.}
        a._orbit_entered_t = 90.
        self.assertIsNotNone(a._reobserve_goal(50., 20., 100.1))
        self.assertEqual(a._visual_observations, {})

    def test_new_obstacle_invalidates_cached_observation_move(self):
        a = self.obstacle_agent()
        self.assertIsNotNone(a._obstructed_view_goal(50., 20., 1., 0., 100.1))
        a.grid.is_free = lambda p: False
        self.assertIsNone(a._obstructed_view_goal(50., 20., 1., 0., 100.2))

    def test_good_track_does_not_make_bad_frame_reportable(self):
        tr = pr.Track('white', 8., 0., .9, (376., 400.), (50., 100.),
                      8., 1.7, 100., loc_ok=True)
        self.assertTrue(tr.can_report_image())
        tr.update(8.1, 0., .2, .9, (376., 400.), (50., 100.),
                  8., 1.7, 100.2, observation_s=100.2, loc_ok=False)
        self.assertFalse(tr.can_report_image())
        self.assertEqual(tr.miss, 0)  # steering can still use the detected box
        tr.update(8.2, 0., .2, .9, (376., 400.), (50., 100.),
                  8., 1.7, 100.4, observation_s=100.4, loc_ok=True)
        self.assertTrue(tr.can_report_image())
        tr.coast(.2)
        self.assertFalse(tr.can_report_image())


if __name__ == '__main__':
    unittest.main()
