"""Small regressions for six-actor match blockers; run with ROS environments sourced."""
import math
import json
import threading
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))

from perception import perception_real as vision
import cooperative_tracker
import swarm_agent
import swarm_manager
import swarm_task
import yolo_target_bridge
from slam_grid import SlamGrid


class MatchBlockers(unittest.TestCase):
    def test_search_defers_outer_y_row_until_interior_is_covered(self):
        grid = swarm_task.CoverageGrid(0, 14, 0, 21, cell_m=7, num_uavs=1)
        outer = min(grid.cells, key=lambda k: grid.cell(k).cy)
        inner = next(k for k in grid.cells if grid.cell(k).cy == 10.5)
        for cell in grid.cells.values():
            cell.state = swarm_task.STATE_COVERED
        grid.cell(outer).state = swarm_task.STATE_FREE
        grid.cell(inner).state = swarm_task.STATE_FREE
        alloc = swarm_task.TaskAllocator(grid, w_flight=1.0, w_zone=0.0,
                                         w_novelty=0.0, cruise_speed=1.0)
        ox, oy = grid.cell(outer).cx, grid.cell(outer).cy
        with patch.object(swarm_task, '_now', return_value=200.0), \
             patch.object(swarm_task, 'SEARCH_HARD_HOME_ZONE', False):
            self.assertEqual(alloc.allocate({'uav_0': (ox, oy)})['uav_0'], inner)
            self.assertEqual(alloc.allocate({'uav_0': (ox, oy)})['uav_0'], outer)

    def test_observation_height_keeps_near_actor_low_and_far_actor_visible(self):
        self.assertAlmostEqual(swarm_agent.observation_altitude_cap(4.0), 2.6)
        self.assertAlmostEqual(swarm_agent.observation_altitude_cap(8.0), 3.5)
        self.assertAlmostEqual(swarm_agent.observation_altitude_cap(6.5), 3.05)

    def test_home_search_expires_and_allows_nearby_cross_zone_cell(self):
        grid = swarm_task.CoverageGrid(0, 40, 0, 7, cell_m=7, num_uavs=2)
        home = next(k for k in grid.cells if grid.get_zone_id(k) == 0)
        away = next(k for k in grid.cells if grid.get_zone_id(k) == 1)
        for cell in grid.cells.values():
            cell.state = swarm_task.STATE_COVERED
        grid.cell(home).state = swarm_task.STATE_FREE
        grid.cell(away).state = swarm_task.STATE_FREE
        alloc = swarm_task.TaskAllocator(grid, w_flight=1.0, w_zone=0.0,
                                         w_novelty=0.0, cruise_speed=1.0)
        alloc.mission_start = 100.0
        alloc.near_penalty = 0.0
        ax, ay = grid.cell(away).cx, grid.cell(away).cy
        with patch.object(swarm_task, '_now', return_value=150.0), \
             patch.object(swarm_task, 'SEARCH_HOME_HARD_S', 120.0):
            self.assertEqual(alloc.allocate({'uav_0': (ax, ay)})['uav_0'], home)
        grid.cell(home).state = swarm_task.STATE_FREE
        grid.cell(away).state = swarm_task.STATE_FREE
        with patch.object(swarm_task, '_now', return_value=221.0), \
             patch.object(swarm_task, 'SEARCH_HOME_HARD_S', 120.0):
            self.assertEqual(alloc.allocate({'uav_0': (ax, ay)})['uav_0'], away)

    def test_review_reopens_on_simulation_clock(self):
        grid = swarm_task.CoverageGrid(0, 7, 0, 7, cell_m=7)
        cell = next(iter(grid.cells.values()))
        with patch.object(swarm_task, '_now', return_value=100.0):
            grid.mark_review(next(iter(grid.cells)), 0.4)
        self.assertEqual(cell.review_t, 100.0)
        with patch.object(swarm_task, '_now', return_value=120.0):
            self.assertEqual(grid.reopen_review(max_age=60.0), 0)
        with patch.object(swarm_task, '_now', return_value=161.0):
            self.assertEqual(grid.reopen_review(max_age=60.0), 1)

    def test_fresh_camera_preferred_over_geometric_echo(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.uav_ids = ['uav_0', 'uav_1']
        manager.status = {
            'uav_0': SimpleNamespace(x=9.0, y=0.0, connected=True),
            'uav_1': SimpleNamespace(x=11.0, y=0.0, connected=True),
        }
        manager._visual_source = {}
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(100.2)):
            manager._visual_source_cb(SimpleNamespace(data=json.dumps({
                'target_id': 't2', 'source_uav': 'uav_1',
                'sample_s': 100.0, 'x': 0.0, 'y': 0.0})))
        self.assertEqual(manager._preferred_camera('t2', 0.0, 0.0, 100.3, set()),
                         ('uav_1', 11.0))
        self.assertIsNone(manager._preferred_camera('t2', 0.0, 0.0, 102.0, set()))

    def test_new_camera_target_can_repurpose_backup_with_old_primary_nearby(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.status = {
            'uav3': SimpleNamespace(x=149.0, y=-3.0, connected=True),
            'uav5': SimpleNamespace(x=150.0, y=15.0, connected=True),
            'uav0': SimpleNamespace(x=90.0, y=8.0, connected=True),
        }
        manager._tracking = {'t0': 'uav3'}
        manager._backup = {'t0': 'uav5'}
        manager._visual_source = {'t2': ('uav5', 100.1, 150.0, 9.0)}
        manager._truth_cache = {'t0': (150.0, 4.0, 100.0)}
        old = SimpleNamespace(observers={'uav3', 'uav5'})
        manager.tracker = SimpleNamespace(targets={'t0': old})
        self.assertTrue(manager._free_camera_backup_for_new_target(
            't2', 150.0, 9.0, 100.2))
        self.assertEqual(manager._backup, {})
        self.assertEqual(old.observers, {'uav3'})
        manager._backup['t0'] = 'uav5'
        old.observers.add('uav5')
        manager.status['uav0'].x = 148.0
        self.assertFalse(manager._free_camera_backup_for_new_target(
            't2', 150.0, 9.0, 100.2))
        self.assertEqual(manager._backup, {'t0': 'uav5'})

    def test_bridge_preserves_accepted_camera_source(self):
        bridge = yolo_target_bridge._mk_bridge()
        bridge.source_pub = yolo_target_bridge._FakePub()
        bridge._advance(100.0)
        report = {'target_id': 'green', 'frame_id': 'world_enu',
                  'xyz': [3.0, 4.0, 1.2], 'confidence': 0.9,
                  'range_m': 7.0, 'source_uav': 'typhoon_h480_2',
                  'sample_s': 99.9}
        bridge._report_cb(SimpleNamespace(data=json.dumps(report)))
        self.assertEqual(len(bridge.source_pub.msgs), 1)
        source = json.loads(bridge.source_pub.msgs[0].data)
        self.assertEqual((source['target_id'], source['source_uav']),
                         ('t0', 'typhoon_h480_2'))

    def test_first_dispatch_assigns_near_camera_owner(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.grid = swarm_task.CoverageGrid(-20, 20, -20, 20)
        manager.uav_ids = ['uav_0', 'uav_1']
        manager.status = {
            'uav_0': SimpleNamespace(x=9.0, y=0.0, connected=True),
            'uav_1': SimpleNamespace(x=11.0, y=0.0, connected=True),
        }
        manager._tracking = {}
        manager._backup = {}
        manager._visual_source = {'t0': ('uav_1', 100.0, 0.0, 0.0)}
        manager._sanitize_dispatch_xy = lambda x, y, _tid: (x, y)
        manager._idle_uavs = lambda: ['uav_0', 'uav_1']
        sent = []
        manager.assign_pub = SimpleNamespace(publish=sent.append)
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(100.2)):
            manager._dispatch_tracker('t0', 0.0, 0.0)
        self.assertEqual(manager._tracking, {'t0': 'uav_1'})
        self.assertEqual((sent[0].uav_id, sent[0].task_type), ('uav_1', 1))

    def test_only_judge_correlated_red_visual_track_is_retired(self):
        bridge = yolo_target_bridge._mk_bridge()
        bridge._advance(100.0)
        yolo_target_bridge._activate(bridge, 'red1', 10.0, 0.0, 6.0)
        yolo_target_bridge._activate(bridge, 'red2', 40.0, 0.0, 6.0)
        bridge._note_red_left({0, 1, 2, 3, 4, 5})
        bridge._red_find_binding[4] = ('red1', 99.5)
        bridge._note_red_left({0, 1, 2, 3, 5})
        self.assertTrue(bridge.core.tracks['red1'].elim_pending)
        self.assertFalse(bridge.core.tracks['red2'].elim_pending)
        eliminated = [ev['tag'] for ev in bridge.core.tick(100.0)
                      if ev['eliminated']]
        self.assertEqual(eliminated, ['red1'])

    def test_cross_class_duplicate_uses_shirt_color_not_yolo_confidence(self):
        box = np.array([[20.0, 10.0, 80.0, 110.0]])
        blue = SimpleNamespace(cls=2, conf=0.98, xyxy=box)
        white = SimpleNamespace(cls=3, conf=0.70, xyxy=box)
        image = np.zeros((120, 100, 3), dtype=np.uint8)
        image[30:62, 38:62] = (170, 170, 170)
        self.assertEqual(vision.filter_blue_white_duplicate_boxes(
            image, [blue, white]), [white])
        image[30:62, 38:62] = (110, 30, 20)
        self.assertEqual(vision.filter_blue_white_duplicate_boxes(
            image, [blue, white]), [blue])
        image[30:62, 38:62] = (35, 35, 35)
        self.assertEqual(vision.filter_blue_white_duplicate_boxes(
            image, [blue, white]), [blue, white])

    def test_officially_found_target_keeps_observer_through_short_occlusion(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._tracking = {'t0': 'uav3'}
        manager._official_find_t = {'t0': 186.3}
        manager._last_detect = {'t0': 188.5}
        self.assertTrue(manager._hold_recent_official_target('t0', 191.6))
        self.assertTrue(manager._hold_recent_official_target('t0', 198.0))
        self.assertFalse(manager._hold_recent_official_target('t0', 201.0))
        manager._official_find_t.clear()
        self.assertFalse(manager._hold_recent_official_target('t0', 191.6))

    def test_red_visual_slot_can_be_officially_found_as_other_red_actor(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._tracking = {'t5': 'uav3'}
        manager._official_find_t = {'t4': 116.4}
        manager._last_detect = {'t5': 123.2}
        self.assertTrue(manager._hold_recent_official_target('t5', 129.2))
        self.assertEqual(manager._official_find_for_visual('t5'), 116.4)
        self.assertFalse(manager._hold_recent_official_target('t5', 135.3))

    def test_near_camera_target_survives_short_cropped_view(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._tracking = {'t2': 'uav5'}
        manager._visual_near_seen = {'t2': (56.1, 23.6, 148.5)}
        manager._last_detect = {'t2': 148.5}
        manager.status = {'uav5': SimpleNamespace(x=52.8, y=25.4)}
        self.assertTrue(manager._hold_near_visual_target('t2', 151.5))
        self.assertFalse(manager._hold_near_visual_target('t2', 157.0))
        manager.status['uav5'].x = 80.0
        self.assertFalse(manager._hold_near_visual_target('t2', 152.0))

    def test_stable_near_boundary_actor_not_rejected_by_motion_score(self):
        track = vision.Track('brown', 149.6, 8.3, 0.84, (290.0, 475.0),
                             (36.0, 92.0), 5.5, 1.7, 398.0,
                             observation_s=398.0)
        track.hits = 110
        track.t = 400.0
        track.last_seen_t = 400.0
        track.score_ema = 0.17
        track.recent_observed_xy = [
            (398.4, 149.60, 8.31), (398.8, 149.59, 8.32),
            (399.2, 149.61, 8.33), (400.0, 149.60, 8.32)]
        self.assertEqual(track.verdict(400.0, attach_check=False), (True, ''))
        track.conf = 0.57
        self.assertEqual(track.verdict(400.0, attach_check=False), (False, 'static'))

    def test_near_clipped_head_box_can_start_new_track(self):
        # 20:08 run, actor_2 at (150, 8.7): valid head-ray location, but
        # cropped-foot aspect ratio ~2.55 exceeded ordinary strict max 2.2.
        self.assertTrue(vision.near_cropped_strict_ok(
            True, 5.55, 0.84, 1.70, 2.55))
        self.assertFalse(vision.near_cropped_strict_ok(
            True, 5.55, 0.57, 1.70, 2.55))
        self.assertFalse(vision.near_cropped_strict_ok(
            True, 9.53, 0.84, 1.70, 2.55))
        self.assertFalse(vision.near_cropped_strict_ok(
            False, 5.55, 0.84, 1.70, 2.55))

    def test_clear_narrow_boundary_actor_starts_track_before_flyby(self):
        # 20:21 match: UAV5 saw blue at (150, 51.5) for ten 0.2 s frames,
        # but h/w=2.58 rejected every frame before any track existed.
        self.assertTrue(vision.near_narrow_person_strict_ok(
            'blue', 9.9, 0.887, 1.88, 71.1 / 27.6,
            71.1, 197.0, 225.0, 207.0, 752))
        self.assertTrue(vision.near_narrow_person_strict_ok(
            'white', 10.1, 0.898, 1.71, 63.6 / 23.3,
            63.6, 437.0, 460.0, 169.0, 752))
        # The confirmed false blue at (89, -27) had only 0.40-0.64 confidence.
        self.assertFalse(vision.near_narrow_person_strict_ok(
            'blue', 2.0, 0.64, 1.70, 2.55,
            70.0, 300.0, 328.0, 230.0, 752))
        self.assertTrue(vision.near_narrow_person_strict_ok(
            'red', 9.9, 0.90, 1.88, 2.58,
            71.1, 197.0, 225.0, 207.0, 752))
        self.assertFalse(vision.near_narrow_person_strict_ok(
            'red', 9.9, 0.61, 1.88, 2.58,
            71.1, 197.0, 225.0, 207.0, 752))

    def test_assigned_red_keeps_nose_locked_track_through_parallax_gate(self):
        # The tracked red actor in the 20:38 run had 89 hits but was rejected
        # as self-attached after UAV3 locked its nose on t5.  Red has no
        # TID_OF_COLOR entry; both red assignments need the same exemption.
        self.assertTrue(vision.skip_attached_verdict('red', 't5', 8.1))
        self.assertTrue(vision.skip_attached_verdict('red', 't4', 8.1))
        self.assertFalse(vision.skip_attached_verdict('red', None, 8.1))
        self.assertFalse(vision.skip_attached_verdict('red', 't5', 16.0))
        self.assertFalse(vision.skip_attached_verdict('blue', 't5', 8.1))

    def test_short_stable_boundary_candidate_dispatches_but_weak_static_does_not(self):
        track = vision.Track('blue', 149.55, 51.55, 0.89,
                             (210.0, 242.0), (27.6, 71.1), 9.9, 1.88,
                             581.7, observation_s=581.7)
        track.hits = 4
        track.t = track.last_seen_t = 582.32
        track.score_ema = 0.12
        track.recent_observed_xy = [
            (581.7, 149.55, 51.55), (581.9, 149.59, 51.53),
            (582.1, 149.64, 51.51), (582.32, 149.65, 51.52)]
        with patch.object(vision, 'CONFIRM_HITS', 3):
            self.assertEqual(track.verdict(582.32, attach_check=False), (True, ''))
            track.conf = 0.64
            self.assertEqual(track.verdict(582.32, attach_check=False), (False, 'static'))

    def test_stationary_official_median_rejects_one_depth_jump(self):
        h = [(0.0, 0.0, 0.0), (0.3, 0.1, 0.1),
             (0.6, 0.0, -0.1), (0.9, -0.1, 0.1),
             (1.2, 0.1, 0.0), (1.5, 0.0, 1.10)]
        center = yolo_target_bridge.stable_official_point(h, 1.5)
        self.assertIsNotNone(center)
        self.assertLess(math.hypot(*center), 0.2)
        moving = [(i * 0.3, i * 0.3, 0.0) for i in range(6)]
        self.assertIsNone(yolo_target_bridge.stable_official_point(moving, 1.5))

    def test_judge_latency_compensation_requires_coherent_motion(self):
        history = [(100.0, 40.0, 5.5), (100.4, 40.4, 5.55),
                   (100.8, 40.8, 5.48), (101.2, 41.2, 5.52)]
        motion = yolo_target_bridge.coherent_official_velocity(history, 101.2)
        self.assertIsNotNone(motion)
        self.assertAlmostEqual(motion[0], 1.0, delta=0.05)
        jumped = history[:-1] + [(101.2, 42.8, 5.52)]
        self.assertIsNone(yolo_target_bridge.coherent_official_velocity(
            jumped, 101.2))
        stationary = [(100.0, 40.0, 5.5), (100.5, 40.1, 5.5),
                      (101.0, 40.0, 5.5)]
        self.assertIsNone(yolo_target_bridge.coherent_official_velocity(
            stationary, 101.0))

    def test_world_hard_limit_uses_actual_match_grid(self):
        grid = SlamGrid(-55.0, -65.0, 155.0, 65.0, 0.5, 0.0)
        xmin, xmax, ymin, ymax = swarm_agent.estimated_world_hard_bounds(grid)
        self.assertLess(xmin, -55.0)
        self.assertGreater(xmax, 155.0)
        self.assertLess(ymin, -65.0)
        self.assertGreater(ymax, 65.0)
        self.assertLess(155.0, xmax)  # legal east edge cannot be EKF corruption

    def test_first_confirmed_visual_dispatches_without_judge_report(self):
        bridge = yolo_target_bridge._mk_bridge()
        bridge._advance(100.1)
        bridge.core.report(100.0, 'green', 149.8, 9.9, 1.0, 11.9,
                           official_ok=True)
        self.assertFalse(bridge.core.tracks['green'].alive)
        bridge._tick(None)
        self.assertEqual(len(bridge.pub.msgs), 1)
        self.assertEqual(len(bridge._actor_pubs['green'].msgs), 0)
        bridge._advance(0.2)
        bridge.core.report(100.2, 'green', 149.7, 9.8, 1.0, 11.5,
                           official_ok=True)
        self.assertTrue(bridge.core.tracks['green'].alive)
        bridge._tick(None)
        self.assertEqual(len(bridge.pub.msgs), 2)
        # A lone low-confidence detection has no authority to dispatch.
        bridge2 = yolo_target_bridge._mk_bridge()
        bridge2.core.report(100.0, 'blue', -27.0, 13.0, 0.86, 8.0)
        self.assertEqual(bridge2.core.tick(100.1), [])

    def test_bridge_reacquisition_requires_new_reports_after_long_gap(self):
        core = yolo_target_bridge.TargetBridgeCore()
        core.report(100.0, 'green', 10.0, 0.0, 1.0, 8.0)
        core.report(100.2, 'green', 10.1, 0.0, 1.0, 8.0)
        self.assertTrue(core.tracks['green'].alive)
        core.tick(110.0)
        self.assertFalse(core.tracks['green'].alive)
        core.report(148.0, 'green', 150.0, 10.0, 1.0, 8.0)
        self.assertFalse(core.tracks['green'].alive)
        self.assertEqual(core.tracks['green']._high_conf_count, 1)
        self.assertEqual(core.tick(148.1)[0]['official_x'], None)
        core.report(148.2, 'green', 150.1, 10.0, 1.0, 8.0)
        self.assertTrue(core.tracks['green'].alive)

    def test_manager_replaces_per_aircraft_laser_snapshots(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._slam_lock = threading.Lock()
        manager._slam_cells = None
        manager._slam_sources = {}
        manager._slam_w = manager._slam_h = 0
        manager._slam_res = 0.5
        manager._slam_origin = (0.0, 0.0)
        manager._slam_dirty = False

        def report(uid, cells):
            runs = []
            for value in cells:
                if runs and runs[-1][0] == value:
                    runs[-1][1] += 1
                else:
                    runs.append([value, 1])
            msg = SimpleNamespace(data=json.dumps({
                'uav_id': uid, 'width': 3, 'height': 1,
                'resolution': 0.5, 'origin': [0.0, 0.0], 'rle': runs}))
            manager._occ_grid_cb(msg)

        report('u0', [1, 1, 0])
        report('u1', [0, 1, 1])
        self.assertEqual(list(manager._slam_cells), [1, 1, 1])
        report('u0', [0, 0, 0])
        self.assertEqual(list(manager._slam_cells), [0, 1, 1])
        report('u1', [0, 0, 0])
        self.assertEqual(list(manager._slam_cells), [0, 0, 0])
        self.assertTrue(manager._slam_dirty)

    def test_laser_frees_old_dynamic_hit_without_erasing_overlapping_wall_margin(self):
        grid = SlamGrid(0.0, 0.0, 20.0, 20.0, 0.5, 1.0)
        scan = SimpleNamespace(ranges=[5.0], angle_min=0.0,
                               angle_increment=0.1, range_min=0.1,
                               range_max=12.0)
        grid.mark_scan((2.0, 10.0), 0.0, scan, 12.0)
        old = grid.world_to_cell((7.0, 10.0))
        self.assertFalse(grid.is_free_raw(old))
        grid.mark_point(7.5, 10.5)
        scan.ranges = [float('inf')]
        grid.mark_scan((2.0, 10.0), 0.0, scan, 12.0)
        self.assertTrue(grid.is_free_raw(old))
        self.assertFalse(grid.is_free(old))
        self.assertEqual(grid.total_cells, 1)
        self.assertFalse(grid.is_free_raw(grid.world_to_cell((7.5, 10.5))))
        grid.mark_scan((2.0, 10.5), 0.0, scan, 12.0)
        self.assertTrue(grid.is_free(old))
        self.assertEqual(grid.total_cells, 0)

    def test_laser_max_range_is_free_space_not_a_closed_obstacle_ring(self):
        grid = SlamGrid(0.0, 0.0, 30.0, 30.0, 0.5, 1.0)
        scan = SimpleNamespace(ranges=[12.0], angle_min=0.0,
                               angle_increment=0.1, range_min=0.1,
                               range_max=12.0)
        grid.mark_scan((5.0, 15.0), 0.0, scan, 12.0)
        self.assertEqual(grid.total_cells, 0)
        scan.ranges = [11.0]
        grid.mark_scan((5.0, 15.0), 0.0, scan, 12.0)
        self.assertGreater(grid.total_cells, 0)

    def test_close_actor_walking_toward_camera_triggers_outward_recovery(self):
        vx, vy, urgent = swarm_agent.urgent_close_backoff_velocity(
            0.0, 0.0, 0.0, 0.0, 5.0, 0.0, -1.0, 0.0)
        self.assertTrue(urgent)
        self.assertLessEqual(vx, -1.2)
        self.assertEqual(vy, 0.0)
        _, _, urgent = swarm_agent.urgent_close_backoff_velocity(
            0.0, 0.0, 0.0, 0.0, 8.0, 0.0, -1.0, 0.0)
        self.assertFalse(urgent)

    def test_receding_actor_does_not_make_observer_retreat_from_safe_range(self):
        # UAV behind an eastbound actor, 6.5 m apart.  The actor itself is
        # opening the separation; a forced westward standoff lost the view.
        vx, vy = swarm_agent.ring_standoff_velocity(
            0.6, 0.0, 0.0, 0.0, 6.5, 0.0, 8.0, 1.0, 0.0)
        self.assertAlmostEqual(vx, 0.6)
        self.assertAlmostEqual(vy, 0.0)
        vx, _ = swarm_agent.ring_standoff_velocity(
            0.6, 0.0, 0.0, 0.0, 5.0, 0.0, 8.0, 1.0, 0.0)
        self.assertLess(vx, 0.6)  # Too close still backs away.

    def test_unverified_civilian_releases_tracker_without_blocking_real_actor_elsewhere(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._visual_candidate = {'t1': (100.0, -27.0, 13.7)}
        manager._visual_quarantine = {}
        manager._official_find_t = {}
        manager._last_detect = {'t1': 125.5}
        manager._visual_near_seen = {'t1': (-26.9, 13.8, 125.5)}
        manager._tracking = {'t1': 'u2'}
        manager.status = {'u2': SimpleNamespace(x=-25.0, y=14.0)}
        manager.tracker = SimpleNamespace(targets={'t1': object()})
        manager._truth_cache = {'t1': (-27.0, 13.7, 125.0)}
        manager._cur_targets = {'t1': (-27.0, 13.7, 'u2')}
        manager._confirm_done_t = {}
        manager._target_hist = {}
        cancelled = []
        published = []
        manager._release_tracker = lambda tid, reason: cancelled.append((tid, reason))
        manager.visual_reject_pub = SimpleNamespace(publish=published.append)
        manager._release_unverified_visuals(126.0)
        self.assertEqual(cancelled, [('t1', 'unverified_visual')])
        self.assertNotIn('t1', manager.tracker.targets)
        self.assertTrue(manager._quarantined_visual('t1', -27.0, 13.7, 127.0))
        self.assertTrue(manager._quarantined_visual('t1', -27.0, 13.7, 612.0))
        self.assertFalse(manager._quarantined_visual('t1', 150.0, 42.5, 127.0))
        self.assertEqual(json.loads(published[0].data)['target_id'], 't1')
        self.assertEqual(json.loads(published[0].data)['duration_s'], 600.0)

    def test_public_find_keeps_stationary_actor_eligible(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._visual_candidate = {'t3': (100.0, 150.0, 35.0)}
        manager._visual_quarantine = {}
        manager._official_find_t = {'t3': 110.0}
        manager._last_detect = {'t3': 125.5}
        manager._tracking = {'t3': 'u5'}
        manager.status = {'u5': SimpleNamespace(x=145.0, y=35.0)}
        manager._release_tracker = lambda *_args, **_kwargs: self.fail('released true actor')
        manager._release_unverified_visuals(126.0)
        self.assertEqual(manager._visual_quarantine, {})

    def test_brief_close_view_is_not_quarantined_after_observer_leaves(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._visual_candidate = {'t3': (100.0, -27.0, 13.0)}
        manager._visual_near_seen = {'t3': (-26.8, 13.1, 108.0)}
        manager._visual_quarantine = {}
        manager._official_find_t = {}
        manager._last_detect = {'t3': 108.0}  # now stale
        manager._tracking = {}                 # previous observer was released
        manager.status = {}
        manager.tracker = SimpleNamespace(targets={'t3': object()})
        manager._truth_cache = {'t3': (-27.0, 13.0, 108.0)}
        manager._cur_targets = {}
        manager._confirm_done_t = {}
        manager._target_hist = {}
        published = []
        manager.visual_reject_pub = SimpleNamespace(publish=published.append)
        manager._release_tracker = lambda *_args, **_kwargs: None
        manager._release_unverified_visuals(126.0)
        self.assertIn('t3', manager.tracker.targets)
        self.assertFalse(manager._quarantined_visual('t3', -27.0, 13.0, 127.0))
        self.assertFalse(manager._quarantined_visual('t3', 150.0, 39.0, 127.0))
        self.assertEqual(published, [])

    def test_bridge_releases_civilian_color_slot_for_distant_real_target(self):
        bridge = yolo_target_bridge._mk_bridge()
        bridge._advance(100.0)
        for i in range(4):
            bridge.core.report(99.0 + i * 0.2, 'blue', -27.0, 13.7, 0.9, 6.0)
        self.assertTrue(bridge.core.tracks['blue'].alive)
        bridge._rejected_visual_cb(SimpleNamespace(data=json.dumps({
            'target_id': 't1', 'x': -27.0, 'y': 13.7, 'duration_s': 90.0})))
        self.assertFalse(bridge.core.tracks['blue'].alive)
        self.assertTrue(bridge._visual_is_rejected('blue', -27.1, 13.8, 101.0))
        self.assertFalse(bridge._visual_is_rejected('blue', 150.0, 42.5, 101.0))
        bridge._rejected_visual_cb(SimpleNamespace(data=json.dumps({
            'target_id': 't1', 'x': -27.0, 'y': 13.7, 'duration_s': 600.0})))
        self.assertTrue(bridge._visual_is_rejected('blue', -27.1, 13.8, 612.0))
        for i in range(4):
            bridge.core.report(101.0 + i * 0.2, 'blue', 150.0, 42.5, 0.9, 6.0)
        self.assertTrue(bridge.core.tracks['blue'].alive)

    def test_edge_cell_is_not_covered_from_too_far_inland(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.grid = SimpleNamespace(x_min=-55.0, x_max=155.0,
                                       y_min=-65.0, y_max=65.0)
        self.assertFalse(manager._camera_cover_reachable(137.5, 43.5,
                                                          144.5, 43.5))
        self.assertTrue(manager._camera_cover_reachable(144.5, 43.5,
                                                         144.5, 43.5))
        self.assertTrue(manager._camera_cover_reachable(67.5, 8.5,
                                                         74.5, 8.5))

    def test_close_stationary_person_can_enter_visual_candidate(self):
        track = vision.Track('blue', 150.0, 35.0, 0.9,
                             (376.0, 380.0), (50.0, 120.0),
                             5.87, 1.7, 100.0)
        track.hits = 4
        with patch.object(vision, 'CONFIRM_HITS', 3):
            self.assertTrue(track.verdict(102.0)[0])
            track.rng = 12.0
            self.assertEqual(track.verdict(102.0), (False, 'static'))

    def test_short_red_false_detection_cannot_become_confirmed_report(self):
        self.assertFalse(vision.report_ready_hits('red', 3))
        self.assertTrue(vision.report_ready_hits('red', 8))
        self.assertTrue(vision.report_ready_hits('brown', vision.CONFIRM_HITS))

    def test_cropped_person_box_needs_close_head_based_range_for_official_report(self):
        self.assertFalse(vision.official_view_ok((677.0, 472.0),
                                                 (76.0, 156.0)))
        self.assertFalse(vision.official_view_ok((377.0, 472.0),
                                                 (70.0, 150.0), range_m=9.0))
        self.assertTrue(vision.official_view_ok((377.0, 472.0),
                                                (70.0, 150.0), range_m=3.5))
        self.assertFalse(vision.official_view_ok((377.0, 472.0),
                                                 (70.0, 150.0), range_m=3.5,
                                                 clipped_stable=False))
        self.assertFalse(vision.official_view_ok((377.0, 472.0),
                                                 (70.0, 150.0), range_m=3.5,
                                                 height=450))
        self.assertTrue(vision.official_view_ok((400.0, 380.0),
                                                (75.0, 90.0)))
        core = yolo_target_bridge.TargetBridgeCore()
        core.report(1.0, 'white', 10.0, 10.0, 0.9, 6.0,
                    official_ok=True)
        core.report(1.2, 'white', 12.0, 12.0, 0.9, 5.0,
                    official_ok=False)
        report = next(e for e in core.tick(1.25) if e['tag'] == 'white')
        self.assertIsNotNone(report['official_x'])
        self.assertLess(math.dist((report['official_x'], report['official_y']),
                                  (10.0, 10.0)), 0.6)
        report = next(e for e in core.tick(2.1) if e['tag'] == 'white')
        self.assertIsNone(report['official_x'])

    def test_moving_cropped_head_is_withheld_but_stationary_head_can_report(self):
        track = vision.Track('brown', 0.0, 0.0, 0.9, (377, 472),
                             (70, 150), 3.5, 1.7, 100.0)
        for i in range(1, 9):
            t = 100.0 + i * 0.25
            track.update(i * 0.25, 0.0, 0.25, 0.9, (377, 472),
                         (70, 150), 3.5, 1.7, t)
        self.assertFalse(track.clipped_report_stable())
        for i in range(9, 20):
            t = 100.0 + i * 0.25
            track.update(2.0 + 0.02 * (i % 2), 0.0, 0.25, 0.9,
                         (377, 472), (70, 150), 3.5, 1.7, t)
        self.assertTrue(track.clipped_report_stable())

    def test_image_pose_uses_seconds_and_rotation(self):
        history = vision._pose_history
        history[:] = [(100.0, 0.0, 0.0, 4.0, np.eye(3)),
                      (101.0, 1.0, 0.0, 4.0,
                       np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]))]
        try:
            x, y, z, rotation = vision.pose_at_stamp(100.8, None)
            self.assertAlmostEqual(x, 0.8, places=5)
            self.assertAlmostEqual(y, 0.0)
            self.assertAlmostEqual(z, 4.0)
            self.assertAlmostEqual(np.linalg.det(rotation), 1.0, places=5)
            self.assertIsNone(vision.pose_at_stamp(99.0, None))
        finally:
            history.clear()

    def test_coasting_advances_state_once_per_elapsed_interval(self):
        track = vision.Track('red', 0.0, 0.0, 0.9, (0, 0), (20, 50),
                             5.0, 1.7, 100.0)
        track.vx = 1.0
        for _ in range(4):
            track.coast(0.2)
        self.assertAlmostEqual(track.x, 0.8, places=5)
        self.assertAlmostEqual(track.t, 100.8, places=5)
        self.assertEqual(track.last_seen_t, 100.0)

    def test_clipped_feet_use_head_ray(self):
        # Camera 4 m high, 5 m away. Visible bbox from head y≈413 to image edge 480;
        # treating 67 px as full height would estimate about 9.5 m.
        y_top = vision.CY + vision.FY * (4.0 - vision.H_ASSUMED) / 5.0
        result = vision.clipped_head_ground(vision.CX, y_top, 0.0, 0.0, 4.0,
                                            np.eye(3))
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[0], 5.0, places=4)
        self.assertAlmostEqual(result[2], 5.0, places=4)

    def test_bridge_retains_velocity_difference_baseline(self):
        core = yolo_target_bridge.TargetBridgeCore()
        for i in range(5):
            core.report(100.0 + i * 0.05, 'green', i * 0.05, 0.0, 0.95, 5.0)
        self.assertGreater(core.tracks['green'].vx, 0.2)

    def test_bridge_keepalive_keeps_real_observation_stamp(self):
        core = yolo_target_bridge.TargetBridgeCore()
        core.report(100.0, 'green', 1.0, 0.0, 0.95, 5.0)
        core.report(100.2, 'green', 1.2, 0.0, 0.95, 5.0)
        events = [e for e in core.tick(101.0) if e['tag'] == 'green']
        self.assertEqual(len(events), 1)
        self.assertAlmostEqual(events[0]['sample_s'], 100.2)

    def test_bridge_keepalive_caps_displacement_from_real_observation(self):
        bridge = yolo_target_bridge._mk_bridge()
        bridge.core.report(100.0, 'green', 10.0, 20.0, 0.95, 5.0)
        track = bridge.core.tracks['green']
        track.vx = 2.0
        bridge._gate_live['green'] = True
        bridge._pub_last['green'] = 100.3
        bridge._advance(100.8)
        bridge._keepalive(bridge._now())
        self.assertEqual(len(bridge._actor_pubs['green'].msgs), 1)
        self.assertLessEqual(abs(bridge._actor_pubs['green'].msgs[0].x - 10.0),
                             yolo_target_bridge.EXTRAP_MAX_D + 0.001)

    def test_stationary_visual_survives_short_camera_gap(self):
        bridge = yolo_target_bridge._mk_bridge()
        for i in range(5):
            bridge.core.report(100.0 + 0.3 * i, 'brown', 150.0, 9.0,
                               0.93, 5.0)
        bridge._gate_live['brown'] = True
        bridge._pub_last['brown'] = 101.2
        bridge._advance(102.5)  # 1.3 s without a fresh image
        bridge._keepalive(bridge._now())
        self.assertEqual(len(bridge._actor_pubs['brown'].msgs), 1)
        self.assertAlmostEqual(bridge._actor_pubs['brown'].msgs[0].x, 150.0)
        bridge._advance(5.0)  # stale visual must never be reported forever
        bridge._keepalive(bridge._now())
        self.assertEqual(len(bridge._actor_pubs['brown'].msgs), 1)

    def test_stale_red_coordinates_stop_reaching_judge(self):
        bridge = yolo_target_bridge._mk_bridge()
        for i in range(4):
            bridge.core.report(10.0 + i * 0.2, 'red1', -5.0, -9.0,
                               0.95, 5.0)
        bridge._advance(10.6)
        ev = next(e for e in bridge.core.tick(10.6) if e['tag'] == 'red1')
        bridge._emit(ev)
        self.assertEqual(len(bridge._actor_pubs['red1'].msgs), 1)
        bridge._advance(yolo_target_bridge.RED_OFFICIAL_FRESH_S + 0.1)
        ev = next(e for e in bridge.core.tick(bridge._now()) if e['tag'] == 'red1')
        bridge._emit(ev)
        bridge._keepalive(bridge._now())
        self.assertEqual(len(bridge._actor_pubs['red1'].msgs), 1)
        self.assertTrue(bridge.core.tracks['red1'].alive)

    def test_moving_actor_reports_more_often_without_changing_stationary_rate(self):
        bridge = yolo_target_bridge._mk_bridge()
        bridge._advance(1.0)
        bridge._publish_actor('red1', 0.0, 0.0)
        bridge._advance(0.5)
        bridge._publish_actor('red1', 0.5, 0.0)
        self.assertEqual(len(bridge._actor_pubs['red1'].msgs), 1)
        bridge.core.tracks['red1'].vx = 1.0
        bridge._publish_actor('red1', 0.5, 0.0)
        self.assertEqual(len(bridge._actor_pubs['red1'].msgs), 2)

    def test_close_follow_uses_fresh_target_velocity_with_safe_cap(self):
        vx, vy = swarm_agent.close_follow_velocity(0.0, 0.0, 0.0, 1.0, 0.1)
        self.assertAlmostEqual(vx, 0.0)
        self.assertAlmostEqual(vy, swarm_agent.CLOSE_TRANSIT_SPEED)
        stale = swarm_agent.close_follow_velocity(0.0, 0.0, 0.0, 1.0,
                                                  swarm_agent.TARGET_TTL + 1.0)
        self.assertEqual(stale, (0.0, 0.0))

    def test_red_actor_widening_range_triggers_close_chase_only_when_fresh(self):
        # 13:52 run: UAV2 observed red range 5.44 -> 6.20 m in ~1 s,
        # while the filtered tracker velocity remained far below its motion.
        self.assertTrue(swarm_agent.close_target_is_receding(
            6.20, 5.44, 1.05, 0.10, 0.36))
        self.assertFalse(swarm_agent.close_target_is_receding(
            6.20, 5.44, 1.05, 2.7, 0.36))
        self.assertFalse(swarm_agent.close_target_is_receding(
            6.20, 6.0, 1.05, 0.10, 0.36))
        self.assertFalse(swarm_agent.close_target_is_receding(
            5.8, 5.0, 1.05, 0.10, 0.36))
        # A deliberate camera backoff widens range even when the actor is
        # stationary; that alone must never start a chase back into its feet.
        self.assertFalse(swarm_agent.close_target_is_receding(
            6.20, 5.44, 1.05, 0.10, 0.0))

    def test_close_camera_retreats_from_actor_push_zone(self):
        # A 1 m/s target feedforward used to overpower a weak ring correction
        # at 6 m, keeping the person's feet outside the image.
        vx, vy = swarm_agent.ring_standoff_velocity(
            0.8, 0.0, 0.0, 0.0, 6.0, 0.0, 8.0)
        self.assertLess(vx, -0.34)
        self.assertLessEqual(math.hypot(vx, vy),
                             swarm_agent.CLOSE_TRANSIT_SPEED + 1e-6)

    def test_close_ring_brakes_search_velocity_without_faster_acceleration(self):
        # The live run entered the 8 m ring with a 2.87 m/s search command.
        # Normal slew kept > 2 m/s for several frames and carried the aircraft
        # into the actor's 7 m push zone.
        old = (2.87, -0.13)
        desired = (-0.18, 0.93)
        normal = swarm_agent.horizontal_slew_limit(*desired, old)
        close = swarm_agent.horizontal_slew_limit(*desired, old,
                                                  close_mode=True)
        self.assertLess(math.hypot(*close), math.hypot(*normal))
        self.assertLessEqual(math.dist(old, close),
                             swarm_agent.CLOSE_BRAKE_ACC / swarm_agent.CTRL_RATE + 1e-9)
        accelerating = swarm_agent.horizontal_slew_limit(3.0, 0.0, (0.95, 0.0),
                                                         close_mode=True)
        self.assertAlmostEqual(accelerating[0],
                               0.95 + swarm_agent.MAX_ACC / swarm_agent.CTRL_RATE)

    def test_observation_ring_stays_outside_actor_push_zone(self):
        # control_actor.py pushes pedestrians away from UAVs inside 7m.
        self.assertGreater(swarm_agent.ORBIT_RADIUS, 7.0)
        self.assertGreaterEqual(swarm_agent.ORBIT_RADIUS_CLOSE, 7.0)
        self.assertLess(swarm_agent.ORBIT_RADIUS_CLOSE,
                        yolo_target_bridge.ACTOR_PUB_MAX_RANGE_M)

    def test_observation_ring_near_north_edge_has_interior_goal(self):
        goal = swarm_agent.observation_ring_goal(
            129.3, 62.0, 129.0, 59.7, 8.0,
            (-55.0, 155.0, -65.0, 65.0), margin=2.5)
        self.assertLessEqual(goal[1], 62.5)
        self.assertAlmostEqual(math.dist(goal, (129.0, 59.7)), 8.0, places=5)
        # The chosen segment must move away from the actor instead of cutting
        # through its body to reach the south half of the circle.
        start_dist = math.dist((129.3, 62.0), (129.0, 59.7))
        midway = ((129.3 + goal[0]) / 2, (62.0 + goal[1]) / 2)
        self.assertGreaterEqual(math.dist(midway, (129.0, 59.7)), start_dist)

    def test_crash_recovery_timeout_clears_front_branch_latch(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'typhoon_h480_3'
        agent._takeoff_done = True
        agent._crash_flagged = True
        agent._crash_t0 = 100.0
        agent._crash_cnt = 12
        agent.local_z = 1.2
        agent.altitude_layer = 3.5
        published = []
        agent._publish_pos_output = lambda *args: published.append(args)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(104.0)):
            agent._send_vel(0.0, 0.0)
        self.assertFalse(agent._crash_flagged)
        self.assertIsNone(agent._crash_t0)
        self.assertEqual(agent._crash_cnt, 0)
        self.assertEqual(len(published), 1)
        self.assertEqual(published[0][-1], 0.0)

    def test_ekf_frozen_drone_is_not_counted_as_search_resource(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'typhoon_h480_3'
        agent.local_xy = (0.0, 0.0)
        agent.offset = (0.0, 0.0)
        agent.local_z = 1.2
        agent.state = SimpleNamespace(connected=True)
        agent._ekf_frozen = True
        agent._crash_flagged = False
        agent.assignment = None
        agent.cov_grid = SimpleNamespace(world_to_cell=lambda *_args: (0, 0))
        published = []
        agent.status_pub = SimpleNamespace(publish=published.append)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)):
            agent._publish_status()
        self.assertEqual(len(published), 1)
        self.assertFalse(published[0].connected)

    def test_aborted_tracking_waits_facing_target_instead_of_searching(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'typhoon_h480_5'
        agent.local_xy = (0.0, 0.0)
        agent.offset = (0.0, 0.0)
        agent.local_z = 3.0
        agent._landing = False
        agent._takeoff_done = True
        agent.assignment = SimpleNamespace(task_type=1, target_id='t2',
                                           target_x=3.0, target_y=0.0)
        agent._orbit_target = 't2'
        agent._confirm_close = False
        agent._update_orbit = lambda: self.fail('tracking fell into search')
        speeds = []
        agent._send_vel = lambda vx, vy: speeds.append((vx, vy))
        with patch.object(swarm_agent, 'CLAIM_ENABLE', False), \
             patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)):
            agent._abort_orbit('reproduce missing pursuit pointer', clear_path=False)
            agent._control()
        self.assertEqual(agent.assignment.task_type, 1)
        self.assertIsNone(agent._target_to_orbit)
        self.assertEqual(agent._look_at, (3.0, 0.0))
        self.assertEqual(speeds, [(0.0, 0.0)])

    def test_tracking_assignment_cannot_start_four_direction_search_scan(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.assignment = SimpleNamespace(task_type=1)
        agent._search_scan_key = None
        agent._look_at = (3.0, 0.0)
        speeds = []
        agent._send_vel = lambda vx, vy: speeds.append((vx, vy))
        agent._search_scan_step()
        self.assertEqual(agent._look_at, (3.0, 0.0))
        self.assertEqual(speeds, [(0.0, 0.0)])

    def test_search_cell_requires_four_observed_headings(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'typhoon_h480_0'
        agent.local_xy = (0.0, 0.0)
        agent.offset = (0.0, 0.0)
        agent.local_z = 3.0
        agent.state = SimpleNamespace(connected=True)
        agent.assignment = SimpleNamespace(task_type=0, cell_ix=2, cell_iy=3,
                                           target_x=0.0, target_y=0.0)
        agent.cov_grid = SimpleNamespace(world_to_cell=lambda *_args: (2, 3))
        agent.status_pub = SimpleNamespace(publish=lambda st: statuses.append(st))
        agent._search_scan_key = (2, 3, 0.0, 0.0)
        agent._search_scan_yaw0 = None
        agent._search_scan_phase = 0
        agent._search_scan_settle_t = None
        agent._search_scan_started_t = None
        agent._search_scan_done = False
        agent._search_scan_failed = False
        agent._send_vel = lambda vx, vy: speeds.append((vx, vy))
        statuses, speeds = [], []
        for phase in range(4):
            agent.yaw = phase * math.pi / 2.0
            for t in (100.0 + phase, 100.3 + phase):
                with patch.object(swarm_agent.rospy.Time, 'now',
                                  return_value=swarm_agent.rospy.Time(t)):
                    agent._search_scan_step()
                    agent._publish_status()
            if phase < 3:
                self.assertEqual(statuses[-1].confidence, 0.0)
        self.assertTrue(agent._search_scan_done)
        self.assertEqual(statuses[-1].confidence, 1.0)
        self.assertTrue(all(v == (0.0, 0.0) for v in speeds))

    def test_manager_does_not_cover_arrival_without_camera_scan(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.uav_ids = ['u0']
        manager.last_report = {'u0': swarm_manager.rospy.Time(99.5)}
        manager._active_leases = {'u0': (2, 3)}
        manager._lease_hold_since = {('u0', (2, 3)): 99.0}
        status = SimpleNamespace(connected=True, x=1.0, y=2.0,
                                 assigned_cell_ix=2, assigned_cell_iy=3,
                                 confidence=0.0)
        manager.status = {'u0': status}
        covered = []
        manager.grid = SimpleNamespace(is_covered=lambda *args: covered.append(args))
        manager._cell_waypoint = {(2, 3): (1.0, 2.0)}
        manager.lease = SimpleNamespace(renew=lambda *_args: True)
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(100.0)):
            manager._renew_leases()
        self.assertEqual(covered, [])
        self.assertIn('u0', manager._active_leases)

    def test_coverage_does_not_pass_through_occluding_wall(self):
        grid = swarm_task.CoverageGrid(-10.0, 10.0, -10.0, 10.0, 5.0)
        with patch.object(swarm_task, '_now', return_value=100.0):
            covered = grid.is_covered(-2.5, -2.5, 10.0,
                                      visible_fn=lambda x, _y: x < 0.0)
        self.assertTrue(covered)
        self.assertTrue(all(grid.cell(key).cx < 0.0 for key in covered))
        self.assertTrue(all(grid.cell(key).state != swarm_task.STATE_COVERED
                            for key in grid.cells if grid.cell(key).cx > 0.0))

    def test_manager_reclaims_lease_from_unavailable_drone(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.uav_ids = ['typhoon_h480_3']
        manager.last_report = {'typhoon_h480_3': swarm_manager.rospy.Time(99.5)}
        manager._active_leases = {'typhoon_h480_3': (2, 3)}
        manager._lease_hold_since = {('typhoon_h480_3', (2, 3)): 90.0}
        manager.status = {'typhoon_h480_3': SimpleNamespace(connected=False)}
        expired = []
        manager.lease = SimpleNamespace(force_expire=lambda *args: expired.append(args))
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(100.0)):
            manager._renew_leases()
        self.assertEqual(expired, [('typhoon_h480_3', (2, 3))])
        self.assertEqual(manager._active_leases, {})

    def test_search_lease_reclaims_stalled_drone_but_keeps_progressing_drone(self):
        def manager_at(x):
            manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
            manager.uav_ids = ['u0']
            manager.last_report = {'u0': swarm_manager.rospy.Time(120.0)}
            manager._active_leases = {'u0': (2, 3)}
            manager._lease_hold_since = {('u0', (2, 3)): 100.0}
            manager._lease_motion = {'u0': ((2, 3), 100.0, 0.0, 0.0)}
            manager.status = {'u0': SimpleNamespace(connected=True, x=x, y=0.0,
                                                     assigned_cell_ix=2, assigned_cell_iy=3,
                                                     confidence=0.0)}
            manager._cell_waypoint = {(2, 3): (50.0, 0.0)}
            manager.allocator = SimpleNamespace(failed_visit={})
            expired = []
            manager.lease = SimpleNamespace(force_expire=lambda *args: expired.append(args),
                                            renew=lambda *_args: True)
            return manager, expired

        stuck, expired = manager_at(0.1)
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(120.0)):
            stuck._renew_leases()
        self.assertEqual(expired, [('u0', (2, 3))])
        self.assertEqual(stuck._active_leases, {})
        self.assertIn((2, 3), stuck.allocator.failed_visit)

        moving, expired = manager_at(2.1)
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(120.0)):
            moving._renew_leases()
        self.assertEqual(expired, [])
        self.assertIn('u0', moving._active_leases)
        self.assertAlmostEqual(moving._lease_motion['u0'][1], 120.0)

    def test_search_flight_weight_prefers_nearby_unvisited_cell(self):
        grid = swarm_task.CoverageGrid(0, 50, 0, 20, 10)
        auction = swarm_task.TaskAllocator(grid, w_flight=0.16, w_zone=0)
        auction.mission_start = 0.0
        auction.early_phase_sec = 0.0
        with patch.object(swarm_task, '_now', return_value=100.0):
            def score(x):
                key = grid.world_to_cell(x, 5.0)
                return auction.utility('u0', 5.0, 5.0, key, [],
                                       task_counts={'u0': 0},
                                       other_uavs={'u1': (15.0, 20.0)})
            # The far cell receives more spread reward, yet a 30 m detour
            # must not outrank the nearby unvisited cell in the same sector.
            self.assertGreater(score(15.0), score(45.0))

    def test_agent_does_not_refresh_stale_vision_from_bridge_keepalive(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.targets = {}
        agent._target_state = {}
        agent._t_seen = {}
        agent.local_xy = (0.0, 0.0)
        agent.offset = (0.0, 0.0)
        agent._escape_bookkeeping = lambda _tid: None
        msg = SimpleNamespace(target_id='t0', x=1.0, y=0.0, vx=0.0, vy=0.0,
                              state=1, eliminated=False,
                              header=SimpleNamespace(stamp=swarm_agent.rospy.Time(100)))
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(104)), \
             patch.object(swarm_agent, 'TARGET_TTL', 2.5):
            agent._target_cb(msg)
            self.assertAlmostEqual(agent._t_seen['t0'], 100.0)
            self.assertIsNone(agent._nearest_target_dist())

    def test_team_confirm_does_not_slow_a_far_tracker(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'typhoon_h480_5'
        agent.local_xy = (0.0, 0.0)
        agent.offset = (0.0, 0.0)
        agent.targets = {'t2': (43.0, 0.0, 1.0, 0.0)}
        agent._t_seen = {'t2': 100.0}
        agent._track_assigned_id = 't2'
        agent._orbit_target = None
        agent._claims = {'t0': ('other_uav', 100.0)}
        agent._confirm_close = False
        agent.los = SimpleNamespace(visible=lambda *_args: True)
        agent._world_trusted = lambda: True
        entered = []
        agent._enter_close_mode = entered.append
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100)):
            agent._confirmed_cb(SimpleNamespace(data='tid:t2'))
            agent._confirmed_cb(SimpleNamespace(data='tid:t0'))
            self.assertEqual(entered, [])
            agent.targets['t2'] = (5.0, 0.0, 0.0, 0.0)
            agent._confirmed_cb(SimpleNamespace(data='tid:t2'))
            self.assertEqual(entered, ['t2'])

    def test_nearby_free_uav_takes_over_distant_primary(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._tracking = {'t2': 'far'}
        manager._backup = {}
        manager.status = {
            'far': SimpleNamespace(x=0.0, y=0.0, z=3.0, connected=True),
            'near': SimpleNamespace(x=38.0, y=0.0, z=3.0, connected=True),
        }
        manager._release_cooldown = {}
        released = []
        sent = []
        manager._free_search_lease = released.append
        manager.assign_pub = SimpleNamespace(publish=sent.append)
        track = SimpleNamespace(observers={'far'})
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(100)):
            manager._handoff_distant_tracker('t2', 40.0, 0.0, track)
        self.assertEqual(manager._tracking['t2'], 'near')
        self.assertIn('near', track.observers)
        self.assertNotIn('far', track.observers)
        self.assertEqual(released, ['near'])
        self.assertEqual((sent[0].uav_id, sent[0].task_type), ('far', 255))

    def test_formal_manager_dispatches_from_fresh_visual_without_truth_seeding(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._truth_cache = {}
        manager._truth_pos = {}
        manager._eliminated = set()
        manager._visual_candidate = {}
        manager._visual_quarantine = {}
        manager.tracker = cooperative_tracker.CooperativeTracker(official_only=True)
        msg = SimpleNamespace(target_id='t1', x=30.0, y=4.0,
                              eliminated=False,
                              header=SimpleNamespace(stamp=swarm_manager.rospy.Time(100)))
        with patch.object(swarm_manager, 'SEED_TRUTH', 0), \
             patch.object(swarm_manager, 'VISUAL_DISPATCH', 1), \
             patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(100.2)):
            manager._truth_cb(msg)
        self.assertIn('t1', manager.tracker.targets)
        self.assertEqual(manager._truth_cache['t1'], (30.0, 4.0, 100.0))
        self.assertEqual(manager._truth_pos, {})
        msg.target_id = 't3'
        msg.header.stamp = swarm_manager.rospy.Time(90)
        with patch.object(swarm_manager, 'SEED_TRUTH', 0), \
             patch.object(swarm_manager, 'VISUAL_DISPATCH', 1), \
             patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(100.2)):
            manager._truth_cb(msg)
        self.assertNotIn('t3', manager.tracker.targets)

    def test_manager_timeout_uses_real_visual_sample_age(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._truth_cache = {'t4': (1.0, 2.0, 100.0)}
        self.assertTrue(manager._has_fresh_visual('t4', 101.4))
        self.assertFalse(manager._has_fresh_visual('t4', 101.6))
        self.assertFalse(manager._has_fresh_visual('t5', 100.1))

    def test_backups_leave_searchers_for_unknown_actors(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.tracker = SimpleNamespace(targets={t: None for t in ('t0', 't2', 't4', 't5')})
        manager._left_seen = True
        manager._left_actors = set(range(6))
        manager._tracking = dict(zip(('t0', 't2', 't4', 't5'), ('u0', 'u1', 'u2', 'u3')))
        manager._backup = {}
        manager.status = {'u%d' % i: SimpleNamespace(connected=True, z=3.0)
                          for i in range(6)}
        self.assertFalse(manager._backup_search_capacity())
        manager.tracker.targets['t1'] = None
        self.assertTrue(manager._backup_search_capacity())

    def test_walking_target_moving_away_does_not_outpace_approach(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'typhoon_h480_3'
        agent.local_xy = (0.0, 0.0)
        agent.offset = (0.0, 0.0)
        agent.targets = {'t0': (11.0, 0.0, 1.0, 0.0)}
        agent._t_seen = {'t0': 100.0}
        agent._escape_spent = set()
        agent._flee_run_t = {}
        agent._spend_mode = set()
        agent._is_armed = lambda _tid: True
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100)):
            self.assertGreater(agent._approach_cap('t0', 11.0), 1.0)
            self.assertIn('t0', agent._spend_mode)
            agent._spend_mode.clear()
            agent.targets['t0'] = (11.0, 0.0, -1.0, 0.0)
            self.assertAlmostEqual(agent._approach_cap('t0', 11.0),
                                   swarm_agent.SPOOK_SPEED)

    def test_one_official_red_removal_does_not_kill_living_visual_slot(self):
        core = yolo_target_bridge.TargetBridgeCore()
        for i in range(3):
            core.report(100.0 + i * 0.2, 'red1', 1.0, 0.0, 0.95, 5.0)
            core.report(100.0 + i * 0.2, 'red2', 20.0, 0.0, 0.95, 5.0)
        self.assertEqual(core.set_left(101.0, '[0,1,2,3,4]'), [])
        self.assertFalse(core.tracks['red1'].elim_pending)
        self.assertFalse(core.tracks['red2'].elim_pending)
        self.assertEqual(set(core.set_left(102.0, '[0,1,2,3]')),
                         {'red1', 'red2'})

    def test_official_red_removal_retires_only_recently_reporting_visual_track(self):
        bridge = yolo_target_bridge._mk_bridge()
        bridge._now = lambda: 496.0
        for tag, x in (('red1', 149.5), ('red2', 120.0)):
            for i in range(3):
                bridge.core.report(490.0 + i * 0.2, tag, x, 5.0, 0.95, 5.0)
        bridge._red_prev_ids = {4, 5}
        bridge._pub_last = {'red1': 495.8, 'red2': 470.0}
        bridge._red_source_map = {('u3', 'red1'): 'red1',
                                  ('u5', 'red2'): 'red2'}
        bridge._note_red_left({5})
        self.assertTrue(bridge.core.tracks['red1'].elim_pending)
        self.assertFalse(bridge.core.tracks['red2'].elim_pending)
        self.assertNotIn(('u3', 'red1'), bridge._red_source_map)
        events = bridge.core.tick(496.1)
        self.assertTrue(any(ev['tag'] == 'red1' and ev['eliminated']
                            for ev in events))
        self.assertEqual(bridge._associate_red('u5', 'red1', 496.2,
                                               150.0, -13.9), 'red2')

    def test_bridge_red_departure_releases_manager_task_without_stale_timestamp(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._truth_cache = {'t5': (149.5, 5.0, 495.0)}
        manager._eliminated = set()
        target = SimpleNamespace(eliminated=False)
        manager.tracker = SimpleNamespace(targets={'t5': target})
        manager._confirm_done_t = {'t5': 480.0}
        released = []
        manager._release_tracker = lambda tid, reason='': released.append(tid)
        msg = SimpleNamespace(target_id='t5', eliminated=True, x=149.5, y=5.0,
                              header=SimpleNamespace(stamp=swarm_manager.rospy.Time(495.0)))
        with patch.object(swarm_manager, 'SEED_TRUTH', 0), \
             patch.object(swarm_manager, 'VISUAL_DISPATCH', 1), \
             patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(496.2)):
            manager._truth_cb(msg)
        self.assertEqual(released, ['t5'])
        self.assertTrue(target.eliminated)
        self.assertIn('t5', manager._eliminated)
        self.assertNotIn('t5', manager._truth_cache)

    def test_red_slots_from_different_aircraft_associate_by_position(self):
        bridge = yolo_target_bridge.YoloTargetBridge.__new__(
            yolo_target_bridge.YoloTargetBridge)
        bridge.core = yolo_target_bridge.TargetBridgeCore()
        bridge._red_source_map = {}
        bridge._last_source_sample = {}
        bridge.core.report(100.0, 'red1', -19.0, -9.7, 0.95, 5.0)
        bridge.core.report(100.0, 'red2', -16.5, -11.4, 0.95, 5.0)
        self.assertEqual(bridge._associate_red('uav0', 'red1', 100.2,
                                               -19.1, -9.7), 'red1')
        self.assertEqual(bridge._associate_red('uav0', 'red2', 100.2,
                                               -16.6, -11.3), 'red2')
        bridge._last_source_sample[('uav0', 'red1')] = 100.2
        bridge._last_source_sample[('uav0', 'red2')] = 100.2
        # uav2 uses the opposite names for the same two people.
        self.assertEqual(bridge._associate_red('uav2', 'red1', 100.3,
                                               -16.4, -11.4), 'red2')
        self.assertEqual(bridge._associate_red('uav2', 'red2', 100.3,
                                               -19.0, -9.6), 'red1')
        # A third red-looking object cannot corrupt either fresh track.
        self.assertIsNone(bridge._associate_red('uav3', 'red1', 100.4,
                                                -12.0, -8.0))

    def test_distinct_red_person_with_same_local_name_gets_second_slot(self):
        bridge = yolo_target_bridge.YoloTargetBridge.__new__(
            yolo_target_bridge.YoloTargetBridge)
        bridge.core = yolo_target_bridge.TargetBridgeCore()
        bridge._red_source_map = {}
        bridge._last_source_sample = {}
        bridge.core.report(100.0, 'red1', -20.0, -10.0, 0.95, 5.0)
        self.assertEqual(bridge._associate_red('uav2', 'red1', 100.2,
                                               15.0, -10.0), 'red2')

    def test_red_local_slot_swap_after_two_second_gap_keeps_spatial_identity(self):
        bridge = yolo_target_bridge.YoloTargetBridge.__new__(
            yolo_target_bridge.YoloTargetBridge)
        bridge.core = yolo_target_bridge.TargetBridgeCore()
        bridge._red_source_map = {}
        bridge._last_source_sample = {}
        bridge.core.report(100.0, 'red1', -37.7, -21.6, 0.95, 5.0)
        bridge.core.report(100.0, 'red2', -37.7, -25.0, 0.95, 5.0)
        self.assertEqual(bridge._associate_red('uav0', 'red1', 100.2,
                                               -37.7, -21.6), 'red1')
        bridge._last_source_sample[('uav0', 'red1')] = 100.2
        # The aircraft now calls the other red person red1.  Both global
        # tracks have aged past the fresh 2 s window, but the spatial anchor
        # is still valid and must beat the aircraft-local name.
        self.assertEqual(bridge._associate_red('uav0', 'red1', 102.5,
                                               -37.8, -25.0), 'red2')

    def test_team_confirmation_requires_official_removal(self):
        tracker = cooperative_tracker.CooperativeTracker(official_only=True)
        target = tracker.add_target('t0', 100.0)
        tracker.assign_observers('t0', ['uav0'])
        events = []
        for i in range(34):
            t = 100.0 + i * 0.5
            tracker.report('uav0', 't0', t, 0.0, 0.0)
            events.extend(tracker.update(t))
        self.assertIn(('t0', 'confirmed'), events)
        self.assertFalse(target.eliminated)
        self.assertTrue(target.confirmation_pending)

    def test_active_confirmation_does_not_dispatch_backup(self):
        tracker = cooperative_tracker.CooperativeTracker(official_only=True)
        target = tracker.add_target('t4', now=0.0)
        target.confirm_since = 10.0
        target.last_ok_t = 19.0
        self.assertEqual(tracker.needs_backup(20.0, stall_thresh=5.0), [])
        self.assertEqual(tracker.needs_backup(25.0, stall_thresh=5.0),
                         [('t4', 'stall=6.0s')])

    def test_six_aircraft_get_distinct_search_zones(self):
        grid = swarm_task.CoverageGrid(-100, 100, -50, 50)
        zones = [grid.get_uav_zone('typhoon_h480_%d' % i) for i in range(6)]
        self.assertEqual(zones, list(range(6)))

    def test_search_map_includes_fixed_actor_plugin_boundary(self):
        path = ROOT / 'coordination/src/robocup_training_worlds/worlds/generated/robocup_base.json'
        with path.open() as f:
            md = json.load(f)
        b = md['bounds']
        self.assertGreater(b['x_max'], 150.0)
        self.assertLess(b['x_min'], -50.0)
        self.assertIsNotNone(swarm_task.CoverageGrid(
            b['x_min'], b['x_max'], b['y_min'], b['y_max'])
            .world_to_cell(150.0, 36.81))
        self.assertEqual(md['grid']['width'] * md['grid']['height'],
                         md['grid']['data'][0][1])

    def test_covered_search_cell_releases_stale_active_lease(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        grid = swarm_task.CoverageGrid(-7, 7, -7, 7)
        key = next(iter(grid.cells))
        lease = swarm_task.LeaseManager(grid)
        lease.grant('u0', key, 100.0, 20.0)
        grid.cell(key).state = swarm_task.STATE_COVERED
        grid.cell(key).owner = None
        manager.uav_ids = ['u0']
        manager.last_report = {'u0': swarm_manager.rospy.Time(100)}
        manager._active_leases = {'u0': key}
        manager._lease_hold_since = {('u0', key): 10.0}
        manager._cell_waypoint = {}
        manager.grid = grid
        manager.status = {'u0': SimpleNamespace(x=0.0, y=0.0)}
        manager.lease = lease
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(101)):
            manager._renew_leases()
        self.assertEqual(manager._active_leases, {})
        self.assertEqual(manager._lease_hold_since, {})

    def test_far_target_does_not_cancel_search_lease(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.grid = swarm_task.CoverageGrid(-7, 7, -7, 7)
        key = next(iter(manager.grid.cells))
        manager.lease = swarm_task.LeaseManager(manager.grid)
        manager.lease.grant('u0', key, 100.0, 30.0)
        manager._active_leases = {'u0': key}
        manager._lease_hold_since = {('u0', key): 100.0}
        manager.uav_ids = ['u0']
        manager.status = {'u0': SimpleNamespace(x=0.0, y=0.0,
                                                connected=True)}
        manager._tracking = {}
        manager._sanitize_dispatch_xy = lambda x, y, _tid: (x, y)
        manager._idle_uavs = lambda: []
        published = []
        manager.assign_pub = SimpleNamespace(publish=published.append)
        with patch.dict(swarm_manager.os.environ, {'DISPATCH_FAR_LIMIT': '60'}):
            manager._dispatch_tracker('t0', 100.0, 0.0)
        cell = manager.grid.cell(key)
        self.assertEqual(cell.state, swarm_task.STATE_ASSIGNED)
        self.assertEqual(cell.owner, 'u0')
        self.assertEqual(manager._active_leases, {'u0': key})
        self.assertEqual(published, [])

    def test_search_does_not_reopen_old_cells_while_unvisited_cells_remain(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.grid = swarm_task.CoverageGrid(-14, 14, -7, 7)
        keys = list(manager.grid.cells)
        manager.grid.cell(keys[0]).state = swarm_task.STATE_COVERED
        manager.grid.cell(keys[1]).state = swarm_task.STATE_FREE
        manager._blocked_cells = set()
        manager.status = {}
        self.assertEqual(manager._reopen_covered_cells(), 0)
        self.assertEqual(manager.grid.cell(keys[0]).state,
                         swarm_task.STATE_COVERED)

    def test_staggered_takeoff_keeps_first_search_in_each_home_zone(self):
        # Each aircraft enters the auction in a different round. A global
        # corner bonus used to reset its quota per round and send all six west.
        grid = swarm_task.CoverageGrid(-55, 135, -65, 65)
        with patch.object(swarm_task, '_now', return_value=100.0):
            allocator = swarm_task.TaskAllocator(grid)
            self.assertIsNone(allocator.mission_start)
            for i in range(6):
                uid = 'typhoon_h480_%d' % i
                key = allocator.allocate({uid: (-50.0, -27.0 + 10.0 * i)})[uid]
                self.assertEqual(grid.get_zone_id(key), grid.get_uav_zone(uid),
                                 (uid, key))
            self.assertEqual(allocator.mission_start, 100.0)

    def test_home_search_zone_can_be_crossed_after_initial_spread(self):
        grid = swarm_task.CoverageGrid(-55, 135, -65, 65)
        with patch.object(swarm_task, '_now', return_value=500.0), \
             patch.object(swarm_task, 'SEARCH_HARD_HOME_ZONE', True):
            allocator = swarm_task.TaskAllocator(grid)
            allocator.mission_start = 100.0
            crossed = 0
            for i in range(6):
                uid = 'typhoon_h480_%d' % i
                key = allocator.allocate({uid: (-50.0, -25.0)})[uid]
                crossed += grid.get_zone_id(key) != grid.get_uav_zone(uid)
            self.assertGreater(crossed, 0)

    def test_position_mode_preserves_vertical_and_yaw(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        sent = []
        agent.uav_id = 'typhoon_h480_0'
        agent.local_xy = (0.0, 0.0)
        agent.local_z = 4.0
        agent.offset = (0.0, 0.0)
        agent.yaw = 0.0
        agent._landing = False
        agent.pos_pub = SimpleNamespace(publish=sent.append)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100)):
            agent._publish_pos_output(0.0, 0.0, 4.2, 0.0, 0.6)
            self.assertAlmostEqual(sent[-1].pose.position.z, 4.0)
            self.assertGreater(sent[-1].pose.orientation.z, 0.0)
            agent._publish_pos_output(0.0, 0.0, 4.2, -1.0, 0.0)
            self.assertLess(sent[-1].pose.position.z, 4.0)

    def test_friend_guard_rejects_velocity_toward_peer_at_cruise_height(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'typhoon_h480_0'
        agent.local_xy = (0.0, 0.0)
        agent.offset = (0.0, 0.0)
        agent.local_z = 3.0
        agent.altitude_layer = 3.0
        agent._friend_positions = {'typhoon_h480_1': (4.0, 0.0, 3.0)}
        agent._friend_seen = {'typhoon_h480_1': 100.0}
        agent._adaptive_speed = lambda vx, vy: (vx, vy)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100)), \
             patch.object(swarm_agent.rospy, 'loginfo_throttle'):
            vx, vy = agent._apply_friend_avoidance(1.0, 0.0)
        self.assertLessEqual(vx, 0.01, (vx, vy))

    def test_final_laser_check_stops_post_slew_forward_motion(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.yaw = 0.0
        agent._scan_t = 100.0
        readings = [10.0] * 361
        readings[180] = 1.0
        agent._scan = SimpleNamespace(ranges=readings, angle_min=-math.pi,
                                      angle_increment=math.pi / 180.0,
                                      range_max=10.0)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100)), \
             patch.object(swarm_agent, 'RADAR_GUARD', True):
            self.assertEqual(agent._final_scan_cap(1.0, 0.0), (0.0, 0.0))

    def test_close_side_wall_limits_parallel_speed_and_blocks_inward_motion(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.yaw = 0.0
        agent._scan_t = 100.0
        readings = [10.0] * 361
        readings[90] = 0.6  # -90 degrees, wall to the right
        agent._scan = SimpleNamespace(ranges=readings, angle_min=-math.pi,
                                      angle_increment=math.pi / 180.0,
                                      range_max=10.0)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100)), \
             patch.object(swarm_agent, 'RADAR_GUARD', True):
            parallel = agent._final_scan_cap(2.0, 0.0)
            self.assertLessEqual(math.hypot(*parallel),
                                 swarm_agent.RADAR_SIDE_PANIC_SPEED + 1e-6)
            self.assertEqual(agent._final_scan_cap(1.0, -1.0), (0.0, 0.0))

    def test_rear_body_echo_does_not_brake_forward_flight(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.yaw = 0.0
        agent._scan_t = 100.0
        readings = [10.0] * 271
        readings[255] = 0.5  # +120 degrees, outside forward ±100 safety sectors
        agent._scan = SimpleNamespace(ranges=readings,
                                      angle_min=-3.0 * math.pi / 4.0,
                                      angle_increment=math.pi / 180.0,
                                      range_max=10.0)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100)), \
             patch.object(swarm_agent, 'RADAR_GUARD', True):
            self.assertEqual(agent._final_scan_cap(2.0, 0.0), (2.0, 0.0))
            direction = math.radians(120.0)
            self.assertEqual(agent._final_scan_cap(math.cos(direction),
                                                   math.sin(direction)), (0.0, 0.0))

    def test_laser_preserves_clear_side_motion_when_front_is_blocked(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.yaw = 0.0
        agent._scan_t = 100.0
        agent._radar_prev = None
        readings = [10.0] * 271
        readings[135] = 1.0  # front, with -135..+135 degree coverage
        agent._scan = SimpleNamespace(ranges=readings,
                                      angle_min=-3.0 * math.pi / 4.0,
                                      angle_increment=math.pi / 180.0,
                                      range_min=0.1, range_max=10.0)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100)), \
             patch.object(swarm_agent, 'RADAR_GUARD', True):
            vx, vy = agent._radar_guard_velocity(0.0, 1.0)
        self.assertAlmostEqual(vx, 0.0)
        self.assertGreater(vy, 0.0)
        self.assertLessEqual(vy, swarm_agent.RADAR_SIDE_PANIC_SPEED)


if __name__ == '__main__':
    unittest.main()
