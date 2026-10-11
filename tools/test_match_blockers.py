"""Small regressions for six-actor match blockers; run with ROS environments sourced."""
import ast
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
    def test_official_view_numpy_values_are_json_booleans(self):
        # 01:34 run: cameras 2 and 5 crashed on a clipped box whose stability
        # predicate returned numpy.bool_(False). Cover every return branch.
        for uv, wh, distance in [((377., 472.), (100., 200.), 5.),
                                 ((377., 472.), (100., 200.), 8.),
                                 ((400., 380.), (100., 200.), 8.),
                                 ((2., 380.), (100., 200.), 5.)]:
            for stable in (False, True):
                with self.subTest(uv=uv, distance=distance, stable=stable):
                    expected = vision.official_view_ok(uv, wh, range_m=distance,
                                                       clipped_stable=stable)
                    result = vision.official_view_ok(
                        np.array(uv), np.array(wh), range_m=np.float64(distance),
                        clipped_stable=np.bool_(stable))
                    self.assertIs(type(result), bool)
                    self.assertEqual(result, expected)
                    self.assertIs(json.loads(json.dumps({'official_view_ok': result}))[
                        'official_view_ok'], expected)

    def test_formal_swarm_nodes_do_not_subscribe_to_judge_or_gazebo_truth(self):
        for name in ('swarm_manager.py', 'swarm_agent.py'):
            path = ROOT / 'coordination/src/robocup_swarm/scripts' / name
            tree = ast.parse(path.read_text(encoding='utf-8'))
            topics = []
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                if not isinstance(node.func, ast.Attribute):
                    continue
                if node.func.attr not in ('Subscriber', 'wait_for_message'):
                    continue
                if node.args and isinstance(node.args[0], ast.Constant):
                    topics.append(node.args[0].value)
            self.assertFalse(any(str(t).startswith('/find_actor_') for t in topics),
                             (name, topics))
            self.assertNotIn('/left_actors', topics, name)
            self.assertNotIn('/gazebo/model_states', topics, name)
            if name == 'swarm_manager.py':
                self.assertNotIn('MISSION_FINISHED', path.read_text(encoding='utf-8'))

    def test_visual_streak_releases_after_loss_or_bounded_tenure(self):
        should_release = swarm_manager.visual_streak_should_release
        self.assertFalse(should_release(100.0, 125.0, False,
                                        min_hold=30.0, max_fresh_hold=45.0))
        self.assertTrue(should_release(100.0, 131.0, False,
                                       min_hold=30.0, max_fresh_hold=45.0))
        self.assertFalse(should_release(100.0, 144.0, True,
                                        min_hold=30.0, max_fresh_hold=45.0))
        self.assertTrue(should_release(100.0, 145.0, True,
                                       min_hold=30.0, max_fresh_hold=45.0))

    def test_judge_topics_use_official_actor_info_contract(self):
        from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo as OfficialActorInfo

        self.assertEqual(OfficialActorInfo._type,
                         'ros_actor_cmd_pose_plugin_msgs/ActorInfo')
        self.assertEqual(OfficialActorInfo._slot_types,
                         ['string', 'float32', 'float32'])
        for relative_path in (
                'coordination/src/robocup_swarm/scripts/yolo_target_bridge.py',
                'perception/perception_real.py',
                'coordination/src/robocup_swarm/scripts/detection_to_official.py',
                'coordination/src/robocup_swarm/scripts/swarm_viz.py',
                'tools/score_diag.py'):
            tree = ast.parse((ROOT / relative_path).read_text(encoding='utf-8'))
            imports = {(node.module, alias.name)
                       for node in ast.walk(tree)
                       if isinstance(node, ast.ImportFrom)
                       for alias in node.names}
            self.assertIn(('ros_actor_cmd_pose_plugin_msgs.msg', 'ActorInfo'),
                          imports, relative_path)
            self.assertNotIn(('robocup_swarm.msg', 'ActorInfo'),
                             imports, relative_path)

        bridge = yolo_target_bridge._mk_bridge()
        bridge.core.report(100.0, 'green', 1.25, 2.5, 0.95, 5.0)
        bridge._advance(100.0)
        bridge._publish_actor('green', 1.25, 2.5)
        message = bridge._actor_pubs['green'].msgs[0]
        self.assertEqual(message._type, OfficialActorInfo._type)
        self.assertEqual((message.cls, message.x, message.y),
                         ('green', 1.25, 2.5))
        self.assertFalse(hasattr(message, 'header'))
        self.assertFalse(hasattr(message, 'z'))

    def test_unconfirmed_low_confidence_candidate_cannot_trigger_dispatch(self):
        bridge = yolo_target_bridge._mk_bridge()
        bridge.source_pub = yolo_target_bridge._FakePub()
        report = dict(target_id='blue', xyz=[150, 30, 1.25], confidence=.45,
                      range_m=9, source_uav='typhoon_h480_2', real_detection=True,
                      official_view_ok=True)
        for i in range(5):
            bridge._advance(100 if i == 0 else .2)
            report['sample_s'] = bridge._now()
            bridge._report_cb(SimpleNamespace(data=json.dumps(report)))
        self.assertFalse(bridge.core.tracks['blue'].alive)
        self.assertEqual(bridge.source_pub.msgs, [])

    def test_nearby_primary_without_any_real_image_eventually_needs_backup(self):
        tracker = cooperative_tracker.CooperativeTracker(
            official_only=True, strict_samples=True)
        target = tracker.add_target('t0', now=100.0)
        target.set_observers(['u0'])
        self.assertEqual(tracker.needs_backup(104.0, stall_thresh=5), [])
        self.assertEqual(tracker.needs_backup(105.1, stall_thresh=5),
                         [('t0', 'no_camera=5.1s')])

    def test_geometry_echo_cannot_refresh_manager(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._eliminated = set()
        # An 8m geometry detection must be rejected before any other state access.
        manager._detection_cb(SimpleNamespace(source=0, target_id='t0'))
        self.assertFalse(hasattr(manager, '_last_detect'))

    def test_real_source_direct_ingestion_and_echo_are_counted_once(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.uav_ids = ['u0']
        manager._visual_source, manager._visual_observations = {}, {}
        manager._last_detection_sample, manager._last_detect = {}, {}
        manager._eliminated = set()
        manager.status = {'u0': SimpleNamespace(x=8, y=0)}
        manager._visual_near_seen, manager._truth_cache = {}, {}
        manager.tracker = cooperative_tracker.CooperativeTracker(
            official_only=True, strict_samples=True, gap_tol=1.0)
        manager._truth_pos, manager._cur_targets = {}, {}
        manager._tracking = {'t0': 'u0'}
        manager._quarantined_visual = lambda *args: False
        manager._remember_visual_candidate = lambda *args: None
        manager._update_tracker_position = lambda *args: None
        payload = dict(target_id='t0', source_uav='u0', sample_s=100.0,
                       x=0.0, y=0.0, confidence=0.9, range_m=8.0,
                       official_view_ok=True, real_detection=True)
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(100.2)):
            with patch.object(manager.tracker, 'report', wraps=manager.tracker.report) as report:
                manager._visual_source_cb(SimpleNamespace(data=json.dumps(payload)))
                manager._visual_source_cb(SimpleNamespace(data=json.dumps(payload)))
                m = swarm_manager.TargetDetection()
                m.source, m.target_id, m.uav_id = 1, 't0', 'u0'
                m.header.stamp = swarm_manager.rospy.Time(100)
                m.x = m.y = 0.0
                manager._detection_cb(m)
                self.assertEqual(report.call_count, 1)
            self.assertEqual(manager._last_detect['t0'], 100.0)
            payload['sample_s'], payload['official_view_ok'] = 100.1, False
            with patch.object(manager.tracker, 'report') as report:
                manager._visual_source_cb(SimpleNamespace(data=json.dumps(payload)))
                report.assert_not_called()  # partial view still updates pursuit
            self.assertAlmostEqual(manager._last_detect['t0'], 100.1)
            self.assertFalse(manager._visual_observations[('u0', 't0')]['eligible'])
            self.assertTrue(manager._has_fresh_visual('t0', 100.2))
            self.assertFalse(manager._has_fresh_visual('t0', 101.01))
            self.assertEqual(manager._eligible_observations[('u0', 't0')]['sample_s'], 100.0)

    def test_cached_image_polling_does_not_advance_visual_streak_or_filter(self):
        target = cooperative_tracker.CooperativeTarget(
            't0', strict_samples=True, official_only=True, gap_tol=1.0)
        target.set_observers(['u0'])
        target.report('u0', 100.0, 0.0, 0.0)
        target.update(100.0)
        for now in (100.2, 100.4, 100.8):
            target.report('u0', 100.0, 9.0, 9.0)
            target.update(now)
            target.fused(now)
            self.assertEqual(target.progress(now), 0.0)
            self.assertEqual(target.last_ok_t, 100.0)
            self.assertEqual(target.t_fuse, 100.0)
        target.update(101.01)
        self.assertIsNone(target.confirm_since)

    def test_strict_real_images_reach_pending_only_and_gap_resets(self):
        tracker = cooperative_tracker.CooperativeTracker(
            official_only=True, strict_samples=True, obs_ttl=1.0, gap_tol=1.0)
        target = tracker.add_target('t0')
        target.set_observers(['u0'])
        for i in range(31):
            sample = 100+i*0.5
            tracker.report('u0', 't0', sample, 0.0, 0.0)
            events = tracker.update(sample)
        self.assertEqual(events, [('t0', 'confirmed')])
        self.assertTrue(target.confirmation_pending)
        self.assertFalse(target.eliminated)
        self.assertIsNone(target.last_err)
        tracker.update(116.1)
        self.assertEqual(target.progress(116.1), 0.0)
        tracker.report('u0', 't0', 116.2, 0, 0)
        tracker.update(116.2)
        self.assertEqual(target.confirm_since, 116.2)

    def test_local_confirm_needs_own_camera_and_stops_on_stale_frame(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id, agent._orbit_target = 'u0', 't0'
        agent._visual_observations = {}
        agent._confirm_start = agent._last_confirm_t = 0.0
        agent.detect_pub = SimpleNamespace(publish=lambda msg: None)
        payload = dict(target_id='t0', source_uav='u1', sample_s=100.0,
                       x=0, y=0, confidence=0.9, range_m=8,
                       real_detection=True, official_view_ok=True)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.1)):
            agent._visual_source_cb(SimpleNamespace(data=json.dumps(payload)))
            agent._update_orbit()
            self.assertEqual(agent._confirm_start, 0.0)
            payload['source_uav'] = 'u0'
            agent._visual_source_cb(SimpleNamespace(data=json.dumps(payload)))
            agent._update_orbit()
            self.assertEqual(agent._confirm_start, 100.0)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(101.1)):
            agent._update_orbit()
            self.assertEqual(agent._confirm_start, 0.0)

    def test_official_handoff_rejects_white_run_biased_camera(self):
        core = yolo_target_bridge.TargetBridgeCore()
        for i in range(10):
            core.report(340.0+i*0.2, 'white', 149.87, 32.70, .89, 11.3,
                        source_uav='camera5')
        tr = core.tracks['white']
        good = (tr.good_x, tr.good_y, tr.good_t)
        # Exact first bad source3 coordinate from the 21:28 CSV.
        core.report(341.98, 'white', 149.302, 31.345, .879, 9.49,
                    source_uav='camera3')
        self.assertEqual((tr.good_x, tr.good_y, tr.good_t), good)
        core.report(342.18, 'white', 149.239, 31.267, .868, 9.49,
                    source_uav='camera3')
        self.assertEqual((tr.good_x, tr.good_y, tr.good_t), good)
        self.assertEqual(tr.official_rejects, 4)  # 2 warmup + 2 handoff rejects
        core.report(342.3, 'white', 149.9, 32.4, .9, 9.0, source_uav='camera3')
        self.assertEqual(tr.official_source, 'camera5')
        core.report(342.5, 'white', 149.9, 32.4, .9, 9.0, source_uav='camera3')
        core.report(342.7, 'white', 149.9, 32.4, .9, 9.0, source_uav='camera3')
        self.assertEqual(tr.official_source, 'camera3')

    def test_handoff_extrapolates_coherent_motion_and_reacquires_expired_source(self):
        core = yolo_target_bridge.TargetBridgeCore()
        for i in range(10):
            t = 100.0+i*.2
            core.report(t, 'blue', 2*(t-100), 0, .9, 8, source_uav='a')
        core.report(102.0, 'blue', 4.0, 0, .9, 8, source_uav='b')
        self.assertEqual(core.tracks['blue'].official_source, 'a')
        core.report(102.2, 'blue', 4.4, 0, .9, 8, source_uav='b')
        core.report(102.4, 'blue', 4.8, 0, .9, 8, source_uav='b')
        self.assertEqual(core.tracks['blue'].official_source, 'b')
        # A disjoint instance needs six fresh frames even after a long gap.
        for i in range(6):
            core.report(110+i*.2, 'blue', 40+i*.1, 0, .9, 8, source_uav='c')
        self.assertEqual(core.tracks['blue'].official_source, 'c')
        self.assertAlmostEqual(core.tracks['blue'].good_x, 40.5)

    def test_newer_bad_camera_does_not_starve_older_trusted_images(self):
        core = yolo_target_bridge.TargetBridgeCore()
        for i in range(5):
            core.report(100+i*.2, 'white', 149.87, 32.7, .9, 9,
                        source_uav='good')
        core.report(101.1, 'white', 149.2, 31.2, .9, 9, source_uav='bad')
        core.report(101.0, 'white', 149.88, 32.69, .9, 9, source_uav='good')
        target = core.tracks['white']
        self.assertEqual(target.t_obs, 101.1)
        self.assertEqual(target.good_t, 101.0)
        self.assertEqual(target.official_source, 'good')

    def test_output_smoothing_updates_once_per_real_image(self):
        tk = vision.Track('white', 1, 2, .9, (350, 350), (30, 80), 8, 1.7, 100)
        with patch.object(vision, 'PUB_EMA', .5), patch.object(vision, 'LAG_COMP', 0):
            self.assertEqual(tk.pub_xy(), (1, 2))
            tk.observed_s = 100.2
            tk.x, tk.y = 3, 4
            expected = tk.pub_xy()
            self.assertEqual(expected, (2, 3))
            tk.x, tk.y = 5, 6  # coast/repeated callback is not a new image
            for _ in range(10):
                self.assertEqual(tk.pub_xy(), expected)

    def test_search_outer_y_row_remains_assignable_in_first_pass(self):
        grid = swarm_task.CoverageGrid(0, 28, 0, 21, cell_m=7, num_uavs=3)
        outer = next(k for k in grid.cells
                     if grid.get_zone_id(k) == 0 and grid.cell(k).cy == 3.5)
        inner = next(k for k in grid.cells
                     if grid.get_zone_id(k) == 0 and grid.cell(k).cy == 10.5)
        other_inner = next(k for k in grid.cells
                           if grid.get_zone_id(k) == 1 and grid.cell(k).cy == 10.5)
        for cell in grid.cells.values():
            cell.state = swarm_task.STATE_COVERED
        grid.cell(outer).state = swarm_task.STATE_FREE
        grid.cell(inner).state = swarm_task.STATE_FREE
        grid.cell(other_inner).state = swarm_task.STATE_FREE
        alloc = swarm_task.TaskAllocator(grid, w_flight=1.0, w_zone=0.0,
                                         w_novelty=0.0, cruise_speed=1.0)
        ox, oy = grid.cell(outer).cx, grid.cell(outer).cy
        with patch.object(swarm_task, '_now', return_value=200.0), \
             patch.object(swarm_task, 'SEARCH_HARD_HOME_ZONE', False):
            # One sector's boundary stays eligible while another sector still
            # has unseen interior; the old global filter hid it until last.
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
        manager.last_report = {uid: swarm_manager.rospy.Time(100.0)
                               for uid in manager.status}
        manager._visual_source = {}
        manager._visual_observations = {}
        manager._detection_cb = lambda msg: None
        manager._quarantined_visual = lambda *args: False
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(100.2)):
            manager._visual_source_cb(SimpleNamespace(data=json.dumps({
                'target_id': 't2', 'source_uav': 'uav_1',
                'sample_s': 100.0, 'x': 0.0, 'y': 0.0,
                'real_detection': True, 'confidence': 0.9})))
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
        manager.last_report = {uid: swarm_manager.rospy.Time(100.0)
                               for uid in manager.status}
        manager.assign_pub = SimpleNamespace(publish=lambda msg: None)
        manager._eligible_observations = {('uav3', 't0'): dict(
            sample_s=100., eligible=True, x=150., y=4.)}
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
                  'sample_s': 99.9, 'real_detection': True, 'official_view_ok': True}
        bridge._report_cb(SimpleNamespace(data=json.dumps(report)))
        self.assertEqual(len(bridge.source_pub.msgs), 0)
        report['sample_s'] = 100.0
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
        manager.last_report = {uid: swarm_manager.rospy.Time(100.0)
                               for uid in manager.status}
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

    def test_real_discovery_preempts_near_searcher_before_far_idle_drone(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.grid = swarm_task.CoverageGrid(-50, 50, -20, 20)
        key = manager.grid.world_to_cell(7.0, 0.0)
        manager.grid.cell(key).state = swarm_task.STATE_ASSIGNED
        manager.grid.cell(key).owner = 'near'
        manager.uav_ids = ['far', 'near']
        manager.status = {'far': SimpleNamespace(x=-35.0, y=0.0, connected=True),
                          'near': SimpleNamespace(x=7.0, y=0.0, connected=True)}
        manager.last_report = {uid: swarm_manager.rospy.Time(100.0)
                               for uid in manager.status}
        manager._tracking, manager._backup = {}, {}
        manager._visual_source, manager._release_cooldown = {}, {}
        manager._active_leases = {'near': key}
        manager._lease_hold_since = {('near', key): 100.0}
        manager._lease_motion = {}
        manager._idle_uavs = lambda: ['far']
        manager.lease = SimpleNamespace(force_expire=lambda *_args: True)
        sent = []
        manager.assign_pub = SimpleNamespace(publish=sent.append)
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time.from_sec(100.0)):
            manager._dispatch_tracker('t1', 0.0, 0.0)
        self.assertEqual(manager._tracking, {'t1': 'near'})
        self.assertEqual((sent[-1].uav_id, sent[-1].task_type), ('near', 1))

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

    def test_near_camera_observer_keeps_target_through_short_occlusion(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._tracking = {'t0': 'uav3'}
        manager._visual_near_seen = {'t0': (10.0, 2.0, 188.5)}
        manager._last_detect = {'t0': 188.5}
        manager.status = {'uav3': SimpleNamespace(x=11.0, y=2.0)}
        self.assertTrue(manager._hold_near_visual_target('t0', 191.6))
        self.assertFalse(manager._hold_near_visual_target('t0', 198.0))
        manager.status['uav3'].x = 30.0
        self.assertFalse(manager._hold_near_visual_target('t0', 190.0))

    def test_red_visual_slots_have_independent_reacquisition_holds(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._tracking = {'t4': 'uav2', 't5': 'uav3'}
        manager._visual_near_seen = {'t4': (10.0, 0.0, 123.2),
                                     't5': (40.0, 0.0, 123.2)}
        manager._last_detect = {'t4': 123.2, 't5': 123.2}
        manager.status = {'uav2': SimpleNamespace(x=10.0, y=0.0),
                          'uav3': SimpleNamespace(x=40.0, y=0.0)}
        self.assertTrue(manager._hold_near_visual_target('t4', 129.2))
        self.assertTrue(manager._hold_near_visual_target('t5', 129.2))
        manager._last_detect['t4'] = 100.0
        self.assertFalse(manager._hold_near_visual_target('t4', 129.2))
        self.assertTrue(manager._hold_near_visual_target('t5', 129.2))

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

    def test_stationary_world_track_is_not_a_camera_attached_false_box(self):
        track = vision.Track('green', 126.0, 45.0, .9,
                             (376, 350), (45, 90), 7.0, 1.7, 100.0)
        track.d_view = 4.0
        track.d_rel = 0.1  # nose lock alone would call it camera-attached
        track.recent_observed_xy = [
            (100.0, 126.0, 45.0), (100.2, 126.1, 45.1),
            (100.4, 126.0, 45.0), (100.6, 126.1, 45.0),
        ]
        self.assertFalse(track.attached_to_cam())
        # A box that follows the aircraft through world space remains gated.
        track.recent_observed_xy = [
            (100.0, 126.0, 45.0), (100.2, 126.8, 45.0),
            (100.4, 127.6, 45.0), (100.6, 128.4, 45.0),
        ]
        self.assertTrue(track.attached_to_cam())

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
        for i in range(3):
            core.report(100.+i*.2, 'green', 10.+i*.1, 0., 1., 8., source_uav='u0')
        self.assertTrue(core.tracks['green'].alive)
        core.tick(110.)
        self.assertFalse(core.tracks['green'].alive)
        for i in range(5):
            core.report(148.+i*.2, 'green', 150.+i*.1, 10., 1., 8., source_uav='u0')
            self.assertFalse(core.tracks['green'].alive)
            self.assertLess(core.tracks['green'].x, 11.)
            self.assertIsNone(yolo_target_bridge.official_point_at(core.tracks['green'], 148.+i*.2))
        core.report(149., 'green', 150.5, 10., 1., 8., source_uav='u0')
        self.assertTrue(core.tracks['green'].alive)
        self.assertIn('green', core._hypothesis_switches)

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

    def test_slam_replans_only_when_remaining_route_is_blocked(self):
        grid = SlamGrid(0.0, 0.0, 20.0, 20.0, 0.5, 0.0)
        route = [(2.25 + 0.5 * i, 10.25) for i in range(13)]
        self.assertFalse(swarm_agent.remaining_path_blocked(
            grid, route, (4.25, 10.25)))
        grid.mark_point(2.25, 10.25)  # Already traversed route prefix.
        grid.mark_point(6.25, 12.25)  # Map update away from the route.
        self.assertFalse(swarm_agent.remaining_path_blocked(
            grid, route, (4.25, 10.25)))
        grid.mark_point(6.25, 10.25)
        self.assertTrue(swarm_agent.remaining_path_blocked(
            grid, route, (4.25, 10.25)))
        # A side obstacle may inflate over the aircraft and first route metre
        # while the raw route remains free.  It must not cause a replan loop.
        margin = SlamGrid(0.0, 0.0, 20.0, 20.0, 0.5, 2.5)
        margin.mark_point(4.25, 12.25)
        self.assertFalse(margin.is_free(margin.world_to_cell((4.25, 10.25))))
        self.assertTrue(margin.is_free_raw(margin.world_to_cell((4.25, 10.25))))
        self.assertFalse(swarm_agent.remaining_path_blocked(
            margin, route, (4.25, 10.25)))
        margin.mark_point(5.25, 10.25)
        self.assertTrue(swarm_agent.remaining_path_blocked(
            margin, route, (4.25, 10.25)))

    def test_slam_update_ignores_off_route_changes_but_replans_blocked_route(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'u0'
        agent.local_xy, agent.offset = (4.25, 10.25), (0.0, 0.0)
        agent.slam = agent.grid = SlamGrid(0.0, 0.0, 20.0, 20.0, 0.5, 0.0)
        agent.slam.mark_scan = lambda *_args: 1
        agent._scan = SimpleNamespace(ranges=[10.0])
        agent._scan_t = 100.0
        agent._slam_last_t = agent._slam_replan_t = 0.0
        agent._slam_pub_t = 100.0
        agent._slam_dirty = False
        agent.pitch = agent.roll = agent.yaw = 0.0
        agent.path = [(4.25, 10.25), (5.25, 10.25),
                      (6.25, 10.25), (7.25, 10.25)]
        agent.path_target = (7.25, 10.25)
        agent._plan_lock = threading.Lock()
        agent._plan_pending = None
        agent.occ_pub = SimpleNamespace(publish=lambda *_args: None)
        requested = []
        agent._request_plan = requested.append
        agent.grid.mark_point(6.25, 12.25)
        agent._slam_update()
        self.assertEqual(requested, [])
        agent.grid.mark_point(6.25, 10.25)
        agent._scan_t = 103.0
        agent._slam_update()
        self.assertEqual(requested, [(7.25, 10.25)])

    def test_failed_replan_drops_old_route_when_laser_now_blocks_it(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'u0'
        agent.local_xy, agent.offset = (4.25, 10.25), (0.0, 0.0)
        agent.grid = SlamGrid(0.0, 0.0, 20.0, 20.0, 0.5, 0.0)
        agent.grid.mark_point(6.25, 10.25)
        agent.path = [(4.25, 10.25), (5.25, 10.25),
                      (6.25, 10.25), (7.25, 10.25)]
        agent.path_target = (7.25, 10.25)
        agent._last_flight_v = (1.0, 0.0)
        agent._plan_lock = threading.Lock()
        agent._plan_pending = (7.25, 10.25)
        agent._plan_epoch = 0
        agent._plan_event = threading.Event()
        agent._plan_event.set()
        agent._compute_plan = lambda goal: ([], goal, False)
        calls = [0]

        def shutdown_after_one_plan():
            calls[0] += 1
            return calls[0] > 1

        with patch.object(swarm_agent.rospy, 'is_shutdown',
                          side_effect=shutdown_after_one_plan), \
                patch.object(swarm_agent.rospy.Time, 'now',
                             return_value=swarm_agent.rospy.Time(100.0)):
            agent._planner_loop()
        self.assertEqual(agent.path, [])
        self.assertEqual(agent._last_flight_v, (0.0, 0.0))

    def test_close_range_final_cap_never_invents_an_escape_direction(self):
        cap = swarm_agent.cap_inward_close_speed
        self.assertEqual(cap(0.8, 0.0, 0.0, 0.0, 5.0, 0.0), (0.0, 0.0))
        self.assertEqual(cap(-0.8, 0.0, 0.0, 0.0, 5.0, 0.0), (-0.8, 0.0))
        self.assertEqual(cap(0.0, 0.8, 0.0, 0.0, 5.0, 0.0), (0.0, 0.8))
        vx, vy = cap(0.8, 0.4, 0.0, 0.0, 6.0, 0.0)
        self.assertGreater(vx, 0.0)
        self.assertLess(vx, 0.8)
        self.assertAlmostEqual(vy / vx, 0.5)  # Radar-vetted bearing retained.
        self.assertEqual(cap(0.8, 0.0, 0.0, 0.0, 8.0, 0.0), (0.8, 0.0))

    def test_close_range_final_guard_yields_to_radar_and_friend(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'u0'
        agent.local_xy, agent.offset = (0.0, 0.0), (0.0, 0.0)
        agent.local_z = agent.altitude_layer = 3.5
        agent.assignment = SimpleNamespace(task_type=1, target_id='t0')
        agent._orbit_target = 't0'
        agent.targets = {'t0': (5.0, 0.0, 0.0, 0.0)}
        agent._t_seen = {'t0': 100.0}
        agent._radar_strict = False
        agent._radar_retreating = False
        agent._friend_positions = {}
        agent._friend_seen = {}
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)):
            self.assertEqual(agent._close_range_final_guard(0.8, 0.0),
                             (0.0, 0.0))
            agent._radar_strict = True
            self.assertEqual(agent._close_range_final_guard(0.8, 0.0),
                             (0.8, 0.0))

    def test_pixel_streak_only_holds_world_bearing_for_two_seconds(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'u0'
        agent.local_xy, agent.offset = (5.0, 5.0), (0.0, 0.0)
        agent.assignment = SimpleNamespace(task_type=0)
        agent._search_scan_key = (1, 1, 5.0, 5.0)
        agent._search_scan_started_t = None
        agent._camera_cue = None
        agent._camera_cue_cooldown = 0.0
        agent._camera_cue_seen = {}
        agent._team_tracks = {}
        commands = []
        agent._send_vel = lambda vx, vy: commands.append((vx, vy))
        cue = dict(purpose='camera_reacquire', source_uav='u0',
                   real_detection=True, sample_s=100.0,
                   candidate_id='white:17', bearing=0.5,
                   confidence=0.64, hits=3)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)):
            agent._visual_candidate_cb(SimpleNamespace(data=json.dumps(cue)))
            self.assertIsNotNone(agent._camera_cue)
            self.assertAlmostEqual(agent._camera_cue['until'], 102.0)
            self.assertTrue(agent._hold_camera_candidate())
            self.assertEqual(commands, [(0.0, 0.0)])
            self.assertAlmostEqual(agent._look_at[0], 5.0 + 10.0 * math.cos(0.5))
            self.assertAlmostEqual(agent._look_at[1], 5.0 + 10.0 * math.sin(0.5))
            # The bounded image-only look now also applies to blue.
            agent._camera_cue = None
            agent._camera_cue_cooldown = 0.0
            cue['candidate_id'] = 'blue:17'
            agent._visual_candidate_cb(SimpleNamespace(data=json.dumps(cue)))
            self.assertIsNotNone(agent._camera_cue)
            self.assertAlmostEqual(agent._camera_cue['until'], 102.0)
            self.assertIsNone(agent._camera_cue['xy'])
            agent._radar_strict = False
            agent._friend_positions = {'u1': (3.0, 0.0, 3.5)}
            agent._friend_seen = {'u1': 100.0}
            self.assertEqual(agent._close_range_final_guard(0.8, 0.0),
                             (0.8, 0.0))

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

    def test_prolonged_ineligible_view_recovers_then_releases_without_quarantine(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._visual_candidate = {'t1': (100.0, -27.0, 13.7)}
        manager._visual_quarantine = {}
        manager._visual_observations = {}
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
        manager._visual_observations = {('u2', 't1'):
            {'sample_s': 124.3, 'eligible': True}}
        manager._release_unverified_visuals(126.0)
        self.assertEqual(cancelled, [])  # 1.7 s camera gap is not a decoy
        manager._visual_observations = {}
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(126.0)):
            manager._release_unverified_visuals(126.0)
        self.assertEqual(cancelled, [])  # first allow bounded recovery
        manager._last_detect['t1'] = 146.0
        manager._visual_near_seen['t1'] = (-26.9, 13.8, 146.0)
        manager._release_unverified_visuals(146.0)
        self.assertEqual(cancelled, [('t1', 'unverified_visual')])
        self.assertIn('t1', manager.tracker.targets)  # still a pending person
        self.assertFalse(manager._quarantined_visual('t1', -27.0, 13.7, 147.0))
        self.assertEqual(published, [])  # no target-wide visual suppression

    def test_unverified_attempt_age_cannot_cancel_current_good_image(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._visual_candidate = {'t1': (100.0, -27.0, 13.7)}
        manager._visual_quarantine = {}
        manager._visual_observations = {('u2', 't1'):
            {'sample_s': 190.5, 'eligible': True}}
        manager._last_detect = {'t1': 190.5}
        manager._visual_near_seen = {'t1': (-26.9, 13.8, 190.5)}
        manager._tracking = {'t1': 'u2'}
        manager.status = {'u2': SimpleNamespace(x=-25.0, y=14.0)}
        manager.tracker = SimpleNamespace(targets={'t1': object()})
        manager._truth_cache = {'t1': (-27.0, 13.7, 190.5)}
        manager._cur_targets = {'t1': (-27.0, 13.7, 'u2')}
        manager._confirm_done_t = {}
        manager._target_hist = {}
        cancelled = []
        manager._release_tracker = lambda tid, reason: cancelled.append((tid, reason))
        manager.visual_reject_pub = SimpleNamespace(publish=lambda *_: None)
        manager._release_unverified_visuals(191.0)
        self.assertEqual(cancelled, [])

    def test_fresh_eligible_images_keep_stationary_actor_eligible(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._visual_candidate = {'t3': (100.0, 150.0, 35.0)}
        manager._visual_quarantine = {}
        manager._visual_observations = {('u5', 't3'):
            {'sample_s': 125.5, 'eligible': True}}
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
        manager._visual_observations = {}
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
        bridge._pub_last['green'] = 99.4
        bridge._advance(100.3)
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
        bridge._pub_last['brown'] = 100.9
        bridge._advance(101.7)  # only 0.5 s without a fresh image
        bridge._keepalive(bridge._now())
        self.assertEqual(len(bridge._actor_pubs['brown'].msgs), 1)
        self.assertAlmostEqual(bridge._actor_pubs['brown'].msgs[0].x, 150.0)
        bridge._advance(0.8)  # 1.3 s image gap must already stop reports
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
        agent.state = SimpleNamespace(connected=True, armed=True, mode='OFFBOARD')
        agent._takeoff_done = True
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
        agent.state = SimpleNamespace(connected=True, armed=True, mode='OFFBOARD')
        agent._takeoff_done = True
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
                    agent._processed_camera_frame_cb(SimpleNamespace(data=json.dumps({
                        'source_uav': agent.uav_id, 'sample_s': t,
                        'x': 0.0, 'y': 0.0, 'yaw': agent.yaw,
                        'processed': True})))
                    agent._search_scan_step()
                    agent._publish_status()
            if phase < 3:
                self.assertEqual(statuses[-1].confidence, 0.0)
        self.assertTrue(agent._search_scan_done)
        self.assertEqual(statuses[-1].confidence, 1.0)
        self.assertTrue(all(v == (0.0, 0.0) for v in speeds))

    def test_pixel_cue_survives_unreliable_ground_range_but_needs_three_images(self):
        tracker = vision.PixelCueTracker()
        box = dict(cls='blue', conf=0.91, u=208.0, v=240.0,
                   h=78.0, bearing=0.24)
        self.assertEqual(tracker.update(100.0, [box]), [])
        self.assertEqual(tracker.update(100.2, [box]), [])
        self.assertEqual(tracker.update(100.2, [box]), [])
        cues = tracker.update(100.4, [box])
        self.assertEqual(len(cues), 1)
        self.assertEqual(cues[0]['candidate_id'], 'blue:0')
        self.assertAlmostEqual(cues[0]['bearing'], 0.24)

    def test_white_cue_associates_across_yaw_without_promoting_official_range(self):
        tracker = vision.PixelCueTracker()
        # UAV5, 00:36 match: one white person swept u=66 -> 453 in three
        # independent images while the aircraft yawed. World bearing remains
        # stable; these low-confidence boxes are only a two-second look cue.
        # The y displacement is 3 m/s, the maximum search speed. Across
        # images 2->3, the true bearing changes by 0.16 rad, exceeding the
        # old hard 0.12 rad gate even though this is the same actor.
        samples = ((2723.408, 65.989, 422.836, 132.659, .704, 0.0),
                   (2723.616, 216.068, 424.376, 116.381, .485, .624),
                   (2724.016, 453.406, 459.138, 123.103, .639, 1.824))
        self.assertGreater(abs(math.atan2(-1.824, 7.5) -
                               math.atan2(-.624, 7.5)), .12)
        for i, (t, u, v, h, conf, cam_y) in enumerate(samples):
            cues = tracker.update(t, [dict(cls='white', conf=conf, u=u,
                                           v=v, h=h, cam_x=0.0,
                                           cam_y=cam_y,
                                           bearing=math.atan2(-cam_y, 7.5))])
            self.assertEqual(len(cues), 1 if i == 2 else 0)
        self.assertEqual(cues[0]['candidate_id'], 'white:0')
        self.assertEqual(cues[0]['hits'], 3)
        self.assertAlmostEqual(cues[0]['confidence'], .639)
        self.assertTrue(cues[0]['attention_only'])
        # Even a subsequent high-confidence image cannot let this mixed
        # quality cue bypass the independent geometry/official-view gates.
        cues = tracker.update(2724.204, [dict(cls='white', conf=.886,
            u=604.168, v=473.108, h=129.272, cam_x=0.0, cam_y=2.388,
            bearing=math.atan2(-2.388, 7.5))])
        self.assertEqual(cues[0]['candidate_id'], 'white:0')
        self.assertFalse(vision.pixel_promoted_strict(
            cues, 'white', 604.168, 473.108, .886, 8.0, 1.8,
            2.6, 129.272, 580.0, 630.0, 408.0, 752))

    def test_moderate_blue_streak_gets_attention_without_range_promotion(self):
        # 02:19 run UAV2 blue confidence streak: .608/.484/.585.
        # Keep image association independent of the later ground projection.
        for color in ('blue', 'green', 'brown', 'red'):
            tracker = vision.PixelCueTracker()
            for index, confidence in enumerate((.608, .484, .585)):
                self.assertTrue(vision.pixel_cue_box_ok(
                    color, confidence, 330., 247.5, 370., 352.5))
                cues = tracker.update(100.0 + .2*index, [dict(
                    cls=color, conf=confidence, u=350., v=300., h=105., bearing=.2)])
                self.assertEqual(len(cues), int(index == 2))
            self.assertTrue(cues[0]['attention_only'])
            self.assertFalse(vision.pixel_promoted_strict(
                cues, color, 350., 300., .9, 8., 1.8,
                2.6, 105., 330., 370., 247.5, 752))

    def test_pixel_attention_still_rejects_noise_and_short_streaks(self):
        for confidence in (.3, float('nan'), float('inf')):
            self.assertFalse(vision.pixel_cue_box_ok(
                'blue', confidence, 330., 247.5, 370., 352.5))
        self.assertFalse(vision.pixel_cue_box_ok('blue', .7, 10., 10., 500., 50.))
        self.assertFalse(vision.pixel_cue_box_ok('blue', .7, 10., 10., 12., 100.))
        tracker = vision.PixelCueTracker()
        box = dict(cls='blue', conf=.6, u=350., v=300., h=100., bearing=.2)
        for t in (100., 100., 102., 104.):
            self.assertEqual(tracker.update(t, [box]), [])

    def test_bearing_compensation_keeps_nearby_same_color_people_separate(self):
        tracker = vision.PixelCueTracker()
        tracker.update(100.0, [dict(cls='white', conf=.9, u=300.0,
            v=400.0, h=120.0, cam_x=0.0, cam_y=0.0, bearing=0.0)])
        # Another person is 2 m to the side at the same apparent size. A
        # translating UAV must not accept them as the first person's ray.
        tracker.update(100.4, [dict(cls='white', conf=.9, u=300.0,
            v=400.0, h=120.0, cam_x=0.0, cam_y=1.2,
            bearing=math.atan2(2.0 - 1.2, 7.5))])
        self.assertEqual(len(tracker.tracks), 2)

    def test_odd_shape_can_link_after_three_images_without_reporting_bad_range(self):
        tracker = vision.PixelCueTracker()
        box = dict(cls='blue', conf=.91, u=210.0, v=200.0,
                   h=105.0, bearing=.2)
        self.assertEqual(tracker.update(100.0, [box]), [])
        self.assertEqual(tracker.update(100.2, [box]), [])
        cues = tracker.update(100.4, [box])
        self.assertTrue(vision.pixel_promoted_strict(
            cues, 'blue', 210.0, 200.0, .91, 8.0, 1.8,
            3.5, 105.0, 195.0, 225.0, 147.5, 752))
        self.assertFalse(vision.pixel_promoted_strict(
            [], 'blue', 210.0, 200.0, .91, 8.0, 1.8,
            3.5, 105.0, 195.0, 225.0, 147.5, 752))
        track = vision.Track('blue', 8.0, 0.0, .91,
                             (210.0, 200.0), (30.0, 105.0), 8.0, 1.8,
                             100.4, loc_ok=False)
        track.hits = vision.CONFIRM_HITS
        self.assertEqual(track.verdict(100.4)[1], 'localization')
        for i in range(3):
            track.update(8.0, 0.0, .2, .91, (210.0, 200.0),
                         (36.0, 105.0), 8.0, 1.8, 100.6 + .2 * i,
                         loc_ok=True)
        self.assertFalse(track.loc_suspect)

    def test_search_scan_uses_only_fresh_processed_images_at_its_waypoint(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'typhoon_h480_0'
        agent.assignment = SimpleNamespace(task_type=0, target_x=0.0, target_y=0.0)
        agent._search_scan_key = (0, 0, 0.0, 0.0)
        agent._search_scan_seen = set()
        agent._search_scan_first = {}
        for sample, x, yaw in ((100.0, 0.0, 0.0),
                               (100.3, 5.0, 0.0),
                               (100.4, 0.0, math.pi / 2.0),
                               (100.5, 0.0, 0.0),
                               (100.7, 0.0, 0.0)):
            with patch.object(swarm_agent.rospy.Time, 'now',
                              return_value=swarm_agent.rospy.Time(sample)):
                agent._processed_camera_frame_cb(SimpleNamespace(data=json.dumps({
                    'source_uav': agent.uav_id, 'sample_s': sample,
                    'x': x, 'y': 0.0, 'yaw': yaw, 'processed': True})))
        self.assertEqual(agent._search_scan_seen, {0})

    def test_transit_coverage_requires_two_processed_frames_with_los(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.grid = swarm_task.CoverageGrid(-21, 21, -21, 21, num_uavs=1)
        manager.uav_ids = ['u0']
        manager.status = {'u0': SimpleNamespace(x=-3.5, y=-3.5, z=3.0,
                                                connected=True)}
        manager.allocator = SimpleNamespace(visit_time={}, last_seen={})
        manager._slam_lock = threading.Lock()
        manager._merged_grid = lambda: SimpleNamespace(
            is_free=lambda *_args: True, resolution=1.0,
            origin=(-21.0, -21.0))
        front = manager.grid.world_to_cell(3.5, -3.5)
        behind = manager.grid.world_to_cell(-10.5, -3.5)
        for sample in (100.0, 100.3):
            payload = dict(source_uav='u0', sample_s=sample,
                           x=-3.5, y=-3.5, z=3.0, yaw=0.0,
                           down_pitch=0.0, processed=True)
            with patch.object(swarm_manager.rospy.Time, 'now',
                              return_value=swarm_manager.rospy.Time.from_sec(sample)):
                manager._processed_camera_frame_cb(
                    SimpleNamespace(data=json.dumps(payload)))
            if sample == 100.0:
                self.assertEqual(manager.grid.cell(front).state,
                                 swarm_task.STATE_FREE)
        self.assertEqual(manager.grid.cell(front).state,
                         swarm_task.STATE_COVERED)
        self.assertEqual(manager.grid.cell(behind).state,
                         swarm_task.STATE_FREE)
        self.assertFalse(manager._in_processed_view(
            -3.5, -3.5, 0.0, -1.5, -3.5, 3.0, 0.0))
        blocked = swarm_task.CoverageGrid(-21, 21, -21, 21, num_uavs=1)
        manager.grid = blocked
        manager._camera_cell_evidence = {}
        with patch.object(swarm_manager.LineOfSight, 'visible',
                          return_value=False):
            for sample in (101.0, 101.3):
                payload['sample_s'] = sample
                with patch.object(swarm_manager.rospy.Time, 'now',
                                  return_value=swarm_manager.rospy.Time.from_sec(sample)):
                    manager._processed_camera_frame_cb(
                        SimpleNamespace(data=json.dumps(payload)))
        self.assertEqual(blocked.cell(front).state, swarm_task.STATE_FREE)

    def test_camera_cue_pauses_only_unowned_search_target_and_expires(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'typhoon_h480_0'
        agent.local_xy, agent.offset = (0.0, 0.0), (0.0, 0.0)
        agent.assignment = SimpleNamespace(task_type=0)
        agent._search_scan_key = (2, 3, 0.0, 0.0)
        agent._team_tracks = {'t1': {'typhoon_h480_1'}}
        agent._team_visual = {('typhoon_h480_1', 't1'):
                             dict(sample_s=100.0, since_s=99.5, hits=3,
                                  x=8.0*math.cos(.3), y=8.0*math.sin(.3))}
        agent._look_at = None
        agent._send_vel = lambda *_args: None
        msg = SimpleNamespace(data=json.dumps(dict(
            source_uav=agent.uav_id, sample_s=100.0,
            purpose='camera_reacquire', real_detection=True,
            candidate_id='blue:4', bearing=0.3, confidence=.91, hits=3)))
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time.from_sec(100.1)):
            agent._visual_candidate_cb(msg)
            self.assertFalse(agent._hold_camera_candidate())
            agent._team_tracks.clear()
            agent._visual_candidate_cb(msg)
            self.assertTrue(agent._hold_camera_candidate())
            self.assertAlmostEqual(math.atan2(agent._look_at[1], agent._look_at[0]), .3)
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time.from_sec(102.2)):
            self.assertFalse(agent._hold_camera_candidate())

    def test_far_approach_lease_survives_position_ttl_only_while_moving(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.status = {'u0': SimpleNamespace(x=0.0, y=0.0, connected=True)}
        manager._tracking = {'t1': 'u0'}
        manager._record_approach('t1', 'u0', 45.0, 0.0, 100.0)
        self.assertTrue(manager._hold_approaching_target('t1', 103.5))
        self.assertFalse(manager._hold_approaching_target('t1', 106.0))
        manager.status['u0'].x = 7.0
        self.assertTrue(manager._hold_approaching_target('t1', 106.1))
        self.assertFalse(manager._hold_approaching_target('t1', 122.1))

    def test_route_continues_nearby_and_failed_cell_is_not_reissued(self):
        grid = swarm_task.CoverageGrid(-21, 21, -21, 21, num_uavs=1)
        auction = swarm_task.TaskAllocator(grid)
        with patch.object(swarm_task, '_now', return_value=100.0):
            first = auction.allocate({'uav_0': (-17.5, -17.5)})['uav_0']
        self.assertIsNotNone(first)
        grid.cell(first).state = swarm_task.STATE_FREE
        grid.cell(first).owner = None
        auction.failed_visit[first] = 101.0
        with patch.object(swarm_task, '_now', return_value=101.0):
            second = auction.allocate({'uav_0': (grid.cell(first).cx,
                                                  grid.cell(first).cy)})['uav_0']
        self.assertIsNotNone(second)
        self.assertNotEqual(second, first)
        self.assertLessEqual(math.hypot(grid.cell(first).cx - grid.cell(second).cx,
                                        grid.cell(first).cy - grid.cell(second).cy), 30.0)

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

    def test_completed_search_scan_does_not_mark_ground_beneath_camera_seen(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.grid = swarm_task.CoverageGrid(-14, 14, -14, 14, num_uavs=1)
        key = manager.grid.world_to_cell(-3.5, -3.5)
        manager.lease = swarm_task.LeaseManager(manager.grid)
        manager.lease.grant('u0', key, 99.0, 20.0)
        manager.uav_ids = ['u0']
        manager.last_report = {'u0': swarm_manager.rospy.Time.from_sec(99.5)}
        manager._active_leases = {'u0': key}
        manager._lease_hold_since = {('u0', key): 99.0}
        manager._lease_motion = {}
        manager._cell_waypoint = {key: (-3.5, -3.5)}
        manager.status = {'u0': SimpleNamespace(connected=True, x=-3.5,
            y=-3.5, assigned_cell_ix=key[0], assigned_cell_iy=key[1],
            confidence=1.0)}
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time.from_sec(100.0)):
            manager._renew_leases()
        self.assertNotIn('u0', manager._active_leases)
        self.assertEqual(manager.grid.cell(key).state,
                         swarm_task.STATE_REVIEW)

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

        sideways, expired = manager_at(0.0)
        sideways.status['u0'].y = 3.0
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(120.0)):
            sideways._renew_leases()
        self.assertEqual(expired, [('u0', (2, 3))])
        self.assertNotIn('u0', sideways._active_leases)

        detouring, expired = manager_at(0.0)
        detouring.status['u0'].y = 3.0
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(110.0)):
            detouring._renew_leases()
        self.assertEqual(expired, [])
        self.assertIn('u0', detouring._active_leases)

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
        manager.last_report = {uid: swarm_manager.rospy.Time(100.0)
                               for uid in manager.status}
        manager._eligible_observations = {('near', 't2'): dict(
            eligible=True, sample_s=100., since_s=99.4, hits=3, x=40., y=0.)}
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
        self.assertEqual([(m.uav_id, m.task_type) for m in sent],
                         [('near', 1), ('far', 255)])

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
        manager._visual_observations = {('u0', 't4'):
            {'sample_s': 100.0, 'eligible': True}}
        self.assertTrue(manager._has_fresh_visual('t4', 100.9))
        self.assertFalse(manager._has_fresh_visual('t4', 101.4))
        self.assertFalse(manager._has_fresh_visual('t4', 101.6))
        self.assertFalse(manager._has_fresh_visual('t5', 100.1))
        manager._visual_observations[('u0', 't4')]['eligible'] = False
        self.assertFalse(manager._has_fresh_visual('t4', 100.9))

    def test_backups_leave_searchers_for_unknown_actors(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.tracker = SimpleNamespace(targets={t: None for t in ('t0', 't2', 't4', 't5')})
        manager._visual_observations = {('u0', t):
            {'sample_s': 100.0, 'eligible': True}
            for t in manager.tracker.targets}
        manager._tracking = dict(zip(('t0', 't2', 't4', 't5'), ('u0', 'u1', 'u2', 'u3')))
        manager._backup = {}
        manager.status = {'u%d' % i: SimpleNamespace(connected=True, z=3.0)
                          for i in range(6)}
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(100.0)):
            self.assertFalse(manager._backup_search_capacity())
            manager.tracker.targets['t1'] = None
            manager._visual_observations[('u4', 't1')] = {
                'sample_s': 100.0, 'eligible': True}
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

    def test_agent_does_not_retire_either_red_slot_from_unproven_team_flag(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.targets = {'t4': (150.0, 6.5, 0.0, 0.0),
                         't5': (150.0, 2.5, 0.0, 0.0)}
        agent._t_seen = {'t4': 100.0, 't5': 100.0}
        agent._target_state = {}
        agent._orbit_target = 't5'
        agent._giveup_until = {}
        agent._target_cb(SimpleNamespace(target_id='t5', eliminated=True))
        self.assertEqual(set(agent.targets), {'t4', 't5'})
        self.assertEqual(agent._orbit_target, 't5')
        self.assertEqual(agent._giveup_until, {})

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

    def test_unproven_red_departure_cannot_retire_manager_task(self):
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
        self.assertEqual(released, [])
        self.assertFalse(target.eliminated)
        self.assertNotIn('t5', manager._eliminated)
        self.assertIn('t5', manager._truth_cache)

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
        manager.last_report = {uid: swarm_manager.rospy.Time(100.0)
                               for uid in manager.status}
        manager._tracking = {}
        manager._sanitize_dispatch_xy = lambda x, y, _tid: (x, y)
        manager._idle_uavs = lambda: []
        published = []
        manager.assign_pub = SimpleNamespace(publish=published.append)
        with patch.dict(swarm_manager.os.environ, {'DISPATCH_FAR_LIMIT': '60'}):
            with patch.object(swarm_manager.rospy.Time, 'now',
                              return_value=swarm_manager.rospy.Time.from_sec(100.0)):
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
        official_spawn = ((0.0, -3.0), (3.0, -3.0), (0.0, 0.0),
                          (3.0, 0.0), (0.0, 3.0), (3.0, 3.0))
        with patch.object(swarm_task, '_now', return_value=100.0):
            allocator = swarm_task.TaskAllocator(grid)
            self.assertIsNone(allocator.mission_start)
            for i in range(6):
                uid = 'typhoon_h480_%d' % i
                key = allocator.allocate({uid: official_spawn[i]})[uid]
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
                key = allocator.allocate({uid: (1.5, 0.0)})[uid]
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
        agent._scan = SimpleNamespace(range_min=.1, ranges=readings, angle_min=-math.pi,
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
        agent._scan = SimpleNamespace(range_min=.1, ranges=readings, angle_min=-math.pi,
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
        agent._scan = SimpleNamespace(range_min=.1, ranges=readings,
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
