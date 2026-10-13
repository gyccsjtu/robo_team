"""Ordinary six-region search cannot be silently replaced by fixed patrols."""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_navigation/src'))
import swarm_task as tasklib
import swarm_manager as sm
from search_segments import SegmentBook
from search_segment_manager import SearchSegmentManager
import test_southeast_patrol as manager_fixture


class NormalPartitionAllocatorTests(unittest.TestCase):
    def fixture(self):
        with patch.dict(os.environ, {'NORMAL_PARTITION_SEARCH': '1',
                                     'SEARCH_DECONFLICTED_ALLOCATION': '1'}):
            grid = tasklib.CoverageGrid(-50., 130., -60., 60., 7., 6)
            allocator = tasklib.TaskAllocator(grid)
        allocator.mission_start = 1.
        return grid, allocator

    def allocate(self, allocator, positions, now=100., **kwargs):
        with patch.object(tasklib, '_now', return_value=now):
            return allocator.allocate(positions, **kwargs)

    def test_six_home_image_debts_survive_poses_and_thirty_second_handoff(self):
        # Aircraft can return from a pursuit anywhere. An eastern pose is not
        # permission to abandon six still-unimaged home regions after 30 s.
        for now in (20., 40., 160., 400.):
            for xy in ((126., 54.), (-48., -58.), (149., 64.)):
                with self.subTest(now=now, xy=xy):
                    grid, allocator = self.fixture()
                    positions = {f'typhoon_h480_{i}': xy for i in range(6)}
                    assigned = self.allocate(allocator, positions, now)
                    self.assertEqual(len(set(assigned.values())), 6)
                    for uid, key in assigned.items():
                        self.assertIsNotNone(key)
                        self.assertEqual(grid.get_zone_id(key), grid.get_uav_zone(uid))
                    self.assertEqual(allocator._search_image_first, {})

    def test_scanning_all_home_waypoints_without_images_does_not_discharge_debt(self):
        grid, allocator = self.fixture()
        uid = 'typhoon_h480_0'
        home = grid.zone_cells[grid.get_uav_zone(uid)]
        for key in home:
            allocator.complete_waypoint(uid, key, observed=False, now=80.)
        # These remain available but have nonzero route visits, the historical
        # condition that erroneously ended the hard home preference.
        chosen = self.allocate(allocator, {uid: (126., 54.)}, now=100.)[uid]
        self.assertEqual(grid.get_zone_id(chosen), grid.get_uav_zone(uid))
        self.assertTrue(all(key in allocator.visit_time for key in home))
        self.assertFalse(set(home) & set(allocator._search_image_first))

    def orphan_fixture(self):
        grid, allocator = self.fixture()
        donor, owner = 'typhoon_h480_0', 'typhoon_h480_1'
        home_zone, occupied_zone = grid.get_uav_zone(donor), grid.get_uav_zone(owner)
        home = grid.zone_cells[home_zone]
        east = grid.zone_cells[occupied_zone]
        one_home, occupied_key = home[0], east[0]
        east_debt = east[1:13]
        keep = set(east_debt) | {one_home}
        for key, cell in grid.cells.items():
            if key not in keep:
                cell.state = tasklib.STATE_COVERED
        position = (grid.cell(east_debt[0]).cx, grid.cell(east_debt[0]).cy)
        manager = SearchSegmentManager()
        manager._segments = SegmentBook()
        manager.status = {}
        cell = grid.cell(occupied_key)
        task = manager._segments.reserve(owner, [occupied_key], [(cell.cx, cell.cy)],
                                         99., 20., 90.)
        return grid, allocator, manager, task, donor, occupied_zone, position

    def test_covered_still_executing_task_cells_prevent_false_orphan_takeover(self):
        grid, allocator, manager, task, donor, zone, position = self.orphan_fixture()
        self.assertEqual(grid.cell(task.cells[0]).state, tasklib.STATE_COVERED)
        remaining = set(grid.uncovered_cells())
        poses = {donor: position}
        old_inference = allocator._orphaned_zone_takeovers(poses, remaining, 100.)
        self.assertEqual(old_inference, {donor: zone})
        layout = manager._active_search_layout(100.)
        self.assertEqual(layout[task.uid]['cells'], task.cells)
        self.assertEqual(allocator._orphaned_zone_takeovers(
            poses, remaining, 100., active_search=layout), {})
        chosen = self.allocate(allocator, poses, active_search=layout)[donor]
        self.assertEqual(grid.get_zone_id(chosen), grid.get_uav_zone(donor))

    def test_completed_task_snapshot_does_not_hold_region_after_work_ends(self):
        grid, allocator, manager, task, donor, zone, position = self.orphan_fixture()
        manager._segments.advance(task.uid, task.task_id, task.revision, 1)
        layout = manager._active_search_layout(100.)
        self.assertEqual(layout, {})
        takeover = allocator._orphaned_zone_takeovers(
            {donor: position}, set(grid.uncovered_cells()), 100., active_search=layout)
        self.assertEqual(takeover, {donor: zone})

    def test_expired_task_snapshot_does_not_hold_region_after_lease_ends(self):
        grid, allocator, manager, task, donor, zone, position = self.orphan_fixture()
        self.assertEqual(manager._active_search_layout(119.), {})
        takeover = allocator._orphaned_zone_takeovers(
            {donor: position}, set(grid.uncovered_cells()), 119.,
            active_search=manager._active_search_layout(119.))
        self.assertEqual(takeover, {donor: zone})

    def test_fresh_lost_person_revisit_preempts_home_and_existing_search_layout(self):
        grid, allocator = self.fixture()
        uid = 'typhoon_h480_0'
        east = grid.zone_cells[grid.get_uav_zone('typhoon_h480_1')][10]
        cell = grid.cell(east)
        cell.state = tasklib.STATE_COVERED
        allocator.observed_cell(east, now=80.)
        with patch.object(tasklib, '_now', return_value=100.):
            hint = allocator.queue_revisit('lost', cell.cx, cell.cy, now=100.)
        selected = self.allocate(allocator, {uid: (cell.cx, cell.cy)},
            active_search={'typhoon_h480_1': {
                'cells': [east], 'points': [(cell.cx, cell.cy)],
                'look_points': [(cell.cx, cell.cy)]}})[uid]
        self.assertEqual(selected, hint)
        self.assertNotEqual(grid.get_zone_id(selected), grid.get_uav_zone(uid))
        self.assertTrue(any(key not in allocator._search_image_first
                            for key in grid.zone_cells[grid.get_uav_zone(uid)]))

    def test_no_available_home_work_allows_cross_region_without_deadlock(self):
        for blocked_home in (False, True):
            with self.subTest(blocked_home=blocked_home):
                grid, allocator = self.fixture()
                uid = 'typhoon_h480_0'
                home = set(grid.zone_cells[grid.get_uav_zone(uid)])
                if not blocked_home:
                    for key in home:
                        grid.cell(key).state = tasklib.STATE_COVERED
                        allocator.observed_cell(key, now=80.)
                blocked = home if blocked_home else set()
                chosen = self.allocate(allocator, {uid: (-40., -40.)},
                                       blocked=blocked)[uid]
                self.assertIsNotNone(chosen)
                self.assertNotIn(chosen, home)

    def test_fixed_priority_bonus_is_disabled_even_if_inherited_environment_is_on(self):
        grid, allocator = self.fixture()
        key = grid.world_to_cell(-35., -22.)
        with patch.object(tasklib, '_PRIORITY_ON', True):
            self.assertEqual(allocator._priority_bonus(key, now=100.,
                            uav_id='typhoon_h480_0'), 0.)


class NormalPartitionManagerTests(unittest.TestCase):
    use_default_flight_bounds = True

    def setUp(self):
        manager_fixture.SoutheastPatrolTests.setUp(self)
        self.m.allocator.normal_partition_search = True

    def test_allocate_passes_no_west_corner_preemption_while_building_views_remain_on(self):
        with patch.object(sm, 'BUILDING_SUBPOINT_REVIEW', True), \
             patch.object(self.m, '_allocate_segments', return_value='ordinary') as call:
            self.assertEqual(self.m._allocate(), 'ordinary')
            self.assertTrue(sm.BUILDING_SUBPOINT_REVIEW)
        call.assert_called_once()
        self.assertIs(call.call_args.kwargs['west_corner_review'], False)
        self.assertEqual(self.m._regional_search_patrols(), ())

    def test_existing_building_view_options_are_not_disabled_by_normal_mode(self):
        key = next(iter(self.m.grid.cells))
        options = (((-40., -40.), (-35., -35.)),)
        self.m._building_viewpoints = {key: options}
        self.assertEqual(self.m._subpoint_views(key), options)
        self.assertTrue(self.m.allocator.normal_partition_search)


class NormalPartitionWrapperTests(unittest.TestCase):
    def test_real_wrapper_chain_clears_inherited_fixed_patrols_and_preserves_models(self):
        names = ('NORMAL_PARTITION_SEARCH', 'SOUTH_PATROL', 'SOUTH_PATROL_SOUTHEAST',
                 'SOUTH_PATROL_NORTHEAST', 'EAST_PATROL', 'EAST_SPLIT_PATROL',
                 'EDGE_SEARCH_REVIEW', 'PRIORITY_CORNER', 'HOT_TARGET_ENABLE',
                 'BUILDING_SUBPOINT_REVIEW', 'SEARCH_SEGMENTS', 'MOTION_SAFETY',
                 'EARLY_VIEW_PURSUIT', 'LOCALIZED_SEARCH_FOLLOW',
                 'IMMEDIATE_VISIBLE_DISPATCH', 'GIMBAL_AUX_MODE', 'PR_WEIGHTS',
                 'PR_DISTRACTOR_WEIGHTS')
        with tempfile.TemporaryDirectory(prefix='robocup_normal_wrapper_') as tmp:
            directory = Path(tmp)
            for name in ('run_stable_6.sh', 'run_sculpture.sh'):
                shutil.copy2(ROOT / name, directory / name)
            (directory / 'weights').mkdir()
            for name in ('best_yolo11n_bino_v3_clean.pt',
                         'best_yolo11n_bino_sculpture_v3.pt'):
                (directory / 'weights' / name).symlink_to(ROOT / 'weights' / name)
            # Only these public configuration values are printed. This stub
            # is the terminal wrapper target and starts no node or simulator.
            stub = directory / 'run_match.sh'
            stub.write_text('#!/usr/bin/env bash\nexec python3 - <<\'PY\'\n'
                'import json,os\n'
                'print("TEST_WRAPPER_ENV "+json.dumps({k:os.environ.get(k) '
                'for k in '+repr(names)+'}))\nPY\n')
            stub.chmod(0o755)
            env = dict(os.environ)
            env.update({name: '1' for name in names[:9]})
            result = subprocess.run(['bash', str(directory / 'run_stable_6.sh'),
                                     'preflight'], env=env, text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    timeout=15.)
            self.assertEqual(result.returncode, 0, result.stderr)
            line = next(line for line in result.stdout.splitlines()
                        if line.startswith('TEST_WRAPPER_ENV '))
            actual = json.loads(line.split(' ', 1)[1])
            self.assertEqual(actual['NORMAL_PARTITION_SEARCH'], '1')
            for name in names[1:9]:
                self.assertEqual(actual[name], '0', name)
            for name in ('BUILDING_SUBPOINT_REVIEW', 'SEARCH_SEGMENTS',
                         'MOTION_SAFETY', 'EARLY_VIEW_PURSUIT',
                         'LOCALIZED_SEARCH_FOLLOW', 'IMMEDIATE_VISIBLE_DISPATCH'):
                self.assertEqual(actual[name], '1', name)
            self.assertEqual(actual['GIMBAL_AUX_MODE'], 'off')
            self.assertEqual(Path(actual['PR_WEIGHTS']).resolve(),
                             ROOT / 'weights/best_yolo11n_bino_v3_clean.pt')
            self.assertEqual(Path(actual['PR_DISTRACTOR_WEIGHTS']).resolve(),
                             ROOT / 'weights/best_yolo11n_bino_sculpture_v3.pt')


if __name__ == '__main__':
    unittest.main()
