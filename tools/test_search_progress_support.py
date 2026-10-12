"""Search support uses only dated completion/processed-camera evidence."""
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
import swarm_task


class SearchProgressSupportTests(unittest.TestCase):
    def fixture(self):
        grid = swarm_task.CoverageGrid(-55, 155, -65, 65,
                                       cell_m=7, num_uavs=6)
        allocator = swarm_task.TaskAllocator(grid)
        with patch.object(swarm_task, '_now', return_value=100.):
            owner_key = allocator.allocate({'typhoon_h480_5': (3., 3.)})['typhoon_h480_5']
        self.assertEqual(grid.get_zone_id(owner_key), 5)
        for zone in (0, 1, 2, 3):
            key = grid.zone_cells[zone][0]
            grid.cell(key).state = swarm_task.STATE_ASSIGNED
            grid.cell(key).owner = 'typhoon_h480_%d' % zone
        home_keys = [k for k in grid.zone_cells[4]
                     if grid.cell(k).state == swarm_task.STATE_FREE][:8]
        target_keys = [k for k in grid.zone_cells[5]
                       if grid.cell(k).state == swarm_task.STATE_FREE][:5]
        for i, key in enumerate(home_keys[:6]):
            grid.cell(key).state = swarm_task.STATE_COVERED
            allocator.complete_waypoint('typhoon_h480_4', key,
                                        observed=True, now=165.+2*i)
        for i, key in enumerate(target_keys[:2]):
            grid.cell(key).state = swarm_task.STATE_COVERED
            allocator.observed_cell(key, now=167.+5*i)
        return grid, allocator, owner_key, home_keys, target_keys

    def test_attended_but_slow_sector_receives_one_idle_helper(self):
        grid, a, owner, _, _ = self.fixture()
        with patch.object(swarm_task, '_now', return_value=190.), \
                patch.object(swarm_task.rospy, 'loginfo') as log:
            allocated = a.allocate({'typhoon_h480_4': (39.5, 29.5)})
        key = allocated['typhoon_h480_4']
        self.assertEqual(grid.get_zone_id(key), 5)
        self.assertEqual(grid.cell(owner).owner, 'typhoon_h480_5')
        self.assertNotEqual(key, owner)
        self.assertGreaterEqual(math.hypot(grid.cell(key).cx-grid.cell(owner).cx,
            grid.cell(key).cy-grid.cell(owner).cy), 14.)
        self.assertEqual(a._search_support_active, ('typhoon_h480_4', 5, key))
        self.assertEqual(a._search_support_next_s, {5:280.})
        self.assertTrue(any('[SEARCH_PROGRESS_SUPPORT]' in c.args[0]
                            for c in log.call_args_list))

    def test_repeated_same_cell_images_or_scans_do_not_hide_slow_progress(self):
        grid, a, _, _, keys = self.fixture()
        first = dict(a._search_progress_first)
        for stamp in (175., 180., 189.):
            for key in keys[:2]:
                a.observed_cell(key, stamp)
                a.complete_waypoint('typhoon_h480_5', key, now=stamp)
        self.assertEqual(a._search_progress_first, first)
        with patch.object(swarm_task, '_now', return_value=190.):
            key = a.allocate({'typhoon_h480_4': (39.5, 29.5)})['typhoon_h480_4']
        self.assertEqual(grid.get_zone_id(key), 5)

    def test_real_new_cells_restore_normal_sector_search(self):
        grid, a, _, _, keys = self.fixture()
        for key in keys[2:4]:
            grid.cell(key).state = swarm_task.STATE_COVERED
            a.observed_cell(key, 185.)
        with patch.object(swarm_task, '_now', return_value=190.):
            key = a.allocate({'typhoon_h480_4': (39.5, 29.5)})['typhoon_h480_4']
        self.assertEqual(grid.get_zone_id(key), 4)
        self.assertIsNone(a._search_support_active)

    def test_long_initial_transit_has_distance_based_grace(self):
        grid = swarm_task.CoverageGrid(-55,155,-65,65,cell_m=7,num_uavs=6)
        a = swarm_task.TaskAllocator(grid)
        # Enter an eastern home sector from the western spawn side. Global
        # 45s grace expires well before this legitimate long transit does.
        with patch.object(swarm_task, '_now', return_value=100.):
            a.allocate({'typhoon_h480_5': (-50.,30.)})
        self.assertGreater(a._search_entry_ready[5][1], 180.)
        for i,key in enumerate(grid.zone_cells[4][:6]):
            grid.cell(key).state = swarm_task.STATE_COVERED
            a.observed_cell(key,155.+i)
        with patch.object(swarm_task, '_now', return_value=160.):
            helper = a.allocate({'typhoon_h480_4':(39.5,29.5)})['typhoon_h480_4']
        self.assertEqual(grid.get_zone_id(helper),4)
        self.assertIsNone(a._search_support_active)

    def test_no_new_evidence_and_assignments_never_advance_progress_clock(self):
        _, a, _, _, _ = self.fixture()
        a._search_progress_first.clear()
        a._search_image_first.clear()
        with patch.object(swarm_task, '_now', return_value=190.):
            a.allocate({'typhoon_h480_4': (39.5, 29.5)})
        self.assertEqual(a._search_progress_first, {})
        self.assertIsNone(a._search_support_active)

    def test_busy_owner_is_never_retasked_and_existing_helper_blocks_more(self):
        grid, a, _, home_keys, _ = self.fixture()
        # An unrelated tracker is not in manager's idle-uavs input at all.
        before = [(k,c.owner,c.state) for k,c in grid.cells.items()
                  if c.owner == 'typhoon_h480_3']
        with patch.object(swarm_task, '_now', return_value=190.):
            result = a.allocate({'typhoon_h480_4': (39.5, 29.5)})
        self.assertEqual(set(result), {'typhoon_h480_4'})
        self.assertEqual(before, [(k,c.owner,c.state) for k,c in grid.cells.items()
                                  if c.owner == 'typhoon_h480_3'])
        # Even after the zone cooldown, the same active job is enough.
        a.mission_start = 200.
        a._search_progress_first = {k:290. for k in home_keys[:6]}
        self.assertEqual(a._slow_zone_support({'typhoon_h480_2': (75., 15.)},
            set(grid.uncovered_cells()), 300., {}), {})

    def test_only_one_helper_in_a_single_batch(self):
        grid, a, _, _, _ = self.fixture()
        other = [k for k in grid.zone_cells[3]
                 if grid.cell(k).state == swarm_task.STATE_FREE][:6]
        for key in other:
            grid.cell(key).state = swarm_task.STATE_COVERED
            a.observed_cell(key, 170.)
        for c in grid.cells.values():
            if c.owner == 'typhoon_h480_3':
                c.owner = None; c.state = swarm_task.STATE_FREE
        with patch.object(swarm_task, '_now', return_value=190.):
            result = a.allocate({'typhoon_h480_4': (39.5, 29.5),
                                 'typhoon_h480_3': (60.5, 15.5)})
        self.assertEqual(sum(grid.get_zone_id(k)==5 for k in result.values()), 1)

    def test_cooldown_does_not_recruit_after_helper_finishes(self):
        grid, a, _, _, _ = self.fixture()
        with patch.object(swarm_task, '_now', return_value=190.):
            key = a.allocate({'typhoon_h480_4': (39.5, 29.5)})['typhoon_h480_4']
        grid.cell(key).owner = None
        grid.cell(key).state = swarm_task.STATE_COVERED
        a.complete_waypoint('typhoon_h480_4', key, now=200.)
        self.assertEqual(a._slow_zone_support({'typhoon_h480_4': (39.5,29.5)},
            set(grid.uncovered_cells()), 210., {}), {})

    def test_blocked_failed_distant_and_overlap_cells_are_not_support_targets(self):
        grid, a, owner, _, _ = self.fixture()
        blocked = set(grid.zone_cells[5])
        with patch.object(swarm_task, '_now', return_value=190.):
            key = a.allocate({'typhoon_h480_4': (39.5,29.5)},
                             blocked=blocked)['typhoon_h480_4']
        self.assertEqual(grid.get_zone_id(key), 4)
        self.assertIsNone(a._search_support_active)
        self.assertEqual(a._slow_zone_support({'typhoon_h480_4':(-50.,30.)},
            set(grid.uncovered_cells()), 190., {}), {})
        for k in grid.zone_cells[5]:a.failed_visit[k]=185.
        with patch.object(swarm_task, '_now', return_value=190.):
            result=a.allocate({'typhoon_h480_4':(39.5,29.5)})
        self.assertNotEqual(grid.get_zone_id(result['typhoon_h480_4']),5)

    def test_fresh_lost_revisit_has_priority_over_slow_sector_support(self):
        grid, a, _, _, _ = self.fixture()
        key = a.queue_revisit('lost', 32.5, 36.5, now=188., ttl_s=20.)
        with patch.object(swarm_task, '_now', return_value=190.):
            result = a.allocate({'typhoon_h480_4':(39.5,29.5)})
        self.assertEqual(result['typhoon_h480_4'], key)
        self.assertIsNone(a._search_support_active)
        self.assertEqual(a._search_support_next_s, {})

    def test_completed_support_cell_does_not_pin_later_ordinary_sector_job(self):
        grid,a,_,_,_=self.fixture()
        with patch.object(swarm_task,'_now',return_value=190.):
            first=a.allocate({'typhoon_h480_4':(39.5,29.5)})['typhoon_h480_4']
        grid.cell(first).state=swarm_task.STATE_COVERED
        grid.cell(first).owner=None
        a.complete_waypoint('typhoon_h480_4',first,now=200.)
        # This is a different ordinary waypoint in the same region. It must
        # not extend the old support job, though it still prevents crowding.
        next_key=next(k for k in grid.zone_cells[5]
                      if grid.cell(k).state==swarm_task.STATE_FREE)
        grid.cell(next_key).state=swarm_task.STATE_ASSIGNED
        grid.cell(next_key).owner='typhoon_h480_4'
        a._slow_zone_support({'typhoon_h480_2':(70.,15.)},
                             set(grid.uncovered_cells()),210.,{})
        self.assertIsNone(a._search_support_active)
        self.assertEqual(a._search_support_next_s,{5:280.})

    def test_route_completion_is_progress_but_never_camera_coverage(self):
        grid,a,_,home,_=self.fixture()
        key=home[6]
        a.complete_waypoint('typhoon_h480_4',key,observed=False,now=180.)
        self.assertEqual(a._search_route_first[key],180.)
        self.assertNotIn(key,a._search_image_first)
        self.assertNotIn(key,a.last_seen)
        a.observed_cell(key,185.)
        self.assertEqual(a._search_progress_first[key],180.)
        self.assertEqual(a._search_image_first[key],185.)

    def test_sim_clock_reset_or_wall_clock_change_cannot_trigger_support(self):
        grid, a, _, _, _ = self.fixture()
        with patch.object(swarm_task.time,'time',side_effect=AssertionError('wall clock')):
            self.assertEqual(a._slow_zone_support({'typhoon_h480_4':(39.5,29.5)},
                set(grid.uncovered_cells()),90.,{}),{})
            self.assertEqual(a._slow_zone_support({'typhoon_h480_4':(39.5,29.5)},
                set(grid.uncovered_cells()),float('nan'),{}),{})


if __name__ == '__main__':unittest.main()
