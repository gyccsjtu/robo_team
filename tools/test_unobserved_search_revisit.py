"""Failed camera visits remain search debt, independently of route progress."""
import unittest
from unittest.mock import patch

import test_search_progress_support as support_fixture
import swarm_task as st


class UnobservedSearchRevisitTests(unittest.TestCase):
    def test_four_unimaged_waypoints_do_not_cancel_sector_support(self):
        grid, allocator, _, _, _ = support_fixture.SearchProgressSupportTests().fixture()
        keys = [key for key in grid.zone_cells[5]
                if grid.cell(key).state == st.STATE_FREE][:4]
        for key in keys:
            allocator.complete_waypoint('typhoon_h480_5', key,
                                        observed=False, now=185.)
            grid.cell(key).state = st.STATE_REVIEW
        with patch.object(st, '_now', return_value=190.):
            selected = allocator.allocate({'typhoon_h480_4': (39.5, 29.5)})
        self.assertEqual(grid.get_zone_id(selected['typhoon_h480_4']), 5)
        self.assertIsNotNone(allocator._search_support_active)
        self.assertTrue(all(key not in allocator._search_image_first for key in keys))

    def test_route_only_donor_does_not_abandon_its_unobserved_home(self):
        grid, allocator, _, home, _ = support_fixture.SearchProgressSupportTests().fixture()
        for key in home[:6]:
            allocator._search_image_first.pop(key, None)
            allocator.last_seen.pop(key, None)
            grid.cell(key).state = st.STATE_REVIEW
        with patch.object(st, '_now', return_value=190.):
            selected = allocator.allocate({'typhoon_h480_4': (39.5, 29.5)})
        self.assertEqual(grid.get_zone_id(selected['typhoon_h480_4']), 4)
        self.assertIsNone(allocator._search_support_active)

    def test_visited_but_unimaged_review_cells_are_valid_support_debt(self):
        grid, allocator, _, _, _ = support_fixture.SearchProgressSupportTests().fixture()
        for key in grid.zone_cells[5]:
            if grid.cell(key).state == st.STATE_FREE:
                grid.cell(key).state = st.STATE_REVIEW
                allocator.visit_time[key] = 185.
        result = allocator._slow_zone_support({'typhoon_h480_4': (39.5, 29.5)},
            set(grid.uncovered_cells()), 190., {})
        self.assertIn('typhoon_h480_4', result)
        self.assertTrue(result['typhoon_h480_4']['keys'])
        self.assertTrue(all(grid.cell(key).state == st.STATE_REVIEW
                            for key in result['typhoon_h480_4']['keys']))

    def blind_fixture(self):
        grid = st.CoverageGrid(0, 35, 0, 21, cell_m=7, num_uavs=1)
        allocator = st.TaskAllocator(grid)
        blind = grid.world_to_cell(17.5, 3.5)
        grid.cell(blind).state = st.STATE_REVIEW
        allocator.complete_waypoint('uav_0', blind, observed=False, now=100.)
        self.assertEqual(allocator.queue_revisit('blind', 17.5, 3.5,
            now=100., delay_s=8., ttl_s=90.), blind)
        return grid, allocator, blind

    def test_due_nearby_blind_revisit_is_not_hidden_by_new_free_cells(self):
        grid, allocator, blind = self.blind_fixture()
        with patch.object(st, '_now', return_value=108.1):
            selected = allocator.allocate({'uav_0': (3.5, 3.5)})
        self.assertEqual(selected['uav_0'], blind)
        self.assertEqual(grid.cell(blind).state, st.STATE_ASSIGNED)
        self.assertNotIn(blind, allocator.last_seen)
        self.assertNotIn(blind, allocator._search_image_first)
        with patch.object(st, '_now', return_value=108.2):
            other = allocator.allocate({'uav_1': (3.5, 3.5)})['uav_1']
        self.assertNotEqual(other, blind)

    def test_blind_delay_expiry_obstacle_and_failed_view_cooldown_remain_effective(self):
        cases = [('delay', 107.9, False, False),
                 ('expired', 190., False, False),
                 ('blocked', 108.1, True, False),
                 ('failed', 108.1, False, True)]
        for label, now, blocked, failed in cases:
            with self.subTest(case=label):
                grid, allocator, blind = self.blind_fixture()
                if failed:
                    allocator.failed_visit[blind] = 108.
                with patch.object(st, '_now', return_value=now):
                    selected = allocator.allocate({'uav_0': (3.5, 3.5)},
                        blocked={blind} if blocked else ())['uav_0']
                self.assertNotEqual(selected, blind)
                self.assertNotIn(blind, allocator.last_seen)

    def test_recently_served_blind_view_cannot_restart_without_cooldown(self):
        grid, allocator, blind = self.blind_fixture()
        with patch.object(st, '_now', return_value=108.1):
            allocator.allocate({'uav_0': (3.5, 3.5)})
        allocator.complete_waypoint('uav_0', blind, observed=False, now=110.)
        grid.cell(blind).state = st.STATE_REVIEW
        grid.cell(blind).owner = None
        self.assertIsNone(allocator.queue_revisit('blind', 17.5, 3.5, now=111.))
        self.assertEqual(allocator.queue_revisit('blind', 17.5, 3.5, now=171.), blind)


if __name__ == '__main__':
    unittest.main()
