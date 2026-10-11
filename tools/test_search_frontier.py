"""Allocator replays: reach the long-axis frontier without omitting cells.

These are geometric route distances, not flight or camera-coverage promises.
Use ROBOCUP_LOG_DIR in /tmp and source the ROS environment before running.
"""
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
import swarm_task


# Own-aircraft positions at each first auction in logs_20261011_122632.
# No actor position, world obstacle or simulator state enters this replay.
STARTS = (
    (-0.03534385561943054, -3.016512567177415),
    (2.938948415219784, -2.947980225086212),
    (0.03874625638127327, -0.08605334907770157),
    (3.0793956592679024, -0.16550487279891968),
    (-0.012041369453072548, 2.8271051347255707),
    (2.9539361596107483, 2.9711398612707853),
)


def replay(axis, zone, excluded=()):
    """Complete each available home-sector cell once through real allocate()."""
    grid = swarm_task.CoverageGrid(-50, 130, -50, 50, cell_m=7, num_uavs=6)
    with patch.object(swarm_task, 'SEARCH_ROUTE_AXIS', axis):
        allocator = swarm_task.TaskAllocator(grid)
    uid = 'typhoon_h480_%d' % zone
    available = set(grid.zone_cells[zone]) - set(excluded)
    for key, cell in grid.cells.items():
        if key not in available:
            cell.state = swarm_task.STATE_COVERED
    position, length, route, frontier = STARTS[zone], 0.0, [], {}
    # Other sectors are unavailable in this isolated route replay. The clock
    # is fixed so it measures ordering, not lease expiry or aircraft dynamics.
    with patch.object(swarm_task, '_now', return_value=1000.0):
        for _ in range(len(available)):
            key = allocator.allocate({uid: position})[uid]
            if key is None:
                raise AssertionError('First-pass route left an available cell')
            cell = grid.cell(key)
            new = (cell.cx, cell.cy)
            length += math.hypot(new[0]-position[0], new[1]-position[1])
            route.append(key)
            for boundary in (100, 120):
                if cell.cx >= boundary and boundary not in frontier:
                    frontier[boundary] = dict(cells=len(route), metres=length,
                                              travel_seconds_at_3mps=length/3.0)
            cell.state, cell.owner = swarm_task.STATE_COVERED, None
            allocator.complete_waypoint(uid, key, observed=False, now=1000.0)
            position = new
    return dict(route=route, available=available, metres=length,
                frontier=frontier, camera_observations=allocator.last_seen)


class SearchFrontierTests(unittest.TestCase):
    def test_eastern_frontier_arrives_before_full_short_axis_sweep(self):
        old_total = new_total = 0.
        for zone in (1, 3, 5):
            old, new = replay('column', zone), replay('long_axis', zone)
            old_total += old['metres']
            new_total += new['metres']
            with self.subTest(zone=zone):
                self.assertEqual(new['route'][0], old['route'][0])
                self.assertLessEqual(new['frontier'][100]['cells'], 9)
                self.assertLessEqual(new['frontier'][120]['cells'], 15)
                self.assertLess(new['frontier'][120]['metres'],
                                old['frontier'][120]['metres']*.50)
        # The final partial north row is narrower than 7 m. A horizontal
        # sweep adds 27 m in that sector while reaching x=120 154 m earlier;
        # total eastern coverage travel must still improve, without omissions.
        self.assertLessEqual(new_total, old_total + 1e-8)

    def test_all_six_sectors_preserve_complete_unique_first_pass(self):
        for zone in range(6):
            new = replay('long_axis', zone)
            with self.subTest(zone=zone):
                self.assertEqual(set(new['route']), new['available'])
                self.assertEqual(len(new['route']), len(new['available']))
                self.assertEqual(new['camera_observations'], {})
                if zone in (0, 1, 4, 5):
                    self.assertTrue(any(y == 0 or y == 14 for _, y in new['route']))

    def test_unavailable_frontier_cells_are_skipped_without_skipping_remainder(self):
        # A synthetic missing segment represents existing unavailable-cell
        # filtering, without assuming a map or granting route reachability.
        excluded = {(x, 12) for x in range(18, 24)}
        result = replay('long_axis', 5, excluded)
        self.assertEqual(set(result['route']), result['available'])
        self.assertFalse(set(result['route']) & excluded)
        self.assertEqual(len(result['route']), len(result['available']))
        self.assertIn(120, result['frontier'])

    def test_tall_sector_keeps_vertical_sweep(self):
        grid = swarm_task.CoverageGrid(0, 21, 0, 70, cell_m=7, num_uavs=1)
        with patch.object(swarm_task, 'SEARCH_ROUTE_AXIS', 'column'):
            old = swarm_task.TaskAllocator(grid).route_order
        with patch.object(swarm_task, 'SEARCH_ROUTE_AXIS', 'long_axis'):
            new = swarm_task.TaskAllocator(grid).route_order
        self.assertEqual(new, old)

    def test_startup_reports_selected_mode_and_actual_sweep_axis(self):
        grid = swarm_task.CoverageGrid(0, 70, 0, 21, cell_m=7, num_uavs=1)
        for mode, sweep in [('column', 'y'), ('long_axis', 'x')]:
            with patch.object(swarm_task, 'SEARCH_ROUTE_AXIS', mode), \
                    patch.object(swarm_task.rospy, 'loginfo') as log:
                allocator = swarm_task.TaskAllocator(grid)
            self.assertEqual(allocator.route_axis, mode)
            self.assertEqual(allocator.route_sweep_axes, {0: sweep})
            log.assert_called_once()
            message, *arguments = log.call_args[0]
            self.assertIn('version=long_axis_v1 axis=' + mode,
                          message % tuple(arguments))


if __name__ == '__main__':
    unittest.main()
