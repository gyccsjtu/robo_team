"""Two opt-in east lanes share reservations, lawful viewpoints and preemption."""
import math
import threading
import unittest
from unittest.mock import patch

import test_southeast_patrol as patrol_fixture
import test_joint_dispatch as dispatch_fixture
import boundary_patrol as bp
import swarm_manager as sm
import swarm_task as st


class EastSplitPatrolTests(unittest.TestCase):
    def setUp(self):
        patrol_fixture.SoutheastPatrolTests.setUp(self)
        self.m._regional_patrols = tuple(bp.SouthPatrol(
            (-50, 150, -60, 60), southeast=False, northeast=False,
            east=True, east_half=half) for half in ('north', 'south'))
        self.m._south_patrol = self.m._regional_patrols[0]

    def refresh(self, now):
        self.now = now
        for uid, status in self.m.status.items():
            status.header.stamp = sm.rospy.Time.from_sec(now)
            self.m.last_report[uid] = sm.rospy.Time.from_sec(now)

    def allocate(self):
        self.m._allocate()
        return [task for task in self.m._segments.tasks.values()
                if task.purpose == 'south_patrol']

    def test_both_lanes_are_reserved_before_ordinary_routes_under_one_lock(self):
        original = self.m.allocator.allocate
        def ordinary(positions, **kwargs):
            regional = [t for t in self.m._segments.tasks.values()
                        if t.purpose == 'south_patrol']
            self.assertEqual(len(regional), 2)
            self.assertTrue(self.m._task_lock._is_owned())
            self.assertTrue(set(t.uid for t in regional).isdisjoint(positions))
            self.assertTrue(set(self.m._segments.reserved) <= kwargs['blocked'])
            return original(positions, **kwargs)
        with patch.object(self.m.allocator, 'allocate', side_effect=ordinary):
            tasks = self.allocate()
        self.assertEqual(len(tasks), 2)
        self.assertEqual(len(self.m._segments.tasks), 6)
        self.assertEqual(len({t.uid for t in tasks}), 2)
        cells = [key for task in self.m._segments.tasks.values() for key in task.cells]
        self.assertEqual(len(cells), len(set(cells)))
        for patrol in self.m._regional_patrols:
            task = self.m._segments.tasks[patrol.uid]
            for key, point, look in zip(task.cells, task.points, task.look_points):
                self.assertGreaterEqual(look[0], 105.)
                self.assertEqual(look[1] >= 0., patrol.east_half == 'north')
                self.assertAlmostEqual(math.dist(point, look), 8.95)
                self.assertTrue(self.m._search_point_free(point))
                self.assertTrue(self.m._blind_viewpoint_valid(self.m.grid.cell(key), point))
                self.assertNotEqual(self.m.grid.cell(key).state, st.STATE_COVERED)

    def test_concurrent_scheduler_calls_cannot_duplicate_lanes_or_cells(self):
        barrier = threading.Barrier(2)
        errors = []
        def tick():
            try:
                barrier.wait()
                self.m._allocate()
            except Exception as exc:
                errors.append(exc)
        workers = [threading.Thread(target=tick) for _ in range(2)]
        for worker in workers: worker.start()
        for worker in workers: worker.join(5.)
        self.assertFalse(errors)
        self.assertTrue(all(not worker.is_alive() for worker in workers))
        tasks = [t for t in self.m._segments.tasks.values() if t.purpose == 'south_patrol']
        self.assertEqual(len(tasks), 2)
        self.assertEqual(len(self.m._segments.tasks), 6)
        cells = [key for t in self.m._segments.tasks.values() for key in t.cells]
        self.assertEqual(len(cells), len(set(cells)))

    def test_outer_northeast_and_southeast_cells_have_safe_inward_viewpoints(self):
        north, south = (28, 17), (28, 0)
        self.m._blocked_cells = set(self.m.grid.cells)-{north, south}
        cell = self.m.grid.cell(north)
        self.assertEqual((cell.cx, cell.cy), (148., 59.5))
        # Test a corner leg from an eastern entry. The unchanged bounded
        # A* work budget may reject a direct 150 m diagonal from spawn;
        # neither this experiment nor its test bypasses that reachability gate.
        for uid, y in zip(self.m.uav_ids[:2], (45., -45.)):
            self.m.status[uid].x, self.m.status[uid].y = 110., y
        original = sm.flight_waypoint
        for margin in (1.5, 2.0):
            with self.subTest(margin=margin):
                legal = lambda x, y: original(x, y, inset=margin)
                self.m._segment_flight_waypoint = legal
                with patch.object(sm, 'flight_waypoint', side_effect=legal):
                    for patrol, key in zip(self.m._regional_patrols, (north, south)):
                        proposal = patrol.choose(self.m, self.m.uav_ids, self.now)
                        self.assertIsNotNone(proposal)
                        self.assertEqual(proposal[1], [key])
                        point = proposal[2][0]
                        look = self.m.grid.cell(key)
                        self.assertEqual(legal(*point), point)
                        self.assertAlmostEqual(math.dist(point, (look.cx, look.cy)), 8.95)
                        self.assertGreater(point[0], 133.)  # old band's omitted outer strip
                        self.assertTrue(self.m._blind_viewpoint_valid(look, point))
                        self.assertTrue(self.m._in_processed_view(
                            *point, math.atan2(look.cy-point[1], look.cx-point[0]),
                            look.cx, look.cy, 4.5, 0.))
                        self.assertEqual((cell.cx, cell.cy), (148., 59.5))

    def test_empty_map_corner_deployment_can_plan_from_the_initial_aircraft_positions(self):
        # The original float priorities exhausted 2500 nodes even without
        # buildings; canonical move counts must support both initial legs.
        corners={(28,17),(28,0)}
        self.m._blocked_cells=set(self.m.grid.cells)-corners
        tasks=self.allocate()
        self.assertEqual(len(tasks),2)
        self.assertEqual({key for task in tasks for key in task.cells},corners)
        self.assertEqual(len({task.uid for task in tasks}),2)
        for task in tasks:
            self.assertEqual(len(task.points),1)
            self.assertTrue(self.m._search_point_free(task.points[0]))

    def test_person_preemption_replaces_only_its_lane_without_false_coverage(self):
        self.allocate()
        north, south = self.m._regional_patrols
        first = self.m._segments.tasks[north.uid]
        southern = self.m._segments.tasks[south.uid]
        self.m._tracking['real_person'] = first.uid
        self.m._free_search_lease(first.uid)
        self.assertIsNone(north.uid)
        self.assertEqual((south.uid, south.task_id), (southern.uid, southern.task_id))
        self.assertTrue(set(first.cells).isdisjoint(self.m._segments.reserved))
        donor = next(uid for uid in self.m.uav_ids
                     if uid not in (first.uid, southern.uid))
        self.m._drop_segment(donor, 'test_idle')
        self.refresh(103.)
        self.assertEqual(len(self.allocate()), 2)
        self.assertEqual(north.uid, donor)
        self.assertEqual(south.task_id, southern.task_id)
        self.assertNotIn(first.uid, self.m._segments.tasks)
        for key in first.cells:
            self.assertNotIn(key, self.m.allocator.failed_visit)
            self.assertNotEqual(self.m.grid.cell(key).state, st.STATE_COVERED)

    def test_extended_grid_metadata_cannot_add_outside_flight_region_cells(self):
        self.m.grid = st.CoverageGrid(-50, 157, -60, 67, 7, 6)
        self.m.allocator = st.TaskAllocator(self.m.grid)
        self.m.lease = st.LeaseManager(self.m.grid)
        legal, outside_x, outside_y = (28, 16), (29, 16), (28, 17)
        self.m._blocked_cells = set(self.m.grid.cells)-{legal, outside_x, outside_y}
        uid = self.m.uav_ids[0]
        self.m.status[uid].x, self.m.status[uid].y = 145., 58.
        north = self.m._regional_patrols[0]
        proposal = north.choose(self.m, [uid], self.now)
        self.assertIsNotNone(proposal)
        self.assertEqual(proposal[1], [legal])
        self.assertGreater(self.m.grid.cell(outside_x).cx, north.bounds[1])
        self.assertGreater(self.m.grid.cell(outside_y).cy, north.bounds[3])

    def test_failed_lane_keeps_owner_during_cooldown_then_uses_new_task(self):
        self.allocate()
        north, south = self.m._regional_patrols
        old = self.m._segments.tasks[north.uid]
        south_id = south.task_id
        self.m._drop_segment(old.uid, 'blocked_viewpoint', failed=True)
        self.m._release_cooldown[old.uid] = sm.rospy.Time(102.)
        self.refresh(101.)
        self.allocate()
        self.assertEqual(north.uid, old.uid)
        self.assertNotIn(old.uid, self.m._segments.tasks)
        self.assertEqual(south.task_id, south_id)
        self.refresh(102.1)
        self.allocate()
        new = self.m._segments.tasks[old.uid]
        self.assertNotEqual(new.task_id, old.task_id)
        self.assertTrue(set(new.cells).isdisjoint(old.cells))
        self.assertEqual(south.task_id, south_id)

    def test_unhealthy_owner_releases_only_its_lane_and_allows_replacement(self):
        self.allocate()
        north, south = self.m._regional_patrols
        old = self.m._segments.tasks[north.uid]
        south_id = south.task_id
        self.m.status[old.uid].connected = False
        donor = next(uid for uid in self.m.uav_ids if uid not in (old.uid, south.uid))
        self.m._drop_segment(donor, 'test_idle')
        self.refresh(103.)
        self.allocate()
        self.assertEqual(north.uid, donor)
        self.assertEqual(south.task_id, south_id)
        self.assertNotIn(old.uid, self.m._segments.tasks)

    def test_unreachable_lane_releases_idle_owner_after_bounded_retry(self):
        self.allocate()
        north, south = self.m._regional_patrols
        old = self.m._segments.tasks[north.uid]
        south_id = south.task_id
        self.m._drop_segment(old.uid, 'blocked_viewpoint')
        self.m._blocked_cells |= {key for key, cell in self.m.grid.cells.items() if cell.cy >= 0.}
        self.refresh(103.)
        self.assertIsNone(north.choose(self.m, [old.uid], self.now))
        self.assertEqual(north.uid, old.uid)
        self.refresh(114.)
        self.assertIsNone(north.choose(self.m, [old.uid], self.now))
        self.assertIsNone(north.uid)
        self.assertGreater(north.owner_retry[old.uid], self.now)
        self.assertEqual(south.task_id, south_id)

    def test_split_lane_argument_does_not_change_default_east_band(self):
        legacy = bp.SouthPatrol((-50, 150, -60, 60), southeast=False,
                                northeast=False, east=True)
        self.assertIsNone(legacy.east_half)
        for kwargs in (dict(east=False, east_half='north'),
                       dict(east=True, east_half='west')):
            with self.assertRaises(ValueError):
                bp.SouthPatrol((-50, 150, -60, 60), southeast=False, northeast=False, **kwargs)


class EastSplitPersonDispatchTests(unittest.TestCase):
    def setUp(self):
        dispatch_fixture.JointDispatchTests.setUp(self)
        self.m._regional_patrols = tuple(bp.SouthPatrol(
            (-50, 150, -60, 60), southeast=False, northeast=False,
            east=True, east_half=half) for half in ('north', 'south'))
        for patrol, uid in zip(self.m._regional_patrols, ('u0', 'u1')):
            patrol.uid = uid

    def test_both_lane_owners_reject_blind_west_but_own_camera_preempts(self):
        self.assertFalse(self.m._patrol_accepts_target('u0', 1., 0., False))
        self.assertFalse(self.m._patrol_accepts_target('u1', 1., 0., False))
        self.assertTrue(self.m._patrol_accepts_target('u2', 1., 0., False))
        for uid in ('u0', 'u1'):
            with self.subTest(uid=uid), patch.object(self.m, '_camera_candidates',
                                                    return_value=[(uid, 0.)]):
                self.m._tracking.clear()
                self.m._dispatch_target_batch([('t0', 1., 0.)], 100.)
                self.assertEqual(self.m._tracking, {'t0': uid})


if __name__ == '__main__':
    unittest.main()
