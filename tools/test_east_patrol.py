"""Persistent region ownership, finite routes and independent person tasks."""
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

from test_southeast_patrol import SoutheastPatrolTests
from test_joint_dispatch import JointDispatchTests
import boundary_patrol as bp
import swarm_manager as sm
import swarm_task as st


class EastPatrolTests(unittest.TestCase):
    def setUp(self):
        SoutheastPatrolTests.setUp(self)
        self.m._south_patrol = bp.SouthPatrol((-50, 150, -60, 60),
                                             southeast=False, northeast=False, east=True)

    def allocate(self):
        self.m._allocate()
        return [t for t in self.m._segments.tasks.values() if t.purpose == 'south_patrol']

    def refresh(self, now):
        self.now = now
        for uid, s in self.m.status.items():
            s.header.stamp = sm.rospy.Time.from_sec(now)
            self.m.last_report[uid] = sm.rospy.Time.from_sec(now)

    def test_one_east_owner_five_regular_unique_routes_and_checked_camera_geometry(self):
        tasks = self.allocate()
        self.assertEqual(len(tasks), 1)
        t = tasks[0]
        self.assertEqual(len(self.m._segments.tasks), 6)
        all_cells = [k for t in self.m._segments.tasks.values() for k in t.cells]
        self.assertEqual(len(all_cells), len(set(all_cells)))
        for point, look in zip(t.points, t.look_points):
            self.assertGreaterEqual(look[0], 105.)
            self.assertGreater(point[0], 90.)
            self.assertTrue(self.m._in_processed_view(*point, 0., *look, 4.5, 0.))
        self.assertGreater(t.deadline-t.created, 90.)
        self.assertLessEqual(t.deadline-t.created, 180.)

    def test_timeout_releases_cells_but_retains_owner_and_replans_after_cooldown(self):
        first = self.allocate()[0]
        uid = first.uid
        self.refresh(first.deadline+.1)
        self.m._renew_segments(sm.rospy.Time.from_sec(self.now))
        p = self.m._south_patrol
        self.assertEqual(p.uid, uid)
        self.assertIsNone(p.task_id)
        self.assertTrue(set(first.cells).isdisjoint(self.m._segments.reserved))
        self.allocate()  # two-second replan pause must not dispatch him west
        self.assertNotIn(uid, self.m._segments.tasks)
        self.refresh(self.now+2.1)
        second = self.allocate()[0]
        self.assertEqual(second.uid, uid)
        self.assertNotEqual(second.task_id, first.task_id)
        self.assertTrue(set(second.cells).isdisjoint(first.cells))
        self.assertTrue(all(self.m.grid.cell(k).state != st.STATE_COVERED for k in first.cells))

    def test_person_preempts_and_another_idle_aircraft_takes_region(self):
        t = self.allocate()[0]
        self.m._tracking['person'] = t.uid
        self.m._free_search_lease(t.uid)
        other = next(uid for uid in self.m.uav_ids if uid != t.uid)
        self.m._drop_segment(other, 'test_idle')
        self.refresh(103.)
        second = self.allocate()[0]
        self.assertEqual(second.uid, other)
        self.assertEqual(self.m._tracking['person'], t.uid)
        self.assertTrue(set(t.cells).isdisjoint(self.m._segments.reserved))

    def test_dead_owner_is_replaced_without_reusing_old_task(self):
        t = self.allocate()[0]
        self.m.status[t.uid].connected = False
        other = next(uid for uid in self.m.uav_ids if uid != t.uid)
        self.m._drop_segment(other, 'test_idle')
        self.refresh(104.)
        self.allocate()
        self.refresh(106.1)
        result = self.allocate()
        self.assertEqual(len(result), 1)
        self.assertNotEqual(result[0].uid, t.uid)

    def test_stale_covered_region_can_be_revisited_without_erasing_coverage(self):
        for c in self.m.grid.cells.values():
            c.state, c.covered_t = st.STATE_COVERED, 10.
        t = self.allocate()[0]
        self.assertTrue(all(self.m.grid.cell(k).state == st.STATE_COVERED for k in t.cells))
        self.m._drop_segment(t.uid, 'test_cancel')
        self.assertTrue(all(self.m.grid.cell(k).state == st.STATE_COVERED for k in t.cells))
        self.assertTrue(all(self.m.grid.cell(k).covered_t == 10. for k in t.cells))

    def test_all_blocked_entry_does_not_pin_same_aircraft_forever(self):
        t = self.allocate()[0]
        self.m._drop_segment(t.uid, 'blocked_viewpoint')
        p = self.m._south_patrol
        self.m._blocked_cells = set(self.m.grid.cells)
        self.refresh(103.)
        self.assertIsNone(p.choose(self.m, [t.uid], self.now))
        self.assertEqual(p.uid, t.uid)
        self.refresh(114.)
        self.assertIsNone(p.choose(self.m, [t.uid], self.now))
        self.assertIsNone(p.uid)
        self.assertGreater(p.owner_retry[t.uid], self.now)


class EastPersonDispatchTests(unittest.TestCase):
    def setUp(self):
        JointDispatchTests.setUp(self)
        self.m._south_patrol = bp.SouthPatrol((-50, 150, -60, 60),
                                             southeast=False, northeast=False, east=True)
        self.m._south_patrol.uid = 'u0'

    def test_blind_west_job_does_not_remove_east_owner(self):
        self.m._dispatch_target_batch([('t0', 1., 0.)], 100.)
        self.assertEqual(self.m._tracking, {'t0': 'u1'})

    def test_actual_camera_can_interrupt_east_transit(self):
        with patch.object(self.m, '_camera_candidates', return_value=[('u0', 0.)]):
            self.m._dispatch_target_batch([('t0', 1., 0.)], 100.)
        self.assertEqual(self.m._tracking, {'t0': 'u0'})

    def test_two_east_people_get_different_owners_then_existing_owner_stays(self):
        with patch.object(self.m, '_camera_candidates', return_value=[('u0', 0.)]):
            self.m._dispatch_target_batch([('t0', 115., 4.), ('t1', 120., 8.)], 100.)
        before = dict(self.m._tracking)
        self.assertEqual(set(before), {'t0', 't1'})
        self.assertEqual(set(before.values()), {'u0', 'u1'})
        self.m._dispatch_target_batch([('t0', 116., 4.), ('t1', 121., 8.)], 100.)
        self.assertEqual(self.m._tracking, before)


if __name__ == '__main__':
    unittest.main()
