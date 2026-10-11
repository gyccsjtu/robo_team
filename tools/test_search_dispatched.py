"""An active search lease follows the issued point, including map updates."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                     'coordination/src/robocup_swarm/scripts'))
import swarm_manager as sm
import swarm_task as st


class SearchDispatchedTests(unittest.TestCase):
    uid = 'typhoon_h480_0'

    def setUp(self):
        self.now = 100.
        clock = patch.object(sm.rospy.Time, 'now', side_effect=lambda:
                             sm.rospy.Time.from_sec(self.now))
        clock.start()
        self.addCleanup(clock.stop)
        edge = patch.object(sm, 'EDGE_ENABLE', False)
        edge.start()
        self.addCleanup(edge.stop)

    def raw_grid(self, obstacle=False, all_blocked=False):
        grid = sm.GridMap(80, 80, 1., (-20., -20.), bytes(80 * 80))
        cells = bytearray(grid.cells)
        for iy in range(grid.height):
            for ix in range(grid.width):
                x, y = grid.cell_to_world((ix, iy))
                if all_blocked or obstacle and (x-3.5)**2+(y-3.5)**2 <= 9.:
                    cells[iy*grid.width+ix] = 1
        return sm.GridMap(grid.width, grid.height, grid.resolution,
                          grid.origin, cells)

    def manager(self, obstacle=False):
        m = sm.SwarmManager.__new__(sm.SwarmManager)
        m.grid = st.CoverageGrid(0, 7, 0, 7, cell_m=7, num_uavs=1)
        key = next(iter(m.grid.cells))
        m._merged_grid = Mock(return_value=self.raw_grid(obstacle))
        m._blocked_cells = m._find_blocked_cells()
        m.allocator = st.TaskAllocator(m.grid)
        m.lease = st.LeaseManager(m.grid)
        m.lease.grant(self.uid, key, self.now, 30.)
        m.uav_ids = [self.uid]
        m._active_leases = {self.uid: key}
        m._lease_hold_since = {(self.uid, key): self.now}
        m._lease_motion = {}
        m._blind_viewpoints = {}
        m._tracking, m._backup = {}, {}
        m.assign_pub = NS(publish=Mock())
        m._publish_assignment(self.uid, key, m.grid.cell(key))
        point = m._search_dispatched[self.uid][1]
        m._lease_motion[self.uid] = (key, self.now, *point)
        self.acknowledge(m, point=point)
        m.assign_pub.publish.reset_mock()
        return m, key, point

    def acknowledge(self, m, point=None, confidence=0., stamp=None):
        key, target, sent_s = m._search_dispatched[self.uid]
        point = target if point is None else point
        sample = self.now if stamp is None else stamp
        m.status = {self.uid: NS(connected=True, x=point[0], y=point[1], z=4.,
            assigned_cell_ix=key[0], assigned_cell_iy=key[1],
            assigned_target_x=target[0], assigned_target_y=target[1],
            confidence=confidence, header=NS(stamp=sm.rospy.Time.from_sec(sample)))}
        m.last_report = {self.uid: sm.rospy.Time.from_sec(self.now)}

    def test_clearance_does_not_change_safe_sent_task(self):
        m, key, sent = self.manager(obstacle=True)
        m._merged_grid.return_value = self.raw_grid()
        m._rebuild_blocked_cells()
        self.assertGreater(math.dist(sent, m._cell_waypoint[key]), sm.COVER_ARRIVE_M)
        self.assertEqual(m._search_dispatched[self.uid][1], sent)
        m.assign_pub.publish.assert_not_called()
        self.assertEqual(m._lease_hold_since[(self.uid, key)], 100.)

    def test_original_five_metre_drift_releases_finished_task(self):
        m, key, sent = self.manager(obstacle=True)
        m._merged_grid.return_value = self.raw_grid()
        m._rebuild_blocked_cells()
        self.acknowledge(m, point=sent, confidence=1.)
        m._renew_leases()
        self.assertNotIn(self.uid, m._active_leases)
        self.assertEqual(m.grid.cell(key).state, st.STATE_REVIEW)
        self.assertNotIn(key, m.allocator.last_seen)
        m.assign_pub.publish.assert_not_called()

    def test_new_obstacle_explicitly_reissues_and_preserves_deadline(self):
        m, key, old = self.manager()
        self.now = 101.
        m._merged_grid.return_value = self.raw_grid(obstacle=True)
        m._rebuild_blocked_cells()
        msg = m.assign_pub.publish.call_args.args[0]
        self.assertEqual(msg.task_type, 0)
        self.assertNotEqual((msg.target_x, msg.target_y), old)
        self.assertTrue(m._search_point_free((msg.target_x, msg.target_y)))
        self.assertEqual(m._search_dispatched[self.uid][2], 101.)
        self.assertEqual(m._lease_hold_since[(self.uid, key)], 100.)
        m.assign_pub.publish.assert_called_once()
        m._rebuild_blocked_cells()
        m.assign_pub.publish.assert_called_once()  # No repeated safe reissue.

    def test_same_cell_old_coordinates_do_not_finish_new_task(self):
        m, key, old = self.manager()
        self.now = 101.
        m._merged_grid.return_value = self.raw_grid(obstacle=True)
        m._rebuild_blocked_cells()
        self.acknowledge(m, confidence=1.)
        m.status[self.uid].assigned_target_x = old[0]
        m.status[self.uid].assigned_target_y = old[1]
        m._renew_leases()
        self.assertEqual(m._active_leases[self.uid], key)
        self.assertEqual(m.grid.cell(key).state, st.STATE_ASSIGNED)

    def test_same_cell_old_timestamp_does_not_finish_new_task(self):
        m, key, _ = self.manager()
        self.now = 101.
        m._merged_grid.return_value = self.raw_grid(obstacle=True)
        m._rebuild_blocked_cells()
        self.acknowledge(m, confidence=1., stamp=100.9)
        m._renew_leases()
        self.assertEqual(m._active_leases[self.uid], key)
        self.acknowledge(m, confidence=1., stamp=101.)
        m._renew_leases()
        self.assertNotIn(self.uid, m._active_leases)

    def test_all_blocked_cancels_without_coverage_credit(self):
        m, key, _ = self.manager()
        m._merged_grid.return_value = self.raw_grid(all_blocked=True)
        m._rebuild_blocked_cells()
        self.assertEqual(m.assign_pub.publish.call_args.args[0].task_type, 255)
        self.assertNotIn(self.uid, m._active_leases)
        self.assertNotIn(self.uid, m._search_dispatched)
        self.assertEqual(m.grid.cell(key).state, st.STATE_FREE)
        self.assertEqual(m.allocator.last_seen, {})

    def test_no_map_does_not_cancel_sent_task(self):
        m, key, point = self.manager()
        m._g_infl = None
        m._merged_grid.return_value = None
        m._rebuild_blocked_cells()
        self.assertEqual(m._search_dispatched[self.uid][1], point)
        self.assertEqual(m._active_leases[self.uid], key)
        m.assign_pub.publish.assert_not_called()

    def test_tracking_and_backup_are_not_retasked_by_stale_search(self):
        for role in ('_tracking', '_backup'):
            with self.subTest(role=role):
                m, key, _ = self.manager()
                setattr(m, role, {'t0': self.uid})
                m._merged_grid.return_value = self.raw_grid(all_blocked=True)
                m._rebuild_blocked_cells()
                m._renew_leases()
                m.assign_pub.publish.assert_not_called()
                self.assertEqual(getattr(m, role), {'t0': self.uid})

    def test_mismatched_dispatch_key_cannot_complete_or_cancel(self):
        m, key, point = self.manager()
        m._search_dispatched[self.uid] = ((99, 99), point, 100.)
        m._merged_grid.return_value = self.raw_grid(all_blocked=True)
        m._rebuild_blocked_cells()
        m.status[self.uid].confidence = 1.
        m._renew_leases()
        self.assertEqual(m._active_leases[self.uid], key)
        m.assign_pub.publish.assert_not_called()

    def test_observed_cell_is_not_reopened_or_cancelled_by_map(self):
        m, key, _ = self.manager()
        cell = m.grid.cell(key)
        cell.state, cell.owner = st.STATE_COVERED, None
        m._merged_grid.return_value = self.raw_grid(all_blocked=True)
        m._rebuild_blocked_cells()
        self.assertEqual(cell.state, st.STATE_COVERED)
        m.assign_pub.publish.assert_not_called()

    def test_reissue_and_old_ack_cannot_extend_maximum_hold(self):
        m, key, _ = self.manager()
        m._lease_hold_since[(self.uid, key)] = 10.
        self.now = 101.
        m.last_report[self.uid] = sm.rospy.Time.from_sec(self.now)
        m._merged_grid.return_value = self.raw_grid(obstacle=True)
        m._rebuild_blocked_cells()
        self.assertEqual(m._lease_hold_since[(self.uid, key)], 10.)
        m._renew_leases()  # Status still describes old point and old timestamp.
        self.assertNotIn(self.uid, m._active_leases)
        self.assertEqual(m.grid.cell(key).state, st.STATE_FREE)

    def test_future_dispatch_timestamp_does_not_accept_completion(self):
        m, key, point = self.manager()
        m._search_dispatched[self.uid] = (key, point, 101.)
        self.acknowledge(m, confidence=1.)
        m._renew_leases()
        self.assertEqual(m._active_leases[self.uid], key)


if __name__ == '__main__':
    unittest.main()
