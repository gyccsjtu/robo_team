"""Real manager packets must reach the aircraft without widening look permissions."""
import copy
import io
import json
import math
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))

import swarm_agent as sa
import swarm_manager as sm
import swarm_task as st
from geometry_msgs.msg import Point
from robocup_swarm.msg import SearchAssignment, UavStatus
from search_observation import cell_points
from search_segments import SegmentBook


class CornerSearchProtocolTests(unittest.TestCase):
    def setUp(self):
        self.packets = json.loads((ROOT / 'tools/fixtures/corner_search_packets_014123.json')
                                  .read_text())['messages']
        self.now = 100.
        for context in (patch.object(sa.rospy.Time, 'now', side_effect=lambda:
                                    sa.rospy.Time.from_sec(self.now)),
                        patch.object(st.rospy, 'get_time', side_effect=lambda: self.now)):
            context.start()
            self.addCleanup(context.stop)

    def producer_message(self, index=0):
        packet = self.packets[index]
        self.now = packet['stamp'] + .1
        m = self.manager = sm.SwarmManager.__new__(sm.SwarmManager)
        m.grid = st.CoverageGrid(*st.ACTOR_SEARCH_BOUNDS, 7.)
        m.allocator = st.TaskAllocator(m.grid)
        m.lease = st.LeaseManager(m.grid)
        m._segments = SegmentBook()
        m._segment_max_hold = 90.
        m._active_leases = {}
        sent = []
        m.assign_pub = NS(publish=sent.append)
        task = m._start_search_segment(packet['uav'],
            [tuple(key) for key in packet['cells']], packet['points'], packet['stamp'],
            purpose=packet['purpose'], publish=False, look_points=packet['look_points'])
        task.task_id, task.revision, task.expires = packet['task'], packet['version'], packet['until']
        for key in task.cells:
            m._segments.reserved[key] = task.task_id
        m._send_segment(task)
        buffer = io.BytesIO()
        sent[-1].serialize(buffer)
        msg = SearchAssignment()
        msg.deserialize(buffer.getvalue())
        a = self.agent = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id, a.cov_grid = packet['uav'], m.grid
        a._plan_lock, a._plan_epoch = threading.Lock(), 0
        a._plan_pending, a.path_target, a.path = None, None, []
        a._search_scan_key = None
        # The recorded run used a 1.5 m flight inset; retain its actual
        # waypoints rather than silently altering this producer fixture.
        a._segment_flight_waypoint = lambda x,y: sa.flight_waypoint(x,y,inset=1.5)
        return msg

    def test_real_north_and_south_packets_cross_producer_wire_and_receiver(self):
        for index in range(2):
            with self.subTest(index=index):
                msg = self.producer_message(index)
                self.assertTrue(self.agent._accept_segment_message(msg))
                task = self.agent._search_segment
                self.assertEqual(task['look_points'],
                                 tuple(map(tuple, self.packets[index]['look_points'])))
                self.assertEqual(task['cells'], tuple(map(tuple, self.packets[index]['cells'])))
                self.assertTrue(self.manager._segments.matches(
                    msg.uav_id, task['id'], task['revision']))
                self.assertTrue(all(cell.state != st.STATE_COVERED
                                    for cell in self.manager.grid.cells.values()))
                self.assertTrue(all(cell.covered_t == 0.
                                    for cell in self.manager.grid.cells.values()))
                # The aircraft can now ACK the actual accepted identity.
                self.agent.local_xy, self.agent.offset = tuple(self.packets[index]['points'][0]), (0.,0.)
                status = UavStatus()
                self.agent._segment_status(status)
                self.assertEqual(status.search_task_id, msg.search_task_id)
                self.assertEqual(status.search_revision, msg.search_revision)
                self.assertEqual(status.search_index, 0)

    def test_callbacks_establish_the_actual_corner_assignment(self):
        for index in range(2):
            with self.subTest(index=index):
                msg = self.producer_message(index)
                self.agent._assign_cb(msg)
                self.assertIs(self.agent.assignment, msg)
                self.assertEqual(self.agent._search_segment['look_points'][0],
                                 tuple(self.packets[index]['look_points'][0]))
                self.assertFalse(self.agent._search_scan_done)

    def test_arbitrary_point_inside_corner_cell_is_rejected(self):
        msg = self.producer_message()
        msg.search_look_points[0].x -= .2
        self.assertFalse(self.agent._accept_segment_message(msg))
        self.assertIsNone(getattr(self.agent, '_search_segment', None))

    def test_corner_belonging_to_another_reserved_cell_is_rejected(self):
        msg = self.producer_message()
        msg.search_look_points[0] = Point(x=129.99,y=52.99)
        self.assertFalse(self.agent._accept_segment_message(msg))

    def test_east_edge_middle_row_and_north_edge_non_east_column_are_rejected(self):
        for key, index in (((25,1),2), ((24,17),8)):
            with self.subTest(key=key):
                msg = self.producer_message()
                msg.search_cell_ix, msg.search_cell_iy = [key[0]], [key[1]]
                look = cell_points(self.agent.cov_grid, self.agent.cov_grid.cell(key))[index]
                msg.search_look_points = [Point(x=look[0],y=look[1])]
                self.assertFalse(self.agent._accept_segment_message(msg))

    def test_ordinary_coverage_still_requires_the_cell_centre(self):
        msg = self.producer_message()
        msg.search_purpose = 'coverage'
        self.assertFalse(self.agent._accept_segment_message(msg))
        cell = self.agent.cov_grid.cell((25,17))
        msg.search_look_points = [Point(x=cell.cx,y=cell.cy)]
        self.assertTrue(self.agent._accept_segment_message(msg))

    def test_nonfinite_or_outside_search_points_are_rejected(self):
        for point in ((math.nan,59.99), (math.inf,59.99), (130.01,59.99), (129.99,60.01)):
            with self.subTest(point=point):
                msg = self.producer_message()
                msg.search_look_points = [Point(x=point[0],y=point[1])]
                self.assertFalse(self.agent._accept_segment_message(msg))

    def test_unsafe_aircraft_waypoint_remains_rejected(self):
        msg = self.producer_message()
        msg.search_waypoints[0].y = 59.99
        self.assertFalse(self.agent._accept_segment_message(msg))

    def test_expired_future_or_unbounded_lease_is_rejected(self):
        for condition in ('expired','future_stamp','unbounded'):
            with self.subTest(condition=condition):
                msg = self.producer_message()
                if condition == 'future_stamp':
                    msg.header.stamp = sa.rospy.Time.from_sec(self.now+.1)
                else:
                    msg.search_valid_until = sa.rospy.Time.from_sec(
                        self.now if condition == 'expired' else self.now+121.)
                self.assertFalse(self.agent._accept_segment_message(msg))

    def mark_observed_scan(self):
        a = self.agent
        a._search_scan_done = a._search_scan_directed_done = True
        a._search_scan_seen, a._search_scan_first = {8}, {8: (self.now-.4,self.now-.1)}
        a._search_scan_directed_first = self.now-.4
        a._search_scan_directed_last = self.now-.1

    def next_revision(self, msg):
        self.now += .2
        revised = copy.deepcopy(msg)
        revised.search_revision += 1
        revised.header.stamp = sa.rospy.Time.from_sec(self.now-.05)
        return revised

    def assert_scan_reset(self):
        self.assertFalse(self.agent._search_scan_done)
        self.assertFalse(self.agent._search_scan_directed_done)
        self.assertEqual(self.agent._search_scan_seen,set())
        self.assertEqual(self.agent._search_scan_first,{})
        self.assertIsNone(self.agent._search_scan_directed_first)
        self.assertIsNone(self.agent._search_scan_directed_last)

    def test_same_waypoint_revision_changing_centre_to_corner_resets_all_scan_evidence(self):
        corner = self.producer_message()
        centre = copy.deepcopy(corner)
        cell = self.agent.cov_grid.cell((25,17))
        centre.search_look_points = [Point(x=cell.cx,y=cell.cy)]
        self.agent._assign_cb(centre)
        self.mark_observed_scan()
        corner = self.next_revision(corner)
        self.agent._assign_cb(corner)
        self.assertEqual(self.agent._search_segment['revision'],2)
        self.assertEqual(self.agent._search_segment['look_points'][0],(129.99,59.99))
        self.assert_scan_reset()

    def test_same_look_revision_changing_purpose_resets_scan_evidence(self):
        msg = self.producer_message()
        cell = self.agent.cov_grid.cell((25,17))
        msg.search_look_points = [Point(x=cell.cx,y=cell.cy)]
        msg.search_purpose = 'coverage'
        self.agent._assign_cb(msg)
        self.mark_observed_scan()
        revised = self.next_revision(msg)
        revised.search_purpose = 'south_patrol'
        self.agent._assign_cb(revised)
        self.assertEqual(self.agent._search_segment['purpose'],'south_patrol')
        self.assert_scan_reset()

    def test_unchanged_look_new_revision_keeps_valid_scan_progress(self):
        msg = self.producer_message()
        self.agent._assign_cb(msg)
        self.mark_observed_scan()
        self.agent._assign_cb(self.next_revision(msg))
        self.assertTrue(self.agent._search_scan_done)
        self.assertEqual(self.agent._search_scan_seen,{8})

    def test_old_revision_old_cancel_and_mutated_heartbeat_cannot_replace_task(self):
        msg = self.producer_message()
        self.agent._assign_cb(msg)
        self.agent._assign_cb(self.next_revision(msg))
        self.assertFalse(self.agent._accept_segment_message(msg))
        cancel = SearchAssignment(uav_id=msg.uav_id,task_type=255,
            search_task_id=msg.search_task_id,search_revision=1)
        cancel.header.stamp=sa.rospy.Time.from_sec(self.now)
        self.assertFalse(self.agent._accept_segment_message(cancel))
        heartbeat = self.next_revision(msg)
        cell = self.agent.cov_grid.cell((25,17))
        heartbeat.search_look_points = [Point(x=cell.cx,y=cell.cy)]
        self.assertFalse(self.agent._accept_segment_message(heartbeat))
        self.assertEqual(self.agent._search_segment['revision'],2)
        self.assertEqual(self.agent._search_segment['look_points'][0],(129.99,59.99))

    def test_unversioned_single_cell_message_cannot_downgrade_active_corner_task(self):
        msg = self.producer_message()
        self.agent._assign_cb(msg)
        legacy = SearchAssignment(uav_id=msg.uav_id,task_type=0,
            cell_ix=25,cell_iy=17,target_x=msg.target_x,target_y=msg.target_y)
        legacy.header.stamp=sa.rospy.Time.from_sec(self.now)
        self.assertFalse(self.agent._accept_segment_message(legacy))
        self.assertEqual(self.agent._search_segment['id'],msg.search_task_id)


if __name__ == '__main__':
    unittest.main()
