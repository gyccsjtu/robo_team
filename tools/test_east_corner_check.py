"""Only an existing eastern searcher may inspect an unseen actor-area corner."""
import json
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import test_search_segments as fixture
import swarm_manager as sm
import swarm_task as st
from building_viewpoints import valid_view
from robocup_swarm.msg import UavStatus
from search_segments import ATTEND


class EastCornerCheckTests(unittest.TestCase):
    uid = fixture.ManagerSegmentTests.uid

    def setUp(self):
        fixture.ManagerSegmentTests.setUp(self)
        m = self.m
        m.grid = st.CoverageGrid(*st.ACTOR_SEARCH_BOUNDS, 7., 6)
        m.allocator, m.lease = st.TaskAllocator(m.grid), st.LeaseManager(m.grid)
        m.uav_ids = ['typhoon_h480_%d' % i for i in range(3)]
        m.status = {uid: UavStatus(uav_id=uid, x=x, y=y, z=4.2, connected=True)
                    for uid, (x, y) in zip(m.uav_ids,
                                            ((108., -50.), (106., 50.), (-10., 0.)))}
        m._cell_waypoint = {key: (c.cx, c.cy) for key, c in m.grid.cells.items()}
        self.raw = sm.GridMap(210, 130, 1., (-55., -65.), bytes(210*130))
        m._merged_grid = Mock(return_value=self.raw)
        m._g_infl = sm.inflate_grid(self.raw, sm.INFLATE_M)
        m._slam_lock = object()
        m._east_corner_check = True
        m._processed_frame_times = {uid: self.now-.2 for uid in m.uav_ids}
        m._processed_camera_poses = {uid: dict(sample_s=self.now-.2, z=4.45)
                                     for uid in m.uav_ids}
        m._camera_point_seen, m._east_corner_seen = {}, {}
        for uid, status in m.status.items():
            status.header.stamp = sm.rospy.Time.from_sec(self.now)
            m.last_report[uid] = sm.rospy.Time.from_sec(self.now)

    def tasks(self):
        return [task for task in self.m._segments.tasks.values()
                if task.purpose == 'building_review' and
                task.look_points[0][0] > 129.]

    def test_one_existing_eastern_aircraft_uses_checked_inner_view(self):
        idle = list(self.m.uav_ids)
        self.m._allocate_east_corner_check(idle, self.now)
        self.assertEqual(len(self.tasks()), 1)
        task = self.tasks()[0]
        self.assertIn(task.uid, self.m.uav_ids[:2])
        self.assertNotIn(task.uid, idle)
        self.assertIn(task.look_points[0], ((129.99, -59.99), (129.99, 59.99)))
        self.assertLess(task.points[0][0], 130.)
        self.assertTrue(valid_view(self.raw, self.m._g_infl,
                                   task.look_points[0], task.points[0],
                                   self.m._segment_flight_waypoint))
        self.assertEqual(self.m.grid.cell(task.cells[0]).covered_t, 0.)
        self.assertEqual(self.m._east_corner_seen, {})
        self.m._allocate_east_corner_check(idle, self.now)
        self.assertEqual(len(self.tasks()), 1)

    def test_no_far_or_attending_aircraft_is_recruited(self):
        self.m.status[self.m.uav_ids[0]].x = 70.
        self.m.status[self.m.uav_ids[1]].x = 70.
        self.m._allocate_east_corner_check(list(self.m.uav_ids), self.now)
        self.assertEqual(self.tasks(), [])
        self.m.status[self.m.uav_ids[0]].x = 108.
        self.m.status[self.m.uav_ids[0]].search_state = ATTEND
        self.m._allocate_east_corner_check(list(self.m.uav_ids), self.now)
        self.assertEqual(self.tasks(), [])

    def test_fresh_person_view_keeps_control_and_other_corner_may_run(self):
        protected = self.m.uav_ids[0]
        self.m._eligible_observations = {(protected, 't0'): dict(
            source_uav=protected, sample_s=self.now)}
        self.m._allocate_east_corner_check(list(self.m.uav_ids), self.now)
        self.assertEqual(self.tasks()[0].uid, self.m.uav_ids[1])

    def test_recent_corner_images_and_stale_camera_do_not_create_work(self):
        self.m._east_corner_seen = {'southeast': self.now,
                                    'northeast': self.now}
        self.m._allocate_east_corner_check(list(self.m.uav_ids), self.now)
        self.assertEqual(self.tasks(), [])
        self.m._east_corner_seen.clear()
        self.m._processed_camera_poses.clear()
        self.m._allocate_east_corner_check(list(self.m.uav_ids), self.now)
        self.assertEqual(self.tasks(), [])

    def test_blocked_ground_endpoint_never_gets_a_flight_task(self):
        cells = bytearray(self.raw.cells)
        for _, _, _, look in self.m._east_corner_subpoints():
            ix, iy = self.raw.world_to_cell(look)
            cells[iy*self.raw.width+ix] = 1
        self.m._merged_grid.return_value = sm.GridMap(
            self.raw.width, self.raw.height, self.raw.resolution,
            self.raw.origin, cells)
        self.m._allocate_east_corner_check(list(self.m.uav_ids), self.now)
        self.assertEqual(self.tasks(), [])

    def test_actual_processed_images_not_task_creation_credit_corner(self):
        self.m._allocate_east_corner_check(list(self.m.uav_ids), self.now)
        task = self.tasks()[0]
        point, look = task.points[0], task.look_points[0]
        status = self.m.status[task.uid]
        status.x, status.y = point
        yaw = math.atan2(look[1]-point[1], look[0]-point[0])
        def frame(sample):
            payload = dict(processed=True, source_uav=task.uid, sample_s=sample,
                           x=point[0], y=point[1], z=4.45,
                           down_pitch=0., yaw=yaw)
            self.m._processed_camera_frame_cb(NS(data=json.dumps(payload)))
        self.assertEqual(self.m._east_corner_seen, {})
        frame(self.now)
        self.assertEqual(self.m._east_corner_seen, {})
        self.now += .3
        frame(self.now)
        name = 'southeast' if look[1] < 0 else 'northeast'
        self.assertAlmostEqual(self.m._east_corner_seen[name], self.now)


if __name__ == '__main__':
    unittest.main()
