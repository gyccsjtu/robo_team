"""A coarse blocked flight cell can still contain observable free ground."""
import io
import json
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import test_search_segments as fixture
import swarm_agent as sa
import swarm_manager as sm
import swarm_task as st
from building_viewpoints import reserved_subpoint, valid_view, view_options
from geometry_msgs.msg import Point
from robocup_swarm.msg import SearchAssignment
from search_observation import cell_points, in_view
from search_segments import SegmentBook, TRAVEL, LOOK, DONE, clear_line


class BuildingSubpointReviewTests(unittest.TestCase):
    uid = fixture.ManagerSegmentTests.uid

    def setUp(self):
        fixture.ManagerSegmentTests.setUp(self)
        self.uid = fixture.ManagerSegmentTests.uid
        self.m.grid = st.CoverageGrid(0., 35., 0., 35., 7., 1)
        self.key = self.m.grid.world_to_cell(17.5, 17.5)
        self.m.allocator = st.TaskAllocator(self.m.grid)
        self.m.lease = st.LeaseManager(self.m.grid)
        self.m._merged_grid = Mock(return_value=self.building())
        self.m._slam_lock = object()
        self.m.status[self.uid].x, self.m.status[self.uid].y = 5., 29.
        self.m._cell_edge = {}
        del self.m._choose_blind_viewpoint  # exercise the real production selector

    @staticmethod
    def building(halfwidth=3.):
        n, resolution, origin = 240, .25, (-10., -10.)
        cells = bytearray(n*n)
        for iy in range(n):
            for ix in range(n):
                x, y = origin[0]+(ix+.5)*resolution, origin[1]+(iy+.5)*resolution
                if abs(x-17.5) <= halfwidth and abs(y-17.5) <= halfwidth:
                    cells[iy*n+ix] = 1
        return sm.GridMap(n, n, resolution, origin, cells)

    def rebuild(self, enabled=True):
        with patch.object(sm, 'BUILDING_SUBPOINT_REVIEW', enabled), \
                patch.object(sm, 'EDGE_ENABLE', False):
            self.m._blocked_cells = self.m._find_blocked_cells()
        return self.m._blocked_cells

    def reserve_review(self):
        self.rebuild()
        view = self.m._select_building_view(self.uid, self.key)
        self.assertIsNotNone(view)
        point, look = view
        task = self.m._start_search_segment(self.uid, [self.key], [point], self.now,
            purpose='building_review', look_points=[look])
        return task

    def wire(self, task):
        self.m._send_segment(task)
        msg = self.m.assign_pub.publish.call_args.args[0]
        buffer = io.BytesIO()
        msg.serialize(buffer)
        received = SearchAssignment()
        received.deserialize(buffer.getvalue())
        a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        holder = NS(a=a, addCleanup=self.addCleanup)
        # Reuse the actual receiver fixture without duplicating agent setup.
        fixture.AgentSegmentTests.setUp(holder)
        a = holder.a
        a.uav_id, a.cov_grid = self.uid, self.m.grid
        return a, received

    def test_disabled_keeps_actual_six_metre_building_counterexample(self):
        self.assertIn(self.key, self.rebuild(enabled=False))
        self.assertEqual(self.m.grid.cell(self.key).state, st.STATE_FREE)
        self.assertNotIn(self.key, self.m._cell_waypoint)

    def test_enabled_observes_free_subpoint_instead_of_flying_into_building(self):
        self.assertNotIn(self.key, self.rebuild())
        raw, inflated = self.m._merged_grid.return_value, self.m._g_infl
        centre = self.m.grid.cell(self.key)
        self.assertFalse(raw.is_free(raw.world_to_cell((centre.cx, centre.cy))))
        task = self.reserve_review()
        point, look = task.points[0], task.look_points[0]
        self.assertTrue(raw.is_free(raw.world_to_cell(look)))
        self.assertTrue(valid_view(raw, inflated, look, point, self.m._segment_flight_waypoint))
        self.assertTrue(reserved_subpoint(self.m.grid, self.key, look))
        self.assertNotEqual(look, (centre.cx, centre.cy))
        self.assertNotEqual(self.m.grid.world_to_cell(*point), self.key)
        self.assertTrue(in_view(*point, math.atan2(look[1]-point[1],look[0]-point[0]),
                                *look, 4.5, 0.))
        self.assertEqual(task.cells, (self.key,))
        self.assertEqual(task.purpose, 'building_review')
        self.assertEqual(self.m.grid.cell(self.key).state, st.STATE_ASSIGNED)
        self.assertEqual(self.m.grid.cell(self.key).covered_t, 0.)
        self.assertEqual(self.m.allocator.last_seen, {})

    def test_actual_checked_aircraft_path_preserves_inflation_and_no_corner_cutting(self):
        task = self.reserve_review()
        origin = (self.m.status[self.uid].x, self.m.status[self.uid].y)
        route = sm.plan(self.m._g_infl, origin, task.points[0], connectivity=8,
                        prevent_corner_cutting=True, max_expansions=2500)
        self.assertTrue(route.success)
        path = [origin]+list(route.points)+[task.points[0]]
        self.assertTrue(all(clear_line(self.m._g_infl,a,b) for a,b in zip(path,path[1:])))

    def test_whole_cell_raw_solid_stays_blocked_and_uncovered(self):
        self.m._merged_grid.return_value = self.building(halfwidth=3.6)
        self.assertIn(self.key, self.rebuild())
        self.assertNotIn(self.key, self.m._building_viewpoints)
        self.assertEqual(self.m.grid.cell(self.key).state, st.STATE_FREE)

    def test_no_safe_air_viewpoint_does_not_admit_free_ground(self):
        raw = self.building()
        occupied = sm.GridMap(raw.width,raw.height,raw.resolution,raw.origin,
                              bytes([1])*(raw.width*raw.height))
        self.assertEqual(view_options(self.m.grid,self.key,raw,occupied,
                                     self.m._segment_flight_waypoint), ())

    def test_ground_endpoint_and_occluding_wall_are_not_exempted(self):
        raw = self.building()
        inflated = sm.inflate_grid(raw, sm.INFLATE_M)
        self.assertFalse(valid_view(raw,inflated,(17.5,17.5),(17.5,29.49),
                                    self.m._segment_flight_waypoint))
        self.assertFalse(valid_view(raw,inflated,(14.01,17.5),(29.49,17.5),
                                    self.m._segment_flight_waypoint))

    def test_no_checked_path_releases_seed_and_uses_existing_failure_cooldown(self):
        self.rebuild()
        self.m.allocator.allocate = Mock(return_value={self.uid:self.key})
        self.m.lease.grant(self.uid,self.key,self.now,20.)
        self.m._renew_segments = Mock()
        self.m._idle_uavs = Mock(return_value=[self.uid])
        with patch('search_segment_manager.plan',return_value=NS(success=False)):
            self.m._allocate_segments(sm.rospy.Time.from_sec(self.now))
        self.assertEqual(self.m._segments.tasks, {})
        self.assertEqual(self.m._segments.reserved, {})
        self.assertEqual(self.m.grid.cell(self.key).state,st.STATE_FREE)
        self.assertEqual(self.m.allocator.failed_visit[self.key],self.now)

    def test_manager_wire_to_agent_accepts_exact_bound_subpoint_without_coverage(self):
        task = self.reserve_review()
        a,msg = self.wire(task)
        self.assertTrue(a._accept_segment_message(msg))
        self.assertEqual(a._search_segment['look_points'],task.look_points)
        self.assertEqual(a._search_segment['purpose'],'building_review')
        self.assertEqual(a._search_segment['state'],TRAVEL)
        self.assertNotEqual(self.m.grid.cell(self.key).state,st.STATE_COVERED)

    def test_arbitrary_wrong_cell_future_expired_and_unsafe_protocol_points_rejected(self):
        task = self.reserve_review()
        for error in ('arbitrary','wrongcell','future','expired','unsafe','nonfinite'):
            with self.subTest(error=error):
                a,msg=self.wire(task)
                if error=='arbitrary':msg.search_look_points[0].x += .1
                if error=='wrongcell':msg.search_look_points[0]=Point(x=3.5,y=3.5)
                if error=='future':msg.header.stamp=sa.rospy.Time.from_sec(self.now+.1)
                if error=='expired':msg.search_valid_until=sa.rospy.Time.from_sec(self.now)
                if error=='unsafe':msg.search_waypoints[0].x=-50.
                if error=='nonfinite':msg.search_look_points[0].x=math.nan
                self.assertFalse(a._accept_segment_message(msg))

    def test_ordinary_coverage_cannot_gain_building_subpoint_permission(self):
        a,msg=self.wire(self.reserve_review())
        msg.search_purpose='coverage'
        self.assertFalse(a._accept_segment_message(msg))

    def test_building_centre_outside_actor_search_envelope_is_rejected(self):
        a,msg=self.wire(self.reserve_review())
        a.cov_grid=st.CoverageGrid(-70.,-63.,0.,7.,7.,1)
        msg.search_cell_ix,msg.search_cell_iy=[0],[0]
        msg.search_look_points=[Point(x=-66.5,y=3.5)]
        self.assertFalse(a._accept_segment_message(msg))

    def test_person_preemption_and_disconnect_release_single_reservation(self):
        for reason in ('person_task','timeout_or_unavailable'):
            with self.subTest(reason=reason):
                task=self.reserve_review()
                self.m._drop_segment(self.uid,reason)
                self.assertEqual(self.m._segments.reserved,{})
                self.assertNotIn(self.uid,self.m._segments.tasks)
                self.assertEqual(self.m.grid.cell(self.key).state,st.STATE_FREE)
                cancel=self.m.assign_pub.publish.call_args.args[0]
                self.assertEqual((cancel.task_type,cancel.search_task_id,cancel.search_revision),
                                 (255,task.task_id,task.revision))

    def test_obstacle_change_cancels_instead_of_replacing_with_wall_centre(self):
        self.reserve_review()
        self.m._merged_grid.return_value=self.building(halfwidth=3.6)
        self.m._reconcile_segments()
        self.assertEqual(self.m._segments.tasks,{})
        self.assertEqual(self.m._segments.reserved,{})
        self.assertNotEqual(self.m.grid.cell(self.key).state,st.STATE_COVERED)

    def test_two_aircraft_cannot_reserve_same_building_cell(self):
        task=self.reserve_review()
        self.assertIsNone(self.m._start_search_segment('typhoon_h480_1',task.cells,
            task.points,self.now,purpose='building_review',look_points=task.look_points))
        self.assertEqual(self.m._segments.reserved,{self.key:task.task_id})

    def test_replan_keeps_ground_identity_and_original_deadline(self):
        task=self.reserve_review()
        oldlook, deadline, oldrevision=task.look_points,task.deadline,task.revision
        self.m._revise_segment(task,list(task.cells),list(task.points),self.now+2.,'test')
        self.assertEqual(task.look_points,oldlook)
        self.assertEqual(task.deadline,deadline)
        self.assertEqual(task.revision,oldrevision+1)
        self.assertIsNone(self.m._segments.release(self.uid,task.task_id,oldrevision))

    def test_building_review_is_one_cell_only(self):
        with self.assertRaises(ValueError):
            SegmentBook().reserve(self.uid,[(1,1),(2,1)],[(1.,1.),(2.,1.)],
                self.now,20.,90.,purpose='building_review',look_points=[(1.,1.),(2.,1.)])

    def west_review(self, north=False):
        self.m.grid = st.CoverageGrid(*st.ACTOR_SEARCH_BOUNDS, 7., 6)
        self.key = (0,17 if north else 0)
        self.m.allocator = st.TaskAllocator(self.m.grid)
        self.m.lease = st.LeaseManager(self.m.grid)
        self.m.grid.cell(self.key).state = st.STATE_REVIEW
        raw = sm.GridMap(420,260,.5,(-55.,-65.),bytes(420*260))
        self.m._merged_grid.return_value = raw
        self.m._g_infl = sm.inflate_grid(raw,sm.INFLATE_M)
        self.m._camera_point_seen = {}
        self.m.status[self.uid].x = -40.
        self.m.status[self.uid].y = 50. if north else -50.
        self.m._blocked_cells = set()
        with patch.object(sm,'BUILDING_SUBPOINT_REVIEW',True):
            point = self.m._choose_blind_viewpoint(self.key,self.m.grid.cell(self.key),
                (self.m.status[self.uid].x,self.m.status[self.uid].y))
        self.assertIsNotNone(point)
        return self.m._select_building_view(self.uid,self.key)

    def test_real_western_corners_use_safe_interior_aircraft_and_exact_ground_point(self):
        for north in (False,True):
            with self.subTest(north=north):
                point,look=self.west_review(north)
                expected=(-49.99,59.99 if north else -59.99)
                self.assertEqual(look,expected)
                self.assertEqual(point,self.m._segment_flight_waypoint(*point))
                self.assertAlmostEqual(math.dist(point,look),8.95)
                self.assertTrue(valid_view(self.m._merged_grid.return_value,self.m._g_infl,
                    look,point,self.m._segment_flight_waypoint))
                self.assertEqual(self.m.grid.cell(self.key).state,st.STATE_REVIEW)
                self.assertEqual(self.m.grid.cell(self.key).covered_t,0.)

    def test_west_review_is_same_single_reservation_and_cannot_outlive_ninety_second_evidence(self):
        point,look=self.west_review()
        self.m.allocator.allocate=Mock(return_value={self.uid:self.key})
        self.m._renew_segments=Mock()
        self.m._idle_uavs=Mock(return_value=[self.uid])
        self.now=180.
        self.m._allocate_segments(sm.rospy.Time.from_sec(self.now))
        task=self.m._segments.tasks[self.uid]
        self.assertEqual(task.purpose,'building_review')
        self.assertEqual(task.look_points,(look,))
        self.assertEqual(task.cells,(self.key,))
        self.assertEqual(task.deadline,190.)
        self.assertNotEqual(self.m.grid.cell(self.key).state,st.STATE_COVERED)
        self.now=190.
        self.assertEqual(self.m._subpoint_views(self.key),())

    def test_subpoints_already_actually_imaged_do_not_gain_forced_directed_review(self):
        self.west_review()
        self.m._camera_point_seen={(self.key,j):100. for j in range(9)}
        with patch.object(sm,'BUILDING_SUBPOINT_REVIEW',True):
            self.assertIsNone(self.m._choose_blind_viewpoint(self.key,
                self.m.grid.cell(self.key),(-40.,-50.)))

    def test_real_two_image_coverage_credits_only_visible_raw_free_subpoints(self):
        task=self.reserve_review()
        point,look=task.points[0],task.look_points[0]
        self.m.status[self.uid].x,self.m.status[self.uid].y=point
        self.m.status[self.uid].z=4.2
        self.m._camera_point_seen={}
        for now in (100.,100.3):
            self.now=now
            self.m._processed_camera_frame_cb(NS(data=json.dumps(dict(processed=True,
                source_uav=self.uid,sample_s=now,x=point[0],y=point[1],
                z=4.45,down_pitch=0.,yaw=math.atan2(look[1]-point[1],look[0]-point[0])))))
        index=cell_points(self.m.grid,self.m.grid.cell(self.key)).index(look)
        self.assertIn((self.key,index),self.m._camera_point_seen)
        for (key,j),_sample in self.m._camera_point_seen.items():
            p=cell_points(self.m.grid,self.m.grid.cell(key))[j]
            self.assertTrue(self.m._merged_grid.return_value.is_free(
                self.m._merged_grid.return_value.world_to_cell(p)))
        self.assertIs(self.m._segments.tasks[self.uid],task)
        self.assertEqual(task.index,0)

    def scan_agent(self):
        case=fixture.AgentSegmentTests('test_invalid_or_expired_segment_is_rejected')
        case.setUp()
        self.addCleanup(case.doCleanups)
        a=case.directed_fixture()
        a._search_segment.update(purpose='building_review',cells=((0,0),),
            points=((3.5,-8.95),),look_points=((3.5,0.),))
        a.local_xy=(3.5,-8.95)
        a.local_z=4.2
        a._search_camera_pose=(100.,3.5,-8.95,math.pi/2,4.45,0.)
        a._world_trusted=Mock(return_value=True)
        a._hold_edge_review=Mock(return_value=False)
        a.path=[a.local_xy]
        a.cov_grid=st.CoverageGrid(0.,7.,-3.5,3.5,7.,1)
        a.slam=NS(world_to_cell=a.grid.world_to_cell,is_free_raw=a.grid.is_free)
        a.los=st.LineOfSight(lambda ix,iy:not a.grid.is_free((ix,iy)),
                            a.grid.resolution,a.grid.origin)
        a._search_observable_points=sa.SwarmAgent._search_observable_points.__get__(a)
        return a

    def test_building_review_requires_actual_camera_before_first_look(self):
        a=self.scan_agent()
        a._search_camera_pose=None
        a._segment_control(3.,5.,.8,2.,2.)
        self.assertEqual(a._search_segment['state'],TRAVEL)
        self.assertIsNone(a._search_scan_started_t)
        self.assertEqual(a._search_segment['index'],0)

    def test_building_review_uses_original_two_frame_dwell_without_fake_credit(self):
        a=self.scan_agent()
        a._segment_control(3.,5.,.8,2.,2.)
        self.assertEqual(a._search_segment['state'],LOOK)
        self.assertFalse(a._search_scan_done)
        for sample in (100.1,100.4):
            a._pose_sample_s=sample
            with patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time.from_sec(sample)):
                a._processed_camera_frame_cb(NS(data=json.dumps(dict(processed=True,
                    source_uav=a.uav_id,sample_s=sample,x=3.5,y=-8.95,
                    yaw=math.pi/2,z=4.45,down_pitch=0.))))
                a._search_scan_step()
        self.assertTrue(a._search_scan_done)
        self.assertEqual(a._search_scan_seen,{4})

    def test_building_review_keeps_existing_eight_second_timeout(self):
        a=self.scan_agent()
        a._search_scan_step()
        with patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time.from_sec(108.1)):
            a._search_scan_step()
        self.assertTrue(a._search_scan_failed)
        self.assertEqual(a._search_scan_seen,set())
        a._send_vel.assert_called_with(0.,0.)

    def test_newly_solid_ground_cannot_complete_a_directed_building_look(self):
        a=self.scan_agent()
        a.slam.is_free_raw=Mock(return_value=False)
        a._search_scan_step()
        sample=100.4
        a._pose_sample_s=sample
        a._search_camera_pose=(sample,3.5,-8.95,math.pi/2,4.45,0.)
        with patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time.from_sec(sample)):
            a._search_scan_step()
        self.assertFalse(a._search_scan_done)
        self.assertEqual(a._search_scan_seen,set())

    def test_review_cache_drops_samples_after_real_processed_image_credit(self):
        self.west_review()
        samples=cell_points(self.m.grid,self.m.grid.cell(self.key))
        self.m._camera_point_seen={(self.key,j):100. for j in range(len(samples))}
        self.assertEqual(self.m._subpoint_views(self.key),())

    def test_reselecting_review_does_not_refresh_its_original_ninety_second_deadline(self):
        self.west_review()
        self.now=180.
        with patch.object(sm,'BUILDING_SUBPOINT_REVIEW',True):
            self.assertIsNotNone(self.m._choose_blind_viewpoint(self.key,
                self.m.grid.cell(self.key),(-40.,-50.)))
        self.assertEqual(self.m._review_subpoint_views[self.key]['expires_s'],190.)


if __name__ == '__main__':
    unittest.main()
