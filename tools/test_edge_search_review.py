"""Boundary observations remain finite, legal and below person attention."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/
                       'coordination/src/robocup_swarm/scripts'))
from edge_search_review import EdgeReview, review_points
from swarm_task import ACTOR_SEARCH_BOUNDS,CoverageGrid
from search_observation import in_view
import swarm_agent as sa
from search_segments import LOOK
import test_search_segments as segment_fixture


class EdgeReviewTests(unittest.TestCase):
    def setUp(self):
        self.grid = CoverageGrid(*ACTOR_SEARCH_BOUNDS, 7., 6)
        self.bounds = ACTOR_SEARCH_BOUNDS
        self.visible = lambda *args: True

    def camera(self, now=100., x=127., y=57., yaw=math.pi/2.):
        return (now, x, y, yaw, 3., 0.)

    def test_corners_look_inward_and_along_edges_using_existing_footprint(self):
        for x in (-47., 127.):
            for y in (-57., 57.):
                with self.subTest(x=x, y=y):
                    points = review_points(self.grid, self.camera(x=x,y=y),
                                           self.bounds, self.visible)
                    self.assertTrue(points)
                    for px,py in points.values():
                        self.assertTrue(-50. <= px <= 130. and -60. <= py <= 60.)
                        self.assertLessEqual(math.hypot(px-x,py-y),9.)

    def test_centre_outside_pose_bad_vertical_geometry_and_obstacles_do_not_start(self):
        for camera, visible in ((self.camera(x=0.,y=0.),self.visible),
                (self.camera(x=151.),self.visible),
                ((100.,127.,57.,0.,6.,0.),self.visible),
                (self.camera(),lambda *args:False)):
            self.assertEqual(review_points(self.grid,camera,self.bounds,visible),{})

    def test_northbound_actor_corner_turn_selects_real_ground_instead_of_outer_padding(self):
        # Retain the northbound arrival pattern, inside the corrected real
        # actor corner rather than the former X=150 padded search boundary.
        c=self.camera(x=118.85,y=56.67,yaw=1.64)
        state=EdgeReview(self.bounds)
        heading=state.step('corner',100.,c[1:3],c,self.grid,self.visible)
        self.assertIsNotNone(heading)
        self.assertTrue(math.isfinite(heading))
        self.assertTrue(any(in_view(c[1],c[2],heading,*point,c[4],c[5])
                            for point in state.active['points'].values()))
        self.assertTrue(all(-50.<=x<=130. and -60.<=y<=60.
                            for x,y in state.active['points'].values()))

    def test_safe_aircraft_outside_search_box_can_look_back_at_real_search_space(self):
        c=self.camera(x=133.,y=0.,yaw=math.pi)
        self.assertEqual(sa.flight_waypoint(c[1],c[2]),c[1:3])
        points=review_points(self.grid,c,self.bounds,self.visible)
        self.assertTrue(points)
        self.assertTrue(all(-50.<=x<=130. and -60.<=y<=60.
                            for x,y in points.values()))
        self.assertEqual(review_points(self.grid,self.camera(x=140.,y=0.),
                                      self.bounds,self.visible),{})

    def test_true_corner_above_aircraft_is_not_removed_by_relative_outward_filter(self):
        # The clipped corner is actual search space even though its Y lies
        # beyond the aircraft's safe inset. It is not an infinite outer ray.
        c=self.camera(x=127.5-math.sqrt(8.95**2-1.5**2),y=58.,yaw=0.)
        points=review_points(self.grid,c,self.bounds,self.visible)
        self.assertTrue(any(x>c[1] and y>c[2] for x,y in points.values()))
        self.assertTrue(all(x<=130. and y<=60. for x,y in points.values()))

    def test_repeated_or_previsit_frame_cannot_complete_and_timeout_is_absolute(self):
        state=EdgeReview(self.bounds)
        c=self.camera(now=99.95)
        state.step('a',100.,c[1:3],c,self.grid,self.visible)
        self.assertEqual(state.active['first'],{})
        heading=state.step('a',100.1,c[1:3],self.camera(100.1),self.grid,self.visible)
        aligned=self.camera(100.1,yaw=heading)
        state.step('a',100.2,c[1:3],aligned,self.grid,self.visible)
        state.step('a',100.4,c[1:3],aligned,self.grid,self.visible)
        self.assertFalse(state.active['seen'])
        self.assertIsNone(state.step('a',108.,c[1:3],self.camera(108.),self.grid,self.visible))
        self.assertEqual(state.last_result['reason'],'timeout')
        self.assertIsNone(state.active)

    def test_processed_new_images_finish_and_cooldown_survives_task_changes(self):
        state=EdgeReview(self.bounds)
        own=(127.,57.)
        yaw=math.pi/2.
        now=100.
        while now<106. and (state.active is not None or now==100.):
            c=self.camera(now,yaw=yaw)
            heading=state.step('a',now,own,c,self.grid,self.visible)
            if heading is not None:yaw=heading
            now+=.3
        self.assertIsNone(state.active)
        self.assertEqual(state.last_result['reason'],'imaged')
        self.assertIsNone(state.step('new',now,own,self.camera(now),self.grid,self.visible))
        self.assertNotIn('covered',state.last_result)

    def test_missing_frame_holds_only_original_budget_and_new_task_cannot_renew(self):
        state=EdgeReview(self.bounds)
        c=self.camera()
        state.step('a',100.,c[1:3],c,self.grid,self.visible)
        self.assertTrue(math.isnan(state.step('a',101.,c[1:3],None,self.grid,self.visible)))
        self.assertIsNone(state.step('b',101.1,c[1:3],self.camera(101.1),self.grid,self.visible))
        self.assertIsNone(state.active)

    def test_active_person_task_cancels_review_without_changing_yaw_or_motion(self):
        a=sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.assignment=NS(task_type=1)
        a._edge_review=Mock()
        a._set_search_heading=Mock()
        a._send_vel=Mock()
        with patch.object(sa,'EDGE_SEARCH_REVIEW',True):
            self.assertFalse(a._hold_edge_review())
        a._edge_review.cancel.assert_called_once_with('person_or_task')
        a._set_search_heading.assert_not_called()
        a._send_vel.assert_not_called()

    def test_search_wrapper_uses_single_slewed_yaw_and_zero_motion_outlet(self):
        a=sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id='edge_test'
        a.assignment=NS(task_type=0)
        a.local_xy, a.offset, a.local_z=(127.,57.),(0.,0.),3.
        a.yaw=math.pi/2.
        a._pose_sample_s=a._scan_t=100.
        a._radar_prev=(10.,10.,10.)
        a._world_trusted=lambda:True
        a.grid=sa.GridMap(420,260,.5,(-55.,-65.),bytes(420*260))
        a.cov_grid=self.grid
        a.los=NS(visible=self.visible)
        a._search_scan_key=('corner',1)
        a._search_camera_pose=self.camera()
        a._send_vel=Mock()
        with patch.object(sa,'EDGE_SEARCH_REVIEW',True), \
                patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time(100.)):
            self.assertTrue(a._hold_edge_review())
            a._send_vel.assert_called_once_with(0.,0.)
            self.assertLessEqual(abs(a._search_heading[1]-a.yaw),
                                 sa.SEARCH_HEADING_RATE/sa.CTRL_RATE+1e-9)
            a._radar_prev=(.5,10.,10.)
            a._send_vel.reset_mock()
            self.assertFalse(a._hold_edge_review())
            a._send_vel.assert_not_called()
            self.assertIsNone(a._edge_review.active)


class EdgeSegmentIntegrationTests(unittest.TestCase):
    def test_first_visit_does_not_advance_while_bounded_edge_review_is_active(self):
        fixture=segment_fixture.AgentSegmentTests('test_safe_intermediate_visit_continues_without_zero_command')
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        a=fixture.control_fixture()
        a._hold_edge_review=Mock(return_value=True)
        a._segment_control(3.,5.,.8,2.,2.)
        self.assertEqual(a._search_segment['index'],0)
        self.assertEqual(a._search_segment['state'],LOOK)
        a._hold_edge_review.return_value=False
        a._segment_control(3.,5.,.8,2.,2.)
        self.assertEqual(a._search_segment['index'],1)


if __name__=='__main__':
    unittest.main()
