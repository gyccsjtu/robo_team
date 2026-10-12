"""Replay a real building-review failure before opening its finite LOOK timer."""
import copy
import json
import math
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import test_search_segments as fixture
import swarm_agent as sa
import swarm_manager as sm
import swarm_task as st
from search_segments import TRAVEL, LOOK, FAILED


class DirectedHoverViewTests(unittest.TestCase):
    message = fixture.AgentSegmentTests.message
    control_fixture = fixture.AgentSegmentTests.control_fixture

    def setUp(self):
        fixture.AgentSegmentTests.setUp(self)
        self.record = json.loads((Path(__file__).parent/'fixtures'/
            'building_first_look_hover_2547_040734.json').read_text())
        self.now = self.record['first_look']['status']['received_ros']
        clock = patch.object(sa.rospy.Time,'now',side_effect=lambda:
                             sa.rospy.Time.from_sec(self.now))
        a = self.a = fixture.AgentSegmentTests.directed_fixture(self)
        clock.start()
        self.addCleanup(clock.stop)
        assignment = self.record['assignment']
        point = tuple(assignment['points'][0])
        a.uav_id = assignment['uav']
        a.grid = sm.GridMap(420,260,.5,(-55.,-65.),bytes(420*260))
        a.cov_grid = st.CoverageGrid(*st.ACTOR_SEARCH_BOUNDS,7.,6)
        a.offset = (0.,0.)
        a.state = NS(connected=True,armed=True,mode='OFFBOARD')
        a._world_trusted = Mock(return_value=True)
        a._search_segment.update(id=assignment['task'],revision=assignment['version'],
            created=assignment['stamp'],until=assignment['until'],state=TRAVEL,
            cells=tuple(map(tuple,assignment['cells'])),points=(point,),
            purpose=assignment['purpose'],look_points=tuple(map(tuple,assignment['look_points'])))
        a.assignment.uav_id = a.uav_id
        a.assignment.cell_ix,a.assignment.cell_iy = assignment['cells'][0]
        a.assignment.target_x,a.assignment.target_y = point
        a._search_scan_key = (*assignment['cells'][0],*point)
        a._search_scan_seen = set()
        a._search_scan_first = {}
        a._search_camera_pose = None
        a._search_scan_started_t = None
        a._search_scan_step = Mock()
        a._search_observable_points = Mock(return_value={})
        a._hold_edge_review = Mock(return_value=False)
        a._request_plan = Mock()
        a._segment_path_key = (assignment['task'],assignment['version'])
        a.path_target = point
        a._takeoff_done,a._landing = True,False
        self.body(self.record['first_look']['status'])

    def body(self, status):
        self.a.local_xy = (status['x'],status['y'])
        self.a.local_z = status['z']
        self.a._pose_sample_s = status['stamp']
        self.a.path = [self.a.world_xy,self.a._search_segment['points'][0]]

    def image(self, camera=None):
        camera = copy.deepcopy(camera or self.record['first_look']['camera'])
        self.a._processed_camera_frame_cb(NS(data=json.dumps(camera)))

    def good_sample(self):
        self.now = self.record['later_status']['received_ros']
        self.body(self.record['later_status'])
        self.image(self.record['later_camera'])

    def control(self):
        return self.a._segment_control(3.,5.,.8,2.,2.)

    def identity(self):
        task = self.a._search_segment
        return task['id'],task['revision'],task['created'],task['until'],task['index']

    def assert_no_credit(self):
        self.assertEqual(self.a._search_segment['index'],0)
        self.assertIsNone(self.a._search_scan_started_t)
        self.assertEqual(self.a._search_scan_seen,set())
        self.assertFalse(self.a._search_scan_done)
        cell = self.a.cov_grid.cell(tuple(self.record['assignment']['cells'][0]))
        self.assertNotEqual(cell.state,st.STATE_COVERED)
        self.assertEqual(cell.covered_t,0.)

    def test_real_braking_sample_keeps_travel_toward_original_safe_viewpoint(self):
        self.image()
        before = self.identity()
        f = self.record['first_look']['camera']
        look = self.record['assignment']['look_points'][0]
        angle = math.degrees(math.atan2(f['z'],math.dist((f['x'],f['y']),look)))
        self.assertGreater(angle,27.)  # level camera cannot see this ground yet
        self.assertLess(angle-math.degrees(f['down_pitch']),27.)
        self.control()
        self.assertEqual(self.a._search_segment['state'],TRAVEL)
        self.a._search_scan_step.assert_not_called()
        self.assertEqual(self.a._last_local_goal,tuple(self.record['assignment']['points'][0]))
        vx,vy = self.a._send_vel.call_args.args
        goal = self.a._search_segment['points'][0]
        dx,dy = goal[0]-self.a.world_xy[0],goal[1]-self.a.world_xy[1]
        self.assertGreater(vx*dx+vy*dy,0.)
        self.assertEqual(before,self.identity())
        self.assert_no_credit()

    def test_original_waypoint_is_checked_safe_and_link_kept(self):
        self.image()
        point = self.a._search_segment['points'][0]
        self.assertEqual(point,self.a._segment_flight_waypoint(*point))
        self.assertTrue(sa.clear_line(self.a.grid,self.a.world_xy,point))
        self.control()
        self.assertEqual(self.a.path_target,point)
        self.assert_no_credit()

    def test_at_waypoint_without_hover_qualification_stops_without_starting_scan(self):
        self.image()
        self.a.local_xy = self.a._search_segment['points'][0]
        self.control()
        self.assertEqual(self.a._search_segment['state'],TRAVEL)
        self.a._send_vel.assert_called_with(0.,0.)
        self.a._search_scan_step.assert_not_called()
        self.assert_no_credit()

    def test_real_later_frame_starts_look_without_changing_task_or_granting_coverage(self):
        before = self.identity()
        self.good_sample()
        self.control()
        self.assertEqual(self.a._search_segment['state'],LOOK)
        self.a._search_scan_step.assert_called_once()
        self.assertEqual(before,self.identity())
        self.assert_no_credit()

    def test_hover_geometry_does_not_replace_actual_image_geometry(self):
        self.good_sample()
        f = copy.deepcopy(self.record['later_camera'])
        f['down_pitch'] = -.1
        self.a._search_camera_pose = None
        self.a._search_last_frame = 0.
        self.image(f)
        self.control()
        self.assertEqual(self.a._search_segment['state'],TRAVEL)
        self.a._search_scan_step.assert_not_called()
        self.assert_no_credit()

    def test_missing_stale_future_wrong_source_or_unprocessed_camera_remains_rejected(self):
        for error in ('missing','stale','future','wrong_source','unprocessed'):
            with self.subTest(error=error):
                self.now = self.record['later_status']['received_ros']
                self.body(self.record['later_status'])
                self.a._search_camera_pose = None
                self.a._search_last_frame = 0.
                f = copy.deepcopy(self.record['later_camera'])
                if error=='stale':f['sample_s']=self.now-1.
                if error=='future':f['sample_s']=self.now+.1
                if error=='wrong_source':f['source_uav']='typhoon_h480_5'
                if error=='unprocessed':f['processed']=False
                if error!='missing':self.image(f)
                self.assertIsNone(self.a._search_camera_pose)
                self.control()
                self.assertEqual(self.a._search_segment['state'],TRAVEL)
                self.assert_no_credit()
        self.a._search_scan_step.assert_not_called()

    def test_stale_or_untrusted_body_cannot_start_an_otherwise_valid_look(self):
        self.good_sample()
        for trusted in (False,True):
            with self.subTest(trusted=trusted):
                self.a._world_trusted.return_value=trusted
                self.a._pose_sample_s=self.now if not trusted else self.now-1.
                self.control()
                self.assertEqual(self.a._search_segment['state'],TRAVEL)
                self.assert_no_credit()
        self.a._search_scan_step.assert_not_called()

    def test_existing_look_does_not_restart_or_extend_its_original_eight_second_budget(self):
        self.image()
        self.a._search_segment['state']=LOOK
        self.a._search_scan_step=sa.SwarmAgent._search_scan_step.__get__(self.a)
        self.a._search_scan_yaw0=self.a.yaw
        self.a._search_scan_started_t=self.now-8.1
        before = self.identity()
        self.control()
        self.assertTrue(self.a._search_scan_failed)
        self.assertEqual(self.a._search_scan_started_t,self.now-8.1)
        self.assertEqual(before,self.identity())
        self.a._send_vel.assert_called_with(0.,0.)

    def test_south_patrol_uses_same_observable_hover_entry(self):
        self.image()
        self.a._search_segment['purpose']='south_patrol'
        self.control()
        self.assertEqual(self.a._search_segment['state'],TRAVEL)
        self.a._search_scan_step.assert_not_called()
        self.assert_no_credit()

    def test_ordinary_coverage_arrival_keeps_existing_semantics(self):
        self.image()
        self.a._search_segment['purpose']='coverage'
        self.a._search_segment['look_points']=()
        self.control()
        self.assertEqual(self.a._search_segment['index'],1)
        self.a._search_scan_step.assert_not_called()
        self.assertEqual(self.a._search_scan_seen,set())

    def test_expired_task_cannot_buy_time_waiting_for_hover_geometry(self):
        self.image()
        self.now=self.a._search_segment['until']+.01
        self.control()
        self.assertEqual(self.a._search_segment['state'],FAILED)
        self.a._send_vel.assert_called_with(0.,0.)
        self.a._search_scan_step.assert_not_called()
        self.assert_no_credit()


if __name__ == '__main__':
    unittest.main()
