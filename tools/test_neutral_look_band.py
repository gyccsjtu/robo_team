"""Actual empty-pitch-band regression; no credit from neutral-pose predictions."""
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from tools import test_search_segments as fixture

sa, sm = fixture.sa, fixture.sm
RECORDED = json.loads((Path(__file__).parent/'fixtures/neutral_look_band_050056.json').read_text())


class NeutralLookBandTests(unittest.TestCase):
    def setUp(self):
        case=fixture.AgentSegmentTests('test_invalid_or_expired_segment_is_rejected')
        case.setUp()
        self.addCleanup(case.doCleanups)
        self.a=case.directed_fixture()
        a=self.a
        job=RECORDED['initial_assignment']['data']
        frame=RECORDED['focus_frame']['data']
        pose=RECORDED['pose_before_focus']['pose']
        a.uav_id=RECORDED['source_uav']
        a.offset=tuple(RECORDED['fixed_world_offset'])
        a.local_xy=tuple(pose['xyz'][:2]);a.local_z=pose['xyz'][2]
        a._pose_sample_s=pose['sample_s']
        a._search_segment.update(id=job['task'], revision=job['version'], index=0,
            state=fixture.LOOK, purpose=job['purpose'],
            cells=tuple(map(tuple,job['cells'])),points=tuple(map(tuple,job['points'])),
            look_points=tuple(map(tuple,job['look_points'])))
        a.assignment.cell_ix,a.assignment.cell_iy=job['cells'][0]
        a.assignment.target_x,a.assignment.target_y=job['points'][0]
        a._search_scan_key=(*job['cells'][0],*job['points'][0])
        a._search_scan_yaw0=frame['yaw']
        a._search_scan_started_t=RECORDED['look_started_s']
        a._search_scan_directed_first=a._search_scan_directed_last=None
        a._search_scan_directed_done=False
        a._hold_edge_review=Mock(return_value=False)
        a.grid=sm.GridMap(500,400,.1,(-30.,-40.),bytes(500*400))
        a._search_camera_pose=tuple(frame[k]for k in('sample_s','x','y','yaw','z','down_pitch'))
        a.yaw=frame['yaw']
        self.now=RECORDED['focus_frame']['ros']

    def step(self):
        with patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time.from_sec(self.now)):
            self.a._search_scan_step()
        return self.a._send_vel.call_args.args

    def position(self, distance, pitch=None, altitude=None):
        old=self.a._search_camera_pose
        look=self.a._search_segment['look_points'][0]
        bearing=math.atan2(look[1]-old[2],look[0]-old[1])
        cam=(look[0]-distance*math.cos(bearing),look[1]-distance*math.sin(bearing))
        shift=(cam[0]-old[1],cam[1]-old[2])
        self.a.local_xy=(self.a.local_xy[0]+shift[0],self.a.local_xy[1]+shift[1])
        sample=sa.rospy.Time.from_sec(self.now).to_sec()
        self.a._search_camera_pose=(sample,*cam,bearing,
            old[4]if altitude is None else altitude,old[5]if pitch is None else pitch)
        self.a._pose_sample_s=sample

    def no_credit(self):
        self.assertFalse(self.a._search_scan_done)
        self.assertFalse(self.a._search_scan_directed_done)
        self.assertEqual(self.a._search_scan_seen,set())

    def test_actual_empty_band_too_near_now_requests_bounded_backward(self):
        c=self.a._search_camera_pose
        self.assertIsNone(sa.ground_distance_band(c[4],c[5]))
        neutral=sa.ground_distance_band(c[4],0.)
        self.assertAlmostEqual(neutral[0],RECORDED['expected']['neutral_band'][0])
        self.assertEqual(neutral[1],9.)
        vx,vy=self.step()
        look=self.a._search_segment['look_points'][0]
        self.assertLess(vx*(look[0]-c[1])+vy*(look[1]-c[2]),0.)
        self.assertGreater(math.hypot(vx,vy),0.)
        self.assertLessEqual(math.hypot(vx,vy),.350001)
        self.assertEqual(self.a._search_scan_started_t,RECORDED['look_started_s'])
        self.no_credit()

    def test_too_far_requests_forward_with_same_arrival_and_speed_limits(self):
        self.position(9.5)
        c=self.a._search_camera_pose;look=self.a._search_segment['look_points'][0]
        vx,vy=self.step()
        self.assertGreater(vx*(look[0]-c[1])+vy*(look[1]-c[2]),0.)
        self.assertLessEqual(math.hypot(vx,vy),.350001)
        self.no_credit()

    def test_neutral_valid_transient_pitch_does_not_chase_distance_or_credit(self):
        self.position(8.85)
        self.assertIsNone(sa.ground_distance_band(self.a._search_camera_pose[4],
                                                self.a._search_camera_pose[5]))
        self.assertEqual(self.step(),(0.,0.))
        self.now+=.3;self.position(8.85)
        self.assertEqual(self.step(),(0.,0.))
        self.no_credit()

    def test_nonempty_actual_band_cannot_chase_transient_pitch_at_neutral_valid_range(self):
        # Large downward pitch makes a nonempty actual band too short; neutral
        # range is valid, so do not move closer just to follow this attitude.
        self.position(8.85,pitch=math.radians(55.))
        self.assertIsNotNone(sa.ground_distance_band(self.a._search_camera_pose[4],
                                                   self.a._search_camera_pose[5]))
        self.assertEqual(self.step(),(0.,0.))
        self.no_credit()

    def test_empty_neutral_band_or_stale_or_untrusted_input_stops(self):
        self.position(8.0,altitude=5.0)
        self.assertIsNone(sa.ground_distance_band(5.,0.))
        self.assertEqual(self.step(),(0.,0.))
        self.no_credit()
        self.a._world_trusted=Mock(return_value=False)
        self.assertEqual(self.step(),(0.,0.))
        self.a._world_trusted=Mock(return_value=True)
        self.a._pose_sample_s=self.now-1.
        self.assertEqual(self.step(),(0.,0.))
        self.a._pose_sample_s=sa.rospy.Time.from_sec(self.now).to_sec()
        self.a._search_camera_pose=(self.now-1.,*self.a._search_camera_pose[1:])
        self.assertEqual(self.step(),(0.,0.))
        self.no_credit()

    def test_obstacle_or_outside_original_arrival_neighbourhood_stops(self):
        c=self.a._search_camera_pose;look=self.a._search_segment['look_points'][0]
        desired=sum(sa.ground_distance_band(c[4],0.))*.5
        distance=math.dist(c[1:3],look);own=self.a.world_xy
        corrected=(own[0]+(distance-desired)*(look[0]-c[1])/distance,
                   own[1]+(distance-desired)*(look[1]-c[2])/distance)
        midpoint=((own[0]+corrected[0])*.5,(own[1]+corrected[1])*.5)
        g=self.a.grid;data=bytearray(g.cells);ix,iy=g.world_to_cell(midpoint)
        data[iy*g.width+ix]=1
        self.a.grid=sm.GridMap(g.width,g.height,g.resolution,g.origin,data)
        self.assertEqual(self.step(),(0.,0.))
        self.a.grid=g
        self.a._search_segment['points']=((own[0]+5.,own[1]+5.),)
        self.assertEqual(self.step(),(0.,0.))
        self.no_credit()

    def test_friend_stop_is_preserved_and_correction_does_not_complete_a_visit(self):
        self.a._apply_friend_avoidance=Mock(return_value=(0.,0.))
        self.assertEqual(self.step(),(0.,0.))
        vx,vy=self.a._apply_friend_avoidance.call_args.args
        self.assertLessEqual(math.hypot(vx,vy),.350001)
        self.no_credit()

    def test_blocked_actual_ground_view_never_completes_building_review(self):
        self.position(8.85,pitch=0.)
        self.a._search_observable_points.return_value={}
        self.step()
        self.now+=.3;self.position(8.85,pitch=0.)
        self.step()
        self.assertTrue(self.a._search_scan_directed_done)
        self.assertFalse(self.a._search_scan_done)
        self.assertEqual(self.a._search_scan_seen,set())

    def test_original_eight_second_timeout_cannot_be_renewed_by_correction(self):
        self.step()
        self.now=RECORDED['look_started_s']+8.01
        self.position(8.0)
        self.assertEqual(self.step(),(0.,0.))
        self.assertTrue(self.a._search_scan_failed)
        self.assertEqual(self.a._search_scan_started_t,RECORDED['look_started_s'])
        self.assertEqual(self.a._search_scan_seen,set())


if __name__ == '__main__':
    unittest.main()
