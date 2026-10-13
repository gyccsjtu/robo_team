"""Recorded blue avoidance conflict, joint constraints, and retained hard stops."""
import math,os,random,unittest
from unittest.mock import patch
from types import SimpleNamespace as NS
import test_view_motion_envelope as fixtures
sa=fixtures.sa
from friend_motion import project_no_closing
from slam_grid import SlamGrid

class FriendProjectionTests(unittest.TestCase):
    def test_recorded_blue_radar_direction_can_be_made_nonclosing_without_speedup(self):
        n=(2.5304183959960938,4.109279632568359)
        raw=(1.23,-.36);v=project_no_closing(raw,[n])
        self.assertGreater(math.hypot(*v),1.)
        self.assertLessEqual(v[0]*n[0]+v[1]*n[1],1e-8)
        self.assertLessEqual(math.hypot(*v),math.hypot(*raw))
        self.assertAlmostEqual(v[0],1.0525632037559176)

    def test_zero_invalid_and_opposing_constraints_never_create_motion(self):
        self.assertEqual(project_no_closing((0.,0.),[(1.,0.)]),(0.,0.))
        self.assertEqual(project_no_closing((math.nan,0.),[]),(0.,0.))
        self.assertEqual(project_no_closing((1.,0.),[(0.,0.)]),(0.,0.))
        self.assertEqual(project_no_closing((1.,1.),[(1.,0.),(0.,1.),(-1.,0.),(0.,-1.)]),(0.,0.))
        self.assertEqual(project_no_closing((1.,1.),[(1.,0.),(-1.,0.)]),(0.,1.))

    def test_random_joint_constraints_all_hold_and_total_speed_cannot_increase(self):
        rng=random.Random(17)
        for _ in range(1000):
            raw=(rng.uniform(-6.,6.),rng.uniform(-6.,6.))
            angles=[rng.uniform(-math.pi,math.pi) for _ in range(rng.randint(0,5))]
            normals=[(math.cos(a),math.sin(a)) for a in angles]
            v=project_no_closing(raw,normals)
            self.assertLessEqual(math.hypot(*v),math.hypot(*raw)+1e-9)
            self.assertTrue(all(v[0]*x+v[1]*y<=1e-9 for x,y in normals))

class FriendBufferIntegrationTests(unittest.TestCase):
    setUp=fixtures.ViewMotionEnvelopeTests.setUp
    agent=fixtures.ViewMotionEnvelopeTests.agent
    view=fixtures.ViewMotionEnvelopeTests.view

    def recorded(self):
        a=self.agent();a.local_xy=(86.43343353271484,5.594844818115234);a.local_z=3.0400049686431885
        a.assignment=NS(task_type=1,target_id='t1');a.targets={'t1':(96.,5.,1.,0.)}
        a._friend_positions={'u3':(88.96385192871094,9.704124450683594,3.9386699199676514)}
        a._friend_seen={'u3':self.now}
        a._tracking_image=None;a._last_cmd_v=(1.23,-.36)
        a._pose_sample_s=self.now;a._measured_velocity=(self.now,(0.,0.))
        return a

    def enabled(self):return patch.dict(os.environ,TRACKING_FRIEND_RECHECK='1')

    def test_real_friend_gate_passes_projection_and_blocks_original_radar_direction(self):
        a=self.recorded()
        self.assertEqual(a._final_motion_gate(1.23,-.36),(0.,0.))
        with self.enabled():v=a._buffer_friend_velocity(1.23,-.36)
        self.assertEqual(a._final_motion_gate(*v),v)

    def test_actual_position_output_respects_all_guards_after_recheck(self):
        a=self.recorded();a._radar_guard_velocity=lambda x,y:(1.23,-.36)
        with self.enabled(),patch.object(sa,'MOTION_SAFETY',True),patch.object(sa,'FLIGHT_OUTPUT','pos'):
            a._send_vel(1.38,-.85)
        self.assertGreater(math.hypot(*a._last_flight_v),1.)
        p=a.pos_pub.publish.call_args.args[0].pose.position
        self.assertAlmostEqual(p.x-a.local_xy[0],a._last_flight_v[0]*sa.POS_SP_LEAD)
        self.assertLessEqual(a._last_flight_v[0]*2.5304183959960938+a._last_flight_v[1]*4.109279632568359,1e-8)

    def test_laser_map_and_explicit_stop_are_not_revived_by_recheck(self):
        for guard in ('_radar_guard_velocity','_grid_guard_velocity','_map_guard_velocity','_final_scan_cap','request'):
            with self.subTest(guard=guard):
                a=self.recorded()
                if guard!='request':setattr(a,guard,lambda x,y:(0.,0.))
                requested=(0.,0.) if guard=='request' else (1.23,-.36)
                with self.enabled(),patch.object(sa,'MOTION_SAFETY',True),patch.object(sa,'FLIGHT_OUTPUT','pos'):a._send_vel(*requested)
                self.assertEqual(a._last_flight_v,(0.,0.))

    def test_projected_direction_still_stops_at_measured_obstacle(self):
        a=self.recorded();a._final_scan_cap=sa.SwarmAgent._final_scan_cap.__get__(a)
        a._scan.ranges[:]=[.8]*361
        with self.enabled(),patch.object(sa,'RADAR_GUARD',True):v=a._buffer_friend_velocity(1.23,-.36)
        self.assertEqual(a._final_motion_gate(*v),(0.,0.))
        a._scan.ranges[:]=[20.]*361;a._scan_t=self.now-1.
        self.assertEqual(a._final_motion_gate(*v),(0.,0.))

    def test_projected_direction_is_rejected_by_the_actual_occupied_grid(self):
        a=self.recorded()
        a.grid=SlamGrid(80.,0.,100.,15.,.1,0.)
        with self.enabled():v=a._buffer_friend_velocity(1.23,-.36)
        self.assertEqual(a._final_motion_gate(*v),v)
        a.grid.mark_point(a.local_xy[0]+v[0]*.25,a.local_xy[1]+v[1]*.25)
        self.assertEqual(a._final_motion_gate(*v),(0.,0.))

    def test_stale_position_and_velocity_still_stop_executed_motion(self):
        for stale in ('pose','velocity'):
            a=self.recorded();a._radar_guard_velocity=lambda x,y:(1.23,-.36)
            if stale=='pose':a._pose_sample_s=self.now-1.
            else:a._measured_velocity=(self.now-1.,(0.,0.))
            with self.subTest(stale=stale),self.enabled(),patch.object(sa,'MOTION_SAFETY',True),patch.object(sa,'FLIGHT_OUTPUT','pos'):
                a._send_vel(1.38,-.85)
            self.assertEqual(a._last_flight_v,(0.,0.))

    def test_inner_repulsion_search_retreat_and_disabled_mode_keep_old_intent(self):
        raw=(1.23,-.36)
        a=self.recorded()
        with patch.dict(os.environ,TRACKING_FRIEND_RECHECK='0'):self.assertEqual(a._buffer_friend_velocity(*raw),raw)
        for mode in ('inner','search','retreat','takeoff'):
            a=self.recorded()
            if mode=='inner':a._friend_positions={'u3':(a.local_xy[0]+3.,a.local_xy[1],a.local_z)}
            if mode=='search':a.assignment.task_type=0
            if mode=='retreat':a._radar_retreating=True
            if mode=='takeoff':a._takeoff_done=False
            with self.subTest(mode=mode),self.enabled():self.assertEqual(a._buffer_friend_velocity(*raw),raw)

    def test_old_friend_and_vertical_separation_do_not_create_false_constraint(self):
        a=self.recorded();a._friend_seen['u3']=self.now-1.001
        with self.enabled():self.assertEqual(a._buffer_friend_velocity(1.23,-.36),(1.23,-.36))
        a._friend_seen['u3']=self.now;a._friend_positions['u3']=(a.local_xy[0]+1.,a.local_xy[1],a.local_z+6.)
        with self.enabled():self.assertEqual(a._buffer_friend_velocity(1.23,-.36),(1.23,-.36))

if __name__=='__main__':unittest.main()
