"""Regression for UAV4 descending beside 0.60--0.76 m scaffold returns."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa
from slam_grid import SlamGrid


class ObstacleHeightSafetyTests(unittest.TestCase):
    def setUp(self):
        clock = patch.object(sa.rospy.Time, 'now', return_value=sa.rospy.Time(100))
        clock.start()
        self.addCleanup(clock.stop)
        enabled = patch.object(sa, 'RADAR_GUARD', True)
        enabled.start()
        self.addCleanup(enabled.stop)

    def agent(self, distance=20., bearing=0.):
        a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id='u4'
        a.local_xy=a.offset=(0.,0.)
        a.local_z=4.
        a.yaw=0.
        a._takeoff_done=True
        a._landing=False
        a._scan_t=100.
        a._scan=NS(angle_min=-math.pi, angle_increment=math.pi/180,
                   range_min=.1, range_max=20., ranges=[20.]*361)
        a._scan.ranges[round(math.degrees(bearing))+180]=distance
        return a

    def test_scaffold_horizontal_stop_also_blocks_routine_descent(self):
        for distance in (.76, .67, .60, .40):
            a=self.agent(distance)
            self.assertEqual(a._final_scan_cap(1.,0.),(0.,0.))
            self.assertEqual(a._guard_vertical_clearance(-1.),0.)
            self.assertEqual(a._guard_vertical_clearance(.6),0.)

    def test_close_valid_return_is_not_erased_by_mapping_echo_floor(self):
        a=self.agent(.30,math.pi/2)
        self.assertEqual(a._final_scan_cap(0.,1.),(0.,0.))

    def test_clear_space_allows_view_height_control(self):
        a=self.agent()
        self.assertEqual(a._guard_vertical_clearance(-.3),-.3)
        self.assertEqual(a._guard_vertical_clearance(.4),.4)

    def test_invalid_scan_is_not_clear_space_and_future_scan_cannot_authorize_motion(self):
        a=self.agent()
        a._scan.ranges[:]=[float('nan')]*361
        self.assertEqual(a._guard_vertical_clearance(-.3),0.)
        a._scan.ranges[:]=[float('inf')]*361
        self.assertEqual(a._guard_vertical_clearance(-.3),-.3)
        a._scan_t=101.
        self.assertEqual(a._final_scan_cap(1.,0.),(0.,0.))
        self.assertEqual(a._radar_guard_velocity(1.,0.),(0.,0.))

    def test_rear_airframe_echo_does_not_block_routine_height(self):
        a=self.agent(.5,math.radians(120))
        self.assertEqual(a._guard_vertical_clearance(-.3),-.3)

    def test_stale_or_missing_scan_cannot_authorize_normal_height_change(self):
        a=self.agent()
        for scan,t in ((a._scan,99.),(None,100.),(a._scan,101.)):
            a._scan,a._scan_t=scan,t
            self.assertEqual(a._guard_vertical_clearance(-1.),0.)

    def test_takeoff_and_landing_use_their_existing_control(self):
        a=self.agent(.6)
        a._takeoff_done=False
        self.assertEqual(a._guard_vertical_clearance(.5),.5)
        a._takeoff_done=True
        a._landing=True
        self.assertEqual(a._guard_vertical_clearance(-1.),-1.)

    def output_agent(self):
        a=self.agent(.6)
        a.altitude_layer=3.5
        a._orbit_target=a._look_at=None
        a._confirm_close=False
        a._est_trustworthy=lambda:True
        a._bounds_recovery_velocity=lambda:None
        a._radar_guard_velocity=lambda vx,vy:(0.,0.)
        a._grid_guard_velocity=a._map_guard_velocity=lambda vx,vy:(vx,vy)
        a._compute_desired_altitude=lambda:0.
        a._last_cmd_v=(0.,0.)
        a._last_csv_t=100.
        a._climb_latch=False
        a._climb_t0=None
        a._alt_sag=0.
        a.pos_pub=Mock()
        return a

    def test_actual_position_output_holds_height_beside_scaffold(self):
        a=self.output_agent()
        with patch.object(sa,'FLIGHT_OUTPUT','pos'),patch.object(sa,'CRASH_DETECT',False):
            a._send_vel(1.,0.)
        point=a.pos_pub.publish.call_args.args[0].pose.position
        self.assertEqual((point.x,point.y,point.z),(0.,0.,a.local_z))

    def test_near_obstacle_height_hold_cannot_override_hard_ceiling_descent(self):
        a=self.output_agent()
        a.local_z=4.3
        with patch.object(sa,'FLIGHT_OUTPUT','pos'),patch.object(sa,'CRASH_DETECT',False), \
             patch.object(sa,'ALT_HARD_CEIL',4.2):
            a._send_vel(0.,0.)
        point=a.pos_pub.publish.call_args.args[0].pose.position
        self.assertLess(point.z,a.local_z)

    def grid(self):
        g=SlamGrid(-5.,-5.,5.,5.,.25,.75)
        return g

    def test_inflated_own_cell_is_not_an_unconditional_forward_exemption(self):
        a=self.agent(.6)
        a.grid=self.grid()
        # A wall east of us inflates into the current cell. Exit west or sideways.
        for y in (-.5,-.25,0.,.25,.5):
            cell=a.grid.world_to_cell((.5,y))
            a.grid.cells[cell[1]*a.grid.width+cell[0]]=1
        for y in (-1.,-.5,0.,.5,1.):
            for x in (-.25,0.,.25,.5,.75):
                cell=a.grid.world_to_cell((x,y))
                a.grid.infl_cells[cell[1]*a.grid.width+cell[0]]=1
        self.assertFalse(a.grid.is_free(a.grid.world_to_cell((0.,0.))))
        v=a._occupied_cell_exit(3.,0.)
        self.assertLessEqual(math.hypot(*v),.5+1e-9)
        self.assertLessEqual(v[0],.01)

    def test_exit_cannot_revive_stop_or_increase_requested_speed(self):
        a=self.agent()
        a.grid=self.grid()
        cell=a.grid.world_to_cell((0.,0.))
        a.grid.infl_cells[cell[1]*a.grid.width+cell[0]]=1
        self.assertEqual(a._occupied_cell_exit(0.,0.),(0.,0.))
        self.assertLessEqual(math.hypot(*a._occupied_cell_exit(.1,0.)),.1+1e-9)

    def test_no_raw_free_escape_route_means_stop(self):
        a=self.agent()
        a.grid=self.grid()
        a.grid.cells[:]=bytes([1])*len(a.grid.cells)
        a.grid.infl_cells[:]=bytes([1])*len(a.grid.infl_cells)
        self.assertEqual(a._occupied_cell_exit(3.,0.),(0.,0.))

    def test_uav2_recorded_small_command_is_not_amplified_by_backoff(self):
        a=self.agent()
        a.yaw=math.atan2(-.04,-.12)
        for i in range(361):
            degrees=i-180
            a._scan.ranges[i]=.62 if abs(degrees)<=30 else (3.68 if degrees>30 else .50)
        v=a._radar_guard_velocity(-.12,-.04)
        self.assertLessEqual(math.hypot(*v),math.hypot(.12,.04)+1e-9)
        self.assertLessEqual(math.hypot(*a._final_scan_cap(*v)),math.hypot(.12,.04)+1e-9)

    def test_three_sided_escape_cannot_revive_a_stop_or_bypass_acceleration(self):
        a=self.agent()
        for i in range(361):
            degrees=i-180
            a._scan.ranges[i]=.55 if abs(degrees)>30 else .62
        self.assertEqual(a._radar_guard_velocity(0.,0.),(0.,0.))
        a._radar_retreating=False
        v=a._radar_guard_velocity(.1,0.)
        self.assertLessEqual(math.hypot(*v),.1+1e-9)
        self.assertFalse(a._radar_retreating)

    def test_clear_forward_command_preserves_requested_speed(self):
        a=self.agent()
        self.assertEqual(a._radar_guard_velocity(3.,0.),(3.,0.))


if __name__=='__main__':unittest.main()
