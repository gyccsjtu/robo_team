"""Measured-speed braking, corner constraints and common output regressions."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'coordination/src/robocup_swarm/scripts'))
from motion_safety import braking_cap,boundary_constraints,HeightClearanceHold
import swarm_agent as sa
from geometry_msgs.msg import TwistStamped,PoseStamped


class BrakingTests(unittest.TestCase):
    def test_slow_intent_does_not_hide_fast_measured_approach(self):
        wall=[(1.,0.,4.5,'wall')]
        slow,_=braking_cap((.5,0.),(0.,0.),wall,1.6,.25,.8)
        fast,why=braking_cap((.5,0.),(3.,0.),wall,1.6,.25,.8)
        self.assertEqual(slow,(.5,0.));self.assertEqual(fast,(0.,0.))
        self.assertIn('measured_braking',why)

    def test_longer_sensor_delay_requires_earlier_braking(self):
        c=[(1.,0.,4.7,'wall')]
        self.assertNotEqual(braking_cap((1.,0.),(2.,0.),c,1.6,.1,.8)[0],(0.,0.))
        self.assertEqual(braking_cap((1.,0.),(2.,0.),c,1.6,.5,.8)[0],(0.,0.))

    def test_corner_constraints_apply_together_without_new_direction(self):
        constraints=[(1.,0.,12.,'xwall'),(0.,1.,2.,'ywall')]
        v,_=braking_cap((2.,2.),(0.,2.),constraints,1.6,.25,.8)
        self.assertEqual(v,(0.,0.))
        v,_=braking_cap((-1.,-1.),(0.,0.),constraints,1.6,.25,.8)
        self.assertEqual(v,(-1.,-1.))

    def test_parallel_wall_and_return_inside_boundary_remain_possible(self):
        self.assertEqual(braking_cap((2.,0.),(2.,0.),[(0.,1.,1.7,'side')],1.6,.25,.8)[0],(2.,0.))
        c=boundary_constraints((9.,9.),(-10.,10.,-10.,10.),2.)
        self.assertEqual(braking_cap((-1.,-1.),(0.,0.),c,0.,.25,.8)[0],(-1.,-1.))
        self.assertEqual(braking_cap((1.,1.),(0.,0.),c,0.,.25,.8)[0],(0.,0.))
        self.assertEqual(braking_cap((0.,0.),(3.,3.),c,0.,.25,.8)[0],(0.,0.))


class OutputTests(unittest.TestCase):
    def setUp(self):
        for patcher in (patch.object(sa,'MOTION_SAFETY',True),
                        patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time(100))):
            patcher.start();self.addCleanup(patcher.stop)
        a=self.a=sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id='u0';a.local_xy=(0.,0.);a.offset=(0.,0.);a.local_z=3.;a.yaw=0.
        a._pose_sample_s=100.;a._measured_velocity=(100.,(3.,0.))
        a._takeoff_done=True;a._landing=False
        a._scan_t=100.;a._scan=NS(angle_min=-math.pi,angle_increment=math.pi/180,
            range_min=.1,range_max=20.,ranges=[float('inf')]*361)
        a._scan.ranges[180]=4.5
        a.pos_pub=Mock();a.vel_pub=Mock()

    def test_position_and_velocity_modes_use_same_measured_brake(self):
        a=self.a
        with patch.object(sa,'FLIGHT_OUTPUT','vel'):
            a._flight_output(.5,0.,0.)
        self.assertEqual(a.vel_pub.publish.call_args.args[0].twist.linear.x,0.)
        with patch.object(sa,'FLIGHT_OUTPUT','pos'):
            a._flight_output(.5,0.,0.)
        self.assertEqual(a.pos_pub.publish.call_args.args[0].pose.position.x,0.)

    def test_stale_pose_uses_zero_velocity_not_a_stale_position_target(self):
        a=self.a;a._pose_sample_s=99.
        with patch.object(sa,'FLIGHT_OUTPUT','pos'):
            a._flight_output(2.,0.,.6,.4,4.)
        a.pos_pub.publish.assert_not_called()
        cmd=a.vel_pub.publish.call_args.args[0]
        self.assertEqual((cmd.twist.linear.x,cmd.twist.linear.z,cmd.twist.angular.z),(0.,0.,0.))

    def test_recovery_climb_does_not_bypass_close_obstacle_gate(self):
        a=self.a;a._scan.ranges[180]=.8
        with patch.object(sa,'FLIGHT_OUTPUT','pos'):
            a._flight_output(0.,0.,.6,target_alt=4.)
        self.assertEqual(a.pos_pub.publish.call_args.args[0].pose.position.z,3.)

    def test_stale_velocity_cannot_authorize_horizontal_motion(self):
        a=self.a;a._measured_velocity=(99.,(0.,0.))
        self.assertEqual(a._output_motion_safety(2.,0.,0.,0.)[:2],(0.,0.))

    def test_old_or_nonfinite_velocity_cannot_overwrite_current_sample(self):
        a=self.a;m=TwistStamped();m.header.stamp=sa.rospy.Time(99)
        a._measured_velocity_cb(m);self.assertEqual(a._measured_velocity,(100.,(3.,0.)))
        m.header.stamp=sa.rospy.Time.from_sec(100.01);m.twist.linear.x=float('nan')
        a._measured_velocity_cb(m);self.assertEqual(a._measured_velocity,(100.,(3.,0.)))

    def test_old_image_pose_cannot_refresh_aircraft_pose_age(self):
        a=self.a;m=PoseStamped();m.header.stamp=sa.rospy.Time(99);m.pose.orientation.w=1.
        m.pose.position.x=123.
        a._local_cb(m)
        self.assertEqual(a.local_xy,(0.,0.));self.assertEqual(a._pose_sample_s,100.)


class HeightHoldTests(unittest.TestCase):
    def test_vanished_return_cannot_release_hold_without_horizontal_escape(self):
        h=HeightClearanceHold()
        self.assertTrue(h.update(100.,(0.,0.),True,False,1.6))
        for t in (100.1,101.,110.):
            self.assertTrue(h.update(t,(.5,0.),False,True,1.6))
        self.assertTrue(h.update(111.,(2.,0.),False,True,1.6))
        self.assertTrue(h.update(111.,(2.,0.),False,True,1.6))
        self.assertTrue(h.update(111.4,(2.,0.),False,True,1.6))
        self.assertFalse(h.update(111.6,(2.,0.),False,True,1.6))

    def test_nearby_new_obstacle_restarts_clearance_but_never_moves_aircraft(self):
        h=HeightClearanceHold()
        h.update(100.,(0.,0.),True,False,1.6)
        h.update(101.,(2.,0.),False,True,1.6)
        self.assertTrue(h.update(101.4,(2.,0.),True,False,1.6))
        self.assertTrue(h.update(110.,(2.,0.),False,True,1.6))

    def test_actual_height_goal_no_longer_drops_inside_inflation(self):
        a=sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.local_xy=a.offset=(0.,0.);a.local_z=2.85;a.altitude_layer=2.8
        a._alt_sag=.1;a._orbit_target=None;a.los=True
        a.grid=NS(world_to_cell=lambda p:(2,2),is_free=lambda c:False,
                  origin=(-10.,-10.),width=40,height=40,resolution=.5)
        with patch.object(sa,'MOTION_SAFETY',True):
            goal=a.altitude_layer+a._compute_desired_altitude()-a._alt_sag
        self.assertAlmostEqual(goal,a.local_z)

    def test_actual_guard_remembers_near_scan_after_height_plane_becomes_clear(self):
        a=sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.local_xy=a.offset=(0.,0.);a.local_z=2.85;a.uav_id='u5'
        a._takeoff_done=True;a._landing=False;a._scan_t=100.
        a._scan=NS(angle_min=0.,angle_increment=1.,range_min=.1,range_max=20.,ranges=[1.65])
        with patch.object(sa,'MOTION_SAFETY',True),patch.object(sa,'RADAR_GUARD',True):
            with patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time(100)):
                self.assertEqual(a._guard_vertical_clearance(-.2),0.)
            a._scan.ranges=[5.2];a.local_xy=(.5,0.);a._scan_t=101.
            with patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time(101)):
                self.assertEqual(a._guard_vertical_clearance(-.5),0.)
                self.assertEqual(a._guard_vertical_clearance(.03),0.)
            a.local_xy=(2.,0.)
            for t in (102.,102.6):
                a._scan_t=sa.rospy.Time.from_sec(t).to_sec()
                with patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time.from_sec(t)):
                    value=a._guard_vertical_clearance(.3)
            self.assertEqual(value,.3)


if __name__=='__main__':unittest.main()
