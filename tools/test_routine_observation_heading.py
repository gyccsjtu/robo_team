"""Actual-ray search camera admission; no position or flight-control output."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'coordination/src/robocup_swarm/scripts'))
from routine_observation_heading import routine_observation_heading


class RoutineObservationHeadingTests(unittest.TestCase):
    def inputs(self, **changes):
        scan = NS(angle_min=math.radians(-135.), angle_increment=math.radians(1.),
                  range_min=.5, range_max=20., ranges=[12.]*271)
        data = dict(route_heading=0.,own_xy=(20.,-50.),look_xy=(20.,-42.),
            actor_bounds=(-50.,150.,-60.,60.),purpose='south_patrol',task_type=0,
            own_yaw=0.,own_sample_s=100.,camera_pose=(100.,20.,-50.,0.,3.,0.),
            scan=scan,scan_sample_s=100.,now=100.1,velocity=(3.,0.),
            measured_velocity=(100.,(3.,0.)))
        data.update(changes)
        return data

    def result(self, **changes):
        return routine_observation_heading(**self.inputs(**changes))

    def assert_fallback(self, data):
        heading,reason = routine_observation_heading(**data)
        self.assertEqual(heading,data['route_heading'])
        self.assertNotEqual(reason,'measured_routine_look')

    def test_supported_inward_look_is_90_degrees_and_never_105(self):
        heading,reason = self.result()
        self.assertAlmostEqual(heading,math.pi/2.)
        self.assertEqual(reason,'measured_routine_look')
        heading,reason = self.result(look_xy=(12.,-48.))
        self.assertAlmostEqual(heading,math.pi/2.)  # Behind-side is limited.
        heading,reason = self.result(look_xy=(20.,-58.))
        self.assertAlmostEqual(heading,-math.pi/2.)

    def test_camera_positions_and_yaw_are_measurements_not_commands(self):
        for change in (dict(camera_pose=None),dict(camera_pose=(99.5,20.,-50.,0.,3.,0.)),
                dict(own_sample_s=99.8),dict(scan_sample_s=99.8),
                dict(measured_velocity=(99.8,(0.,0.))),dict(own_sample_s=100.2),
                dict(camera_pose=(100.,24.,-50.,0.,3.,0.)),
                dict(camera_pose=(100.,20.,-50.,.5,3.,0.)),
                dict(camera_pose=(100.,20.,-50.,0.,0.,0.))):
            with self.subTest(change=change):self.assert_fallback(self.inputs(**change))

    def test_person_priority_other_purposes_and_outside_look_are_unchanged(self):
        for change in (dict(task_type=1),dict(attention_active=True),dict(purpose='coverage'),
                       dict(purpose='building_review'),dict(look_xy=(151.,-50.)),
                       dict(look_xy=(20.,-61.))):
            with self.subTest(change=change):self.assert_fallback(self.inputs(**change))

    def test_current_and_proposed_sensor_cones_need_actual_support(self):
        for change in (dict(own_yaw=math.pi,camera_pose=(100.,20.,-50.,math.pi,3.,0.)),
                       dict(scan=NS(angle_min=math.radians(-90.),
                            angle_increment=math.radians(1.),range_min=.5,
                            range_max=20.,ranges=[12.]*181))):
            with self.subTest(change=change):self.assert_fallback(self.inputs(**change))
        for body_angle in (0.,-90.):
            data=self.inputs()
            # A real close ray in either cone vetoes the camera change; the
            # helper cannot rely only on the manufacturer saying '270 deg'.
            data['scan'].ranges[135+int(body_angle)]=1.8
            self.assert_fallback(data)

    def test_three_finite_rays_must_cover_centre_and_both_sides(self):
        data=self.inputs()
        data['scan'].ranges=[math.inf]*271
        self.assert_fallback(data)
        for angle in (-105,-90,-75,-15,0,15):
            data['scan'].ranges[135+angle]=12.
        self.assertEqual(routine_observation_heading(**data)[1],'measured_routine_look')
        data['scan'].ranges[135-105]=math.inf
        self.assert_fallback(data)
        data=self.inputs()
        for angle in range(-10,11):data['scan'].ranges[135+angle]=float('nan')
        self.assert_fallback(data)

    def test_measured_speed_and_delayed_braking_are_not_hidden_by_slow_intent(self):
        data=self.inputs(velocity=(.5,0.),measured_velocity=(100.,(3.,0.)))
        data['scan'].ranges=[8.]*271  # Less than 9.125m + cone projection.
        self.assert_fallback(data)
        data['measured_velocity']=(100.,(.5,0.))
        self.assertEqual(routine_observation_heading(**data)[1],'measured_routine_look')
        data=self.inputs(brake_delay=1.)
        data['scan'].ranges=[10.]*271
        self.assert_fallback(data)

    def test_nonfinite_bad_scan_and_braking_inputs_preserve_route(self):
        for change in (dict(now=float('nan')),dict(velocity=(float('inf'),0.)),
                dict(actor_bounds=(1.,1.,-1.,1.)),dict(decel=2.),dict(decel=0.),
                dict(brake_delay=-1.),dict(margin=-1.),dict(velocity=(0.,0.)),
                dict(look_xy=(20.,-50.))):
            with self.subTest(change=change):self.assert_fallback(self.inputs(**change))
        for field,value in (('angle_increment',0.),('angle_min',float('nan')),
                            ('ranges',[]),('range_min',21.),('ranges',[12.]*400)):
            data=self.inputs();setattr(data['scan'],field,value)
            with self.subTest(field=field,value=value):self.assert_fallback(data)

    def test_wraparound_is_bounded_against_actual_translation(self):
        route=-math.pi+.02
        data=self.inputs(route_heading=route,own_yaw=route,
            velocity=(3.*math.cos(route),3.*math.sin(route)),
            measured_velocity=(100.,(3.*math.cos(route),3.*math.sin(route))),
            camera_pose=(100.,20.,-50.,route,3.,0.),look_xy=(12.,-49.9))
        heading,reason=routine_observation_heading(**data)
        self.assertEqual(reason,'measured_routine_look')
        self.assertLess(abs(math.atan2(math.sin(heading-route),math.cos(heading-route))),.04)

    def test_delayed_4_and_5hz_camera_does_not_alternate_look_and_route(self):
        for hz in (4.,5.):
            for latency in (.1,.2):
                with self.subTest(hz=hz,latency=latency):
                    headings=[]
                    for step in range(1,81):
                        now=100.+step*.05
                        sample=100.+math.floor((now-100.-latency+1e-9)*hz)/hz
                        yaw=min(math.pi/2.,(now-100.)*.9)
                        camera_yaw=max(0.,min(math.pi/2.,(sample-100.)*.9))
                        data=self.inputs(now=now,own_sample_s=now,scan_sample_s=now,
                            own_xy=(20.+3.*(now-100.),-50.),own_yaw=yaw,
                            camera_pose=(sample,20.+3.*(sample-100.),-50.,camera_yaw,3.,0.),
                            measured_velocity=(now,(3.,0.)))
                        heading,reason=routine_observation_heading(**data)
                        self.assertEqual(reason,'measured_routine_look')
                        headings.append(heading)
                    self.assertTrue(all(abs(h-math.pi/2.)<1e-9 for h in headings))

    def test_a_duplicate_image_cannot_renew_its_age_or_blind_scan(self):
        data=self.inputs(now=100.4,own_sample_s=100.4,scan_sample_s=100.4,
            own_yaw=.4,camera_pose=(100.,20.,-50.,0.,3.,0.),
            measured_velocity=(100.4,(0.,0.)))
        self.assertEqual(routine_observation_heading(**data)[1],'measured_routine_look')
        data.update(now=100.51,own_sample_s=100.51,scan_sample_s=100.51,
                    measured_velocity=(100.51,(0.,0.)))
        self.assert_fallback(data)
        data.update(now=100.4,own_sample_s=100.4,scan_sample_s=100.,
                    measured_velocity=(100.4,(0.,0.)))
        self.assert_fallback(data)


if __name__=='__main__':unittest.main()
