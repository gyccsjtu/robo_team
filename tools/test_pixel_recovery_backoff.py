"""A cropped foot at the height floor needs a bounded, safely checked retreat."""
import math
import unittest
from unittest.mock import patch
import test_view_motion_envelope as fixtures
from camera_attention import pixel_backoff

sa = fixtures.sa


class PixelBackoffTests(unittest.TestCase):
    def view(self, **changes):
        view = dict(sample_s=100., position_sample_s=99., bearing=0., camera_tilt=0.,
                    box=(340., 290., 375., 480.), width=752., height=480.,
                    image_recovery=True, geometry_anchor_ok=True,
                    geometry_ok=False, report_ok=False, recovery_origin=(50.,20.))
        view.update(changes)
        return view

    def scan(self, value=20.):
        from types import SimpleNamespace
        return SimpleNamespace(angle_min=-math.pi, angle_increment=math.pi/180.,
            range_min=.1, range_max=20., ranges=[value]*361)

    def decide(self, view=None, now=100., xy=(50.,20.), scan=None, scan_stamp=100., yaw=0.):
        return pixel_backoff(view or self.view(), now, yaw, xy,
                             scan or self.scan(), scan_stamp)

    def test_crop_requests_backward_motion_with_no_depth_or_reporting(self):
        view=self.view()
        before=dict(view)
        vx,vy,reason=self.decide(view)
        self.assertAlmostEqual(vx,-.6)
        self.assertAlmostEqual(vy,0.)
        self.assertEqual(reason,'cropped_backoff')
        self.assertEqual(view,before)

    def test_motion_follows_world_bearing_with_body_relative_scan(self):
        view=self.view(bearing=math.pi/2.)
        vx,vy,_=self.decide(view,yaw=math.pi/2.)
        self.assertAlmostEqual(vx,0.)
        self.assertAlmostEqual(vy,-.6)

    def test_rear_obstacle_invalid_or_missing_rays_stop(self):
        for raw in (.8,math.nan,-math.inf,-1.,25.):
            with self.subTest(raw=raw):
                self.assertEqual(self.decide(scan=self.scan(raw))[:2],(0.,0.))
        self.assertLess(self.decide(scan=self.scan(math.inf))[0],0.)
        scan=self.scan();scan.ranges=[20.]*180  # no rear rays
        self.assertEqual(self.decide(scan=scan)[:2],(0.,0.))

    def test_front_obstacle_does_not_invalidate_a_clear_rear_retreat(self):
        scan=self.scan();scan.ranges[180]=.5
        self.assertLess(self.decide(scan=scan)[0],0.)

    def test_travel_budget_and_deadline_are_not_renewed_by_fresh_images(self):
        self.assertEqual(self.decide(xy=(49.04,20.))[:2],(0.,0.))
        self.assertAlmostEqual(self.decide(xy=(49.1,20.))[0],-.1)
        self.assertEqual(self.decide(self.view(sample_s=105.),now=105.,scan_stamp=105.)[:2],(0.,0.))

    def test_stale_unbound_healthy_unaligned_or_tilted_view_cannot_request_retreat(self):
        for changes in (dict(sample_s=99.64),dict(position_sample_s=99.6),
                dict(geometry_anchor_ok=False),dict(image_recovery=False),
                dict(geometry_ok=True),dict(report_ok=True),dict(bearing=.4),
                dict(camera_tilt=.2),dict(box=(340.,290.,375.,450.)),
                dict(recovery_origin=None),dict(bearing=math.nan)):
            with self.subTest(changes=changes):
                self.assertEqual(self.decide(self.view(**changes))[:2],(0.,0.))
        self.assertEqual(self.decide(scan_stamp=99.69)[:2],(0.,0.))


class PixelBackoffControlTests(unittest.TestCase):
    setUp=fixtures.ViewMotionEnvelopeTests.setUp
    agent=fixtures.ViewMotionEnvelopeTests.agent
    view=fixtures.ViewMotionEnvelopeTests.view

    def cropped(self):
        agent=self.agent()
        agent.local_z=agent.altitude_layer=2.6
        agent._pose_sample_s=self.now
        agent._measured_velocity=(self.now,(0.,0.))
        agent._tracking_image.update(PixelBackoffTests().view())
        agent._tracking_image.update(pixel_only=True,foot_down_sin=.6,
            recovery_until=105.,altitude_goal=2.6)
        return agent

    def run_recovery(self, agent, enabled=True):
        with patch.object(sa,'PIXEL_VIEW_RECOVERY',True),\
             patch.object(sa,'PIXEL_RECOVERY_APPROACH',True),\
             patch.object(sa,'PIXEL_RECOVERY_BACKOFF',enabled),\
             patch.object(sa,'MOTION_SAFETY',True),\
             patch.object(sa,'FLIGHT_OUTPUT','pos'):
            return agent._hold_pixel_recovery(self.now)

    def test_real_height_floor_counterexample_and_final_position_output(self):
        old=self.cropped()
        self.assertTrue(self.run_recovery(old,False))
        self.assertEqual(old._last_flight_v,(0.,0.))
        agent=self.cropped()
        self.assertTrue(self.run_recovery(agent))
        self.assertLess(agent._last_flight_v[0],0.)
        position=agent.pos_pub.publish.call_args.args[0].pose.position
        self.assertLess(position.x,agent.local_xy[0])
        self.assertAlmostEqual(position.z,2.6)
        self.assertFalse(agent._tracking_image['report_ok'])

    def test_hard_stops_and_stale_pose_are_not_overridden(self):
        for guard in ('_radar_guard_velocity','_grid_guard_velocity',
                      '_map_guard_velocity','_final_scan_cap','stale_pose'):
            with self.subTest(guard=guard):
                agent=self.cropped()
                if guard=='stale_pose':agent._pose_sample_s=self.now-1.
                else:setattr(agent,guard,lambda x,y:(0.,0.))
                self.run_recovery(agent)
                self.assertEqual(agent._last_flight_v,(0.,0.))

    def test_other_task_and_expired_recovery_do_not_request_motion(self):
        for change in ('other_target','search','expired'):
            with self.subTest(change=change):
                agent=self.cropped()
                if change=='other_target':agent.assignment.target_id='t3'
                if change=='search':agent.assignment.task_type=0
                if change=='expired':agent._tracking_image['recovery_until']=self.now
                self.assertFalse(self.run_recovery(agent))
                agent.pos_pub.publish.assert_not_called()


if __name__=='__main__':
    unittest.main()
