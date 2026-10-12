"""Replay widening view geometry without assuming map, colour, or actor truth."""
import math
import unittest
from unittest.mock import patch
import test_tracking_plan as fixture

sa = fixture.swarm_agent


class EarlyViewPursuitTests(unittest.TestCase):
    def replay(self, enabled=True, angle=0., distance=8.3, away=.8,
               age=.1, pixel=False, cropped=False):
        ux, uy = math.cos(angle), math.sin(angle)
        wx, wy = 50.+.2*ux, 20.+.2*uy
        tx, ty = wx+distance*ux, wy+distance*uy
        agent, commands = fixture.TrackingPlanTests().make_agent(
            x=wx, y=wy, tx=tx, ty=ty, target_vx=ux, target_vy=uy)
        agent.local_z = 3.3
        agent.grid = fixture.SimpleNamespace(origin=(-50., -60.),
                                              width=420, height=260, resolution=.5)
        agent._landing = False
        agent._confirm_close, agent._confirm_close_tid = True, 't0'
        agent._confirm_close_until = 110.
        agent._close_range_ref = ('t0', 99.2, distance-away+.2,
                                  tx-away*ux, ty-away*uy, 50., 20.)
        agent._tracking_image = dict(target_id='t0', sample_s=100.-age,
            position_sample_s=100.-age, geometry_ok=True, report_ok=not cropped,
            pixel_only=pixel, needs_space=False, height=480.,
            box=(300., 280., 340., 480. if cropped else 420.))
        agent._visual_observations['t0'] = dict(sample_s=100.-age, eligible=True)
        agent._spend_mode = set()
        agent._reobserve_goal = lambda *a: None
        agent._obstructed_view_goal = lambda *a, **kw: None
        with patch.object(sa, 'EARLY_VIEW_PURSUIT', enabled, create=True), \
             patch.object(sa.rospy.Time, 'now', return_value=sa.rospy.Time(100.)):
            agent._fly_orbit()
        return agent, commands[-1]

    def test_walking_actor_gets_translation_before_leaving_observation_ring(self):
        for angle in (0., .7, 1.8, 3.2, 5.):
            with self.subTest(angle=angle):
                agent, velocity = self.replay(angle=angle)
                self.assertIn('t0', agent._spend_mode)
                along = velocity[0]*math.cos(angle)+velocity[1]*math.sin(angle)
                self.assertGreater(along, 1.)
                self.assertLessEqual(math.hypot(*velocity), sa.FLEE_SHEPHERD_SPEED)
                self.assertTrue(agent._confirm_close)

    def test_old_mode_stale_pixels_cropping_and_close_range_cannot_trigger(self):
        for change in [dict(enabled=False), dict(age=.6), dict(pixel=True),
                       dict(cropped=True), dict(distance=6.8), dict(away=0.)]:
            with self.subTest(change=change):
                agent, _ = self.replay(**change)
                self.assertNotIn('t0', agent._spend_mode)


if __name__ == '__main__':
    unittest.main()
