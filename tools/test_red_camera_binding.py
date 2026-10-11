"""Red camera feedback must follow the bridge's same-image identity mapping."""
import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                     'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa
import yolo_target_bridge as bridge


class RedCameraBindingTests(unittest.TestCase):
    def setUp(self):
        clock = patch.object(sa.rospy.Time, 'now',
                             return_value=sa.rospy.Time.from_sec(100.1))
        clock.start()
        self.addCleanup(clock.stop)

    def agent(self):
        a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id = 'u0'
        a.assignment = NS(task_type=1, target_id='t5')
        a.local_xy, a.offset, a.local_z = (0., 0.), (0., 0.), 3.
        a.targets = {'t5': (8., 0., 0., 0.), 't4': (8., .65, 0., 0.)}
        a._visual_observations = {}
        a._team_tracks = {'t5': {'u0', 'u1'}}
        a.detect_pub = Mock()
        a._confirm_start = a._last_confirm_t = 0.
        return a

    def view(self, **change):
        data = dict(source_uav='u0', target_id='t4', real_detection=True,
                    sample_s=100., track_id=7, candidate_id='red:7', hits=10,
                    confidence=.9, box=[200., 220., 245., 340.],
                    width=752, height=480, fy=376., bearing=-.25,
                    x=8., y=0., camera_tilt=0., geometry_ok=True)
        data.update(change)
        return NS(data=json.dumps(data))

    def evidence(self, **change):
        data = dict(source_uav='u0', target_id='t5', local_target_id='t4',
                    real_detection=True, sample_s=100., confidence=.9,
                    range_m=8., x=8., y=0., official_view_ok=True)
        data.update(change)
        return NS(data=json.dumps(data))

    def test_actual_bridge_mapping_reaches_camera_in_either_callback_order(self):
        for image_first in (False, True):
            with self.subTest(image_first=image_first):
                a = self.agent()
                b = bridge._mk_bridge()
                b.source_pub = bridge._FakePub()
                for t in (99.4, 99.6):
                    b.core.report(t, 'red1', 8., 0., .9, 8.)
                    b.core.report(t, 'red2', 8., .65, .9, 8.)
                b._advance(100.1)
                b._report_cb(NS(data=json.dumps(dict(
                    source_uav='u0', target_id='red2', xyz=[8., 0., 1.25],
                    real_detection=True, sample_s=100., confidence=.9,
                    range_m=8., official_view_ok=True))))
                self.assertEqual(len(b.source_pub.msgs), 1)
                evidence = b.source_pub.msgs[0]
                self.assertEqual(json.loads(evidence.data)['target_id'], 't5')
                if image_first:
                    a._tracking_view_cb(self.view())
                a._visual_source_cb(evidence)
                if not image_first:
                    a._tracking_view_cb(self.view())
                view = a._fresh_tracking_view(100.1)
                self.assertIsNotNone(view)
                self.assertEqual((view['target_id'], view['track_id']), ('t5', 7))
                self.assertEqual(view['bearing'], -.25)
                self.assertEqual((a._confirm_start, a._last_confirm_t), (0., 0.))

    def test_same_local_name_cannot_steer_the_other_global_red_person(self):
        a = self.agent()
        a._visual_source_cb(self.evidence(target_id='t4', local_target_id='t5', y=.65))
        a._tracking_view_cb(self.view(target_id='t5', y=.65))
        self.assertIsNone(a._fresh_tracking_view(100.1))

    def test_unmapped_same_named_red_box_waits_for_its_identity(self):
        a = self.agent()
        a._tracking_view_cb(self.view(target_id='t5'))
        self.assertIsNone(a._fresh_tracking_view(100.1))

    def test_peer_other_image_or_other_position_cannot_bind_a_view(self):
        for change in (dict(source_uav='u1'), dict(sample_s=99.9),
                       dict(x=8.1), dict(local_target_id='t5')):
            with self.subTest(change=change):
                a = self.agent()
                a._visual_source_cb(self.evidence(**change))
                a._tracking_view_cb(self.view())
                self.assertIsNone(a._fresh_tracking_view(100.1))

    def test_delayed_binding_cannot_revive_an_expired_image(self):
        a = self.agent()
        a._tracking_view_cb(self.view())
        with patch.object(sa.rospy.Time, 'now', return_value=sa.rospy.Time.from_sec(100.6)):
            a._visual_source_cb(self.evidence())
        self.assertIsNone(a._fresh_tracking_view(100.6))

    def test_unreportable_frame_can_steer_but_not_become_qualified(self):
        a = self.agent()
        a._visual_source_cb(self.evidence(official_view_ok=False))
        a._tracking_view_cb(self.view(geometry_ok=False))
        self.assertIsNotNone(a._fresh_tracking_view(100.1))
        self.assertFalse(a._visual_observations['t5']['eligible'])
        self.assertEqual(getattr(a, '_eligible_observations', {}), {})

    def test_bound_red_ray_reaches_flight_yaw_while_radar_stops_translation(self):
        a = self.agent()
        a._visual_source_cb(self.evidence())
        a._tracking_view_cb(self.view())
        a.yaw, a._look_at, a.altitude_layer = 0., (8., 0.), 3.
        a._orbit_target = 't5'
        a._scan_t, a._radar_prev = 100., (20., 20., 20.)
        a._takeoff_done, a._landing = True, False
        a._est_trustworthy = lambda: True
        a._bounds_recovery_velocity = lambda: None
        a._radar_guard_velocity = lambda vx, vy: (0., 0.)
        a._grid_guard_velocity = lambda vx, vy: (vx, vy)
        a._map_guard_velocity = lambda vx, vy: (vx, vy)
        a._final_scan_cap = lambda vx, vy: (vx, vy)
        a._last_cmd_v, a._last_csv_t = (1., 0.), 100.1
        a._climb_latch, a._climb_t0, a._alt_sag = False, None, 0.
        a._scan = NS(angle_min=-math.pi, angle_increment=math.pi/180.,
                     range_min=.1, range_max=20., ranges=[20.]*361)
        a.pos_pub = Mock()
        with patch.object(sa, 'FLIGHT_OUTPUT', 'pos'):
            a._send_vel(1., 0.)
        output = a.pos_pub.publish.call_args.args[0].pose
        self.assertEqual((output.position.x, output.position.y), a.local_xy)
        self.assertLess(output.orientation.z, 0.)
        self.assertEqual((a._confirm_start, a._last_confirm_t), (0., 0.))


if __name__ == '__main__':
    unittest.main()
