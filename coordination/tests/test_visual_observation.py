import json
import sys
import unittest
from types import SimpleNamespace
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'))
from visual_observation import VisualEvidence, TAG_TO_TID
from yolo_target_bridge import TargetBridgeCore, parse_left_actors, YoloTargetBridge


class VisualTests(unittest.TestCase):
    def test_prediction_has_no_fixed_advance_at_original_observation_time(self):
        core = TargetBridgeCore()
        for stamp in (10.,10.2,10.4):
            core.report(stamp,'green',stamp-10.,0.,.9)
        track = core.tracks['green']
        track.vx, track.vy = 1., 0.
        event = core.tick(track.position_s)[0]
        self.assertAlmostEqual(event['x'],track.x)
        self.assertAlmostEqual(event['y'],track.y)
        event = core.tick(track.position_s+.2)[0]
        self.assertAlmostEqual(event['x'],track.x+.2)
        self.assertEqual(track.t_obs,10.4)

    def test_fused_position_is_predicted_from_its_actual_weighted_time(self):
        core = TargetBridgeCore()
        for stamp in (10., 10.2, 10.4):
            core.report(stamp, 'green', 2.*(stamp-10.), 0., 1.)
        track = core.tracks['green']
        track.vx, track.vy = 2., 0.
        self.assertAlmostEqual(track.position_s, 10.2)
        self.assertAlmostEqual(core.tick(10.8)[0]['x'], 1.6)
        self.assertEqual(track.t_obs, 10.4)

    def test_high_frequency_observations_accumulate_velocity_baseline(self):
        core = TargetBridgeCore()
        for index in range(101):
            stamp = 10. + index*.02
            core.report(stamp, 'green', 2.*(stamp-10.), 0., 1.)
        track = core.tracks['green']
        self.assertGreater(track.vx, 1.9)
        self.assertLess(track.vx, 2.01)
        self.assertLess(abs(core.tick(12.2)[0]['x']-4.4), .1)
        self.assertEqual(track.t_obs, 12.)

    def test_reacquisition_cannot_inherit_teleport_velocity_or_activation(self):
        core = TargetBridgeCore()
        for stamp in (10., 10.2, 10.4):
            core.report(stamp, 'brown', 2.*(stamp-10.), 0., .9)
        self.assertTrue(core.tracks['brown'].alive)
        core.report(30., 'brown', 100., 50., .9)
        track = core.tracks['brown']
        self.assertEqual((track.vx, track.vy), (0., 0.))
        self.assertFalse(track.alive)
        self.assertEqual(track._high_conf_count, 1)
        self.assertFalse(core.tick(30.2))
        core.report(30.2, 'brown', 100.2, 50., .9)
        self.assertFalse(track.alive)
        core.report(30.4, 'brown', 100.4, 50., .9)
        self.assertTrue(track.alive)
        self.assertLess(track.vx, 1.01)
        self.assertEqual(track.vy, 0.)
        self.assertEqual(track.t_obs, 30.4)

    def message(self, uid='a', seq=1, stamp=10.):
        return dict(schema_version=2, run_id='run', uav_id=uid, seq=seq, sample_s=stamp,
            target_id='green', frame_id='world_enu', xyz=[1., 2., 0.], confidence=.8,
            observation_id='run:%s:%d' % (uid, seq))

    def test_each_camera_requires_real_new_image_and_run_identity(self):
        gate = VisualEvidence('run', ['a', 'b'])
        self.assertIsNotNone(gate.receive(self.message(), 10.1))
        self.assertIsNone(gate.receive(self.message(seq=2), 10.2))
        self.assertIsNotNone(gate.receive(self.message(uid='b'), 10.2))
        self.assertIsNone(gate.receive(dict(self.message(seq=3, stamp=10.3), run_id='old'), 10.4))
        self.assertIsNone(gate.receive(self.message(uid='unknown', stamp=10.3), 10.4))

    def test_stale_future_and_replayed_evidence_cannot_extend_confirmation(self):
        gate = VisualEvidence('run', ['a'])
        self.assertIsNone(gate.receive(self.message(stamp=8.), 10.))
        self.assertIsNone(gate.receive(self.message(stamp=11.), 10.))
        self.assertIsNotNone(gate.receive(self.message(), 10.))
        self.assertIsNone(gate.receive(self.message(), 10.1))
        self.assertIsNone(gate.receive(self.message(seq=2, stamp=10.1), 9.9))

    def test_coast_does_not_refresh_original_observation_stamp(self):
        core = TargetBridgeCore()
        for t in (10., 10.2, 10.4):
            core.report(t, 'green', 1., 2., .9)
        self.assertTrue(core.tick(10.5))
        self.assertTrue(core.tick(11.5))
        self.assertEqual(core.tracks['green'].t_obs, 10.4)
        core.report(10.3, 'green', 1., 2., .9)
        self.assertEqual(core.tracks['green'].t_obs, 10.4)

    def test_official_red_slot_identity_and_invalid_list(self):
        self.assertEqual(TAG_TO_TID['red1'], 't5')
        self.assertEqual(TAG_TO_TID['red2'], 't4')
        self.assertIsNone(parse_left_actors('error 0 2 5'))
        self.assertIsNone(parse_left_actors('[99]'))
        self.assertIsNone(parse_left_actors('bad range(0, 6)'))

    def test_empty_bootstrap_does_not_eliminate_every_target(self):
        core = TargetBridgeCore()
        self.assertFalse(core.set_left(0., '[]'))
        self.assertFalse(core.eliminated)
        core.set_left(1., '[0, 1, 2, 3, 4, 5]')
        core.set_left(2., '[]')
        self.assertEqual(core.eliminated, set(TAG_TO_TID))

    def test_actual_bridge_output_uses_red_class_and_preserves_sample_time(self):
        bridge = YoloTargetBridge.__new__(YoloTargetBridge)
        bridge.core = TargetBridgeCore()
        bridge.core.tracks['red1'].t_obs = 10.
        bridge._now = lambda: 10.5
        bridge._rospy = SimpleNamespace(Time=SimpleNamespace(from_sec=lambda seconds: seconds))
        bridge._TargetState = lambda: SimpleNamespace(header=SimpleNamespace())
        bridge._ActorInfo = SimpleNamespace
        states, official = [], []
        bridge.pub = SimpleNamespace(publish=states.append)
        bridge._actor_pubs = {'red1': SimpleNamespace(publish=official.append)}
        event = dict(tag='red1', tid='t5', x=1., y=2., vx=0., vy=0., state=0, eliminated=False)
        bridge._emit(event)
        self.assertEqual(states[-1].header.stamp, 10.)
        self.assertEqual(official[-1].cls, 'red')
        bridge._now = lambda: 11.6
        bridge._emit(event)
        self.assertEqual(len(official), 1)
        self.assertEqual(states[-1].header.stamp, 10.)
