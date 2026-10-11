"""Competition bridge must use only its cameras and its own report stream."""

import json
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
import yolo_target_bridge as bridge


class BridgeJudgeIndependenceTests(unittest.TestCase):
    def test_recorded_slot_rename_keeps_one_global_red_trajectory(self):
        fixture = json.loads((ROOT/'tools/fixtures/red_slot_rename_093017.json').read_text())
        node = bridge._mk_bridge()
        for r in fixture['reports']:
            node._advance(r['t']+.1-node._now())
            node._report_cb(bridge._FakeString(json.dumps(dict(
                target_id=r['tag'], xyz=[r['x'], r['y'], 1.25],
                confidence=r['conf'], range_m=r['range_m'], sample_s=r['t'],
                source_uav=r['source'], real_detection=True,
                official_view_ok=r['official_ok']))))
        # Baseline on these exact original images: red2 first created at
        # 2567.056; its only two points were the red1 reappearance.
        self.assertAlmostEqual(node.core.tracks['red1'].t_obs, 2567.26)
        self.assertLess(node.core.tracks['red2'].t_obs, 0.)

    def test_replaced_local_slot_cannot_create_a_second_person_at_same_anchor(self):
        node = bridge._mk_bridge()
        # 09:30 run: a cropped red1 was lost, then red2 appeared 1.58 s
        # later at its last full-body position. There was no second anchor.
        node.core.report(2565.48, 'red1', 48.3, -12., .9, 8.)
        node._red_source_map[('u1', 'red1')] = 'red1'
        node._last_source_sample[('u1', 'red1')] = 2565.48
        node._red_source_observations[('u1', 'red1')] = (2565.48, 47.05, -12.61)
        chosen = node._associate_red('u1', 'red2', 2567.056, 48.29, -12.31)
        self.assertEqual(chosen, 'red1')
        self.assertNotIn(('u1', 'red1'), node._red_source_map)
        self.assertLess(node.core.tracks['red2'].t_obs, 0.)

    def test_slot_replacement_preserves_an_independent_second_red_anchor(self):
        node = bridge._mk_bridge()
        node.core.report(10., 'red1', 0., 0., .9, 8.)
        node.core.report(10.2, 'red2', 1.5, 0., .9, 8.)
        node._red_source_map[('u1', 'red1')] = 'red1'
        node._last_source_sample[('u1', 'red1')] = 10.
        node._red_source_observations[('u1', 'red1')] = (10., 0., 0.)
        self.assertEqual(node._associate_red('u1', 'red2', 11.5, 1.5, 0.), 'red2')

    def test_second_red_far_from_replaced_slot_is_not_merged(self):
        node = bridge._mk_bridge()
        node.core.report(10., 'red1', 0., 0., .9, 8.)
        node._red_source_map[('u1', 'red1')] = 'red1'
        node._last_source_sample[('u1', 'red1')] = 10.
        node._red_source_observations[('u1', 'red1')] = (10., 0., 0.)
        self.assertEqual(node._associate_red('u1', 'red2', 11.5, 10., 0.), 'red2')

    def test_disagreeing_camera_cannot_take_stationary_judge_stream(self):
        # 00:36 match: two cameras saw the same blue person about 2 m apart.
        # A smooth but biased second camera used to seize the official stream
        # after the first camera's image was only 0.9 s old.
        core = bridge.TargetBridgeCore()
        for t in (10.0, 10.4, 10.8):
            core.report(t, 'blue', 129.0, 43.0, .9, 6.0,
                        official_ok=True, source_uav='u1')
        track = core.tracks['blue']
        self.assertEqual(track.official_source, 'u1')
        for t in (11.2, 11.6, 12.0):
            core.report(t, 'blue', 127.2, 41.4, .9, 7.0,
                        official_ok=True, source_uav='u5')
        self.assertEqual(track.official_source, 'u1')
        self.assertEqual((track.good_x, track.good_y), (129.0, 43.0))

        # A lost first camera must not lock a real actor forever.  After the
        # bounded anchor expires, three independent smooth images start a
        # new confirmation segment at the new source.
        for t in (13.4, 13.8, 14.2):
            core.report(t, 'blue', 127.2, 41.4, .9, 7.0,
                        official_ok=True, source_uav='u5')
        self.assertEqual(track.official_source, 'u5')
        self.assertEqual(track.good_history, [(14.2, 127.2, 41.4)])

    def test_border_red_anchors_survive_cross_camera_label_swap(self):
        # Replay the 22:45 geometry: actor 4 at y≈2.6, actor 5 at y≈6.6.
        # Camera-local red1/red2 names swapped as their own tracks expired.
        node = bridge._mk_bridge()
        node.core.report(396.0, 'red1', 150.0, 2.6, .95, 6.0)
        node.core.report(396.0, 'red2', 150.0, 6.6, .95, 6.0)
        node._red_source_map[('u3', 'red1')] = 'red2'
        node._red_source_map[('u3', 'red2')] = 'red1'
        node._last_source_sample[('u3', 'red1')] = 396.0
        node._last_source_sample[('u3', 'red2')] = 396.0
        node._red_source_observations[('u3', 'red1')] = (396.0, 150.0, 6.6)
        node._red_source_observations[('u3', 'red2')] = (396.0, 150.0, 2.6)

        first = node._associate_red('u3', 'red2', 398.328, 150.05, 2.37)
        self.assertEqual(first, 'red1')
        node.core.report(398.328, first, 150.05, 2.37, .95, 6.0)
        node._last_source_sample[('u3', 'red2')] = 398.328
        # A second local slot at the same actor must not seize red2 merely
        # because red2's true anchor has aged past the fresh 2 s window.
        self.assertIsNone(node._associate_red('u3', 'red1', 398.928,
                                               149.91, 3.03))
        self.assertAlmostEqual(node.core.tracks['red2'].y, 6.6)

        # Both global tracks are stale (>2 s), but still have distinct
        # positions.  New cameras may use the opposite local slot names.
        second = node._associate_red('u1', 'red2', 406.896, 150.47, 6.61)
        self.assertEqual(second, 'red2')
        node.core.report(406.896, second, 150.47, 6.61, .95, 6.0)
        third = node._associate_red('u0', 'red2', 410.864, 149.76, 2.64)
        self.assertEqual(third, 'red1')

    def test_red_rebinds_after_long_gap_or_large_motion(self):
        node = bridge._mk_bridge()
        node.core.report(1.0, 'red1', 0.0, 0.0, .95, 6.0)
        node.core.report(1.0, 'red2', 5.0, 0.0, .95, 6.0)
        # A short ambiguous jump cannot be relabelled from the new camera's
        # local name; after >30 s it may be a legitimate new actor position.
        self.assertIsNone(node._associate_red('u1', 'red1', 8.0, 100.0, 0.0))
        self.assertEqual(node._associate_red('u1', 'red1', 32.0,
                                             100.0, 0.0), 'red1')

    def test_one_fresh_red_leaves_stale_other_slot_reacquirable(self):
        node = bridge._mk_bridge()
        node.core.report(10.0, 'red1', 0.0, 0.0, .95, 6.0)
        node.core.report(7.0, 'red2', 5.0, 0.0, .95, 6.0)
        # One known person is still present.  A distant second person may
        # take the stale slot without swapping that fresh person's identity.
        self.assertEqual(node._associate_red('u1', 'red1', 11.0,
                                             100.0, 0.0), 'red2')

    def test_close_real_red_people_keep_separate_global_slots(self):
        node = bridge._mk_bridge()
        node.core.report(10.0, 'red1', 0.0, 0.0, .95, 6.0)
        node.core.report(10.0, 'red2', 0.65, 0.0, .95, 6.0)
        self.assertEqual(node._associate_red('u0', 'red1', 10.2,
                                             0.0, 0.0), 'red1')
        node._last_source_sample[('u0', 'red1')] = 10.2
        self.assertEqual(node._associate_red('u0', 'red2', 10.4,
                                             0.65, 0.0), 'red2')

    def test_same_camera_red_slots_remain_distinct_across_slow_frames(self):
        node = bridge._mk_bridge()
        node.core.report(10.0, 'red1', 0.0, 0.0, .95, 6.0)
        node.core.report(10.0, 'red2', 1.2, 0.0, .95, 6.0)
        node._red_source_map[('u0', 'red1')] = 'red1'
        node._red_source_map[('u0', 'red2')] = 'red2'
        node._last_source_sample[('u0', 'red1')] = 10.0
        node._last_source_sample[('u0', 'red2')] = 10.0

        # Both people are inside the 2.4 m association gate.  A 1.3 s gap
        # made the old 1 s mutual-exclusion rule choose the other slot.
        red1 = node._associate_red('u0', 'red1', 11.3, 1.1, 0.0)
        self.assertEqual(red1, 'red1')
        self.assertEqual(node._red_source_map[('u0', 'red2')], 'red2')
        red2 = node._associate_red('u0', 'red2', 11.3, 0.1, 0.0)
        self.assertEqual(red2, 'red2')
        self.assertNotEqual(red1, red2)

    def test_stale_other_local_red_slot_can_be_reassigned(self):
        node = bridge._mk_bridge()
        node.core.report(10.0, 'red1', 0.0, 0.0, .95, 6.0)
        node.core.report(10.0, 'red2', 1.2, 0.0, .95, 6.0)
        node._red_source_map[('u0', 'red1')] = 'red1'
        node._red_source_map[('u0', 'red2')] = 'red2'
        node._last_source_sample[('u0', 'red2')] = 9.0

        # Once the other local slot exceeds the shared freshness window,
        # a detector-side label swap may legitimately claim its old track.
        self.assertEqual(node._associate_red('u0', 'red1', 11.3, 1.1, 0.0),
                         'red2')

    def test_unmatched_red_cannot_take_other_active_local_slot(self):
        node = bridge._mk_bridge()
        node.core.report(10.0, 'red1', 0.0, 0.0, .95, 6.0)
        node.core.report(11.0, 'red1', 0.0, 0.0, .95, 6.0)
        node.core.report(10.0, 'red2', 1.2, 0.0, .95, 6.0)
        node._red_source_map[('u0', 'red1')] = 'red1'
        node._red_source_map[('u0', 'red2')] = 'red2'
        node._last_source_sample[('u0', 'red2')] = 10.7

        # red2's global anchor just became stale, but the camera saw that
        # local slot only 1.4 s ago.  An unrelated distant red1 box must not
        # seize red2 by the empty-slot fallback.
        self.assertIsNone(node._associate_red('u0', 'red1', 12.1, 100.0, 0.0))
        self.assertEqual(node._red_source_map[('u0', 'red1')], 'red1')
        self.assertEqual(node._red_source_map[('u0', 'red2')], 'red2')

    def test_live_constructor_subscribes_only_to_team_topics(self):
        subscriptions = []
        publications = []

        rospy = types.ModuleType('rospy')
        rospy.Publisher = lambda name, *_args, **_kwargs: publications.append(name) or bridge._FakePub()
        rospy.Subscriber = lambda name, *_args, **_kwargs: subscriptions.append(name)
        rospy.Timer = lambda *_args, **_kwargs: None
        rospy.Duration = lambda duration: duration
        rospy.loginfo = lambda *_args, **_kwargs: None

        team = types.ModuleType('robocup_swarm')
        team.msg = types.ModuleType('robocup_swarm.msg')
        team.msg.TargetState = bridge._FakeMsg
        actor = types.ModuleType('ros_actor_cmd_pose_plugin_msgs')
        actor.msg = types.ModuleType('ros_actor_cmd_pose_plugin_msgs.msg')
        actor.msg.ActorInfo = bridge._FakeActorInfo
        standard = types.ModuleType('std_msgs')
        standard.msg = types.ModuleType('std_msgs.msg')
        standard.msg.String = bridge._FakeString

        with patch.dict(sys.modules, {
            'rospy': rospy,
            'robocup_swarm': team, 'robocup_swarm.msg': team.msg,
            'ros_actor_cmd_pose_plugin_msgs': actor,
            'ros_actor_cmd_pose_plugin_msgs.msg': actor.msg,
            'std_msgs': standard, 'std_msgs.msg': standard.msg,
        }):
            bridge.YoloTargetBridge()

        self.assertEqual(set(subscriptions), {
            '/coordination/target_report', '/swarm/rejected_visual',
        })
        self.assertIn('/swarm/report_telemetry', publications)
        self.assertEqual(len([name for name in publications
                              if name.startswith('/actor_') and name.endswith('_info')]), 6)

    def test_both_red_streams_are_independent_and_loss_is_reversible(self):
        node = bridge._mk_bridge()
        node._advance(100.0)
        for sample in (99.4, 99.6, 99.8):
            node.core.report(sample, 'red1', 10.0, 1.0, .95, 6.0,
                             source_uav='u0')
            node.core.report(sample, 'red2', 40.0, 1.0, .95, 6.0,
                             source_uav='u1')
        node._tick(None)
        self.assertEqual(len(node._actor_pubs['red1'].msgs), 1)
        self.assertEqual(len(node._actor_pubs['red2'].msgs), 1)
        self.assertEqual(node._actor_pubs['red1'].msgs[0].x, 10.0)
        self.assertEqual(node._actor_pubs['red2'].msgs[0].x, 40.0)
        reports = [json.loads(msg.data) for msg in node.report_pub.msgs]
        self.assertEqual({item['tag'] for item in reports}, {'red1', 'red2'})
        self.assertTrue(all(item['event'] == 'published' for item in reports))
        self.assertTrue(all(item['fresh_image'] for item in reports))
        self.assertTrue(all('judge' not in item for item in reports))

        node._red_source_map[('u0', 'red1')] = 'red1'
        node._advance(bridge.DROP_TIME + 1.0)
        node._tick(None)
        lost = [json.loads(msg.data) for msg in node.report_pub.msgs
                if json.loads(msg.data)['event'] == 'visual_lost']
        self.assertEqual({item['tag'] for item in lost}, {'red1', 'red2'})
        self.assertNotIn(('u0', 'red1'), node._red_source_map)
        self.assertEqual(node.core.eliminated, set())
        self.assertFalse(any(msg.eliminated for msg in node.pub.msgs))

        for sample in (109.1, 109.3, 109.5):
            node.core.report(sample, 'red1', 12.0, 1.0, .95, 6.0,
                             source_uav='u0')
        node._advance(0.6)
        node._tick(None)
        self.assertTrue(node.core.tracks['red1'].alive)
        self.assertGreaterEqual(len(node._actor_pubs['red1'].msgs), 2)
        self.assertEqual(node._actor_pubs['red1'].msgs[-1].x, 12.0)
        self.assertFalse(any(msg.eliminated for msg in node.pub.msgs))


if __name__ == '__main__':
    unittest.main()
