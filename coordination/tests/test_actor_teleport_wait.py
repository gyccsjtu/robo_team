import ast
from pathlib import Path
import sys
import textwrap
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from prepare_competition_scene import adapt_actor_source, adapt_actor_wait, patch_actor_wait

VENDOR = ROOT/'vendor'/'official_robocup'

SOURCE = '''
def actor_teleportation_callback(self, msg):
    self.teleportation_time = msg.data
    responce = SetModelStateResponse()
    responce.success = False
    while not responce.success:
        if rospy.get_time() - self.teleportation_time < self.teleportation_interval:
            print(rospy.get_time() - self.teleportation_time)
            continue
        else:
            new_point = SetModelStateRequest()
            new_point.model_state.model_name = "actor_%s"%self.id
            new_point.model_state.pose.position.x, new_point.model_state.pose.position.y = self.create_human_point()
            tele = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            responce = tele(new_point)
'''


class TeleportWaitTests(unittest.TestCase):
    def callback(self, shutdown=False):
        self.clock = 29.996
        self.requests = []
        self.sleeps = []
        def sleep(seconds):
            self.sleeps.append(seconds)
            self.clock += seconds
        def service(request):
            self.requests.append(self.clock)
            return SimpleNamespace(success=True)
        env = dict(rospy=SimpleNamespace(get_time=lambda: self.clock,
            is_shutdown=lambda: shutdown, sleep=sleep, ServiceProxy=lambda *a: service),
            SetModelStateResponse=lambda: SimpleNamespace(success=False),
            SetModelStateRequest=lambda: SimpleNamespace(model_state=SimpleNamespace(
                model_name='', pose=SimpleNamespace(position=SimpleNamespace(x=0,y=0)))),
            SetModelState=object)
        source = 'class Actor:\n' + textwrap.indent(SOURCE.lstrip(), '    ')
        method = ast.parse(adapt_actor_wait(source)).body[0].body[0]
        exec(compile(ast.Module(body=[method], type_ignores=[]), 'actor_wait', 'exec'), env)
        actor = SimpleNamespace(id='1', teleportation_interval=30.,
                                create_human_point=lambda: (29,30))
        return lambda: env['actor_teleportation_callback'](actor, SimpleNamespace(data=0.))

    def test_wait_yields_and_does_not_teleport_before_thirty_seconds(self):
        callback = self.callback()
        callback()
        self.assertEqual(self.sleeps, [.02])
        self.assertEqual(len(self.requests), 1)
        self.assertGreaterEqual(self.requests[0], 30.)

    def test_shutdown_leaves_wait_without_teleport(self):
        callback = self.callback(shutdown=True)
        callback()
        self.assertEqual(self.requests, [])
        self.assertEqual(self.sleeps, [])

    def test_unknown_source_does_not_silently_claim_adaptation(self):
        with self.assertRaisesRegex(ValueError, 'SOURCE_CHANGED'):
            adapt_actor_wait('def unexpected(): pass')


class PatchActorWaitTests(unittest.TestCase):
    """patch_actor_wait tolerates upstream's removal, but not an unrecognised rewrite."""

    def test_upstream_removal_is_tolerated_and_reported(self):
        # Upstream control_actor.py (2026-10-05) has no teleportation code at all.
        upstream = ('class ControlActor:\n'
                    '    def _plan_safe_route(self, start_pose, target, clearance=1.5):\n'
                    '        return []\n')
        text, applied = patch_actor_wait(upstream)
        self.assertEqual(text, upstream, 'upstream source must pass through unchanged')
        self.assertFalse(applied, 'caller must be told the guard was not applied')

    def test_known_loop_is_patched_and_reported(self):
        source = 'class Actor:\n' + textwrap.indent(SOURCE.lstrip(), '    ')
        text, applied = patch_actor_wait(source)
        self.assertTrue(applied)
        self.assertIn('rospy.sleep(0.02)', text)
        self.assertNotIn('print(rospy.get_time()', text)

    def test_rewritten_loop_still_present_is_not_silently_accepted(self):
        # Teleportation code still exists but the wait loop shape changed: we
        # cannot tell whether the spin/flood defect survives, so it must raise.
        rewritten = ('def actor_teleportation_callback(self, msg):\n'
                     '    self.teleportation_time = msg.data\n'
                     '    while not responce.success:\n'
                     '        do_something_else()\n')
        with self.assertRaisesRegex(ValueError, 'SOURCE_CHANGED'):
            patch_actor_wait(rewritten)


def read_vendored(variant, name):
    return (VENDOR/variant/name).read_text(encoding='utf-8')


class AdaptActorSourceTests(unittest.TestCase):
    """The scene manifest must state what was applied, not what was intended.

    These run against the pinned vendor assets, so upstream drift shows up as a
    failing test instead of a quietly wrong manifest.
    """

    def test_upstream_variant_reports_no_teleport_edits(self):
        adapted, applied = adapt_actor_source(
            read_vendored('20261006', 'control_actor.py'), Path('black_box.txt'))
        self.assertFalse(applied['teleport_deadline_30s_per_rule'],
                         'upstream removed teleportation; the 25s->30s edit must not be claimed')
        self.assertFalse(applied['teleport_wait_yield_guard'])
        self.assertNotIn('teleportation', adapted)
        self.assertTrue(applied['isolated_black_box_path'])
        self.assertIn("'black_box.txt'", adapted)

    def test_legacy_variant_rewrites_deadline_and_guard(self):
        adapted, applied = adapt_actor_source(
            read_vendored('legacy_actor_teleport', 'control_actor.py'), Path('black_box.txt'))
        self.assertTrue(applied['teleport_deadline_30s_per_rule'])
        self.assertTrue(applied['teleport_wait_yield_guard'])
        self.assertIn('self.teleportation_interval = 30', adapted)
        self.assertIn('rospy.sleep(0.02)', adapted)
        self.assertNotIn('self.teleportation_interval = 25', adapted)

    def test_missing_black_box_anchor_fails_loudly(self):
        # Without this rewrite the actor silently reads the organizer's own copy.
        with self.assertRaisesRegex(ValueError, 'BLACK_BOX_PATH_SOURCE_CHANGED'):
            adapt_actor_source('def control(): pass', Path('black_box.txt'))

    def test_absent_optional_anchor_is_reported_not_assumed(self):
        source = "os.path.expanduser('~/XTDrone/robocup/black_box.txt')\n"
        _, applied = adapt_actor_source(source, Path('black_box.txt'))
        self.assertTrue(applied['isolated_black_box_path'])
        self.assertFalse(applied['python3_actor_range'])
        self.assertFalse(applied['teleport_deadline_30s_per_rule'])
        self.assertFalse(applied['transient_service_retry'])
        self.assertIn('teleport_wait_yield_guard', applied)
