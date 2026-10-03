import ast
from pathlib import Path
import sys
import textwrap
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from prepare_competition_scene import adapt_actor_wait

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
