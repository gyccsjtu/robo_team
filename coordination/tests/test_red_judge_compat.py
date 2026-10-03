"""Exercise the adapted judge's actual callbacks, independently of ROS."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
from red_judge_compat import adapt_red_judge

SOURCE = '''
def actor_info_callback(msg):
    _process_actor_detection(msg, actor_id_dict.get(msg.cls, []))
def actor_info1_callback(msg):
    _process_actor_detection(msg, [5])
def actor_info2_callback(msg):
    _process_actor_detection(msg, [4])
if __name__ == '__main__':
    actor_red2_sub = rospy.Subscriber("/actor_red2_info",ActorInfo,actor_info2_callback,queue_size=1)
'''


class RedJudgeCompatibilityTests(unittest.TestCase):
    def callbacks(self):
        calls=[]
        scope=dict(left_actors=[0,1,2,3,4,5],actors_pos=[None]*4+[
            SimpleNamespace(x=10.,y=0.),SimpleNamespace(x=30.,y=0.)],
            err_threshold=1.,actor_id_dict={'white':[3]},
            _process_actor_detection=lambda msg,ids:calls.append(ids))
        tree=ast.parse(adapt_red_judge(SOURCE))
        functions=[n for n in tree.body if isinstance(n,ast.FunctionDef)]
        exec(compile(ast.Module(body=functions,type_ignores=[]),'adapted_judge_callbacks','exec'),scope)
        return scope,calls

    def test_red_color_matches_either_truth_without_stream_identity(self):
        scope,calls=self.callbacks()
        for name in ('actor_info_callback','actor_info1_callback','actor_info2_callback'):
            for x,expected in ((10.2,[4]),(29.8,[5]),(20.,[]),(11.,[])):
                scope[name](SimpleNamespace(cls='red',x=x,y=0.))
                self.assertEqual(calls[-1],expected)

    def test_interleaved_red_reports_do_not_reset_the_other_person(self):
        scope,calls=self.callbacks()
        for _ in range(160):
            for x in (10.,30.):
                scope['actor_info_callback'](SimpleNamespace(cls='red',x=x,y=0.))
        self.assertEqual(calls,[[4],[5]]*160)
        scope['left_actors'].remove(4)
        scope['actor_info_callback'](SimpleNamespace(cls='red',x=10.,y=0.))
        self.assertEqual(calls[-1],[])

    def test_single_report_cannot_complete_both_targets_and_non_red_is_unchanged(self):
        scope,calls=self.callbacks();scope['actors_pos'][5]=SimpleNamespace(x=10.1,y=0.)
        scope['actor_info_callback'](SimpleNamespace(cls='red',x=10.04,y=0.))
        self.assertEqual(calls[-1],[4])
        scope['actor_info_callback'](SimpleNamespace(cls='white',x=10.,y=0.))
        self.assertEqual(calls[-1],[3])
        self.assertIn('/actor_red_info',adapt_red_judge(SOURCE))
        with self.assertRaises(ValueError):adapt_red_judge('pass')
