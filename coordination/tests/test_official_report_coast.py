import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import unittest

SOURCE=Path(__file__).parents[1]/'src/robocup_swarm/scripts/yolo_target_bridge.py'
tree=ast.parse(SOURCE.read_text(encoding='utf-8'))
method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_emit')
scope=dict(COAST_TIME=1.5,TID_TO_TAG={'t5':'red1'})
exec(compile(ast.Module(body=[method],type_ignores=[]),str(SOURCE),'exec'),scope)


class OfficialReportCoastTests(unittest.TestCase):
    def emit(self, age, eliminated=False, state=0):
        publisher=Mock()
        bridge=SimpleNamespace(core=SimpleNamespace(tracks={'red1':SimpleNamespace(t_obs=10.)}),
            _TargetState=lambda:SimpleNamespace(header=SimpleNamespace()),
            _ActorInfo=SimpleNamespace,_rospy=SimpleNamespace(Time=SimpleNamespace(from_sec=lambda t:t)),
            _now=lambda:10.+age,pub=Mock(),_actor_pubs={'red1':publisher})
        scope['_emit'](bridge,dict(tag='red1',tid='t5',x=3.2,y=4.1,vx=1.,vy=0.,
                                   state=state,eliminated=eliminated))
        return bridge,publisher

    def test_short_prediction_can_bridge_gap_without_relabeling_image_time(self):
        bridge,publisher=self.emit(1.4)
        message=publisher.publish.call_args.args[0]
        self.assertEqual((message.cls,message.x,message.y),('red',3.2,4.1))
        self.assertEqual(bridge.pub.publish.call_args.args[0].header.stamp,10.)

    def test_prediction_ends_at_existing_coast_limit(self):
        for age in (-.1,1.500001,6.):
            with self.subTest(age=age):
                _,publisher=self.emit(age)
                publisher.publish.assert_not_called()
        _,publisher=self.emit(1.5)
        publisher.publish.assert_called_once()

    def test_eliminated_or_dead_track_never_reports(self):
        for eliminated,state in ((True,0),(False,3)):
            _,publisher=self.emit(.2,eliminated,state)
            publisher.publish.assert_not_called()
