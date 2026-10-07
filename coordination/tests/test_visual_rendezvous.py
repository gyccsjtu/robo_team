import ast
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import unittest

SOURCE=Path(__file__).parents[1]/'src/robocup_swarm/scripts/swarm_manager.py'
tree=ast.parse(SOURCE.read_text(encoding='utf-8'))
method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)
            and n.name=='_dispatch_pending_targets')
scope=dict(math=math,TRUTH_TTL=8.,
           SearchAssignment=lambda:SimpleNamespace(header=SimpleNamespace(),target_id=''),
           rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:10.)),
               loginfo=Mock(),loginfo_throttle=Mock()))
exec(compile(ast.Module(body=[method],type_ignores=[]),str(SOURCE),'exec'),scope)


class VisualRendezvousTests(unittest.TestCase):
    def manager(self, position=(18.5,0.), held=False):
        return SimpleNamespace(_eliminated=set(),_tracking={},_backup={},
            tracker=SimpleNamespace(targets={'t1':SimpleNamespace(eliminated=False,observers=[])},
                assign_observers=Mock()),
            _get_target_pos=lambda tid,now:position,_target_authority_held=lambda tid:held,
            _cancel_stale_tracking_intent=Mock(),
            status={'uav_5':SimpleNamespace(connected=True,x=0.,y=0.)},
            _tracker_rank=lambda tid,uid,d:d,
            grid=SimpleNamespace(x_min=-50.,x_max=130.,y_min=-60.,y_max=60.),
            _authorized_publish=Mock())

    def test_confirmed_eighteen_meter_target_gets_approach_offer(self):
        manager=self.manager()
        scope['_dispatch_pending_targets'](manager)
        message=manager._authorized_publish.call_args.args[0]
        self.assertEqual((message.uav_id,message.target_id,message.task_type,message.target_x),
                         ('uav_5','t1',1,18.5))

    def test_held_target_authority_still_prevents_second_offer(self):
        manager=self.manager(held=True)
        scope['_dispatch_pending_targets'](manager)
        manager._authorized_publish.assert_not_called()

    def test_missing_current_position_never_produces_approach_offer(self):
        manager=self.manager(position=(None,None))
        scope['_dispatch_pending_targets'](manager)
        manager._authorized_publish.assert_not_called()
