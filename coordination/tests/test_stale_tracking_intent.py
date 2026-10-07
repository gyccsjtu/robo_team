import ast
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest

SCRIPTS=Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'
sys.path.insert(0,str(SCRIPTS))
from task_authority import TaskAuthority


def task(tid='',cell=0):
    return dict(task_type=1 if tid else 0,target_id=tid,cell_ix=cell,cell_iy=0,
                target_x=float(cell),target_y=0.)


class StaleTrackingIntentTests(unittest.TestCase):
    def test_actual_dispatch_cancels_only_matching_intent_keeps_occupancy(self):
        core=TaskAuthority('run',['uav_1','uav_2','uav_3'])
        core.offer('uav_1',task(cell=1),90.)
        core.offer('uav_1',task('t1'),91.)
        core.offer('uav_2',task(cell=2),92.)
        core.offer('uav_2',task('t3'),93.)
        core.offer('uav_3',task(cell=3),94.)
        core.offer('uav_3',task(cell=4),95.)
        old_key=core.active['uav_1']['key'];old_generation=core.active['uav_1']['generation']
        tree=ast.parse((SCRIPTS/'swarm_manager.py').read_text())
        nodes=[n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)
               and n.name in ('_dispatch_pending_targets','_cancel_stale_tracking_intent')]
        scope=dict(TRUTH_TTL=8.,math=math,rospy=NS(Time=NS(now=lambda:NS(to_sec=lambda:100.)),
            loginfo=lambda *a:None,loginfo_throttle=lambda *a:None))
        exec(compile(ast.Module(body=nodes,type_ignores=[]),'actual_manager_methods','exec'),scope)
        outputs=[]
        manager=NS(_authority=core,_authority_lock=threading.RLock(),_emit_authority=outputs.extend,
            _eliminated=set(),tracker=NS(targets={'t1':NS(eliminated=False)}),
            _get_target_pos=lambda tid,now:(None,None),_tracking={'t1':'uav_1'},_backup={})
        manager._cancel_stale_tracking_intent=lambda tid,now:scope['_cancel_stale_tracking_intent'](manager,tid,now)
        scope['_dispatch_pending_targets'](manager)
        self.assertNotIn('uav_1',core.pending)
        self.assertEqual(core.pending['uav_2']['target_id'],'t3')
        self.assertEqual(core.pending['uav_3']['cell_ix'],4)
        self.assertIn(old_key,core.locks)
        self.assertFalse(core.locks[old_key]['retired'])
        self.assertEqual(core.active['uav_1']['generation'],old_generation)
        self.assertTrue(core.active['uav_1']['stopping'])
        self.assertEqual([m['action'] for m in outputs],['STOP'])
        self.assertTrue(any(e['event']=='TASK_INTENT_CANCELLED' for e in core.events))
        # Reacquired evidence arriving between dispatch and cancellation must
        # keep the existing intent. No new task or generation is fabricated.
        core.offer('uav_1',task('t1'),100.1)
        manager._get_target_pos=lambda tid,now:(3.,4.)
        scope['_cancel_stale_tracking_intent'](manager,'t1',100.2)
        self.assertEqual(core.pending['uav_1']['target_id'],'t1')


if __name__=='__main__':
    unittest.main()
