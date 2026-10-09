from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).parents[1]/'src/robocup_swarm/scripts'))
from yolo_target_bridge import TargetBridgeCore, COAST_TIME


class RedMotionLimitsTests(unittest.TestCase):
    def track(self, tag, enabled):
        core=TargetBridgeCore(red_motion_limits=enabled)
        tr=core.tracks[tag]
        tr.alive=True
        tr.x,tr.y=10.,5.
        tr.vx,tr.vy=2.,0.
        tr.position_s=100.
        tr.t_obs=100.2
        return core,tr

    def test_red_escape_compensation_is_bounded_by_observed_motion(self):
        old,_=self.track('red1',False)
        new,tr=self.track('red1',True)
        self.assertEqual(old.tick(101.5)[0]['x'],11.5)
        self.assertEqual(new.tick(101.5)[0]['x'],13.)
        self.assertEqual(new.tick(103.)[0]['x'],13.)
        self.assertEqual(tr.t_obs,100.2)

    def test_other_colors_keep_original_prediction(self):
        for tag in ('green','blue','brown','white'):
            old,_=self.track(tag,False)
            new,_=self.track(tag,True)
            self.assertEqual(old.tick(101.5),new.tick(101.5))

    def test_actual_upload_age_gate_is_not_extended(self):
        # tick may expose bounded navigation prediction; the actual ROS emitter
        # still requires original_s within COAST_TIME. Test its real condition.
        from types import SimpleNamespace as NS
        from unittest.mock import Mock
        import yolo_target_bridge as module
        b=module.YoloTargetBridge.__new__(module.YoloTargetBridge)
        b._emit_stats={};b._skip_trace_t={}
        b.core,tr=self.track('red1',True)
        b._now=lambda:tr.t_obs+COAST_TIME+.01
        b._ActorInfo=lambda:NS()
        publisher=Mock()
        b._actor_pubs={'red1':publisher}
        b._TargetState=lambda:NS(header=NS())
        b.pub=Mock()
        b._rospy=NS(Time=NS(from_sec=lambda x:x))
        b._emit(b.core.tick(b._now())[0])
        publisher.publish.assert_not_called()
