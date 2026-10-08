from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).parents[1]/'src/robocup_swarm/scripts'))
from green_source_motion import GreenSourceMotion
from yolo_target_bridge import TargetBridgeCore


class GreenMotionTests(unittest.TestCase):
    def test_actual_camera_timestamps_fit_two_meters_per_second(self):
        m=GreenSourceMotion()
        for t in (100.,100.3,100.7):r=m.observe('a',t,(2*(t-100),0),.9,str(t))
        self.assertAlmostEqual(r['velocity'][0],2.)
        self.assertEqual(r['xy'],(1.4000000000000057,0))
        self.assertEqual(r['position_s'],100.7)
        self.assertIsNone(m.observe('a',100.7,(10,0),.9,'newid'))

    def test_other_camera_gap_and_nonfinite_do_not_invent_motion(self):
        m=GreenSourceMotion();m.observe('a',100.,(0,0),.9,'1')
        self.assertEqual(m.observe('b',100.3,(50,0),.9,'b')['velocity'],(0.,0.))
        self.assertEqual(m.observe('a',102.,(10,0),.9,'2')['velocity'],(0.,0.))
        with self.assertRaises(ValueError):m.observe('a',103.,(float('nan'),0),.9,'3')

    def test_fit_bound_and_bridge_prediction_remain_limited(self):
        m=GreenSourceMotion();m.observe('a',100.,(0,0),.9,'1')
        r=m.observe('a',100.3,(2,0),.9,'2')
        self.assertAlmostEqual(r['velocity'][0],3.)
        c=TargetBridgeCore(green_alignment=True)
        for t in (100.,100.3,100.7):
            self.assertTrue(c.report(t,'green',2*(t-100),0,.9,'a',str(t)))
        ev=next(e for e in c.tick(101.) if e['tag']=='green')
        self.assertAlmostEqual(ev['x'],2.)
        self.assertEqual(c.tracks['green'].position_s,100.7)
        self.assertFalse(c.report(100.7,'green',20,0,.9,'a','duplicate'))

    def test_old_default_and_non_green_paths_stay_unchanged(self):
        old=TargetBridgeCore();new=TargetBridgeCore(green_alignment=True)
        for color in ('blue','brown','white','red1'):
            for t in (100.,100.3,100.7):
                for c in (old,new):c.report(t,color,t-100.,0.,.9,'a',str(t))
        self.assertEqual(old.tick(100.9),new.tick(100.9))
