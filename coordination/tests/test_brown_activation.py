import sys
from pathlib import Path
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'))
import yolo_target_bridge as bridge


class BrownActivationTests(unittest.TestCase):
    def report(self,core,t,confidence,tag='brown',source='uav_6',identity=None):
        return core.report(t,tag,104.,17.,confidence,source,
                           identity if identity is not None else 'run:%s:%s' % (source,t))

    def test_original_confidence_sequence_no_longer_resets_brown_activation(self):
        old = bridge.TargetBridgeCore(brown_alignment=True)
        new = bridge.TargetBridgeCore(brown_alignment=True,brown_activation=True)
        for t,c in ((2476.296,.692),(2477.628,.810),(2477.956,.609)):
            for core in (old,new):
                self.assertTrue(self.report(core,t,c))
        self.assertFalse(old.tracks['brown'].alive)
        self.assertTrue(new.tracks['brown'].alive)
        self.assertEqual(new.tracks['brown'].t_obs,2477.956)
        self.assertEqual(new.tracks['brown'].obs[-1][4],.609)

    def test_duplicate_coasted_old_or_missing_source_cannot_count_as_new_image(self):
        core = bridge.TargetBridgeCore(brown_activation=True)
        self.assertTrue(self.report(core,10.,.8,identity='frame1'))
        for time,source,identity in ((10.,'uav_6','frame2'),(9.9,'uav_6','older'),
                                     (10.1,'uav_6','frame1'),(10.1,None,'frame2'),
                                     (10.1,[],'frame2'),(10.1,'uav_6','')):
            self.assertFalse(self.report(core,time,.8,source=source,identity=identity))
        self.assertEqual(core.tracks['brown']._high_conf_count,1)
        self.assertFalse(core.tracks['brown'].alive)

    def test_low_confidence_and_gap_restart_confirmation(self):
        core = bridge.TargetBridgeCore(brown_activation=True)
        self.report(core,10.,.8)
        self.report(core,10.2,.8)
        self.report(core,10.4,.59)
        self.assertEqual(core.tracks['brown']._high_conf_count,0)
        self.report(core,10.6,.8)
        self.report(core,12.2,.8)
        self.assertEqual(core.tracks['brown']._high_conf_count,1)
        self.assertFalse(core.tracks['brown'].alive)

    def test_other_colors_and_disabled_brown_keep_existing_thresholds(self):
        for tag in ('green','blue','red1','red2'):
            core = bridge.TargetBridgeCore(brown_activation=True)
            for time in (10.,10.2,10.4):
                self.report(core,time,.65,tag=tag)
            self.assertFalse(core.tracks[tag].alive)
        core = bridge.TargetBridgeCore(brown_activation=True)
        for time in (10.,10.2,10.4):
            self.report(core,time,.45,tag='white')
        self.assertTrue(core.tracks['white'].alive)
        core = bridge.TargetBridgeCore()
        for time in (10.,10.2,10.4):
            self.report(core,time,.65)
        self.assertFalse(core.tracks['brown'].alive)

    def test_invalid_enabled_threshold_fails_explicitly(self):
        for value in (float('nan'),0.,1.01):
            with patch.object(bridge,'BROWN_NEW_TRACK_CONF',value):
                with self.assertRaises(ValueError):
                    bridge.TargetBridgeCore(brown_activation=True)


if __name__ == '__main__':
    unittest.main()
