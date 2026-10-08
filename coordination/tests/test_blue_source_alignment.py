"""Actual pure bridge and image-time coordinate selection, no ROS or truth."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).parents[2]
sys.path.insert(0, str(ROOT/'perception'))
sys.path.insert(0, str(ROOT/'coordination/src/robocup_swarm/scripts'))
from camera_geometry import image_time_position
from yolo_target_bridge import TargetBridgeCore


class BlueSourceAlignmentTests(unittest.TestCase):
    def test_original_point_selected_only_for_configured_colors(self):
        for color in ('blue','white'):
            self.assertEqual(image_time_position(color,(1.,2.),(5.,6.),('blue','white')),(1.,2.))
        for color in ('red','green','brown'):
            self.assertEqual(image_time_position(color,(1.,2.),(5.,6.),('blue','white')),(5.,6.))
        self.assertEqual(image_time_position('blue',(1.,2.),(5.,6.)),(5.,6.))

    def test_enabled_blue_uses_source_alignment_at_latest_original_time(self):
        core = TargetBridgeCore(blue_alignment=True)
        for i in range(5):
            core.report(10.+i*.2,'blue',i*.2,0.,.9,'uav_1','a%d'%i)
        core.report(10.9,'blue',.9,0.,.9,'uav_2','b')
        track=core.tracks['blue']
        self.assertEqual(track.position_s,10.9)
        self.assertEqual(track.alignment['reason'],'TIME_ALIGNED')
        self.assertGreater(track.x,.8)
        self.assertEqual({s['uav_id'] for s in track.alignment['sources']},{'uav_1','uav_2'})

    def test_source_switch_does_not_create_velocity_from_camera_offset(self):
        core=TargetBridgeCore(blue_alignment=True)
        core.report(10.,'blue',0.,0.,.9,'uav_1','a')
        core.report(10.2,'blue',2.,0.,.9,'uav_2','b')
        track=core.tracks['blue']
        self.assertEqual(track.alignment['reason'],'SOURCE_DISAGREEMENT')
        self.assertEqual((track.vx,track.vy),(0.,0.))

    def test_option_does_not_change_other_colors_or_default_blue(self):
        for color in ('blue','white','green','red1','brown'):
            old=TargetBridgeCore();new=TargetBridgeCore(blue_alignment=True)
            for i in range(4):
                for core in (old,new):core.report(10.+i*.2,color,i*.2,0.,.9,'uav_1',str(i))
            if color!='blue':
                self.assertEqual(old.tick(10.7),new.tick(10.7))
                self.assertIsNone(new.tracks[color].alignment)
            else:
                self.assertIsNone(old.tracks[color].alignment)
                self.assertIsNotNone(new.tracks[color].alignment)

    def test_replayed_original_is_not_new_alignment_evidence(self):
        core=TargetBridgeCore(blue_alignment=True)
        self.assertTrue(core.report(10.,'blue',0.,0.,.9,'uav_1','a'))
        self.assertFalse(core.report(10.,'blue',4.,0.,.9,'uav_1','a'))
        self.assertEqual(core.tracks['blue'].position_s,10.)
