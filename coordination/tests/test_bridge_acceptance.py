import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'))
import yolo_target_bridge as module
from visual_observation import VisualEvidence


class BridgeAcceptanceTests(unittest.TestCase):
    def bridge(self):
        b=module.YoloTargetBridge.__new__(module.YoloTargetBridge)
        b.core=module.TargetBridgeCore()
        b._lock=threading.RLock()
        b.evidence=VisualEvidence('run',['uav_1','uav_2'])
        b._now=lambda:100.5
        b._rospy=NS(Time=NS(from_sec=lambda x:x))
        b._TargetDetection=lambda:NS(header=NS())
        b.detected=[];b.confirmed=[]
        b._detection_pub=NS(publish=b.detected.append)
        b._confirmed_pub=NS(publish=b.confirmed.append)
        return b

    def image(self,uid,seq,stamp,x):
        return NS(data=json.dumps(dict(schema_version=2,run_id='run',uav_id=uid,
            seq=seq,sample_s=stamp,target_id='green',frame_id='world_enu',xyz=[x,0.,1.25],
            confidence=.9,observation_id='run:%s:%d'%(uid,seq))))

    def warm(self,b):
        for i,stamp in enumerate((100.,100.2,100.4)):
            b._visual_cb(self.image('uav_1',i+1,stamp,0.))
        self.assertEqual(len(b.confirmed),1)

    def test_rejected_other_camera_same_timestamp_is_not_confirmed(self):
        b=self.bridge()
        with patch.object(module,'_msg_string_cls',return_value=NS):
            self.warm(b)
            b._visual_cb(self.image('uav_2',1,100.4,75.))
        self.assertEqual(len(b.confirmed),1)
        self.assertEqual(len(b.detected),1)
        self.assertEqual(b.core.tracks['green'].t_obs,100.4)
        self.assertEqual(b.core.tracks['green'].x,0.)

    def test_valid_second_camera_same_timestamp_still_confirmed(self):
        b=self.bridge()
        with patch.object(module,'_msg_string_cls',return_value=NS):
            self.warm(b)
            b._visual_cb(self.image('uav_2',1,100.4,.1))
        self.assertEqual(len(b.confirmed),2)
        self.assertEqual(json.loads(b.confirmed[-1].data)['uav_id'],'uav_2')

    def test_core_acceptance_is_explicit_for_spread_old_and_eliminated_input(self):
        c=module.TargetBridgeCore()
        self.assertTrue(c.report(100.,'white',0.,0.,.9))
        self.assertFalse(c.report(100.,'white',75.,0.,.9))
        self.assertFalse(c.report(99.9,'white',0.,0.,.9))
        c.eliminated.add('white')
        self.assertFalse(c.report(100.1,'white',0.,0.,.9))


if __name__=='__main__':
    unittest.main()
