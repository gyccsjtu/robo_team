"""ROS wrapper scheduling tests; no master and no simulation."""
from collections import deque
import importlib.util
import json
from pathlib import Path
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
from test_gimbal_attention import AttitudeHistory,ScanAndHandoff

path=Path(__file__).resolve().parents[1]/'perception/gimbal_auxiliary.py'
spec=importlib.util.spec_from_file_location('gimbal_runtime',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.n=module.GimbalAuxiliary.__new__(module.GimbalAuxiliary)
        n=self.n;n.lock=threading.RLock();n.rospy=NS(get_time=lambda:100.1)
        h=AttitudeHistory();h.add(100.,(0.,0.,0.,1.),12)
        self.d=dict(state=NS(connected=True,armed=True,mode='OFFBOARD'),
                    pose=(100.,0.,0.,3.),assigned=False,gimbal=h,bino_s=100.,
                    bino_delay=deque([.08],maxlen=12),assignment_s=99.,
                    action=ScanAndHandoff())
        n.aircraft={'u0':self.d}

    def test_primary_latency_pose_loss_or_landing_disables_auxiliary(self):
        self.assertTrue(self.n.available(self.d,100.1))
        for changes in (dict(bino_s=99.),dict(pose=(99.,0.,0.,3.)),
                        dict(assigned=True),dict(bino_delay=deque([.4])),
                        dict(state=NS(connected=True,armed=True,mode='AUTO.LAND'))):
            self.assertFalse(self.n.available(dict(self.d,**changes),100.1))

    def test_stale_task_update_cannot_reenable_auxiliary(self):
        n=self.n
        n.assignment(NS(uav_id='u0',header=NS(stamp=NS(to_sec=lambda:100.)),task_type=1))
        n.assignment(NS(uav_id='u0',header=NS(stamp=NS(to_sec=lambda:99.9)),task_type=0))
        self.assertTrue(self.d['assigned'])

    def test_feedback_must_be_same_handoff_and_fresh(self):
        self.d['action'].active=dict(candidate_id='g1',color='red',bearing=0.,origin=(0.,0.))
        self.n.feedback(NS(data=json.dumps(dict(source_uav='u0',candidate_id='g2',
            sample_s=100.1,result='bino_acquired'))))
        self.assertIsNotNone(self.d['action'].active)
        self.n.feedback(NS(data=json.dumps(dict(source_uav='u0',candidate_id='g1',
            sample_s=100.1,result='bino_acquired'))))
        self.assertIsNone(self.d['action'].active)

    def test_latest_camera_slot_rejects_out_of_order_image(self):
        self.d['image']=None
        def msg(t):return NS(header=NS(stamp=t))
        self.n.receive(msg(100.),('u0','image'))
        self.n.receive(msg(99.),('u0','image'))
        self.assertEqual(self.d['image'].header.stamp,100.)


if __name__=='__main__':unittest.main()
