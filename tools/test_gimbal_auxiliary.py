#!/usr/bin/env python3
"""Node-level auxiliary camera checks without ROS master, models or simulation."""
import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import struct
import sys
import threading
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'perception'))
spec = importlib.util.spec_from_file_location('gimbal_auxiliary_under_test',
                                            ROOT / 'perception/gimbal_auxiliary.py')
aux = importlib.util.module_from_spec(spec)
spec.loader.exec_module(aux)
from gimbal_attention import AttitudeHistory, AuxiliaryTracks, ScanAndHandoff, validate_handoff

NS = types.SimpleNamespace


class Stamp:
    def __init__(self, value): self.value = value
    def to_sec(self): return self.value
    def __gt__(self, other): return self.value > other.value


class Pixels:
    def cpu(self): return self
    def numpy(self): return np.array([[300., 150., 320., 220.]])


def box():
    return NS(cls=np.array(0), conf=np.array(.9), xyxy=Pixels())


def proof_filter(image, boxes, names, sample, camera):
    for b in boxes:
        b.appearance_status = 'actor'
        b.secondary_actor_confirmed = True
        b.secondary_actor_label = names[int(b.cls)]
        b.secondary_actor_sample_s = sample
        b.secondary_actor_camera = camera
    return boxes, []


class AuxiliaryNodeTests(unittest.TestCase):
    def setUp(self):
        self.uid = 'typhoon_h480_2'
        self.sample, self.now, self.wall = 100., 100.1, 200.
        n = self.node = aux.GimbalAuxiliary.__new__(aux.GimbalAuxiliary)
        n.lock = threading.RLock()
        n.args = NS(hz=4., imgsz=640, mode='handoff')
        n.rospy = NS(get_time=lambda: self.now, logwarn=Mock(),
                     logwarn_throttle=Mock(), loginfo_throttle=Mock(),
                     Time=NS(from_sec=lambda value: Stamp(value)))
        n.String = lambda **kwargs: NS(**kwargs)
        n.MountControl = lambda: NS(header=NS())
        n.bridge = NS(imgmsg_to_cv2=Mock(return_value=np.zeros((360, 640, 3), np.uint8)))
        n.model = NS(predict=Mock(side_effect=lambda *a, **kw: [NS(boxes=[box()], names={0:'blue'})]))
        n.secondary_appearance = NS(model=object(), mode='enforce', histories={}, filter=proof_filter)
        n.observation, n.handoff = Mock(), Mock()
        self.data = dict(body=AttitudeHistory(), gimbal=AttitudeHistory(), image=None,
            info=(640, 360, [205.47,0.,320.5,0.,205.47,180.5,0.,0.,1.], [0.]*5),
            pose=None, state=NS(connected=True, armed=True, mode='OFFBOARD'),
            body_yaw=0., last_image_s=-1., next_wall=0., bino_s=-1e18,
            assignment_s=-1., assigned=False, configured=True, configuring=False,
            configure_after_wall=0., config_result=None, state_s=-1e18,
            tracks=AuxiliaryTracks('test:'+self.uid), action=ScanAndHandoff(),
            inference=[], bino_delay=aux.deque(maxlen=12), last_command=None,
            log_s=-1e18, first_inference=False, pub=Mock())
        n.aircraft = {self.uid:self.data}
        self.frame(100.)

    def frame(self, sample):
        self.sample, self.now = sample, sample+.1
        self.wall += .3
        d = self.data
        d['body'].add(sample, (0.,0.,0.,1.))
        d['gimbal'].add(sample, (0.,0.,0.,1.), 12)
        d['pose'] = (sample, 0.,0.,3.)
        d['state_s'] = sample
        d['bino_s'] = sample
        d['bino_delay'].append(.1)
        d['image'] = NS(header=NS(stamp=Stamp(sample)))

    def step(self):
        with patch.object(aux.time, 'monotonic', return_value=self.wall), contextlib.redirect_stdout(io.StringIO()):
            return self.node.step(self.uid)

    def three_frames(self):
        for t in (100.,100.2,100.4):
            self.frame(t)
            self.assertTrue(self.step())

    def observation(self):
        return json.loads(self.node.observation.publish.call_args.args[0].data)

    def test_positive_secondary_measured_frames_produce_bearing_only_cue(self):
        self.three_frames()
        self.assertEqual(self.node.handoff.publish.call_count, 1)
        cue = json.loads(self.node.handoff.publish.call_args.args[0].data)
        self.assertEqual(validate_handoff(cue, self.uid, self.now)['hits'], 3)
        self.assertFalse(any(k in cue for k in ('xyz','x','y','z','target_id','position')))
        self.assertFalse(cue['official_view_ok'])
        self.assertEqual(cue['attitude_sample_s'], self.sample)

    def test_missing_secondary_still_records_observation_without_handoff(self):
        self.node.secondary_appearance = None
        self.three_frames()
        self.assertEqual(self.node.handoff.publish.call_count, 0)
        self.assertEqual(len(self.observation()['candidates']), 1)
        self.assertFalse(self.observation()['handoff_allowed'])

    def test_swallowed_secondary_failure_cannot_use_primary_box_as_proof(self):
        self.node.secondary_appearance.filter = lambda image,boxes,*args:(boxes,[])
        self.three_frames()
        self.assertFalse(self.observation()['handoff_allowed'])
        self.assertEqual(self.node.handoff.publish.call_count, 0)

    def test_partial_proof_then_exception_is_fail_closed_and_clears_cue(self):
        self.three_frames()
        self.assertIsNotNone(self.data['action'].active)
        def fail(*args):
            proof_filter(*args)
            raise RuntimeError('verifier interrupted after a partial result')
        self.node.secondary_appearance.filter = fail
        self.frame(100.6)
        self.step()
        self.assertIsNone(self.data['action'].active)
        self.assertFalse(self.observation()['handoff_allowed'])
        self.assertEqual(self.node.handoff.publish.call_count, 1)

    def test_stale_or_wrong_camera_color_ambiguous_proof_never_handoff(self):
        for field,value in [('secondary_actor_sample_s',99.),
                            ('secondary_actor_camera','typhoon_h480_5:cgo3'),
                            ('secondary_actor_label','green'),
                            ('appearance_status','ambiguous')]:
            with self.subTest(field=field):
                self.setUp()
                def mismatched(*args):
                    result = proof_filter(*args)
                    setattr(args[1][0], field, value)
                    return result
                self.node.secondary_appearance.filter = mismatched
                self.three_frames()
                self.assertEqual(self.node.handoff.publish.call_count, 0)

    def test_duplicate_frame_is_not_inferred_or_recounted(self):
        self.step()
        self.wall += .3
        self.assertFalse(self.step())
        self.assertEqual(self.node.model.predict.call_count, 1)
        self.assertEqual(self.data['tracks'].last_s, 100.)

    def test_image_without_its_measured_attitude_cannot_infer(self):
        self.data['image'] = NS(header=NS(stamp=Stamp(100.05)))
        self.assertFalse(self.step())  # No extrapolation beyond history.
        self.assertEqual(self.node.model.predict.call_count, 0)

    def test_readiness_requires_actual_pose_both_histories_config_and_primary_health(self):
        self.assertTrue(self.node.available(self.data, self.now))
        for key, value in [('configured',False), ('state_s',90.), ('pose',(99.,0.,0.,3.)),
                           ('assigned',True), ('bino_s',99.), ('bino_delay',aux.deque([.3]))]:
            with self.subTest(key=key):
                original = self.data[key]
                self.data[key] = value
                self.assertFalse(self.node.available(self.data, self.now))
                self.data[key] = original
        for key in ('body','gimbal'):
            with self.subTest(key=key):
                rows = self.data[key].rows
                self.data[key].rows = []
                self.assertFalse(self.node.available(self.data, self.now))
                self.data[key].rows = rows

    def test_primary_latency_rising_during_inference_blocks_output(self):
        def delayed(*args, **kwargs):
            self.data['bino_delay'].clear()
            self.data['bino_delay'].append(.4)
            return [NS(boxes=[box()],names={0:'blue'})]
        self.node.model.predict.side_effect = delayed
        self.assertTrue(self.step())
        self.assertEqual(self.node.observation.publish.call_count, 0)
        self.assertEqual(self.node.handoff.publish.call_count, 0)

    def test_body_duplicate_or_old_time_cannot_overwrite_latest_pose(self):
        for t in (99.95,100.):
            msg = NS(header=NS(stamp=Stamp(t)),pose=NS(
                position=NS(x=100.,y=100.,z=100.),orientation=NS(x=0.,y=0.,z=1.,w=0.)))
            self.node.receive(msg, (self.uid,'body'))
        self.assertEqual(self.data['pose'], (100.,0.,0.,3.))
        self.assertEqual(self.data['body_yaw'], 0.)

    def test_native_sender_is_sensor_one_154_not_aircraft_autopilot_id(self):
        self.assertTrue(aux.native_gimbal_sender(NS(msgid=285,sysid=1,compid=154)))
        for changes in ({'sysid':3},{'sysid':True},{'compid':1},{'compid':True},{'msgid':280}):
            msg = dict(msgid=285,sysid=1,compid=154);msg.update(changes)
            self.assertFalse(aux.native_gimbal_sender(NS(**msg)))

    def test_actual_native_packet_used_without_command_pose(self):
        sample = 100.05
        data = struct.pack('<I4f3fIHBB',100050,1.,0.,0.,0.,0.,0.,0.,0,12,0,0)
        payload = struct.unpack('<5Q',data)
        msg = NS(msgid=285,sysid=1,compid=154,framing_status=1,len=40,payload64=payload)
        self.node.receive(msg,(self.uid,'gimbal'))
        self.assertEqual(self.data['gimbal'].rows[-1][0], sample)
        msg.compid = 1
        self.node.receive(msg,(self.uid,'gimbal'))
        self.assertEqual(len(self.data['gimbal'].rows), 2)

    def test_configure_requires_both_successful_ack_and_retries_bounded(self):
        srv = types.ModuleType('mavros_msgs.srv')
        srv.MountConfigure, srv.CommandLong = object(), object()
        parent = types.ModuleType('mavros_msgs');parent.srv=srv
        for rejected in (None,0,1):
            with self.subTest(rejected=rejected):
                results = [NS(success=True,result=0), NS(success=True,result=0)]
                if rejected is not None: results[rejected] = NS(success=False,result=4)
                self.node.rospy.wait_for_service = Mock()
                self.node.rospy.ServiceProxy = Mock(side_effect=[Mock(return_value=r) for r in results])
                self.data['configuring'] = True
                with patch.dict(sys.modules,{'mavros_msgs':parent,'mavros_msgs.srv':srv}), patch.object(aux.time,'monotonic',return_value=200.):
                    self.node.configure(self.uid)
                self.assertEqual(self.data['configured'], rejected is None)
                self.assertFalse(self.data['configuring'])
                self.assertEqual(self.data['configure_after_wall'],205.)

    def test_disconnect_drops_pose_and_lease_without_flight_commands(self):
        self.three_frames()
        self.node.receive(NS(connected=False,armed=False,mode=''),(self.uid,'state'))
        self.assertFalse(self.data['configured'])
        self.assertIsNone(self.data['pose'])
        self.assertFalse(self.data['gimbal'].rows)
        self.assertFalse(self.data['body'].rows)
        self.assertIsNone(self.data['action'].active)
        self.assertEqual(self.data['pub'].publish.call_count,0)

    def test_mount_first_command_slews_from_actual_sensor_not_scan_default(self):
        before = list(self.data['gimbal'].rows)
        self.node.control(None)
        msg = self.data['pub'].publish.call_args.args[0]
        self.assertLessEqual(abs(msg.pitch),1.5+1e-9)
        self.assertLessEqual(abs(msg.yaw),1.5+1e-9)
        self.assertEqual(msg.mode,2)
        self.assertEqual(self.data['gimbal'].rows,before)
        self.now += .05
        self.node.control(None)
        second = self.data['pub'].publish.call_args.args[0]
        self.assertLessEqual(abs(second.pitch-msg.pitch),1.5+1e-9)
        self.assertLessEqual(abs(second.yaw-msg.yaw),1.5+1e-9)

    def test_old_assignment_cannot_release_current_person_task(self):
        def assignment(t,task):
            return NS(uav_id=self.uid,header=NS(stamp=Stamp(t)),task_type=task)
        self.node.assignment(assignment(100.,1))
        for t in (99.99,100.): self.node.assignment(assignment(t,0))
        self.assertTrue(self.data['assigned'])
        self.node.assignment(assignment(100.05,0))
        self.assertFalse(self.data['assigned'])

    def test_repeated_processed_frame_cannot_refresh_primary_health(self):
        self.node.bino_frame(NS(data=json.dumps(dict(source_uav=self.uid,sample_s=100.))))
        self.now = 100.5
        self.node.bino_frame(NS(data=json.dumps(dict(source_uav=self.uid,sample_s=100.))))
        self.assertEqual(self.data['bino_s'],100.)
        self.assertEqual(len(self.data['bino_delay']),1)

    def test_invalid_nonzero_distortion_cannot_emit_auxiliary_detection(self):
        w,h,k,d = self.data['info']
        self.data['info'] = w,h,k,[.1,0.,0.,0.,0.]
        self.three_frames()
        self.assertEqual(self.node.handoff.publish.call_count,0)
        self.assertEqual(self.observation()['candidates'],[])


if __name__ == '__main__': unittest.main()
