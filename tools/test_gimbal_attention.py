"""Measured attitude, image identity and bounded auxiliary control contracts."""
import json
import math
from pathlib import Path
import struct
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'coordination/src/robocup_swarm/scripts'))
from gimbal_attention import (AttitudeHistory,AuxiliaryTracks,ScanAndHandoff,
    measured_packet,camera_rotation,image_bearing,validate_handoff,selected_uavs)


def packet(t=100.,q=(1.,0.,0.,0.),flags=12,failure=0):
    raw=struct.pack('<I4f3fIHBB',round(t*1000),*q,0.,0.,0.,failure,flags,0,0).rstrip(b'\0')
    padded=raw.ljust((len(raw)+7)//8*8,b'\0')
    return NS(msgid=285,framing_status=1,len=len(raw),
              payload64=struct.unpack('<%dQ'%(len(padded)//8),padded))


def cue(**changes):
    d=dict(purpose='gimbal_handoff',camera='cgo3',source_uav='u0',mode='handoff',
           real_detection=True,official_view_ok=False,color='white',candidate_id='gimbal:1',
           sample_s=100.,first_s=99.,started_s=99.8,until_s=105.8,bearing=.5,
           confidence=.8,hits=4,attitude_source='mavlink_285',attitude_sample_s=100.)
    d.update(changes);return d


class GeometryTests(unittest.TestCase):
    def test_native_trimmed_packet_and_invalid_clock_or_flags(self):
        self.assertEqual(packet().len,37)
        self.assertEqual(measured_packet(packet(),100.1),(100.,(0.,0.,0.,1.),12))
        for msg in (packet(t=99.),packet(t=101.),packet(flags=0),
                    packet(flags=44),packet(failure=1),packet(q=(0.,0.,0.,0.))):
            self.assertIsNone(measured_packet(msg,100.1))

    def test_pose_interpolation_never_uses_command_or_crosses_mode_change(self):
        h=AttitudeHistory();h.add(100.,(0.,0.,0.,1.),12)
        h.add(100.1,(0.,0.,0.,-1.),12)
        np.testing.assert_allclose(h.at(100.05)[0],(0,0,0,1))
        self.assertIsNone(h.at(100.11))
        h.add(100.2,(0.,0.,0.,1.),28)
        self.assertIsNone(h.at(100.15))
        h.add(101.,(0.,0.,0.,1.),28)
        self.assertIsNone(h.at(100.5))

    def test_follow_frame_uses_body_heading_once_world_lock_uses_none(self):
        for yaw in (0.,.7,-2.):
            body=((0.,0.,math.sin(yaw/2),math.cos(yaw/2)),0)
            r=camera_rotation(((0.,0.,0.,1.),12),body)
            self.assertAlmostEqual(math.atan2(r[1,0],r[0,0]),yaw)
            locked=camera_rotation(((0.,0.,0.,1.),28),body)
            self.assertAlmostEqual(math.atan2(locked[1,0],locked[0,0]),math.pi/2)

    def test_actual_camera_intrinsics_and_right_pixel_bearing(self):
        k=(640,360,[205.469637,0,320.5,0,205.469637,180.5,0,0,1],[0]*5)
        b,e=image_bearing([310.5,150.5,330.5,210.5],k,np.eye(3))
        self.assertAlmostEqual(b,0.);self.assertAlmostEqual(e,0.)
        b,_=image_bearing([410.5,150.5,430.5,210.5],k,np.eye(3))
        self.assertAlmostEqual(b,-math.atan(100/205.469637))
        with self.assertRaises(ValueError):image_bearing([310.,150.,330.,360.],k,np.eye(3))

    def test_uav_selection_rejects_duplicates_and_invalid_ids(self):
        self.assertEqual(selected_uavs('0,5'),['typhoon_h480_0','typhoon_h480_5'])
        for raw in ('','0,0','6','foo'):
            with self.assertRaises(ValueError):selected_uavs(raw)

    def test_oversized_frame_or_invalid_calibration_does_not_supply_a_bearing(self):
        k=(640,360,[205.469637,0,320.5,0,205.469637,180.5,0,0,1],[0]*5)
        for box in ([220.,10.,420.,350.], [100.,20.,500.,340.]):
            with self.subTest(box=box),self.assertRaises(ValueError):
                image_bearing(box,k,np.eye(3))
        with self.assertRaises(ValueError):
            image_bearing([300.,150.,330.,210.],k,np.full((3,3),float('nan')))
        bad=list(k[2]);bad[2]=1000.
        with self.assertRaises(ValueError):
            image_bearing([300.,150.,330.,210.],(640,360,bad,k[3]),np.eye(3))

    def test_unsupported_reference_and_nonfinite_capture_time_fail_closed(self):
        for flags in (0,4,20,44):
            with self.assertRaises(ValueError):
                camera_rotation(((0.,0.,0.,1.),flags),((0.,0.,0.,1.),0))
        h=AttitudeHistory();h.add(100.,(0.,0.,0.,1.),12)
        for stamp in (float('nan'),float('inf'),-100.,None):
            self.assertIsNone(h.at(stamp))


class AttentionTests(unittest.TestCase):
    def test_three_unique_images_required_and_two_close_reds_abstain(self):
        t=AuxiliaryTracks('test')
        d=dict(color='red',confidence=.8,bearing=0.,elevation=-.2)
        self.assertEqual(t.update(100.,[d]),[])
        self.assertEqual(t.update(100.,[d]),[])
        self.assertEqual(t.update(100.2,[d]),[])
        self.assertEqual(len(t.update(100.4,[d])),1)
        self.assertEqual(t.update(100.6,[d,dict(d,bearing=.1)]),[])

    def test_separated_red_tracks_cannot_silently_exchange_ids(self):
        t=AuxiliaryTracks('test');rows=[dict(color='red',confidence=.8,bearing=b,elevation=-.2) for b in (-.5,.5)]
        for s in (100.,100.2,100.4):r=t.update(s,rows)
        first={round(v['bearing'],1):v['candidate_id'] for v in r}
        r=t.update(100.6,rows[::-1])
        self.assertEqual(first,{round(v['bearing'],1):v['candidate_id'] for v in r})

    def test_handoff_budget_and_pose_loss_do_not_extend_by_new_images(self):
        action=ScanAndHandoff();row=dict(candidate_id='g1',color='white',confidence=.8,
            bearing=.5,elevation=-.2,hits=3,first_s=99.,sample_s=100.)
        action.offer([row],100.,(0.,0.))
        for t in (100.5,103.,105.9):
            action.offer([dict(row,sample_s=t)],t,(0.,0.))
            self.assertEqual(action.active['until_s'],106.)
            self.assertIsNotNone(action.command(t,0.,True))
        action.command(106.,0.,True)
        self.assertIsNone(action.active)
        action.offer([dict(row,candidate_id='g2',sample_s=106.)],106.,(0.,0.))
        self.assertIsNone(action.active)
        self.assertIsNone(action.command(106.,0.,False))

    def test_body_turn_is_compensated_and_stale_bearing_releases(self):
        a=ScanAndHandoff();a.active=dict(candidate_id='g1',color='white',confidence=.8,
            bearing=.5,elevation=-.2,hits=3,first_s=99.,sample_s=100.,until_s=106.,origin=(0.,0.))
        self.assertAlmostEqual(a.command(100.,.2,True)[1],-math.degrees(.3))
        a.command(101.,.2,True);self.assertIsNone(a.active)

    def test_bad_auxiliary_packets_never_grant_coordinate_or_task_identity(self):
        self.assertEqual(validate_handoff(cue(),'u0',100.1)['bearing'],.5)
        for changes in ({'x':10.},{'target_id':'t4'},{'source_uav':'u1'},
                        {'mode':'shadow'},{'attitude_source':'command'},
                        {'attitude_sample_s':99.9},{'sample_s':99.},
                        {'until_s':200.},{'hits':2},{'confidence':float('nan')}):
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                validate_handoff(cue(**changes),'u0',100.1)

    def test_lease_fields_are_immutable_and_replayed_images_cannot_refresh_bearing(self):
        action=ScanAndHandoff()
        row=dict(candidate_id='g1',color='white',confidence=.8,
                 bearing=.5,elevation=-.2,hits=3,first_s=99.,sample_s=100.)
        action.offer([row],100.,(0.,0.))
        initial=dict(action.active)
        action.offer([dict(row,bearing=1.,hits=10,started_s=104.,until_s=1000.,
                           origin=(100.,100.))],100.1,(0.,0.))
        self.assertEqual(action.active,initial)  # duplicate image
        action.offer([dict(row,sample_s=100.2,hits=4,bearing=.55,
                           started_s=104.,until_s=1000.,first_s=99.8,
                           origin=(100.,100.))],100.2,(0.,0.))
        self.assertEqual(action.active['started_s'],100.)
        self.assertEqual(action.active['until_s'],106.)
        self.assertEqual(action.active['first_s'],99.)
        self.assertEqual(action.active['origin'],(0.,0.))
        self.assertEqual(action.active['bearing'],.55)
        action.offer([dict(row,sample_s=106.,hits=30)],106.,(0.,0.))
        self.assertIsNone(action.active)

    def test_missing_nan_or_out_of_range_visual_fields_cannot_make_stable_tracks(self):
        base=dict(color='blue',confidence=.8,bearing=0.,elevation=-.2)
        for field,value in (('confidence',float('nan')),('confidence',1.1),
                            ('bearing',float('inf')),('elevation',float('nan')),
                            ('elevation',math.pi)):
            tracks=AuxiliaryTracks('test')
            for sample in (100.,100.2,100.4):
                self.assertEqual(tracks.update(sample,[dict(base,**{field:value})]),[])
            self.assertEqual(tracks.rows,{})
        tracks=AuxiliaryTracks('test')
        self.assertEqual(tracks.update(float('nan'),[base]),[])
        self.assertEqual(tracks.last_s,-1.)
        for sample in (100.,100.2,100.4):
            self.assertEqual(tracks.update(sample,[dict(color='blue')]),[])

    def test_dual_red_crossing_abstains_and_reacquires_distinct_new_instances(self):
        tracks=AuxiliaryTracks('test')
        separated=[dict(color='red',confidence=.8,bearing=b,elevation=-.2) for b in (-.5,.5)]
        for sample in (100.,100.2,100.4):rows=tracks.update(sample,separated)
        first={row['candidate_id'] for row in rows}
        crossing=[dict(color='red',confidence=.8,bearing=b,elevation=-.2) for b in (-.05,.05)]
        self.assertEqual(tracks.update(100.6,crossing),[])
        for sample in (100.8,101.,101.2):rows=tracks.update(sample,separated[::-1])
        self.assertEqual(len(rows),2)
        self.assertTrue(first.isdisjoint({row['candidate_id'] for row in rows}))

    def test_different_colors_are_not_merged_or_allowed_to_replace_active_identity(self):
        tracks=AuxiliaryTracks('test')
        detections=[dict(color=color,confidence=.8,bearing=0.,elevation=-.2)
                    for color in ('green','brown')]
        for sample in (100.,100.2,100.4):rows=tracks.update(sample,detections)
        self.assertEqual(len(rows),2)
        self.assertEqual({row['color'] for row in rows},{'green','brown'})
        action=ScanAndHandoff();action.offer(rows,100.4,(0.,0.))
        original=dict(action.active)
        changed=dict(original,sample_s=100.6,hits=4,color='blue',bearing=.1)
        action.offer([changed],100.6,(0.,0.))
        self.assertEqual(action.active,original)

    def test_stale_candidate_invalid_pose_or_clock_produces_no_command(self):
        action=ScanAndHandoff()
        row=dict(candidate_id='g1',color='white',confidence=.8,
                 bearing=.5,elevation=-.2,hits=3,first_s=99.,sample_s=100.)
        action.offer([row],101.,(0.,0.));self.assertIsNone(action.active)
        action.offer([row],100.,(float('nan'),0.));self.assertIsNone(action.active)
        action.offer([row],100.,(0.,0.))
        self.assertIsNone(action.command(100.1,float('nan'),True))
        self.assertIsNone(action.active)
        self.assertIsNone(action.command(float('nan'),0.,True))

    def test_extra_coordinates_and_frames_before_episode_are_rejected(self):
        for key in ('z','position','pose','world_position','target_position','target_x'):
            with self.subTest(key=key),self.assertRaises(ValueError):
                validate_handoff(cue(**{key:0.}),'u0',100.1)
        with self.assertRaises(ValueError):
            validate_handoff(cue(started_s=100.5,until_s=106.5,sample_s=100.,
                                 attitude_sample_s=100.),'u0',100.5)
        # The first image legitimately predates lease creation by inference
        # latency; it is still measured and fresh, not a replayed old image.
        self.assertEqual(validate_handoff(cue(started_s=100.1,until_s=106.1),
                                         'u0',100.1)['sample_s'],100.)


class AgentHandoffTests(unittest.TestCase):
    def setUp(self):
        import swarm_agent as sa
        self.sa=sa
        a=self.a=sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id='u0';a.local_xy=(0.,0.);a.offset=(0.,0.);a.local_z=3.
        a._world_trusted=lambda:True
        a._takeoff_done=True;a._landing=False;a._gimbal_handoff_enabled=True
        a.assignment=NS(task_type=0)
        a.state=NS(mode='OFFBOARD',connected=True,armed=True)
        a._gimbal_feedback_pub=Mock();a._team_tracks={};a._camera_cue=None
        a._gimbal_cue=None;a._gimbal_retry_after={};a._friend_positions={}
        a._orbit_target=None;a._target_to_orbit=None;a.yaw=0.
        a.targets={};a._visual_observations={};a._t_seen={}
        a._confirm_start=0.;a._last_confirm_t=0.
        a._send_vel=Mock()
        self.clock=patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time.from_sec(100.1))
        self.clock.start();self.addCleanup(self.clock.stop)

    def test_auxiliary_holds_only_yaw_through_existing_safe_output(self):
        a=self.a
        assignment=a.assignment
        a._gimbal_candidate_cb(NS(data=json.dumps(cue())))
        self.assertEqual(a._gimbal_cue['camera'],'cgo3')
        self.assertTrue(a._hold_gimbal_candidate())
        a._send_vel.assert_called_once_with(0.,0.)
        self.assertIsNone(a._camera_cue)
        self.assertNotIn('image',a._gimbal_cue)
        for key in ('x','y','z','xy','xyz','target_id'):
            self.assertNotIn(key,a._gimbal_cue)
        self.assertEqual(a.assignment.task_type,0)
        self.assertIs(a.assignment,assignment)
        self.assertEqual(a.targets,{})
        self.assertEqual(a._visual_observations,{})
        self.assertEqual(a._t_seen,{})
        self.assertEqual((a._confirm_start,a._last_confirm_t),(0.,0.))

    def test_primary_tracking_and_native_attention_cannot_be_preempted(self):
        a=self.a;a.assignment=NS(task_type=1,target_id='t4')
        a._gimbal_candidate_cb(NS(data=json.dumps(cue())))
        self.assertIsNone(a._gimbal_cue)
        a.assignment=NS(task_type=0);a._camera_cue=dict(candidate_id='blue:4',until=102.)
        before=dict(a._camera_cue)
        a._gimbal_candidate_cb(NS(data=json.dumps(cue())))
        self.assertEqual(a._camera_cue,before)
        self.assertIsNone(a._gimbal_cue)

    def test_binocular_independently_takes_over_without_inheriting_identity_or_budget(self):
        a=self.a
        a._gimbal_candidate_cb(NS(data=json.dumps(cue())))
        cloud_end=a._gimbal_cue['until']
        native=dict(purpose='camera_reacquire',source_uav='u0',real_detection=True,
                    sample_s=100.05,bearing=.51,confidence=.8,hits=4,candidate_id='white:7')
        a._visual_candidate_cb(NS(data=json.dumps(native)))
        self.assertEqual(a._camera_cue['candidate_id'],'white:7')
        self.assertNotEqual(a._camera_cue['until'],cloud_end)
        self.assertNotEqual(a._camera_cue['candidate_id'],a._gimbal_cue['candidate_id'])
        self.assertIsNone(a._camera_cue['xy'])
        self.assertFalse(a._hold_gimbal_candidate())
        self.assertIsNone(a._gimbal_cue)
        self.assertTrue(a._hold_camera_candidate())
        result=json.loads(a._gimbal_feedback_pub.publish.call_args.args[0].data)
        self.assertEqual(result['result'],'preempted')
        self.assertEqual(a.targets,{})
        self.assertEqual(a._visual_observations,{})

    def test_gimbal_disabled_or_stale_cannot_interrupt_search(self):
        self.a._gimbal_handoff_enabled=False
        self.a._gimbal_candidate_cb(NS(data=json.dumps(cue())))
        self.assertIsNone(self.a._gimbal_cue)
        self.a._gimbal_handoff_enabled=True
        self.a._gimbal_candidate_cb(NS(data=json.dumps(cue(sample_s=99.))))
        self.assertIsNone(self.a._gimbal_cue)

    def test_repeated_old_new_and_relabelled_images_cannot_extend_handoff(self):
        a=self.a
        a._gimbal_candidate_cb(NS(data=json.dumps(cue())))
        original=dict(a._gimbal_cue)
        for payload in (cue(bearing=1.),cue(sample_s=99.9,attitude_sample_s=99.9,bearing=1.),
                        cue(candidate_id='other-red-instance',bearing=1.)):
            a._gimbal_candidate_cb(NS(data=json.dumps(payload)))
            self.assertEqual(a._gimbal_cue,original)
        with patch.object(self.sa.rospy.Time,'now',return_value=self.sa.rospy.Time.from_sec(103.1)):
            a._gimbal_candidate_cb(NS(data=json.dumps(cue(sample_s=103.,attitude_sample_s=103.,
                                      started_s=103.,until_s=109.,bearing=.6))))
            self.assertEqual(a._gimbal_cue['until'],original['until'])
            self.assertEqual(a._gimbal_cue['started'],original['started'])
            self.assertEqual(a._gimbal_cue['sample_s'],103.)
            self.assertTrue(a._hold_gimbal_candidate())
        with patch.object(self.sa.rospy.Time,'now',return_value=self.sa.rospy.Time.from_sec(105.9)):
            self.assertFalse(a._hold_gimbal_candidate())
            self.assertIsNone(a._gimbal_cue)

    def test_half_second_image_loss_cancels_without_using_old_ray(self):
        a=self.a
        a._gimbal_candidate_cb(NS(data=json.dumps(cue())))
        with patch.object(self.sa.rospy.Time,'now',return_value=self.sa.rospy.Time.from_sec(100.51)):
            self.assertFalse(a._hold_gimbal_candidate())
        a._send_vel.assert_not_called()
        self.assertIsNone(a._gimbal_cue)
        feedback=json.loads(a._gimbal_feedback_pub.publish.call_args.args[0].data)
        self.assertEqual(feedback['result'],'expired')

    def test_tracking_assignment_preempts_cloud_and_sends_only_cancel_feedback(self):
        a=self.a
        a._gimbal_candidate_cb(NS(data=json.dumps(cue())))
        a.assignment=NS(task_type=1,target_id='t2')
        self.assertFalse(a._hold_gimbal_candidate())
        self.assertIsNone(a._gimbal_cue)
        a._send_vel.assert_not_called()
        feedback=json.loads(a._gimbal_feedback_pub.publish.call_args.args[0].data)
        self.assertEqual(feedback['result'],'preempted')
        self.assertEqual(a.assignment.target_id,'t2')

    def test_cloud_cooldown_does_not_block_a_new_fixed_camera_candidate(self):
        a=self.a
        a._gimbal_candidate_cb(NS(data=json.dumps(cue())))
        a._cancel_gimbal_cue('expired',100.1)
        self.assertGreater(a._gimbal_retry_after['gimbal:1'],100.1)
        native=dict(purpose='camera_reacquire',source_uav='u0',real_detection=True,
                    sample_s=100.05,bearing=.5,confidence=.8,hits=4,candidate_id='white:7')
        a._visual_candidate_cb(NS(data=json.dumps(native)))
        self.assertIsNotNone(a._camera_cue)
        self.assertEqual(a._camera_cue['candidate_id'],'white:7')
        self.assertTrue(a._hold_camera_candidate())
        self.assertEqual(a.targets,{})

    def test_landing_or_untrusted_world_never_receives_cloud_flight_intent(self):
        a=self.a
        a._gimbal_candidate_cb(NS(data=json.dumps(cue())))
        a.state.mode='AUTO.LAND'
        self.assertFalse(a._hold_gimbal_candidate())
        a._send_vel.assert_not_called()
        a.state.mode='OFFBOARD';a._gimbal_retry_after={}
        a._gimbal_candidate_cb(NS(data=json.dumps(cue())))
        a._world_trusted=lambda:False
        self.assertFalse(a._hold_gimbal_candidate())
        a._send_vel.assert_not_called()


if __name__=='__main__':unittest.main()
