"""Bounded view correction without new localization or reporting evidence."""
import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa
from tracking_view import retained_view_recovery, retain_pixel_view, parse_tracking_view


class ReimageTests(unittest.TestCase):
    def view(self, **kw):
        v=dict(target_id='t3',candidate_id='white:18',pixel_only=True,
               position_sample_s=100.,sample_s=100.2,bearing=0.,confidence=.8,
               camera_tilt=0.,box=[350.,290.,375.,350.],width=752.,height=480.,
               foot_sin_down=.27,camera_height_m=3.5,
               geometry_ok=False,report_ok=False)
        v.update(kw)
        return v

    def scan(self):
        return NS(angle_min=-math.pi,angle_increment=math.pi/180.,
                  range_min=.5,range_max=20.,ranges=[float('inf')]*361)

    def run_view(self,v=None,state=None,now=100.3,xy=(0.,0.),scan=None,stamp=None):
        return retained_view_recovery(state,v or self.view(),now,xy,0.,3.3,4.2,
                                      scan or self.scan(),now if stamp is None else stamp)

    def test_shallow_full_body_gets_bounded_approach_and_height(self):
        s,velocity,height,reason=self.run_view()
        self.assertEqual(reason,'shallow_approach')
        self.assertGreater(velocity[0],0.)
        self.assertLessEqual(math.hypot(*velocity),.95)
        self.assertGreater(height,3.3)
        self.assertLessEqual(height,4.2)
        self.assertEqual((s['started'],s['until']),(100.,102.))

    def test_same_frames_cannot_extend_time_or_travel_budget(self):
        s,*_=self.run_view()
        s,v,h,r=self.run_view(self.view(sample_s=102.01),s,now=102.1)
        self.assertEqual((v,h,r),((0.,0.),None,'expired'))
        _,v,h,r=self.run_view(self.view(sample_s=100.4),s,now=100.5,xy=(1.51,0.))
        self.assertEqual(r,'travel_budget');self.assertEqual(v,(0.,0.))

    def test_old_unknown_tilt_small_or_bad_foot_never_authorizes_motion(self):
        for change in (dict(sample_s=99.9),dict(camera_tilt=.2),
                       dict(small_box_continuation=True,footpoint_valid=False),
                       dict(footpoint_valid=False),dict(foot_sin_down=.10),
                       dict(confidence=.5),dict(appearance_status='distractor')):
            _,v,h,_=self.run_view(self.view(**change))
            self.assertEqual(v,(0.,0.),change);self.assertIsNone(h,change)

    def test_cropped_foot_retreat_requires_rear_not_front_clearance(self):
        v=self.view(box=[330.,260.,390.,479.],footpoint_valid=False)
        _,motion,height,reason=self.run_view(v)
        self.assertEqual(reason,'cropped_retreat');self.assertLess(motion[0],0.)
        self.assertIsNone(height)
        scan=self.scan();scan.ranges[0]=1.
        _,motion,_,reason=self.run_view(v,scan=scan)
        self.assertEqual(motion,(0.,0.));self.assertEqual(reason,'retreat_blocked')

    def test_frame_changes_cannot_oscillate_retreat_into_approach(self):
        s,*_=self.run_view(self.view(box=[330.,260.,390.,479.]))
        _,motion,_,reason=self.run_view(self.view(sample_s=100.4),s,now=100.5)
        self.assertEqual(motion,(0.,0.));self.assertEqual(reason,'direction_changed_hold')

    def test_forward_obstacle_or_stale_scan_stops_height_and_motion(self):
        scan=self.scan();scan.ranges[180]=2.
        for scan,stamp in ((scan,100.3),(self.scan(),99.)):
            _,v,h,_=self.run_view(scan=scan,stamp=stamp)
            self.assertEqual(v,(0.,0.));self.assertIsNone(h)

    def test_live_white_messages_now_request_reimage_but_keep_frozen_position(self):
        fixture=json.loads((ROOT/'tools/fixtures/white_shallow_030240.json').read_text())
        initial=fixture['anchor'];view=parse_tracking_view(initial['data'],'typhoon_h480_3','t3',initial['ros'])
        anchor=view['sample_s'];xy=(view['x'],view['y']);state=None;actions=[]
        for row in fixture['frames']:
            updated=retain_pixel_view(view,row['data'],'typhoon_h480_3','t3',row['ros'])
            if updated is None:continue
            view=updated
            state,motion,_,reason=retained_view_recovery(state,view,row['ros'],(47.6,0.),view['bearing'],3.97,4.2,self.scan(),row['ros'])
            actions.append(reason)
            self.assertEqual((view['x'],view['y']),xy)
            self.assertEqual(view['position_sample_s'],anchor)
            self.assertFalse(view['geometry_ok']);self.assertFalse(view['report_ok'])
        self.assertIn('shallow_approach',actions)

    def test_invalid_optional_ray_data_cannot_break_control_or_enable_approach(self):
        fixture=json.loads((ROOT/'tools/fixtures/white_shallow_030240.json').read_text())
        initial=fixture['anchor']
        previous=parse_tracking_view(initial['data'],'typhoon_h480_3','t3',initial['ros'])
        row=fixture['frames'][0]
        for bad in (None,'invalid',float('nan'),float('inf')):
            raw=dict(row['data'],foot_sin_down=bad,camera_height_m=bad)
            view=retain_pixel_view(previous,raw,'typhoon_h480_3','t3',row['ros'])
            self.assertIsNotNone(view)
            _,velocity,height,_=retained_view_recovery(None,view,row['ros'],(47.6,0.),
                view['bearing'],3.97,4.2,self.scan(),row['ros'])
            self.assertEqual(velocity,(0.,0.));self.assertIsNone(height)

    def test_complete_pixel_frame_does_not_park_fresh_localized_follow(self):
        a=sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id='u0';a.assignment=NS(task_type=1,target_id='t3')
        a.local_xy=a.offset=(0.,0.);a.local_z=3.3;a.yaw=0.
        a._tracking_image=self.view(foot_sin_down=.4,footpoint_valid=True)
        a._scan=self.scan();a._scan_t=100.29
        a.targets={'t3':(8.,0.,1.,0.)};a._t_seen={'t3':100.}
        a._friend_positions={}
        a._apply_friend_avoidance=Mock(side_effect=lambda x,y:(x,y))
        a._send_vel=Mock()
        with patch.object(sa,'CANDIDATE_VIEW_RECOVERY',True),patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time.from_sec(100.3)):
            self.assertFalse(a._hold_tracking_reimage())
            a._send_vel.assert_not_called()
            # Identical pixels cannot extend an expired world position.
            a._t_seen['t3']=99.
            self.assertTrue(a._hold_tracking_reimage())
            self.assertEqual(a._send_vel.call_args.args,(0.,0.))

    def test_agent_uses_final_control_and_rejects_blocked_map(self):
        a=sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id='u0';a.assignment=NS(task_type=1,target_id='t3')
        a.local_xy=a.offset=(0.,0.);a.local_z=3.3;a.yaw=0.
        a._tracking_image=self.view();a._scan=self.scan();a._scan_t=100.29
        a._friend_positions={};a.grid=NS(origin=(-50.,-50.),width=400,height=200,resolution=.5)
        a._safe_reobserve_segment=Mock(return_value=True)
        a._apply_friend_avoidance=Mock(side_effect=lambda x,y:(x,y));a._send_vel=Mock()
        a._visual_observations={};a.targets={'t3':(8.,0.,0.,0.)};a._t_seen={'t3':100.}
        with patch.object(sa,'CANDIDATE_VIEW_RECOVERY',True),patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time.from_sec(100.3)):
            self.assertTrue(a._hold_tracking_reimage())
            self.assertGreater(a._send_vel.call_args.args[0],0.)
            a._safe_reobserve_segment.return_value=False
            self.assertTrue(a._hold_tracking_reimage())
            self.assertEqual(a._send_vel.call_args.args,(0.,0.))
            self.assertIsNone(a._send_vel.call_args.kwargs['view_altitude'])
        self.assertEqual(a.targets,{'t3':(8.,0.,0.,0.)})
        self.assertEqual(a._t_seen,{'t3':100.});self.assertEqual(a._visual_observations,{})


if __name__=='__main__':unittest.main()
