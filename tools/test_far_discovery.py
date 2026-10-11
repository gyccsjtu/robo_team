"""Recorded far-person approach hints must never qualify official coordinates."""
import json,math,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from perception import perception_real as pr
import test_discovery_dispatch as fixtures
from visual_evidence import parse_visual_evidence


class FarGeometryTests(unittest.TestCase):
    def project_box(self,height=1.7,distance=18.,z=3.):
        # Level optical camera: +X forward and -Z down in the body frame.
        bottom=pr.CY+pr.FY*z/distance
        top=pr.CY+pr.FY*(z-height)/distance
        return [pr.CX-10.,top,pr.CX+10.,bottom]

    def test_complete_far_person_has_navigation_geometry_only(self):
        d=pr.coarse_approach_geometry(self.project_box(),(10.,20.,3.),np.eye(3))
        self.assertAlmostEqual(d['x'],28.)
        self.assertAlmostEqual(d['y'],20.)
        self.assertAlmostEqual(d['body_height_m'],1.7)
        self.assertNotIn('official_view_ok',d)

    def test_small_object_crop_tilt_extreme_range_or_invalid_height_abstains(self):
        self.assertIsNone(pr.coarse_approach_geometry(self.project_box(height=.8),(0.,0.,3.),np.eye(3)))
        for distance in (8.,30.):
            self.assertIsNone(pr.coarse_approach_geometry(self.project_box(distance=distance),(0.,0.,3.),np.eye(3)))
        for box in ([0.,240.,20.,300.],[350.,1.,380.,50.],[350.,400.,380.,480.]):
            self.assertIsNone(pr.coarse_approach_geometry(box,(0.,0.,3.),np.eye(3)))
        R=np.array([[math.cos(.3),0,math.sin(.3)],[0,1,0],[-math.sin(.3),0,math.cos(.3)]])
        self.assertIsNone(pr.coarse_approach_geometry(self.project_box(),(0.,0.,3.),R))

    def test_recorded_brown_can_request_approach_before_precise_range_is_available(self):
        rows=json.loads((Path(__file__).parent/'fixtures/brown_far_044233.json').read_text())['frames']
        t=pr.PixelCueTracker();packets=[]
        with patch.dict('os.environ',PR_FAR_DISCOVERY='1'):
            for row in rows:
                box=row['box'];R=np.array(row['rotation']);cam=row['camera_xyz'];conf=row['confidence']
                regular=pr.pixel_cue_box_ok('brown',conf,*box)
                small=pr.pixel_cue_box_ok('brown',conf,*box,allow_small_continuation=True)
                raw=[]
                if regular or small:
                    u,v=(box[0]+box[2])/2.,(box[1]+box[3])/2.;ray=pr.foot_ground_ray(u,v,R)
                    raw=[dict(cls='brown',conf=conf,u=u,v=v,h=box[3]-box[1],box=box,
                        bearing=math.atan2(ray[1],ray[0]),elevation=math.atan2(ray[2],math.hypot(*ray[:2])),
                        cam_x=cam[0],cam_y=cam[1],small_box_continuation_only=not regular,
                        appearance_status='actor',coarse_approach=pr.coarse_approach_geometry(box,cam,R))]
                cues=t.update(row['sample_s'],raw)
                packet=pr.discovery_candidate_frame([], 'u0',row['sample_s'],'recorded',cues)
                packets.extend(packet['candidates'])
        self.assertTrue(packets)
        self.assertLess(packets[0]['sample_s'],1963.916)
        self.assertTrue(all(p['target_id']=='t2' and not p['official_view_ok'] for p in packets))
        self.assertGreater(packets[0]['range_m'],18.)
        for p in packets:self.assertFalse(parse_visual_evidence(json.dumps(p),p['sample_s'],['u0'])['eligible'])

    def test_disabling_feature_and_red_instances_never_emit_far_candidates(self):
        cue=dict(candidate_id='red:1',sample_s=100.,box=self.project_box(),confidence=.9,
            approach_geometry=dict(x=18.,y=0.,range_m=18.,body_height_m=1.7,
                                   sensitivity_m=.8,hits=5,span_s=.8))
        with patch.dict('os.environ',PR_FAR_DISCOVERY='1'):
            self.assertEqual(pr.discovery_candidate_frame([],'u0',100.,'s',[cue])['candidates'],[])
        cue['candidate_id']='blue:1'
        with patch.dict('os.environ',PR_FAR_DISCOVERY='0'):
            self.assertEqual(pr.discovery_candidate_frame([],'u0',100.,'s',[cue])['candidates'],[])


class FarDispatchTests(unittest.TestCase):
    setUp=fixtures.DiscoveryDispatchTests.setUp
    payload=fixtures.DiscoveryDispatchTests.payload
    def far(self,**changes):
        data=self.payload(x=22.,range_m=22.,position_quality='approach_only',
            sample_span_s=.8,body_height_m=1.7,sensitivity_m=.8)
        data.update(changes)
        self.m._discovery_cb(NS(data=json.dumps(data)))

    def test_far_camera_can_own_approach_but_never_confirms_person(self):
        with patch.dict('os.environ',PR_FAR_DISCOVERY='1'):
            self.far()
            m=self.m
            self.assertEqual(m._get_target_pos('t1',100.),(22.,0.))
            m._dispatch_target_batch([('t1',22.,0.)],100.)
        self.assertEqual(m._tracking,{'t1':'u0'})
        m.tracker.report.assert_not_called()
        self.assertEqual(m._eligible_observations,{})

    def test_disabled_weak_unconfirmed_or_ill_conditioned_hint_is_rejected(self):
        with patch.dict('os.environ',PR_FAR_DISCOVERY='0'):self.far()
        self.assertEqual(self.m.tracker.targets,{})
        with patch.dict('os.environ',PR_FAR_DISCOVERY='1'):
            for change in (dict(hits=4),dict(sample_span_s=.2),dict(body_height_m=.7),
                dict(sensitivity_m=2.),dict(sensitivity_m=float('nan')),dict(range_m=25.),
                dict(confidence=.69),dict(official_view_ok=True),dict(appearance_status='ambiguous')):
                self.far(**change)
                self.assertEqual(self.m.tracker.targets,{},change)

    def test_precise_same_person_upgrade_replaces_approach_without_duplicate_time(self):
        with patch.dict('os.environ',PR_FAR_DISCOVERY='1'):self.far()
        self.now=100.2
        d=self.payload(candidate_id='accurate',x=19.,range_m=9.,official_view_ok=True)
        self.m._visual_source_cb(NS(data=json.dumps(d)))
        self.assertEqual(self.m.tracker.report.call_count,1)
        with patch.dict('os.environ',PR_FAR_DISCOVERY='1'):self.far()
        self.assertTrue(self.m._visual_observations[('u0','t1')]['eligible'])
        self.assertEqual(self.m.tracker.report.call_count,1)

    def test_assigned_camera_can_continue_after_newer_frame_from_other_discoverer(self):
        m=self.m
        with patch.dict('os.environ',PR_FAR_DISCOVERY='1'):
            self.far(candidate_id='camera0')
            self.now=100.125;self.far(source_uav='u1',candidate_id='camera1')
            self.now=100.25;self.far(candidate_id='camera0')
            self.assertEqual(m._visual_source['t1'][0],'u0')
            m._commit_target_assignment('t1','u1',22.,0.,self.now)
            self.now=100.375;self.far(source_uav='u1',candidate_id='camera1',x=22.2)
        self.assertEqual(m._visual_source['t1'][:2],('u1',100.375))
        self.assertEqual(m._get_target_pos('t1',self.now),(22.2,0.))
        m.tracker.report.assert_not_called()
        self.assertEqual(m._eligible_observations,{})

    def test_precise_camera_drops_provisional_owner_binding(self):
        m=self.m
        with patch.dict('os.environ',PR_FAR_DISCOVERY='1'):self.far()
        m._commit_target_assignment('t1','u0',22.,0.,100.)
        self.assertIn('t1',m._approach_owner_binding)
        self.now=100.2
        m._visual_source_cb(NS(data=json.dumps(self.payload(
            official_view_ok=True,range_m=9.,x=22.,candidate_id='accurate'))))
        self.assertNotIn('t1',m._approach_owner_binding)
        self.assertEqual(m.tracker.report.call_count,1)
        with patch.dict('os.environ',PR_FAR_DISCOVERY='1'):self.far()
        self.assertTrue(self.m._visual_observations[('u0','t1')]['eligible'])
        self.assertEqual(self.m.tracker.report.call_count,1)


if __name__=='__main__':unittest.main()
