"""Negative appearance, task release and stale-coverage behavioural regressions."""
import json
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np

import test_attention_priority as attention
import test_search_segments as segments
import test_hypothesis_dispatch as dispatch
from perception import perception_real as pr
import swarm_agent as sa
import swarm_manager as sm
import swarm_task as st


class PixelRejectionTests(unittest.TestCase):
    def fixture(self):
        tracker=pr.PixelCueTracker()
        ray=pr.foot_ground_ray(376.,300.,np.eye(3))
        box=dict(cls='blue',conf=.9,u=376.,v=300.,h=80.,
            box=[361.,260.,391.,340.],bearing=pr.image_world_bearing(376.,300.,np.eye(3)),
            elevation=math.atan2(ray[2],math.hypot(ray[0],ray[1])),cam_x=0.,cam_y=0.)
        for stamp in (100.,100.2,100.4): tracker.update(stamp,[box])
        negative=dict(status='secondary_distractor',primary_label='blue',
                      box=box['box'],confidence=.9)
        return tracker,box,negative

    def test_geometry_veto_names_the_exact_previously_active_instance(self):
        t,b,_=self.fixture()
        for stamp in (100.6,100.8,101.):t.update(stamp,[dict(b,body_height_m=.8)])
        self.assertEqual(t.rejections[0]['candidate_id'],'blue:0')
        self.assertEqual(t.rejections[0]['sample_s'],101.)
        self.assertEqual(t.rejections[0]['reason'],'non_person_geometry')

    def test_confirmed_secondary_can_revoke_recent_matching_pixel_only(self):
        t,b,n=self.fixture()
        result=t.reject_appearance(100.6,[n],np.eye(3),(0.,0.),[],True)
        self.assertEqual(result[0]['candidate_id'],'blue:0')
        self.assertEqual(result[0]['reason'],'confirmed_distractor')
        self.assertEqual(t.tracks[0]['sample_s'],100.4)  # no fake observation

    def test_shadow_pending_stale_conflicting_and_ambiguous_veto_abstain(self):
        t,b,n=self.fixture()
        for sample,evidence,positive,enforce in (
            (100.6,[n],[],False),(100.6,[dict(n,status='secondary_pending')],[],True),
            (101.,[n],[],True),(100.4,[n],[],True),(100.6,[n],[b],True),
            (100.6,[dict(n,primary_label='red')],[],True)):
            self.assertEqual(t.reject_appearance(sample,evidence,np.eye(3),(0.,0.),positive,enforce),[])
        t.tracks.append(dict(t.tracks[0],id=1))
        self.assertEqual(t.reject_appearance(100.6,[n],np.eye(3),(0.,0.),[],True),[])


class AttentionReleaseTests(unittest.TestCase):
    setUp=attention.AttentionPriorityTests.setUp
    cue=attention.AttentionPriorityTests.cue

    def reject(self,**changes):
        d=dict(source_uav='u0',real_detection=True,purpose='camera_rejected',
               candidate_id='blue:1',sample_s=self.now,reason='non_person_geometry')
        d.update(changes)
        self.a._visual_candidate_cb(NS(data=json.dumps(d)))

    def track(self,tid='t2',cid='brown:0'):
        self.a.assignment=NS(task_type=1,target_id=tid)
        self.a._track_assigned_id=tid
        self.a._tracking_image=dict(candidate_id=cid)

    def release(self,tid='t2'):
        self.a._assign_cb(NS(uav_id='u0',task_type=255,target_id=tid))

    def test_negative_stops_pending_look_and_prevents_id_churn_retry(self):
        self.cue();self.now=100.2;self.reject()
        self.assertIsNone(self.a._camera_cue)
        self.now=100.4;self.cue(cid='blue:99')
        self.assertIsNone(self.a._camera_cue)
        self.assertEqual(self.a.assignment.task_type,0)
        self.assertEqual(self.detections,[])

    def test_wrong_source_old_sample_different_id_or_uncertain_negative_cannot_cancel(self):
        self.cue()
        for args in (dict(source_uav='u1'),dict(sample_s=99.9),
                     dict(candidate_id='red:1'),dict(reason='stationary'),
                     dict(real_detection=False),dict(sample_s=101.)):
            self.reject(**args)
            self.assertIsNotNone(self.a._camera_cue,args)

    def test_negative_cannot_cancel_assigned_person(self):
        self.track('t1','blue:1');self.reject()
        self.assertEqual(self.a.assignment.target_id,'t1')
        self.assertEqual(self.a._tracking_image['candidate_id'],'blue:1')

    def test_neighbour_waits_for_release_then_interrupts_search_without_report(self):
        self.track();self.cue(cid='white:5')
        self.assertIsNone(self.a._camera_cue)
        self.assertEqual(self.a.assignment.target_id,'t2')
        self.now=100.2;self.release()
        self.assertEqual(self.a._camera_cue['candidate_id'],'white:5')
        self.assertEqual(self.a._camera_cue['sample_s'],100.)
        self.assertEqual(self.detections,[])
        self.assertTrue(self.a._hold_camera_candidate())

    def test_stale_weak_single_frame_or_rejected_neighbour_not_resumed(self):
        for case in ('stale','weak','single','negative'):
            self.a._nearby_candidates={};self.track()
            self.cue(cid='blue:1',confidence=.4 if case=='weak' else .9,
                     hits=1 if case=='single' else 3)
            if case=='negative':self.reject()
            if case=='stale':self.now+=.6
            self.release()
            self.assertIsNone(self.a._camera_cue,case)

    def test_red_neighbour_is_distinct_instance_not_other_colour_slot(self):
        self.track('t4','red:4');self.cue(cid='red:4');self.cue(cid='red:8')
        self.release('t4')
        self.assertEqual(self.a._camera_cue['candidate_id'],'red:8')
        self.assertIsNone(self.a._camera_cue.get('target_id'))

    def test_old_cancel_cannot_preempt_current_person_for_pending_neighbour(self):
        self.track();self.cue(cid='white:5');self.release('t1')
        self.assertIsNone(self.a._camera_cue)
        self.assertEqual(self.a.assignment.target_id,'t2')


class ReleaseDispatchTests(unittest.TestCase):
    setUp=dispatch.HypothesisDispatchTests.setUp
    def setup_release(self):
        m=self.m
        m._release_tracker('t2',reason='lost')
        self.now=100.6
        m._eligible_observations[('u0','t4')]=dict(eligible=True,sample_s=100.5,
            since_s=100.1,hits=3,x=-25.,y=13.)
        return m

    def test_new_own_camera_target_can_dispatch_during_normal_search_cooldown(self):
        m=self.setup_release()
        self.assertTrue(m._release_blocks_target('u0','t2',self.now))
        self.assertFalse(m._release_blocks_target('u0','t4',self.now))
        self.assertNotIn('u0',m._dispatch_available(self.now,set()))
        available=m._dispatch_available(self.now,set(),pending_targets=['t4','t5'])
        self.assertIn('u0',available)
        costs=m._target_assignment_costs({'t4':(-25.,13.),'t5':(-24.,13.)},self.now,set(),available)
        self.assertIn('u0',costs['t4'])
        self.assertNotIn('u0',costs['t5'])

    def test_fault_cooldown_stale_single_frame_or_grounded_uav_still_blocks(self):
        m=self.setup_release();d=m._eligible_observations[('u0','t4')]
        self.assertTrue(m._release_blocks_target('u0','t4',100.3))
        for changes in (dict(sample_s=99.),dict(hits=1),dict(eligible=False),dict(since_s=100.5)):
            saved=dict(d);d.update(changes)
            self.assertTrue(m._release_blocks_target('u0','t4',self.now),changes)
            d.clear();d.update(saved)
        m.status['u0'].z=.5
        self.assertTrue(m._release_blocks_target('u0','t4',self.now))
        m.status['u0'].z=3.
        m._cancel_unavailable_owner('u0','t2',self.now)
        self.assertTrue(m._release_blocks_target('u0','t4',self.now))


class CoverageReopenTests(unittest.TestCase):
    uid='typhoon_h480_0'
    setUp=segments.ManagerSegmentTests.setUp
    def test_empty_auction_cannot_erase_fresh_coverage_or_owned_cells(self):
        m=self.m
        self.now=200.
        m.status[self.uid].x=-100.
        for i,c in enumerate(m.grid.cells.values()):
            c.state=st.STATE_COVERED;c.covered_t=195.
        m.grid.cell((3,0)).covered_t=100.
        m.grid.cell((4,0)).covered_t=100.;m.grid.cell((4,0)).owner='other'
        m.grid.cell((5,0)).state=st.STATE_FREE  # distant first pass remains
        with patch.dict('os.environ',REV_COVER_AGE='60'):
            self.assertEqual(sm.SwarmManager._reopen_covered_cells(m,force=True),1)
        self.assertEqual(m.grid.cell((2,0)).state,st.STATE_COVERED)
        self.assertEqual(m.grid.cell((3,0)).state,st.STATE_FREE)
        self.assertEqual(m.grid.cell((4,0)).state,st.STATE_COVERED)
        self.assertEqual(m.grid.cell((5,0)).state,st.STATE_FREE)

    def test_blind_revisit_can_reopen_after_timeout_but_not_when_reserved(self):
        m=self.m;self.now=200.;m.status[self.uid].x=-100.
        for c in m.grid.cells.values():c.state=st.STATE_COVERED;c.covered_t=195.
        c=m.grid.cell((3,0));c.state=st.STATE_REVIEW;c.review_t=100.
        m._segments.reserved[(3,0)]='other'
        self.assertEqual(sm.SwarmManager._reopen_covered_cells(m,force=True),0)
        m._segments.reserved.clear()
        self.assertEqual(sm.SwarmManager._reopen_covered_cells(m,force=True),1)
        self.assertEqual(c.state,st.STATE_FREE)


if __name__=='__main__':unittest.main()
