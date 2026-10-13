"""A complete walking actor is not proved to be airframe by relative fixation."""
import os
import unittest
from unittest.mock import patch
from perception import perception_real as pr
from tracking_view import ReportViewWindow, parse_tracking_view


class MovingBodyAttachmentTests(unittest.TestCase):
    def setUp(self):
        flag = patch.dict(os.environ, {'PR_COHERENT_BODY_ATTACH': '1'})
        flag.start()
        self.addCleanup(flag.stop)

    def track(self, speed=1., **changes):
        tr=pr.Track('white',8.,0.,.8,(376.,350.),(35.,75.),8.,1.7,100.)
        for i in range(1,5):
            t=100.+.2*i
            kwargs=dict(loc_ok=True,appearance_status='actor',range_method='foot_ground')
            kwargs.update(changes)
            tr.update(8.+speed*(t-100.),0.,.2,.8,(376.,350.),(35.,75.),8.,1.7,t,
                      uav=(speed*(t-100.),0.,0.),**kwargs)
        tr.d_view,tr.d_rel=3.,0.
        return tr

    def test_parallel_walking_or_running_full_person_can_be_reported_without_assignment(self):
        for speed in (1.,2.):
            tr=self.track(speed)
            self.assertTrue(tr.attached_to_cam()) # old heuristic really rejects
            self.assertTrue(tr.coherent_body_observation())
            self.assertTrue(tr.verdict(attach_check=True,motion_independent=True)[0])
            with patch.dict(os.environ,{'PR_COHERENT_BODY_ATTACH':'0'}):
                self.assertEqual(tr.verdict(attach_check=True,motion_independent=True),(False,'self'))

    def test_decoy_ambiguous_cropped_or_bad_geometry_cannot_override(self):
        for changes in (dict(appearance_status='distractor'),dict(appearance_status='ambiguous'),
                        dict(loc_ok=False),dict(range_method='head_height')):
            tr=self.track(**changes)
            self.assertFalse(tr.coherent_body_observation())
            self.assertFalse(tr.verdict(attach_check=True,motion_independent=True)[0])
        tr=self.track();tr.update(8.9,0.,.2,.8,(376.,475.),(35.,100.),8.,1.7,101.)
        self.assertFalse(tr.coherent_body_observation())

    def test_impossible_motion_and_coasted_appearance_remain_rejected(self):
        tr=self.track(speed=5.)
        self.assertFalse(tr.coherent_body_observation())
        tr=self.track();tr.coast(.2)
        self.assertFalse(tr.coherent_body_observation())

    def test_duplicate_images_and_old_quality_do_not_establish_proof(self):
        tr=self.track();tr.body_evidence=tr.body_evidence[-2:]
        self.assertFalse(tr.coherent_body_observation())
        for _ in range(10):tr.remember_body_evidence()
        self.assertEqual(len(tr.body_evidence),2)
        self.assertFalse(tr.coherent_body_observation())

    def test_person_rejection_cannot_look_like_healthy_reporting_to_controller(self):
        w=ReportViewWindow()
        for i in range(21):
            t=100.+i*.2
            v=parse_tracking_view(dict(source_uav='u3',target_id='t3',real_detection=True,
                sample_s=t,width=752,height=480,fy=376,box=[350,300,380,370],bearing=0.,
                confidence=.794,x=8.,y=0.,camera_tilt=0.,hits=4,track_id=2,
                geometry_ok=True,report_ok=True,person_verdict_ok=False,
                person_verdict_reason='self'),'u3','t3',t)
            w.observe(v)
        self.assertTrue(w.degraded('t3',104.1))
        self.assertFalse(w.recovered('t3',104.1))


if __name__=='__main__':unittest.main()
