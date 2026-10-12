"""Dual-model small-image looks grant no location/report/ordinary-track credit.

The 060353 original CSV boxes and same-image JPEG secondary replay below are
actual observations, not actor truth. Synthetic streaks test the new protocol;
they do not claim missing live secondary outputs were historically positive.
"""
import copy
import json
import math
import unittest
from types import SimpleNamespace as NS

from perception import perception_real as pr
from perception.distractor_verifier import unique_actor_evidence
from tools import test_attention_priority as attention_fixture


SOURCE = 'typhoon_h480_4'
EPOCH = '35c28d86f5ab46a7aa9e7dd42072fbd4'
SAMPLE = 1956.268
# Original inference CSV: u/v-inset/w/h reconstructed to actual xyxy.
RECORDED = [
    dict(label='red', confidence=.6719515323638916,
         box=[360.4671630859375,312.4273681640625,378.61138916015625,358.030029296875]),
    dict(label='green', confidence=.6854570508003235,
         box=[590.1924438476562,284.3171691894531,604.4629516601562,312.74761962890625],
         foot_down_sin=.1698713481329542),
    dict(label='brown', confidence=.6858744621276855,
         box=[345.93896484375,287.673583984375,358.90570068359375,312.97802734375],
         foot_down_sin=.19569155257817533),
    dict(label='brown', confidence=.4710887670516968,
         box=[360.0191650390625,312.084228515625,378.7807312011719,357.434814453125]),
]
# Saved views_fair/0007.jpg, same image/sample/source; JPEG is not the original
# raw Image byte buffer. Its positive labels support only this one picture.
SECONDARY_JPEG = [
    dict(label='green', confidence=.8708882331848145,
         box=[590.0634155273438,283.8724670410156,605.1488037109375,312.8562927246094]),
    dict(label='brown', confidence=.8193616271018982,
         box=[346.5392150878906,287.55010986328125,358.52142333984375,312.0076599121094]),
    dict(label='red', confidence=.7617310285568237,
         box=[360.2033386230469,312.4822082519531,378.6053161621094,358.79632568359375]),
]


class DistantSmallCueTests(unittest.TestCase):
    def setUp(self):
        self.tracker = pr.DistantSmallCueTracker()

    def box(self, color='green', bearing=0., xy=(360.,280.,374.,308.), **changes):
        result = dict(cls=color, conf=.7, box=list(xy), width=752, height=480,
            fy=376., foot_down_sin=.18, camera_tilt=.0058, bearing=bearing,
            appearance_status='actor', secondary_actor_confirmed=True,
            secondary_actor_label=color, secondary_actor_camera=SOURCE)
        result.update(changes)
        return result

    def send(self, sample, boxes=None, source=SOURCE, epoch=EPOCH, now=None, **kwargs):
        boxes = [self.box()] if boxes is None else copy.deepcopy(boxes)
        for b in boxes:
            b.setdefault('secondary_actor_sample_s', sample)
        return self.tracker.update(source, epoch, sample, sample+.1 if now is None else now,
                                   boxes, **kwargs)

    def establish(self, boxes=None):
        self.assertEqual(self.send(100., boxes), [])
        self.assertEqual(self.send(100.2, boxes), [])
        return self.send(100.4, boxes)

    def test_recorded_true_people_pass_new_input_but_not_old_40_or_32_gate(self):
        evidence = unique_actor_evidence(RECORDED, SECONDARY_JPEG)
        self.assertEqual(set(evidence), {1, 2})
        rotation = pr.quat_to_R(-.00122,.00264,.58576,.81048)
        boxes = []
        for index in (1, 2):
            original = RECORDED[index]
            bounds = original['box']
            self.assertFalse(pr.pixel_cue_box_ok(original['label'], original['confidence'], *bounds))
            self.assertFalse(pr.pixel_cue_box_ok(original['label'], original['confidence'], *bounds,
                                                allow_small_continuation=True))
            u,v = (bounds[0]+bounds[2])/2.,(bounds[1]+bounds[3])/2.
            b = self.box(color=original['label'],xy=bounds,conf=original['confidence'],
                bearing=pr.image_world_bearing(u,v,rotation),
                foot_down_sin=original['foot_down_sin'],secondary_actor_sample_s=SAMPLE)
            self.assertTrue(self.tracker._input_ok(b,SOURCE,SAMPLE))
            boxes.append(b)
        # Only one verified retained image: no invented extra samples.
        self.assertEqual(self.send(SAMPLE, boxes, now=SAMPLE+.128), [])
        self.assertEqual(self.send(SAMPLE, boxes, now=SAMPLE+.2), [])
        self.assertEqual(len(self.tracker.tracks),2)

    def test_three_fresh_synthetic_images_create_only_local_attention(self):
        cues = self.establish()
        self.assertEqual(len(cues),1)
        cue = cues[0]
        self.assertTrue(cue['candidate_id'].startswith('green:distant:'+EPOCH+':'))
        self.assertTrue(cue['attention_only'] and cue['distant_small_seed'])
        self.assertTrue(cue['association_unique'])
        for field in ('control_eligible','report_eligible','footpoint_valid'):
            self.assertIs(cue[field],False)
        for field in ('x','y','target_id','world_candidate_id','normal_anchor_s'):
            self.assertNotIn(field,cue)
        self.assertEqual(cue['source_epoch'],EPOCH)
        self.assertEqual(cue['seed_started_s'],100.)

    def test_h35_without_positive_evidence_does_not_change_old_continuation(self):
        box=self.box(color='red',xy=(360.,280.,374.,315.),secondary_actor_confirmed=False)
        self.assertFalse(pr.pixel_cue_box_ok('red',.7,*box['box']))
        self.assertTrue(pr.pixel_cue_box_ok('red',.7,*box['box'],allow_small_continuation=True))
        for sample in (100.,100.2,100.4,100.6):
            self.assertEqual(self.send(sample,[box]),[])
        self.assertEqual(self.tracker.tracks,[])
        original=pr.PixelCueTracker()
        box.update(h=35.,u=367.,v=297.5,small_box_continuation_only=True)
        for sample in (100.,100.2,100.4):
            self.assertEqual(original.update(sample,[box]),[])
        self.assertEqual(original.next_id,0)

    def test_absence_of_decoy_is_never_positive_actor_evidence(self):
        variants=[dict(secondary_actor_confirmed=v) for v in (False,None,1,'true')]
        variants += [dict(secondary_actor_label='brown'),dict(secondary_actor_camera='typhoon_h480_0'),
            dict(secondary_actor_sample_s=99.9),dict(secondary_actor_sample_s=True),
            dict(secondary_actor_sample_s=float('nan')),dict(appearance_status='ambiguous'),
            dict(appearance_status='distractor')]
        for change in variants:
            with self.subTest(change=change):
                self.tracker=pr.DistantSmallCueTracker()
                b=self.box(**change)
                for sample in (100.,100.2,100.4):
                    self.assertEqual(self.send(sample,[b]),[])
        legacy=self.box()
        del legacy['secondary_actor_confirmed']
        self.assertEqual(self.send(101.,[legacy]),[])

    def test_small_sculpture_and_conflicting_model_labels_never_seed(self):
        primary=[dict(label='blue',confidence=.9,box=[360.,280.,374.,308.])]
        decoy=[dict(label='distractor_person',confidence=.95,box=primary[0]['box'])]
        for secondary in (decoy,decoy+[dict(label='blue',confidence=.95,box=primary[0]['box'])],
                          [dict(label='white',confidence=.95,box=primary[0]['box'])]):
            self.assertEqual(unique_actor_evidence(primary,secondary),{})
        for sample in (100.,100.2,100.4):
            self.assertEqual(self.send(sample,[self.box(color='blue',appearance_status='distractor')]),[])

    def test_geometry_pose_tokens_and_small_size_bounds_are_fail_closed(self):
        variants=(dict(conf=.449),dict(conf=float('nan')),dict(conf=True),
            dict(box=[360.,280.,367.,307.]),dict(box=[360.,280.,394.,307.]),
            dict(box=[360.,280.,374.,303.99]),dict(box=[360.,280.,374.,320.]),
            dict(box=[0.,280.,14.,308.]),dict(box=[740.,280.,752.,308.]),
            dict(box=[360.,3.,374.,31.]),dict(box=[360.,458.,374.,486.]),
            dict(box=[360.,280.,374.]),dict(box=None),dict(width=0),dict(fy=0),
            dict(bearing=float('nan')),dict(foot_down_sin=0.),dict(foot_down_sin=1.1),
            dict(camera_tilt=None),dict(camera_tilt=float('nan')))
        for changes in variants:
            with self.subTest(changes=changes):
                self.tracker=pr.DistantSmallCueTracker()
                for sample in (100.,100.2,100.4):
                    self.assertEqual(self.send(sample,[self.box(**changes)]),[])

    def test_short_span_single_frame_duplicate_and_reverse_cannot_make_votes(self):
        for sample in (100.,100.,99.9,100.05,100.1):
            self.assertEqual(self.send(sample,now=100.2),[])
        self.assertEqual(self.tracker.tracks[0]['samples'],[100.,100.05,100.1])
        self.assertEqual(self.send(100.2,now=100.2),[])
        self.assertEqual(len(self.send(100.4,now=100.5)),1)

    def test_stale_future_badclock_and_gap_do_not_reuse_earlier_streak(self):
        self.send(100.); self.send(100.2)
        self.assertEqual(self.send(100.4,now=100.39),[])
        self.assertEqual(self.send(100.4,now=100.751),[])
        self.assertEqual(self.send(100.6),[])  # gap since last accepted image=.4
        self.assertEqual(self.send(float('nan')),[])
        self.assertEqual(self.send(100.7,now=float('inf')),[])
        self.assertEqual(self.send(True,now=100.8),[])
        self.assertEqual(self.send(1.,now=1.),[])  # backwards clock clears all
        self.assertEqual(self.tracker.tracks,[])
        self.assertEqual(self.send(1.2,now=1.3),[])
        self.assertEqual(self.send(1.4,now=1.5),[])
        self.assertEqual(len(self.send(1.6,now=1.7)),1)

    def test_source_or_epoch_switch_cannot_lend_two_image_votes(self):
        self.send(100.);self.send(100.2)
        self.assertEqual(self.send(100.4,epoch='another_epoch'),[])
        self.assertEqual(len(self.tracker.tracks[0]['samples']),1)
        self.assertEqual(self.send(100.6,source='other_camera'),[])
        self.assertEqual(self.tracker.tracks,[])  # old camera positive token is invalid
        for bad in (None,'',True):
            self.assertEqual(self.send(100.8,epoch=bad),[])

    def test_two_independent_colours_survive_but_same_object_double_colour_does_not(self):
        a=self.box();b=self.box(color='brown',bearing=.4,xy=(500.,280.,514.,308.))
        self.assertEqual(len(self.establish([a,b])),2)
        self.tracker=pr.DistantSmallCueTracker()
        overlap=self.box(color='brown')
        for sample in (100.,100.2,100.4):
            self.assertEqual(self.send(sample,[a,overlap]),[])

    def test_two_separate_red_people_keep_distinct_local_ids(self):
        a=self.box(color='red',bearing=0.,xy=(200.,280.,214.,308.))
        b=self.box(color='red',bearing=.3,xy=(600.,280.,614.,308.))
        cues=self.establish([a,b])
        self.assertEqual(len(cues),2)
        self.assertNotEqual(cues[0]['candidate_id'],cues[1]['candidate_id'])
        self.assertTrue(all(c['candidate_id'].startswith('red:distant:') for c in cues))
        self.assertTrue(all('target_id' not in c for c in cues))

    def test_ambiguous_two_red_histories_or_two_boxes_cannot_share_votes(self):
        a=self.box(color='red',bearing=0.,xy=(200.,280.,214.,308.))
        b=self.box(color='red',bearing=.14,xy=(500.,280.,514.,308.))
        self.assertEqual(len(self.establish([a,b])),2)
        middle=self.box(color='red',bearing=.07)
        self.assertEqual(self.send(100.6,[middle]),[])
        self.tracker=pr.DistantSmallCueTracker();self.establish()
        a=self.box(bearing=-.02,xy=(200.,280.,214.,308.))
        b=self.box(bearing=.02,xy=(500.,280.,514.,308.))
        self.assertEqual(self.send(100.6,[a,b]),[])

    def test_fresh_matching_frame_is_required_and_normal_cue_owns_the_view(self):
        a=self.box()
        reserved=[dict(box=a['box'],candidate_id='green:0')]
        for sample in (100.,100.2,100.4):
            self.assertEqual(self.send(sample,[a],reserved=reserved),[])
        self.assertEqual(self.tracker.tracks,[])

    def test_active_person_clears_accumulation_including_a_duplicate_frame(self):
        self.assertEqual(len(self.establish()),1)
        self.assertEqual(self.send(100.4,active_person=True),[])
        self.assertEqual(self.tracker.tracks,[])
        self.assertEqual(self.send(100.6),[])
        self.assertEqual(self.send(100.8,active_person=True),[])
        self.assertEqual(self.tracker.tracks,[])

    def test_no_small_seed_can_promote_official_geometry(self):
        cue=self.establish()[0]
        cue['attention_only']=False  # losing the advisory flag cannot grant range
        self.assertFalse(pr.pixel_promoted_strict([cue],'green',367.,294.,.95,
            8.,1.8,2.0,80.,350.,390.,280.,752))
        self.assertLess(cue['foot_down_sin'],.30)
        self.assertEqual(pr.fast_tracker_seed_frame([cue],SOURCE,100.4,EPOCH)['detections'],[])

    def test_world_foot_ray_association_keeps_identity_across_pure_camera_pitch(self):
        self.send(100.,[self.box(xy=(360.,100.,374.,128.),camera_tilt=.3)])
        self.send(100.2,[self.box(xy=(360.,200.,374.,228.),camera_tilt=.15)])
        cues=self.send(100.4,[self.box(xy=(360.,300.,374.,328.),camera_tilt=.0058)])
        self.assertEqual(len(cues),1)
        self.assertEqual(cues[0]['seed_started_s'],100.)
        self.assertNotIn('normal_anchor_s',cues[0])

    def test_same_pixel_row_with_different_world_ray_cannot_borrow_identity(self):
        self.send(100.)
        self.send(100.2)
        self.assertEqual(self.send(100.4,[self.box(foot_down_sin=.30)]),[])
        self.assertEqual(self.tracker.tracks[0]['samples'],[100.4])


class DistantSmallAgentTests(unittest.TestCase):
    """Existing consumer protects tasks and bounds this added input path."""
    def setUp(self):
        attention_fixture.AttentionPriorityTests.setUp(self)

    def make_cue(self):
        builder=DistantSmallCueTests();builder.setUp()
        return builder.establish()[0]

    def send_small(self,cue):
        self.a.uav_id=SOURCE
        self.a._visual_candidate_cb(NS(data=json.dumps(cue)))

    def test_generated_small_cue_holds_only_search_and_never_sends_detection(self):
        cue=self.make_cue();self.now=100.5
        before=(copy.deepcopy(self.a._visual_observations),copy.deepcopy(self.a._eligible_observations))
        self.send_small(cue)
        self.assertIsNotNone(self.a._camera_cue)
        deadline=self.a._camera_cue['until']
        self.assertEqual(self.a.assignment.task_type,0)
        self.assertIsNone(self.a._camera_cue['xy'])
        self.a._control()
        self.assertEqual(self.commands,[(0.,0.)])  # no LiDAR, cannot approach
        self.assertEqual(self.detections,[])
        self.now=100.7;cue['sample_s']=100.6
        self.send_small(cue)
        self.assertEqual(self.a._camera_cue['until'],deadline)
        self.assertEqual(before,(self.a._visual_observations,self.a._eligible_observations))
        self.now=deadline+.01
        self.assertFalse(self.a._hold_camera_candidate())

    def test_generated_small_cue_cannot_take_current_person_or_landing(self):
        cue=self.make_cue();self.now=100.5
        for task_type in (1,2,-1):
            self.a.assignment=NS(task_type=task_type,target_id='t3')
            self.send_small(cue)
            self.assertIsNone(self.a._camera_cue)
            self.assertEqual(self.detections,[])

    def test_existing_stable_peer_and_cooldown_still_block_the_new_input(self):
        cue=self.make_cue();self.now=100.5
        self.a._stable_other_owner=lambda *args:True
        self.send_small(cue)
        self.assertIsNone(self.a._camera_cue)
        self.a._stable_other_owner=lambda *args:False
        self.a._camera_cue_retry_after={cue['candidate_id']:106.}
        self.send_small(cue)
        self.assertIsNone(self.a._camera_cue)


if __name__=='__main__':
    unittest.main()
