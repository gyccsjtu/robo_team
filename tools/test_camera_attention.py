"""Discovery/control separation, bounded approach, and image cadence."""
import math
import json
import sys
import unittest
import numpy as np
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
from camera_attention import approach_image, candidate_velocity, candidate_motion, ImagePublishClock
from perception import perception_real as pr


class CameraAttentionTests(unittest.TestCase):
    def test_off_centre_pixel_ray_preserves_roll_coupling(self):
        angle = .25
        rotation = np.array([[1.,0.,0.],[0.,math.cos(angle),-math.sin(angle)],
                             [0.,math.sin(angle),math.cos(angle)]])
        ray = rotation @ (pr.M_OPT2LINK @ np.array([0.,.4,1.]))
        actual = pr.image_world_bearing(pr.CX,pr.CY+.4*pr.FY,rotation)
        self.assertAlmostEqual(actual,math.atan2(ray[1],ray[0]))
        self.assertGreater(abs(actual-pr.image_world_bearing(pr.CX,pr.CY,rotation)),.05)

    def image(self, **changes):
        data = dict(box=[340., 200., 380., 300.], width=752, height=480, camera_tilt=0.)
        data.update(changes)
        return data

    def cue(self):
        return dict(image=approach_image(self.image()), approach_origin=(0., 0.),
                    sample_s=100., started=99., bearing=0.)

    def scan(self, **changes):
        data = dict(angle_min=-math.pi, angle_increment=math.pi/180,
                    ranges=[float('inf')]*361, range_min=.1, range_max=30.)
        data.update(changes)
        return NS(**data)

    def test_unknown_range_only_requests_bounded_velocity(self):
        cue = self.cue()
        vx, vy = candidate_velocity(cue, 100.1, 0., (0., 0.), self.scan(), 100.1)
        self.assertGreater(vx, 0.)
        self.assertLessEqual(vx, .95)
        self.assertEqual(vy, 0.)
        self.assertNotIn('xy', cue)

    def test_clipped_large_tilted_or_missing_image_cannot_approach(self):
        for changes in (dict(box=[340., 100., 380., 480.]),
                        dict(box=[0., 200., 40., 300.]),
                        dict(box=[340., 100., 380., 310.]),
                        dict(camera_tilt=.3), dict(camera_tilt=float('nan'))):
            self.assertIsNone(approach_image(self.image(**changes)))
        self.assertIsNone(approach_image({}))

    def test_missing_stale_obstructed_or_invalid_scan_stops(self):
        scans = [None, self.scan(range_max=3.),
                 self.scan(ranges=[float('nan')]*361),
                 self.scan(ranges=[float('-inf')]*361),
                 self.scan(ranges=[.05]*361), self.scan(ranges=[3.9]*361)]
        for scan in scans:
            self.assertEqual(candidate_velocity(self.cue(),100.1,0.,(0.,0.),scan,100.),(0.,0.))
        self.assertEqual(candidate_velocity(self.cue(),100.4,0.,(0.,0.),self.scan(),99.),(0.,0.))

    def test_time_travel_yaw_and_image_age_are_hard_limits(self):
        for now, yaw, xy in ((105.1,0.,(0.,0.)),(100.1,.5,(0.,0.)),
                              (100.1,0.,(3.,0.)),(100.4,0.,(0.,0.))):
            self.assertEqual(candidate_velocity(self.cue(),now,yaw,xy,self.scan(),now),(0.,0.))

    def test_localized_candidate_waits_for_manager_instead_of_pixel_approach(self):
        cue = self.cue()
        cue['xy'] = (8., 0.)
        self.assertEqual(candidate_velocity(cue,100.1,0.,(0.,0.),self.scan(),100.),(0.,0.))

    def test_four_hz_on_five_hz_images_does_not_alias_to_two_point_five(self):
        clock = ImagePublishClock(4.)
        images = [100.+i*.2 for i in range(51)]
        accepted = [t for t in images if clock.ready(t)]
        self.assertEqual(len(accepted), 41)
        self.assertLessEqual(max(b-a for a,b in zip(accepted,accepted[1:])), .400001)
        self.assertFalse(clock.ready(images[-1]))
        self.assertFalse(clock.ready(images[-2]))
        self.assertFalse(clock.ready(float('nan')))

    def test_cadence_has_no_catchup_burst_after_gap(self):
        clock = ImagePublishClock(4.)
        self.assertTrue(clock.ready(100.))
        self.assertTrue(clock.ready(200.01))
        self.assertFalse(clock.ready(200.02))
        self.assertFalse(clock.ready(200.03))

    def track(self, color, x, confidence=.7):
        t = pr.Track(color,x,0.,confidence,(376.,300.),(40.,100.),8.,1.7,100.)
        t.hits = 3
        return t

    def test_two_same_color_control_tracks_survive_report_verdict(self):
        first, second = self.track('blue', 8.), self.track('blue', 17., .5)
        first.verdict = second.verdict = lambda *a, **k: (False, 'static')
        chosen = pr.control_tracks([first, second], [None,None], 100.)
        self.assertEqual([t.id for _,_,t in chosen], [first.id,second.id])
        self.assertEqual(pr.control_tracks([first,second],[None,None],100.2), [])
        first.miss = 1
        self.assertEqual(len(pr.control_tracks([first,second],[None,None],100.)),1)

    def test_red_control_requires_established_slot_identity(self):
        t = self.track('red',8.)
        self.assertEqual(pr.control_tracks([t],[None,None],100.), [])
        self.assertEqual(pr.control_tracks([t],[None,t.id],100.)[0][:2], ('red',1))

    def test_pixel_association_preserves_full_frame_metadata(self):
        tracker = pr.PixelCueTracker()
        box = dict(cls='brown', conf=.8, u=360., v=250., h=100.,
                   bearing=0., cam_x=0., cam_y=0., **self.image())
        for t in (100.,100.2,100.4):
            cues = tracker.update(t,[box])
        self.assertEqual(len(cues),1)
        self.assertEqual(cues[0]['box'],box['box'])
        self.assertEqual(cues[0]['height'],480)

    def pixel_box(self, coords, confidence=.8):
        x1,y1,x2,y2 = coords
        return dict(cls='red', conf=confidence, u=(x1+x2)/2, v=(y1+y2)/2,
                    h=y2-y1, bearing=0., cam_x=0., cam_y=0., box=coords,
                    seed_eligible=pr.pixel_cue_box_ok('red', confidence, *coords),
                    width=752, height=480, camera_tilt=0.)

    def test_logged_red_track_continues_when_box_shrinks_below_seed_size(self):
        fixture=json.loads((ROOT/'tools/fixtures/pixel_red_143110.json').read_text())
        tracker=pr.PixelCueTracker()
        continued=0
        for index, frame in enumerate(fixture['frames']):
            box=self.pixel_box(frame['box'], frame['confidence'])
            self.assertTrue(pr.pixel_cue_box_ok('red', frame['confidence'],
                                               *frame['box'], continuation=True))
            cues=tracker.update(frame['sample_s'], [box])
            if index < 2:
                self.assertFalse(cues)
                continue
            self.assertEqual(len(cues),1)
            self.assertEqual(cues[0]['candidate_id'],'red:0')
            if not box['seed_eligible']:
                continued+=1
                self.assertTrue(cues[0]['attention_only'])
        self.assertEqual(continued,26)

    def test_small_boxes_cannot_start_or_complete_unconfirmed_seed(self):
        small=self.pixel_box([300.,280.,310.,310.])
        big=self.pixel_box([300.,270.,314.,314.])
        tracker=pr.PixelCueTracker()
        for t in (100.,100.2,100.4,100.6):
            self.assertFalse(tracker.update(t,[small]))
        self.assertFalse(tracker.tracks)
        tracker.update(101.,[big])
        for t in (101.2,101.4,101.6):
            self.assertFalse(tracker.update(t,[small]))

    def test_small_continuation_cannot_reuse_other_red_or_expired_track(self):
        big=self.pixel_box([300.,270.,314.,314.])
        small=self.pixel_box([300.,280.,310.,310.])
        tracker=pr.PixelCueTracker()
        for t in (100.,100.2,100.4):tracker.update(t,[big])
        other=dict(small,bearing=.7)
        self.assertFalse(tracker.update(100.6,[other]))
        self.assertFalse(tracker.update(102.,[small]))

    def test_gate_reason_distinguishes_stale_image_from_obstacle(self):
        self.assertEqual(candidate_motion(self.cue(),100.4,0.,(0.,0.),
            self.scan(),100.4)[2],'stale_image')
        self.assertEqual(candidate_motion(self.cue(),100.1,0.,(0.,0.),
            self.scan(ranges=[3.9]*361),100.1)[2],'obstacle')


if __name__ == '__main__':
    unittest.main()
