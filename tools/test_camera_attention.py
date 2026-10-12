"""Discovery/control separation, bounded approach, and image cadence."""
import math
import sys
import unittest
import numpy as np
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
from camera_attention import approach_image, candidate_velocity, ImagePublishClock
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

    def recorded_weak_white(self, **changes):
        # 225123/perception_typhoon_h480_2.csv: image 2290.700 was
        # strict=True at raw confidence .4226698, range 11.254201 m;
        # track_publish confirms existing track 6 had 30 lifetime hits.
        # Establish identity with actual Track.update calls before replaying
        # this final observation. This is a producer fixture, not actor truth.
        sample = 2290.700
        x, y = 89.17203847186536, 41.85064785998252
        uv = (384.2190399169922, 299.99767684936523)
        wh = (26.834686279296875, 57.48176574707031)
        rng, height = 11.254201043554712, 1.7327573441522528
        tr = pr.Track('white', x-.6, y, .8, uv, wh, rng, height, sample-1.2)
        for i in range(1, 6):
            tr.update(x-.6+.1*i, y, .2, .8, uv, wh, rng, height, sample-1.2+.2*i)
        self.assertTrue(tr.world_candidate_ready())
        tr.update(x, y, .2, .42266982793807983, uv, wh, rng, height, sample)
        for key, value in changes.items():
            setattr(tr, key, value)
        return tr, sample

    def test_recorded_mature_white_reaches_camera_control_at_detector_threshold(self):
        for confidence in (.30, .35, .42266982793807983, .423, .44):
            tr, sample = self.recorded_weak_white(conf=confidence)
            before = dict(tr.__dict__)
            chosen = pr.control_tracks([tr], [None, None], sample)
            self.assertEqual(chosen, [('white', 0, tr)])
            self.assertIs(chosen[0][2], tr)
            self.assertEqual(pr.candidate_identity('session', chosen[0][2]),
                             pr.candidate_identity('session', tr))
            self.assertEqual(tr.__dict__, before)
            # The dispatch .55 floor remains independent of image control.
            self.assertEqual(pr.discovery_candidate_frame(
                chosen, 'u2', sample, 'session')['candidates'], [])

    def test_weak_white_cannot_create_identity_or_use_failed_current_geometry(self):
        for changes in (dict(hits=1), dict(hits=3), dict(world_ready=False),
                        dict(miss=1), dict(observation_view_ok=False),
                        dict(loc_suspect=True), dict(appearance_status='ambiguous'),
                        dict(appearance_status='distractor'), dict(rng=12.01),
                        dict(range_method='head_height'), dict(last_strict_s=2288.699),
                        dict(last_strict_s=2290.701), dict(conf=.29),
                        dict(conf=math.nan), dict(observed_s=2290.5)):
            with self.subTest(changes=changes):
                tr, sample = self.recorded_weak_white(**changes)
                self.assertEqual(pr.control_tracks([tr], [None, None], sample), [])
        for color in ('red', 'blue', 'green', 'brown'):
            tr, sample = self.recorded_weak_white(cls=color)
            self.assertEqual(pr.control_tracks([tr], [None, tr.id], sample), [])

    def test_weak_white_control_cannot_upgrade_report_geometry_or_other_instance(self):
        tr, sample = self.recorded_weak_white(observation_loc_ok=False)
        tr.last_strict_s = sample-.2
        self.assertFalse(tr.can_report_image())
        peer = self.track('white', 90., .44)  # unestablished nearby white box
        peer.observed_s = sample
        chosen = pr.control_tracks([peer, tr], [None, None], sample)
        self.assertEqual(chosen, [('white', 0, tr)])
        self.assertFalse(tr.can_report_image())
        self.assertEqual(peer.hits, 3)
        self.assertFalse(peer.world_ready)
        # Camera selection does not invoke the separate report-candidate path.
        self.assertFalse(pr.report_candidate_frame(chosen, 'u2', sample,
            'session', (80., 40.), assigned_target='t3')['candidates'][0]['official_view_ok'])

    def test_pixel_association_preserves_full_frame_metadata(self):
        tracker = pr.PixelCueTracker()
        box = dict(cls='brown', conf=.8, u=360., v=250., h=100.,
                   bearing=0., cam_x=0., cam_y=0., **self.image())
        for t in (100.,100.2,100.4):
            cues = tracker.update(t,[box])
        self.assertEqual(len(cues),1)
        self.assertEqual(cues[0]['box'],box['box'])
        self.assertEqual(cues[0]['height'],480)


class CandidateSideLookTests(unittest.TestCase):
    def test_near_wall_allows_only_bounded_sensor_checked_side_look(self):
        from camera_attention import candidate_reposition
        from types import SimpleNamespace as NS
        cue = dict(sample_s=100., started=99., bearing=0., approach_origin=(0.,0.),
                   image=dict(box=[300.,150.,360.,250.], height=480.))
        scan = NS(angle_min=-math.pi*3/4., angle_increment=math.pi/180.,
                  ranges=[10.]*271, range_min=.1, range_max=12.)
        scan.ranges[135] = 2.
        grid = NS(world_to_cell=lambda p:p, is_free=lambda p:True)
        vx, vy, reason = candidate_reposition(cue,100.,0.,(0.,0.),scan,100.,grid)
        self.assertAlmostEqual(vx,0.)
        self.assertAlmostEqual(abs(vy),.35)
        self.assertEqual(reason,'side_reobserve')
        cue['sample_s'] = 101.6
        self.assertEqual(candidate_reposition(cue,101.6,0.,(0.,.4),scan,101.6,grid)[:2],(0.,0.))
        cue.pop('reposition')
        self.assertEqual(candidate_reposition(cue,101.6,0.,(0.,.4),scan,101.,grid)[:2],(0.,0.))
        grid.is_free = lambda p:False
        self.assertEqual(candidate_reposition(cue,101.6,0.,(0.,.4),scan,101.6,grid)[:2],(0.,0.))
        self.assertNotIn('reposition',cue)


if __name__ == '__main__':
    unittest.main()
