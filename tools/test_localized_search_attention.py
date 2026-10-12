"""Own qualified images may retain a view while bridge identity matures.

The 014123 trial's two white images are sensor records, not actor truth or
proof of a completed match. No test grants detection/reporting eligibility.
"""
import copy
import json
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

from tools import test_attention_priority as attention_fixture
from tools import test_search_segments as segment_fixture

sa = attention_fixture.sa
FIXTURE = json.loads((Path(__file__).parent / 'fixtures' /
                     'localized_white_search_2812_014123.json').read_text())


class LocalizedSearchAttentionTests(unittest.TestCase):
    def setUp(self):
        attention_fixture.AttentionPriorityTests.setUp(self)
        a = self.a
        a.uav_id = FIXTURE['source_uav']
        a.local_xy = tuple(FIXTURE['world_xy'])
        a.local_z = FIXTURE['local_z']
        a._pose_sample_s = FIXTURE['pose_sample_s']
        a._target_to_orbit = None
        a._friend_positions = {}
        a.state = NS(mode='OFFBOARD')
        a.targets = {'t3': (FIXTURE['old_bridge_hypothesis']['x'],
                            FIXTURE['old_bridge_hypothesis']['y'], 0., 0.)}
        a._t_seen = {'t3': FIXTURE['old_bridge_hypothesis']['sample_s']}
        a._hypothesis_switch_s = {}
        a._segment_control = NS(calls=[])
        a.assignment.target_x, a.assignment.target_y = FIXTURE['search_target']
        self.now = FIXTURE['events'][0]['ros']

    def payload(self, **changes):
        data = copy.deepcopy(FIXTURE['events'][0]['data'])
        data.update(changes)
        return data

    def send(self, data=None, **changes):
        self.a._tracking_view_cb(NS(data=json.dumps(
            self.payload(**changes) if data is None else data)))

    def snapshot(self):
        a = self.a
        return copy.deepcopy((a.targets, a._t_seen, a._visual_observations,
                              a._eligible_observations, a._team_visual,
                              a._team_tracks))

    def test_actual_two_white_images_hold_search_without_bridge_credit(self):
        before = self.snapshot()
        # Before the patch task_type=0 returned without retaining either image.
        self.send(FIXTURE['events'][0]['data'])
        cue = self.a._camera_cue
        self.assertIsNotNone(cue)
        self.assertTrue(cue['localized_search_view'])
        self.assertEqual(cue['xy'], (121.55, -11.51))
        self.assertAlmostEqual(cue['until'], 2816.404)
        self.a._control()
        self.assertEqual(self.commands, [(0., 0.)])
        self.assertEqual(self.a._look_at, (121.55, -11.51))
        self.now = FIXTURE['events'][1]['ros']
        self.send(FIXTURE['events'][1]['data'])
        self.a._control()
        self.assertEqual(self.a._look_at, (121.67, -11.46))
        self.assertAlmostEqual(cue['until'], 2816.404)
        self.assertEqual(cue['attempt']['count'], 1)
        self.assertEqual(before, self.snapshot())
        self.assertEqual(self.detections, [])
        self.assertEqual(self.a.assignment.task_type, 0)

    def test_real_actor_geometry_source_and_freshness_all_required(self):
        variants = (
            dict(source_uav='typhoon_h480_4'), dict(real_detection=False),
            dict(sample_s=self.now-.501), dict(sample_s=self.now+.001),
            dict(sample_s=0.), dict(sample_s=float('nan')),
            dict(geometry_ok=False), dict(report_ok=False),
            dict(geometry_ok=1), dict(report_ok=1),
            dict(appearance_status='ambiguous'), dict(appearance_status='distractor'),
            dict(appearance_status=None), dict(hits=2), dict(confidence=.54),
            dict(confidence=1.1), dict(x=float('nan')), dict(y=float('inf')),
            dict(box=[585.,195.,617.,481.]), dict(track_id=-1),
            dict(world_candidate_id=None), dict(world_candidate_id=''),
            dict(world_candidate_id=6), dict(world_candidate_id='session: 6'),
            dict(world_candidate_id='session:\0'), dict(world_candidate_id='x'*129),
            dict(candidate_id='blue:23'), dict(candidate_id=23),
            dict(target_id='t4'), dict(target_id='t5'), dict(target_id='unknown'))
        before = self.snapshot()
        for change in variants:
            with self.subTest(change=change):
                self.send(**change)
                self.assertIsNone(self.a._camera_cue)
                self.assertEqual(before, self.snapshot())
                self.assertEqual(self.detections, [])

    def test_four_nonred_colors_can_hold_but_never_create_target_evidence(self):
        before=self.snapshot()
        for tid,color in (('t0','green'),('t1','blue'),('t2','brown'),('t3','white')):
            with self.subTest(tid=tid):
                self.a._camera_cue=None
                self.send(target_id=tid,candidate_id=color+':23')
                self.assertEqual(self.a._camera_cue['target_id'],tid)
                self.assertEqual(self.a._camera_cue['color'],color)
                self.assertEqual(before,self.snapshot())
                self.assertEqual(self.detections,[])

    def test_active_person_landing_or_untrusted_pose_cannot_be_preempted(self):
        a = self.a
        cases = (
            ('task', 1), ('task', 2), ('task', 255), ('_landing', True),
            ('_track_assigned_id', 't2'), ('_target_to_orbit', (8.,0.)),
            ('_orbit_target', 'tracking'),
            ('mode', 'AUTO.LAND'), ('mode', 'LAND'), ('mode', 'AUTO.RTL'),
            ('local_xy', None), ('local_z', None), ('trust', False))
        for key, value in cases:
            with self.subTest(key=key, value=value):
                original = (a.assignment.task_type if key=='task' else
                            a.state.mode if key=='mode' else
                            a._world_trusted if key=='trust' else getattr(a,key))
                if key=='task': a.assignment.task_type=value
                elif key=='mode': a.state.mode=value
                elif key=='trust': a._world_trusted=lambda:False
                else: setattr(a,key,value)
                self.send()
                self.assertIsNone(a._camera_cue)
                if key=='task': a.assignment.task_type=original
                elif key=='mode': a.state.mode=original
                elif key=='trust': a._world_trusted=original
                else: setattr(a,key,original)

    def test_current_matching_peer_view_suppresses_duplicate_hold(self):
        a = self.a
        a._team_tracks = {'t3': {'typhoon_h480_2'}}
        a._team_visual = {('typhoon_h480_2','t3'): dict(
            sample_s=self.now-.1, since_s=self.now-.6, hits=4,
            x=121.6, y=-11.5)}
        self.send()
        self.assertIsNone(a._camera_cue)
        # A distant hypothesis of the same colour is not an owner of this view.
        a._team_visual[('typhoon_h480_2','t3')]['x']=61.
        self.now+=.2
        self.send(sample_s=self.now-.1)
        self.assertIsNotNone(a._camera_cue)

    def test_stale_peer_view_does_not_suppress_own_qualified_image(self):
        self.a._team_tracks={'t3':{'typhoon_h480_2'}}
        self.a._team_visual={('typhoon_h480_2','t3'):dict(
            sample_s=self.now-1.01,since_s=self.now-2.,hits=4,
            x=121.55,y=-11.51)}
        self.send()
        self.assertIsNotNone(self.a._camera_cue)

    def test_duplicates_and_older_images_do_not_change_or_restart_hold(self):
        self.send()
        original = copy.deepcopy(self.a._camera_cue)
        self.now+=.1
        self.send(x=122.)  # duplicate image, not a new sample
        self.send(sample_s=2812.1, x=122.)
        self.assertEqual(original,self.a._camera_cue)
        self.assertEqual(self.a._localized_search_view_samples,{'t3':2812.284})
        self.a._hypothesis_switch_s['t3']=self.now
        self.send(sample_s=self.now-.05)
        self.assertEqual(original,self.a._camera_cue)

    def test_new_world_instance_cannot_steal_active_localized_hold(self):
        self.send()
        original = copy.deepcopy(self.a._camera_cue)
        self.now+=.2
        self.send(sample_s=self.now-.1,world_candidate_id='other-session:7',x=122.)
        self.assertEqual(original,self.a._camera_cue)

    def start_pixel(self, cid='white:23'):
        self.a._visual_candidate_cb(NS(data=json.dumps(dict(
            source_uav=self.a.uav_id, real_detection=True,
            purpose='camera_reacquire',candidate_id=cid,hits=3,
            sample_s=2812.284,bearing=-2.127424401808321,confidence=.753,
            box=FIXTURE['events'][0]['data']['box'],width=752,height=480,
            camera_tilt=.3529187240954237))))
        self.assertIsNotNone(self.a._camera_cue)

    def test_same_image_explicit_pixel_binding_promotes_once_without_new_lease(self):
        self.start_pixel()
        cue = self.a._camera_cue
        started, sample, count = cue['started'],cue['sample_s'],cue['attempt']['count']
        self.assertIsNone(cue['xy'])
        self.assertEqual(cue['until'],started+2.)
        self.send(candidate_id='white:23')
        self.assertIs(self.a._camera_cue,cue)
        self.assertEqual((cue['started'],cue['sample_s'],cue['attempt']['count']),
                         (started,sample,count))
        self.assertEqual(cue['xy'],(121.55,-11.51))
        self.assertEqual(cue['until'],started+4.)
        original=copy.deepcopy(cue)
        self.now+=.1
        self.send(candidate_id='white:23',x=122.)
        self.assertEqual(original,cue)
        self.assertEqual(self.detections,[])

    def test_same_image_without_explicit_matching_pixel_id_cannot_promote(self):
        for pixel_id in (None,'white:24'):
            with self.subTest(pixel_id=pixel_id):
                self.a._camera_cue=None
                self.a._camera_cue_retry_after={}
                self.a._camera_attention_attempts=[]
                self.a._localized_search_view_samples={}
                self.start_pixel()
                original=copy.deepcopy(self.a._camera_cue)
                self.send(candidate_id=pixel_id)
                self.assertEqual(original,self.a._camera_cue)

    def test_continuous_new_images_never_extend_absolute_four_second_deadline(self):
        self.send()
        started=self.now
        for elapsed in (.2,.8,1.5,2.8,3.9):
            self.now=started+elapsed
            self.send(sample_s=self.now-.1)
            self.assertTrue(self.a._hold_camera_candidate())
            self.assertAlmostEqual(self.a._camera_cue['until'],started+4.)
        self.now=started+4.01
        self.assertFalse(self.a._hold_camera_candidate())
        self.assertEqual(len(self.revisits),1)
        self.send(sample_s=self.now-.1)
        self.assertIsNone(self.a._camera_cue)  # per-view cooldown still applies
        self.assertEqual(self.detections,[])

    def test_hold_release_resumes_latest_versioned_segment_without_old_task(self):
        a=self.a
        # Use the real segment protocol, then let attention suspend its control.
        a._plan_lock=threading.Lock();a._plan_epoch=0;a.path=[]
        a._plan_pending=None;a.path_target=None;a._plan_fail_t=0.
        a._last_flight_v=(0.,0.)
        helper=segment_fixture.AgentSegmentTests()
        old=helper.message();old.uav_id=a.uav_id
        old.header.stamp=sa.rospy.Time.from_sec(self.now-.1)
        old.search_valid_until=sa.rospy.Time.from_sec(self.now+19.)
        a._assign_cb(old)
        self.send()
        a._segment_control=lambda *args: self.fail('search moved during camera hold')
        a._control()
        newer=copy.deepcopy(old);newer.search_revision=2
        newer.header.stamp=sa.rospy.Time.from_sec(self.now+.2)
        self.now+=.2
        a._assign_cb(newer)
        self.assertTrue(a._hold_camera_candidate())
        self.now+=4.
        self.assertFalse(a._hold_camera_candidate())
        self.assertEqual(a.assignment.search_revision,2)
        self.assertEqual(a._search_segment['revision'],2)
        a._assign_cb(old)
        self.assertEqual(a.assignment.search_revision,2)
        self.assertEqual(a._search_segment['revision'],2)


if __name__=='__main__':
    unittest.main()
