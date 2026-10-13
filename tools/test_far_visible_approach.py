"""Recorded 13 m green is a flight intent, not permission to report it."""
import json
import math
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

from test_joint_dispatch import JointDispatchTests, sm
from test_visible_person_follow import VisibleFollowTests, sa

FIXTURE = Path(__file__).parent/'fixtures/green_far_visible_071332.json'


class FarVisibleControlTests(unittest.TestCase):
    setUp = VisibleFollowTests.setUp
    payload = VisibleFollowTests.payload

    def test_actual_first_green_view_immediately_requests_approach_without_report(self):
        fixture = json.loads(FIXTURE.read_text())
        raw = fixture['views'][0]['data']
        pose = fixture['status'][0]['data']
        self.a.uav_id = raw['source_uav']
        self.a.local_xy = (pose['x'], pose['y'])
        self.a.local_z = pose['z']
        self.a.yaw = raw['bearing']
        self.now = raw['sample_s']+.1
        self.a._scan_t = self.now
        self.a._tracking_view_cb(NS(data=json.dumps(raw)))
        self.assertEqual(len(self.a._camera_cue['follow_samples']), 1)
        distance = math.dist(self.a.world_xy, (raw['x'], raw['y']))
        self.assertGreater(distance, 12.)
        self.a._hold_camera_candidate()
        self.assertGreater(math.hypot(*self.commands[-1]), 0.)
        self.assertLessEqual(math.hypot(*self.commands[-1]), .95+1e-9)
        dx, dy = raw['x']-pose['x'], raw['y']-pose['y']
        self.assertGreater(self.commands[-1][0]*dx+self.commands[-1][1]*dy, 0.)
        self.assertEqual(self.detections, [])
        self.assertEqual(self.a._eligible_observations, {})

    def test_stationary_far_person_requests_approach_instead_of_waiting_for_motion(self):
        for i in range(3):
            self.now = 100.+.2*i
            self.a._scan_t = self.now
            self.a._tracking_view_cb(NS(data=json.dumps(self.payload(13.))))
        self.a._hold_camera_candidate()
        self.assertGreater(self.commands[-1][0], 0.)
        self.assertEqual(self.detections, [])

    def test_distance_is_not_a_veto_but_expired_image_or_blocked_scan_still_stops(self):
        self.a._tracking_view_cb(NS(data=json.dumps(self.payload(26.))))
        self.a._scan_t = self.now
        self.a._hold_camera_candidate()
        self.assertGreater(self.commands[-1][0], 0.)
        self.a._scan.ranges = [3.5]*361
        # Obstacle may trigger a checked reposition; no unguarded forward intent.
        self.a.grid = None
        self.a._hold_camera_candidate()
        self.assertEqual(self.commands[-1], (0., 0.))
        self.now += .4
        self.a._scan_t = self.now
        self.a._scan.ranges = [float('inf')]*361
        self.a._hold_camera_candidate()
        self.assertEqual(self.commands[-1], (0., 0.))

    def test_other_target_assignment_keeps_its_camera_and_has_no_search_approach(self):
        self.a.assignment = NS(task_type=1, target_id='t3')
        self.a._tracking_view_cb(NS(data=json.dumps(self.payload(13.))))
        self.assertIsNone(self.a._camera_cue)
        self.assertEqual(self.a.assignment.target_id, 't3')


class FarVisibleDispatchTests(unittest.TestCase):
    setUp = JointDispatchTests.setUp
    _east_visual_owner_scenario = JointDispatchTests._east_visual_owner_scenario
    _east_white_frame = JointDispatchTests._east_white_frame

    def test_original_green_bridge_packets_choose_seeing_free_camera_not_blind_owner(self):
        fixture = json.loads(FIXTURE.read_text())
        m = self._east_visual_owner_scenario()
        uid, old = 'typhoon_h480_0', 'typhoon_h480_3'
        m.uav_ids = [uid, old]
        m.status = {}
        m._tracking = {'t0': old}
        m._tracking_since = {'t0': 2766.744}
        m.tracker.targets['t0'].observers = {old}
        for event in fixture['accepted']:
            now = event['ros']
            for name in m.uav_ids:
                snapshots = [r for r in fixture['status']
                             if r['data']['uav'] == name and r['ros'] <= now]
                row = snapshots[-1]['data']
                m.status[name] = NS(x=row['x'], y=row['y'], z=row['z'], connected=True)
            m.last_report = {name: sm.rospy.Time.from_sec(now) for name in m.uav_ids}
            raw = event['data']
            with patch.object(sm.rospy.Time, 'now', return_value=sm.rospy.Time.from_sec(now)):
                m._visual_source_cb(NS(data=json.dumps(raw)))
                m._handoff_distant_tracker('t0', raw['x'], raw['y'], m.tracker.targets['t0'])
        self.assertEqual(m._tracking['t0'], uid)
        self.assertEqual(m.tracker.targets['t0'].observers, {uid})
        self.assertEqual(m._eligible_observations, {})
        self.assertIsNone(m.tracker.targets['t0'].confirm_since)

    def test_same_camera_can_take_ownership_beyond_old_twenty_metre_gate(self):
        m = self._east_visual_owner_scenario()
        m.status['u2'].x, m.status['u2'].y = 145., 24.
        for sample in (100., 100.4, 100.8):
            self._east_white_frame(m, sample, x=121., source='u2', distance=24.)
        self.assertEqual(m._tracking['t3'], 'u2')
        self.assertNotIn(('u2', 't3'), m._eligible_observations)
        self.assertIsNone(m.tracker.targets['t3'].confirm_since)


if __name__ == '__main__':
    unittest.main()
