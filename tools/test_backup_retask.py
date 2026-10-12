"""Replay the 17:59 run's camera blocked by its obsolete backup role."""
import copy
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
import swarm_manager as sm


class BackupRetaskTests(unittest.TestCase):
    def setUp(self):
        fixture = json.loads((ROOT / 'tools/fixtures/backup_new_person_175916.json').read_text())
        self.now = fixture['now']
        clock = patch.object(sm.rospy.Time, 'now', return_value=sm.rospy.Time.from_sec(self.now))
        clock.start()
        self.addCleanup(clock.stop)
        m = self.m = sm.SwarmManager.__new__(sm.SwarmManager)
        m.uav_ids = list(fixture['status'])
        m.status = {uid: NS(**row, connected=True) for uid, row in fixture['status'].items()}
        m.last_report = {uid: sm.rospy.Time.from_sec(self.now) for uid in m.uav_ids}
        m._tracking, m._backup = fixture['tracking'], fixture['backup']
        m._tracking_since = {'t2': 2091.968, 't5': 2139.968, 't1': 2141.968}
        m._release_cooldown = {}
        m._visual_observations = {}
        m._eligible_observations = {(d['source_uav'], d['target_id']): d for d in fixture['eligible']}
        m._backup_approach_leases = {'t2': {'uid': 'typhoon_h480_3'}}
        m._free_search_lease = Mock()
        m.assign_pub = Mock()
        m.tracker = NS(targets={tid: NS(observers={uid}) for tid, uid in m._tracking.items()})
        m.tracker.targets['t2'].observers.add('typhoon_h480_3')
        self.image = m._eligible_observations[('typhoon_h480_3', 't5')]

    def handoff(self):
        self.m._handoff_distant_tracker('t5', self.image['x'], self.image['y'],
                                      self.m.tracker.targets['t5'])

    def test_recorded_backup_hands_new_red_to_seeing_camera_preserving_brown_primary(self):
        self.handoff()
        m = self.m
        self.assertEqual(m._tracking, {'t2': 'typhoon_h480_5', 't5': 'typhoon_h480_3',
                                       't1': 'typhoon_h480_1'})
        self.assertEqual(m._backup, {})
        self.assertNotIn('t2', m._backup_approach_leases)
        self.assertEqual(m.tracker.targets['t2'].observers, {'typhoon_h480_5'})
        messages = [c.args[0] for c in m.assign_pub.publish.call_args_list]
        self.assertEqual([(d.uav_id, d.target_id, d.task_type) for d in messages], [
            ('typhoon_h480_3', 't5', 1), ('typhoon_h480_3', 't2', 255),
            ('typhoon_h480_2', 't5', 255)])
        self.assertNotIn('typhoon_h480_3', m._release_cooldown)

    def test_sole_remaining_camera_for_old_target_is_protected(self):
        old = self.m._eligible_observations[('typhoon_h480_3', 't2')]
        old['sample_s'] = self.now - .1
        self.handoff()
        self.assertEqual(self.m._tracking['t5'], 'typhoon_h480_2')
        self.assertEqual(self.m._backup, {'t2': 'typhoon_h480_3'})
        self.m.assign_pub.publish.assert_not_called()

    def test_healthy_old_primary_allows_backup_to_follow_another_person(self):
        for uid in ('typhoon_h480_3', 'typhoon_h480_5'):
            self.m._eligible_observations[(uid, 't2')]['sample_s'] = self.now - .1
        self.handoff()
        self.assertEqual(self.m._tracking['t2'], 'typhoon_h480_5')
        self.assertEqual(self.m._tracking['t5'], 'typhoon_h480_3')

    def test_new_target_existing_camera_owner_is_not_interrupted(self):
        self.m._eligible_observations[('typhoon_h480_2', 't5')] = dict(self.image)
        self.handoff()
        self.m.assign_pub.publish.assert_not_called()

    def test_primary_with_own_fresh_old_person_view_cannot_be_borrowed(self):
        self.m._backup.clear()
        self.m._tracking['t2'] = 'typhoon_h480_3'
        self.m._eligible_observations[('typhoon_h480_3', 't2')]['sample_s'] = self.now-.1
        self.handoff()
        self.m.assign_pub.publish.assert_not_called()

    def test_blind_primary_with_no_recent_old_view_may_take_its_new_person(self):
        # The deployed VISUAL_OWNER_RETASK policy already permits this case.
        # A role label without a recent image does not prove a judge streak.
        self.m._backup.clear()
        self.m._tracking['t2'] = 'typhoon_h480_3'
        self.handoff()
        self.assertEqual(self.m._tracking['t5'], 'typhoon_h480_3')
        self.assertNotIn('t2', self.m._tracking)
        self.assertIn('t2', self.m.tracker.targets)  # pending, never declared eliminated
        messages = [c.args[0] for c in self.m.assign_pub.publish.call_args_list]
        self.assertEqual([(d.uav_id, d.target_id, d.task_type) for d in messages], [
            ('typhoon_h480_3', 't5', 1), ('typhoon_h480_3', 't2', 255),
            ('typhoon_h480_2', 't5', 255)])

    def test_single_stale_future_or_different_place_images_do_not_retask(self):
        original = copy.deepcopy(self.image)
        for changes in ({'hits': 1}, {'since_s': original['sample_s']},
                        {'sample_s': self.now-1.1}, {'sample_s': self.now+.1},
                        {'x': original['x']+10}):
            self.image.clear()
            self.image.update(original, **changes)
            self.m._handoff_distant_tracker('t5', original['x'], original['y'],
                                          self.m.tracker.targets['t5'])
            self.m.assign_pub.publish.assert_not_called()

    def test_unavailable_old_primary_blocks_backup_retask(self):
        self.m.status['typhoon_h480_5'].connected = False
        self.handoff()
        self.m.assign_pub.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
