"""Passive report evidence cannot steer, renew imagery or confirm a person."""
import copy
import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import test_unavailable_tracking as fixtures
import swarm_manager
from formal_report_flow import ReportFlowHealth


class ReportFlowManagerTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.UnavailableTrackingTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def manager(self):
        manager, target = self.fixture._backup_manager(99.8)
        manager._formal_report_flow = ReportFlowHealth(manager.status)
        manager._formal_report_log_s = {}
        image = dict(target_id='t0', source_uav='u0', candidate_id='green:A',
                     sample_s=99.8, eligible=True, x=0., y=0.)
        manager._formal_report_flow.remember_image(image, 99.9)
        return manager, target

    def published(self, **changes):
        data = dict(event='published', target_id='t0', tag='green',
                    source_uav='u0', candidate_id='green:A', sample_s=99.8,
                    report_s=99.95, report_policy='fresh', x=0., y=0.)
        data.update(changes)
        return SimpleNamespace(data=json.dumps(data))

    def test_actual_publish_changes_only_passive_book(self):
        manager, target = self.manager()
        imagery = copy.deepcopy(manager._visual_observations)
        progress = (target.confirm_since, target.last_ok_t, target.eliminated)
        manager._formal_report_cb(self.published())
        record = manager._formal_report_flow.latest('t0', 100.)
        self.assertEqual(record['sample_s'], 99.8)
        self.assertEqual(manager._visual_observations, imagery)
        self.assertEqual((target.confirm_since, target.last_ok_t, target.eliminated),
                         progress)
        self.assertEqual(manager._tracking, {'t0': 'u0'})
        self.assertEqual(manager._backup, {})
        manager.assign_pub.publish.assert_not_called()

    def test_accepted_visual_without_publish_does_not_add_backup_behavior(self):
        manager, target = self.manager()
        self.assertIsNone(manager._formal_report_flow.latest('t0', 100.))
        manager._dispatch_backup()
        self.assertEqual(manager._backup, {})
        self.assertFalse(target.eliminated)
        manager.assign_pub.publish.assert_not_called()

    def test_disabled_or_mismatched_evidence_cannot_change_tasks(self):
        for disabled, payload in ((True, self.published()),
                                  (False, self.published(candidate_id='green:B')),
                                  (False, self.published(event='observed'))):
            with self.subTest(disabled=disabled, payload=payload.data):
                manager, target = self.manager()
                if disabled:
                    manager._formal_report_flow = None
                manager._formal_report_cb(payload)
                if not disabled:
                    self.assertIsNone(manager._formal_report_flow.latest('t0', 100.))
                self.assertEqual(manager._tracking, {'t0': 'u0'})
                self.assertEqual(manager._backup, {})
                self.assertFalse(target.eliminated)
                manager.assign_pub.publish.assert_not_called()

    def test_hypothesis_reset_does_not_advance_clock_past_current_callback(self):
        manager, _ = self.manager()
        event = dict(switch_s=100., x=10., y=0.)
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time(100.05)):
            manager._reset_visual_hypothesis('t0', event)
        image = dict(target_id='t0', source_uav='u0', candidate_id='green:B',
                     sample_s=100., eligible=True, x=10., y=0.)
        self.assertTrue(manager._formal_report_flow.remember_image(image, 100.))


if __name__ == '__main__':
    unittest.main()
