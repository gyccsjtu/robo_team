"""Offline regressions for a fixed MAVROS-to-official world frame.

Run after sourcing ROS Noetic and the catkin workspace; no simulator is used.
"""

import ast
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
AGENT_FILE = ROOT / 'coordination/src/robocup_swarm/scripts/swarm_agent.py'
sys.path.insert(0, str(AGENT_FILE.parent))

import swarm_agent


def _pose(x, y, z=4.0, sample=None):
    stamp = swarm_agent.rospy.Time.now() if sample is None else swarm_agent.rospy.Time.from_sec(sample)
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp), pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))


class LocalizationFailClosedTests(unittest.TestCase):
    def setUp(self):
        self.clock = [100.0]
        self.now_patch = patch.object(
            swarm_agent.rospy.Time, 'now',
            side_effect=lambda: swarm_agent.rospy.Time(self.clock[0]))
        self.now_patch.start()
        self.addCleanup(self.now_patch.stop)

    def _agent(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'typhoon_h480_0'
        agent.offset = (0.0, -3.0)
        agent._offset_param = agent.offset
        agent.local_xy = (0.0, 0.0)
        agent.local_z = 4.0
        agent._local_prev_t = self.clock[0]
        agent._pose_sample_s = self.clock[0]
        agent._anchor_done = True
        agent._localization_fault = False
        agent._ekf_window = [((0.0, 0.0), self.clock[0])]
        agent._est_hist = []
        agent._last_cmd_v = (1.0, 0.0)
        agent._last_flight_v = (1.0, 0.0)
        agent._last_good_world = (0.0, -3.0)
        return agent

    def test_bunched_callbacks_preserve_actual_motion_time(self):
        agent = self._agent()
        agent._last_cmd_v = (2., 0.)
        agent._est_hist = [(100., 0., 0.)]
        # Two metres/second, delayed by a variable amount; arrivals catch up
        # within milliseconds. The old receipt dt declared an EKF jump.
        for sample, receipt, x in ((100.2,100.48,.4),
                                   (100.4,100.485,.8),
                                   (100.6,100.61,1.2)):
            self.clock[0] = receipt
            agent._local_cb(_pose(x, 0., sample=sample))
            self.assertFalse(agent._localization_fault)
            self.assertAlmostEqual(agent._est_recent_speed(),2.)
            self.assertAlmostEqual(agent._local_prev_t,receipt)
            self.assertAlmostEqual(agent._pose_sample_s,sample)

    def test_delayed_real_jump_still_latches(self):
        agent = self._agent()
        self.clock[0] = 100.4
        agent._local_cb(_pose(.73, 0., sample=100.036))
        self.assertTrue(agent._localization_fault)
        self.assertIsNone(agent.world_xy)

    def test_rejected_samples_cannot_change_pose_altitude_or_history(self):
        agent = self._agent()
        original = list(agent._ekf_window)
        for sample in (100., 99.9, 0., 100.3, 99.):
            self.clock[0] = 100.2
            agent._local_cb(_pose(100., 100., z=100., sample=sample))
            self.assertEqual(agent.local_xy,(0.,0.))
            self.assertEqual(agent.local_z,4.)
            self.assertEqual(agent._ekf_window,original)
            self.assertEqual(agent._local_prev_t,100.)
            self.assertEqual(agent._pose_sample_s,100.)
            self.assertFalse(agent._localization_fault)

    def test_bad_pose_never_refreshes_receipt_even_with_safety_flag_off(self):
        agent = self._agent()
        self.clock[0] = 100.2
        with patch.object(swarm_agent,'MOTION_SAFETY',False):
            agent._local_cb(_pose(float('nan'),0.,sample=100.1))
        self.assertEqual(agent._local_prev_t,100.)
        self.assertEqual(agent._pose_sample_s,100.)

    def test_fixed_offset_does_not_move_with_pose_reads_or_ekf_jump(self):
        agent = self._agent()
        for t, x in ((100.05, 0.05), (100.10, 0.10)):
            self.clock[0] = t
            agent._local_cb(_pose(x, 0.0))
            self.assertEqual(agent.world_xy, (x, -3.0))
            self.assertEqual(agent.world_xy, (x, -3.0))
            self.assertEqual(agent.offset, (0.0, -3.0))
        # The 23:30 match produced 0.52m per 0.032s during UAV3 EKF failure.
        self.clock[0] = 100.132
        agent._local_cb(_pose(0.62, 0.0))
        self.assertTrue(agent._localization_fault)
        self.assertIsNone(agent.world_xy)
        self.assertEqual(agent.offset, (0.0, -3.0))
        self.clock[0] = 100.20
        agent._local_cb(_pose(0.20, 0.0))
        self.assertTrue(agent._localization_fault)  # no self-anchored recovery
        self.assertIsNone(agent.world_xy)

    def test_fault_stops_task_and_reports_disconnected(self):
        agent = self._agent()
        agent._mark_localization_fault('test EKF jump')
        agent._takeoff_done = True
        agent._crash_flagged = False
        agent._est_hold_n = swarm_agent.EST_FROZEN_N - 1
        agent._ekf_frozen = False
        agent.altitude_layer = 4.0
        agent.state = SimpleNamespace(connected=True, mode='OFFBOARD')
        agent.mode_srv = Mock()
        agent._send_vel(3.0, -2.0, vz=1.0)
        agent.mode_srv.assert_called_once_with(custom_mode='AUTO.LAND')
        self.assertEqual(agent._last_cmd_v, (0.0, 0.0))
        self.assertFalse(agent._est_trustworthy())

        agent.status_pub = Mock()
        agent.assignment = None
        agent._publish_status()
        status = agent.status_pub.publish.call_args.args[0]
        self.assertFalse(status.connected)
        self.assertEqual((status.x, status.y), (0.0, -3.0))
        self.assertEqual((status.cell_ix, status.cell_iy), (-1, -1))

    def test_runtime_offset_assignment_only_happens_at_calibration(self):
        tree = ast.parse(AGENT_FILE.read_text(encoding='utf-8'))
        assignments = []
        for node in ast.walk(tree):
            if not isinstance(node, (ast.Assign, ast.AnnAssign)):
                continue
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            if any(isinstance(t, ast.Attribute) and
                   isinstance(t.value, ast.Name) and t.value.id == 'self' and
                   t.attr == 'offset' for t in targets):
                assignments.append(node.lineno)
        self.assertEqual(len(assignments), 2)  # constructor + calibration


if __name__ == '__main__':
    unittest.main()
