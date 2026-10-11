"""Keep inter-aircraft avoidance radial and as close to the desired path as possible."""

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
import swarm_agent


class OrcaHeadingTests(unittest.TestCase):
    def _agent(self, friend_xy):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'typhoon_h480_3'
        agent.local_xy = (0.0, 0.0)
        agent.offset = (0.0, 0.0)
        agent.local_z = 3.0
        agent.altitude_layer = 3.0
        agent._friend_positions = {
            'typhoon_h480_2': (friend_xy[0], friend_xy[1], 3.0)}
        agent._friend_seen = {'typhoon_h480_2': 100.0}
        agent._adaptive_speed = lambda vx, vy: (vx, vy)
        return agent

    def test_blocked_northwest_request_does_not_become_full_speed_south(self):
        agent = self._agent((-2.4, 2.4))
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)), \
             patch.object(swarm_agent.rospy, 'loginfo_throttle'):
            vx, vy = agent._apply_friend_avoidance(-2.11, 2.13)
        self.assertGreaterEqual(vx - vy, 0.0)  # radial motion away from NW peer
        self.assertLess(math.hypot(vx, vy), 1.0, (vx, vy))

    def test_sideways_motion_must_not_close_on_east_peer(self):
        agent = self._agent((4.0, 0.0))
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)), \
             patch.object(swarm_agent.rospy, 'loginfo_throttle'):
            vx, vy = agent._apply_friend_avoidance(1.0, 0.0)
        self.assertLessEqual(vx, 1e-6, (vx, vy))

    def test_opposite_close_peers_stop_when_constraints_conflict(self):
        agent = self._agent((3.3, 0.0))
        agent._friend_positions['typhoon_h480_4'] = (-3.3, 0.0, 3.0)
        agent._friend_seen['typhoon_h480_4'] = 100.0
        with patch.object(swarm_agent.rospy.Time, 'now',
                          return_value=swarm_agent.rospy.Time(100.0)), \
             patch.object(swarm_agent.rospy, 'loginfo_throttle'), \
             patch.object(swarm_agent.rospy, 'logwarn'):
            self.assertEqual(agent._apply_friend_avoidance(2.0, 0.0),
                             (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
