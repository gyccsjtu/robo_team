"""Verify autonomous-mode parameters at the agent's actual startup entry."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                      'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa


class AutonomousConfigurationTests(unittest.TestCase):
    def configure(self, exception_readback):
        agent = sa.SwarmAgent.__new__(sa.SwarmAgent)
        agent.uav_id = 'typhoon_h480_3'
        agent._set_param = Mock(return_value=True)
        pull = Mock(return_value=NS(success=True, param_received=100))
        get = Mock(side_effect=lambda name: NS(success=True, value=NS(
            integer=exception_readback if name == 'COM_RCL_EXCEPT' else 0)))
        proxy = lambda name, *_: pull if name.endswith('/pull') else get
        with patch.object(sa.rospy, 'wait_for_service'), \
                patch.object(sa.rospy, 'ServiceProxy', side_effect=proxy):
            agent._configure_fcu()
        return agent, get

    def test_offboard_and_auto_takeoff_rc_exceptions_are_read_back(self):
        agent, get = self.configure(6)
        agent._set_param.assert_any_call('COM_RCL_EXCEPT', 6)
        agent._set_param.assert_any_call('NAV_RCL_ACT', 0)
        get.assert_any_call('COM_RCL_EXCEPT')
        self.assertEqual(agent._set_param.call_count, 2)

    def test_old_offboard_only_readback_prevents_start(self):
        with self.assertRaisesRegex(RuntimeError, 'FCU_CONFIGURATION_NOT_VERIFIED'):
            self.configure(4)


if __name__ == '__main__':
    unittest.main()
