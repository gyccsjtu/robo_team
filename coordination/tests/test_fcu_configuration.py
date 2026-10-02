import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'src/robocup_swarm/scripts'))
from fcu_configuration import configure


class ConfigurationTests(unittest.TestCase):
    def test_unloaded_parameter_table_retries_before_acceptance(self):
        pulls = iter([False, True])
        values = {}
        configure(lambda: next(pulls), lambda n, v: values.update({n: v}) is None,
                  lambda n: values.get(n), {'COM_RCL_EXCEPT': 4})
        self.assertEqual(values, {'COM_RCL_EXCEPT': 4})

    def test_set_failure_cannot_be_silent(self):
        with self.assertRaisesRegex(RuntimeError, 'NOT_VERIFIED'):
            configure(lambda: True, lambda n, v: False, lambda n: 4, {'COM_RCL_EXCEPT': 4})

    def test_success_ack_with_wrong_actual_value_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'NOT_VERIFIED'):
            configure(lambda: True, lambda n, v: True, lambda n: 0, {'COM_RCL_EXCEPT': 4})
