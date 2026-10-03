import sys
import time
import threading
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from city_swarm_run import bounded_query


class ServiceDeadlineTests(unittest.TestCase):
    def test_success_and_service_failure_remain_distinct(self):
        self.assertEqual(bounded_query(lambda: {'uav_1': [1, 2, 3]}), {'uav_1': [1, 2, 3]})
        def failed_service():
            raise ConnectionError('Gazebo disconnected')
        with self.assertRaisesRegex(ConnectionError, 'Gazebo disconnected'):
            bounded_query(failed_service)

    def test_unresponsive_service_returns_timeout_instead_of_freezing(self):
        release = threading.Event()
        started = time.monotonic()
        try:
            with self.assertRaisesRegex(TimeoutError, 'CITY_MODEL_STATE_TIMEOUT'):
                bounded_query(release.wait, timeout=.03)
            self.assertLess(time.monotonic()-started, .5)
        finally:
            release.set()
