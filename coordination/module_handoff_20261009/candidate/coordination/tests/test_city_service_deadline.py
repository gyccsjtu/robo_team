import sys
import time
import threading
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from city_swarm_run import bounded_query,query_positions


class ServiceDeadlineTests(unittest.TestCase):
    def test_single_connection_fault_recovers_with_new_positions_after_health_check(self):
        trace=[]
        def service():
            trace.append('query')
            if trace.count('query')==1:raise ConnectionResetError('reset by peer')
            return {'uav_1':[9.,8.,3.]}
        positions=query_positions(service,lambda:trace.append('health'),
                                  lambda error:trace.append(error))
        self.assertEqual(positions,{'uav_1':[9.,8.,3.]})
        self.assertEqual(trace,['query','health','reset by peer','query'])

    def test_permanent_fault_gets_only_two_calls_and_keeps_last_error(self):
        calls=[];retries=[]
        class ServiceError(Exception):pass
        def failed():
            calls.append(1)
            raise ServiceError('failure '+str(len(calls)))
        with self.assertRaisesRegex(ServiceError,'failure 2'):
            query_positions(failed,on_retry=retries.append,retryable=(ServiceError,))
        self.assertEqual(len(calls),2)
        self.assertEqual(retries,['failure 1'])

    def test_dead_child_timeout_and_non_rpc_errors_cannot_retry(self):
        calls=[]
        def failed():
            calls.append(1);raise ConnectionResetError('reset')
        def dead():raise RuntimeError('OWNED_CHILD_EXITED')
        with self.assertRaisesRegex(RuntimeError,'OWNED_CHILD_EXITED'):
            query_positions(failed,health_check=dead)
        self.assertEqual(len(calls),1)
        for error in (TimeoutError('hung'),ValueError('invalid data')):
            calls=[]
            def invalid():calls.append(1);raise error
            with self.assertRaises(type(error)):
                query_positions(invalid,health_check=lambda: calls.append('health'))
            self.assertEqual(calls,[1])

    def test_retry_is_bounded_at_one_second_when_service_then_hangs(self):
        calls=[];release=threading.Event();started=time.monotonic()
        def service():
            calls.append(1)
            if len(calls)==1:raise ConnectionError('reset')
            release.wait()
        try:
            with self.assertRaisesRegex(TimeoutError,'CITY_MODEL_STATE_TIMEOUT'):
                query_positions(service)
            self.assertEqual(len(calls),2)
            self.assertLess(time.monotonic()-started,1.7)
        finally:release.set()

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
