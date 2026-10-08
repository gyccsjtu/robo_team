"""Exercise the actual launcher's signal callbacks without creating processes."""
import ast
import json
from pathlib import Path
import signal
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest

SCRIPTS=Path(__file__).parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from city_swarm_run import OwnedRunInterrupted,query_positions


class OwnedInterruptTests(unittest.TestCase):
    def callbacks(self,out):
        tree=ast.parse((SCRIPTS/'six_radar_connectivity.py').read_text())
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        functions=[n for n in main.body if isinstance(n,ast.FunctionDef)
                   and n.name in ('interrupted','watchdog_expired')]
        kills=[]
        scope=dict(out=out,report={},signal=signal,json=json,
            watchdog_fired=threading.Event(),OwnedRunInterrupted=OwnedRunInterrupted,
            interrupt_started_wall=100.,interrupt_started_monotonic=10.,
            time=SimpleNamespace(time=lambda:36100.,monotonic=lambda:15.),
            os=SimpleNamespace(getpid=lambda:123,kill=lambda *args:kills.append(args)))
        exec(compile(ast.fix_missing_locations(ast.Module(body=functions,type_ignores=[])),
                     str(SCRIPTS/'six_radar_connectivity.py'),'exec'),scope)
        return scope,kills

    def test_external_signal_is_not_rpc_timeout_and_keeps_both_clocks(self):
        with tempfile.TemporaryDirectory() as directory:
            scope,_=self.callbacks(Path(directory))
            for number in (signal.SIGTERM,signal.SIGINT,signal.SIGUSR1):
                with self.assertRaises(OwnedRunInterrupted) as raised:
                    scope['interrupted'](number,None)
                self.assertEqual(str(raised.exception),'OWNED_INTERRUPT_'+signal.Signals(number).name)
                self.assertFalse(raised.exception.evidence['owned_watchdog_fired'])
            saved=json.loads((Path(directory)/'owned_interrupt.json').read_text())
            self.assertEqual(len(saved),3)
            self.assertEqual(saved[-1]['wall_elapsed_s'],36000.)
            self.assertEqual(saved[-1]['monotonic_elapsed_s'],5.)

    def test_actual_watchdog_marks_itself_before_issuing_signal(self):
        with tempfile.TemporaryDirectory() as directory:
            scope,kills=self.callbacks(Path(directory));scope['watchdog_expired']()
            self.assertEqual(kills,[(123,signal.SIGUSR1)])
            with self.assertRaisesRegex(OwnedRunInterrupted,'OWNED_WATCHDOG_TIMEOUT'):
                scope['interrupted'](signal.SIGUSR1,None)
            with self.assertRaisesRegex(OwnedRunInterrupted,'OWNED_INTERRUPT_SIGINT'):
                scope['interrupted'](signal.SIGINT,None)

    def test_record_failure_does_not_hide_the_interrupt(self):
        scope,_=self.callbacks(Path('/nonexistent/owned_interrupt_test'))
        with self.assertRaisesRegex(OwnedRunInterrupted,'OWNED_INTERRUPT_SIGTERM'):
            scope['interrupted'](signal.SIGTERM,None)

    def test_signal_exception_during_query_is_not_retried_as_connection_failure(self):
        calls=[]
        error=OwnedRunInterrupted(dict(signal_name='SIGTERM',owned_watchdog_fired=False))
        def query():calls.append('query');raise error
        with self.assertRaises(OwnedRunInterrupted) as raised:
            query_positions(query,lambda:calls.append('health'),lambda m:calls.append('retry'))
        self.assertIs(raised.exception,error)
        self.assertEqual(calls,['query'])
