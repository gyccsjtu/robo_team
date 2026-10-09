import ast
from pathlib import Path
import types
import unittest


class BarrierTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = Path(__file__).parents[1] / 'src/robocup_swarm/scripts/swarm_manager.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))
        manager = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SwarmManager')
        method = next(n for n in manager.body if isinstance(n, ast.FunctionDef) and n.name == '_allocate')
        class Stamp:
            def __init__(self, value):
                self.value = value
            def __sub__(self, other):
                return Stamp(self.value-other.value)
            def to_sec(self):
                return self.value
        cls.Stamp = Stamp
        env = {'rospy': types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda: Stamp(10.)),
                                             logwarn_throttle=lambda *args: None)}
        module = ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[]))
        exec(compile(module, str(source), 'exec'), env)
        cls.allocate = staticmethod(env['_allocate'])

    def object(self):
        return types.SimpleNamespace(uav_ids=['a', 'b'],
            status={k: types.SimpleNamespace(connected=True) for k in ['a', 'b']},
            last_report={k: self.Stamp(10.) for k in ['a', 'b']})

    def test_unknown_member_does_not_touch_lease_or_auction(self):
        obj = self.object()
        del obj.status['b']
        self.allocate(obj)  # There is no lease/allocator: any attempted auction fails the test.

    def test_disconnected_member_does_not_allocate(self):
        obj = self.object()
        obj.status['b'].connected = False
        self.allocate(obj)

    def test_stale_or_future_member_does_not_allocate(self):
        for stamp in (8., 11.):
            obj = self.object()
            obj.last_report['b'] = self.Stamp(stamp)
            self.allocate(obj)
