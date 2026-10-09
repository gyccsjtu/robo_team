"""Legacy actor spawn knowledge must not silently change task selection."""
import importlib.util
import os
from pathlib import Path
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).parents[1]/'src/robocup_swarm/scripts/swarm_task.py'


def load(extra=None):
    env = dict(os.environ)
    for name in list(env):
        if name.startswith('PRIORITY_'):
            del env[name]
    env.update(extra or {})
    spec = importlib.util.spec_from_file_location('priority_defaults_test', str(SOURCE))
    module = importlib.util.module_from_spec(spec)
    with patch.dict(os.environ, env, clear=True):
        spec.loader.exec_module(module)
    return module


class PriorityDefaultsTests(unittest.TestCase):
    def auction(self, module):
        grid = module.CoverageGrid(-40.,40.,-40.,40.,10.,1)
        allocator = module.TaskAllocator(grid)
        near, old_birth = (4,4), (0,1)
        allocator.utility = lambda uid,x,y,key,*args: 3. if key == near else 2.
        result = allocator.allocate({'uav_1': (0.,0.)},
            candidate_filter=lambda uid,key,assigned:key in (near,old_birth))
        return result['uav_1']

    def test_default_auction_does_not_use_old_actor_birth(self):
        module = load()
        self.assertEqual(module._PRIORITY_POINTS, [])
        self.assertEqual(self.auction(module), (4,4))

    def test_explicit_enable_without_points_does_not_restore_old_birth(self):
        self.assertEqual(self.auction(load({'PRIORITY_CORNER':'1'})), (4,4))

    def test_only_explicit_enabled_custom_points_change_selection(self):
        explicit = {'PRIORITY_CORNER':'1','PRIORITY_POINTS':'-35,-25'}
        self.assertEqual(self.auction(load(explicit)), (0,1))
        explicit['PRIORITY_CORNER'] = '0'
        self.assertEqual(self.auction(load(explicit)), (4,4))
