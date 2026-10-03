import ast
import re
import sys
from pathlib import Path
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).parents[1]/'src/robocup_swarm/scripts'))
from red_observations import RedObservations, actor_slot_remaining
from search_completion import parse_actor_list


agent_source = Path(__file__).parents[1]/'src/robocup_swarm/scripts/swarm_agent.py'
agent_tree = ast.parse(agent_source.read_text(encoding='utf-8'))
callbacks = [node for cls in agent_tree.body if isinstance(cls, ast.ClassDef)
             for node in cls.body if isinstance(node, ast.FunctionDef)
             and node.name == '_left_actors_cb']
callback_scope = dict(re=re, parse_actor_list=parse_actor_list,
                      actor_slot_remaining=actor_slot_remaining)
exec(compile(ast.fix_missing_locations(ast.Module(body=callbacks, type_ignores=[])),
             str(agent_source), 'exec'), callback_scope)


class RedObservationTests(unittest.TestCase):
    def test_cameras_observing_same_red_person_share_one_geometric_slot(self):
        r=RedObservations()
        self.assertEqual(r.observe(10.,(0.,0.)),'red1')
        self.assertEqual(r.observe(10.1,(.2,.1)),'red1')
        self.assertEqual(r.observe(10.2,(20.,0.)),'red2')
        self.assertEqual(r.observe(10.3,(20.2,0.)),'red2')
        self.assertEqual(r.observe(10.4,(.6,.1)),'red1')
        self.assertIsNone(r.observe(10.5,(50.,50.)))

    def test_old_frame_cannot_rewind_slots_and_expired_slot_can_reacquire(self):
        r=RedObservations();r.observe(10.,(0.,0.));r.observe(10.,(20.,0.))
        self.assertIsNone(r.observe(9.,(0.,0.)))
        self.assertEqual(r.observe(17.,(100.,0.)),'red1')
        self.assertIsNone(r.observe(float('nan'),(0.,0.)))

    def test_one_official_red_elimination_does_not_invalidate_an_arbitrary_camera_slot(self):
        for remaining in ([4],[5]):
            self.assertTrue(actor_slot_remaining(4,remaining))
            self.assertTrue(actor_slot_remaining(5,remaining))
        self.assertFalse(actor_slot_remaining(4,[1,2]))
        self.assertTrue(actor_slot_remaining(1,[1,2]))

    def test_actual_agent_retains_both_red_camera_slots_when_one_red_remains(self):
        for remaining in ('[4]', '[5]'):
            a = SimpleNamespace(targets={'t4': (), 't5': (), 't1': ()},
                _t_seen={'t4': 10., 't5': 10., 't1': 10.},
                _giveup_until={}, _left_seen=True, _orbit_target='t5',
                _abort_orbit=Mock())
            callback_scope['_left_actors_cb'](a, SimpleNamespace(data=remaining))
            self.assertEqual(set(a.targets), {'t4', 't5'})
            self.assertEqual(set(a._t_seen), {'t4', 't5'})
            self.assertNotIn('t4', a._giveup_until)
            self.assertNotIn('t5', a._giveup_until)
            a._abort_orbit.assert_not_called()

    def test_actual_agent_retires_both_red_slots_only_after_both_are_gone(self):
        a = SimpleNamespace(targets={'t4': (), 't5': ()},
            _t_seen={'t4': 10., 't5': 10.}, _giveup_until={},
            _left_seen=True, _orbit_target='t5', _abort_orbit=Mock())
        callback_scope['_left_actors_cb'](a, SimpleNamespace(data='[2,3]'))
        self.assertEqual(a.targets, {})
        self.assertEqual(a._t_seen, {})
        self.assertEqual(a._giveup_until, {'t4': float('inf'), 't5': float('inf')})
        a._abort_orbit.assert_called_once()
