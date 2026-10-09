import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'))
from route_reservation import RouteAuthority, RouteGate, compact_pairs, exclude_peers
from fleet_motion_guard import MotionCache


class RouteTests(unittest.TestCase):
    def setUp(self):
        self.core = RouteAuthority('run', ['a', 'b'])
        self.motion = MotionCache('run', ['a', 'b'])
        self.tasks = {uid: dict(generation=1, stopping=False, expires_s=10) for uid in ('a', 'b')}
        self.sample('a', (0, 0))
        self.sample('b', (12, 0))

    def sample(self, uid, position, now=1):
        self.motion.receive(dict(schema_version=1, run_id='run', uav_id=uid, seq=int(now*100),
            sample_s=now, frame='world_enu_xy', position_xy=position, velocity_xy=(0, 0)), now)

    def offer(self, uid, points, offer_id=1, now=1):
        return self.core.offer(dict(schema_version=1, run_id='run', uav_id=uid,
            generation=self.tasks[uid]['generation'], offer_id=offer_id, points=points), now, self.tasks, self.motion)

    def test_actual_detour_conflicts_despite_disjoint_task_legs(self):
        self.assertTrue(self.offer('a', [(0, 0), (0, 20), (10, 20)]))
        self.assertFalse(self.offer('b', [(12, 0), (12, 20), (20, 20)]))
        self.assertEqual(self.core.events[-1]['reason'], 'ROUTE_CONFLICT')

    def test_timeout_and_replan_keep_original_occupancy(self):
        self.offer('a', [(0, 0), (0, 20)])
        self.offer('a', [(0, 0), (-10, 0)], 2)
        self.assertIn(((0, 0), (0, 20)), self.core.reserved['a'])
        self.tasks['a']['stopping'] = True
        self.assertFalse(self.core.tick(11, self.tasks, self.motion))
        self.assertIn('a', self.core.reserved)
        self.core.retire_stopped('a', 2)
        self.assertIn('a', self.core.reserved)
        self.core.retire_stopped('a', 1)
        self.assertNotIn('a', self.core.reserved)

    def test_stale_peer_blocks_grant(self):
        self.assertFalse(self.offer('a', [(0, 0), (-10, 0)], now=2))
        self.assertEqual(self.core.events[-1]['reason'], 'FLEET_EVIDENCE_MISSING')

    def test_gate_fences_old_offer_run_generation_and_expiry(self):
        grant = self.offer('a', [(0, 0), (0, 20)])[0]
        gate = RouteGate('run', 'a')
        self.assertFalse(gate.receive(grant, 1, 2, 1))
        self.assertFalse(gate.receive(grant, 1, 1, 2))
        self.assertFalse(gate.receive(dict(grant, run_id='old'), 1, 1, 1))
        self.assertTrue(gate.receive(grant, 1, 1, 1))
        self.assertFalse(gate.receive(grant, 1, 1, 1))
        self.assertTrue(gate.command_clear((0, 0), (0, 1), (0, 0), 1, 1))
        self.assertFalse(gate.command_clear((0, 0), (0, 1), (0, 0), 3, 1))

    def test_measured_momentum_and_sideways_command_cannot_leave_route(self):
        gate = RouteGate('run', 'a')
        gate.receive(self.offer('a', [(0, 0), (0, 20)])[0], 1, 1, 1)
        self.assertFalse(gate.command_clear((0, 0), (1, 0), (0, 0), 1, 1))
        self.assertFalse(gate.command_clear((0, 0), (0, .1), (1, 0), 1, 1))

    def test_retired_route_still_blocks_through_fresh_owner_position(self):
        self.offer('a', [(0, 0), (0, 20)])
        self.core.retire_stopped('a', 1)
        self.assertFalse(self.offer('b', [(12, 0), (0, 0)]))

    def test_interval_compaction_preserves_full_occupied_union(self):
        pairs = [((0, 0), (0, 5)), ((0, 3), (0, 8)), ((0, 12), (0, 10)), ((1, 1), (3, 3))]
        self.assertEqual(set(compact_pairs(pairs)), {((0, 0), (0, 8)), ((0, 10), (0, 12)), ((1, 1), (3, 3))})

    def test_planner_excludes_retained_peer_corridor(self):
        from robocup_navigation.astar import GridMap
        grid = GridMap(80, 80, .5, (-10., -10.), bytes(6400), 'map')
        safe = exclude_peers(grid, 'a', self.motion,
            {'b': dict(generation=1, segments=[((12, 0), (12, 20))])}, 1)
        self.assertTrue(safe.is_free(safe.world_to_cell((0, 0))))
        self.assertFalse(safe.is_free(safe.world_to_cell((8, 10))))

    def test_invalid_or_rollback_time_stops_authority_and_execution(self):
        grant = self.offer('a', [(0, 0), (0, 20)])[0]
        gate = RouteGate('run', 'a')
        self.assertFalse(gate.receive(grant, float('nan'), 1, 1))
        self.assertTrue(gate.receive(grant, 1, 1, 1))
        self.assertFalse(gate.command_clear((0, 0), (0, .1), (0, 0), float('nan'), 1))
        self.assertFalse(self.core.tick(.9, self.tasks, self.motion))


if __name__ == '__main__':
    unittest.main()
