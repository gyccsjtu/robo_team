"""Swarm task lifecycle and executor regression tests; no ROS."""
import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/robocup_swarm/scripts'))
from task_authority import TaskAuthority, TaskGate


def task(target='t0', x=0., typ=1, cell=0):
    return dict(cell_ix=cell, cell_iy=0, target_x=x, target_y=0.,
                task_type=typ, target_id=target if typ == 1 else '')


class AuthorityTests(unittest.TestCase):
    def setUp(self):
        self.core = TaskAuthority('run', ['uav_1', 'uav_2'], lease_s=3.)
        self.gate = TaskGate('run', 'uav_1')
        self.ack_seq = 0

    def ack(self, now, xyz=(0., 0., 4.), speed=0., duration=1., status='STOPPED', **changes):
        self.ack_seq += 1
        msg = dict(schema_version=2, run_id='run', uav_id='uav_1', seq=self.ack_seq,
                   generation=self.core.generation.get('uav_1', 0), sample_s=now,
                   xyz=list(xyz), speed_mps=speed, stopped_s=duration, status=status)
        msg.update(changes)
        return self.core.ack(msg, now)

    def stop_for(self, now):
        for i in range(10):
            self.ack(now - 1. + i * .1, status='STATE')
        return self.ack(now)

    def test_single_target_owner_even_for_backup(self):
        self.core.offer('uav_1', task(), 0.)
        self.assertEqual(self.core.offer('uav_2', task(), .1), [])
        self.assertEqual(self.core.locks[('target', 't0')]['owner'], 'uav_1')

    def test_refresh_preserves_unique_owner_and_fences_old_generation(self):
        old = self.core.offer('uav_1', task(), 0.)[0]
        self.gate.receive(old, 0.)
        stop = self.core.refresh('uav_1', .1)[0]
        self.gate.receive(stop, .1)
        self.assertFalse(self.gate.can_move(.1))
        self.assertEqual(self.core.offer('uav_2', task(), .2), [])
        new = self.stop_for(1.4)[0]
        self.assertEqual(new['generation'], 2)
        self.assertEqual(new['task'], old['task'])
        self.assertTrue(self.gate.receive(new, 1.4))
        self.assertEqual(self.core.locks[('target', 't0')]['owner'], 'uav_1')
        self.assertFalse(self.gate.receive(old, 1.5))

    def test_close_cancels_pending_and_never_restarts_after_measured_stop(self):
        self.core.offer('uav_1', task(), 0.)
        self.core.offer('uav_1', task('t1', 20.), .1)
        self.core.offer('uav_2', task(), .2)
        stop = self.core.close(.3)
        self.assertEqual([m['action'] for m in stop], ['STOP'])
        self.assertEqual(self.core.pending, {})
        self.assertEqual(self.core.offer('uav_2', task('new'), .4), [])
        self.assertEqual(self.stop_for(1.5), [])
        self.assertEqual(self.core.tick(10.), [])
        self.assertTrue(self.core.locks[('target', 't0')]['retired'])
        self.assertEqual(self.core.close(11.), [])

    def test_switch_requires_stopped_ack_and_retains_old_lock(self):
        grant = self.core.offer('uav_1', task(), 0.)[0]
        self.assertTrue(self.gate.receive(grant, 0.))
        stop = self.core.offer('uav_1', task('t1', 20.), .1)[0]
        self.assertEqual(stop['action'], 'STOP')
        self.assertTrue(self.gate.receive(stop, .1))
        self.assertFalse(self.gate.can_move(.1))
        self.assertEqual(self.ack(.2, speed=.3), [])
        self.assertEqual(self.ack(.3, duration=.9), [])
        new = self.stop_for(1.4)[0]
        self.assertEqual(new['generation'], 2)
        self.assertTrue(self.gate.receive(new, 1.4))
        self.assertTrue(self.core.locks[('target', 't0')]['retired'])
        self.assertEqual(self.core.offer('uav_2', task(), 1.5), [])
        self.ack(1.6, xyz=(5., 0., 4.), status='STATE')
        messages = self.core.tick(1.7)
        self.assertTrue(any(m['uav_id'] == 'uav_2' and m['action'] == 'GRANT' for m in messages))

    def test_expiry_never_releases_lock(self):
        self.core.offer('uav_1', task(), 0.)
        self.assertEqual(self.core.tick(4.)[0]['action'], 'STOP')
        self.assertIn(('target', 't0'), self.core.locks)
        self.assertEqual(self.core.offer('uav_2', task(), 5.), [])

    def test_same_owner_can_resume_only_after_stop_ack_with_new_generation(self):
        self.core.offer('uav_1', task(), 0.)
        self.core.tick(4.)
        new = self.stop_for(5.2)[0]
        self.assertEqual(new['generation'], 2)
        self.assertEqual(new['task']['target_id'], 't0')

    def test_other_aircraft_detection_does_not_extend_lost_aircraft_lease(self):
        self.core.offer('uav_1', task(), 0.)
        refreshed = self.core.offer('uav_1', task(x=1.), 2.9)[0]
        self.assertEqual(refreshed['expires_s'], 3.)
        self.assertEqual(self.core.tick(3.1)[0]['action'], 'STOP')

    def test_stale_wrong_run_and_generation_ack_cannot_handoff(self):
        self.core.offer('uav_1', task(), 0.)
        self.core.offer('uav_1', task('t1', 20.), .1)
        for now, changes in ((1., dict(sample_s=0.)), (1.1, dict(run_id='old')),
                             (1.2, dict(generation=0)), (1.3, dict(speed_mps=float('nan')))):
            self.assertEqual(self.ack(now, **changes), [])
            self.assertEqual(self.core.generation['uav_1'], 1)

    def test_old_ack_sequence_cannot_clear_retired_resource(self):
        self.core.offer('uav_1', task(), 0.)
        self.core.offer('uav_1', task('t1', 20.), .1)
        self.stop_for(1.2)
        self.ack(1.3, xyz=(10., 0., 4.), status='STATE', seq=1)
        self.assertIn(('target', 't0'), self.core.locks)

    def test_same_target_coordinate_update_keeps_generation(self):
        a = self.core.offer('uav_1', task(), 0.)[0]
        b = self.core.offer('uav_1', task(x=2.), .1)[0]
        self.assertEqual(a['generation'], b['generation'])
        self.assertEqual(b['task']['target_x'], 2.)

    def test_time_rollback_is_rejected_by_core(self):
        self.core.tick(2.)
        with self.assertRaisesRegex(ValueError, 'TIME_ROLLBACK'):
            self.core.tick(1.)

    def test_six_aircraft_task_switches_keep_unique_locks(self):
        core = TaskAuthority('six', ['uav_%d' % i for i in range(1, 7)])
        gates = {u: TaskGate('six', u) for u in core.fleet}
        for i, u in enumerate(core.fleet):
            self.assertTrue(gates[u].receive(core.offer(u, task(typ=0, cell=i, x=i * 10.), 0.)[0], 0.))
        outputs = core.offer('uav_1', task('t0'), .1)
        self.assertTrue(gates['uav_1'].receive(outputs[0], .1))
        ack = dict(schema_version=2, run_id='six', uav_id='uav_1', seq=1,
                   generation=1, sample_s=1.2, xyz=[0., 0., 4.], speed_mps=.01,
                   stopped_s=1., status='STOPPED')
        for i in range(10):
            sample = .2 + i * .1
            core.ack(dict(ack, seq=i + 1, sample_s=sample, status='STATE'), sample)
        grant = core.ack(dict(ack, seq=11), 1.2)[0]
        self.assertTrue(gates['uav_1'].receive(grant, 1.2))
        self.assertEqual(len(core.active), 6)
        self.assertEqual(len(core.locks), 7)  # old search task still held

    def test_single_stopped_claim_and_sample_gap_are_not_continuous_proof(self):
        self.core.offer('uav_1', task(), 0.)
        self.core.offer('uav_1', task('t1', 20.), .1)
        self.assertEqual(self.ack(1.2, duration=100.), [])
        self.assertEqual(self.ack(2.2, duration=101.), [])
        self.assertEqual(self.core.generation['uav_1'], 1)

    def test_local_stop_claim_does_not_renew_same_generation_forever(self):
        self.core.offer('uav_1', task(), 0.)
        self.ack(1., status='STOPPED')
        renewals = self.core.tick(1.1)
        self.assertEqual(renewals, [])
        self.assertEqual(self.core.tick(3.1)[0]['action'], 'STOP')


class GateTests(unittest.TestCase):
    def setUp(self):
        self.core = TaskAuthority('run', ['uav_1'])
        self.gate = TaskGate('run', 'uav_1')
        self.grant = self.core.offer('uav_1', task(), 0.)[0]
        self.gate.receive(self.grant, 0.)

    def test_wrong_run_uav_and_sequence_rejected(self):
        for patch in (dict(run_id='old'), dict(uav_id='uav_2'), dict(seq=0), dict(seq=True)):
            message = dict(self.grant, seq=2)
            message.update(patch)
            self.assertFalse(self.gate.receive(message, .1))

    def test_stop_cannot_be_undone_by_same_generation_renewal(self):
        stop = self.core.offer('uav_1', task('t1', 10.), .1)[0]
        self.gate.receive(stop, .1)
        renewal = dict(self.grant, seq=3, expires_s=20.)
        self.assertFalse(self.gate.receive(renewal, .2))
        self.assertFalse(self.gate.can_move(.2))

    def test_local_expiry_and_time_rollback_latch_stop(self):
        self.assertFalse(self.gate.can_move(4.))
        self.assertFalse(self.gate.receive(dict(self.grant, seq=2, expires_s=10.), 4.1))
        other = TaskGate('run', 'uav_1')
        other.receive(self.grant, 1.)
        self.assertFalse(other.can_move(.5))

    def test_task_identity_cannot_change_inside_same_generation(self):
        wrong = dict(self.grant, seq=2, task=task('t1'))
        self.assertFalse(self.gate.receive(wrong, .1))

    def test_new_generation_requires_prior_stop_and_fresh_start_requires_generation_one(self):
        self.assertFalse(self.gate.receive(dict(self.grant, seq=2, generation=2), .1))
        fresh = TaskGate('run', 'uav_1')
        self.assertFalse(fresh.receive(dict(self.grant, generation=9), .1))


if __name__ == '__main__':
    unittest.main()
