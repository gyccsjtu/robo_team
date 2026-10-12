"""Exercise camera veto -> bridge -> dispatch, including delayed ROS messages."""
import json
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import test_candidate_bridge as cb_tests
import test_discovery_dispatch as dispatch_tests
from appearance_rejection import parse_rejections, RejectionBook
from distractor_verifier import rejected_instances
import swarm_manager as sm


def negative(sample=100.3, cid='10', source='u0', positive=100.):
    return dict(schema_version=1, source_uav=source, sample_s=sample,
                real_detection=True, reason='confirmed_secondary_distractor',
                rejected_candidates=[dict(candidate_id=cid, confidence=.92,
                    last_positive_s=positive,
                    evidence_samples=[sample-.3, sample-.15, sample])])


class EvidenceTests(unittest.TestCase):
    def test_missing_weak_duplicate_stale_negative_cannot_withdraw(self):
        for change in (dict(confidence=.84), dict(evidence_samples=[100.,100.,100.3]),
                       dict(evidence_samples=[100.1,100.2,100.3]),
                       dict(evidence_samples=[100.,100.3]), dict(candidate_id='')):
            data = negative(); data['rejected_candidates'][0].update(change)
            with self.assertRaises(ValueError):
                parse_rejections(json.dumps(data), 100.31)
        with self.assertRaises(ValueError):
            parse_rejections(json.dumps(negative()), 102.)

    def test_book_scoped_to_old_images_of_exact_instance(self):
        event = parse_rejections(json.dumps(negative()),100.31)[0]
        book = RejectionBook()
        self.assertTrue(book.record(event,100.31))
        self.assertFalse(book.record(event,100.32))
        self.assertTrue(book.blocks('u0','10',100.2))
        self.assertFalse(book.blocks('u1','10',100.2))
        self.assertFalse(book.blocks('u0','11',100.2))
        self.assertFalse(book.blocks('u0','10',100.4))

    def test_association_requires_recent_unique_box_not_colour_or_position(self):
        c = dict(candidate_id='10', sample_s=100., box=[100,100,130,180])
        e = dict(status='secondary_distractor', confidence=.92, box=c['box'],
                 evidence_samples=[100.,100.15,100.3])
        self.assertEqual(rejected_instances([c],[e],100.3)[0]['candidate_id'],'10')
        for candidates, evidence, sample in (([c],[dict(e,status='secondary_pending')],100.3),
                ([c],[e],102.), ([c,dict(c,candidate_id='11')],[e],100.3),
                ([dict(c,box=[135,100,165,180])],[e],100.3)):
            self.assertEqual(rejected_instances(candidates,evidence,sample),[])


class BridgeWithdrawalTests(unittest.TestCase):
    def setUp(self):
        self.helper = cb_tests.CandidateBridgeTests()
        self.b, self.t = self.helper.incumbent()

    def veto(self, **changes):
        data = negative(sample=self.t+.3, cid='old', positive=self.t)
        data.update(changes)
        self.b._advance(max(0.,data['sample_s']+.01-self.b._now()))
        self.b._appearance_rejection_cb(NS(data=json.dumps(data)))

    def test_veto_stops_reports_and_early_states_without_eliminating(self):
        self.veto()
        tr = self.b.core.tracks['blue']
        self.assertFalse(tr.alive)
        self.assertIsNone(tr.good_x)
        self.assertEqual(self.b.core.tick(self.b._now()),[])
        self.assertEqual(self.b.core.eliminated,set())
        self.assertFalse(tr.elim_pending)

    def test_late_positive_frame_cannot_resurrect_withdrawn_instance(self):
        self.veto()
        self.helper.send(self.b,self.t+.2,[self.helper.candidate('old',0.)])
        self.assertLess(self.b.core.tracks['blue'].t_obs,0.)
        for dt in (.4,.6,.8,1.):
            self.helper.send(self.b,self.t+dt,[self.helper.candidate('old',0.)])
        self.assertTrue(self.b.core.tracks['blue'].alive)

    def test_another_candidate_and_camera_keep_their_good_history(self):
        tr = self.b.core.tracks['blue']
        before = list(tr.good_history)
        self.veto(source_uav='u1')
        self.assertTrue(tr.alive)
        self.assertEqual(tr.good_history,before)
        self.assertEqual(tr.official_source,'u0')

    def test_bad_tracking_source_does_not_clear_independent_official_source(self):
        tr = self.b.core.tracks['blue']
        before = list(tr.good_history)
        tr.tracking_source, tr.tracking_candidate_id = 'u1', 'old'
        self.veto(source_uav='u1')
        self.assertTrue(tr.alive)
        self.assertEqual(tr.official_source,'u0')
        self.assertEqual(tr.tracking_source,'u0')
        self.assertEqual(tr.good_history,before)

    def test_older_negative_cannot_erase_newer_positive(self):
        self.helper.send(self.b,self.t+.4,[self.helper.candidate('old',0.)])
        self.veto()
        self.assertTrue(self.b.core.tracks['blue'].alive)
        self.assertEqual(self.b.core.tracks['blue'].good_t,self.t+.4)

    def test_red_instances_are_independent(self):
        for i in range(6):
            self.helper.send(self.b,102.+i*.2,[self.helper.candidate('red-a',20.,tag='red1'),
                                              self.helper.candidate('red-b',40.,tag='red2')])
        self.b.core.reject_instance('u0','red-a',103.2)
        live = [t for tag,t in self.b.core.tracks.items() if tag.startswith('red') and t.alive]
        self.assertEqual(len(live),1)
        self.assertEqual(live[0].tracking_candidate_id,'red-b')
        self.assertEqual(self.b.core.eliminated,set())


class ManagerWithdrawalTests(unittest.TestCase):
    setUp = dispatch_tests.DiscoveryDispatchTests.setUp
    payload = dispatch_tests.DiscoveryDispatchTests.payload
    send = dispatch_tests.DiscoveryDispatchTests.send

    def prepare(self):
        self.send()
        self.m._tracking['t1']='u0'
        self.m.allocator = NS(revisit_jobs={}, clear_hot_target=Mock(), queue_revisit=Mock())

    def veto(self):
        self.now = max(self.now,100.31)
        self.m._appearance_rejection_cb(NS(data=json.dumps(negative())))

    def test_release_cancels_owner_and_backup_without_lost_revisit(self):
        self.prepare(); self.m._backup['t1']='u2'
        self.veto()
        self.assertEqual(self.m._tracking,{})
        self.assertEqual(self.m._backup,{})
        self.assertNotIn('t1',self.m.tracker.targets)
        self.assertNotIn('t1',self.m._reacquire_sites)
        self.m.allocator.queue_revisit.assert_not_called()
        self.assertEqual({c.args[0].uav_id for c in self.m.assign_pub.publish.call_args_list},
                         {'u0','u2'})
        self.assertTrue(all(c.args[0].task_type == 255
                            for c in self.m.assign_pub.publish.call_args_list))
        self.assertEqual(self.m._eliminated,set())

    def test_delayed_discovery_source_and_state_cannot_restore_old_task(self):
        self.prepare(); self.veto()
        old = self.payload(sample_s=100.2)
        self.m._discovery_cb(NS(data=json.dumps(old)))
        self.m._visual_source_cb(NS(data=json.dumps(dict(old,official_view_ok=True))))
        state = sm.TargetState()
        state.header.stamp = sm.rospy.Time.from_sec(100.2)
        state.target_id,state.x,state.y = 't1',8.,0.
        self.m._truth_cb(state)
        self.assertNotIn('t1',self.m.tracker.targets)
        self.assertNotIn('t1',self.m._truth_cache)
        self.m._queue_lost_revisit('t1',self.now)
        self.m.allocator.queue_revisit.assert_not_called()

    def test_independent_same_colour_observation_is_preserved(self):
        self.prepare()
        self.m._visual_source_cb(NS(data=json.dumps(self.payload(source_uav='u1',
            candidate_id='true',sample_s=99.95,official_view_ok=True,x=8.1))))
        self.veto()
        self.assertEqual(self.m._tracking,{'t1':'u0'})
        self.assertEqual(self.m._visual_source['t1'][0],'u1')
        self.assertIn(('u1','t1'),self.m._eligible_observations)
        self.assertEqual(self.m._truth_cache['t1'],(8.1,0.,99.95))
        self.m.assign_pub.publish.assert_not_called()

    def test_newer_real_image_beats_delayed_negative(self):
        self.prepare(); self.now=100.4
        self.send(sample_s=100.4)
        self.veto()
        self.assertEqual(self.m._tracking,{'t1':'u0'})
        self.assertEqual(self.m._visual_source['t1'][1],100.4)
        self.m.assign_pub.publish.assert_not_called()

    def test_new_camera_detection_not_blacklisted(self):
        self.prepare(); self.veto()
        self.send(source_uav='u1',candidate_id='10',sample_s=100.2)
        self.assertIn('t1',self.m.tracker.targets)
        self.assertEqual(self.m._visual_source['t1'][0],'u1')

    def test_same_camera_different_qualified_instance_survives(self):
        self.prepare()
        peer = dict(self.payload(candidate_id='real',sample_s=99.95,x=9.),eligible=True)
        self.m._eligible_observations[('u0','t1')]=peer
        self.veto()
        self.assertEqual(self.m._eligible_observations[('u0','t1')]['candidate_id'],'real')
        self.assertEqual(self.m._truth_cache['t1'],(9.,0.,99.95))
        self.assertEqual(self.m._tracking,{'t1':'u0'})

    def test_withdrawal_removes_only_matching_queued_revisit(self):
        self.prepare()
        self.m._reacquire_sites['t1'].update(queued_key=(2,3),queued=True)
        self.m.allocator.revisit_jobs[('lost',(2,3))]=(100.,125.)
        self.m.allocator.revisit_jobs[('lost',(3,3))]=(100.,125.)
        self.veto()
        self.assertNotIn(('lost',(2,3)),self.m.allocator.revisit_jobs)
        self.assertIn(('lost',(3,3)),self.m.allocator.revisit_jobs)

    def test_recorded_sculpture_sequence_releases_actual_camera_instance(self):
        fixture = json.loads((Path(__file__).parent/
            'fixtures/sculpture_withdrawal_130851.json').read_text())
        view = next(r['data'] for r in fixture['observations'] if r['event']=='tracking_view')
        discovery = next(r['data'] for r in fixture['observations'] if r['event']=='discovery')
        report = next(r['data'] for r in fixture['observations'] if r['event']=='image_report')
        last = fixture['frames'][-1]
        cid, uid = view['world_candidate_id'], view['source_uav']
        old = dict(candidate_id=cid, sample_s=view['sample_s'], box=view['box'])
        rejected = rejected_instances([old],last['evidence'],last['sample_s'])
        self.assertEqual([r['candidate_id'] for r in rejected],[cid])
        packet = dict(schema_version=1, source_uav=uid, sample_s=last['sample_s'],
            reason='confirmed_secondary_distractor',real_detection=True,
            rejected_candidates=rejected)
        self.now = view['sample_s']+.01
        self.m.uav_ids.append(uid)
        self.m._discovery_cb(NS(data=json.dumps(discovery)))
        self.assertIn('t1',self.m.tracker.targets)
        self.m._tracking['t1']=uid
        self.m.allocator=NS(revisit_jobs={},clear_hot_target=Mock(),queue_revisit=Mock())
        b = cb_tests.CandidateBridgeTests().node()
        b._advance(self.now)
        b._report_cb(NS(data=json.dumps(report)))
        self.assertGreater(b.core.tracks['blue'].t_obs,0.)
        self.now = last['sample_s']+.01
        b._advance(self.now-b._now())
        msg=NS(data=json.dumps(packet))
        b._appearance_rejection_cb(msg)
        self.m._appearance_rejection_cb(msg)
        self.assertNotIn('t1',self.m.tracker.targets)
        self.assertNotIn('t1',self.m._tracking)
        self.assertLess(b.core.tracks['blue'].t_obs,0.)
        self.m._queue_lost_revisit('t1',self.now)
        self.m.allocator.queue_revisit.assert_not_called()
        self.assertEqual(self.m._eliminated,set())


if __name__ == '__main__':
    unittest.main()
