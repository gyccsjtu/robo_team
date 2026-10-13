"""Real image callbacks/timers: qualified points must not miss their age window."""
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                     'coordination/src/robocup_swarm/scripts'))
import yolo_target_bridge as bridge


def replay_slot(enabled, hz=4., angle=math.pi/2, speed=1., jitter=.008,
                tags=('blue',), noise=None, late_copies=False, qualify=True,
                range_block=False, rename=False):
    """Uninterrupted exact images; qualify=False is a cropped/invalid world view."""
    node = bridge._mk_bridge()
    outputs, acceptance = [], []
    for tag in tags:
        def publish(msg, tag=tag):
            tr = node.core.tracks[tag]
            outputs.append(dict(tag=tag, report_s=node._now(), sample_s=tr.good_t,
                source=tr.official_source, candidate=tr.official_candidate_id,
                x=msg.x, y=msg.y))
        node._actor_pubs[tag].publish = publish
    def position(t, tag):
        dt = t-100.;offset = 20. if tag=='red2' else 0.
        return (30.+speed*min(dt,8.1)+speed*max(0.,dt-8.1)*math.cos(angle),
                10.+offset+speed*max(0.,dt-8.1)*math.sin(angle))
    def frame(sample):
        rows=[]
        for index,tag in enumerate(tags):
            x,y = position(sample, tag)
            dx,dy = noise(sample) if noise else (0.,0.)
            rows.append(dict(target_id=tag, candidate_id='session:%d' % (
                index+11 if rename and sample>=108. else index+1),
                xyz=[x+dx,y+dy,1.25], frame_id='world_enu', confidence=.95,
                visual_confidence=.95, track_confirmed=True, track_hits=10,
                range_m=10.8 if range_block and sample>=108.1 else 8.,
                official_view_ok=qualify or sample<108.1,
                appearance_status='actor', appearance_confidence=.95))
        return dict(schema_version=2, source_uav='typhoon_h480_0',
                    sample_s=sample, real_detection=True, candidates=rows)
    events=[]
    for i in range(int(18.*hz)):
        sample=100.+i/hz
        arrival=sample+.3+(-jitter if sample<108.1 else jitter)
        events.append((arrival,0,frame(sample)))
        if late_copies and 104.<=sample<106.:
            duplicate=frame(sample)
            for row in duplicate['candidates']: row['xyz'][0]+=20.
            events += [(arrival+.003,0,duplicate),
                       (arrival+.01,0,frame(sample-1./hz))]
    events += [(100.+i/15.,1,None) for i in range(270)]
    with patch.object(bridge,'REPORT_POLICY','fresh'), \
         patch.object(bridge,'FRESH_IMAGE_SLOT',enabled), \
         patch.object(bridge,'_core_log',lambda *a:None):
        for now,kind,payload in sorted(events,key=lambda e:(e[0],e[1])):
            node._advance(now-node._now())
            if kind: node._tick(None)
            else:
                if range_block and payload['sample_s']>=108.1:
                    for tag in tags:node._gate_live[tag]=False
                node._report_cb(bridge._FakeString(json.dumps(payload)))
                for row in payload['candidates']:
                    tr=node.core.tracks[row['target_id']]
                    key=node.core._source_key(payload['source_uav'],row['candidate_id'])
                    acceptance.append((payload['sample_s'],row['candidate_id'],
                        tr.accepted_source_samples.get(key),tr.good_t,
                        tr.official_source,tr.official_candidate_id))
    gaps=[]
    for tag in tags:
        points=[p for p in outputs if p['tag']==tag]
        gaps.extend(b['report_s']-a['report_s'] for a,b in zip(points,points[1:]))
    return dict(node=node,outputs=outputs,acceptance=acceptance,max_gap=max(gaps,default=0.),
        min_gap=min(gaps,default=math.inf),max_age=max(
            (p['report_s']-p['sample_s'] for p in outputs),default=0.),
        max_error=max((math.hypot(p['x']-position(p['report_s'],p['tag'])[0],
                      p['y']-position(p['report_s'],p['tag'])[1]) for p in outputs),default=0.))


class FreshImageSlotTests(unittest.TestCase):
    def setUp(self):
        p=patch.object(bridge,'REPORT_POLICY','fresh');p.start();self.addCleanup(p.stop)
        p=patch.object(bridge,'FRESH_IMAGE_SLOT',True);p.start();self.addCleanup(p.stop)

    def test_current_counterexample_is_fixed_only_when_enabled(self):
        old=replay_slot(False)
        new=replay_slot(True)
        self.assertAlmostEqual(old['max_gap'],1.016,places=6)
        self.assertLess(new['max_gap'],.90)
        self.assertLess(new['max_error'],1.)

    def test_four_five_hz_straight_turn_reverse_with_bounded_receipt_jitter(self):
        for hz in (4.,5.):
          for speed in (1.,2.):
           for angle in (0.,math.pi/2,math.pi):
            for jitter in (0.,.004,.008):
             with self.subTest(hz=hz,speed=speed,angle=angle,jitter=jitter):
                r=replay_slot(True,hz,angle,speed,jitter)
                self.assertLess(r['max_gap'],.90)
                self.assertLess(r['max_error'],1.)
                self.assertLessEqual(r['max_age'],bridge.FRESH_REPORT_MAX_AGE+1e-9)
                self.assertGreaterEqual(r['min_gap'],bridge.FRESH_IMAGE_SLOT_MIN_INTERVAL-1e-9)

    def test_duplicates_and_out_of_order_cannot_refresh_or_add_reports(self):
        reference=replay_slot(True)
        copies=replay_slot(True,late_copies=True)
        self.assertEqual(copies['outputs'],reference['outputs'])

    def test_one_bad_depth_cannot_become_a_fresh_raw_point(self):
        noise=lambda t:(1.2 if abs(t-105.25)<1e-6 else 0.,0.)
        r=replay_slot(True,noise=noise)
        original=replay_slot(False,noise=noise)
        self.assertEqual(r['acceptance'],original['acceptance'])
        self.assertLess(r['max_error'],1.)
        self.assertFalse(any(abs(p['sample_s']-105.25)<1e-6 for p in r['outputs']))

    def test_scheduling_does_not_change_qualified_image_acceptance(self):
        for parameters in ({},{'rename':True},{'range_block':True},
                           {'qualify':False},{'tags':('red1','red2')}):
            with self.subTest(parameters=parameters):
                old=replay_slot(False,**parameters)
                new=replay_slot(True,**parameters)
                self.assertEqual(new['acceptance'],old['acceptance'])

    def test_fresh_but_unqualified_old_source_does_not_block_valid_challenger(self):
        import test_candidate_bridge as fixtures
        tests=fixtures.CandidateBridgeTests()
        node,t=tests.incumbent()
        for i in range(3):
            sample=t+.2*(i+1)
            tests.send(node,sample,[tests.candidate('old',0.,official_view_ok=False)])
            tests.send(node,sample+.02,[tests.candidate('new',0.)],source='u1')
        tr=node.core.tracks['blue']
        self.assertEqual(tr.official_source,'u1')
        self.assertAlmostEqual(tr.good_t,t+.62,places=7)
        self.assertEqual(node._pub_last['blue'],node._now())

    def test_two_red_instances_keep_independent_report_identity_and_coordinates(self):
        r=replay_slot(True,tags=('red1','red2'))
        for tag,candidate in (('red1','session:1'),('red2','session:2')):
            points=[p for p in r['outputs'] if p['tag']==tag]
            self.assertTrue(points)
            self.assertEqual({p['candidate'] for p in points},{candidate})
        self.assertLess(r['max_error'],1.)
        self.assertLess(r['max_gap'],.90)

    def test_cropped_unqualified_input_cannot_refresh_formal_sample(self):
        r=replay_slot(True,qualify=False)
        self.assertTrue(r['outputs'])
        self.assertLessEqual(max(p['sample_s'] for p in r['outputs']),108.)
        self.assertLessEqual(r['max_age'],bridge.FRESH_REPORT_MAX_AGE+1e-9)

    def test_closed_distance_gate_stays_closed_with_qualified_new_images(self):
        r=replay_slot(True,range_block=True)
        self.assertFalse(r['node']._gate_live['blue'])
        self.assertFalse(any(p['sample_s']>108. for p in r['outputs']))

    def test_local_id_rename_keeps_original_identity_protection(self):
        old=replay_slot(False,angle=0.,rename=True)
        new=replay_slot(True,angle=0.,rename=True)
        self.assertGreater(new['max_gap'],2.)
        self.assertEqual({p['candidate'] for p in new['outputs']},
                         {'session:1','session:11'})
        self.assertGreater(old['max_gap'],2.)

    def test_early_slot_needs_current_same_instance_and_new_image(self):
        tr=NS(official_source='typhoon_h480_0',official_candidate_id='session:1',good_t=100.25)
        previous=('typhoon_h480_0','session:1',100.)
        self.assertTrue(bridge.fresh_image_slot_allowed(tr,100.55,100.30,previous))
        for old in (None,('typhoon_h480_1','session:1',100.),
                    ('typhoon_h480_0','new-session:1',100.),
                    ('typhoon_h480_0','session:1',100.25),
                    ('typhoon_h480_0','session:1',101.)):
            self.assertFalse(bridge.fresh_image_slot_allowed(tr,100.55,100.30,old))
        for now in (100.2,100.61,float('nan')):
            self.assertFalse(bridge.fresh_image_slot_allowed(tr,now,100.30,previous))
        self.assertFalse(bridge.fresh_image_slot_allowed(tr,100.55,100.36,previous))
        with patch.object(bridge,'REPORT_POLICY','fit'):
            self.assertFalse(bridge.fresh_image_slot_allowed(tr,100.55,100.30,previous))

    def test_timer_cannot_use_the_callback_only_early_slot(self):
        node=bridge._mk_bridge();tr=node.core.tracks['blue']
        tr.official_source,tr.official_candidate_id='typhoon_h480_0','session:1'
        tr.good_history=[(99.2,-.8,0.),(99.6,-.4,0.),(100.,0.,0.)]
        tr.good_x,tr.good_y,tr.good_t=0.,0.,100.
        node._advance(100.3);node._publish_actor('blue',.15,0.)
        tr.good_x,tr.good_t=.25,100.25;tr.good_history.append((100.25,.25,0.))
        node._advance(.25)
        point=bridge.official_point_at(tr,node._now())
        self.assertIsNotNone(point)
        node._publish_actor('blue',*point)
        self.assertEqual(len(node._actor_pubs['blue'].msgs),1)
        node._publish_actor('blue',*point,from_image=True)
        self.assertEqual(len(node._actor_pubs['blue'].msgs),2)
        node._publish_actor('blue',*point,from_image=True)
        self.assertEqual(len(node._actor_pubs['blue'].msgs),2)

    def test_accepted_auxiliary_image_cannot_borrow_slot_for_old_official_image(self):
        import test_candidate_bridge as fixtures
        tests=fixtures.CandidateBridgeTests()
        node,t=tests.incumbent()
        node._advance(t+.30-node._now())
        for sample in (t+.02,t+.21):
            tests.send(node,sample,[tests.candidate('new',0.)],source='u1')
        tr=node.core.tracks['blue']
        node._publish_actor('blue',*bridge.official_point_at(tr,node._now()))
        count=len(node._actor_pubs['blue'].msgs)
        node._advance(t+.55-node._now())
        with patch.object(bridge,'FRESH_IMAGE_SLOT',False):
            tests.send(node,t+.25,[tests.candidate('old',0.)])
        self.assertEqual(tr.good_t,t+.25)
        self.assertEqual(len(node._actor_pubs['blue'].msgs),count)
        node._advance(.01)
        tests.send(node,t+.35,[tests.candidate('new',0.)],source='u1')
        self.assertEqual(tr.accepted_source_samples[('u1','new')],t+.35)
        self.assertEqual(tr.official_source,'u0')
        self.assertEqual(tr.good_t,t+.25)
        self.assertEqual(len(node._actor_pubs['blue'].msgs),count)
if __name__=='__main__':unittest.main()
