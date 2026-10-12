"""Recorded moving-green re-ID must not strand the assigned observer."""
import json,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'coordination/src/robocup_swarm/scripts'))
import yolo_target_bridge as b

class LocalContinuationTests(unittest.TestCase):
    def row(self,t,cid,x,**kw):
        r=dict(tag='green',source_uav='u0',candidate_id=cid,sample_s=t,x=x,y=0.,
               conf=.9,range_m=8.,official_ok=True,camera_confirmed=True,
               appearance_status='actor',appearance_confidence=.9)
        r.update(kw);return r
    def core(self):
        c=b.TargetBridgeCore()
        for i in range(11):c.report_frame([self.row(100+i*.2,'old',i*.2)])
        return c
    def test_exact_live_green_reconnects_without_relocation_or_elimination(self):
        c=b.TargetBridgeCore();accepted=[]
        f=json.loads((ROOT/'tools/fixtures/green_rebuild_034605.json').read_text())
        for r in f['frames']:
            rows=[x for x in b.parse_report_frame(json.dumps(r['data']),r['ros']) if x['tag']=='green']
            selected=c.report_frame(rows)
            accepted.extend(x['sample_s'] for x in selected if x['candidate_id'].endswith(':6'))
        self.assertAlmostEqual(accepted[0],2014.544)
        self.assertIn(2015.744,accepted) # control-only frame keeps following
        self.assertGreater(c.tracks['green'].t_obs,2018.)
        self.assertFalse(c._hypothesis_switches);self.assertFalse(c._deferred_sites)
        self.assertFalse(c.eliminated)
    def test_three_fresh_agreeing_images_continue_same_motion(self):
        c=self.core()
        for i in range(3):
            t=102.2+i*.2;c.report_frame([self.row(t,'new',t-100.)])
            self.assertEqual(c.tracks['green'].tracking_candidate_id,'old' if i<2 else 'new')
    def test_duplicates_ambiguity_and_wrong_motion_do_not_rebind(self):
        for mode in ('duplicate','two_people','jump','stationary_intruder','red','unqualified','late','unconfirmed'):
            c=self.core()
            for i in range(4):
                t=102.2+(0. if mode=='duplicate' else i*.2)
                r=self.row(t,'new',t-100.)
                if mode=='jump':r['x']+=3.
                if mode=='stationary_intruder':r['x']=2.2
                if mode=='red':r['tag']='red1'
                if mode=='unqualified':r['official_ok']=False
                if mode=='late':r['sample_s']+=3.
                if mode=='unconfirmed':r['camera_confirmed']=False
                rows=[r]
                if mode=='two_people':rows.append(self.row(t,'old',t-100.))
                c.report_frame(rows)
            self.assertEqual(c.tracks['green'].tracking_candidate_id,'old',mode)
    def test_other_camera_uses_existing_handoff_not_local_id_rebind(self):
        c=self.core();tr=c.tracks['green'];before=dict(tr.source_candidates)
        for i in range(4):
            t=102.2+i*.2
            c._continue_local_instance(tr,[self.row(t,'new',t-100.,source_uav='u1')])
        self.assertEqual(tr.source_candidates,before)
        self.assertFalse(c._local_continuations)
    def test_alias_cannot_move_fresh_report_on_single_bad_point(self):
        c=self.core()
        for i in range(3):
            t=102.2+i*.2;c.report_frame([self.row(t,'new',t-100.)])
        tr=c.tracks['green'];good=tr.good_t
        c.report_frame([self.row(102.8,'new',4.5)])
        self.assertEqual(tr.good_t,good)

if __name__=='__main__':unittest.main()
