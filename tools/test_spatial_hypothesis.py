"""A long-lived same-colour decoy must not permanently mask another camera."""
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'coordination/src/robocup_swarm/scripts'))
import yolo_target_bridge as bridge


class SpatialHypothesisTests(unittest.TestCase):
    def incumbent(self, tag='blue', seconds=40., speed=0.):
        core = bridge.TargetBridgeCore()
        for k in range(int(seconds*5)+1):
            t = 100.+k*.2
            core.report(t, tag, speed*(t-100.), 0., .9, 8., source_uav='u0')
        return core, t

    def challenge(self, core, t, tag='blue', **kw):
        for k in range(8):
            sample = t+.2*(k+1)
            core.report(sample, tag, core.tracks[tag].x, 0., .9, 8., source_uav='u0')
            core.report(sample+.01, tag, 100., 20., .9, 8.,
                        source_uav='u1', **kw)
        return core.tracks[tag]

    def test_mature_stationary_incumbent_yields_without_coordinate_averaging(self):
        core, t = self.incumbent()
        tr = self.challenge(core,t)
        self.assertEqual(tr.official_source,'u1')
        self.assertEqual((tr.good_x,tr.good_y),(100.,20.))
        self.assertTrue(all(p[1]>90. for p in tr.good_history))
        core.report(t+2.,'blue',0.,0.,.9,8.,source_uav='u0')
        self.assertEqual(core.tracks['blue'].official_source,'u1')
        self.assertEqual(core.tracks['blue'].x,100.)
        self.assertFalse(core.eliminated)

    def test_active_confirmation_window_cannot_be_stolen(self):
        core,t = self.incumbent(seconds=10.)
        tr = self.challenge(core,t)
        self.assertEqual(tr.official_source,'u0')

    def test_moving_incumbent_cannot_be_stolen(self):
        core,t = self.incumbent(speed=1.)
        for k in range(8):
            sample=t+.2*(k+1)
            core.report(sample,'blue',sample-100.,0.,.9,8.,source_uav='u0')
            core.report(sample+.01,'blue',100.,20.,.9,8.,source_uav='u1')
        self.assertEqual(core.tracks['blue'].official_source,'u0')

    def test_unqualified_images_and_duplicate_images_cannot_switch(self):
        core,t = self.incumbent()
        self.assertEqual(self.challenge(core,t,official_ok=False).official_source,'u0')
        for _ in range(30):
            core.report(t+2.,'blue',100.,20.,.9,8.,source_uav='u1')
        self.assertEqual(core.tracks['blue'].official_source,'u0')

    def test_red_identities_never_use_single_colour_hypothesis_switch(self):
        core,t = self.incumbent(tag='red1')
        self.assertEqual(self.challenge(core,t,tag='red1').official_source,'u0')

    def test_actual_two_camera_blue_stream_reaches_eastern_candidate(self):
        rows=json.loads((ROOT/'tools/fixtures/blue_disjoint_102820.json').read_text())['reports']
        core=bridge.TargetBridgeCore()
        accepted=[]
        for r in rows:
            core.report(r['sample'],'blue',r['x'],r['y'],r['confidence'],r['range_m'],
                        official_ok=r['official_ok'],source_uav=r['source'])
            tr=core.tracks['blue']
            if tr.accepted_source_samples.get('typhoon_h480_5')==r['sample']:
                accepted.append(r['sample'])
        self.assertGreaterEqual(len(accepted),10)
        self.assertLess(accepted[0],2828.)
        self.assertGreater(core.tracks['blue'].x,100.)


if __name__=='__main__':
    unittest.main()
