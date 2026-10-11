"""Multi-camera control/report regressions from the 08:05:56 run."""
import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'coordination/src/robocup_swarm/scripts'))
import yolo_target_bridge as bridge


class CameraContinuityTests(unittest.TestCase):
    def test_live_source_is_not_replaced_by_agreeing_alternating_camera(self):
        core = bridge.TargetBridgeCore()
        for i in range(20):
            t = 100.+i*.2
            core.report(t,'white',t,0.,.9,8.,source_uav='u0')
            core.report(t+.1,'white',t+.3,0.,.9,8.,source_uav='u1')
        tr = core.tracks['white']
        self.assertEqual(tr.official_source,'u0')
        self.assertGreater(len(tr.good_history),5)
        self.assertAlmostEqual(tr.accepted_source_samples['u1'],103.9)
        # The auxiliary camera remains eligible for dispatch/task health.
        self.assertAlmostEqual(bridge.official_point_at(tr,104.)[0],104.)
        core.report(104.2,'white',104.4,0.,.9,8.,source_uav='u1')
        core.report(104.5,'white',104.7,0.,.9,8.,source_uav='u1')
        self.assertEqual(tr.official_source,'u1')
        self.assertEqual(len(tr.good_history),1)

    def test_source_bias_does_not_create_velocity_or_escape(self):
        for speed in (0.,1.,2.):
            core = bridge.TargetBridgeCore()
            for i in range(30):
                t = 100.+i*.2
                core.report(t,'brown',speed*(t-100.),0.,.9,8.,source_uav='u0')
                # Other camera intermittently disappears: fusion weights vary.
                if i%7 < 4:
                    core.report(t+.08,'brown',speed*(t+.08-100.)+.9,.4,
                                .9,8.,source_uav='u1')
                if i>=5:
                    tr = core.tracks['brown']
                    self.assertAlmostEqual(tr.vx,speed,places=6)
                    self.assertAlmostEqual(tr.vy,0.,places=6)

    def test_expired_or_unmodelled_motion_cannot_drive_following(self):
        self.assertEqual(bridge.camera_motion_velocity({'u0':[(1.,0.,0.),(1.4,.4,0.),
                         (1.8,.8,0.)]},3.),(0.,0.))
        self.assertEqual(bridge.camera_motion_velocity({'u0':[(1.,0.,0.),(1.4,4.,0.),
                         (1.8,8.,0.)]},1.8),(0.,0.))

    def test_recorded_reports_reduce_source_churn_without_stale_publication(self):
        fixture=json.loads((ROOT/'tools/fixtures/tracking_reports_080556.json').read_text())
        core=bridge.TargetBridgeCore(); switches={'white':0,'brown':0}; last={}
        for r in fixture['reports']:
            core.report(r['t'],r['tag'],r['x'],r['y'],r['conf'],r['range_m'],
                        r['official_ok'],r['source'])
            tr=core.tracks[r['tag']]
            if tr.official_source!=last.get(r['tag']):
                switches[r['tag']]+=1;last[r['tag']]=tr.official_source
            self.assertLessEqual(math.hypot(tr.vx,tr.vy),2.5+1e-6)
            self.assertIsNone(bridge.official_point_at(tr,tr.good_t+.61))
        # Recorded baseline on exactly these reports: white 86, brown 195.
        self.assertLess(switches['white'],43)
        self.assertLess(switches['brown'],98)


if __name__=='__main__':
    unittest.main()
