"""Fresh-image experiment: accuracy AND continuity, not just publish count."""
import math
import unittest
from unittest.mock import patch
from test_report_turn_continuity import bridge,replay


class FreshReportPolicyTests(unittest.TestCase):
    def setUp(self):
        p=patch.object(bridge,'REPORT_POLICY','fresh');p.start();self.addCleanup(p.stop)
        p=patch.object(bridge,'_core_log',lambda *args:None);p.start();self.addCleanup(p.stop)

    def test_turn_accuracy_and_continuity_across_timer_phases(self):
        for speed in (0.,1.,2.):
          for angle in (0.,math.pi/2,math.pi):
           for hz in (4.,5.):
            for corner in (8.,8.05,8.1,8.15,8.2):
             for phase in (0.,.025,.05):
              with self.subTest(speed=speed,angle=angle,hz=hz,corner=corner,phase=phase):
                r=replay(speed,angle,hz,.2,corner,phase)
                self.assertLess(r['max_error'],.85)
                self.assertLess(r['max_gap'],.90)
                self.assertGreater(r['count'],30)

    def test_noisy_motion_and_isolated_bad_depth_do_not_become_raw_reports(self):
        for tag in ('blue','red1'):
          for angle in (0.,math.pi/2,math.pi):
            # 10cm bounded measurement noise and an isolated 1.2m depth error.
            noise=lambda t: ((1.2 if abs(t-105.)<.01 else .08*math.sin(t*7.)),
                             .05*math.cos(t*11.))
            with self.subTest(tag=tag,angle=angle):
                r=replay(2.,angle,5.,.2,8.15,.025,noise,tag)
                self.assertLess(r['max_error'],1.)
                self.assertLess(r['max_gap'],1.)

    def test_expired_or_rejected_newest_image_has_no_raw_fallback(self):
        tr=bridge._Track('blue')
        tr.good_history=[(100.+i*.2,float(i)*.2,0.) for i in range(10)]
        tr.good_x,tr.good_y,tr.good_t=1.8,0.,101.8
        self.assertIsNotNone(bridge.official_point_at(tr,102.1))
        self.assertIsNone(bridge.official_point_at(tr,102.16))
        tr.good_history.append((102.,3.2,0.))
        tr.good_x,tr.good_t=3.2,102.
        self.assertIsNone(bridge.official_point_at(tr,102.1))

    def test_stationary_coordinate_keeps_robust_location(self):
        c=bridge.TargetBridgeCore()
        for i in range(15):
            c.report(100.+i*.2,'blue',20.+.04*math.sin(i),10.,.9,8.,source_uav='a')
        tr=c.tracks['blue']
        p=bridge.official_point_at(tr,tr.good_t+.2)
        self.assertLess(math.hypot(p[0]-20.,p[1]-10.),.04)
        self.assertEqual(bridge.official_report_interval(tr),
                         bridge.official_report_interval(tr,True))


if __name__=='__main__':unittest.main()
