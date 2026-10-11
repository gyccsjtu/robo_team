"""A sporadic valid frame must not indefinitely suppress bounded reimaging."""
import sys, unittest
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa
from tracking_view import ReportViewWindow

class ReportViewWindowTests(unittest.TestCase):
    def window(self,hz=5,good_every=3):
        w=ReportViewWindow()
        for i in range(int(4*hz)+1):
            w.observe(dict(target_id='t5',world_candidate_id='red-A',sample_s=96.+i/hz,
                           geometry_ok=True,report_ok=i%good_every==0))
        return w

    def test_fresh_sparse_good_views_request_reimage_at_multiple_rates(self):
        for hz in (4,5,10,20):
            self.assertTrue(self.window(hz).degraded('t5',100.1))
        self.assertFalse(self.window(good_every=1).degraded('t5',100.1))

    def test_stale_other_person_and_duplicate_frames_cannot_build_quality_time(self):
        w=self.window()
        self.assertFalse(w.degraded('t4',100.1))
        self.assertFalse(w.degraded('t5',101.))
        w=ReportViewWindow()
        for i in range(100):
            w.observe(dict(target_id='t5',world_candidate_id='red-A',sample_s=100.,geometry_ok=True,report_ok=False))
        self.assertFalse(w.degraded('t5',100.1))
        w=self.window()
        w.observe(dict(target_id='t5',world_candidate_id='red-B',sample_s=100.1,geometry_ok=True,report_ok=False))
        self.assertFalse(w.degraded('t5',100.2))

    def test_good_current_frame_does_not_cancel_persistent_poor_geometry_recovery(self):
        a=sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id='u5';a.local_xy=(8.,0.);a.offset=(0.,0.)
        a._orbit_target='t5';a._orbit_entered_t=90.
        a._last_eligible_image={'t5':100.};a._t_seen={'t5':100.}
        a._fresh_tracking_view=lambda now:dict(geometry_ok=True,report_ok=True,
                                              box=(300,220,340,330),height=480)
        a._safe_reobserve_segment=lambda *args:True;a.los=NS(visible=lambda *args:True)
        a._report_view_window=self.window()
        self.assertIsNotNone(a._reobserve_goal(0.,0.,100.1))
        self.assertAlmostEqual(a._reobserve_view['until'],106.1)
        self.assertAlmostEqual(a._reobserve_next,110.1)
        # Recovery has the same fixed budget; a new failing image cannot extend it.
        self.assertIsNone(a._reobserve_goal(0.,0.,107.))
        a._report_view_window=self.window(good_every=1)
        self.assertIsNone(a._reobserve_goal(0.,0.,100.1))

if __name__=='__main__':unittest.main()
