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

    def test_recovery_requires_elapsed_good_images_and_survives_duplicates(self):
        w=ReportViewWindow()
        for i in range(5):
            w.observe(dict(target_id='t3', world_candidate_id='white-A',
                           sample_s=100.+i*.2, geometry_ok=True, report_ok=i>0))
            self.assertEqual(w.recovered('t3',100.+i*.2+.01),i==4)
        self.assertFalse(w.recovered('t3',101.4))
        self.assertFalse(w.recovered('t2',100.9))
        w.observe(dict(target_id='t3',world_candidate_id='white-B',sample_s=100.9,
                       geometry_ok=True,report_ok=True))
        self.assertFalse(w.recovered('t3',100.91))

    def test_single_good_frame_cannot_cancel_active_side_move_or_extend_deadline(self):
        a=sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id='u3';a.local_xy=(8.,0.);a.offset=(0.,0.)
        a._orbit_target='t3';a._orbit_entered_t=90.
        a._last_eligible_image={'t3':100.};a._t_seen={'t3':100.}
        a._fresh_tracking_view=lambda now:dict(geometry_ok=True,report_ok=True,
                                               box=(300,220,340,330),height=480)
        a._safe_reobserve_segment=lambda *args:True
        a._reobserve_view=dict(tid='t3',goal=(7.,4.),tx=0.,ty=0.,until=102.)
        a._reobserve_next=106.
        a._report_view_window=ReportViewWindow()
        a._report_view_window.observe(dict(target_id='t3',world_candidate_id='A',
                          sample_s=100.,geometry_ok=True,report_ok=True))
        self.assertEqual(a._reobserve_goal(0.,0.,100.1),(7.,4.))
        self.assertEqual(a._reobserve_view['until'],102.)
        # Real safety changes and expired targets still veto the motion.
        a._safe_reobserve_segment=lambda *args:False
        self.assertIsNone(a._reobserve_goal(0.,0.,100.2))

if __name__=='__main__':unittest.main()
