"""Image coverage is not completion of an explicitly queued lost-person job."""
import sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'coordination/src/robocup_swarm/scripts'))
import swarm_task as st
import test_search_segments as fixtures

class ReacquireCoveragePriorityTests(unittest.TestCase):
    def setup_job(self):
        g=st.CoverageGrid(0,42,0,7,7,num_uavs=1)
        a=st.TaskAllocator(g)
        key=a.queue_revisit('lost',31.5,3.5,now=100.,ttl_s=17.)
        with patch.object(st,'_now',return_value=101.):
            g.is_covered(31.5,3.5,.01)
        return g,a,key

    def test_processed_ground_coverage_does_not_hide_pending_person_revisit(self):
        g,a,key=self.setup_job()
        with patch.object(st,'_now',return_value=106.):
            assigned=a.allocate({'u0':(17.5,3.5)})
        self.assertEqual(assigned['u0'],key)

    def test_expired_blocked_or_owned_revisit_cannot_override_safety_or_ownership(self):
        for case in ('expired','blocked','owned'):
            g,a,key=self.setup_job()
            if case=='owned':g.cell(key).state=st.STATE_ASSIGNED;g.cell(key).owner='u1'
            with patch.object(st,'_now',return_value=118. if case=='expired' else 106.):
                assigned=a.allocate({'u0':(17.5,3.5)},blocked={key} if case=='blocked' else set())
            self.assertNotEqual(assigned['u0'],key)

class ReacquireSegmentDeadlineTests(unittest.TestCase):
    uid='typhoon_h480_0'
    setUp=fixtures.ManagerSegmentTests.setUp
    def test_started_revisit_keeps_original_image_deadline(self):
        key=(1,0)
        self.m.allocator.revisit_jobs[('lost',key)]=(99.,108.)
        task=self.m._start_search_segment(self.uid,[key],[(10.5,3.5)],100.)
        self.assertEqual(task.deadline,108.)
        self.assertEqual(task.expires,108.)

if __name__=='__main__':unittest.main()
