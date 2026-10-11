"""Patrol uses the same reservations, camera coverage and person preemption."""
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'coordination/src/robocup_swarm/scripts'))
import swarm_manager as sm
import swarm_agent as sa
import swarm_task as st
from search_segments import SegmentBook, TRAVEL
from boundary_patrol import SouthPatrol
from robocup_swarm.msg import UavStatus


class PatrolTests(unittest.TestCase):
    def setUp(self):
        self.now=100.
        for p in (patch.object(sm.rospy.Time,'now',side_effect=lambda:sm.rospy.Time.from_sec(self.now)),
                  patch.object(st.rospy,'get_time',side_effect=lambda:self.now)):
            p.start();self.addCleanup(p.stop)
        m=self.m=sm.SwarmManager.__new__(sm.SwarmManager)
        m._task_lock=threading.RLock();m._segments=SegmentBook(m._task_lock)
        m._segment_max_hold=90.;m._segment_stall_s=20.
        m.grid=st.CoverageGrid(-55,155,-65,65,7,6)
        m.allocator=st.TaskAllocator(m.grid);m.lease=st.LeaseManager(m.grid)
        m._g_infl=sm.GridMap(420,260,.5,(-55.,-65.),bytes(420*260))
        m._cell_waypoint={};m._blocked_cells=set();m._blind_viewpoints={}
        m._search_view_job=Mock(return_value=None);m._choose_blind_viewpoint=Mock(return_value=None)
        m._tracking={};m._backup={};m._active_leases={};m._lease_hold_since={};m._lease_motion={}
        m._release_cooldown={};m.assign_pub=NS(publish=Mock());m._auction_cycle=0
        m._reopen_covered_cells=Mock();m._south_patrol=SouthPatrol((-50,150,-60,60))
        m.uav_ids=['typhoon_h480_%d'%i for i in range(6)]
        m.status={};m.last_report={}
        for i,uid in enumerate(m.uav_ids):
            s=UavStatus(uav_id=uid,connected=True,x=float(i),y=0.,z=2.8)
            s.header.stamp=sm.rospy.Time(100)
            m.status[uid]=s;m.last_report[uid]=sm.rospy.Time(100)

    def allocate(self):
        self.m._allocate()
        return [t for t in self.m._segments.tasks.values() if t.purpose=='south_patrol']

    def test_one_nearest_idle_patrol_and_five_regular_routes_with_no_overlap(self):
        self.m.status[self.m.uav_ids[4]].y=-25.
        tasks=self.allocate();self.assertEqual(len(tasks),1)
        p=tasks[0];self.assertEqual(p.uid,self.m.uav_ids[4])
        self.assertEqual(len(self.m._segments.tasks),6)
        cells=[k for t in self.m._segments.tasks.values() for k in t.cells]
        self.assertEqual(len(cells),len(set(cells)))
        self.assertLessEqual(len(p.cells),3)
        for key,point,look in zip(p.cells,p.points,p.look_points):
            self.assertLess(look[1],-58.)
            self.assertGreater(point[1],look[1])
            self.assertNotEqual(point,look)
            self.assertNotEqual(self.m.grid.cell(key).state,st.STATE_COVERED)
        self.assertEqual(self.allocate(),tasks)

    def test_person_owner_and_stale_aircraft_cannot_be_selected(self):
        uid=self.m.uav_ids[4];self.m.status[uid].y=-50.
        self.m._tracking['t1']=uid
        other=self.m.uav_ids[3];self.m.status[other].y=-51.
        self.m.last_report[other]=sm.rospy.Time(98)
        p=self.allocate()[0]
        self.assertNotIn(p.uid,(uid,other))

    def test_search_to_person_releases_entire_patrol_without_false_coverage(self):
        p=self.allocate()[0];keys=p.cells
        self.m._free_search_lease(p.uid)
        self.assertNotIn(p.uid,self.m._segments.tasks)
        self.assertTrue(all(k not in self.m._segments.reserved for k in keys))
        self.assertIsNone(self.m._south_patrol.uid)
        self.assertTrue(all(self.m.grid.cell(k).state!=st.STATE_COVERED for k in keys))

    def test_first_visit_keeps_tail_and_unobserved_cell_reopens(self):
        p=self.allocate()[0];self.assertGreater(len(p.cells),1)
        s=self.m.status[p.uid];self.now=101.
        s.header.stamp=sm.rospy.Time(101);s.search_task_id=p.task_id;s.search_revision=p.revision
        s.search_state=TRAVEL;s.search_index=1;s.search_path_remaining=20.
        self.m.last_report[p.uid]=sm.rospy.Time(101)
        self.m._renew_segments(sm.rospy.Time(101))
        self.assertIn(p.uid,self.m._segments.tasks)
        self.assertEqual(p.index,1)
        self.assertNotEqual(self.m.grid.cell(p.cells[0]).state,st.STATE_COVERED)
        self.assertTrue(all(k in self.m._segments.reserved for k in p.cells[1:]))

    def test_blocked_south_band_does_not_create_an_unsafe_patrol(self):
        self.m._blocked_cells={k for k,c in self.m.grid.cells.items() if c.cy < -56.}
        self.assertEqual(self.allocate(),[])
        self.assertTrue(self.m._segments.tasks)

    def test_cycle_budget_and_empty_patrol_cooldown_do_not_reset_per_leg(self):
        p=SouthPatrol((-50,150,-60,60))
        for n in range(3):
            t=NS(uid='u0',task_id=str(n),deadline=190.+n*10,expires=120.+n*10)
            p.started(t,1,100.+n*10)
            self.assertEqual(t.deadline,190.)
            p.ended(t,'complete',105.+n*10)
        self.assertIsNone(p.uid);self.assertGreater(p.next_s,125.)
        before=p.empty_rounds;p.finish(200.)
        self.assertGreater(p.empty_rounds,before)

    def test_wire_regions_accepted_but_wrong_region_or_old_revision_cannot_replace(self):
        p=self.allocate()[0]
        msg=[c.args[0] for c in self.m.assign_pub.publish.call_args_list
             if c.args[0].search_task_id==p.task_id][0]
        a=sa.SwarmAgent.__new__(sa.SwarmAgent);a.uav_id=p.uid;a.cov_grid=self.m.grid
        a._plan_lock=threading.Lock();a._plan_epoch=0;a._plan_pending=None;a.path=[]
        self.assertTrue(a._accept_segment_message(msg))
        self.assertEqual(a._search_segment['look_points'],p.look_points)
        msg.search_revision+=1;msg.search_look_points[0].y+=4.
        self.assertFalse(a._accept_segment_message(msg))
        self.assertEqual(a._search_segment['revision'],1)


if __name__=='__main__':unittest.main()
