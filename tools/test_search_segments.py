"""Search ownership, completion, cancellation and detour regression scenarios."""
import math
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'coordination/src/robocup_swarm/scripts'))
from search_segments import SegmentBook, TRAVEL, YIELD, DONE, FAILED, clear_line, path_remaining
import swarm_task as tasklib
import swarm_manager as sm
import swarm_agent as sa
from robocup_swarm.msg import SearchAssignment, UavStatus
from geometry_msgs.msg import Point


class ReservationTests(unittest.TestCase):
    def test_simultaneous_whole_segment_claims_are_disjoint(self):
        book=SegmentBook(); barrier=threading.Barrier(6); results=[]
        def request(uid):
            barrier.wait()
            def choose(reserved):
                keys=[(i,0) for i in range(18) if (i,0) not in reserved][:3]
                return keys,[(float(i),0.) for i,_ in keys]
            results.append(book.allocate(uid,choose,100.,20.,90.))
        workers=[threading.Thread(target=request,args=(str(i),)) for i in range(6)]
        for w in workers:w.start()
        for w in workers:w.join(2.)
        self.assertTrue(all(not w.is_alive() for w in workers))
        self.assertEqual(len(book.reserved),18)
        self.assertEqual(len({key for r in results for key in r.cells}),18)

    def test_conflict_never_leaves_partial_reservation(self):
        b=SegmentBook()
        b.reserve('a',[(1,0)],[(1.,0.)],100.,20.,90.)
        self.assertIsNone(b.reserve('b',[(0,0),(1,0)],[(0.,0.),(1.,0.)],100.,20.,90.))
        self.assertNotIn((0,0),b.reserved)
        self.assertNotIn('b',b.tasks)

    def test_first_waypoint_is_progress_not_whole_task_completion(self):
        b=SegmentBook();t=b.reserve('a',[(0,0),(1,0),(2,0)],[(0.,0.),(1.,0.),(2.,0.)],100.,20.,90.)
        self.assertEqual(b.advance('a',t.task_id,1,1),((0,0),))
        self.assertIn('a',b.tasks)
        self.assertEqual(set(b.reserved),{(1,0),(2,0)})
        self.assertIsNone(b.advance('a',t.task_id,1,0))
        self.assertIsNone(b.advance('a',t.task_id,1,4))

    def test_revise_releases_tail_and_rejects_old_cancel_without_extending_deadline(self):
        b=SegmentBook();t=b.reserve('a',[(0,0),(1,0)],[(0.,0.),(1.,0.)],100.,20.,90.)
        b.revise('a',[(0,0)],[(0.,1.)],110.)
        self.assertEqual(t.deadline,190.)
        self.assertEqual(t.revision,2)
        self.assertNotIn((1,0),b.reserved)
        self.assertIsNone(b.release('a',t.task_id,1))
        self.assertIn('a',b.tasks)

    def test_path_arc_progress_handles_a_detour_away_from_goal(self):
        path=[(0.,0.),(-3.,0.),(-3.,5.),(5.,5.),(5.,0.)]
        self.assertGreater(math.dist((-2.,0.),path[-1]),math.dist(path[0],path[-1]))
        self.assertAlmostEqual(path_remaining(path,(0.,0.))-path_remaining(path,(-2.,0.)),2.)

    def test_link_check_rejects_obstacles_and_diagonal_corner_cutting(self):
        g=sm.GridMap(3,3,1.,(0.,0.),bytes([0,1,0,0,0,0,0,0,0]))
        self.assertFalse(clear_line(g,(.5,.5),(1.5,1.5)))
        self.assertFalse(clear_line(g,(.5,.5),(2.5,.5)))
        self.assertTrue(clear_line(g,(.5,.5),(.5,2.5)))


class ManagerSegmentTests(unittest.TestCase):
    uid='typhoon_h480_0'
    def setUp(self):
        self.now=100.
        self.clock=patch.object(sm.rospy.Time,'now',side_effect=lambda:sm.rospy.Time.from_sec(self.now))
        self.clock.start();self.addCleanup(self.clock.stop)
        m=self.m=sm.SwarmManager.__new__(sm.SwarmManager)
        m._task_lock=threading.RLock();m._segments=SegmentBook(m._task_lock)
        m._segment_max_hold=90.;m._segment_stall_s=20.
        m.grid=tasklib.CoverageGrid(0,42,0,7,7,num_uavs=1)
        m.allocator=tasklib.TaskAllocator(m.grid);m.lease=tasklib.LeaseManager(m.grid)
        m._g_infl=sm.GridMap(80,30,1.,(-10.,-10.),bytes(80*30))
        m._cell_waypoint={k:(c.cx,c.cy) for k,c in m.grid.cells.items()}
        m._blocked_cells=set();m._blind_viewpoints={}
        m._search_view_job=Mock(return_value=None)
        m._choose_blind_viewpoint=Mock(return_value=None)
        m._tracking={};m._backup={};m._active_leases={};m._lease_hold_since={};m._lease_motion={}
        m._release_cooldown={};m.assign_pub=NS(publish=Mock());m._auction_cycle=0
        m.uav_ids=[self.uid];m._reopen_covered_cells=Mock()
        st=UavStatus(uav_id=self.uid,x=-3.5,y=3.5,z=3.,connected=True)
        st.header.stamp=sm.rospy.Time.from_sec(self.now)
        m.status={self.uid:st};m.last_report={self.uid:sm.rospy.Time.from_sec(self.now)}

    def reserve(self):
        keys=[(0,0),(1,0),(2,0)]
        t=self.m._segments.reserve(self.uid,keys,[self.m._cell_waypoint[k] for k in keys],100.,20.,90.)
        for k in keys:self.m.lease.grant(self.uid,k,100.,20.)
        self.m._active_leases[self.uid]=keys[0]
        self.feedback(t)
        return t

    def feedback(self,t,index=0,state=TRAVEL,remaining=30.,path_revision=1):
        st=self.m.status[self.uid]
        st.header.stamp=sm.rospy.Time.from_sec(self.now)
        st.search_task_id=t.task_id;st.search_revision=t.revision
        st.search_index=index;st.search_state=state
        st.search_path_revision=path_revision;st.search_path_remaining=remaining
        self.m.last_report[self.uid]=sm.rospy.Time.from_sec(self.now)
        return st

    def test_allocation_reserves_and_sends_three_cells_once(self):
        self.m._allocate()
        t=self.m._segments.tasks[self.uid]
        self.assertEqual(len(t.cells),3)
        self.assertEqual(len(self.m._segments.reserved),3)
        msg=self.m.assign_pub.publish.call_args.args[0]
        self.assertEqual(len(msg.search_waypoints),3)
        self.assertEqual(msg.search_task_id,t.task_id)
        self.m._allocate()
        self.assertEqual(len(self.m._segments.tasks),1)

    def test_blocked_connection_shortens_segment(self):
        g=self.m._g_infl;data=bytearray(g.cells)
        for y in range(g.height):data[y*g.width+17]=1 # x=7, between first/second
        self.m._g_infl=sm.GridMap(g.width,g.height,g.resolution,g.origin,data)
        self.m._allocate()
        self.assertEqual(len(self.m._segments.tasks[self.uid].cells),1)

    def test_image_coverage_of_first_cell_cannot_make_aircraft_idle(self):
        t=self.reserve();cell=self.m.grid.cell(t.cells[0])
        self.m.grid.is_covered(cell.cx,cell.cy,.01)
        self.assertEqual(self.m._idle_uavs(),[])
        self.assertEqual(len(self.m._segments.reserved),3)
        self.now=101.;self.feedback(t,index=1)
        self.m._renew_leases()
        self.assertEqual(self.m._segments.tasks[self.uid].index,1)
        self.assertEqual(self.m._active_leases[self.uid],(1,0))
        self.assertEqual(self.m.grid.cell((0,0)).state,tasklib.STATE_COVERED)
        self.assertEqual(self.m._idle_uavs(),[])

    def test_pursuit_release_frees_all_unfinished_cells_and_keeps_coverage(self):
        t=self.reserve();cell=self.m.grid.cell((0,0));self.m.grid.is_covered(cell.cx,cell.cy,.01)
        self.m._free_search_lease(self.uid)
        self.assertEqual(self.m._segments.tasks,{})
        self.assertEqual(self.m._segments.reserved,{})
        self.assertEqual(self.m.grid.cell((0,0)).state,tasklib.STATE_COVERED)
        self.assertEqual(self.m.grid.cell((1,0)).state,tasklib.STATE_FREE)
        msg=self.m.assign_pub.publish.call_args.args[0]
        self.assertEqual((msg.task_type,msg.search_task_id,msg.search_revision),(255,t.task_id,1))

    def test_real_image_coverage_callback_keeps_whole_segment_ownership(self):
        import json
        t=self.reserve();m=self.m
        m._slam_lock=threading.Lock()
        m._merged_grid=Mock(return_value=m._g_infl)
        m._in_processed_view=Mock(return_value=True)
        # Two processed camera samples cover first cell; they do not represent
        # flight completion and must not free this aircraft or its later cells.
        for now in (100.,100.3):
            self.now=now
            m._processed_camera_frame_cb(NS(data=json.dumps(dict(processed=True,
                source_uav=self.uid,sample_s=now,x=-3.5,y=3.5,yaw=0.,z=3.,down_pitch=0.))))
        self.assertEqual(m.grid.cell((0,0)).state,tasklib.STATE_COVERED)
        self.assertIs(m._segments.tasks[self.uid],t)
        self.assertEqual(t.index,0)
        self.assertEqual(set(m._segments.reserved),{(0,0),(1,0),(2,0)})
        self.assertEqual(m._idle_uavs(),[])

    def test_old_revision_completion_does_not_complete_revised_route(self):
        t=self.reserve();old=t.revision
        self.m._revise_segment(t,[(0,0)],[(3.5,4.5)],100.,'test')
        self.now=101.;st=self.feedback(t,index=1,state=DONE);st.search_revision=old
        self.m._renew_leases()
        self.assertIn(self.uid,self.m._segments.tasks)
        self.assertEqual(t.index,0)
        self.assertEqual(t.expires,120.)

    def test_completion_reopens_unseen_cells_and_does_not_credit_camera(self):
        t=self.reserve();self.now=101.;self.feedback(t,index=3,state=DONE)
        self.m._renew_leases()
        self.assertNotIn(self.uid,self.m._segments.tasks)
        self.assertTrue(all(self.m.grid.cell(k).state==tasklib.STATE_REVIEW for k in t.cells))
        self.assertEqual(self.m.allocator.last_seen,{})
        self.assertEqual(len(self.m.allocator.revisit_jobs),3)

    def test_disconnect_recovers_every_reserved_cell(self):
        self.reserve();self.m.status[self.uid].connected=False
        self.m._renew_leases()
        self.assertEqual(self.m._segments.reserved,{})
        self.assertEqual(self.m._active_leases,{})

    def test_fresh_heartbeat_without_matching_ack_does_not_extend_lease(self):
        t=self.reserve();self.now=119.;self.feedback(t).search_task_id='old'
        self.m._renew_leases();self.assertEqual(t.expires,120.)
        self.now=120.1;self.m.last_report[self.uid]=sm.rospy.Time.from_sec(self.now)
        self.m._renew_leases();self.assertEqual(self.m._segments.tasks,{})

    def test_detour_progress_renews_even_when_euclidean_goal_distance_grows(self):
        t=self.reserve();self.now=101.;self.feedback(t,remaining=30.)
        self.m._renew_leases()
        self.now=119.;st=self.feedback(t,remaining=28.);st.x-=2.
        self.m._renew_leases()
        self.assertEqual(t.progress_s,119.)
        self.assertIn(self.uid,self.m._segments.tasks)

    def test_replanning_cannot_reset_stall_clock(self):
        t=self.reserve()
        for now in (101.,110.,119.,121.):
            self.now=now;self.feedback(t,path_revision=int(now))
            self.m._renew_leases()
        self.assertNotIn(self.uid,self.m._segments.tasks)

    def test_temporary_friend_yield_does_not_become_waypoint_failure(self):
        t=self.reserve();self.now=101.;self.feedback(t,state=YIELD)
        self.m._renew_leases()
        self.now=119.;self.feedback(t,state=YIELD);self.m._renew_leases()
        self.now=125.;self.feedback(t,state=YIELD);self.m._renew_leases()
        self.assertIn(self.uid,self.m._segments.tasks)
        self.assertEqual(self.m.allocator.failed_visit,{})

    def test_local_blocked_tail_revises_and_releases_without_losing_first_cell(self):
        t=self.reserve();self.now=101.;st=self.feedback(t)
        st.search_blocked_waypoint=2;st.search_blocked_reason='GOAL_OCCUPIED'
        self.m._renew_leases()
        self.assertEqual(t.revision,2)
        self.assertEqual(t.cells,((0,0),))
        self.assertEqual(set(self.m._segments.reserved),{(0,0)})
        self.assertEqual(t.deadline,190.)

    def test_current_goal_failure_releases_promptly_without_coverage_credit(self):
        t=self.reserve();self.now=101.;st=self.feedback(t)
        st.search_blocked_waypoint=1;st.search_blocked_reason='GOAL_OCCUPIED'
        self.m._renew_leases()
        self.assertEqual(self.m._segments.tasks,{})
        self.assertEqual(self.m._segments.reserved,{})
        self.assertTrue(all(c.state!=tasklib.STATE_COVERED for c in self.m.grid.cells.values()))

    def test_inflation_exit_wait_is_bounded_even_with_fresh_feedback(self):
        t=self.reserve()
        for now in (101.,105.,108.):
            self.now=now;st=self.feedback(t,state=YIELD)
            st.search_blocked_waypoint=1;st.search_blocked_reason='START_OCCUPIED'
            self.m._renew_leases();self.assertIn(self.uid,self.m._segments.tasks)
        self.now=109.;self.feedback(t,state=YIELD);self.m._renew_leases()
        self.assertNotIn(self.uid,self.m._segments.tasks)


class AgentSegmentTests(unittest.TestCase):
    def setUp(self):
        self.clock=patch.object(sa.rospy.Time,'now',return_value=sa.rospy.Time(100.))
        self.clock.start();self.addCleanup(self.clock.stop)
        a=self.a=sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id='u0';a._plan_lock=threading.Lock();a._plan_epoch=0;a.path=[]
        a._plan_pending=None;a.path_target=None;a._plan_fail_t=0.;a._last_flight_v=(1.,0.)

    def message(self,revision=1):
        m=SearchAssignment(uav_id='u0',task_type=0,search_task_id='task-a',search_revision=revision)
        m.header.stamp=sa.rospy.Time(99.);m.search_valid_until=sa.rospy.Time(119.)
        m.search_cell_ix=[0,1,2];m.search_cell_iy=[0,0,0]
        m.search_waypoints=[Point(x=x,y=0.) for x in (3.5,10.5,17.5)]
        return m

    def test_renewal_keeps_partial_progress_and_attention_budget(self):
        m=self.message();self.assertTrue(self.a._accept_segment_message(m))
        self.a._search_segment['index']=1
        self.a._camera_cue={'until':102.}
        m.search_valid_until=sa.rospy.Time(120.)
        self.assertFalse(self.a._accept_segment_message(m))
        self.assertEqual(self.a._search_segment['index'],1)
        self.assertEqual(self.a._camera_cue['until'],102.)

    def test_revision_and_old_cancel_have_scoped_effect(self):
        self.a._accept_segment_message(self.message())
        self.assertTrue(self.a._accept_segment_message(self.message(2)))
        self.assertFalse(self.a._accept_segment_message(self.message(1)))
        cancel=SearchAssignment(uav_id='u0',task_type=255,search_task_id='task-a',search_revision=1)
        cancel.header.stamp=sa.rospy.Time(100.)
        self.assertFalse(self.a._accept_segment_message(cancel))
        self.assertEqual(self.a._search_segment['revision'],2)
        self.assertFalse(self.a._accept_segment_message(self.message(2)))

    def test_pursuit_cannot_be_overwritten_by_old_search_or_old_cancel(self):
        m=self.message();self.a._accept_segment_message(m)
        track=SearchAssignment(uav_id='u0',task_type=1,target_id='t2')
        track.header.stamp=sa.rospy.Time(100.)
        self.assertTrue(self.a._accept_segment_message(track))
        self.assertIsNone(self.a._search_segment)
        self.assertFalse(self.a._accept_segment_message(m))
        cancel=SearchAssignment(uav_id='u0',task_type=255,search_task_id='task-a',search_revision=1)
        self.assertFalse(self.a._accept_segment_message(cancel))

    def test_invalid_or_expired_segment_is_rejected(self):
        m=self.message();m.search_valid_until=sa.rospy.Time(99.)
        self.assertFalse(self.a._accept_segment_message(m))
        m=self.message();m.search_cell_iy=[]
        self.assertFalse(self.a._accept_segment_message(m))

    def test_old_planner_result_is_rejected_after_new_segment_version(self):
        a=self.a;a._plan_event=threading.Event();a._plan_event.set()
        m=self.message();a._accept_segment_message(m)
        a._request_plan((17.5,0.))
        started,finish=threading.Event(),threading.Event()
        def delayed(points):
            started.set();self.assertTrue(finish.wait(2.))
            return [(0.,0.),(17.5,0.)],(17.5,0.),True,0,''
        a._compute_segment_plan=delayed
        calls=[]
        def shutdown():calls.append(1);return len(calls)>1
        with patch.object(sa.rospy,'is_shutdown',side_effect=shutdown):
            worker=threading.Thread(target=a._planner_loop,daemon=True);worker.start()
            self.assertTrue(started.wait(2.))
            a._accept_segment_message(self.message(2));finish.set();worker.join(2.)
        self.assertFalse(worker.is_alive());self.assertEqual(a.path,[])
        self.assertIsNone(a.path_target)

    def control_fixture(self):
        a=self.a;m=self.message();a._accept_segment_message(m)
        a.assignment=m;a.assignment.cell_ix=a.assignment.cell_iy=0
        a.assignment.target_x=3.5;a.assignment.target_y=0.
        a.local_xy=(3.5,0.);a.offset=(0.,0.);a.local_z=3.
        a._search_camera_pose=(100.,3.5,0.,0.,3.,0.)
        a._search_scan_seen=set();a._search_scan_done=a._search_scan_failed=False
        a._search_scan_key=(0,0,3.5,0.)
        a._segment_path_key=('task-a',1)
        a.path=[(3.5,0.),(10.5,0.),(17.5,0.)];a.path_target=(17.5,0.)
        a.grid=sm.GridMap(50,20,1.,(-10.,-10.),bytes(50*20))
        a._send_vel=Mock();a._search_observable_points=Mock(return_value={})
        a._search_scan_step=Mock();a._set_search_heading=Mock()
        a._search_sweep_bearing=lambda bearing,distance:bearing
        a._transit_cap=lambda:10.;a._search_start_cap=lambda vx,vy:(vx,vy)
        a._apply_friend_avoidance=lambda vx,vy:(vx,vy)
        return a

    def test_safe_intermediate_visit_continues_without_zero_command(self):
        a=self.control_fixture()
        self.assertTrue(a._segment_control(3.,5.,.8,2.,2.))
        self.assertEqual(a._search_segment['index'],1)
        self.assertEqual(a.assignment.cell_ix,1)
        self.assertGreater(a._send_vel.call_args.args[0],0.)
        a._search_scan_step.assert_not_called()
        self.assertEqual(a.path_target,(17.5,0.))

    def test_missing_observable_direction_is_still_examined(self):
        a=self.control_fixture();a._search_observable_points.return_value={0:(8.,2.)}
        a._segment_control(3.,5.,.8,2.,2.)
        a._search_scan_step.assert_called_once()
        self.assertEqual(a._search_segment['index'],0)

    def test_real_candidate_interrupts_patrol_before_next_waypoint(self):
        a=self.control_fixture();a.local_xy=(0.,0.);a.yaw=0.
        a._takeoff_done=True;a._landing=False
        a._search_segment['purpose']='south_patrol'
        a._camera_cue=dict(candidate_id='blue:8',color='blue',target_id='t1',
            started=100.,until=102.,bearing=math.pi/2,xy=(0.,8.),image={'fresh':True})
        a._stable_other_owner=Mock(return_value=False);a._world_trusted=Mock(return_value=True)
        a._segment_control=Mock();a._search_scan_started_t=None
        with patch.object(sa,'candidate_decision',return_value=(0.,.3,'approach')):
            a._control()
        a._segment_control.assert_not_called();a._search_scan_step.assert_not_called()
        a._send_vel.assert_called_once_with(0.,.3)
        self.assertEqual(a._look_at,(0.,8.))

    def test_patrol_transit_disables_free_sweep_and_arrival_keeps_directed_look(self):
        a=self.control_fixture();a._search_segment['purpose']='south_patrol'
        a.local_xy=(0.,0.);a._search_sweep_bearing=Mock(side_effect=AssertionError('ordinary sweep'))
        a._segment_control(3.,5.,.8,2.,2.)
        a._search_sweep_bearing.assert_not_called()
        a.local_xy=(3.5,0.);a._search_observable_points.return_value={0:(8.,-6.)}
        a._segment_control(3.,5.,.8,2.,2.)
        a._search_scan_step.assert_called_once()

    def test_safe_prefix_survives_a_blocked_later_goal(self):
        a=self.control_fixture();g=a.grid;data=bytearray(g.cells)
        x,y=g.world_to_cell((10.5,0.));data[y*g.width+x]=1
        a.grid=sm.GridMap(g.width,g.height,g.resolution,g.origin,data)
        path,target,ok,blocked,reason=a._compute_segment_plan(((3.5,0.),(10.5,0.),(17.5,0.)))
        self.assertTrue(ok);self.assertEqual(blocked,2);self.assertEqual(reason,'GOAL_OCCUPIED')
        self.assertEqual(target,(3.5,0.));self.assertEqual(path[-1],target)
        self.assertTrue(all(clear_line(a.grid,p,q) for p,q in zip(path,path[1:])))

    def test_blocked_tail_is_never_executed_before_manager_revision(self):
        a=self.control_fixture();a._search_segment.update(blocked=2,reason='GOAL_OCCUPIED')
        a._segment_control(3.,5.,.8,2.,2.)
        self.assertEqual(a._search_segment['index'],1)
        a._send_vel.assert_called_once_with(0.,0.)

    def test_tail_revision_does_not_reset_scan_budget(self):
        a=self.control_fixture();a._search_scan_started_t=97.;a._search_scan_phase=2
        msg=self.message(2);msg.search_waypoints=msg.search_waypoints[:1]
        msg.search_cell_ix=msg.search_cell_ix[:1];msg.search_cell_iy=msg.search_cell_iy[:1]
        a._accept_segment_message(msg)
        self.assertEqual(a._search_scan_key,(0,0,3.5,0.))
        self.assertEqual(a._search_scan_started_t,97.)
        self.assertEqual(a._search_scan_phase,2)

    def test_inflation_start_can_only_use_checked_exit_not_direct_goal_velocity(self):
        a=self.control_fixture();a._search_segment.update(blocked=1,reason='START_OCCUPIED')
        a.grid.is_free=Mock(return_value=False);a.grid.is_free_raw=Mock(return_value=True)
        a.local_xy=(2.,0.);a._occupied_cell_exit=Mock(return_value=(0.,.5))
        a._segment_control(3.,5.,.8,2.,2.)
        a._occupied_cell_exit.assert_called_once()
        a._send_vel.assert_called_once_with(0.,.5)
        a.grid.is_free_raw.return_value=False;a._send_vel.reset_mock()
        a._segment_control(3.,5.,.8,2.,2.)
        a._send_vel.assert_called_once_with(0.,0.)

    def test_expired_segment_stops_without_reusing_previous_velocity(self):
        a=self.control_fixture();a._search_segment['until']=99.
        a._segment_control(3.,5.,.8,2.,2.)
        self.assertEqual(a._search_segment['state'],FAILED)
        self.assertEqual(a.path,[])
        a._send_vel.assert_called_once_with(0.,0.)

    def test_generated_wire_protocol_preserves_segment_fields(self):
        import io
        msg=self.message(2);msg.search_purpose='south_patrol'
        msg.search_look_points=[Point(x=3.5,y=-8.5,z=0.)]
        buffer=io.BytesIO();msg.serialize(buffer)
        received=SearchAssignment();received.deserialize(buffer.getvalue())
        self.assertEqual(received.search_task_id,'task-a')
        self.assertEqual(received.search_revision,2)
        self.assertEqual(tuple(received.search_cell_ix),(0,1,2))
        self.assertEqual(received.search_waypoints[-1].x,17.5)
        self.assertEqual(received.search_valid_until.to_sec(),119.)
        self.assertEqual(received.search_purpose,'south_patrol')
        self.assertEqual(received.search_look_points[0].y,-8.5)


if __name__=='__main__':unittest.main()
