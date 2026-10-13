"""Assignment-bound image direction review; no actor/world report authority."""
import json
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import team_time_ros as rospy
import swarm_agent
import swarm_manager
import swarm_task
from search_view_hint import (CameraOrigins, SearchViewInbox, failed_view,
                              parse_failed_view, make_hint)
from robocup_swarm.msg import SearchAssignment


def assignment(stamp=109., uid='u0', key=(0, 0), kind=0):
    msg = SearchAssignment()
    msg.header.stamp = rospy.Time.from_sec(stamp)
    msg.uav_id = uid
    msg.cell_ix, msg.cell_iy = key
    msg.task_type = kind
    return msg


def message(image=100., sent=100.2, bearing=.7):
    cue = dict(sample_s=image, bearing=bearing, color='brown', candidate_id='brown:2',
               view_origin=dict(sample_s=image, x=-3.3, y=-3.1))
    return failed_view(cue, 'u0', sent)


def hint(stamp=109., bearing=.7):
    return make_hint(parse_failed_view(message(bearing=bearing), 100.2), assignment(stamp))


class SearchViewHintTests(unittest.TestCase):
    def state(self, bearing=.7, stamp=109.):
        state = SearchViewInbox()
        state.assign(assignment(stamp), stamp)
        self.assertTrue(state.offer(hint(stamp, bearing), 'u0', stamp+.01))
        return state

    def test_origin_requires_exact_image_time_and_cache_is_bounded(self):
        cache = CameraOrigins()
        cue = dict(sample_s=100., bearing=.7, color='brown', candidate_id='brown:2')
        cache.remember(99.9, 1., 2., 100.1)
        cache.attach(cue)
        self.assertIsNone(failed_view(cue, 'u0', 100.2))
        cache.remember(100., 3., 4., 100.1)
        cache.attach(cue)
        data = failed_view(cue, 'u0', 100.2)
        self.assertEqual((data['x'], data['y'], data['origin_sample_s']), (3., 4., 100.))
        cue['sample_s'] = 100.2
        cache.attach(cue)
        self.assertIsNone(cue['view_origin'])
        for k in range(200):cache.remember(100.+k*.02, 1., 2., 100.+k*.02)
        self.assertLessEqual(len(cache.frames), 32)

    def test_failed_view_expiry_uses_image_time_not_notification_time(self):
        view = parse_failed_view(message(image=100., sent=120.), 120.)
        self.assertEqual(view['expires_s'], 135.)
        self.assertIsNone(failed_view(dict(sample_s=100.,bearing=0.,color='brown',candidate_id='brown:2',view_origin=dict(sample_s=100.,x=0.,y=0.)), 'u0', 136.))
        self.assertIsNone(parse_failed_view(dict(message(), origin_sample_s=99.), 100.2))

    def test_hint_before_assignment_is_buffered_and_exactly_bound(self):
        state = SearchViewInbox()
        self.assertTrue(state.offer(hint(), 'u0', 109.01))
        self.assertIsNone(state.active)
        state.assign(assignment(109.02, key=(1, 0)), 109.02)
        self.assertIsNone(state.active)
        state = SearchViewInbox()
        state.offer(hint(), 'u0', 109.01)
        state.assign(assignment(), 109.02)
        self.assertEqual(state.active['assignment_s'], 109.)

    def test_wrong_uid_cell_stamp_cancel_and_tracking_cannot_use_old_hint(self):
        state = self.state()
        self.assertFalse(state.offer(hint(), 'u1', 109.1))
        state.assign(assignment(109.2, key=(1, 1)), 109.2)
        self.assertIsNone(state.active)
        self.assertFalse(state.offer(hint(), 'u0', 109.3))
        state.assign(assignment(109.4, kind=255), 109.4)
        self.assertFalse(state.offer(hint(109.2), 'u0', 109.5))
        state.assign(assignment(109.6, kind=1), 109.6)
        self.assertIsNone(state.active)

    def test_outside_two_metres_stale_camera_and_untrusted_pose_never_start(self):
        state = self.state()
        for xy, image in [((-.9, -3.1),109.1), ((-3.3,-3.1),108.)]:
            self.assertIsNone(state.step(109.1,xy,image,0.,0.))
        self.assertEqual(state.started,{})
        self.assertIsNone(state.step(109.2,(-3.3,-3.1),109.2,.7,.7,trusted=False))
        self.assertIsNone(state.active)

    def test_180_degree_turn_gets_total_four_second_budget_not_false_two_second_success(self):
        state = self.state(bearing=0.)
        first = state.step(109.1,(-3.3,-3.1),109.1,math.pi,math.pi)
        self.assertEqual(first['phase'],'turning')
        turning = state.step(111.2,(-3.3,-3.1),111.2,1.2,1.2)
        self.assertEqual(turning['phase'],'turning')
        self.assertIsNone(state.step(113.101,(-3.3,-3.1),113.101,.3,.3))
        self.assertEqual(state.last_result['reason'],'look_timeout')

    def test_only_actual_aligned_heading_and_new_processed_image_completes_look(self):
        state = self.state(bearing=0.)
        state.step(109.1,(-3.3,-3.1),109.1,math.pi,math.pi)
        self.assertEqual(state.step(110.9,(-3.3,-3.1),110.9,0.,.5)['phase'],'turning')
        self.assertEqual(state.step(111.,(-3.3,-3.1),111.,0.,0.)['phase'],'observing')
        self.assertIsNone(state.step(112.01,(-3.3,-3.1),112.01,0.,0.))
        self.assertEqual(state.last_result['reason'],'direction_observed')

    def test_duplicate_or_reassigned_same_job_never_extends_total_look(self):
        state = self.state(bearing=0.)
        state.step(109.1,(-3.3,-3.1),109.1,math.pi,math.pi)
        self.assertFalse(state.offer(hint(bearing=0.),'u0',109.5))
        state.assign(assignment(111.),111.)
        self.assertTrue(state.offer(hint(111.,0.),'u0',111.01))
        self.assertEqual(state.step(111.1,(-3.3,-3.1),111.1,1.,1.)['started_s'],109.1)
        self.assertIsNone(state.step(113.11,(-3.3,-3.1),113.11,1.,1.))
        self.assertEqual(state.last_result['reason'],'look_timeout')

    def test_expired_nonfinite_or_report_capable_payload_rejected(self):
        for changes in (dict(expires_s=109.),dict(expires_s=150.),dict(image_sample_s=90.),
                        dict(bearing=float('nan')),dict(real_detection=True),dict(report_eligible=True),
                        dict(assignment_s=108.),dict(origin_sample_s=101.)):
            state = SearchViewInbox()
            self.assertFalse(state.offer(dict(hint(),**changes),'u0',111.))


class SearchViewIntegrationTests(unittest.TestCase):
    def manager(self):
        m=swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        m.uav_ids=['u0'];m.status={'u0':NS(x=-3.3,y=-3.1,connected=True)}
        m._blocked_cells=set();m.grid=swarm_task.CoverageGrid(-7,7,-7,7,num_uavs=1)
        m.allocator=swarm_task.TaskAllocator(m.grid)
        m._cell_waypoint={};m._blind_viewpoints={}
        m._g_infl=NS(is_free=lambda p:True,world_to_cell=lambda p:p)
        m.assign_pub=Mock();m._search_view_hint_pub=Mock()
        return m

    def enqueue(self,m):
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(100.2)):
            m._visual_candidate_missed_cb(NS(data=json.dumps(message())))
        return (0,0)

    def test_publish_keeps_camera_origin_bearing_stamp_and_actual_dispatched_point(self):
        m=self.manager();key=self.enqueue(m)
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(109.)):
            m._publish_assignment('u0',key,m.grid.cell(key))
        msg=m.assign_pub.publish.call_args.args[0]
        data=json.loads(m._search_view_hint_pub.publish.call_args.args[0].data)
        self.assertEqual((msg.target_x,msg.target_y),(-3.3,-3.1))
        self.assertEqual(data['bearing'],.7)
        self.assertEqual(data['image_sample_s'],100.)
        self.assertEqual(data['assignment_s'],109.)
        self.assertEqual(data['expires_s'],135.)
        self.assertEqual(m._search_dispatched['u0'],(key,(-3.3,-3.1),109.))
        self.assertFalse(hasattr(m,'tracker'))

    def test_publish_failure_does_not_record_dispatched_or_send_hint(self):
        m=self.manager();key=self.enqueue(m);m.assign_pub.publish.side_effect=RuntimeError('send failure')
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(109.)):
            with self.assertRaises(RuntimeError):m._publish_assignment('u0',key,m.grid.cell(key))
        self.assertFalse(hasattr(m,'_search_dispatched'))
        m._search_view_hint_pub.publish.assert_not_called()

    def test_blocked_or_expired_origin_does_not_move_hint_to_some_other_point(self):
        for blocked,now in [(True,109.),(False,136.)]:
            m=self.manager();key=self.enqueue(m)
            m._g_infl.is_free=lambda p:not blocked
            with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(now)):
                m._publish_assignment('u0',key,m.grid.cell(key))
            m._search_view_hint_pub.publish.assert_not_called()
            msg=m.assign_pub.publish.call_args.args[0]
            self.assertEqual((msg.target_x,msg.target_y),(-3.5,-3.5))

    def test_plain_search_still_has_no_hint(self):
        m=self.manager()
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(109.)):
            m._publish_assignment('u0',(0,0),m.grid.cell((0,0)))
        m._search_view_hint_pub.publish.assert_not_called()

    def test_repeated_failed_view_cannot_renew_or_move_job_after_id_colour_change(self):
        m=self.manager();key=self.enqueue(m)
        original=dict(m.allocator.revisit_jobs)
        original_view=dict(m._search_view_jobs[key])
        # The original observation point has just been reserved. Previously
        # queue_revisit picked the adjacent free cell for the same failed look.
        m.grid.cell(key).owner='u0';m.grid.cell(key).state=swarm_task.STATE_ASSIGNED
        repeated=dict(message(image=103.,sent=103.2),color='blue',candidate_id='blue:88')
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(103.2)):
            m._visual_candidate_missed_cb(NS(data=json.dumps(repeated)))
        self.assertEqual(m.allocator.revisit_jobs,original)
        self.assertEqual(m._search_view_jobs[key],original_view)

    def test_failed_direction_budget_allows_a_different_direction_and_later_retry(self):
        m=self.manager();self.enqueue(m)
        m.allocator.queue_revisit=Mock(return_value=None)
        for now,bearing,expected in [(104.,.72,False),(105.,1.5,True),(161.,.7,True)]:
            m.allocator.queue_revisit.reset_mock()
            data=message(image=now-.2,sent=now,bearing=bearing)
            with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(now)):
                m._visual_candidate_missed_cb(NS(data=json.dumps(data)))
            self.assertEqual(m.allocator.queue_revisit.called,expected)

    def test_agent_hint_only_looks_and_holds_without_target_or_coverage_changes(self):
        state=SearchViewInbox();state.assign(assignment(),109.);state.offer(hint(),'u0',109.)
        a=NS(assignment=assignment(),_search_view_inbox=state,world_xy=(-3.3,-3.1),
             _world_trusted=lambda:True,_search_view_image_s=109.1,_search_view_image_yaw=0.,
             yaw=0.,_send_vel=Mock(),uav_id='u0',targets={'unchanged':(1,2)},_search_scan_seen={1})
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(109.1)):
            self.assertTrue(swarm_agent.SwarmAgent._hold_search_view_hint(a))
        a._send_vel.assert_called_once_with(0.,0.)
        self.assertAlmostEqual(math.atan2(a._look_at[1]+3.1,a._look_at[0]+3.3),.7)
        self.assertEqual(a.targets,{'unchanged':(1,2)});self.assertEqual(a._search_scan_seen,{1})
        a.assignment=assignment(109.2,kind=1)
        self.assertFalse(swarm_agent.SwarmAgent._hold_search_view_hint(a))


if __name__=='__main__':unittest.main()
