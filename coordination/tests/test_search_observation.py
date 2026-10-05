"""Real callback extraction + search evidence/occupancy boundary regressions."""
import ast
import copy
import io
import json
import math
import os
from pathlib import Path
import sys
import threading
from types import MethodType, SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT/'coordination/src/robocup_swarm/scripts'
sys.path.insert(0, str(SCRIPTS))
from pose_rate import PoseRate, motion_evidence
sys.path.insert(0, str(ROOT/'perception'))
from search_observation import (FrameCache, ObservationLedger, SearchSweep, valid_frame,
    visible_samples, proven_points, in_image, MIN_FRAMES, probes)
sys.path.insert(0, str(ROOT/'coordination/src/robocup_navigation/src'))
from online_radar_planner import OnlinePlanner
from robocup_navigation.astar import GridMap
from swarm_task import CoverageGrid, TaskAllocator, STATE_ASSIGNED, STATE_FREE, STATE_COVERED, STATE_REVIEW
from radar_observed_map import ObservedMap
from navigation_feedback import RejectedTasks
from task_authority import TaskAuthority, TaskGate
from search_occupancy import apply_authority_release, apply_intent_cancellation
from camera_task import CameraTask

UID = 'uav_1'
RUN = 'search-run'


def task():
    return dict(task_type=0, target_id='', cell_ix=0, cell_iy=0, target_x=3.5, target_y=0.)


def active():
    return dict(task=task(), generation=1, stopping=False, started_s=100., expires_s=110.)


def frame(seq=1, stamp=102.5):
    return dict(schema_version=1, run_id=RUN, uav_id=UID, generation=1, cell=[0, 0], seq=seq,
        image_s=stamp, sample_s=stamp+.05, inference_complete=True, size=[640, 360],
        intrinsics=[300., 300., 320., 180.], camera_xyz=[-3.5, 0., 2.2],
        camera_rotation=[1., 0., 0., 0., 1., 0., 0., 0., 1.])


def feedback(frames=None, **overrides):
    result = dict(schema_version=1, run_id=RUN, uav_id=UID, generation=1, seq=1,
        sample_s=103.05, cell=[0, 0], position_xy=[-3.5, 0.], view_idx=0, view_xy=[-3.5, 0.],
        phase='NEXT_VIEW', outcome='OBSERVED', finished=True, window_start_s=100.5,
        blocked_since_s=100., frames=frames or [])
    result.update(overrides)
    return result


def evidence(mask=31):
    cache = FrameCache(RUN, [UID])
    proofs = []
    for seq, stamp in enumerate((101.5, 102., 102.5), 1):
        f = frame(seq, stamp)
        assert cache.receive(f, active(), stamp+.1)
        proofs.append(dict(seq=seq, image_s=stamp, visible=[dict(cell=[0, 0], mask=mask)]))
    return cache, proofs


def motion(now=103.05):
    return {UID: dict(sample_s=now, position_xy=[-3.5, 0.], velocity_xy=[0., 0.])}


def accept(ledger, message, cache, now=103.05, tasks=None, samples=None):
    return ledger.accept(message, RUN, [UID], tasks or {UID: active()}, samples or motion(now),
                         cache, CoverageGrid(0., 14., -3.5, 3.5, 7.), now)


class FrameEvidenceTests(unittest.TestCase):
    def test_empty_inference_is_evidence_but_received_only_is_not(self):
        self.assertTrue(valid_frame(frame(), 102.6))
        self.assertFalse(valid_frame(dict(frame(), inference_complete=False), 102.6))

    def test_stale_future_and_nonorthogonal_geometry_rejected(self):
        for f, now in [(frame(), 103.6), (frame(), 102.4),
                       (dict(frame(), camera_rotation=[0.]*9), 102.6),
                       (dict(frame(), intrinsics=[-1., 300., 320., 180.]), 102.6),
                       (dict(frame(), seq=True), 102.6)]:
            with self.subTest(f=f):
                self.assertFalse(valid_frame(f, now))

    def test_run_generation_pregrant_stop_and_duplicate_rejected(self):
        cache = FrameCache(RUN, [UID])
        self.assertTrue(cache.receive(frame(), active(), 102.6))
        self.assertFalse(cache.receive(frame(2), active(), 102.7))
        for f, a in [(dict(frame(2, 102.7), run_id='other'), active()),
                     (dict(frame(2, 102.7), generation=2), active()),
                     (frame(2, 102.7), dict(active(), stopping=True)),
                     (frame(2, 102.7), dict(active(), started_s=102.8))]:
            self.assertFalse(cache.receive(f, a, 102.8))

    def test_cached_geometry_cannot_be_changed_after_receive(self):
        cache, f = FrameCache(RUN, [UID]), frame()
        self.assertTrue(cache.receive(f, active(), 102.6))
        f['camera_rotation'][0] = 0.
        self.assertEqual(cache.frames[UID][1][0]['camera_rotation'][0], 1.)

    def test_actual_fov_and_fresh_planar_clearance_both_required(self):
        grid = CoverageGrid(0., 14., -3.5, 3.5, 7.)
        observed = ObservedMap(120, 80, .25, (-10., -10.))
        observed.cells = [0]*len(observed.cells)
        observed.observed_s = [102.5]*len(observed.cells)
        observed.last_scan_s = 102.5
        observed.range_min = .5
        entries = visible_samples(frame(), grid, observed, 102.6)
        self.assertEqual(next(e['mask'] for e in entries if e['cell'] == [0, 0]), 31)
        for i in range(len(observed.cells)):
            if i % observed.width == observed.index(0., 0.) % observed.width:
                observed.cells[i] = 100
        self.assertEqual(visible_samples(frame(), grid, observed, 102.6), [])
        observed.cells = [-1]*len(observed.cells)
        self.assertEqual(visible_samples(frame(), grid, observed, 102.6), [])
        self.assertFalse(in_image(frame(), (-4., 0.)))

    def test_obstacle_inside_one_meter_is_not_skipped_as_body_clearance(self):
        grid = CoverageGrid(0., 14., -3.5, 3.5, 7.)
        observed = ObservedMap(120, 80, .25, (-10., -10.))
        observed.cells, observed.observed_s = [0]*len(observed.cells), [102.5]*len(observed.cells)
        observed.last_scan_s, observed.range_min = 102.5, .5
        for i in range(len(observed.cells)):
            if i % observed.width == observed.index(-2.75, 0.) % observed.width:
                observed.cells[i] = 100
        self.assertEqual(visible_samples(frame(), grid, observed, 102.6), [])

    def test_observation_age_uses_original_images_and_revisits_without_full_map(self):
        cache, proofs = evidence()
        ledger = ObservationLedger()
        self.assertTrue(accept(ledger, feedback(proofs), cache))
        grid = CoverageGrid(0., 14., -3.5, 3.5, 7.)
        ledger.sync_grid(grid, {}, {}, 103.05)
        self.assertEqual(grid.cell((0, 0)).state, STATE_COVERED)
        self.assertEqual(grid.cell((1, 0)).state, STATE_FREE)
        ledger.sync_grid(grid, {}, {}, 132.5)
        self.assertEqual(grid.cell((0, 0)).state, STATE_REVIEW)
        self.assertEqual(ledger.latest((0, 0)), 102.5)

    def test_receipt_does_not_release_active_or_retired_search_occupancy(self):
        cache, proofs = evidence()
        ledger, grid = ObservationLedger(), CoverageGrid(0., 14., -3.5, 3.5, 7.)
        c = grid.cell((0, 0))
        c.state, c.owner = STATE_ASSIGNED, UID
        self.assertTrue(accept(ledger, feedback(proofs), cache))
        for retired in (False, True):
            ledger.sync_grid(grid, {('search', 0, 0): dict(owner=UID, retired=retired)}, {}, 103.05)
            self.assertEqual((c.state, c.owner), (STATE_ASSIGNED, UID))
        event = dict(schema_version=2, run_id=RUN, event='TASK_RELEASED', uav_id=UID, details=dict(key=['search', 0, 0]))
        self.assertTrue(apply_authority_release(grid, event, RUN, {}))
        ledger.sync_grid(grid, {}, {}, 103.1)
        self.assertEqual(c.state, STATE_COVERED)

    def test_two_frames_duplicate_images_bad_masks_and_old_generation_do_not_cover(self):
        cache, proofs = evidence()
        cases = [feedback(proofs[:2]), feedback([proofs[0]]*3),
                 feedback(proofs, generation=2), feedback(proofs, view_idx=2),
                 feedback(proofs, window_start_s=102.), feedback(proofs, position_xy=[9., 0.])]
        wrong = copy.deepcopy(proofs)
        wrong[0]['visible'][0]['mask'] = 32
        cases.append(feedback(wrong))
        for message in cases:
            ledger = ObservationLedger()
            self.assertFalse(accept(ledger, message, cache), message)
            self.assertEqual(ledger.points, {})

    def test_partial_points_and_repeated_receipts_do_not_fake_full_coverage(self):
        cache, proofs = evidence(mask=3)
        ledger = ObservationLedger()
        self.assertTrue(accept(ledger, feedback(proofs), cache))
        self.assertEqual(ledger.mask((0, 0), 103.05), 3)
        self.assertFalse(accept(ledger, feedback(proofs, seq=2), cache))
        self.assertFalse(ledger.complete((0, 0), 103.05))

    def test_blocked_route_requires_actual_six_second_stopped_window(self):
        cache = FrameCache(RUN, [UID])
        message = feedback(outcome='NO_ROUTE_COMMIT', phase='REVIEW_PENDING', sample_s=106.1)
        self.assertTrue(accept(ObservationLedger(), message, cache, 106.1))
        self.assertFalse(accept(ObservationLedger(), dict(message, blocked_since_s=104.), cache, 106.1))
        moving = motion(106.1)
        moving[UID]['velocity_xy'] = [.5, 0.]
        self.assertFalse(accept(ObservationLedger(), message, cache, 106.1, samples=moving))

    def test_frame_timeout_is_review_and_does_not_mark_seen(self):
        ledger, grid = ObservationLedger(), CoverageGrid(0., 14., -3.5, 3.5, 7.)
        message = feedback(outcome='NO_FRESH_FRAMES', phase='REVIEW_PENDING', sample_s=103.6)
        self.assertTrue(accept(ledger, message, FrameCache(RUN, [UID]), 103.6))
        ledger.sync_grid(grid, {}, {}, 103.6)
        self.assertEqual(grid.cell((0, 0)).state, STATE_REVIEW)
        self.assertEqual(ledger.points, {})

    def test_planned_visibility_never_updates_actual_seen_timestamp(self):
        allocator = TaskAllocator(CoverageGrid(0., 14., -3.5, 3.5, 7.))
        allocator.vis_enable = allocator.vis_commit = True
        allocator.visible_set[(0, 0)] = [(0, 0), (1, 0)]
        allocator.commit_visible((0, 0), now=100.)
        self.assertEqual(allocator.last_seen, {})
        self.assertEqual(allocator.planned_seen[(1, 0)], 100.)


class SweepTests(unittest.TestCase):
    def test_replanning_does_not_reset_clock_and_observing_is_not_stalled(self):
        sweep = SearchSweep(1, (0, 0), (10., 0.), 100.)
        for t in (100., 102., 104.):
            self.assertIsNone(sweep.blocked_reason(t, (0., 0.), 0., True, 'ONLINE_SPACE_UNKNOWN'))
        self.assertEqual(sweep.blocked_reason(106.01, (0., 0.), 0., True, 'ONLINE_SPACE_UNKNOWN'), 'EXECUTION_GATE_BLOCKED')
        self.assertIsNone(sweep.blocked_reason(106.02, (1., 0.), 0., True, 'MOVING'))
        self.assertTrue(sweep.start_observing(107., (10., 0.), 0., 0.))
        self.assertIsNone(sweep.blocked_reason(120., (10., 0.), 0., False, 'REQUESTED_STOP'))

    def test_three_distinct_new_images_and_bounded_wait(self):
        sweep = SearchSweep(1, (0, 0), (-3.5, 0.), 100.)
        self.assertTrue(sweep.start_observing(100.5, (-3.5, 0.), 0., 0.))
        self.assertFalse(sweep.add_frame(frame(1, 100.4), [], 100.6))
        for seq, stamp in enumerate((101.5, 102., 102.5), 1):
            self.assertTrue(sweep.add_frame(frame(seq, stamp), [], stamp+.1))
            self.assertFalse(sweep.add_frame(frame(seq+10, stamp), [], stamp+.1))
        self.assertFalse(sweep.view_ready(102.9))
        self.assertTrue(sweep.view_ready(103.))
        empty = SearchSweep(1, (0, 0), (-3.5, 0.), 100.)
        empty.start_observing(100.5, (-3.5, 0.), 0., 0.)
        self.assertFalse(empty.view_ready(103.4))
        self.assertTrue(empty.view_ready(103.5))

    def test_preempted_next_view_is_retained_but_old_frames_are_not(self):
        old = SearchSweep(1, (0, 0), (3.5, 0.), 100.)
        old.pending_view, old.phase = (8.5, 0.), 'NEXT_VIEW'
        saved = old.remaining()
        resumed = SearchSweep(2, (0, 0), (3.5, 0.), 110., saved)
        self.assertEqual(resumed.goal, (8.5, 0.))
        self.assertEqual(resumed.frames, [])
        resumed.start_observing(110.1, (8.5, 0.), 0., 0.)
        self.assertFalse(resumed.add_frame(frame(1, 110.2), [], 110.3))

    def test_camera_binds_at_capture_and_stop_cannot_refresh_it(self):
        camera = CameraTask(RUN, UID)
        grant = dict(schema_version=2, run_id=RUN, uav_id=UID, generation=1, seq=1,
                     action='GRANT', expires_s=110., task=task())
        self.assertTrue(camera.receive(grant, 100.))
        self.assertIsNone(camera.search_context(100.1, 99.9))
        self.assertEqual(camera.search_context(100.2, 100.1), dict(generation=1, cell=[0, 0]))
        self.assertTrue(camera.receive(dict(grant, action='STOP', seq=2), 101.))
        self.assertIsNone(camera.search_context(101.1, 101.05))


def extracted(path, class_name, names, scope):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    selected = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), 'exec'), scope)
    return {name: scope[name] for name in names}


class Stamp:
    def __init__(self, t): self.t = t
    def to_sec(self): return self.t
    def __sub__(self, other): return Stamp(self.t-other.t)


def clock_scope(now):
    clock = SimpleNamespace(now=now)
    ros = SimpleNamespace(Time=SimpleNamespace(now=lambda: Stamp(clock.now)),
        loginfo=lambda *a: None, logwarn_throttle=lambda *a: None)
    return clock, dict(math=math, json=json, os=os, rospy=ros,
                       String=lambda **k: SimpleNamespace(**k), MIN_FRAMES=MIN_FRAMES, ARRIVE_TOL=.8,
                       proven_points=proven_points, RADAR_LATENCY_S=.5, RADAR_BRAKE_MPS2=.5)


class ActualCallbackTests(unittest.TestCase):
    def agent(self):
        clock, scope = clock_scope(100.5)
        names = {'_control_search_view', '_finish_search_view', '_publish_search_result', '_measured_motion',
                 '_search_feedback_ack_cb', '_pause_search'}
        scope['motion_evidence'] = motion_evidence
        functions = extracted(SCRIPTS/'swarm_agent.py', 'SwarmAgent', names, scope)
        messages, commands = [], []
        a = SimpleNamespace(uav_id=UID, world_xy=(-3.5, 0.), yaw=0., _pose_sample_s=100.5,
            _authority_lock=threading.RLock(),
            _velocity_sample=(0., 0., 100.5), _search_sweep=SearchSweep(1, (0, 0), (-3.5, 0.), 100.),
            _search_ledger=ObservationLedger(), _search_remaining={}, _search_feedback_seq=0,
            _search_feedback_s=-1., _blocked_feedback_seq=0, _final_stop_reason='REQUESTED_STOP',
            _online_planner=None, _route_gate=SimpleNamespace(record=None, clear=lambda: None),
            _gate=SimpleNamespace(run_id=RUN, can_move=lambda now: True),
            _search_feedback_pub=SimpleNamespace(publish=lambda m: messages.append(json.loads(m.data))),
            _navigation_pub=SimpleNamespace(publish=lambda m: None),
            _send_vel=lambda vx, vy: commands.append((vx, vy)),
            _choose_search_view=lambda sweep, now: (1.5, 0.), _plan_lock=threading.Lock(),
            _plan_ticket=0, _plan_pending=None, _last_online_request_s=0.)
        for name, f in functions.items(): setattr(a, name, MethodType(f, a))
        return a, clock, messages, commands

    def test_actual_agent_changes_view_only_after_current_receipt(self):
        a, clock, messages, commands = self.agent()
        self.assertTrue(a._control_search_view(clock.now))
        for seq, stamp in enumerate((101.5, 102., 102.5), 1):
            a._search_sweep.add_frame(frame(seq, stamp), [], stamp+.1)
        clock.now = 103.05
        a._pose_sample_s = clock.now
        a._velocity_sample = (0., 0., clock.now)
        self.assertTrue(a._control_search_view(clock.now))
        self.assertEqual(a._search_sweep.phase, 'NEXT_VIEW')
        self.assertFalse(messages[-1]['finished'])
        self.assertEqual(a._search_sweep.goal, (-3.5, 0.))
        reply = dict(schema_version=1, run_id=RUN, uav_id=UID, generation=2, seq=messages[-1]['seq'],
                     view_idx=0, accepted=True)
        a._search_feedback_ack_cb(SimpleNamespace(data=json.dumps(reply)))
        self.assertEqual(a._search_sweep.goal, (-3.5, 0.))
        reply['generation'] = 1
        a._search_feedback_ack_cb(SimpleNamespace(data=json.dumps(reply)))
        self.assertEqual(a._search_sweep.goal, (1.5, 0.))
        self.assertEqual(a._search_sweep.view_idx, 1)
        self.assertEqual(a._search_sweep.frames, [])
        self.assertEqual(commands, [(0., 0.), (0., 0.)])

    def test_actual_agent_without_images_finishes_as_review(self):
        a, clock, messages, commands = self.agent()
        a._control_search_view(clock.now)
        clock.now = 103.6
        a._pose_sample_s = clock.now
        a._velocity_sample = (0., 0., clock.now)
        a._control_search_view(clock.now)
        self.assertEqual(messages[-1]['outcome'], 'NO_FRESH_FRAMES')
        self.assertTrue(messages[-1]['finished'])
        self.assertEqual(messages[-1]['phase'], 'REVIEW_PENDING')
        self.assertEqual(a._search_ledger.points, {})

    def test_actual_arrival_does_not_begin_observing_while_pose_drifts_vertically(self):
        a, clock, messages, commands = self.agent()
        a.offset=(0.,0.)
        a._pose_rate_enabled=True
        a._pose_rate=PoseRate()
        for k in range(16):
            a._pose_rate.add((-3.5,0.,2.2+.4*k*.02),clock.now-.3+k*.02,a.offset)
        self.assertTrue(a._control_search_view(clock.now))  # Controls the stop; does not enter OBSERVE.
        self.assertEqual(a._search_sweep.phase,'GO_TO_VIEW')
        self.assertIsNone(a._search_sweep.window_s)
        self.assertEqual(messages,[])
        self.assertEqual(commands,[(0.,0.)])

    def test_actual_agent_checks_three_views_and_never_waits_for_fourth(self):
        a, clock, messages, commands = self.agent()
        choices = iter(((1.5, 0.), (-3.5, 5.)))
        a._choose_search_view = lambda sweep, now: next(choices)
        seq = 0
        for view in range(3):
            a.world_xy = a._search_sweep.goal
            start = 100.5+view*5.
            clock.now = start
            a._pose_sample_s, a._velocity_sample = start, (0., 0., start)
            self.assertTrue(a._control_search_view(start))
            for offset in (1., 1.5, 2.):
                seq += 1
                f = frame(seq, start+offset)
                f['camera_xyz'][:2] = a.world_xy
                self.assertTrue(a._search_sweep.add_frame(f, [], start+offset+.1))
            clock.now = start+2.6
            a._pose_sample_s, a._velocity_sample = clock.now, (0., 0., clock.now)
            self.assertTrue(a._control_search_view(clock.now))
            result = messages[-1]
            self.assertEqual(result['view_idx'], view)
            if view < 2:
                self.assertFalse(result['finished'])
                reply = dict(schema_version=1, run_id=RUN, uav_id=UID, generation=1,
                             seq=result['seq'], view_idx=view, accepted=True)
                a._search_feedback_ack_cb(SimpleNamespace(data=json.dumps(reply)))
            else:
                self.assertTrue(result['finished'])
                self.assertTrue(a._search_sweep.finished)
                self.assertEqual(a._search_sweep.phase, 'REVIEW_PENDING')
        self.assertEqual(len(messages), 3)
        self.assertTrue(all(cmd == (0., 0.) for cmd in commands))

    def test_lost_feedback_receipt_has_bounded_existing_stop_request(self):
        a, clock, messages, commands = self.agent()
        navigation = []
        a._navigation_pub.publish = lambda m: navigation.append(json.loads(m.data))
        a._control_search_view(clock.now)
        for seq, stamp in enumerate((101.5, 102., 102.5), 1):
            a._search_sweep.add_frame(frame(seq, stamp), [], stamp+.1)
        clock.now = 103.05
        a._pose_sample_s, a._velocity_sample = clock.now, (0., 0., clock.now)
        a._control_search_view(clock.now)
        clock.now = 109.1
        a._pose_sample_s, a._velocity_sample = clock.now, (0., 0., clock.now)
        a._control_search_view(clock.now)
        self.assertEqual(navigation[-1]['reason'], 'NO_REACHABLE_PROGRESS')
        self.assertGreaterEqual(navigation[-1]['sample_s']-navigation[-1]['blocked_since_s'], 6.)
        self.assertEqual(a._search_sweep.goal, (-3.5, 0.))

    def test_useful_neighbour_view_does_not_force_three_points_in_every_cell(self):
        a, clock, messages, commands = self.agent()
        a._choose_search_view = lambda *args: self.fail('unnecessary extra view')
        a._control_search_view(clock.now)
        for seq, stamp in enumerate((101.5, 102., 102.5), 1):
            a._search_sweep.add_frame(frame(seq, stamp), [dict(cell=[1, 0], mask=31)], stamp+.1)
        clock.now = 103.05
        a._pose_sample_s, a._velocity_sample = clock.now, (0., 0., clock.now)
        a._control_search_view(clock.now)
        self.assertTrue(messages[-1]['finished'])
        self.assertEqual(messages[-1]['phase'], 'NEXT_VIEW')
        self.assertFalse(a._search_ledger.complete((0, 0), clock.now))
        self.assertTrue(a._search_ledger.complete((1, 0), clock.now))

    def test_view_candidate_uses_current_scan_not_only_cached_free_plan(self):
        clock, scope = clock_scope(103.1)
        choose = extracted(SCRIPTS/'swarm_agent.py', 'SwarmAgent', {'_choose_search_view'}, scope)['_choose_search_view']
        observed = ObservedMap(80, 80, .25, (-10., -10.))
        for i in range(len(observed.cells)):
            p = (-10.+(i % 80+.5)*.25, -10.+(i//80+.5)*.25)
            if math.hypot(*p) <= 2.:
                observed.cells[i], observed.observed_s[i] = 0, 103.
        observed.last_scan_s = 103.
        a = SimpleNamespace(_online_safe_grid=GridMap(80, 80, .25, (-10., -10.), [0]*6400, 'map'),
            _online_safe_s=103., _online_safe_epoch=100., _online_map_epoch_s=100.,
            _online_planner=OnlinePlanner((0., 0.)), _online_map=observed,
            _online_map_lock=threading.RLock(), world_xy=(0., 0.),
            cov_grid=CoverageGrid(-10., 10., -10., 10., 7.))
        sweep = SearchSweep(1, (1, 1), (0., 0.), 100.)
        self.assertIsNone(choose(a, sweep, clock.now))
        observed.cells, observed.observed_s = [0]*6400, [103.]*6400
        observed.version += 1
        self.assertIsNotNone(choose(a, sweep, clock.now))

    def test_actual_manager_result_stops_without_freeing_occupancy(self):
        cache, proofs = evidence()
        clock, scope = clock_scope(103.05)
        scope.update(apply_authority_release=apply_authority_release, apply_intent_cancellation=apply_intent_cancellation)
        names = {'_search_feedback_cb', '_ack_search_feedback', '_emit_authority'}
        functions = extracted(SCRIPTS/'swarm_manager.py', 'SwarmManager', names, scope)
        core, grid = TaskAuthority(RUN, [UID]), CoverageGrid(0., 14., -3.5, 3.5, 7.)
        core.offer(UID, task(), 100.)
        core.active[UID]['expires_s'] = 110.
        c = grid.cell((0, 0))
        c.state, c.owner = STATE_ASSIGNED, UID
        grants, replies = [], []
        m = SimpleNamespace(_authority=core, _authority_lock=threading.RLock(), uav_ids=[UID], grid=grid,
            _route_motion=SimpleNamespace(samples=motion()), _search_frames=cache,
            _search_ledger=ObservationLedger(), _search_rejections=RejectedTasks(), _search_enabled=True,
            _active_leases={UID: (0, 0)}, _authority_event_cursor=0, _authority_log=io.StringIO(),
            _authority_pub=SimpleNamespace(publish=lambda msg: grants.append(json.loads(msg.data))),
            _search_ack_pub=SimpleNamespace(publish=lambda msg: replies.append(json.loads(msg.data))))
        for name, f in functions.items(): setattr(m, name, MethodType(f, m))
        message = feedback(proofs)
        m._search_feedback_cb(SimpleNamespace(data=json.dumps(message)))
        self.assertEqual(grants[-1]['action'], 'STOP')
        self.assertEqual((c.state, c.owner), (STATE_ASSIGNED, UID))
        self.assertIn(('search', 0, 0), core.locks)
        self.assertEqual(m._active_leases, {})
        self.assertTrue(replies[-1]['accepted'])
        old_events = len(core.events)
        m._search_feedback_cb(SimpleNamespace(data=json.dumps(dict(message, seq=2))))
        self.assertEqual(len(core.events), old_events)
        self.assertEqual(replies[-1]['seq'], 2)

    def test_actual_manager_arrival_no_longer_marks_twenty_meter_disk(self):
        clock, scope = clock_scope(103.05)
        scope.update(COVER_ARRIVE_M=4., VIS_RADIUS=20.)
        f = extracted(SCRIPTS/'swarm_manager.py', 'SwarmManager', {'_renew_leases'}, scope)['_renew_leases']
        renewed, covered = [], []
        m = SimpleNamespace(uav_ids=[UID], _search_enabled=True, last_report={UID: Stamp(103.)},
            _active_leases={UID: (0, 0)}, _cell_waypoint={(0, 0): (-3.5, 0.)},
            status={UID: SimpleNamespace(x=-3.5, y=0.)},
            grid=SimpleNamespace(is_covered=lambda *a: covered.append(a)),
            lease=SimpleNamespace(renew=lambda *a: renewed.append(a)))
        f(m)
        self.assertEqual(covered, [])
        self.assertEqual(renewed[0][:2], (UID, (0, 0)))
        self.assertEqual(m._active_leases, {UID: (0, 0)})

    def test_actual_perception_empty_boxes_publish_completed_frame(self):
        tree = ast.parse((ROOT/'perception/perception_real.py').read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        block = next(n for n in ast.walk(main) if isinstance(n, ast.If)
                     and isinstance(n.test, ast.BoolOp) and isinstance(n.test.values[0], ast.Name)
                     and n.test.values[0].id == 'new_frame' and len(n.test.values) == 3)
        published = []
        context = dict(generation=1, cell=[0, 0])
        _, scope = clock_scope(103.)
        scope.update(new_frame=True, processed_camera=SimpleNamespace(publish=published.append),
            search_context=context, processed_frame_seq=0, arb_lock=threading.Lock(),
            camera_task=SimpleNamespace(search_context=lambda *a: context), frame_stamp=102.8,
            UAV=UID, img=SimpleNamespace(shape=(360, 640, 3)), FX=300., FY=300., CX=320., CY=180.,
            px=-3.5, py=0., pz=2.2, R=SimpleNamespace(reshape=lambda *a: frame()['camera_rotation']),
            valid_frame=valid_frame, os=SimpleNamespace(environ={'ROBOCUP_RUN_ID': RUN}))
        exec(compile(ast.Module(body=[block], type_ignores=[]), 'perception_real.py', 'exec'), scope)
        self.assertEqual(len(published), 1)
        result = json.loads(published[0].data)
        self.assertTrue(valid_frame(result, 103.))
        scope['new_frame'] = False
        exec(compile(ast.Module(body=[block], type_ignores=[]), 'perception_real.py', 'exec'), scope)
        self.assertEqual(len(published), 1)


if __name__ == '__main__':
    unittest.main()
