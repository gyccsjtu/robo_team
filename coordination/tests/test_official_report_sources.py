"""Official coordinates and readiness must refer to the same fresh camera."""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS
import threading
sys.path.insert(0, str(Path(__file__).parents[1]/'src/robocup_swarm/scripts'))
from official_report_sources import OfficialReportSources
from report_readiness import ReportReadiness
from yolo_target_bridge import TargetBridgeCore, YoloTargetBridge
from test_approach_reporting import image


class SourceTests(unittest.TestCase):
    def setUp(self):
        self.readiness = ReportReadiness()
        self.sources = OfficialReportSources(TargetBridgeCore, self.readiness)

    def feed(self, stamp, uid='uav_1', distance=10., tag='green', **extra):
        observation = image(stamp, int(stamp*100), uid=uid, distance=distance, tag=tag, **extra)
        self.readiness.observe(observation, stamp, True)
        return self.sources.observe(observation, True)

    def ready(self, uid='uav_1', tag='green', start=10.):
        for t in (start, start+.3, start+.6):
            self.feed(t, uid, tag=tag)

    def test_new_far_source_cannot_poison_near_source_coordinates(self):
        self.ready()
        self.feed(10.8, 'uav_2', distance=15.)
        self.assertFalse(self.readiness.allowed('green', 10.8, 10.8))
        events = self.sources.tick(10.8)
        self.assertEqual(len(events), 1)
        uid, event, track = events[0]
        self.assertEqual(uid, 'uav_1')
        self.assertAlmostEqual(event['x'], 10.)
        self.assertAlmostEqual(track.t_obs, 10.6)

    def test_duplicate_originals_do_not_activate(self):
        self.feed(10.)
        for _ in range(5):
            self.assertFalse(self.feed(10.))
        self.feed(10.6)
        self.assertFalse(self.sources.tick(10.6))

    def test_missing_distance_and_candidate_schema_never_upload(self):
        for version in (2, 4, 5):
            for t in (10., 10.3, 10.6):
                self.feed(t, version=version)
        self.assertFalse(self.sources.cores)
        self.assertFalse(self.sources.tick(10.6))

    def test_stale_or_invalid_original_stops_reporting(self):
        self.ready()
        self.assertTrue(self.sources.tick(10.6))
        self.assertFalse(self.sources.tick(11.601))
        m = image(10.9, 109)
        self.readiness.observe(m, 10.9, False)
        self.sources.observe(m, False)
        self.assertFalse(self.sources.tick(10.9))

    def test_source_rejection_cannot_borrow_newer_readiness(self):
        self.ready()
        m = image(10.9, 109, distance=21.)
        # Global fusion can accept an input that this source's own track rejects.
        self.assertTrue(self.readiness.observe(m, 10.9, True))
        self.assertFalse(self.sources.observe(m, True))
        self.assertFalse(self.sources.tick(10.9))

    def test_sticky_source_then_handoff_to_another_ready_source(self):
        self.ready()
        self.assertEqual(self.sources.tick(10.6)[0][0], 'uav_1')
        self.ready('uav_2', start=10.7)
        self.assertEqual(self.sources.tick(11.3)[0][0], 'uav_1')
        self.assertEqual(self.sources.tick(11.7)[0][0], 'uav_2')

    def test_white_requires_person_proof_and_eliminated_slot_never_revives(self):
        for t in (10., 10.3, 10.6):
            self.feed(t, tag='white', person=False)
        self.assertFalse(self.sources.tick(10.6))
        self.ready(tag='red1')
        self.sources.eliminate('red1')
        self.assertFalse(self.feed(10.9, tag='red1'))
        self.assertFalse(self.sources.tick(10.9))

    def test_multiple_colors_can_report_concurrently(self):
        self.ready(tag='green')
        self.ready(tag='blue', start=10.7)
        self.feed(11.3, tag='green')
        self.assertEqual({e[1]['tag'] for e in self.sources.tick(11.3)}, {'green', 'blue'})

    def test_bridge_internal_emit_cannot_bypass_selected_source(self):
        b = YoloTargetBridge.__new__(YoloTargetBridge)
        b.core = TargetBridgeCore()
        b._official_sources = self.sources
        b._TargetState = lambda: NS(header=NS())
        b._rospy = NS(Time=NS(from_sec=lambda t: t))
        internal = []
        b.pub = NS(publish=internal.append)
        b._emit_actor = lambda *args: self.fail('internal route uploaded coordinates')
        b._emit(dict(tag='green', tid='t0', x=10., y=0., vx=0., vy=0., state=0, eliminated=False))
        self.assertEqual(len(internal), 1)

    def test_actual_timer_reports_selected_source_once_and_keeps_internal_state(self):
        self.ready()
        self.feed(10.8, 'uav_2', distance=15.)
        b = YoloTargetBridge.__new__(YoloTargetBridge)
        b.core = TargetBridgeCore()
        for t in (10., 10.3, 10.6):
            b.core.report(t, 'green', 10., 0., .9)
        b.core.report(10.8, 'green', 11., 0., .9)
        b._official_sources = self.sources
        b._approach_reporting = True
        b._report_readiness = self.readiness
        b._lock = threading.RLock()
        b._emit_stats = {}; b._skip_trace_t = {}
        b._now = lambda: 10.8
        b._heartbeat = lambda: None
        b._TargetState = lambda: NS(header=NS())
        b._ActorInfo = NS
        b._rospy = NS(Time=NS(from_sec=lambda t: t))
        internal, official = [], []
        b.pub = NS(publish=internal.append)
        b._actor_pubs = {'green': NS(publish=official.append)}
        b._tick(None)
        self.assertEqual(len(internal), 1)
        self.assertEqual(len(official), 1)
        self.assertEqual(official[0].x, 10.)
        self.assertEqual(b.core.tracks['green'].t_obs, 10.8)
        b._now = lambda: 11.601
        b._tick(None)
        self.assertEqual(len(official), 1)


if __name__ == '__main__':
    unittest.main()
