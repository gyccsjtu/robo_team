"""Laser evidence ordering, disconnected peers and the final pending update."""
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'coordination/src/robocup_swarm/scripts'))
from map_evidence import MapEvidenceFusion
from slam_grid import SlamGrid
import swarm_agent


class EvidenceTests(unittest.TestCase):
    def test_lost_source_does_not_clear_wall_but_new_rays_can_clear_dynamic_hit(self):
        f = MapEvidenceFusion(2)
        self.assertTrue(f.apply('a', 'boot1', 1, 10., [1, 1], [[0,10.,1,1],[1,10.,1,1]], 10.))
        self.assertEqual(f.active_sources(20.), set())
        self.assertEqual(list(f.cells), [1,1])  # no timeout erasure of buildings
        f.apply('b', 'boot1', 1, 20., [0,0], [[0,20.,0,2]], 20.)
        self.assertEqual(list(f.cells), [1,1])  # too little clear evidence
        f.apply('b', 'boot1', 2, 20.2, [0,0], [[0,20.2,0,3]], 20.2)
        self.assertEqual(list(f.cells), [0,1])
        # a reconnecting publisher repeats its old occupied snapshot.
        f.apply('a', 'boot1', 2, 20.4, [1,1], [[0,10.,1,1],[1,10.,1,1]], 20.4)
        self.assertEqual(list(f.cells), [0,1])
        f.apply('a', 'boot1', 3, 20.6, [1,1], [[0,20.6,1,1],[1,10.,1,1]], 20.6)
        self.assertEqual(list(f.cells), [1,1])  # a new hit restores the obstacle

    def test_restarts_duplicates_future_and_bad_frames_do_not_mutate_map(self):
        f = MapEvidenceFusion(1)
        f.apply('a','boot1',1,10.,[1],[[0,10.,1,1]],10.)
        self.assertFalse(f.apply('a','boot1',1,10.,[0],[[0,10.,0,3]],10.))
        self.assertFalse(f.apply('b','boot1',1,11.,[0],[[0,11.,0,3]],10.))
        self.assertFalse(f.apply('b','boot1',1,10.,[0],[[4,10.,0,3]],10.))
        self.assertEqual(list(f.cells), [1])
        self.assertTrue(f.apply('a','boot2',1,11.,[0],[[0,11.,0,3]],11.))
        self.assertFalse(f.apply('a','boot1',2,12.,[1],[[0,12.,1,1]],12.))
        self.assertEqual(list(f.cells), [0])

    def test_three_distinct_clear_scans_required_locally(self):
        g = SlamGrid(0.,0.,20.,20.,.5,1.)
        scan = NS(ranges=[5.], angle_min=0., angle_increment=.1, range_min=.1, range_max=12.)
        g.mark_scan((2.,10.),0.,scan,12.,sample_s=10.)
        c = g.world_to_cell((7.,10.))
        scan.ranges = [float('inf')]
        for at in (10.2,10.2,10.4):
            g.mark_scan((2.,10.),0.,scan,12.,sample_s=at)
            self.assertFalse(g.is_free_raw(c))
        g.mark_scan((2.,10.),0.,scan,12.,sample_s=10.6)
        self.assertTrue(g.is_free_raw(c))
        self.assertTrue(any(row[0] == c[1]*g.width+c[0] and row[2:] == [0,3]
                            for row in g.evidence_snapshot(10.6)))

    @patch.object(swarm_agent.rospy.Time, 'now', return_value=swarm_agent.rospy.Time(103))
    def test_last_dirty_frame_publishes_after_throttle_without_new_changes(self, clock):
        a = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        a.uav_id = 'u0'
        a.local_xy, a.offset = (2.,10.), (0.,0.)
        a.slam = SlamGrid(0.,0.,20.,20.,.5,0.)
        a.slam.mark_scan = Mock(side_effect=[1,0,0])
        a._scan = NS(ranges=[5.])
        a._slam_last_t = a._slam_replan_t = 0.
        a._slam_pub_t, a._scan_t = 100., 100.3
        a._slam_dirty = False
        a.pitch = a.roll = a.yaw = 0.
        a.path = []
        a.occ_pub = Mock()
        a._slam_update()
        self.assertTrue(a._slam_dirty)
        a._scan_t = 103.
        a.occ_pub.publish.side_effect = RuntimeError('transport failed')
        a._slam_update()
        self.assertTrue(a._slam_dirty)
        self.assertEqual(a._slam_pub_t, 100.)
        a.occ_pub.publish.side_effect = None
        a._scan_t = 103.3
        a._slam_update()
        self.assertFalse(a._slam_dirty)
        msg = json.loads(a.occ_pub.publish.call_args.args[0].data)
        self.assertEqual((msg['schema'],msg['sequence']), (2,1))


if __name__ == '__main__':
    unittest.main()
