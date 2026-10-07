import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/robocup_swarm/scripts'))
sys.path.insert(0,str(ROOT/'src/robocup_navigation/src'))
from bounded_escape import choose_exit, RestEvidence
from robocup_navigation.astar import GridMap
from online_radar_planner import OnlinePlanner
from radar_observed_map import ObservedMap


class BoundedEscapeTests(unittest.TestCase):
    def grids(self):
        normal = bytearray([0]*100)
        normal[55] = 1
        return (GridMap(10,10,.25,(0,0),bytes(normal),'map'),
                GridMap(10,10,.25,(0,0),bytes(100),'map'))

    def test_real_start_not_relocated_and_exit_joins_normal_space(self):
        normal,recovery = self.grids()
        start = (1.38,1.38)
        path = choose_exit(normal,recovery,start,[(1.8,1.8)])
        self.assertIsNotNone(path)
        self.assertEqual(path[0],start)
        self.assertTrue(normal.is_free(normal.world_to_cell(path[-1])))
        self.assertLessEqual(sum(__import__('math').dist(a,b) for a,b in zip(path,path[1:])),2.)

    def test_unknown_or_peer_blocked_recovery_start_stays_blocked(self):
        normal,recovery = self.grids()
        self.assertIsNone(choose_exit(normal,normal,(1.38,1.38),[(1.8,1.8)]))

    def test_no_connector_or_no_away_direction_returns_none(self):
        normal,recovery = self.grids()
        closed = GridMap(10,10,.25,(0,0),bytes([1]*100),'map')
        self.assertIsNone(choose_exit(closed,recovery,(1.38,1.38),[(1.8,1.8)]))
        self.assertIsNone(choose_exit(normal,recovery,(1.375,1.375),
            [(1.0,1.375),(1.75,1.375),(1.375,1.0),(1.375,1.75)]))

    def test_metadata_and_inputs_cannot_create_an_exit(self):
        normal,recovery = self.grids()
        different = GridMap(10,10,.5,(0,0),bytes(100),'map')
        self.assertIsNone(choose_exit(normal,different,(1.38,1.38),[(1.8,1.8)]))
        for hits in ([],[(float('nan'),1.)]):
            self.assertIsNone(choose_exit(normal,recovery,(1.38,1.38),hits))
        self.assertIsNone(choose_exit(normal,recovery,(1.38,1.38),[(1.8,1.8)],max_distance=3.))


class RestEvidenceTests(unittest.TestCase):
    def evidence(self, stamp, raw=(0.,0.,0.), pose=(0.,0.,0.), pose_stamp=None):
        return dict(velocity_local_sample=[*raw,stamp],
                    pose_rate_sample=[*pose,stamp if pose_stamp is None else pose_stamp])

    def test_both_sources_need_a_full_measured_second(self):
        rest = RestEvidence()
        for stamp in (10.,10.25,10.5,10.75):
            self.assertFalse(rest.update(self.evidence(stamp),stamp,('run',2)))
        self.assertTrue(rest.update(self.evidence(11.),11.,('run',2)))
        # Current control time cannot stand in for the second source's stamp.
        rest = RestEvidence()
        for stamp in (10.,10.25,10.5,10.75,11.):
            pose_stamp = 10.+.75*(stamp-10.)
            self.assertFalse(rest.update(self.evidence(stamp,pose_stamp=pose_stamp),
                                         stamp,('run',2)))
        self.assertTrue(rest.update(self.evidence(11.5,pose_stamp=11.25),
                                    11.5,('run',2)))

    def test_duplicate_frames_do_not_accumulate_rest(self):
        rest = RestEvidence()
        for now in (10.,10.1,10.2,10.5):
            self.assertFalse(rest.update(self.evidence(10.),now,2))
        self.assertFalse(rest.update(self.evidence(10.),10.501,2))

    def test_vertical_motion_and_missing_source_reset_the_interval(self):
        for bad in (self.evidence(10.5,raw=(0.,0.,.16)),
                    self.evidence(10.5,pose=(0.,0.,-.16)),
                    dict(velocity_local_sample=[0.,0.,0.,10.5],pose_rate_sample=None),
                    self.evidence(10.5,raw=(float('nan'),0.,0.)), None):
            rest = RestEvidence()
            rest.update(self.evidence(10.),10.,2)
            rest.update(self.evidence(10.25),10.25,2)
            self.assertFalse(rest.update(bad,10.5,2))
            self.assertFalse(rest.update(self.evidence(10.75),10.75,2))

    def test_authority_gap_and_backwards_time_restart_rest(self):
        for stamp,now,authority in ((10.5,10.5,3),(11.,11.,2),(9.,9.,2)):
            rest = RestEvidence()
            rest.update(self.evidence(10.),10.,2)
            rest.update(self.evidence(10.25),10.25,2)
            self.assertFalse(rest.update(self.evidence(stamp),now,authority))
            self.assertFalse(rest.update(self.evidence(stamp+.25),now+.25,authority))


class ExitSnapshotTests(unittest.TestCase):
    def fixture(self):
        observed = ObservedMap(40,40,.25,(0.,0.))
        observed.cells = [0]*1600
        observed.observed_s = [10.]*1600
        observed.last_scan_s = 10.
        observed.version = 3
        observed.cells[20*40+25] = 100
        return observed,OnlinePlanner((5.125,5.125)),(5.125,5.125)

    def test_same_snapshot_and_peer_filter_apply_to_both_radii(self):
        observed,planner,start = self.fixture()
        before = (list(observed.cells),list(observed.observed_s))
        filtered = []
        def peer_filter(grid):
            filtered.append(grid)
            return grid
        result = planner.exit_candidate(observed,start,10.,1,peer_filter)
        self.assertIsNotNone(result)
        self.assertEqual(len(filtered),2)
        self.assertFalse(filtered[0].is_free(filtered[0].world_to_cell(start)))
        self.assertTrue(filtered[1].is_free(filtered[1].world_to_cell(start)))
        self.assertEqual(result['points'][0],start)
        self.assertEqual(result['blocking_hits'],((6.375,5.125),))
        self.assertEqual((result['scan_s'],result['version'],result['epoch']),(10.,3,1))
        self.assertEqual(planner.radius,1.2)
        self.assertEqual(before,(observed.cells,observed.observed_s))

    def test_peer_blocked_start_and_stale_scan_cannot_escape(self):
        observed,planner,start = self.fixture()
        def peer_filter(grid):
            cells = bytearray(grid.cells)
            cell = grid.world_to_cell(start)
            cells[cell[1]*grid.width+cell[0]] = 1
            return GridMap(grid.width,grid.height,grid.resolution,grid.origin,bytes(cells),'map')
        self.assertIsNone(planner.exit_candidate(observed,start,10.,1,peer_filter))
        self.assertIsNone(planner.exit_candidate(observed,start,10.501,1,lambda g:g))
        self.assertIsNone(planner.exit_candidate(observed,start,10.,1,None))

    def test_unknown_only_and_hit_inside_recovery_radius_stay_blocked(self):
        observed,planner,start = self.fixture()
        observed.cells = [-1]*1600
        observed.observed_s = [None]*1600
        self.assertIsNone(planner.exit_candidate(observed,start,10.,1,lambda g:g))
        observed,planner,start = self.fixture()
        observed.cells[20*40+25] = 0
        observed.cells[20*40+23] = 100
        self.assertIsNone(planner.exit_candidate(observed,start,10.,1,lambda g:g))


if __name__ == '__main__':
    unittest.main()
