"""Home-zone patrol ownership without stealing person tasks or stalling startup."""
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import test_east_split_patrol as fixture
import boundary_patrol as bp
import swarm_manager as sm


class SearchPatrolOwnerTests(unittest.TestCase):
    use_default_flight_bounds = True
    refresh = fixture.DefaultEastSplitPatrolTests.refresh

    def setUp(self):
        fixture.DefaultEastSplitPatrolTests.setUp(self)
        self.north, self.south = self.m._regional_patrols
        for patrol in self.m._regional_patrols:
            patrol._prefer_home_owner = True
        self.west, self.southeast, self.northeast = (
            'typhoon_h480_0', 'typhoon_h480_1', 'typhoon_h480_5')

    def test_macro_owners_use_east_home_zones_despite_nearer_west_aircraft(self):
        self.m.status[self.west].x, self.m.status[self.west].y = 98., 12.
        self.m._allocate()
        self.assertEqual(self.north.uid, self.northeast)
        self.assertEqual(self.south.uid, self.southeast)
        self.assertEqual(len(self.m._segments.tasks), 6)
        keys = [k for task in self.m._segments.tasks.values() for k in task.cells]
        self.assertEqual(len(keys), len(set(keys)))
        self.assertEqual(self.m._segments.tasks[self.west].purpose, 'coverage')

    def test_zone_geography_not_fixed_uav_id_selects_owner(self):
        zones = self.m.grid.zone_cells
        zones[0], zones[5] = zones[5], zones[0]
        proposal = self.north.choose(self.m, self.m.uav_ids, self.now)
        self.assertEqual(proposal[0], self.west)

    def test_2475_replay_not_ready_false_waits_then_ready_owns_lane(self):
        # Original first east allocation at2475.028: both matching machines
        # were airborne and climbing, but status.connected included takeoff_done.
        self.refresh(2475.028)
        for uid,z in ((self.southeast, 1.125155), (self.northeast, 1.437572)):
            self.m.status[uid].connected = False
            self.m.status[uid].z = z
        self.m._processed_frame_times = {uid:self.now-.1 for uid in self.m.uav_ids}
        self.assertIsNone(self.north.choose(self.m, [self.west], self.now))
        self.assertIsNone(self.north.uid)
        self.assertFalse(self.m._segments.tasks)
        self.refresh(2477.508)
        self.m._processed_frame_times = {uid:self.now-.1 for uid in self.m.uav_ids}
        self.m.status[self.northeast].connected = True
        self.m.status[self.northeast].z = 2.6
        proposal = self.north.choose(self.m, [self.west,self.northeast], self.now)
        self.assertEqual(proposal[0], self.northeast)

    def test_missing_first_team_status_uses_same_finite_wait(self):
        self.m.status.pop(self.northeast)
        self.m.last_report.pop(self.northeast)
        self.assertIsNone(self.north.choose(self.m, [self.west], self.now))
        origin = self.north._owner_preference_since
        self.refresh(self.now+6.1)
        proposal = self.north.choose(self.m, [self.west], self.now)
        self.assertEqual(proposal[0], self.west)
        self.assertEqual(self.north._owner_preference_since, origin)

    def test_not_ready_wait_expires_without_renewing_or_false_coverage(self):
        self.m.status[self.northeast].connected = False
        self.assertIsNone(self.north.choose(self.m, [self.west], self.now))
        origin = self.north._owner_preference_since
        self.refresh(self.now+5.9)
        self.assertIsNone(self.north.choose(self.m, [self.west], self.now))
        self.refresh(origin+6.01)
        self.assertEqual(self.north.choose(self.m,[self.west],self.now)[0],self.west)
        self.assertEqual(self.north._owner_preference_since, origin)
        self.assertTrue(all(c.covered_t == 0. for c in self.m.grid.cells.values()))

    def test_person_or_backup_busy_is_immediate_fallback_without_stealing(self):
        for field in ('_tracking','_backup'):
            with self.subTest(field=field):
                setattr(self.m,field,{'real_person':self.northeast})
                proposal=self.north.choose(self.m,[self.west],self.now)
                self.assertEqual(proposal[0],self.west)
                self.assertEqual(getattr(self.m,field),{'real_person':self.northeast})
                setattr(self.m,field,{})

    def test_stale_report_or_camera_does_not_wait_for_unavailable_machine(self):
        for reason in ('report','camera','nonfinite'):
            with self.subTest(reason=reason):
                self.refresh(100.)
                self.m.status[self.northeast].connected=False
                self.m.status[self.northeast].z=1.4
                self.m._processed_frame_times={uid:self.now for uid in self.m.uav_ids}
                if reason=='report':self.m.last_report[self.northeast]=sm.rospy.Time(95.)
                elif reason=='camera':self.m._processed_frame_times[self.northeast]=95.
                else:self.m.status[self.northeast].z=float('nan')
                self.assertEqual(self.north.choose(self.m,[self.west],self.now)[0],self.west)

    def test_previously_ready_false_is_fault_not_new_startup_wait(self):
        preferred,pending=self.north._first_owner_candidates(self.m,[self.northeast],self.now)
        self.assertEqual(preferred,[self.northeast]);self.assertFalse(pending)
        self.m.status[self.northeast].connected=False
        self.assertEqual(self.north.choose(self.m,[self.west],self.now)[0],self.west)

    def test_unreachable_preferred_has_safe_fallback_and_total_probe_bound(self):
        original=bp.plan;probes=[]
        near=self.m.status[self.west];far=self.m.status[self.northeast]
        near.x,near.y=98.,12.;far.x,far.y=3.,3.
        def reachable(grid,start,goal,**kw):
            probes.append(start)
            return NS(success=False) if start[0]<10 else original(grid,start,goal,**kw)
        with patch.object(bp,'plan',side_effect=reachable):
            proposal=self.north.choose(self.m,[self.west,self.northeast],self.now)
        self.assertEqual(proposal[0],self.west)
        self.assertLessEqual(len(probes),24)

    def test_started_non_home_owner_stays_stable_and_person_preempts(self):
        cells=[(22,9)];looks=[(107.5,6.5)];points=[(98.55,6.5)]
        task=self.m._start_search_segment(self.west,cells,points,self.now,
            purpose='south_patrol',publish=False,look_points=looks)
        self.north.started(task,1,self.now)
        self.assertIsNone(self.north.choose(self.m,self.m.uav_ids,self.now))
        self.assertEqual(self.north.uid,self.west)
        self.m._tracking['person']=self.west
        self.m._free_search_lease(self.west)
        self.assertIsNone(self.north.uid)
        self.assertEqual(self.m._tracking['person'],self.west)

    def test_default_switch_off_preserves_nearest_old_policy(self):
        with patch.dict('os.environ',{'SEARCH_DECONFLICTED_ALLOCATION':'0'}):
            old=bp.SouthPatrol(self.north.bounds,southeast=False,northeast=False,
                               east=True,east_half='north')
        self.m.status[self.west].x,self.m.status[self.west].y=98.,12.
        proposal=old.choose(self.m,[self.west,self.northeast],self.now)
        self.assertEqual(proposal[0],self.west)
        self.assertIsNone(old._owner_preference_since)


if __name__=='__main__':
    unittest.main()
