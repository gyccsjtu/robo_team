from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).parents[1]/'src/robocup_swarm/scripts'))
sys.path.insert(0,str(Path(__file__).parents[1]/'src/robocup_navigation/src'))
sys.path.insert(0,str(Path(__file__).parents[2]/'perception'))
from online_radar_planner import OnlinePlanner
from radar_observed_map import ObservedMap
from navigation_feedback import accept,refresh_due
from fresh_person import FreshPerson


class FastFixTests(unittest.TestCase):
    def observed(self):
        obs = ObservedMap(40,40,.5,(-10.,-10.))
        for i in range(len(obs.cells)):
            x,y = -10+(i%40+.5)*.5,-10+(i//40+.5)*.5
            if -5 <= x <= 1 and -3 <= y <= 3:
                obs.cells[i],obs.observed_s[i] = 0,10.
        obs.last_scan_s,obs.version = 10.,1
        return obs

    def test_current_scan_can_execute_without_a_completed_full_plan(self):
        obs = self.observed(); planner = OnlinePlanner((-2.,0.))
        self.assertTrue(planner.local_command_clear(obs,(-2.,0.),(.2,0.),10.,'epoch'))
        self.assertFalse(planner.local_command_clear(obs,(-2.,0.),(.2,0.),10.51,'epoch'))
        obstacle=obs.index(-1.,0.)
        obs.cells[obstacle],obs.observed_s[obstacle] = 100,10.1
        obs.version+=1; obs.last_scan_s=10.1
        self.assertFalse(planner.local_command_clear(obs,(-2.,0.),(.2,0.),10.1,'epoch'))

    def test_unknown_or_expired_corridor_is_never_filled_by_local_cache(self):
        obs=self.observed();planner=OnlinePlanner((-2.,0.))
        self.assertFalse(planner.local_command_clear(obs,(-2.,0.),(2.,0.),10.,'epoch'))
        obs.last_scan_s=13.1;obs.version+=1
        self.assertFalse(planner.local_command_clear(obs,(-2.,0.),(.2,0.),13.1,'epoch'))

    def test_frontier_may_move_away_and_does_not_repeat_same_boundary(self):
        obs=self.observed();start=(-.75,.25)
        planner=OnlinePlanner(start,seed_radius=0)
        grid=planner.grid(obs,start,10.,'epoch')
        self.assertFalse(planner.route(grid,start,(9.,.25))['ok'])
        first=planner.route(grid,start,(9.,.25),obs,10.)
        self.assertTrue(first['ok'])
        self.assertTrue(all(grid.is_free(grid.world_to_cell(p)) for p in first['points']))
        second=planner.route(grid,start,(9.,.25),obs,10.1)
        self.assertTrue(second['ok'])
        self.assertNotEqual(first['points'][-1],second['points'][-1])

    def test_person_needs_distinct_verified_frames_and_loses_support_when_missed(self):
        proof=FreshPerson()
        for _ in range(3):proof.observe(10.,True)
        self.assertFalse(proof.allowed('green',0,10.))
        proof.observe(10.2,True);proof.observe(10.4,True)
        for color in ('green','white'):self.assertTrue(proof.allowed(color,0,10.5))
        self.assertFalse(proof.allowed('blue',0,10.5))
        self.assertFalse(proof.allowed('white',1,10.5))
        self.assertFalse(proof.allowed('white',0,11.41))
        proof.observe(10.6,False)
        self.assertFalse(proof.allowed('white',0,10.6))

    def test_navigation_feedback_cannot_skip_freshness_generation_or_measured_stop(self):
        msg=dict(schema_version=1,run_id='run',uav_id='uav_1',generation=2,seq=1,
            sample_s=10.,blocked_since_s=4.,reason='START_CLEARANCE_UNKNOWN',position_xy=[0.,0.])
        active={'uav_1':dict(generation=2,stopping=False)}
        motion={'uav_1':dict(sample_s=10.,position_xy=[0.,0.],velocity_xy=[0.,0.])}
        sequences={}
        self.assertTrue(accept(msg,'run',['uav_1'],active,motion,10.,sequences))
        self.assertFalse(accept(msg,'run',['uav_1'],active,motion,10.,sequences))
        for changes in (dict(generation=1),dict(run_id='old'),dict(sample_s=9.),dict(blocked_since_s=9.)):
            self.assertFalse(accept(dict(msg,**changes),'run',['uav_1'],active,motion,10.,{}))
        motion['uav_1']['velocity_xy']=[.3,0.]
        self.assertFalse(accept(msg,'run',['uav_1'],active,motion,10.,{}))

    def test_tracking_refresh_delay_is_at_most_twenty_seconds(self):
        active=dict(started_s=10.)
        self.assertFalse(refresh_due(active,39.99))
        self.assertTrue(refresh_due(active,40.))
        self.assertFalse(refresh_due(active,59.99,True))
        self.assertTrue(refresh_due(active,60.,True))
