from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).parents[1]/'src/robocup_swarm/scripts'))
sys.path.insert(0,str(Path(__file__).parents[1]/'src/robocup_navigation/src'))
sys.path.insert(0,str(Path(__file__).parents[2]/'perception'))
from online_radar_planner import OnlinePlanner
from radar_observed_map import ObservedMap
from navigation_feedback import accept,refresh_due,RejectedTasks
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

    def test_failed_task_goes_to_other_aircraft_until_fresh_progress(self):
        rejected=RejectedTasks(); key=('search',2,10)
        rejected.reject('uav_5',key,(-44.,13.))
        samples={'uav_5':dict(sample_s=10.,position_xy=[-44.,13.])}
        self.assertFalse(rejected.allowed('uav_5',key,samples,10.))
        self.assertFalse(rejected.allowed('uav_5',key,samples,40.))
        self.assertTrue(rejected.allowed('uav_6',key,samples,10.))
        self.assertTrue(rejected.allowed('uav_5',('search',3,10),samples,10.))
        samples['uav_5']=dict(sample_s=10.,position_xy=[-43.,13.])
        self.assertFalse(rejected.allowed('uav_5',key,samples,11.))
        self.assertTrue(rejected.allowed('uav_5',key,samples,10.1))

    def test_actual_auction_filter_rejects_failed_aircraft_for_same_cell(self):
        import ast
        from types import SimpleNamespace
        path=Path(__file__).parents[1]/'src/robocup_swarm/scripts/swarm_manager.py'
        tree=ast.parse(path.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SwarmManager')
        allowed=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_navigation_task_allowed')
        allocate=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_allocate')
        candidate=next(n for n in ast.walk(allocate) if isinstance(n,ast.FunctionDef) and n.name=='candidate_filter')
        rejected=RejectedTasks();rejected.reject('uav_5',('search',2,10),(-44.,13.))
        manager=SimpleNamespace(_navigation_rejections=rejected,
            _route_motion=SimpleNamespace(samples={'uav_5':dict(sample_s=10.,position_xy=[-44.,13.])}))
        scope=dict(self=manager,executing={},positions={'uav_5':(-44.,13.),'uav_6':(-30.,13.)},
            waypoint=lambda key:(-32.,13.),screen_leg=lambda *a,**k:True,
            rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:10.))),
            os=__import__('os'))
        exec(compile(ast.Module(body=[allowed,candidate],type_ignores=[]),str(path),'exec'),scope)
        manager._navigation_task_allowed=lambda uid,key:scope['_navigation_task_allowed'](manager,uid,key)
        self.assertFalse(scope['candidate_filter']('uav_5',(2,10),{}))
        self.assertTrue(scope['candidate_filter']('uav_6',(2,10),{}))
        self.assertTrue(scope['candidate_filter']('uav_5',(3,10),{}))

    def test_green_white_retry_counts_only_resets_in_current_attempt(self):
        import ast
        from types import SimpleNamespace
        from unittest.mock import Mock
        path=Path(__file__).parents[1]/'src/robocup_swarm/scripts/swarm_agent.py'
        tree=ast.parse(path.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SwarmAgent')
        callback=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_find_cb')
        control=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_control')
        condition=ast.dump(ast.parse("target_id in ('t0','t3') and self._orbit_target != target_id",mode='eval').body)
        entry=next(n for n in ast.walk(control) if isinstance(n,ast.If)
                   and ast.dump(n.test)==condition)
        clock=SimpleNamespace(Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:20.)),logwarn=Mock())
        for idx in range(6):
            agent=SimpleNamespace(_reset_n={idx:7},_find_t={idx:10.},_reset_t={},
                _orbit_target=None,uav_id='uav_1',_giveup_until={},_abort_orbit=Mock())
            scope=dict(self=agent,target_id='t%d'%idx,rospy=clock,
                BACKOFF_ENABLE=True,CONFIRM_RESET_MAX=3,BACKOFF_COOLDOWN=60.)
            exec(compile(ast.Module(body=[entry,callback],type_ignores=[]),str(path),'exec'),scope)
            if idx not in (0,3):
                self.assertEqual(agent._reset_n[idx],7)
                continue
            agent._orbit_target='t%d'%idx
            for timestamp in (11.,12.):
                scope['_find_cb'](agent,SimpleNamespace(data=timestamp),idx)
                # Subsequent control ticks must retain this attempt's failures.
                exec(compile(ast.Module(body=[entry],type_ignores=[]),str(path),'exec'),scope)
                agent._abort_orbit.assert_not_called()
            scope['_find_cb'](agent,SimpleNamespace(data=13.),idx)
            agent._abort_orbit.assert_called_once()
            self.assertEqual(agent._giveup_until['t%d'%idx],80.)
