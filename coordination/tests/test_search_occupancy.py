import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

SCRIPTS=Path(__file__).parents[1]/'src/robocup_swarm/scripts'
sys.path.insert(0,str(SCRIPTS))
from search_occupancy import apply_authority_release
from swarm_task import STATE_ASSIGNED, STATE_FREE, STATE_COVERED
from task_authority import TaskAuthority
from tracker_selection import tracker_rank


class SearchOccupancyTests(unittest.TestCase):
    def grid(self, state=STATE_ASSIGNED, owner='uav_1'):
        self.cell=SimpleNamespace(state=state,owner=owner,lease_until=2.)
        return SimpleNamespace(cells={(1,2):self.cell})

    def release(self, **changes):
        event=dict(schema_version=2,run_id='run',event='TASK_RELEASED',uav_id='uav_1',details=dict(key=['search',1,2]))
        event.update(changes)
        return event

    def test_stop_retire_timeout_and_old_run_cannot_free_search_occupancy(self):
        grid=self.grid()
        for event in ('STOP_REQUESTED','STOPPED_ACKED','TASK_RETIRED','TASK_BLOCKED'):
            self.assertFalse(apply_authority_release(grid,self.release(event=event),'run',{}))
        self.assertFalse(apply_authority_release(grid,self.release(run_id='old'),'run',{}))
        self.assertEqual((self.cell.state,self.cell.owner,self.cell.lease_until),(STATE_ASSIGNED,'uav_1',2.))

    def test_verified_exit_releases_matching_owner_once(self):
        grid=self.grid()
        self.assertTrue(apply_authority_release(grid,self.release(),'run',{}))
        self.assertEqual((self.cell.state,self.cell.owner),(STATE_FREE,None))
        self.assertFalse(apply_authority_release(grid,self.release(),'run',{}))

    def test_new_owner_covered_cell_or_reacquired_lock_is_never_cleared(self):
        for state,owner,locks in ((STATE_ASSIGNED,'uav_2',{}),(STATE_COVERED,'uav_1',{}),
                                 (STATE_ASSIGNED,'uav_1',{('search',1,2):object()})):
            grid=self.grid(state,owner)
            self.assertFalse(apply_authority_release(grid,self.release(),'run',locks))
            self.assertEqual((self.cell.state,self.cell.owner),(state,owner))

    def test_real_dispatch_outside_sensing_range_retains_cell_until_verified_stop(self):
        tree=ast.parse((SCRIPTS/'swarm_manager.py').read_text(encoding='utf-8'))
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SwarmManager')
        function=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_dispatch_tracker')
        scope=dict(math=math,STATE_ASSIGNED=STATE_ASSIGNED,CRUISE_SPEED=5.,DETECT_RADIUS=20.,DISPATCH_MARGIN=2.,
            SearchAssignment=lambda:SimpleNamespace(header=SimpleNamespace(),target_id=''),
            rospy=SimpleNamespace(loginfo=lambda *a:None,logwarn_throttle=lambda *a:None,
                                  Time=SimpleNamespace(now=lambda:0.)))
        exec(compile(ast.Module(body=[function],type_ignores=[]),'swarm_manager.py','exec'),scope)
        grid=self.grid()
        grid.x_min,grid.x_max,grid.y_min,grid.y_max=-100.,100.,-100.,100.
        offers=[]
        manager=SimpleNamespace(_tracking={},_backup={},_target_authority_held=lambda tid:False,_idle_uavs=lambda:[],
            _tracker_rank=lambda tid,uid,d:tracker_rank(uid,d,[],0.,18.),
            _authorized_publish=offers.append,
            uav_ids=['uav_1'],status={'uav_1':SimpleNamespace(x=0.,y=0.,connected=True)},grid=grid)
        scope['_dispatch_tracker'](manager,'t0',50.,0.)
        self.assertEqual((self.cell.state,self.cell.owner,self.cell.lease_until),(STATE_ASSIGNED,'uav_1',2.))
        self.assertEqual(len(offers),1)
        self.assertEqual((offers[0].target_id,offers[0].task_type,offers[0].target_x),('t0',1,50.))

    def test_real_core_stop_then_exit_controls_auction_release(self):
        grid=self.grid()
        authority=TaskAuthority('run',['uav_1','uav_2'])
        search=dict(cell_ix=1,cell_iy=2,target_x=0.,target_y=0.,task_type=0,target_id='')
        target=dict(cell_ix=-1,cell_iy=-1,target_x=10.,target_y=0.,task_type=1,target_id='t0')
        authority.offer('uav_1',search,0.)
        authority.offer('uav_1',target,1.)
        def ack(seq,stamp,x,status):
            authority.ack(dict(schema_version=2,run_id='run',uav_id='uav_1',seq=seq,
                generation=authority.generation['uav_1'],sample_s=stamp,xyz=[x,0.,4.],speed_mps=0.,
                stopped_s=1.,status=status),stamp)
        for seq in range(1,12):
            ack(seq,1.+(seq-1)*.1,0.,'STOPPED')
        for event in authority.events:
            apply_authority_release(grid,event,'run',authority.locks)
        self.assertEqual(self.cell.state,STATE_ASSIGNED)
        self.assertEqual(authority.active['uav_1']['task']['target_id'],'t0')
        ack(12,2.2,5.,'STATE')
        releases=[e for e in authority.events if e['event']=='TASK_RELEASED']
        self.assertEqual(len(releases),1)
        self.assertTrue(apply_authority_release(grid,releases[0],'run',authority.locks))
        self.assertEqual(self.cell.state,STATE_FREE)
