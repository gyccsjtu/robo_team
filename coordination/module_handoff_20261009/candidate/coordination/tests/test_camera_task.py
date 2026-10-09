import ast
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).parents[1]/'src/robocup_swarm/scripts'))
sys.path.insert(0,str(Path(__file__).parents[2]/'perception'))
from task_authority import TaskAuthority
from camera_task import CameraTask


class CameraTaskTests(unittest.TestCase):
    def setUp(self):
        self.authority=TaskAuthority('current',['uav_4'])
        self.task=dict(cell_ix=-1,cell_iy=-1,target_x=10.,target_y=5.,task_type=1,target_id='t0')
        self.message=self.authority.offer('uav_4',self.task,10.)[0]

    def test_logical_vehicle_receives_its_current_target(self):
        camera=CameraTask('current','uav_4')
        self.assertTrue(camera.receive(self.message,10.1))
        self.assertEqual(camera.target(10.2),'t0')

    def test_wrong_run_and_physical_model_name_cannot_relax_parallax(self):
        for run,uid in [('old','uav_4'),('current','typhoon_h480_3'),('current','uav_3')]:
            camera=CameraTask(run,uid)
            self.assertFalse(camera.receive(self.message,10.1))
            self.assertIsNone(camera.target(10.2))

    def test_stop_and_expiry_remove_tracking_marker(self):
        camera=CameraTask('current','uav_4');camera.receive(self.message,10.1)
        self.assertIsNone(camera.target(self.message['expires_s']))
        stop=self.authority.withdraw('uav_4',10.3)[0]
        self.assertTrue(camera.receive(stop,10.3))
        self.assertIsNone(camera.target(10.3))

    def test_replay_cannot_restore_marker_after_stop(self):
        camera=CameraTask('current','uav_4');camera.receive(self.message,10.1)
        camera.receive(self.authority.withdraw('uav_4',10.3)[0],10.3)
        self.assertFalse(camera.receive(self.message,10.4))
        self.assertIsNone(camera.target(10.4))

    def test_camera_iteration_older_than_concurrent_grant_does_not_latch_stop(self):
        camera=CameraTask('current','uav_4');camera.receive(self.message,10.1)
        self.assertIsNone(camera.target(10.05))
        self.assertFalse(camera.gate.stopping)
        self.assertEqual(camera.target(10.2),'t0')

    def test_actual_filter_skips_attachment_for_tracking_at_eighteen_meters(self):
        source=Path(__file__).parents[2]/'perception/perception_real.py'
        tree=ast.parse(source.read_text(encoding='utf-8'))
        assignment=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign)
            and any(isinstance(t,ast.Name) and t.id=='_skip_attach' for t in n.targets))
        env=dict(_cur_tid_for_verdict='t0',_need_tid='t0',tk=type('T',(),{'rng':18.,'cls':'green'})())
        exec(compile(ast.Module(body=[assignment],type_ignores=[]),'actual_parallax_filter','exec'),env)
        self.assertTrue(env['_skip_attach'])
        env['_need_tid']='t2'
        exec(compile(ast.Module(body=[assignment],type_ignores=[]),'actual_parallax_filter','exec'),env)
        self.assertFalse(env['_skip_attach'])
        env.update(_cur_tid_for_verdict=None,_need_tid=None,tk=type('T',(),{'cls':'red'})())
        exec(compile(ast.Module(body=[assignment],type_ignores=[]),'actual_parallax_filter','exec'),env)
        self.assertFalse(env['_skip_attach'])
        env['_cur_tid_for_verdict']='t5'
        exec(compile(ast.Module(body=[assignment],type_ignores=[]),'actual_parallax_filter','exec'),env)
        self.assertTrue(env['_skip_attach'])
