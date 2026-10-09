import sys
import unittest
import json
import os
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from start_city_judge import wait_for_sim_clock,interrupted_state
import start_city_judge


class JudgeClockTests(unittest.TestCase):
    def test_wrapper_accepts_only_official_finish_during_ros_interruption(self):
        class Interrupted(Exception):pass
        ros=SimpleNamespace(init_node=lambda *a:None,get_time=lambda:1931.,
                            is_shutdown=lambda:True,ROSInterruptException=Interrupted)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); (root/'scene').mkdir(); (root/'flight').mkdir()
            script=root/'scene/score_cal.py'; terminal=root/'flight/judge_terminal.json'
            for finished in (False,True):
                script.write_text('from rospy import ROSInterruptException\n'
                    'mission_finished=%r\nleft_actors=[]\ntarget_finish=6\nscore=500\n'
                    'raise ROSInterruptException("shutdown")\n'%finished)
                with patch.dict(sys.modules,rospy=ros),patch.object(sys,'argv',['wrapper',str(script)]), \
                     patch.dict(os.environ,ROBOCUP_JUDGE_TERMINAL=str(terminal),ROBOCUP_RUN_ID='run'):
                    if finished:start_city_judge.main()
                    else:
                        with self.assertRaises(Interrupted):start_city_judge.main()
                        self.assertFalse(terminal.exists())
                diagnostic=json.loads((root/'flight/judge_exit_diagnostic.json').read_text())
                self.assertEqual(diagnostic['state']['mission_finished'],finished)
            self.assertTrue(json.loads(terminal.read_text())['mission_finished'])

    def test_interrupted_judge_globals_are_recovered_from_actual_frame(self):
        scope=dict(mission_finished=True,left_actors=[],target_finish=6,score=500)
        script=Path('/tmp/unchanged_judge_test.py')
        try:exec(compile('raise RuntimeError("shutdown")',str(script),'exec'),scope)
        except RuntimeError as error:
            self.assertEqual(interrupted_state(error,script),dict(mission_finished=True,left_actors=[],target_finish=6,score=500))
            self.assertEqual(interrupted_state(error,Path('/tmp/another_judge.py')), {})
    def test_zero_to_nonzero_clock_jump_is_ready_before_judge_starts_timer(self):
        stamps=iter((0.,0.,1930.))
        sleeps=[]
        self.assertEqual(wait_for_sim_clock(lambda:next(stamps),lambda:False,
                          monotonic=lambda:0.,sleep=sleeps.append),1930.)
        self.assertEqual(len(sleeps),2)

    def test_missing_clock_times_out_in_wall_time(self):
        wall=iter((0.,31.))
        with self.assertRaisesRegex(RuntimeError,'CLOCK_NOT_READY'):
            wait_for_sim_clock(lambda:0.,lambda:False,monotonic=lambda:next(wall))

    def test_shutdown_cannot_start_judge_at_zero_time(self):
        with self.assertRaisesRegex(RuntimeError,'CLOCK_NOT_READY'):
            wait_for_sim_clock(lambda:0.,lambda:True)
