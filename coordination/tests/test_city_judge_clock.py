import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from start_city_judge import wait_for_sim_clock


class JudgeClockTests(unittest.TestCase):
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
