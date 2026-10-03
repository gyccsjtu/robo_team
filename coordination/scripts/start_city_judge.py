#!/usr/bin/env python3
"""Start the unmodified platform judge only after its own ROS clock is ready."""
import runpy
import sys
import time
from pathlib import Path


def wait_for_sim_clock(read_clock, shutdown, monotonic=time.monotonic, sleep=time.sleep):
    deadline = monotonic()+30.
    while True:
        stamp = read_clock()
        if stamp > 0:
            return stamp
        if shutdown() or monotonic() >= deadline:
            raise RuntimeError('CITY_JUDGE_CLOCK_NOT_READY')
        sleep(.02)


def main():
    import rospy
    script = Path(sys.argv[1]).resolve()
    sys.argv = [str(script)] + sys.argv[2:]
    # The platform repeats the same init_node call; rospy accepts identical calls.
    rospy.init_node('score_cal')
    stamp = wait_for_sim_clock(rospy.get_time,rospy.is_shutdown)
    print('CITY_JUDGE_CLOCK_READY %.6f' % stamp, flush=True)
    runpy.run_path(str(script),run_name='__main__')


if __name__ == '__main__':
    main()
