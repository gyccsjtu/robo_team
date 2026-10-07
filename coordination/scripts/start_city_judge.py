#!/usr/bin/env python3
"""Start the isolated platform judge copy after its own ROS clock is ready."""
import runpy
import os
import hashlib
import json
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
    try:
        state=runpy.run_path(str(script),run_name='__main__')
    except BaseException as error:
        state = interrupted_state(error,script)
        diagnostic = script.parent.parent/'flight/judge_exit_diagnostic.json'
        try:
            diagnostic.write_text(json.dumps(dict(exception=type(error).__name__,message=str(error),
                sample_s=rospy.get_time(),ros_shutdown=rospy.is_shutdown(),state=state),indent=2,allow_nan=False))
        except (OSError,ValueError) as diagnostic_error:
            print('CITY_JUDGE_DIAGNOSTIC_FAILED %s'%diagnostic_error,flush=True)
        # The new unchanged judge calls signal_shutdown in _finish; Rate.sleep
        # may throw before runpy returns its globals. Only its own terminal flag
        # proves this was an expected finish; an external shutdown remains failure.
        if not isinstance(error,rospy.ROSInterruptException) or state.get('mission_finished') is not True:
            raise
    terminal=os.environ.get('ROBOCUP_JUDGE_TERMINAL')
    if terminal:
        from judge_terminal import write_terminal
        write_terminal(terminal,os.environ['ROBOCUP_RUN_ID'],
                       hashlib.sha256(script.read_bytes()).hexdigest(),state)


def interrupted_state(error, script):
    expected = str(Path(script).resolve())
    state = {}
    trace = error.__traceback__
    while trace is not None:
        if str(Path(trace.tb_frame.f_code.co_filename).resolve()) == expected:
            scope = trace.tb_frame.f_globals
            state = {key:scope.get(key) for key in
                     ('mission_finished','left_actors','target_finish','score')}
        trace = trace.tb_next
    return state


if __name__ == '__main__':
    main()
