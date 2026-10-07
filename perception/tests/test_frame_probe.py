#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline checks for perception/frame_probe.py, plus the real CSV write path.

Run:  python perception/tests/test_frame_probe.py
No ROS, no cv2, no VM. Exits non-zero on the first failure.
"""
import ast
import csv
import inspect
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
PERCEPTION = HERE.parent
REPO = PERCEPTION.parent
SCRIPTS = REPO / 'coordination' / 'src' / 'robocup_swarm' / 'scripts'

sys.path.insert(0, str(PERCEPTION))
sys.path.insert(0, str(SCRIPTS))

import frame_probe  # noqa: E402


def check(name, condition, detail=''):
    if condition:
        print('ok   %s' % name)
        return True
    print('FAIL %s  %s' % (name, detail))
    raise SystemExit(1)


def test_fields_are_unique():
    check('FIELDS has no duplicates',
          len(frame_probe.FIELDS) == len(set(frame_probe.FIELDS)),
          str(frame_probe.FIELDS))


def test_summary_counts():
    boxes = [
        ('white', 0.61, 300.0, 200.0, 308.0, 216.7),   # w 8.0 h 16.7, in band
        ('blue', 0.88, 100.0, 150.0, 111.1, 175.4),    # w 11.1 h 25.4, in band
        ('blue', 0.42, 400.0, 5.0, 405.0, 9.0),        # above the ROI band
        ('green', 0.33, 500.0, 470.0, 512.1, 493.2),   # below the ROI band
    ]
    summary = frame_probe.summarize_raw_boxes(boxes, roi_top=131.0, roi_bottom=440.0)
    check('raw_boxes counts the model output', summary['raw_boxes'] == 4, str(summary))
    check('raw_cls is sorted per-class counts',
          summary['raw_cls'] == 'blue:2|green:1|white:1', summary['raw_cls'])
    check('roi_drop counts what our own band removes', summary['roi_drop'] == 2, str(summary))
    check('raw_in_roi is the complement', summary['raw_in_roi'] == 2, str(summary))
    check('maxima come from in-ROI boxes only',
          (summary['raw_w_max'], summary['raw_h_max'], summary['raw_conf_max'])
          == (11.1, 25.4, 0.88), str(summary))


def test_empty_frame_is_still_a_row():
    summary = frame_probe.summarize_raw_boxes([], roi_top=0.0, roi_bottom=360.0)
    check('a frame with zero boxes still summarises',
          summary['raw_boxes'] == 0 and summary['raw_cls'] == ''
          and summary['raw_w_max'] == 0.0, str(summary))


def test_row_shape_matches_fields():
    summary = frame_probe.summarize_raw_boxes(
        [('white', 0.5, 300.0, 200.0, 308.0, 216.0)], roi_top=0.0, roi_bottom=360.0)
    row = frame_probe.frame_row(summary, 1.0, 0.9, 0.46, 0.045, 1, 3,
                                (-0.7, -3.5, 2.5), 0.02, (640, 360), True)
    check('row covers every declared field',
          sorted(row) == sorted(frame_probe.FIELDS),
          '%s vs %s' % (sorted(row), sorted(frame_probe.FIELDS)))
    check('shared is recorded as a bool', row['shared'] is True, repr(row['shared']))
    try:
        frame_probe.frame_row({}, 1.0, 0.9, None, None, 0, 0, (0, 0, 0), 0.0, (0, 0), False)
    except ValueError as error:
        check('a row with missing fields is refused', 'FRAME_ROW_MISSING_FIELDS' in str(error))
    else:
        check('a row with missing fields is refused', False, 'no ValueError raised')


def test_csv_writer_round_trip():
    """Exercise the real CsvLogger with the real FIELDS, offline."""
    with tempfile.TemporaryDirectory() as tmp:
        env = dict(os.environ, ROBOCUP_LOG_DIR=tmp)
        code = (
            'import os,sys,csv,json;'
            'sys.path.insert(0,%r);sys.path.insert(0,%r);'
            'import frame_probe;from csv_logger import logger;'
            'w=logger("frame_probe_uav_1",frame_probe.FIELDS);'
            'rows=[frame_probe.frame_row(frame_probe.summarize_raw_boxes([],roi_top=0.,roi_bottom=360.),1.0,0.9,None,None,0,0,(0.,0.,0.),0.,(640,360),True),'
            'frame_probe.frame_row(frame_probe.summarize_raw_boxes([("white",0.5,300.,200.,308.,216.)],roi_top=0.,roi_bottom=360.),2.0,1.9,0.4,0.045,1,3,(-0.7,-3.5,2.5),0.02,(640,360),True)];'
            '[w.write(**r) for r in rows];w.close();'
            'print(json.dumps([r for r in csv.DictReader(open(w.path))]))'
            % (str(PERCEPTION), str(SCRIPTS))
        )
        out = subprocess.run([sys.executable, '-c', code], capture_output=True,
                             text=True, env=env, check=True)
        import json
        written = json.loads(out.stdout)
        check('csv header matches frame_probe.FIELDS',
              list(written[0]) == ['wall_time'] + frame_probe.FIELDS,
              str(list(written[0])))
        check('one row per processed frame is written (including the empty one)',
              len(written) == 2, str(len(written)))
        check('the zero-detection frame is distinguishable from the firing one',
              written[0]['raw_boxes'] == '0' and written[1]['raw_boxes'] == '1'
              and written[1]['raw_cls'] == 'white:1', str(written))


def test_flight_call_arity():
    """Static check of the one place the flight node touches this module.

    The flight call site cannot be executed off the robot (rospy), so verify its
    shape instead: arity against the real signature, and that the write sits inside
    a try/except so a logging bug can never take the perception loop down mid-round.
    """
    source = (PERCEPTION / 'perception_real.py').read_text(encoding='utf-8')
    tree = ast.parse(source)
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name)
             and node.func.id in ('frame_row', 'summarize_raw_boxes')]
    check('flight node calls summarize_raw_boxes and frame_row',
          sorted(call.func.id for call in calls) == ['frame_row', 'summarize_raw_boxes'],
          str([call.func.id for call in calls]))
    for call in calls:
        parameters = list(inspect.signature(getattr(frame_probe, call.func.id)).parameters)
        check('%s call arity matches its signature' % call.func.id,
              len(call.args) == len(parameters),
              '%d args vs %s' % (len(call.args), parameters))
        for keyword in call.keywords:
            check('%s keyword %s exists' % (call.func.id, keyword.arg),
                  keyword.arg in parameters)
    guarded = any(isinstance(node, ast.Try) and any(
        isinstance(inner, ast.Call) and isinstance(inner.func, ast.Name)
        and inner.func.id == 'frame_row' for inner in ast.walk(node))
        for node in ast.walk(tree))
    check('the per-frame write is inside a try (a logging bug cannot kill the loop)',
          guarded)
    check('the frame log is not named perception*.csv',
          'logger("frame_probe_%s" % UAV' in source)


if __name__ == '__main__':
    test_fields_are_unique()
    test_summary_counts()
    test_empty_frame_is_still_a_row()
    test_row_shape_matches_fields()
    test_csv_writer_round_trip()
    test_flight_call_arity()
    print('\nall frame_probe checks passed')
