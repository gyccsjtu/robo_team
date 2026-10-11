#!/usr/bin/env python3
"""Record disk code/weights at startup, without ROS or simulator subscriptions."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, required=True)
    ap.add_argument('--official', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--started', type=float, required=True)
    args = ap.parse_args()
    root = args.root.resolve()
    paths = [root/'run_match.sh', root/'launch/robocup_with_laser.launch']
    paths.extend(sorted((root/'coordination/src/robocup_swarm/msg').glob('*.msg')))
    for folder in ('perception', 'coordination/src/robocup_swarm/scripts',
                   'coordination/src/robocup_navigation/src/robocup_navigation'):
        paths.extend(sorted((root/folder).glob('*.py')))
    weights = ([Path(os.environ['PR_WEIGHTS'])] if os.environ.get('PR_WEIGHTS') else
               [root/'weights/best_yolo11n_bino_v1.pt',
                Path.home()/'catkin_ws/src/yolov11_ros/weights/best.pt'])
    chosen = next((p for p in weights if p.is_file()), None)
    result = dict(
        script_started_wall_s=args.started,
        snapshot_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=str(root),text=True).strip(),
        source_sha256={str(p.relative_to(root)):digest(p) for p in paths if p.is_file()},
        official_sha256={name:digest(args.official/name)
                         for name in ('score_cal.py','control_actor.py')
                         if (args.official/name).is_file()},
        team_only=os.environ.get('ROBOCUP_TEAM_ONLY') == '1',
        search_segments=os.environ.get('SEARCH_SEGMENTS','0'),
        motion_safety=os.environ.get('MOTION_SAFETY','0'),
        south_patrol=os.environ.get('SOUTH_PATROL','0'),
        report_policy=os.environ.get('BRIDGE_REPORT_POLICY','fit'),
        candidate_view_recovery=os.environ.get('CANDIDATE_VIEW_RECOVERY','0'),
        weights={'path':str(chosen),'sha256':digest(chosen)} if chosen else None,
        note='Disk snapshot before process startup; includes dirty working-tree content. '
             'Effective process parameters are logged by each node. No runtime reload is implied.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    temporary = args.output.with_suffix('.tmp')
    temporary.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    temporary.replace(args.output)
    print('[manifest] %s: %d source files' % (args.output,len(result['source_sha256'])))


if __name__ == '__main__':
    main()
