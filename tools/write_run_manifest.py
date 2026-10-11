#!/usr/bin/env python3
"""Record disk code/weights at startup, without ROS or simulator subscriptions."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def git_head(root):
    """Resolve a Windows-created worktree when the runtime is WSL."""
    command = ['git']
    pointer = root / '.git'
    if os.name != 'nt' and pointer.is_file():
        match = re.fullmatch(r'gitdir:\s*([A-Za-z]):[/\\](.+)',
                             pointer.read_text().strip())
        if match:
            git_dir = Path('/mnt') / match[1].lower() / match[2].replace('\\', '/')
            if not git_dir.is_dir():
                raise RuntimeError('Windows worktree Git directory is unavailable: %s' % git_dir)
            command.extend(['--git-dir=' + str(git_dir), '--work-tree=' + str(root)])
    command.extend(['rev-parse', 'HEAD'])
    return subprocess.check_output(command, cwd=str(root), text=True).strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, required=True)
    ap.add_argument('--official', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--started', type=float, required=True)
    args = ap.parse_args()
    root = args.root.resolve()
    paths = [root/'run_match.sh', root/'launch/robocup_with_laser.launch',
             root/'tools/audit_match_graph.py', root/'tools/wait_match_ready.py',
             root/'tools/write_run_manifest.py']
    for folder in ('perception', 'coordination/src/robocup_swarm/scripts',
                   'coordination/src/robocup_navigation/src/robocup_navigation'):
        paths.extend(sorted((root/folder).rglob('*.py')))
    paths.extend([root/'perception/requirements.txt',
                  root/'perception/vendor/ultralytics_trackers/LICENSE',
                  root/'perception/vendor/ultralytics_trackers/SOURCE.md'])
    weights = ([Path(os.environ['PR_WEIGHTS'])] if os.environ.get('PR_WEIGHTS') else
               [root/'weights/best_yolo11n_bino_v1.pt',
                Path.home()/'catkin_ws/src/yolov11_ros/weights/best.pt'])
    chosen = next((p for p in weights if p.is_file()), None)
    runtime_assets = []
    if os.environ.get('PX4_ROOT'):
        px4 = Path(os.environ['PX4_ROOT'])
        runtime_assets.extend([
            px4/'Tools/sitl_gazebo/worlds/robocup.world',
            px4/'Tools/sitl_gazebo/models/typhoon_h480_laser/typhoon_h480_laser.sdf',
            px4/'build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink',
            px4/'build/px4_sitl_default/etc/init.d-posix/airframes/6011_typhoon_h480.post'])
    if os.environ.get('ROBOCUP_ACTOR_PLUGIN'):
        runtime_assets.append(Path(os.environ['ROBOCUP_ACTOR_PLUGIN']))
    result = dict(
        script_started_wall_s=args.started,
        snapshot_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        git_head=git_head(root),
        source_sha256={str(p.relative_to(root)):digest(p) for p in paths if p.is_file()},
        development_trial=os.environ.get('ROBOCUP_TRIAL_KIND'),
        runtime_asset_sha256={str(p):digest(p) for p in runtime_assets if p.is_file()},
        official_sha256={name:digest(args.official/name)
                         for name in ('score_cal.py','control_actor.py')
                         if (args.official/name).is_file()},
        team_only=os.environ.get('ROBOCUP_TEAM_ONLY') == '1',
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
