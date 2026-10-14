#!/usr/bin/env python3
"""Export a small, tracked player-only Docker context from this checkout."""
import argparse
import io
from pathlib import Path
import shutil
import subprocess
import tarfile

BASE = 'e77c3a2cd72ff25c237a12728f29b852f5c14d0e'
ROOT = Path(__file__).resolve().parents[1]
FORBIDDEN = {
    'control_actor.py', 'score_cal.py', 'judge_system.py', 'target_sim_node.py',
    'guide_actor.py', 'ground_truth_bridge.py', 'swarm_offboard_server.py',
    'swarm_viz.py', 'single_uav_avoidance.py', 'dwa_avoidance.py',
    'track_bench.py', 'team_bench.py', 'verify_swarm_avoidance.py',
    'detection_to_actor.py', 'detection_to_official.py',
}
TOOLS = {
    'wait_match_ready.py', 'audit_match_graph.py', 'check_search_protocol.py',
    'write_run_manifest.py', 'fast_visual_shadow.py',
    'record_search_segments.py', 'record_motion_safety.py',
    'record_tracking_images.py',
}


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT)


def selected(path):
    p = Path(path)
    if p.name in FORBIDDEN or p.suffix == '.bak' or '.bak' in p.name:
        return False
    if path in {'run_match.sh', 'start_player.sh', 'stop_player.sh',
                'launch/robocup_with_laser.launch'}:
        return True
    if path.startswith('deploy/'):
        return True
    if path.startswith('weights/'):
        return p.name in {'best_yolo11n_bino_v3_clean.pt',
                          'best_yolo11n_bino_sculpture_v3.pt'}
    if path.startswith('tools/'):
        return p.name in TOOLS
    if path.startswith('perception/'):
        return p.parent == Path('perception') and p.suffix == '.py'
    if path.startswith('coordination/src/robocup_swarm/'):
        return p.name in {'package.xml', 'CMakeLists.txt'} or (
            p.parent.name == 'msg' and p.suffix == '.msg') or (
            p.parent.name == 'scripts' and p.suffix == '.py')
    if path.startswith('coordination/src/robocup_navigation/'):
        return p.name in {'package.xml', 'CMakeLists.txt', 'astar.py', '__init__.py'} or (
            p.parent.name in {'msg', 'srv'} and p.suffix in {'.msg', '.srv'})
    if path.startswith('coordination/src/robocup_training_worlds/'):
        return path.endswith('worlds/generated/robocup_base.json') or (
            p.name in {'package.xml', 'CMakeLists.txt', 'black_box_to_metadata.py'})
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise SystemExit('output already exists: %s' % output)
    commit = git('rev-parse', 'HEAD').decode().strip()
    subprocess.run(['git', 'merge-base', '--is-ancestor', BASE, commit],
                   cwd=ROOT, check=True)
    if git('diff', '--name-only', 'HEAD', '--').strip():
        raise SystemExit('commit tracked deployment changes before export')
    names = git('ls-files', '-z').decode().split('\0')
    paths = sorted(p for p in names if p and selected(p))
    if not {'run_match.sh', 'start_player.sh', 'stop_player.sh'} <= set(paths):
        raise SystemExit('one-click entrypoint is missing from the commit')
    archive = git('archive', '--format=tar', commit, '--', *paths)
    output.mkdir(parents=True)
    try:
        with tarfile.open(fileobj=io.BytesIO(archive), mode='r:') as tar:
            for item in tar.getmembers():
                target = (output / item.name).resolve()
                target.relative_to(output)
                if item.issym() or item.islnk():
                    raise ValueError('symlinks are not allowed in player context: %s' % item.name)
            tar.extractall(output)
        (output / 'deploy/source_commit.txt').write_text(commit + '\n')
        print('context=%s\ncommit=%s\nfiles=%d' % (output, commit, len(paths)))
    except Exception:
        shutil.rmtree(output)
        raise


if __name__ == '__main__':
    main()
