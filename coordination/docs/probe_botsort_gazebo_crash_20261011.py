#!/usr/bin/env python3
"""Read-only, bounded diagnostics of the 2026-10-11 development crash."""
import argparse
import json
from pathlib import Path
import subprocess

BASE = Path('/root/robo_team_build/botsort_sim_20261011')
LOG = BASE/'logs/logs_20261011_183813/01_scene.log'

def run(args):
    result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            universal_newlines=True, timeout=20)
    return dict(returncode=result.returncode, output=result.stdout[-9000:])

kernel = subprocess.run(['dmesg', '--ctime'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        universal_newlines=True, timeout=20)
kernel_lines = [line for line in kernel.stdout.splitlines()
                if any(word in line.lower() for word in ['segfault', 'fatal signal', 'capturecrash', 'out of memory', 'killed process'])]
data = LOG.read_text(errors='replace')
scene_lines = [line for line in data.splitlines()
               if any(word in line for word in ['Segmentation fault', 'process[gazebo', '[gazebo-2]', '1790.892'])]
report = dict(development_only=True, run=str(LOG.parent), core_pattern=Path('/proc/sys/kernel/core_pattern').read_text().strip(),
              scene_exit_lines=scene_lines[-15:], kernel_crash_lines=kernel_lines,
              core_candidates=run(['find', '/root', '/tmp', '/mnt/wslg', '-maxdepth', '4', '-type', 'f', '-name', 'core', '-print']),
              ogre_symbols=run(['gdb', '-nx', '-batch', '-ex', 'info symbol 0x1da094', '-ex', 'info symbol 0xc4094',
                                '/usr/lib/x86_64-linux-gnu/libOgreMain.so.1.9.0']),
              limitation='Kernel instruction is inside OGRE. Without a core/backtrace the initiating cause (actor animation/render race vs other memory misuse) is unproven; no OOM kill found in retained kernel tail.')
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path)
args = parser.parse_args()
serialized = json.dumps(report, indent=2)+'\n'
if args.output:
    destination = args.output.resolve()
    if destination.parent != Path(__file__).resolve().parent:
        raise ValueError('Diagnostic output must be in this documentation directory')
    destination.write_text(serialized)
print(serialized)
