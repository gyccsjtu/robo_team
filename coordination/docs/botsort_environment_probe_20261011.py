"""Read-only inventory of the existing WSL simulator and tracker dependencies."""
import importlib.metadata as metadata
import json
import os
from pathlib import Path
import subprocess
import sys

paths = [
    '/opt/ros/noetic/setup.bash',
    '/root/PX4_Firmware', '/root/XTDrone', '/root/catkin_ws/devel/setup.bash',
    '/root/third_party_official/PX4-Autopilot',
    '/root/third_party_official/XTDrone_official',
    '/root/robo_team_build/codex_authority_v2/devel/setup.bash',
    '/root/robocup_runtime/stereo_20261002T140417Z',
    '/root/robocup/robocup_ws/devel/setup.bash',
    '/root/vendor_eval/d43bac6/models/typhoon_h480_lidar/typhoon_h480_lidar.sdf',
    '/root/robocup_resources/vm_gazebo_cache_20261002/models',
    '/usr/share/GeographicLib/geoids/egm96-5.pgm',
    '/root/robo_team', '/root/robocup-yolo',
    '/tmp/.X11-unix/X0', '/tmp/.X11-unix/X95',
]
versions = {}
package_paths = {}
for name in ['torch', 'ultralytics', 'numpy', 'opencv-python', 'scipy', 'lap', 'lapx', 'filterpy']:
    try:
        versions[name] = metadata.version(name)
        package_paths[name] = str(metadata.distribution(name).locate_file(''))
    except metadata.PackageNotFoundError:
        versions[name] = None
processes = []
for item in Path('/proc').iterdir():
    if not item.name.isdigit():
        continue
    try:
        args = (item/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        if any(x in args for x in ('gzserver', 'rosmaster', '/bin/px4', 'swarm_agent.py', 'person_report_coord.py')):
            if 'botsort_environment_probe' not in args:
                processes.append({'pid': int(item.name), 'args': args[:600]})
    except OSError:
        pass
weights = []
for root in ['/root', '/mnt/d/a/.robocup/robo_team', '/mnt/d/a/.robocup/robo_team_botsort_20261011']:
    command = ['find', root, '-maxdepth', '6', '-type', 'f', '-name', '*best*.pt']
    output = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True).stdout
    for line in output.splitlines():
        p = Path(line)
        weights.append({'path': line, 'bytes': p.stat().st_size})
print(json.dumps({
    'os_release': Path('/etc/os-release').read_text(),
    'python': subprocess.check_output(['python3', '--version'], text=True).strip(),
    'packages': versions,
    'package_paths': package_paths,
    'python_executable': sys.executable,
    'sys_path': sys.path,
    'paths': {p: {'exists': Path(p).exists(), 'resolved': str(Path(p).resolve())} for p in paths},
    'processes': processes,
    'weights': weights,
    'display': os.environ.get('DISPLAY'),
}, ensure_ascii=False, indent=2))
