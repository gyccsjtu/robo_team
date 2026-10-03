"""Generate an isolated city from platform assets; never edit official resources."""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

SPAWNS = [(0., -3.), (3., -3.), (0., 0.), (3., 0.), (0., 3.), (3., 3.)]


def startup_distances(boxes, points=SPAWNS):
    return [min((math.hypot(max(lo[0]-x, 0., x-lo[1]),
                           max(hi[0]-y, 0., y-hi[1])) for lo, hi in boxes),
                default=float('inf')) for x, y in points]


def prepare(source, output, seed=17):
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    for name in ('map_generator.py', 'base1.world', 'base2.world', 'base3.world',
                 'rover_static.world', 'control_actor.py', 'ObstacleAvoid.py',
                 'score_cal.py', 'white_background.png'):
        shutil.copy2(source/name, output/name)
    original = (output/'map_generator.py').read_text()
    # Only redirect outputs and fix deterministic initialization in our own copy.
    text = re.sub(r'^output_path\s*=.*$', 'output_path = '+repr(str(output)+'/'), original, flags=re.M)
    generated = output/'generate_isolated.py'
    attempts = []
    for attempt in range(12):
        selected = seed+attempt
        generated.write_text(text.replace('import random', 'import random\nrandom.seed(%d)' % selected, 1))
        result = subprocess.run([sys.executable, str(generated)], cwd=output,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=45)
        (output/'generator.log').write_text(result.stdout)
        if result.returncode:
            raise RuntimeError('CITY_GENERATION_FAILED: '+result.stdout[-1200:])
        boxes = ast.literal_eval((output/'black_box.txt').read_text())
        distances = startup_distances(boxes)
        attempts.append(dict(seed=selected, clearance_m=distances))
        if min(distances) > 2.:
            break
    else:
        (output/'startup_failures.json').write_text(json.dumps(attempts, indent=2))
        raise RuntimeError('MAP_STARTUP_OVERLAP: all generated cities intersect platform spawn clearance')
    root = ET.parse(output/'robocup.world')
    world = root.getroot().find('world')
    physics = world.find('physics')
    if physics is None:
        physics = ET.SubElement(world, 'physics', name='default_physics', type='ode')
    for key, value in [('max_step_size','0.004'), ('real_time_update_rate','250')]:
        node = physics.find(key)
        if node is None:
            node = ET.SubElement(physics,key)
        node.text=value
    root.write(output/'robocup.world', encoding='unicode')
    # Compatibility changes apply solely to this copy and are disclosed.
    control = (output/'control_actor.py').read_text()
    control = control.replace("os.path.expanduser('~/XTDrone/robocup/black_box.txt')", repr(str(output/'black_box.txt')))
    control = control.replace('self.left_actors = range(self.actor_num)', 'self.left_actors = list(range(self.actor_num))')
    control = control.replace('self.teleportation_interval = 25', 'self.teleportation_interval = 30')
    (output/'control_actor.py').write_text(control)
    report = dict(schema_version=1, development_scene=True, formal_competition_pass=False,
        source_directory=str(source), seed=selected, attempts=attempts,
        positions={'uav_%d'%(i+1):list(p) for i,p in enumerate(SPAWNS)},
        startup_checks={'uav_%d'%(i+1):dict(free_radius_m=2., rectangle_clearance_m=d)
                        for i,d in enumerate(distances)},
        actor_ids=[a.get('name') for a in world.findall('actor')],
        source_sha256={p.name:hashlib.sha256((source/p.name).read_bytes()).hexdigest()
                       for p in output.iterdir() if (source/p.name).is_file()},
        adaptations=['isolated output paths', 'seeded map randomness', '250Hz PX4-compatible physics',
                     'Python3 actor range', '30s teleport per rule; platform copy used 25s'])
    (output/'scene_manifest.json').write_text(json.dumps(report,indent=2))
    return report


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',default='/root/third_party_official/XTDrone_official/robocup')
    parser.add_argument('--output',required=True)
    parser.add_argument('--seed',type=int,default=17)
    args=parser.parse_args()
    print(json.dumps(prepare(args.source,args.output,args.seed),indent=2))
