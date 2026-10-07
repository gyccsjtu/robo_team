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
from red_judge_compat import adapt_red_judge

SPAWNS = [(0., -3.), (3., -3.), (0., 0.), (3., 0.), (0., 3.), (3., 3.)]


def adapt_map_fallback(text):
    """Reject exhausted placement instead of using empty/mismatched base data."""
    original = ('    if count_loop>=10:\n'
                '        lines = content.readlines()\n'
                '        f.writelines(lines)\n'
                '        print(map_num)\n'
                "        print('Base world is used!')")
    if text.count(original) != 1:
        raise ValueError('MAP_FALLBACK_SOURCE_CHANGED')
    return text.replace(original,"    if count_loop>=10:\n        raise RuntimeError('CITY_PLACEMENT_EXHAUSTED')",1)


def adapt_actor_wait(control):
    """Keep the original teleport deadline without spinning or log flooding.

    Contract: an unrecognised source must fail loudly rather than silently pass,
    because a silent pass would let us believe the guard was applied (see
    test_unknown_source_does_not_silently_claim_adaptation). Callers that know
    upstream removed the loop entirely handle the error explicitly instead.
    """
    original = ('while not responce.success:\n'
                '            if rospy.get_time() - self.teleportation_time < self.teleportation_interval:\n'
                '                print(rospy.get_time() - self.teleportation_time)\n'
                '                continue')
    replacement = ('while not responce.success and not rospy.is_shutdown():\n'
                   '            if rospy.get_time() - self.teleportation_time < self.teleportation_interval:\n'
                   '                rospy.sleep(0.02)\n'
                   '                continue')
    if original not in control:
        raise ValueError('ACTOR_TELEPORT_WAIT_SOURCE_CHANGED')
    return control.replace(original, replacement, 1)


def patch_actor_wait(control):
    """Apply adapt_actor_wait, tolerating upstream's 2026-10-05 rewrite.

    Upstream control_actor.py no longer contains a teleport wait loop at all, so
    the defect the guard patched cannot occur. That case is recognised by the
    absence of any teleportation code, and is reported by the caller rather than
    silently assumed. A loop that is merely rewritten (teleportation still
    present) is NOT tolerated - it re-raises, because then we cannot tell whether
    the spin/flood defect survives.
    """
    try:
        return adapt_actor_wait(control), True
    except ValueError as error:
        if 'ACTOR_TELEPORT_WAIT_SOURCE_CHANGED' not in str(error):
            raise
        if 'teleportation' in control:
            raise
        return control, False


VENDOR_ROOT = Path(__file__).resolve().parents[1] / 'vendor' / 'official_robocup'
# Pinned snapshot of the organizer repository. The live tree under /root is
# shared and mutable (it changed under us on 2026-10-06), so scenes default to a
# versioned copy and the manifest records the hash of every asset consumed.
DEFAULT_SOURCE = VENDOR_ROOT / '20261006'
ACTOR_VARIANTS = {'upstream': None, 'legacy': VENDOR_ROOT / 'legacy_actor_teleport'}
ACTOR_FILES = ('control_actor.py', 'ObstacleAvoid.py')


def adapt_actor_source(control, black_box_path):
    """Apply the actor compatibility edits, returning (text, applied_flags).

    Each edit is an exact-match replacement and its flag is reported by the
    caller, so the scene manifest states what was actually done. A hard-coded
    claim already outlived the code it described once (the 25s->30s teleport
    note), and upstream rewrote this file on 2026-10-05.

    The isolated black-box path is REQUIRED: without the rewrite the actor reads
    the organizer's own ~/XTDrone copy, so the run would look healthy while the
    pedestrians were routed against a different map.
    """
    applied = {}

    def sub(key, old, new, required=False):
        nonlocal control
        if old in control:
            applied[key] = True
            control = control.replace(old, new, 1)
            return
        if required:
            raise ValueError('ACTOR_%s_SOURCE_CHANGED' % key.upper())
        applied[key] = False

    sub('isolated_black_box_path',
        "os.path.expanduser('~/XTDrone/robocup/black_box.txt')",
        repr(str(black_box_path)), required=True)
    sub('python3_actor_range',
        'self.left_actors = range(self.actor_num)',
        'self.left_actors = list(range(self.actor_num))')
    sub('teleport_deadline_30s_per_rule',
        'self.teleportation_interval = 25',
        'self.teleportation_interval = 30')
    # Python 2-style print formatting crashed actor_1 on a transient service
    # disconnect. Retry without substituting a fictitious (0,0) actor pose.
    # The defect survives upstream's 2026-10-05 rewrite, so both variants match.
    sub('transient_service_retry',
        'print("Gazebo model state service"+self.id+"  call failed: %s") % e\n'
        '                self.current_pose.x = 0.0\n'
        '                self.current_pose.y = 0.0\n'
        '                self.current_pose.z = 1.25',
        'print(("Gazebo model state service"+self.id+"  call failed: %s") % e)\n'
        '                rate.sleep()\n'
        '                continue')
    control, applied['teleport_wait_yield_guard'] = patch_actor_wait(control)
    return control, applied


def startup_distances(boxes, points=SPAWNS):
    return [min((math.hypot(max(lo[0]-x, 0., x-lo[1]),
                           max(hi[0]-y, 0., y-hi[1])) for lo, hi in boxes),
                default=float('inf')) for x, y in points]


def prepare(source, output, seed=17, actor='upstream'):
    source, output = Path(source), Path(output)
    if actor not in ACTOR_VARIANTS:
        raise ValueError('UNKNOWN_ACTOR_VARIANT: '+str(actor))
    actor_source = ACTOR_VARIANTS[actor] or source
    output.mkdir(parents=True, exist_ok=False)
    origins = {}
    for name in ('map_generator.py', 'base1.world', 'base2.world', 'base3.world',
                 'rover_static.world', 'control_actor.py', 'ObstacleAvoid.py',
                 'score_cal.py', 'white_background.png'):
        shutil.copy2(source/name, output/name)
        origins[name] = source/name
    if actor_source != source:
        # A/B switch: the legacy actor pair reproduces the rounds recorded before
        # upstream removed teleportation (2026-10-05). Nothing else is swapped,
        # so the only difference between variants is pedestrian behaviour.
        for name in ACTOR_FILES:
            shutil.copy2(actor_source/name, output/name)
            origins[name] = actor_source/name
    original = (output/'map_generator.py').read_text()
    # Only redirect outputs and fix deterministic initialization in our own copy.
    text = adapt_map_fallback(re.sub(r'^output_path\s*=.*$', 'output_path = '+repr(str(output)+'/'), original, flags=re.M))
    generated = output/'generate_isolated.py'
    attempts = []
    for attempt in range(12):
        selected = seed+attempt
        generated.write_text(text.replace('import random', 'import random\nrandom.seed(%d)' % selected, 1))
        result = subprocess.run([sys.executable, str(generated)], cwd=output,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=45)
        (output/'generator.log').write_text(result.stdout)
        if result.returncode:
            if 'CITY_PLACEMENT_EXHAUSTED' in result.stdout:
                attempts.append(dict(seed=selected,reason='CITY_PLACEMENT_EXHAUSTED'))
                continue
            raise RuntimeError('CITY_GENERATION_FAILED: '+result.stdout[-1200:])
        boxes = ast.literal_eval((output/'black_box.txt').read_text())
        distances = startup_distances(boxes)
        attempts.append(dict(seed=selected, clearance_m=distances))
        if min(distances) > 2.:
            break
    else:
        (output/'startup_failures.json').write_text(json.dumps(attempts, indent=2))
        raise RuntimeError('MAP_STARTUP_UNUSABLE: exhausted placement or insufficient spawn clearance')
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
    # Compatibility changes apply solely to this copy and are disclosed below.
    control = (output/'control_actor.py').read_text()
    control, actor_edits = adapt_actor_source(control, output/'black_box.txt')
    (output/'control_actor.py').write_text(control)
    # Only the isolated judge copy implements the organizer clarification, and
    # only when upstream has not implemented it yet.
    judge = output/'score_cal.py'
    judge_source = judge.read_text()
    adapted_judge = adapt_red_judge(judge_source)
    red_matching_uses_upstream = adapted_judge == judge_source
    judge.write_text(adapted_judge)
    # Report only what was actually applied to this source. Hard-coded claims
    # previously outlived the code they described (the 25s->30s teleport note)
    # once upstream rewrote control_actor.py, so every optional edit is derived
    # from adapt_actor_source's flags rather than asserted.
    adaptations = ['isolated output paths', 'seeded map randomness',
                   'reject exhausted random placement; original fallback produced empty world and mismatched obstacle data',
                   '250Hz PX4-compatible physics']
    if actor_edits['isolated_black_box_path']:
        adaptations.append("actor reads this scene's black_box.txt instead of the organizer ~/XTDrone copy")
    if actor_edits['python3_actor_range']:
        adaptations.append('Python3 actor range')
    if actor_edits['transient_service_retry']:
        adaptations.append('actor transient service retry without Python2 print crash or fictitious pose')
    if actor_edits['teleport_deadline_30s_per_rule']:
        adaptations.append('actor teleport deadline 30s per the rule PDF; the platform source ships 25s')
    if actor_edits['teleport_wait_yield_guard']:
        adaptations.append('actor teleport wait yields for 20ms simulated time; original deadline unchanged')
    elif actor_edits['teleport_deadline_30s_per_rule']:
        adaptations.append('actor teleport wait loop not found in a source that still teleports; '
                           'guard NOT applied - inspect before trusting the deadline')
    else:
        adaptations.append('upstream control_actor.py has no teleport code at all '
                           '(removed upstream 2026-10-05); no teleport guard needed')
    if red_matching_uses_upstream:
        adaptations.append('red reports match either remaining red actor via the upstream '
                           'implementation; judge copy left unmodified')
    else:
        adaptations.append('red reports match either remaining red actor; isolated judge copy only')
    teleports = actor_edits['teleport_deadline_30s_per_rule']
    report = dict(schema_version=1, development_scene=True, formal_competition_pass=False,
        source_directory=str(source), actor_variant=actor, actor_source_directory=str(actor_source),
        seed=selected, attempts=attempts,
        positions={'uav_%d'%(i+1):list(p) for i,p in enumerate(SPAWNS)},
        startup_checks={'uav_%d'%(i+1):dict(free_radius_m=2., rectangle_clearance_m=d)
                        for i,d in enumerate(distances)},
        actor_ids=[a.get('name') for a in world.findall('actor')],
        actor_teleportation=('fixed deadline 30s; scene copy rewrites the platform 25s value'
                             if teleports else
                             'none; upstream removed teleportation 2026-10-05, /find_actor now only arms escape'),
        actor_edits=actor_edits,
        actor_source_sha256={name: hashlib.sha256(origins[name].read_bytes()).hexdigest()
                             for name in ACTOR_FILES},
        red_matching_revision=('upstream_20261004' if red_matching_uses_upstream
                               else 'organizer_clarification_20261003_v1'),
        red_matching_source=('Upstream _process_red_detection; either red truth matches a red report'
                             if red_matching_uses_upstream else
                             'User-relayed organizer clarification; either red truth matches a red report'),
        adapted_judge_sha256=hashlib.sha256(judge.read_bytes()).hexdigest(),
        source_sha256={name: hashlib.sha256(path.read_bytes()).hexdigest()
                       for name, path in sorted(origins.items())},
        adaptations=adaptations)
    (output/'scene_manifest.json').write_text(json.dumps(report,indent=2))
    return report


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',default=str(DEFAULT_SOURCE),
                        help='official robocup asset directory; defaults to the pinned vendor snapshot. '
                             'Pass the live tree to detect upstream drift.')
    parser.add_argument('--actor',default='upstream',choices=sorted(ACTOR_VARIANTS),
                        help='pedestrian behaviour: upstream (2026-10-05 rewrite, no teleport) or '
                             'legacy (25s teleport, reproduces pre-2026-10-05 rounds)')
    parser.add_argument('--output',required=True)
    parser.add_argument('--seed',type=int,default=17)
    args=parser.parse_args()
    print(json.dumps(prepare(args.source,args.output,args.seed,args.actor),indent=2))
