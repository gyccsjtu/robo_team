#!/usr/bin/env python3
"""Create a nonformal tracking overlay with scene shadows disabled.

The original six actor collision plugins remain. Light color, intensity,
position and cast_shadows settings remain unchanged; only scene/shadows is false.
This script never starts or stops simulator processes.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('botsort_tracking_prepare', HERE/'prepare_botsort_tracking_dev.py')
tracking = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracking)
prepare = tracking.prepare


def disable_shadows(root):
    shadows = root.findall('./world/scene/shadows')
    if len(shadows) != 1:
        raise ValueError('Expected exactly one explicit world/scene/shadows')
    old = (shadows[0].text or '').strip()
    if old.lower() not in ['1', 'true']:
        raise ValueError('Original shadows already disabled or unsupported: '+old)
    shadows[0].text = 'false'
    return old


def collision_count(root):
    return sum(1 for actor in root.findall('.//actor') for plugin in actor.findall('plugin')
               if Path(plugin.get('filename', '')).name == 'libActorCollisionsPlugin.so')


def verify(source, target):
    manifest = json.loads((target/'tracking_no_shadows_manifest.json').read_text())
    expected = ET.parse(tracking.world_path(source)).getroot()
    old = disable_shadows(expected)
    actual = ET.parse(tracking.world_path(target)).getroot()
    if tracking.structure(expected) != tracking.structure(actual):
        raise ValueError('World differs beyond the one scene/shadows setting')
    if collision_count(actual) != 6:
        raise ValueError('The six original actor collision plugins must remain')
    if old != manifest['scene_shadows_before']:
        raise ValueError('Original scene shadow setting changed')
    if prepare.sha(tracking.world_path(source)) != manifest['source_world_sha256']:
        raise ValueError('Original development world changed')
    if prepare.sha(tracking.world_path(target)) != manifest['tracking_world_sha256']:
        raise ValueError('Tracking world changed after preparation')
    prepare.verify_overlay(target)
    return dict(development_only=True, formal_match=False, target=str(target),
                scene_shadows_before=old, scene_shadows_after=False,
                actor_collision_plugins_retained=6, all_other_world_xml_unchanged=True,
                source_world_sha256=manifest['source_world_sha256'],
                tracking_world_sha256=manifest['tracking_world_sha256'],
                private_startup_ports_valid=True,
                limitation='Shadow rendering differs from official scene; visual comparison only, not formal acceptance.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=prepare.ALLOWED/'environment_v2')
    parser.add_argument('--output', type=Path, default=prepare.ALLOWED/'environment_tracking_no_shadows')
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    source, target = args.source.resolve(), args.output.resolve()
    for path in [source, target]:
        if prepare.ALLOWED.resolve() not in path.parents:
            raise ValueError('Path must be inside the private development root: '+str(path))
    if source == target or source in target.parents or target in source.parents:
        raise ValueError('Source and target must be separate sibling overlays')
    prepare.verify_overlay(source)
    if target.exists() or args.verify_only:
        print(json.dumps(verify(source, target), indent=2))
        return
    source_world = tracking.world_path(source)
    tree = ET.parse(source_world)
    previous = disable_shadows(tree.getroot())
    if collision_count(tree.getroot()) != 6:
        raise ValueError('This comparison must start with all six actor collision plugins')
    source_world_sha = prepare.sha(source_world)
    source_ports_sha = prepare.sha(source/'px4_port_manifest.json')
    shutil.copytree(str(source), str(target), symlinks=True)
    target_world = tracking.world_path(target)
    target_world.write_bytes(ET.tostring(tree.getroot(), encoding='utf-8', xml_declaration=True))
    manifest = json.loads((target/'manifest.json').read_text())
    manifest.update(development_only=True, formal_match=False,
                    px4_root=str(target/'PX4_Firmware'), xtdrone_dir=str(target/'XTDrone'),
                    world_sha256=prepare.sha(target_world), world_source=str(source_world),
                    tracking_no_shadows_manifest='tracking_no_shadows_manifest.json')
    (target/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    ports = json.loads((target/'px4_port_manifest.json').read_text())
    ports.update(px4_build=str(target/'PX4_Firmware/build'),
                 startup_root=str(target/'PX4_Firmware/build/px4_sitl_default/etc'))
    (target/'px4_port_manifest.json').write_text(json.dumps(ports, indent=2)+'\n')
    dev = dict(development_only=True, formal_match=False,
               purpose='visual tracking collection with OGRE shadow geometry generation disabled',
               source_overlay=str(source), source_world_sha256=source_world_sha,
               tracking_world_sha256=prepare.sha(target_world), model_sha256=manifest['model_sha256'],
               scene_shadows_before=previous, scene_shadows_after=False,
               actor_collision_plugins_retained=6, all_other_world_xml_unchanged=True,
               light_settings_unchanged=True,
               limitation='Shadow rendering differs from official scene; visual comparison only, not formal acceptance.')
    (target/'tracking_no_shadows_manifest.json').write_text(json.dumps(dev, indent=2)+'\n')
    if prepare.sha(source_world) != source_world_sha or prepare.sha(source/'px4_port_manifest.json') != source_ports_sha:
        raise ValueError('Original development overlay changed unexpectedly')
    print(json.dumps(verify(source, target), indent=2))


if __name__ == '__main__':
    main()
