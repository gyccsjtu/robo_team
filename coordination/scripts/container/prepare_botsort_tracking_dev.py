#!/usr/bin/env python3
"""Prepare a nonformal tracking world without touching the original overlay.

Only ActorCollisionsPlugin elements are removed. Actor appearance, animation,
pose command plugins, buildings, aircraft, cameras and lidar remain unchanged.
This script never starts or stops simulator processes.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('botsort_prepare', HERE/'prepare_botsort_sim_runtime.py')
prepare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare)


def world_path(target):
    return target/'PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world'


def remove_collision_plugins(root):
    removed = []
    for actor in root.findall('.//actor'):
        for plugin in list(actor.findall('plugin')):
            if Path(plugin.get('filename', '')).name == 'libActorCollisionsPlugin.so':
                removed.append(dict(actor=actor.get('name'), name=plugin.get('name'),
                                    filename=plugin.get('filename')))
                actor.remove(plugin)
    return removed


def structure(element):
    return (element.tag, sorted(element.attrib.items()), (element.text or '').strip(),
            [structure(child) for child in element])


def verify(source, target):
    dev = json.loads((target/'tracking_dev_manifest.json').read_text())
    before = ET.parse(world_path(source)).getroot()
    removed = remove_collision_plugins(before)
    after = ET.parse(world_path(target)).getroot()
    if structure(before) != structure(after):
        raise ValueError('Tracking world differs beyond removal of actor collision plugins')
    if removed != dev['removed_plugins'] or len(removed) != 6:
        raise ValueError('Expected exactly the six original actor collision plugins')
    if prepare.sha(world_path(source)) != dev['source_world_sha256']:
        raise ValueError('Original development world changed after this copy was prepared')
    if prepare.sha(world_path(target)) != dev['tracking_world_sha256']:
        raise ValueError('Tracking world differs from its pinned development manifest')
    prepare.verify_overlay(target)
    return dict(development_only=True, formal_match=False,
                purpose='visual tracking data collection; collision/navigation acceptance excluded',
                source_overlay=str(source), tracking_overlay=str(target),
                removed_actor_collision_plugins=len(removed),
                all_other_world_xml_unchanged=True,
                source_world_sha256=dev['source_world_sha256'],
                tracking_world_sha256=dev['tracking_world_sha256'],
                private_startup_ports_valid=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=prepare.ALLOWED/'environment_v2')
    parser.add_argument('--output', type=Path, default=prepare.ALLOWED/'environment_tracking_dev')
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    source, target = args.source.resolve(), args.output.resolve()
    for path in [source, target]:
        if prepare.ALLOWED.resolve() not in path.parents:
            raise ValueError('Path must be a child of the private development root: '+str(path))
    if source == target or source in target.parents or target in source.parents:
        raise ValueError('Source and target overlays must be separate siblings')
    prepare.verify_overlay(source)
    if target.exists() or args.verify_only:
        print(json.dumps(verify(source, target), indent=2))
        return
    source_world_sha = prepare.sha(world_path(source))
    source_ports_sha = prepare.sha(source/'px4_port_manifest.json')
    # Preserve read-only compiled libraries as links and physically duplicate
    # the original overlay's private startup scripts, world and model files.
    shutil.copytree(str(source), str(target), symlinks=True)
    tree = ET.parse(world_path(target))
    removed = remove_collision_plugins(tree.getroot())
    if len(removed) != 6:
        raise ValueError('Expected exactly six actor collision plugins; preserve incomplete copy for inspection')
    world_path(target).write_bytes(ET.tostring(tree.getroot(), encoding='utf-8', xml_declaration=True))
    manifest = json.loads((target/'manifest.json').read_text())
    manifest.update(development_only=True, formal_match=False,
                    px4_root=str(target/'PX4_Firmware'), xtdrone_dir=str(target/'XTDrone'),
                    world_sha256=prepare.sha(world_path(target)),
                    world_source=str(world_path(source)),
                    tracking_dev_manifest='tracking_dev_manifest.json')
    (target/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    ports = json.loads((target/'px4_port_manifest.json').read_text())
    ports.update(px4_build=str(target/'PX4_Firmware/build'),
                 startup_root=str(target/'PX4_Firmware/build/px4_sitl_default/etc'))
    (target/'px4_port_manifest.json').write_text(json.dumps(ports, indent=2)+'\n')
    dev = dict(development_only=True, formal_match=False,
               purpose='visual tracking data collection; collision/navigation acceptance excluded',
               source_overlay=str(source), source_world_sha256=source_world_sha,
               tracking_world_sha256=prepare.sha(world_path(target)),
               model_sha256=manifest['model_sha256'], removed_plugins=removed,
               retained_actor_count=len(tree.getroot().findall('.//actor')),
               all_other_world_xml_unchanged=True)
    (target/'tracking_dev_manifest.json').write_text(json.dumps(dev, indent=2)+'\n')
    if prepare.sha(world_path(source)) != source_world_sha or prepare.sha(source/'px4_port_manifest.json') != source_ports_sha:
        raise ValueError('Original overlay changed unexpectedly')
    print(json.dumps(verify(source, target), indent=2))


if __name__ == '__main__':
    main()
