#!/usr/bin/env python3
"""Prepare a development simulator overlay; never edit the official assets.

This does not start ROS, Gazebo, PX4, actor controllers or aircraft. The two
verified firmware/XTDrone trees are referenced read-only through symlinks;
the world, actor scripts and added sensor model are private physical copies.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

PX4 = Path('/root/third_party_official/PX4-Autopilot')
XT = Path('/root/third_party_official/XTDrone_official')
RUNTIME = Path('/root/robocup_runtime/stereo_20261002T140417Z')
ALLOWED = Path('/root/robo_team_build/botsort_sim_20261011')
REPO = Path(__file__).resolve().parents[3]

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def verify_private_ports(target):
    """Read-only verification, safe while this overlay is in use."""
    target = target.resolve()
    if ALLOWED.resolve() not in target.parents:
        raise ValueError('Unsafe development target: '+str(target))
    build = target/'PX4_Firmware/build'
    startup = build/'px4_sitl_default/etc/init.d-posix'
    if build.is_symlink() or not build.is_dir():
        raise ValueError('PX4 build must have a private physical directory')
    if ALLOWED.resolve() not in startup.resolve().parents:
        raise ValueError('PX4 startup directory points outside the private root')
    record = json.loads((target/'px4_port_manifest.json').read_text())
    for raw_path, expected in record['official_sha256'].items():
        if sha(Path(raw_path)) != expected:
            raise ValueError('Official startup source differs from the pinned manifest: '+raw_path)
    data = (startup/'px4-rc.mavlink').read_text()
    for required in ['udp_offboard_port_local=$((34580+px4_instance))',
                     'udp_offboard_port_remote=$((24540+px4_instance))']:
        if required not in data:
            raise ValueError('Private Offboard configuration is missing: '+required)
    for name in ['6011_typhoon_h480.post', '6012_typhoon_h480_ctrlalloc.post']:
        profile = (startup/'airframes'/name).read_text()
        for required in ['udp_typhoon_extra_port_local=$((14558+px4_instance))',
                         'udp_typhoon_extra_port_remote=$((14530+px4_instance))',
                         '-u $udp_typhoon_extra_port_local',
                         '-o $udp_typhoon_extra_port_remote']:
            if required not in profile:
                raise ValueError('Private Typhoon configuration is missing: '+required)
    rows = record['assignments']
    if [row['instance'] for row in rows] != list(range(6)):
        raise ValueError('Expected exactly six numbered port assignments')
    offsets = dict(offboard_local=34580, offboard_remote=24540,
                   payload_local=14280, payload_remote=14030,
                   gimbal_local=13030, gimbal_remote=13280, gcs_local=18570,
                   extra_local=14558, extra_remote=14530, simulator_tcp=4560)
    for row in rows:
        for key, base in offsets.items():
            if row[key] != base+row['instance']:
                raise ValueError('Invalid assigned port: '+key)
    local = [row[key] for row in rows for key in
             ['offboard_local', 'payload_local', 'gimbal_local', 'gcs_local', 'extra_local']]
    if len(local) != len(set(local)):
        raise ValueError('Six-instance PX4 local UDP ports overlap')
    return record

def verify_overlay(target):
    """Reuse complete overlays without writing or replacing any running file."""
    target = target.resolve()
    record = json.loads((target/'manifest.json').read_text())
    model = target/'PX4_Firmware/Tools/sitl_gazebo/models/typhoon_h480_laser/typhoon_h480_laser.sdf'
    mirror = target/'XTDrone/sitl_config/models/typhoon_h480_laser/typhoon_h480_laser.sdf'
    world = target/'PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world'
    if model.read_bytes() != mirror.read_bytes():
        raise ValueError('PX4/XTDrone private model copies differ')
    if sha(model) != record['model_sha256'] or sha(world) != record['world_sha256']:
        raise ValueError('Overlay model/world differs from the pinned manifest; inspect without overwriting')
    checker = module('botsort_sensor_check', REPO/'tools/check_official_sensor_spec.py')
    checker.check_spec(model, mirror,
                       XT/'sitl_config/models/hokuyo_lidar/model.sdf',
                       XT/'sitl_config/models/stereo_camera/model.sdf')
    verify_private_ports(target)
    return dict(development_only=True, reused_without_writes=True,
                target=str(target), private_startup_ports_valid=True,
                sensor_spec_check=True, model_sha256=sha(model), world_sha256=sha(world))

def isolate_px4_ports(target):
    """Replace only the private build link; copy all PX4 startup files first."""
    target = target.resolve()
    if ALLOWED.resolve() not in target.parents:
        raise ValueError('Unsafe development target: '+str(target))
    px4 = target/'PX4_Firmware'
    build = px4/'build'
    official_build = (PX4/'build').resolve()
    if build.is_dir() and not build.is_symlink():
        return verify_private_ports(target)
    if not build.is_symlink() or build.resolve() != official_build:
        raise ValueError('Expected the untouched private build link; inspect before replacing: '+str(build))
    startup = official_build/'px4_sitl_default/etc/init.d-posix'
    original_files = [startup/'px4-rc.mavlink',
                      startup/'airframes/6011_typhoon_h480.post',
                      startup/'airframes/6012_typhoon_h480_ctrlalloc.post']
    before = {str(p): sha(p) for p in original_files}
    staging = px4/'build_private_20261011'
    if staging.exists():
        raise FileExistsError('Refusing to overwrite private build staging: '+str(staging))
    staging.mkdir()
    for item in official_build.iterdir():
        if item.name != 'px4_sitl_default':
            (staging/item.name).symlink_to(item.resolve(), target_is_directory=item.is_dir())
            continue
        sitl = staging/item.name
        sitl.mkdir()
        for content in item.iterdir():
            out = sitl/content.name
            if content.name == 'etc':
                # Dereference any links so subsequent startup-file writes
                # cannot reach the original compiled firmware tree.
                shutil.copytree(str(content.resolve()), str(out), symlinks=False)
            else:
                out.symlink_to(content.resolve(), target_is_directory=content.is_dir())
    private = staging/'px4_sitl_default/etc/init.d-posix'
    config = private/'px4-rc.mavlink'
    data = config.read_text()
    for old, new in [('udp_offboard_port_local=$((14580+px4_instance))',
                      'udp_offboard_port_local=$((34580+px4_instance))'),
                     ('udp_offboard_port_remote=$((14540+px4_instance))',
                      'udp_offboard_port_remote=$((24540+px4_instance))'),
                     ('udp_offboard_port_remote=14549', 'udp_offboard_port_remote=24549')]:
        if old not in data:
            raise ValueError('Unexpected original PX4 port config: '+old)
        data = data.replace(old, new)
    config.write_text(data)
    for name in ['6011_typhoon_h480.post', '6012_typhoon_h480_ctrlalloc.post']:
        path = private/'airframes'/name
        data = path.read_text()
        if '-u 14558' not in data or '-o 14530' not in data:
            raise ValueError('Unexpected typhoon post profile: '+name)
        prefix = ('# Private development port isolation; not an official firmware change.\n'
                  'udp_typhoon_extra_port_local=$((14558+px4_instance))\n'
                  'udp_typhoon_extra_port_remote=$((14530+px4_instance))\n')
        data = re.sub(r'(?<=-u )14558\b', '$udp_typhoon_extra_port_local', data)
        data = re.sub(r'(?<=-o )14530\b', '$udp_typhoon_extra_port_remote', data)
        path.write_text(prefix+data)
    # Unlinking this validated symlink never removes its official target.
    # Both the staging and destination parent are real private directories.
    if staging.resolve().parent != px4.resolve() or build.parent.resolve() != px4.resolve():
        raise ValueError('Private build paths escaped their intended root')
    build.unlink()
    staging.rename(build)
    after = {str(p): sha(p) for p in original_files}
    if before != after:
        raise ValueError('Official PX4 startup files changed unexpectedly')
    assignments = []
    for instance in range(6):
        assignments.append(dict(instance=instance, system_id=instance+1,
            simulator_tcp=4560+instance, offboard_local=34580+instance,
            offboard_remote=24540+instance, payload_local=14280+instance,
            payload_remote=14030+instance, gimbal_local=13030+instance,
            gimbal_remote=13280+instance, gcs_local=18570+instance,
            extra_local=14558+instance, extra_remote=14530+instance))
    local = [row[key] for row in assignments for key in
             ['offboard_local', 'payload_local', 'gimbal_local', 'gcs_local', 'extra_local']]
    if len(local) != len(set(local)):
        raise ValueError('Six-instance PX4 local UDP ports overlap')
    manifest = dict(development_only=True, official_files_unchanged=(before == after),
                    official_sha256=before, px4_build=str(build),
                    startup_root=str(build/'px4_sitl_default/etc'),
                    assignments=assignments,
                    shared_gcs_remote=14550,
                    gimbal_model_remote_requires_instance_injection=True)
    (target/'px4_port_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest

def module(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result

def overlay(source, target, expand):
    target.mkdir()
    for item in source.iterdir():
        if item.name == '.git':
            continue
        out = target/item.name
        if item.name in expand:
            overlay(item, out, expand[item.name])
        else:
            out.symlink_to(item.resolve(), target_is_directory=item.is_dir())

def text(parent, name, value):
    ET.SubElement(parent, name).text = str(value)

def make_model(destination):
    # PX4's installed tree keeps only the jinja source for this airframe;
    # XTDrone contains the already generated stock SDF used by this runtime.
    source = XT/'sitl_config/models/typhoon_h480/typhoon_h480.sdf'
    root = ET.fromstring(source.read_text())
    model = root.find('model')
    body = model.find("link[@name='base_link']")
    if body is None:
        raise ValueError('Original typhoon model has no base_link')
    for plugin in model.findall('.//plugin'):
        filename = plugin.get('filename', '')
        if filename == 'libgazebo_gps_plugin.so':
            plugin.set('filename', str(RUNTIME/'gps/librobocup_legacy_model_gps_plugin.so'))
        elif filename == 'libgazebo_mavlink_interface.so':
            plugin.set('filename', str(RUNTIME/'gps/librobocup_legacy_gps_mavlink_interface.so'))
            if plugin.find('gpsSubTopic') is None:
                text(plugin, 'gpsSubTopic', '/gps')
    # Use the checked-in camera patch constants, with all optical parameters
    # subsequently compared against the actual stock XTDrone sensor.
    stereo_patch = module('botsort_stereo_patch', REPO/'perception/patch/add_stereo_camera.py')
    if body.find("sensor[@name='stereo_camera_left']") is None:
        for sensor in ET.fromstring('<container>'+stereo_patch.SENSORS+'</container>'):
            body.append(sensor)
    stock = ET.parse(XT/'sitl_config/models/hokuyo_lidar/model.sdf').getroot()
    sensor = copy.deepcopy(stock.find(".//sensor[@name='laser']"))
    if sensor is None:
        raise ValueError('Stock Hokuyo sensor missing')
    sensor.set('name', 'laser_2d')
    sensor.set('type', 'ray')
    old_pose = sensor.find('pose')
    if old_pose is not None:
        sensor.remove(old_pose)
    text(sensor, 'pose', '0 0 0.12 0 0 0')
    for plugin in list(sensor.findall('plugin')):
        sensor.remove(plugin)
    plugin = ET.SubElement(sensor, 'plugin', name='laser_2d_ros', filename='libgazebo_ros_laser.so')
    for field, value in [('robotNamespace', ''), ('topicName', 'scan'),
                         ('frameName', 'base_link'),
                         ('updateRate', sensor.findtext('update_rate')),
                         ('min_range', sensor.findtext('ray/range/min')),
                         ('max_range', sensor.findtext('ray/range/max')),
                         ('gaussianNoise', '0.0')]:
        text(plugin, field, value)
    body.append(sensor)
    destination.mkdir()
    path = destination/'typhoon_h480_laser.sdf'
    path.write_bytes(ET.tostring(root, encoding='utf-8', xml_declaration=True))
    (destination/'model.config').write_text(
        "<?xml version='1.0'?><model><name>typhoon_h480_laser</name>"
        "<version>1.0</version><sdf version='1.6'>typhoon_h480_laser.sdf</sdf>"
        "<author><name>RoboCup development</name></author>"
        "<description>Private comparison sensor model</description></model>")
    return source, path

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ALLOWED/'environment_v2')
    parser.add_argument('--isolate-px4-ports', action='store_true',
                        help='Copy startup files into an existing private overlay and isolate six-instance ports')
    parser.add_argument('--verify-only', action='store_true',
                        help='Read-only verification of a complete overlay, including private startup ports')
    args = parser.parse_args()
    target = args.output.resolve()
    if ALLOWED.resolve() not in target.parents:
        raise ValueError('Output must be a new child of '+str(ALLOWED))
    if args.verify_only:
        print(json.dumps(verify_overlay(target), indent=2))
        return
    if args.isolate_px4_ports:
        print(json.dumps(isolate_px4_ports(target), indent=2))
        return
    if target.exists():
        print(json.dumps(verify_overlay(target), indent=2))
        return
    target.mkdir(parents=True)
    px4 = target/'PX4_Firmware'
    xt = target/'XTDrone'
    overlay(PX4, px4, {'Tools': {'sitl_gazebo': {'models': {}, 'worlds': {}}}})
    overlay(XT, xt, {'sitl_config': {'models': {}}, 'robocup': {}})
    # Replace only private overlay symlinks with physical copies before writes.
    for item in (xt/'robocup').iterdir():
        original = item.resolve()
        item.unlink()
        if original.is_dir():
            shutil.copytree(str(original), str(item), symlinks=True)
        else:
            shutil.copy2(str(original), str(item))
    world = px4/'Tools/sitl_gazebo/worlds/robocup.world'
    original_world = world.resolve()
    world.unlink()
    shutil.copy2(str(original_world), str(world))
    model_name = 'typhoon_h480_laser'
    source, model_path = make_model(px4/'Tools/sitl_gazebo/models'/model_name)
    mirror = xt/'sitl_config/models'/model_name
    shutil.copytree(str(model_path.parent), str(mirror))
    checker = module('botsort_sensor_check', REPO/'tools/check_official_sensor_spec.py')
    checker.check_spec(model_path, mirror/model_path.name,
                       XT/'sitl_config/models/hokuyo_lidar/model.sdf',
                       XT/'sitl_config/models/stereo_camera/model.sdf')
    manifest = dict(development_only=True, official_assets_modified=False,
                    sensor_spec_check=True, px4_root=str(px4), xtdrone_dir=str(xt),
                    runtime=str(RUNTIME), stock_model_sha256=sha(source),
                    model_sha256=sha(model_path), world_sha256=sha(world),
                    world_source=str(original_world))
    (target/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    # A fresh preparation must include the port correction; callers should
    # not need a separate undocumented mutation before starting six aircraft.
    isolate_px4_ports(target)
    print(json.dumps(verify_overlay(target), indent=2))

if __name__ == '__main__':
    main()
