#!/usr/bin/env python3
"""Generate radar fleet SDF copies without changing sensor parameters."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def sensor_signature(root):
    """Physical sensor settings, excluding only ROS topic/frame wiring."""
    result = []
    for sensor in root.findall('.//sensor'):
        physical = copy.deepcopy(sensor)
        for plugin in list(physical.findall('plugin')):
            physical.remove(plugin)
        result.append(ET.tostring(physical, encoding='unicode'))
    return result


def adapt(source, row, overlay):
    root = ET.fromstring(source)
    model = root.find('model')
    if model is None:
        raise ValueError('Expected top-level model')
    before = sensor_signature(root)
    model.set('name', row['model_name'])
    replacements = {'libgazebo_gps_plugin.so': 'librobocup_legacy_model_gps_plugin.so',
                    'libgazebo_mavlink_interface.so': 'librobocup_legacy_gps_mavlink_interface.so'}
    for plugin in root.findall('.//plugin'):
        library = plugin.get('filename', '')
        if library in replacements:
            plugin.set('filename', str(Path(overlay) / replacements[library]))
        if library.startswith('libgazebo_ros_'):
            namespace = plugin.find('robotNamespace')
            if namespace is None:
                namespace = ET.SubElement(plugin, 'robotNamespace')
            namespace.text = '/' + row['uav_id']
            frame = plugin.find('frameName')
            if frame is not None:
                frame.text = row['uav_id'] + '/' + frame.text.lstrip('/')
            camera = plugin.find('cameraName')
            if camera is not None:
                camera.text = camera.text.lstrip('/')
            topic = plugin.find('topicName')
            if library == 'libgazebo_ros_laser.so' and topic is not None:
                topic.text = row['scan_topic']
        if plugin.get('name') == 'mavlink_interface':
            for key, value in [('mavlink_tcp_port', row['simulator_tcp_port']),
                               ('mavlink_udp_port', row['simulator_udp_port']),
                               ('sdk_udp_port', row['mavros_local_port'])]:
                node = plugin.find(key)
                if node is None:
                    raise ValueError('Missing MAVLink field: ' + key)
                node.text = str(value)
    for parent in root.findall('.//parent'):
        if parent.text == 'typhoon_h480::base_link':
            parent.text = 'base_link'
    if sensor_signature(root) != before:
        raise AssertionError('Sensor parameters changed during model adaptation')
    return ET.tostring(root, encoding='unicode')


def generate(source_path, wiring, overlay, output):
    if wiring.get('schema_version') != 1 or len(wiring.get('uavs', [])) != 6:
        raise ValueError('Expected six-aircraft wiring v1')
    for name in ('librobocup_legacy_model_gps_plugin.so', 'librobocup_legacy_gps_mavlink_interface.so'):
        if not (Path(overlay) / name).is_file():
            raise ValueError('Missing runtime overlay: ' + name)
    source = Path(source_path).read_text()
    copies = {row['model_name']: adapt(source, row, overlay) for row in wiring['uavs']}
    if len(copies) != 6:
        raise ValueError('Duplicate model names')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    hashes = {}
    for name, text in copies.items():
        path = output / (name + '.sdf')
        path.write_text(text)
        hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    report = dict(schema_version=1, source_sha256=hashlib.sha256(Path(source_path).read_bytes()).hexdigest(),
                  generated_sha256=hashes, sensor_parameters_preserved=True,
                  competition_equivalence_verified=False)
    (output / 'model_manifest.json').write_text(json.dumps(report, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-sdf', required=True)
    parser.add_argument('--wiring', required=True)
    parser.add_argument('--gps-overlay', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    print(json.dumps(generate(args.source_sdf, json.loads(Path(args.wiring).read_text()),
                              args.gps_overlay, args.output), indent=2))
