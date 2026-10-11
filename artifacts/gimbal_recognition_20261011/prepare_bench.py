"""Build an isolated recognition benchmark; never overwrite official assets."""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
SOURCE = Path('/home/gycc/PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world')
source = ET.parse(str(SOURCE)).getroot().find('world')
root = ET.Element('sdf', version='1.6')
world = ET.SubElement(root, 'world', name='gimbal_recognition_bench')
for tag in ('physics', 'scene', 'light', 'gravity', 'magnetic_field', 'atmosphere'):
    item = source.find(tag)
    if item is not None:
        world.append(copy.deepcopy(item))
for name, pose in [('ground_plane', None), ('house_1_66', '32 0 0 0 0 0'),
                   ('casual_female', '16 4 0 0 0 3.14159265'),
                   ('fire_hydrant_81', '16 -10 0.15 0 0 0')]:
    item = source.find("model[@name='%s']" % name)
    if item is None:
        raise RuntimeError('missing official asset: ' + name)
    model = copy.deepcopy(item)
    if pose is not None:
        model.find('pose').text = pose
    world.append(model)
colors = ['green', 'blue', 'brown', 'white', 'red', 'red']
actors = {}
for i in range(6):
    actor = ET.SubElement(world, 'actor', name='actor_%d' % i)
    original = source.find("actor[@name='actor_%d']" % i)
    for child in original:
        if child.tag in ('skin', 'animation', 'plugin', 'script'):
            actor.append(copy.deepcopy(child))
    y = -7.5 + 1.8 * i
    actor.find("plugin[@filename='libros_actor_cmd_pose_plugin.so']/init_pose").text = '16 %s 1.25 1.57 0 0' % y
    actors['actor_%d' % i] = {'color': colors[i], 'x': 16, 'y_center': y}
gui = ET.SubElement(world, 'gui')
camera = ET.SubElement(gui, 'camera', name='user_camera')
ET.SubElement(camera, 'pose').text = '3 -15 10 0 0.55 0.8'
ET.ElementTree(root).write(str(ROOT / 'recognition_bench.world'), encoding='utf-8', xml_declaration=True)
launch = ET.parse(str(ROOT.parents[1] / 'launch/robocup_with_laser.launch'))
lr = launch.getroot()
ET.SubElement(lr, 'param', name='/recognition_bench/isolated', value='true', type='bool')
for group in list(lr.findall('group')):
    if group.get('ns') != 'typhoon_h480_0':
        lr.remove(group)
lr.find("arg[@name='world']").set('default', str(ROOT / 'recognition_bench.world'))
launch.write(str(ROOT / 'recognition_bench.launch'), encoding='utf-8', xml_declaration=True)
manifest = {
    'scope': 'Controlled recognition benchmark, NOT competition run or elimination proof',
    'source_world': str(SOURCE),
    'source_world_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    'actors': actors,
    'motion': 'Open-loop lateral walking +/-0.75 m about each center; official plugin unchanged',
    'cameras': 'Unmodified native stereo and CGO3 from typhoon_h480_laser',
    'flight': 'Single aircraft, own MAVROS pose only; empty corridor; land after sampling',
    'decoys': {'casual_female': [16, 4], 'fire_hydrant_81': [16, -10]},
}
(ROOT / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Created isolated world and single-aircraft GUI launch:', ROOT)
