#!/usr/bin/env python3
"""Offline-only native-camera dataset capture. Never imported by match nodes.

Generate an isolated world with ``world``; start that world with gazebo_ros
and GUI; then ``capture`` samples it. Raw frames are UNREVIEWED proposals.
The collection refuses a world containing any competition UAV.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET


def make_world(official_world, camera_sdf, output):
    tree = ET.parse(official_world)
    world = tree.getroot().find('world')
    world.set('name', 'distractor_training_only')
    for child in list(world):
        if child.tag in ('state', 'actor', 'plugin'):
            world.remove(child)
    camera = copy.deepcopy(ET.parse(camera_sdf).getroot().find('model'))
    camera.set('name', 'distractor_training_camera')
    ET.SubElement(camera, 'pose').text = '-18.902 13.6572 3.3 0 0.30 3.14159265'
    link = camera.find('link')
    ET.SubElement(link, 'gravity').text = 'false'
    ET.SubElement(link, 'kinematic').text = 'true'
    # Optics/image dimensions/rate/distortion stay exactly as in the input SDF.
    # gazebo_ros fills the empty robotNamespace with the model name.
    camera.find('.//plugin/cameraName').text = '/stereo_camera'
    world.append(camera)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(output), encoding='utf-8', xml_declaration=True)
    return output


def viewpoints(center, small=False, distances=None, heights=None, angles=None):
    angles = angles or ([0, 90, 180, 270] if small else list(range(0, 360, 30)))
    for distance in (distances or ([8.] if small else [4., 6., 8., 12., 16.])):
        for height in (heights or ([3.3] if small else [2.6, 3.5, 4.2])):
            for degrees in angles:
                angle = math.radians(degrees)
                x = center[0] + distance * math.cos(angle)
                y = center[1] + distance * math.sin(angle)
                # Sensor pitch changes through physical camera pose only.
                pitch = math.atan2(height - 0.9, distance)
                yield dict(x=x, y=y, z=height, yaw=angle+math.pi,
                           pitch=pitch, distance=distance, azimuth=degrees)


def capture(args):
    import cv2
    import numpy as np
    import rospy
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import GetModelState, GetWorldProperties, SetModelState
    from sensor_msgs.msg import Image
    from tf.transformations import quaternion_from_euler
    rospy.init_node('offline_distractor_capture', anonymous=True)
    rospy.wait_for_service('/gazebo/get_world_properties', timeout=20)
    props = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)()
    if ('distractor_training_camera' not in props.model_names or
            any('typhoon' in x or 'iris' in x for x in props.model_names)):
        raise RuntimeError('Refusing collection outside the isolated training world')
    get_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    decoy = get_pose('casual_female', 'world')
    if not decoy.success:
        raise RuntimeError('Official casual_female not found')
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    manifest = out/'raw_frames.jsonl'
    if manifest.exists():
        raise ValueError('Use a new output directory for each capture session')
    set_pose = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    topic = args.topic
    with manifest.open('w') as f:
        for i, view in enumerate(viewpoints((decoy.pose.position.x, decoy.pose.position.y), args.sample,
                                            args.distances, args.heights, args.angles)):
            state = ModelState(model_name='distractor_training_camera', reference_frame='world')
            state.pose.position.x, state.pose.position.y, state.pose.position.z = view['x'], view['y'], view['z']
            q = quaternion_from_euler(0., view['pitch'], view['yaw'])
            state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w = q
            result = set_pose(state)
            if not result.success:
                raise RuntimeError(result.status_message)
            changed_s = rospy.Time.now().to_sec()
            time.sleep(.35)
            # Reject a frame captured before movement; wall sleep alone is insufficient.
            for _ in range(20):
                msg = rospy.wait_for_message(topic, Image, timeout=5)
                if msg.header.stamp.to_sec() > changed_s+.15:
                    break
            else:
                raise RuntimeError('No fresh rendered camera frame')
            if msg.encoding not in ('rgb8', 'bgr8'):
                raise ValueError('Unexpected image encoding '+msg.encoding)
            im = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
            im = im[:, :msg.width*3].reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
            path = out/('raw_%04d.png' % i)
            if not cv2.imwrite(str(path), im):
                raise IOError('Image write failed')
            record = dict(image=str(path), group=args.group, split=args.split,
                          sample_s=msg.header.stamp.to_sec(), wall_s=time.time(),
                          camera_pose=view, reviewed=False,
                          sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            f.write(json.dumps(record)+'\n'); f.flush()
            print('captured', i, path, flush=True)


def layout(args):
    """Training-world-only placement; no flight code imports this utility."""
    import rospy
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SetModelState, SpawnModel
    from geometry_msgs.msg import Pose
    from tf.transformations import quaternion_from_euler
    rospy.init_node('offline_training_layout', anonymous=True)
    props = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)()
    if ('distractor_training_camera' not in props.model_names or
            any('typhoon' in x or 'iris' in x for x in props.model_names)):
        raise RuntimeError('Refusing layout changes outside isolated training world')
    x, y = args.center
    state = ModelState(model_name='casual_female', reference_frame='world')
    state.pose.position.x, state.pose.position.y = x, y
    q = quaternion_from_euler(0, 0, args.yaw)
    state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w = q
    result = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)(state)
    if not result.success:
        raise RuntimeError(result.status_message)
    # The official actor mesh/plugin is copied, never patched in its source.
    # No cmd_motion is published; these are deliberately stationary true actors.
    world = ET.parse(args.official_world).getroot().find('world')
    records = [dict(name='casual_female', class_id=5, x=x, y=y, yaw=args.yaw)]
    for idx in range(5):
        name = 'actor_%d' % idx
        if name in props.model_names:
            rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)(name)
        source = world.find("actor[@name='%s']" % name)
        actor = copy.deepcopy(source)
        angle = args.yaw+2*math.pi*idx/5
        ax, ay = x+args.radius*math.cos(angle), y+args.radius*math.sin(angle)
        for plugin in list(actor.findall('plugin')):
            if plugin.get('filename') != 'libros_actor_cmd_pose_plugin.so':
                actor.remove(plugin)
        actor.find('plugin/init_pose').text = '%f %f 1.0191 1.57 0 %f' % (ax, ay, args.yaw)
        sdf = ET.Element('sdf', version='1.6'); sdf.append(actor)
        pose = Pose(); pose.orientation.w = 1
        result = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)(
            name, ET.tostring(sdf, encoding='unicode'), '', pose, 'world')
        if not result.success:
            raise RuntimeError(result.status_message)
        records.append(dict(name=name, class_id=[1,2,4,3,0][idx], x=ax, y=ay, yaw=args.yaw))
    Path(args.output).write_text(json.dumps(records, indent=2)+'\n')
    print(json.dumps(records, indent=2))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    w = sub.add_parser('world')
    w.add_argument('--official-world', required=True)
    w.add_argument('--camera-sdf', required=True)
    w.add_argument('--output', required=True)
    c = sub.add_parser('capture')
    c.add_argument('--output', required=True)
    c.add_argument('--group', required=True)
    c.add_argument('--split', choices=['train', 'val', 'test'], required=True)
    c.add_argument('--sample', action='store_true')
    c.add_argument('--topic', default='/distractor_training_camera/stereo_camera/left/image_raw')
    c.add_argument('--distances', type=float, nargs='+')
    c.add_argument('--heights', type=float, nargs='+')
    c.add_argument('--angles', type=float, nargs='+')
    l = sub.add_parser('layout')
    l.add_argument('--official-world', required=True)
    l.add_argument('--center', type=float, nargs=2, required=True)
    l.add_argument('--yaw', type=float, default=0.)
    l.add_argument('--radius', type=float, default=2.5)
    l.add_argument('--output', required=True)
    args = p.parse_args()
    if args.action == 'world':
        print(make_world(args.official_world, args.camera_sdf, args.output))
    elif args.action == 'capture':
        capture(args)
    else:
        layout(args)


if __name__ == '__main__':
    main()
