"""Classify fixture contacts without confusing sonar sensing cones with bodies."""
import xml.etree.ElementTree as ET
from pathlib import Path


def sonar_virtual_collisions(model_root, fleet):
    path = Path(model_root)/'sonar/model.sdf'
    root = ET.parse(path).getroot()
    model = root.find('model')
    if model is None or model.get('name') != 'sonar':
        raise ValueError('Unverified sonar model')
    link = model.find('link')
    sensor = link.find('sensor') if link is not None else None
    if (link is None or link.get('name') != 'link' or link.findall('collision')
            or sensor is None or sensor.get('name') != 'sonar' or sensor.get('type') != 'sonar'):
        raise ValueError('Unverified sonar sensing shape')
    # Gazebo11 SonarSensor.cc: ScopedName()+"sensor_collision", SENSOR_COLLISION,
    # collideWithoutContact=true. Preserve exact observed nested scope names.
    return {uid+'::sonar::link::default::'+uid+'::sonar::link::sonarsensor_collision' for uid in fleet}


def classify(collision1, collision2, fleet, virtual):
    if collision1 in virtual or collision2 in virtual:
        return 'SONAR_SENSOR_INTERSECTION'
    if any(uid+'::' in collision1 or uid+'::' in collision2 for uid in fleet):
        return 'UAV_BODY_CONTACT'
    return 'OTHER_CONTACT'
