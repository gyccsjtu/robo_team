"""Check per-instance gimbal routing using ROS launch's real substitution parser."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from roslaunch.substitution_args import resolve_args


class GimbalPortTests(unittest.TestCase):
    def test_native_mavros_telemetry_has_a_distinct_legal_topic_for_each_aircraft(self):
        root = Path(__file__).resolve().parents[1]
        tree = ET.parse(root / 'launch/robocup_with_laser.launch')
        groups = tree.findall('group')
        self.assertEqual([group.get('ns') for group in groups],
                         ['typhoon_h480_%d' % uid for uid in range(6)])
        for group in groups:
            remap = group.find("remap[@from='mavlink/from']")
            self.assertIsNotNone(remap)
            self.assertEqual(remap.get('to'), 'mavros/mavlink/from')

    def test_each_vehicle_connects_to_its_own_fcu(self):
        root = Path(__file__).resolve().parents[1]
        tree = ET.parse(root / 'launch/single_vehicle_spawn_serial.launch')
        expression = tree.find("arg[@name='udp_gimbal_port']").get('default')
        ports = [int(resolve_args(expression, context={'arg': {'ID': str(uid)}}))
                 for uid in range(6)]
        self.assertEqual(ports, [13030, 13031, 13032, 13033, 13034, 13035])
        self.assertEqual(len(set(ports)), 6)


if __name__ == '__main__':
    unittest.main()
