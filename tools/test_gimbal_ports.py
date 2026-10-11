"""Check per-instance gimbal routing using ROS launch's real substitution parser."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from roslaunch.substitution_args import resolve_args


class GimbalPortTests(unittest.TestCase):
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
