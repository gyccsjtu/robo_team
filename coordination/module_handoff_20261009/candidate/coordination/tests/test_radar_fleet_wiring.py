import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('wiring', Path(__file__).parents[1] / 'scripts/prepare_radar_fleet.py')
wiring = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wiring)


class FleetWiringTests(unittest.TestCase):
    def prepare(self, directory, local=14580, remote=14540):
        init = Path(directory) / 'etc/init.d-posix'
        init.mkdir(parents=True)
        (init / 'px4-rc.mavlink').write_text(
            'udp_offboard_port_local=$((%d+px4_instance))\n'
            'udp_offboard_port_remote=$((%d+px4_instance))\n'
            'udp_onboard_gimbal_port_local=$((13030+px4_instance))\n'
            'udp_onboard_gimbal_port_remote=$((13280+px4_instance))\n'
            'mavlink start -u $udp_offboard_port_local -m onboard -o $udp_offboard_port_remote\n' % (local, remote))
        (init / 'px4-rc.simulator').write_text('simulator_tcp_port=$((4560+px4_instance))\n')
        (init / 'rcS').write_text('param set MAV_SYS_ID $((px4_instance+1))\n')
        return init

    def test_actual_build_mapping_and_instance_zero_isolation(self):
        with tempfile.TemporaryDirectory() as directory:
            self.prepare(directory)
            result = wiring.derive(directory)
            self.assertEqual(result['uavs'][0]['fcu_url'], 'udp://:14541@127.0.0.1:14581')
            self.assertEqual(result['uavs'][-1]['simulator_tcp_port'], 4566)
            self.assertEqual([r['mavlink_system_id'] for r in result['uavs']], list(range(2, 8)))
            self.assertFalse(result['flight_ready'])
            self.assertEqual(result['schema_version'],2)
            self.assertEqual([r['px4_gimbal_port'] for r in result['uavs']],list(range(13031,13037)))
            self.assertEqual([r['gimbal_local_port'] for r in result['uavs']],list(range(13281,13287)))

    def test_gimbal_ports_follow_actual_px4_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            init=self.prepare(directory)
            p=init/'px4-rc.mavlink'
            p.write_text(p.read_text().replace('13030','33030').replace('13280','33280'))
            result=wiring.derive(directory)
            self.assertEqual(result['uavs'][0]['px4_gimbal_port'],33031)
            self.assertEqual(result['uavs'][-1]['gimbal_local_port'],33286)

    def test_gimbal_collision_with_flight_link_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            init=self.prepare(directory)
            p=init/'px4-rc.mavlink';p.write_text(p.read_text().replace('13280','14580'))
            with self.assertRaises(ValueError):wiring.derive(directory)

    def test_missing_gimbal_wiring_is_not_guessed(self):
        with tempfile.TemporaryDirectory() as directory:
            init=self.prepare(directory)
            p=init/'px4-rc.mavlink';p.write_text(p.read_text().replace('udp_onboard_gimbal_port_remote=$((13280+px4_instance))\n',''))
            with self.assertRaises(ValueError):wiring.derive(directory)

    def test_patched_build_uses_its_actual_ports(self):
        with tempfile.TemporaryDirectory() as directory:
            self.prepare(directory, 34580, 24540)
            self.assertEqual(wiring.derive(directory)['uavs'][0]['px4_local_port'], 34581)

    def test_overlapping_endpoints_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            self.prepare(directory, 14580, 14580)
            with self.assertRaises(ValueError):
                wiring.derive(directory)

    def test_unsupported_sysid_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            init = self.prepare(directory)
            (init / 'rcS').write_text('param set MAV_SYS_ID 1\n')
            with self.assertRaises(ValueError):
                wiring.derive(directory)
