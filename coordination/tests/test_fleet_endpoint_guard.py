import importlib.util
from pathlib import Path
import socket
import sys
import unittest

scripts = Path(__file__).parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
spec = importlib.util.spec_from_file_location('connectivity', scripts / 'six_radar_connectivity.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class EndpointTests(unittest.TestCase):
    def test_listening_tcp_rejected_even_with_reuseaddr(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as occupied:
            occupied.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            occupied.bind(('0.0.0.0', 0))
            occupied.listen(1)
            with self.assertRaises(RuntimeError):
                module.check_endpoints({occupied.getsockname()[1]}, set())

    def test_occupied_udp_rejected(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied:
            occupied.bind(('0.0.0.0', 0))
            with self.assertRaises(RuntimeError):
                module.check_endpoints(set(), {occupied.getsockname()[1]})

    def test_tcp_does_not_block_unrelated_udp_on_same_number(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as occupied:
            occupied.bind(('0.0.0.0', 0))
            occupied.listen(1)
            module.check_endpoints(set(), {occupied.getsockname()[1]})
