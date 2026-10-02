"""Verify an existing FCU endpoint is rejected before launching a simulator."""
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import unittest


@unittest.skipIf(sys.platform == 'win32', 'WSL flight launcher')
class PortGuardTests(unittest.TestCase):
    def test_existing_mavros_udp_port_is_not_attached_to_or_killed(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            try:
                sock.bind(('127.0.0.1', 14540))
            except OSError:
                self.skipTest('another existing runtime owns this standard port')
            with tempfile.TemporaryDirectory() as directory:
                script = Path(__file__).resolve().parents[1] / 'scripts/start_single_radar_stack.sh'
                proc = subprocess.run(['bash', str(script)], capture_output=True, text=True,
                                      timeout=10., env=dict(os.environ, RADAR_STACK_OUT=directory))
                self.assertEqual(proc.returncode, 4, proc.stdout + proc.stderr)
                self.assertIn('port occupied', proc.stdout)
                self.assertFalse(any(p.stat().st_size for p in Path(directory).rglob('pids.txt')))
                self.assertEqual(sock.getsockname()[1], 14540)
        finally:
            sock.close()
