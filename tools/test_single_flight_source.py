"""Exercise inert entry selection and the real cross-topic FCU owner audit."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import audit_match_graph as audit
from test_audit_match_graph import valid_graph


class SingleFlightSourceTests(unittest.TestCase):
    def bridge_stage(self, swarm, team=False, existing=False, bridge_present=True):
        source = (ROOT / 'run_match.sh').read_text(encoding='utf8')
        begin = source.index('    # ---------------- ② ')
        end = source.index('    # PYTHONUNBUFFERED=1', begin)
        # Run the actual conditional inside a function; every command is inert.
        harness = '''set -e
step(){ :; }
info(){ :; }
warn(){ :; }
sleep(){ :; }
partial_fail(){ return 90; }
start_group(){ printf 'START:%s\\n' "$1"; }
timeout(){
    if [ "$FIXTURE_EXISTING" = 1 ]; then
        for i in 0 1 2 3 4 5; do echo "/typhoon_h480_${i}_communication"; done
    fi
    return 0
}
bridge_stage(){
''' + source[begin:end] + '\n}\nbridge_stage\n'
        with tempfile.TemporaryDirectory() as directory:
            bridge = Path(directory) / 'never_execute_bridge.py'
            if bridge_present:
                bridge.write_text('raise RuntimeError("must never execute")\n')
            env = dict(os.environ, REQUIRE_SWARM=str(int(swarm)), TEAM_ONLY=str(int(team)),
                       FIXTURE_EXISTING=str(int(existing)), COMM_BRIDGE=str(bridge), LOGDIR=directory)
            result = subprocess.run(['bash', '-c', harness], env=env,
                                    capture_output=True, text=True, timeout=5.)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return [line[6:] for line in result.stdout.splitlines() if line.startswith('START:')]

    def test_swarm_entry_never_starts_constant_raw_bridges(self):
        for team in (False, True):
            with self.subTest(team=team):
                self.assertEqual(self.bridge_stage(swarm=True, team=team), [])

    def test_explicit_legacy_entry_still_starts_six_bridges(self):
        self.assertEqual(self.bridge_stage(swarm=False), ['comm_%d' % i for i in range(6)])

    def test_legacy_team_reuses_registered_bridges(self):
        self.assertEqual(self.bridge_stage(swarm=False, team=True, existing=True), [])

    def test_missing_bridge_does_not_start_any_process(self):
        self.assertEqual(self.bridge_stage(swarm=False, bridge_present=False), [])

    def test_own_agent_is_allowed_on_each_mavros_input(self):
        state, types = valid_graph()
        for index in range(6):
            for suffix in ('setpoint_position/local', 'setpoint_velocity/cmd_vel', 'setpoint_raw/local'):
                state[0].append(['/typhoon_h480_%d/mavros/%s' % (index, suffix),
                                 ['/swarm_agent_%d' % index]])
        self.assertEqual(audit.audit_graph(state, types), [])

    def test_raw_bridge_competes_even_when_position_topic_has_one_owner(self):
        for index in range(6):
            with self.subTest(index=index):
                state, types = valid_graph()
                state[0].extend([
                    ['/typhoon_h480_%d/mavros/setpoint_position/local' % index,
                     ['/swarm_agent_%d' % index]],
                    ['/typhoon_h480_%d/mavros/setpoint_raw/local' % index,
                     ['/typhoon_h480_%d_communication' % index]],
                ])
                failures = audit.audit_graph(state, types)
                self.assertEqual(len(failures), 1)
                self.assertIn('competing FCU setpoints', failures[0])
                self.assertIn('setpoint_raw/local', failures[0])

    def test_foreign_writer_rejected_on_all_three_input_channels(self):
        for suffix in ('setpoint_position/local', 'setpoint_velocity/cmd_vel', 'setpoint_raw/local'):
            with self.subTest(suffix=suffix):
                state, types = valid_graph()
                state[0].append(['/typhoon_h480_0/mavros/' + suffix,
                                 ['/swarm_agent_0', '/other_controller']])
                failures = audit.audit_graph(state, types)
                self.assertEqual(len(failures), 1)
                self.assertIn('/other_controller publishes competing FCU', failures[0])

    def test_wrong_uav_agent_is_not_an_authorized_writer(self):
        state, types = valid_graph()
        state[0].append(['/typhoon_h480_0/mavros/setpoint_position/local', ['/swarm_agent_1']])
        failures = audit.audit_graph(state, types)
        self.assertEqual(len(failures), 1)
        self.assertIn('expected /swarm_agent_0', failures[0])


if __name__ == '__main__':
    unittest.main()
