"""Exercise orchestration with inert commands: never start ROS or Gazebo."""
import os
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = (ROOT/'run_match.sh').read_text().split('# ============================ 入口')[0]


class TeamEntryTests(unittest.TestCase):
    def run_shell(self, body):
        with tempfile.TemporaryDirectory() as td:
            env = dict(os.environ, RUN_DIR=td, MATCH_HARD_CAP='0',
                       TEAM_TIME_SOURCE='ros_clock', CLOCK_PERMISSION='allowed',
                       CLOCK_PERMISSION_REFERENCE='User confirmed native /clock on 2026-10-13; test fixture')
            # Source only declarations, then replace every side-effect command.
            script = SCRIPT + '''
TEAM_ONLY=1
REPO_ROOT="''' + str(ROOT) + '''"
COORD_WS_SETUP="$REPO_ROOT/coordination/devel/setup.bash"
REG="$RUN_DIR/components.tsv"
LOGDIR="$RUN_DIR/test"
team_preflight(){ :; }
sensor_inventory_check(){
    printf '{"counts":{"mono_cam":6,"stereo_cam":6,"laser2d":6,"gimbal":6},"cost":16200}\n' > "$RUN_DIR/sensor_inventory.json"
    echo '_mono_cam:=6 _stereo_cam:=6 _laser1d:=0 _laser2d:=6 _laser3d:=0 _gimbal:=6 _bino_cam:=0'
}
ros_env(){ :; }
start_group(){ echo "START:$1"; }
stop_group(){ echo "STOP:$1"; }
wait_gate(){ :; }
alive_pgid(){ :; }
group_pgid(){ echo 999999; }
python3(){ :; }
rosparam(){ echo "PARAM:$*"; }
sleep(){ :; }
timeout(){
    case "$*" in
        *"rosnode list"*) for i in 0 1 2 3 4 5; do echo "/typhoon_h480_${i}_communication"; done ;;
        *"audit_match_graph.py"*) printf 'AUDIT:%s\\n' "$*" ;;
    esac
    return 0
}
pkill(){ echo FORBIDDEN_PKILL; return 99; }
rosservice(){ echo FORBIDDEN_SERVICE; return 99; }
''' + body
            completed = subprocess.run(['bash', '-c', script], env=env, cwd=str(ROOT),
                                       capture_output=True, text=True, timeout=10.)
            self.assertEqual(completed.returncode, 0, completed.stdout+completed.stderr)
            self.assertNotIn('FORBIDDEN_', completed.stdout)
            return completed.stdout

    def test_team_start_launches_only_team_and_reuses_existing_bridges(self):
        output = self.run_shell('do_start\n')
        started = [line[6:] for line in output.splitlines() if line.startswith('START:')]
        self.assertEqual(len(started), 14)  # six perception, bridge, manager, six agents
        self.assertFalse(set(started) & {'scene','actors','judge','comm_0'})
        self.assertIn('swarm_agent_5', started)
        self.assertNotIn('PARAM:set /score_cal/', output)
        self.assertIn('_mono_cam:=6 _stereo_cam:=6', output)

    def test_invalid_time_source_blocks_before_creating_runtime_or_flight_processes(self):
        for command in ('start', 'start-team'):
            for source in ('mavros_header', 'clock', 'wall', 'clock_relay', 'invalid'):
                for force in ('0', '1'):
                    with self.subTest(command=command, source=source, force=force), \
                            tempfile.TemporaryDirectory() as td:
                        runtime = Path(td)/'not_created'
                        env = dict(os.environ, RUN_DIR=str(runtime), FORCE=force,
                                   TEAM_TIME_SOURCE=source, CLOCK_PERMISSION='allowed')
                        result = subprocess.run(['bash', str(ROOT/'run_match.sh'), command],
                                                cwd=ROOT, env=env, capture_output=True,
                                                text=True, timeout=5)
                        self.assertEqual(result.returncode, 2, result.stdout+result.stderr)
                        self.assertFalse(runtime.exists(), 'Rejected time source must precede startup')
                        self.assertNotIn('已启动', result.stdout)
                        self.assertRegex(result.stderr, '/clock|TEAM_TIME_SOURCE')

    def test_formal_start_uses_native_clock_and_declared_permission_in_both_audits(self):
        output = self.run_shell('''
echo "CLOCK:$CLOCK_PERMISSION TIME:$TEAM_TIME_SOURCE"
echo "COMM:$COMM_BRIDGE"
start_group(){ printf 'COMMAND:%s\\n' "$*"; }
do_start
cat "$LOGDIR/06_preflight_graph_audit.log" "$LOGDIR/07_graph_audit.log"
''')
        self.assertIn('CLOCK:allowed TIME:ros_clock', output)
        self.assertIn('/communication/multirotor_communication.py', output)
        self.assertNotIn('TEAM_TIME_UAVS=', output)
        for index in range(6):
            self.assertIn('swarm_agent.py __name:=swarm_agent_%d' % index, output)
        audits = [line for line in output.splitlines() if line.startswith('AUDIT:')]
        self.assertEqual(len(audits), 2)
        for line in audits:
            self.assertIn('--clock-permission allowed', line)
            self.assertIn('--clock-permission-reference User confirmed native /clock', line)

    def test_forbidden_or_unconfirmed_clock_permission_cannot_be_forced(self):
        configurations = [('forbidden', 'Site forbids /clock'),
                          ('unconfirmed', 'No permission confirmed'),
                          ('', 'Empty status is invalid'), ('invalid', 'Invalid status'),
                          ('allowed', '')]
        for command in ('start', 'start-team'):
            for permission, reference in configurations:
                for force in ('0', '1'):
                    with self.subTest(command=command, permission=permission, force=force), \
                            tempfile.TemporaryDirectory() as td:
                        runtime = Path(td)/'not_created'
                        env = dict(os.environ, RUN_DIR=str(runtime), FORCE=force,
                                   TEAM_TIME_SOURCE='ros_clock', CLOCK_PERMISSION=permission,
                                   CLOCK_PERMISSION_REFERENCE=reference)
                        result = subprocess.run(['bash', str(ROOT/'run_match.sh'), command],
                                                cwd=ROOT, env=env, capture_output=True,
                                                text=True, timeout=5)
                        self.assertEqual(result.returncode, 2, result.stdout+result.stderr)
                        self.assertFalse(runtime.exists())
                        self.assertNotIn('已启动', result.stdout)
                        self.assertIn('/clock', result.stderr)

    def test_default_native_clock_records_actual_user_confirmation(self):
        with tempfile.TemporaryDirectory() as td:
            env = dict(os.environ, RUN_DIR=td)
            for key in ('TEAM_TIME_SOURCE', 'CLOCK_PERMISSION', 'CLOCK_PERMISSION_REFERENCE'):
                env.pop(key, None)
            result = subprocess.run(['bash', '-c', SCRIPT + '\nclock_permission_check\n'],
                                    cwd=ROOT, env=env, capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            self.assertIn('原生 ROS /clock', result.stdout)
            self.assertIn('用户于2026-10-13本次会话确认', result.stdout)

    def test_manifest_keeps_native_time_baseline_and_fleet_inventory(self):
        with tempfile.TemporaryDirectory() as td:
            folder = Path(td)
            inventory = folder/'inventory.json'
            counts = {'mono_cam':6, 'stereo_cam':6, 'laser1d':0, 'laser2d':6,
                      'laser3d':0, 'gimbal':6, 'bino_cam':0}
            inventory.write_text(json.dumps({'counts':counts, 'cost':16200}))
            weights = folder/'weights.pt'
            weights.write_bytes(b'offline manifest fixture')
            output = folder/'manifest.json'
            env = dict(os.environ, PR_WEIGHTS=str(weights),
                       ROBOCUP_SENSOR_INVENTORY=str(inventory), TEAM_TIME_SOURCE='ros_clock',
                       CLOCK_PERMISSION='allowed', CLOCK_PERMISSION_REFERENCE='User confirmation fixture',
                       ROBOCUP_RUNTIME_BASELINE='721927c5c948e22b956f91a7fdec1394e29ade98')
            result = subprocess.run([sys.executable, '-B', str(ROOT/'tools/write_run_manifest.py'),
                                     '--root', str(ROOT), '--official', str(folder),
                                     '--output', str(output), '--started', '123'],
                                    env=env, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            manifest = json.loads(output.read_text())
            self.assertEqual(manifest['team_time']['source'], 'ros_clock')
            self.assertIn('native rospy', manifest['team_time']['business_timers'])
            self.assertEqual(manifest['runtime_baseline']['source_commit'][:7], '721927c')
            self.assertEqual(manifest['clock_permission']['reference'], 'User confirmation fixture')
            self.assertEqual(manifest['sensor_inventory']['counts'], counts)
            self.assertEqual(manifest['sensor_inventory']['cost'], 16200)
            self.assertNotIn('independent monotonic', json.dumps(manifest['team_time']))

    def test_team_stop_only_uses_registered_groups(self):
        output = self.run_shell('''
printf 'swarm_manager\t999998\tx\ncomm_2\t999997\ty\n' > "$REG"
do_stop
''')
        stopped = [line[5:] for line in output.splitlines() if line.startswith('STOP:')]
        self.assertEqual(stopped, ['comm_2','swarm_manager'])


if __name__ == '__main__':
    unittest.main()
