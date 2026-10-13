"""Exercise orchestration with inert commands: never start ROS or Gazebo."""
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = (ROOT/'run_match.sh').read_text().split('# ============================ 入口')[0]


class TeamEntryTests(unittest.TestCase):
    def run_shell(self, body):
        with tempfile.TemporaryDirectory() as td:
            env = dict(os.environ, RUN_DIR=td, MATCH_HARD_CAP='0',
                       TEAM_TIME_SOURCE='mavros_header', CLOCK_PERMISSION='allowed')
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
            for source in ('clock', 'wall', 'clock_relay', 'invalid'):
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

    def test_formal_start_forces_no_clock_and_agent_own_namespace(self):
        output = self.run_shell('''
echo "CLOCK:$CLOCK_PERMISSION TIME:$TEAM_TIME_SOURCE"
start_group(){ printf 'COMMAND:%s\\n' "$*"; }
do_start
''')
        self.assertIn('CLOCK:forbidden TIME:mavros_header', output)
        for index in range(6):
            self.assertIn("TEAM_TIME_UAVS='typhoon_h480_%d'" % index, output)

    def test_team_stop_only_uses_registered_groups(self):
        output = self.run_shell('''
printf 'swarm_manager\t999998\tx\ncomm_2\t999997\ty\n' > "$REG"
do_stop
''')
        stopped = [line[5:] for line in output.splitlines() if line.startswith('STOP:')]
        self.assertEqual(stopped, ['comm_2','swarm_manager'])


if __name__ == '__main__':
    unittest.main()
