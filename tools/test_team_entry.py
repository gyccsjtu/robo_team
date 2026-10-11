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
            env = dict(os.environ, RUN_DIR=td, MATCH_HARD_CAP='0')
            # Source only declarations, then replace every side-effect command.
            script = SCRIPT + '''
TEAM_ONLY=1
REPO_ROOT="''' + str(ROOT) + '''"
COORD_WS_SETUP="$RUN_DIR/setup.bash"
: > "$COORD_WS_SETUP"
REG="$RUN_DIR/components.tsv"
LOGDIR="$RUN_DIR/test"
team_preflight(){ :; }
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
            runner = Path(td)/'runner.sh'
            runner.write_text(script)
            completed = subprocess.run(['bash', str(runner)], env=env, cwd=str(ROOT),
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

    def test_team_stop_only_uses_registered_groups(self):
        output = self.run_shell('''
printf 'swarm_manager\t999998\tx\ncomm_2\t999997\ty\n' > "$REG"
do_stop
''')
        stopped = [line[5:] for line in output.splitlines() if line.startswith('STOP:')]
        self.assertEqual(stopped, ['comm_2','swarm_manager'])


if __name__ == '__main__':
    unittest.main()
