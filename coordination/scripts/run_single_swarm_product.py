#!/usr/bin/env python3
"""One command to start, fly, independently measure and stop our simulation."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import hashlib
import shutil
import fcntl

ROOT = Path(__file__).resolve().parents[2]
ap = argparse.ArgumentParser()
ap.add_argument('--output', required=True)
ap.add_argument('--goal', nargs=2, type=float, default=[8.,-10.])
ap.add_argument('--timeout', type=float, default=180.)
ap.add_argument('--master-port', type=int, default=11345)
ap.add_argument('--gazebo-port', type=int, default=11346)
args = ap.parse_args()
flight_lock = open('/tmp/robo_team_px4_instance0.lock', 'a+')
try:
    fcntl.flock(flight_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    raise SystemExit('Another owning runner holds PX4 instance 0')
out = Path(args.output).resolve()
out.mkdir(parents=True, exist_ok=False)
source = out/'source_snapshot'
for relative in ('coordination/src/robocup_swarm/scripts', 'coordination/src/robocup_navigation/src'):
    shutil.copytree(ROOT/relative, source/relative,
                    ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '*.bak*'))
for relative in ('coordination/scripts/start_single_radar_stack.sh',
                 'coordination/scripts/record_sim_truth.py', 'coordination/scripts/single_swarm_flight.py',
                 'coordination/src/robocup_training_worlds/worlds/generated/robocup_base.json'):
    dest = source/relative
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(ROOT/relative, dest)
source_sha = {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in source.rglob('*') if p.is_file()}
(out/'source_manifest.json').write_text(json.dumps(dict(source_sha256=source_sha,
    repository_commit=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
    execution_uses_snapshot=True),indent=2))
env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d'%args.master_port,
           GAZEBO_MASTER_URI='http://127.0.0.1:%d'%args.gazebo_port,
           MASTER_PORT=str(args.master_port), GAZEBO_PORT=str(args.gazebo_port),
           RADAR_STACK_OUT=str(out/'stack'))
env.update(ROBOCUP_RUN_ID=str(uuid.uuid4()), ROBOCUP_LOG_DIR=str(out/'algorithm'),
           ROBOCUP_WS=str(source/'coordination'),
           ROBOCUP_METADATA=str(source/'coordination/src/robocup_training_worlds/worlds/generated/robocup_base.json'),
           VIS_ENABLE='0', SEED_TRUTH='0', RADAR_GUARD='1',
           SWARM_MAX_SPEED='1.0', SWARM_ARRIVE_TOL='0.15',
           ALT_BASE='4.5', ALT_NLAYER='1', ALT_TARGET_CAP='4.5')
env.pop('SWARM_CALIB', None)
env['PYTHONPATH'] = str(source/'coordination/src/robocup_navigation/src') + os.pathsep + env.get('PYTHONPATH','')
children = []
stack_ids = {}
exit_status = 1


def identity(pid):
    try:
        return Path('/proc/%d/stat'%pid).read_text().split(')')[-1].split()[19]
    except (OSError, IndexError):
        return None


def spawn(command, name):
    log = (out/(name+'.log')).open('w')
    proc = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    children.append((proc, log))
    return proc


try:
    print('Starting isolated stack; output='+str(out), flush=True)
    stack = spawn(['bash',str(source/'coordination/scripts/start_single_radar_stack.sh')], 'startup')
    try:
        rc = stack.wait(timeout=180.)
    finally:
        latest = out/'stack/LATEST'
        if latest.exists():
            run = Path(latest.read_text().strip())
            pidfile = run/'pids.txt'
            if pidfile.exists():
                for line in pidfile.read_text().splitlines():
                    pid = int(line)
                    birth = identity(pid)
                    if birth:
                        stack_ids[pid] = birth
    if rc:
        raise RuntimeError('STACK_START_FAILED_'+str(rc))
    audit = spawn(['/usr/bin/python3',str(source/'coordination/scripts/record_sim_truth.py'),
                   '--output',str(out/'truth.jsonl')], 'audit')
    agent = spawn(['/usr/bin/python3','-u',str(source/'coordination/src/robocup_swarm/scripts/swarm_agent.py'),
                   '__name:=swarm_agent_1', '_uav_id:=uav_1', '_model_name:=typhoon_h480_0',
                   '_mavros_namespace:=/typhoon_h480_0/mavros', '_scan_topic:=/typhoon_h480_0/scan',
                   '_world_offset_x:=0.0', '_world_offset_y:=-3.0'], 'agent')
    pilot = spawn(['/usr/bin/python3','-u',str(source/'coordination/scripts/single_swarm_flight.py'),
                   '--output',str(out/'mission'),'--goal',*map(str,args.goal),
                   '--timeout',str(args.timeout)], 'pilot')
    print('Flight running; pilot='+str(pilot.pid), flush=True)
    deadline = time.monotonic()+args.timeout*3+180
    result_path = out/'mission/result.json'
    while not result_path.exists():
        if pilot.poll() is not None:
            raise RuntimeError('PILOT_EXIT_WITHOUT_RESULT')
        if audit.poll() is not None:
            raise RuntimeError('AUDITOR_EXITED')
        if agent.poll() is not None:
            raise RuntimeError('AGENT_EXITED')
        if time.monotonic()>deadline:
            raise RuntimeError('WALL_TIMEOUT')
        time.sleep(.25)
    result = json.loads(result_path.read_text())
    time.sleep(2.)  # terminal hold, with a live heartbeat and independent poses
    rows = [json.loads(line) for line in (out/'truth.jsonl').read_text().splitlines()]
    if not rows:
        raise RuntimeError('NO_INDEPENDENT_TRAJECTORY')
    pos = rows[-1]['position']
    report = dict(controller=result, independent_final_position=pos,
        independent_goal_error_xy_m=math.dist(pos[:2],args.goal),
        independent_final_speed_mps=math.dist(rows[-1]['velocity'],[0.,0.,0.]),
        max_true_altitude_m=max(r['position'][2] for r in rows),
        sample_count=len(rows),
        max_sample_gap_s=max((b['sim_s']-a['sim_s'] for a,b in zip(rows,rows[1:])),default=0.),
        collision_verdict='ABSTAIN_NO_COMPLETE_CONTACT_EVIDENCE',
        formal_competition_pass=False)
    report['physical_aircraft'] = 1
    report['fixture_only'] = True
    report['map_source'] = 'stored development metadata; scene equivalence not verified'
    report['source_sha256'] = source_sha
    events = [json.loads(line) for line in (out/'algorithm/authority_events.jsonl').read_text().splitlines()]
    report['stopped_ack_count'] = sum(e['event']=='STOPPED_ACKED' for e in events)
    report['independent_phase_errors_xy_m'] = [
        math.dist(min(rows,key=lambda r:abs(r['sim_s']-phase['sim_s']))['position'][:2],
                  [8.,-3.] if phase['phase']==0 else args.goal)
        for phase in result.get('phases', [])]
    report['prototype_arrival_verified'] = (
        result['status']=='SINGLE_SWARM_TASKS_REACHED' and
        report['independent_goal_error_xy_m']<=.6 and
        report['independent_final_speed_mps']<=.15 and
        report['max_true_altitude_m']<6. and
        report['stopped_ack_count']>=1 and len(report['independent_phase_errors_xy_m'])==2 and
        max(report['independent_phase_errors_xy_m'])<=.6)
    (out/'audit_report.json').write_text(json.dumps(report,indent=2))
    exit_status = 0 if report['prototype_arrival_verified'] else 1
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2),flush=True)
except Exception as exc:
    if not (out/'audit_report.json').exists():
        (out/'audit_report.json').write_text(json.dumps(dict(status='FAILED',reason=str(exc),
            formal_competition_pass=False,prototype_arrival_verified=False),indent=2))
    raise
finally:
    # Snapshot descendants of the verified roots before terminating them.
    table = subprocess.check_output(['ps','-eo','pid=,ppid='],text=True)
    parents = {int(a):int(b) for a,b in (line.split() for line in table.splitlines())}
    owned = {p for p,birth in stack_ids.items() if identity(p)==birth}
    for _ in range(8):
        owned.update(p for p,parent in parents.items() if parent in owned)
    births = {p:identity(p) for p in owned}
    for proc,log in reversed(children):
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGTERM)
        log.close()
    for pid,birth in births.items():
        if birth and identity(pid)==birth:
            try: os.kill(pid,signal.SIGTERM)
            except ProcessLookupError: pass
    time.sleep(1.)
    for pid,birth in births.items():
        if birth and identity(pid)==birth:
            try: os.kill(pid,signal.SIGKILL)
            except ProcessLookupError: pass
    flight_lock.close()
raise SystemExit(exit_status)
