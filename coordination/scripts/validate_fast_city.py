"""Fixed budget or one explicitly requested extra map; reuse the city launcher."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

ROUNDS = {1:(5334,90),2:(2523,300),3:(8159,600)}


def read(path):
    try:return json.loads(path.read_text())
    except (OSError,ValueError):return None


def write(path,value):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value,indent=2,allow_nan=False));tmp.replace(path)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',required=True)
    selection=ap.add_mutually_exclusive_group(required=True)
    selection.add_argument('--round',type=int,choices=ROUNDS)
    selection.add_argument('--single-seed',type=int,help='One separately authorized 600-second map, in a new budget')
    args=ap.parse_args();root=Path(args.root);root.mkdir(parents=True,exist_ok=True)
    single=args.single_seed is not None;maximum=1 if single else 3
    repo=Path(__file__).resolve().parents[2];index=1 if single else args.round
    seed,seconds=(args.single_seed,600) if single else ROUNDS[index]
    if seed < 0:ap.error('seed must be nonnegative')
    physics_rate=float(os.environ.get('CITY_PHYSICS_RATE','40'))
    if not math.isfinite(physics_rate) or not 1 <= physics_rate <= 250:
        ap.error('CITY_PHYSICS_RATE must be finite and between 1 and 250')
    budget=root/'budget.json';state=read(budget) or dict(schema_version=1,maximum_attempts=maximum,attempts=[])
    if state['maximum_attempts'] != maximum:raise RuntimeError('BUDGET_MODE_MISMATCH')
    if len(state['attempts']) >= maximum or any(a['index']==index for a in state['attempts']):
        raise RuntimeError('PHYSICAL_ATTEMPT_ALREADY_USED')
    if index != len(state['attempts'])+1 or any(a['status']=='RUNNING' for a in state['attempts']):
        raise RuntimeError('ATTEMPTS_MUST_BE_SEQUENTIAL')
    for port in (11375,11376,19731):
        with socket.socket() as sock:
            sock.settimeout(.3)
            if sock.connect_ex(('127.0.0.1',port))==0:raise RuntimeError('EXISTING_PROCESS_ON_PORT_%d'%port)
    out=root/('round_%d_seed_%d'%(index,seed));out.mkdir(exist_ok=False)
    source_sha={str(p.relative_to(repo)):hashlib.sha256(p.read_bytes()).hexdigest()
        for parent in ('coordination/scripts','coordination/src','perception')
        for p in (repo/parent).rglob('*.py') if '__pycache__' not in p.parts}
    write(out/'source_manifest.json',source_sha)
    record=dict(index=index,seed=seed,sim_limit_s=seconds,output=str(out),status='RUNNING',started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
    record['physics_rate_hz']=physics_rate
    if single:record['authorization']='user_requested_one_additional_random_map'
    state['attempts'].append(record);write(budget,state)
    env=dict(os.environ,CITY_SEED=str(seed),CITY_SECONDS=str(seconds),CITY_RUN_ROOT=str(out),
        CITY_PHYSICS_RATE=str(physics_rate),CITY_EXPERIMENTAL_V123='0',PR_SHARED_INFER='1',VISION_DEVICE='0',PR_CLIENT_DEVICE='cpu',
        VISION_PYTHON='/root/robo_team_build/vision_env/bin/python',SHARED_VISION_PYTHON='/root/robo_team_build/vision_cuda_20261003/bin/python',
        PR_SHARED_PORT='19731',ROBOCUP_GIMBAL_RUNTIME='/root/robocup_runtime/gimbal_namespaced_20261004',PYTHONDONTWRITEBYTECODE='1')
    interrupted=[]
    def stop(sig,frame):interrupted.append(sig)
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    contacts=None;offset=0;first_static=None;actor_rows=0;fleet_rows=0;start=time.monotonic();stop_started=None
    with (out/'launcher.log').open('w') as log:
        child=subprocess.Popen(['bash',str(repo/'run_city_swarm.sh')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        record['owned_launcher_pid']=child.pid;write(budget,state)
        print(json.dumps(record),flush=True)
        try:
            while child.poll() is None:
                path=out/'flight/city_contacts.log'
                if path.exists():
                    with path.open() as stream:
                        stream.seek(offset)
                        while True:
                            before=stream.tell();line=stream.readline()
                            if not line or not line.endswith('\n'):stream.seek(before);break
                            try:item=json.loads(line)
                            except ValueError:continue
                            if item.get('kind')!='UAV_BODY_CONTACT':continue
                            names=[item['collision1'],item['collision2']]
                            if any(n.startswith('ground_plane::') for n in names):continue
                            if any(n.startswith('actor_') for n in names):actor_rows+=1
                            elif all(n.startswith('typhoon_h480_') for n in names):fleet_rows+=1
                            elif first_static is None:first_static=item
                        offset=stream.tell()
                config=read(out/'flight/city_control_config.json');scene=read(out/'scene/scene_manifest.json')
                record.update(wall_seconds=round(time.monotonic()-start,1),first_static_contact=first_static,
                    actor_contact_rows=actor_rows,fleet_contact_rows=fleet_rows,run_id=(config or {}).get('run_id'),
                    selected_seed=(scene or {}).get('seed'))
                write(budget,state)
                reason=('CONFIRMED_STATIC_BODY_CONTACT' if first_static else 'USER_INTERRUPTED' if interrupted
                        else 'WALL_TIMEOUT' if time.monotonic()-start > seconds*8+600 else None)
                if reason and stop_started is None:
                    command=Path('/proc/%d/cmdline'%child.pid).read_bytes().replace(b'\0',b' ').decode()
                    if str(out) not in command and str(repo/'run_city_swarm.sh') not in command:
                        raise RuntimeError('OWNED_PID_COMMAND_CHANGED')
                    record['early_stop_reason']=reason
                    write(out/'early_stop.json',dict(reason=reason,pid=child.pid,command=command,first_static_contact=first_static))
                    if first_static:
                        windows={p.name:read(p) for p in (out/'flight/algorithm').glob('navigation_*_latest.json')}
                        write(out/'contact_navigation_windows.json',windows)
                    child.terminate();stop_started=time.monotonic()
                if stop_started is not None and time.monotonic()-stop_started > 60:
                    raise RuntimeError('OWNED_LAUNCHER_CLEANUP_TIMEOUT')
                time.sleep(2)
        except BaseException as error:
            record['runner_error']=dict(type=type(error).__name__,message=str(error))
            raise
        finally:
            try:
                if child.poll() is None:child.terminate()
                try:child.wait(timeout=60)
                except subprocess.TimeoutExpired:record['cleanup_incomplete']=True
            finally:
                result=read(out/'flight/result.json')
                checkpoint=read(out/'flight/result_before_cleanup.json')
                record.update(status='CLEANUP_INCOMPLETE' if child.poll() is None else
                    'STOPPED_AFTER_CONTACT' if first_static else 'ENDED',returncode=child.poll(),
                    wall_seconds=round(time.monotonic()-start,1),raw_result=result,
                    before_cleanup_result=checkpoint)
                write(budget,state)
    print(json.dumps(record),flush=True)


if __name__=='__main__':main()
