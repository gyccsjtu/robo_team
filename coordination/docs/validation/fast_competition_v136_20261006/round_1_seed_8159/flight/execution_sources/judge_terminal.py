"""Run-bound terminal evidence from the isolated, unchanged judge program."""
import json
import math
from pathlib import Path


def write_terminal(path, run_id, judge_sha256, state):
    value=dict(schema_version=1,run_id=run_id,judge_sha256=judge_sha256,
        mission_finished=state.get('mission_finished'),left_actors=state.get('left_actors'),
        target_finish=state.get('target_finish'),score=state.get('score'))
    target=Path(path);temporary=target.with_name(target.name+'.tmp')
    with temporary.open('x') as stream:json.dump(value,stream,allow_nan=False)
    temporary.replace(target)
    return value


def completed(path, run_id, judge_sha256, returncode):
    if type(returncode) is not int or returncode != 0:return None
    try:
        value=json.loads(Path(path).read_text())
        score=value['score']
        if (type(value['schema_version']) is int and value['schema_version']==1
                and value['run_id']==run_id and value['judge_sha256']==judge_sha256
                and value['mission_finished'] is True and value['left_actors']==[]
                and type(value['target_finish']) is int and value['target_finish']==6
                and type(score) in (int,float) and math.isfinite(score) and score>0):
            return value
    except (OSError,ValueError,TypeError,KeyError):pass
    return None
