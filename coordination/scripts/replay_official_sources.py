"""Finite method replay of archived bridge inputs; no ROS, no control output."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parents[1]/'src/robocup_swarm/scripts'))
from report_readiness import ReportReadiness
from official_report_sources import OfficialReportSources
from yolo_target_bridge import TargetBridgeCore


def replay(path):
    gate = ReportReadiness()
    sources = OfficialReportSources(TargetBridgeCore, gate)
    result = dict(input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  scope='accepted flags from archive; finite method replay, not ROS or physical proof',
                  inputs=0, sampled_not_ready=0, restored=[], still_blocked=0)
    with path.open() as stream:
        for line in stream:
            row = json.loads(line)
            now = row.get('receipt_s')
            if row.get('kind') == 'input':
                observation = row['observation']
                gate.observe(observation, now, row['accepted'])
                sources.observe(observation, row['accepted'])
                result['inputs'] += 1
            elif row.get('kind') == 'skip' and row.get('reason') == 'not_ready':
                result['sampled_not_ready'] += 1
                events = [e for e in sources.tick(now) if e[1]['tag'] == row['tag']]
                if events:
                    uid, event, track = events[0]
                    result['restored'].append(dict(receipt_s=now, tag=row['tag'],
                        source_uid=uid, original_s=track.t_obs,
                        xy=[event['x'], event['y']], age_s=now-track.t_obs))
                else:
                    result['still_blocked'] += 1
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = replay(args.trace)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2), encoding='utf-8')
    print(json.dumps(data, indent=2))
