"""Offline comparison only: Gazebo truth never enters the controller."""
import argparse
import csv
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', required=True)
    parser.add_argument('--uid', required=True)
    args = parser.parse_args()
    run = Path(args.run)
    truth = [json.loads(line) for line in (run/'six_truth.jsonl').read_text().splitlines()]
    control = list(csv.DictReader((run/'algorithm'/('control_'+args.uid+'.csv')).open()))
    samples = []
    for stamp in range(60, 126, 5):
        actual = min(truth, key=lambda row: abs(row['sim_s']-stamp))
        cmd = min(control, key=lambda row: abs(float(row['ros_time'])-actual['sim_s']))
        pos = actual['positions'][args.uid]
        estimate = [float(cmd['uav_'+axis]) for axis in 'xyz']
        samples.append(dict(sim_s=actual['sim_s'], true_xyz=pos, estimated_xyz=estimate,
            horizontal_error_m=math.dist(pos[:2], estimate[:2]),
            command_xy=[float(cmd['cmd_vx']), float(cmd['cmd_vy'])]))
    print(json.dumps(samples, indent=2))


if __name__ == '__main__':
    main()
