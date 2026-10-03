#!/usr/bin/env python3
"""Independent development observer. Actor truth never goes to controllers."""
import argparse
import bisect
import json
import math
from pathlib import Path
import threading
import time


def paired_error(sample_s, xy, truth):
    index = bisect.bisect_left([row[0] for row in truth], sample_s)
    if index == 0 or index == len(truth):
        return None
    first, second = truth[index-1], truth[index]
    gap = second[0]-first[0]
    if not 0 < gap <= .2:
        return None
    fraction = (sample_s-first[0])/gap
    interpolated = [first[i]+fraction*(second[i]-first[i]) for i in (1,2)]
    return math.dist(xy, interpolated)


def main():
    import rospy
    from gazebo_msgs.srv import GetModelState
    from std_msgs.msg import String
    from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--tag', choices=('green','blue','brown','white'), default='green')
    args = parser.parse_args()
    run = Path(args.run).resolve()
    actor_id = {'green':0, 'blue':1, 'brown':2, 'white':3}[args.tag]
    prefix = 'independent_visual_accuracy' + ('' if args.tag == 'green' else '_'+args.tag)
    record = (run/(prefix+'.jsonl')).open('x')
    rospy.init_node('independent_visual_accuracy', anonymous=True)
    rospy.set_param('/use_sim_time', True)
    lock = threading.Lock()
    truth, images, official = [], [], []
    def save(kind, row, collection):
        with lock:
            collection.append(row)
            record.write(json.dumps(dict(kind=kind, sample=row))+'\n')
    def image_cb(message):
        value = json.loads(message.data)
        if value.get('target_id') == args.tag:
            save('image', value, images)
    def official_cb(message):
        save('official', [rospy.Time.now().to_sec(), message.x, message.y], official)
    subscriptions = [rospy.Subscriber('/swarm/visual_observation', String, image_cb, queue_size=100),
        rospy.Subscriber('/actor_'+args.tag+'_info', ActorInfo, official_cb, queue_size=100)]
    service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    deadline = time.monotonic()+740
    try:
        last_sample = -1.
        while time.monotonic() < deadline and not (run/'result.json').exists():
            before = rospy.Time.now().to_sec()
            if before-last_sample >= .05:
                response = service('actor_'+str(actor_id),'world')
                after = rospy.Time.now().to_sec()
                if response.success and 0 <= after-before <= .05:
                    position = response.pose.position
                    stamp = (before+after)/2.
                    save('actor_truth', [stamp,position.x,position.y], truth)
                    last_sample = stamp
            time.sleep(.03)
    finally:
        for sub in subscriptions:
            sub.unregister()
        rospy.signal_shutdown('Independent observation complete')
        with lock:
            record.close()
        image_errors = [paired_error(v['sample_s'],v['xyz'][:2],truth) for v in images]
        report_errors = [paired_error(row[0],row[1:],truth) for row in official]
        def metrics(values):
            valid = [v for v in values if v is not None]
            return dict(total=len(values), aligned=len(valid), missing=len(values)-len(valid),
                maximum_error_m=max(valid) if valid else None,
                mean_error_m=sum(valid)/len(valid) if valid else None,
                fraction_strictly_below_1m=sum(v<1. for v in valid)/len(valid) if valid else None)
        (run/(prefix+'.json')).write_text(json.dumps(dict(
            actor='actor_'+str(actor_id),tag=args.tag,actor_truth_samples=len(truth),
            original_image_coordinates=metrics(image_errors),official_report_coordinates=metrics(report_errors),
            control_input=False,official_judge_executed=False,formal_competition_pass=False),indent=2))


if __name__ == '__main__':
    main()
