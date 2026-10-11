"""Environment-only open-loop motion for this isolated recognition benchmark."""
import time
import rospy
from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion

rospy.init_node('recognition_bench_actor_environment')
if not rospy.get_param('/recognition_bench/isolated', False):
    raise RuntimeError('requires the isolated benchmark launch; never run against a match')
pubs = [rospy.Publisher('/actor_%d/cmd_motion' % i, ActorMotion, queue_size=1) for i in range(6)]
deadline = time.monotonic() + 600
while not rospy.is_shutdown() and time.monotonic() < deadline:
    shift = .75 if int(rospy.get_time() / 1.5) % 2 else -.75
    for i, pub in enumerate(pubs):
        pub.publish(ActorMotion(x=16., y=-7.5 + 1.8*i + shift, v=1.))
    time.sleep(.1)
