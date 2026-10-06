import rospy,json,time,signal
from pathlib import Path
from std_msgs.msg import String
out=Path(__file__).resolve().parents[1]
rospy.init_node('retained_route_broadcast_observer',anonymous=True)
log=(out/'route_snapshot_readonly_samples.jsonl').open('x')
count=0
def receive(message):
 global count
 if count>=1500:return
 value=json.loads(message.data)
 log.write(json.dumps(dict(received_s=rospy.Time.now().to_sec(),value=value,control_input=False))+'\n');log.flush();count+=1
sub=rospy.Subscriber('/swarm/route_reservations',String,receive,queue_size=100)
try:
 deadline=time.monotonic()+6000
 while not rospy.is_shutdown() and time.monotonic()<deadline and not (out.parent/'flight/result.json').exists():time.sleep(.3)
finally:
 sub.unregister();rospy.signal_shutdown('Read-only retained route capture complete');log.close()
