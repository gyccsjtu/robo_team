"""Bounded single-aircraft camera experiment. Flight uses own MAVROS data only.
No actor state, model state, judge data, or detections feed flight commands.
"""
import json
import math
import threading
import time
from pathlib import Path
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import MountControl, State, ParamValue
from mavros_msgs.srv import SetMode, CommandBool, ParamPull, ParamSet, ParamGet, MountConfigure

ROOT = Path(__file__).resolve().parent
PREFIX = '/typhoon_h480_0/mavros'
rospy.init_node('gimbal_recognition_bench_flight')
if not rospy.get_param('/recognition_bench/isolated', False):
    raise RuntimeError('requires the isolated benchmark launch; never run against a match')
state = State()
pose = None
last_pose_wall = 0.
target = None
angles = (0., 0.)
phase = 'init'
phase_started = 0.
running = True
owns_flight = False
log = (ROOT / 'flight.jsonl').open('w', buffering=1)

def state_cb(msg):
    global state
    state = msg

def pose_cb(msg):
    global pose, last_pose_wall
    pose = msg
    last_pose_wall = time.monotonic()

rospy.Subscriber(PREFIX + '/state', State, state_cb, queue_size=1)
rospy.Subscriber(PREFIX + '/local_position/pose', PoseStamped, pose_cb, queue_size=1)
pos_pub = rospy.Publisher(PREFIX + '/setpoint_position/local', PoseStamped, queue_size=1)
mount_pub = rospy.Publisher(PREFIX + '/mount_control/command', MountControl, queue_size=1)
phase_pub = rospy.Publisher('/recognition_bench/phase', String, queue_size=1, latch=True)
mode = rospy.ServiceProxy(PREFIX + '/set_mode', SetMode)
arm = rospy.ServiceProxy(PREFIX + '/cmd/arming', CommandBool)

def publish_loop():
    while running and not rospy.is_shutdown():
        if target is not None:
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target
            msg.pose.orientation.w = 1.
            pos_pub.publish(msg)
            m = MountControl()
            m.header.stamp = rospy.Time.now()
            m.mode = 2
            m.pitch, m.yaw = angles
            mount_pub.publish(m)
        time.sleep(.04)

def change_phase(name):
    global phase, phase_started
    phase, phase_started = name, rospy.get_time()
    data = dict(phase=phase, stamp=phase_started, target=target, pitch=angles[0], yaw=angles[1])
    phase_pub.publish(String(data=json.dumps(data)))
    log.write(json.dumps(data) + '\n')
    print(data, flush=True)

def check():
    if rospy.is_shutdown() or not state.connected or time.monotonic()-last_pose_wall > .8:
        raise RuntimeError('own flight state stale or disconnected')
    if state.armed and state.mode != 'OFFBOARD':
        raise RuntimeError('unexpected airborne flight mode: ' + state.mode)
    p = pose.pose.position
    if not (-2 < p.x < 13 and abs(p.y-base[1]) < 2 and p.z < base[2]+4.8):
        raise RuntimeError('bounded experiment corridor exceeded')

def hold(duration):
    end = time.monotonic()+duration
    while time.monotonic() < end:
        check()
        p = pose.pose.position
        log.write(json.dumps(dict(phase=phase, stamp=rospy.get_time(), xyz=[p.x,p.y,p.z],
                                  mode=state.mode, armed=state.armed))+'\n')
        time.sleep(.1)

def move(x, z):
    global target
    change_phase('transit')
    end = time.monotonic()+35
    previous = time.monotonic()
    while time.monotonic() < end:
        check()
        now = time.monotonic()
        dt = min(.1, now-previous)
        previous = now
        goal = (x, base[1], z)
        target = tuple(a+max(-.65*dt,min(.65*dt,b-a)) for a,b in zip(target,goal))
        p = pose.pose.position
        if max(abs(a-b) for a,b in zip(target,goal)) < .01 and math.dist((p.x,p.y,p.z),goal) < .2:
            hold(2.)
            return
        time.sleep(.04)
    raise RuntimeError('position settling timeout')

thread = threading.Thread(target=publish_loop, daemon=True)
thread.start()
error = None
try:
    deadline = time.monotonic()+75
    while not (state.connected and pose is not None and time.monotonic()-last_pose_wall < .5):
        if time.monotonic()>deadline:
            raise RuntimeError('FCU/pose not ready')
        time.sleep(.2)
    if state.armed:
        raise RuntimeError('refusing to take over an already armed aircraft')
    # Preserve arming checks; verify only the normal OFFBOARD RC-loss exception.
    pull = rospy.ServiceProxy(PREFIX+'/param/pull', ParamPull)
    setter = rospy.ServiceProxy(PREFIX+'/param/set', ParamSet)
    getter = rospy.ServiceProxy(PREFIX+'/param/get', ParamGet)
    if not pull(force_pull=False).success:
        raise RuntimeError('FCU parameters unavailable')
    current = getter(param_id='COM_RCL_EXCEPT')
    if not current.success:
        raise RuntimeError('cannot read OFFBOARD RC configuration')
    desired = current.value.integer | 4
    if not setter(param_id='COM_RCL_EXCEPT', value=ParamValue(integer=desired, real=0.)).success:
        raise RuntimeError('cannot set OFFBOARD RC configuration')
    if getter(param_id='COM_RCL_EXCEPT').value.integer != desired:
        raise RuntimeError('OFFBOARD RC configuration not verified')
    rospy.ServiceProxy(PREFIX+'/mount_control/configure', MountConfigure)(
        mode=2, stabilize_roll=False, stabilize_pitch=False, stabilize_yaw=False)
    p = pose.pose.position
    base = (p.x, p.y, p.z)
    target = base
    time.sleep(3.)
    deadline = time.monotonic()+30
    while not (state.armed and state.mode=='OFFBOARD'):
        if time.monotonic()>deadline:
            raise RuntimeError('normal OFFBOARD/arming checks timed out')
        if state.mode != 'OFFBOARD':
            mode(custom_mode='OFFBOARD')
        else:
            if arm(value=True).success:
                owns_flight = True
        time.sleep(1.)
    move(0., base[2]+3.)
    for distance in (16,12,8,5):
        angles = (-math.degrees(math.atan2(2.,distance)), 0.)
        move(16.-distance, base[2]+3.)
        for view in ('level','aim'):
            angles = (0. if view=='level' else -math.degrees(math.atan2(2.,distance)),0.)
            change_phase('settle')
            hold(2.)
            change_phase('actors_%dm_%s' % (distance,view))
            hold(7.)
    angles=(-14.,0.)
    move(8.,base[2]+3.)
    for name, y in (('statue',4.),('hydrant',-10.)):
        angles=(-math.degrees(math.atan2(2.,math.hypot(8.,y+3.))),
                -math.degrees(math.atan2(y+3.,8.)))
        change_phase('settle')
        hold(2.)
        change_phase(name+'_8m_forward')
        hold(7.)
except Exception as exc:
    error=repr(exc)
    print('EXPERIMENT ERROR',error,flush=True)
finally:
    change_phase('landing')
    if owns_flight and state.armed:
        mode(custom_mode='AUTO.LAND')
        deadline=time.monotonic()+50
        while state.armed and time.monotonic()<deadline and not rospy.is_shutdown():
            time.sleep(.2)
    change_phase('done')
    running=False
    thread.join(timeout=1)
    log.close()
    (ROOT/'flight_result.json').write_text(json.dumps(dict(error=error,armed=state.armed,
                                                          mode=state.mode),indent=2)+'\n')
    if owns_flight and state.armed:
        raise RuntimeError('landing not confirmed; inspect before stopping Gazebo')
    if error:
        raise RuntimeError(error)
