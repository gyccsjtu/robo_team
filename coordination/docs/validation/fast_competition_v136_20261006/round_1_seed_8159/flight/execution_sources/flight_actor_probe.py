"""Development scene and observer; never supplies actor truth to controllers."""
import hashlib
import json
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET
import math
from evidence_writer import JsonlEvidence


class FlightActorProbe:
    def __init__(self, out, snapshot, flight_env, spawn):
        import rospy
        from std_msgs.msg import String
        from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties
        self.rospy, self.out, self.env = rospy, out, flight_env
        physics = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)()
        changed = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)(
            physics.time_step, 50., physics.gravity, physics.ode_config)
        (out/'development_physics_rate.json').write_text(json.dumps(dict(
            old_update_rate=physics.max_update_rate, requested_update_rate=50.,
            time_step=physics.time_step, success=changed.success,
            reason='Six software-rendered cameras; unchanged original sensor rates, development only.')))
        if not changed.success:
            raise RuntimeError('DEVELOPMENT_PHYSICS_RATE_CHANGE_FAILED')
        self.visual, self.confirmed, self.target_grants, self.routes, self.motion = [], [], [], [], []
        self.current_tasks={}
        self.actor_spawned = False
        self.actor_error = None
        self.record = JsonlEvidence(out/'target_execution_events.jsonl')
        def observation(message, kind, collection):
            value = json.loads(message.data)
            if value.get('run_id') != flight_env['ROBOCUP_RUN_ID']:
                return
            collection.append(value)
            self.record.write(dict(kind=kind,received_s=rospy.Time.now().to_sec(),message=value))
        def task(message):
            value = json.loads(message.data)
            if value.get('run_id') != flight_env['ROBOCUP_RUN_ID']:
                return
            if value.get('action') == 'GRANT':
                self.current_tasks[value['uav_id']] = value
            elif value.get('action') == 'STOP':
                self.current_tasks.pop(value['uav_id'], None)
            if value.get('action')=='GRANT' and value['task']['task_type']==1:
                self.target_grants.append(value)
                self.record.write(dict(kind='target_grant',received_s=rospy.Time.now().to_sec(),message=value))
        self.subs = [rospy.Subscriber('/swarm/visual_observation',String,
            lambda m:observation(m,'visual',self.visual),queue_size=100),
            rospy.Subscriber('/swarm/confirmed_visual_observation',String,
            lambda m:observation(m,'confirmed',self.confirmed),queue_size=100),
            rospy.Subscriber('/swarm/authorized_assignment',String,task,queue_size=100),
            rospy.Subscriber('/swarm/route_grant',String,
                lambda m:observation(m,'route',self.routes),queue_size=100)]
        repo = Path(__file__).resolve().parents[2]
        shutil.copytree(repo/'perception',snapshot/'perception',ignore=shutil.ignore_patterns('__pycache__','*.pyc','*.bak*'))
        scripts = snapshot/'coordination/src/robocup_swarm/scripts'
        self.processes = [spawn(['python3','-u',str(scripts/'yolo_target_bridge.py')],'yolo_target_bridge',flight_env)]
        python = flight_env.get('VISION_PYTHON','/root/robo_team_build/vision_env/bin/python')
        for uid in flight_env['SWARM_UAV_IDS'].split(','):
            node_env = dict(flight_env,PR_UAV=uid,PR_LOGICAL_UAV_ID=uid,
                PR_CAM_LINK=uid+'::cgo3_camera_link',PR_CAM_OFF_BL='0,0,-0.162',
                PR_CAM_TOPIC='/'+uid+'/cgo3_camera/image_raw',PR_WEIGHTS=str(repo/'weights/best_yolo11n_bino_v1.pt'),
                OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
            self.processes.append(spawn([python,'-u',str(snapshot/'perception/perception_real.py')],
                                        'perception_'+uid,node_env))

    def spawn_actor(self, positions):
        from gazebo_msgs.srv import SpawnModel
        from geometry_msgs.msg import Pose
        from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion
        if self.actor_spawned or any(p[2] < 3.5 for p in positions.values()):
            return
        candidates = [(-9.,10.),(-9.,-10.),(7.,10.),(7.,-10.),(-15.,18.)]
        chosen = next((p for p in candidates if all(math.dist(p,q[:2])>=10. for q in positions.values())),None)
        if chosen is None:
            return
        repo = Path(__file__).resolve().parents[2]
        source = repo/'perception/worlds/actor_min.world'
        actor = ET.parse(source).find("world/actor[@name='actor_0']")
        plugin = actor.find("plugin[@filename='libros_actor_cmd_pose_plugin.so']")
        x,y = chosen
        plugin.find('init_pose').text = '%s %s 1.25 1.57 0 0' % (x,y)
        for name,value in (('min_x',-24),('max_x',24),('min_y',y),('max_y',y)):
            ET.SubElement(plugin,name).text = str(value)
        root = ET.Element('sdf',version='1.6')
        root.append(actor)
        actor_xml = ET.tostring(root,encoding='unicode')
        (self.out/'actor_spawn.sdf').write_text(actor_xml)
        shutil.copy2(source,self.out/'actor_source.world')
        (self.out/'actor_spawn_request.json').write_text(json.dumps(dict(initial_xy=chosen,
            initial_world_certificate_only=True,insert_after_takeoff=True,
            observer_positions_for_fixture_insertion_only=positions,
            actor_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            spawn_sdf_sha256=hashlib.sha256(actor_xml.encode()).hexdigest()),indent=2))
        pose=Pose()
        pose.orientation.w=1.
        response=self.rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)('actor_0',actor_xml,'actor_0',pose,'world')
        if not response.success:
            self.actor_error='ACTOR_FACTORY_FAILED: '+response.status_message
            return
        self.actor_spawned=True
        self.actor_command=self.rospy.Publisher('/actor_0/cmd_motion',ActorMotion,queue_size=1)
        self.actor_motion=ActorMotion(x=min(20.,x+10.),y=y,v=.8)
        self.command_sent=False

    def tick(self, positions):
        self.spawn_actor(positions)
        if self.actor_spawned and not self.command_sent and self.actor_command.get_num_connections():
            self.actor_command.publish(self.actor_motion)
            self.command_sent=True

    def observe_command(self, uid, message):
        # Observer of the actual raw executor stream, not a source of commands.
        grant=self.current_tasks.get(uid)
        if (grant and grant['task']['task_type']==1 and math.hypot(message.velocity.x,message.velocity.y)> .1
                and grant['expires_s']-3. <= message.header.stamp.to_sec() < grant['expires_s']
                and any(r['uav_id']==uid and r['generation']==grant['generation'] for r in self.routes)):
            self.motion.append((uid,grant['generation'],message.header.stamp.to_sec()))

    def summary(self, truth):
        pairs={(g['uav_id'],g['generation']) for g in self.target_grants}
        matching=[r for r in self.routes if (r['uav_id'],r['generation']) in pairs]
        execution_pairs={(uid,gen) for uid,gen,_ in self.motion}
        executed=bool(execution_pairs & pairs)
        moved=False
        route_pairs={(r['uav_id'],r['generation']) for r in matching}
        for uid,gen in execution_pairs & pairs & route_pairs:
            stamps=[stamp for owner,generation,stamp in self.motion if (owner,generation)==(uid,gen)]
            samples=[s['positions'][uid][:2] for s in truth if min(stamps) <= s['sim_s'] <= max(stamps)+.2]
            if samples and any(math.dist(samples[0],p)>.5 for p in samples):
                moved=True
                break
        verified=self.actor_spawned and len(self.visual)>=5 and len(self.confirmed)>=3 and bool(matching) and executed and moved
        return dict(physical_visual_tracking_verified=verified,actor_spawned=self.actor_spawned,
            actual_visual_observations=len(self.visual),confirmed_visual_observations=len(self.confirmed),
            target_authority_grants=len(self.target_grants),target_generation_route_grants=len(matching),
            target_execution_motion_observed=executed,actor_fixture_error=self.actor_error,
            true_motion_after_target_grant_observed=moved,
            actor_ground_truth_control_input=False,official_elimination_verified=False)

    def close(self):
        for sub in self.subs:
            sub.unregister()
        self.record.close()
