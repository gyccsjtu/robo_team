#!/usr/bin/env python3
"""Record disk code/weights at startup, without ROS or simulator subscriptions."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, required=True)
    ap.add_argument('--official', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--started', type=float, required=True)
    args = ap.parse_args()
    root = args.root.resolve()
    paths = [root/'run_match.sh', root/'launch/robocup_with_laser.launch']
    paths.extend(root/'tools'/name for name in (
        'audit_match_graph.py', 'wait_match_ready.py', 'match_sensor_inventory.py',
        'record_search_segments.py', 'record_motion_safety.py', 'record_tracking_images.py'))
    paths.extend(sorted((root/'coordination/src/robocup_swarm/msg').glob('*.msg')))
    for folder in ('perception', 'coordination/src/robocup_swarm/scripts',
                   'coordination/src/robocup_navigation/src/robocup_navigation'):
        paths.extend(sorted((root/folder).glob('*.py')))
    weights = ([Path(os.environ['PR_WEIGHTS'])] if os.environ.get('PR_WEIGHTS') else
               [root/'weights/best_yolo11n_bino_sculpture_v3.pt',
                Path.home()/'catkin_ws/src/yolov11_ros/weights/best.pt'])
    chosen = next((p for p in weights if p.is_file()), None)
    inventory_path = os.environ.get('ROBOCUP_SENSOR_INVENTORY')
    inventory = json.loads(Path(inventory_path).read_text()) if inventory_path else None
    result = dict(
        script_started_wall_s=args.started,
        snapshot_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=str(root),text=True).strip(),
        source_sha256={str(p.relative_to(root)):digest(p) for p in paths if p.is_file()},
        official_sha256={name:digest(args.official/name)
                         for name in ('score_cal.py','control_actor.py')
                         if (args.official/name).is_file()},
        team_only=os.environ.get('ROBOCUP_TEAM_ONLY') == '1',
        clock_permission={'status':os.environ.get('CLOCK_PERMISSION','allowed'),
            'reference':os.environ.get('CLOCK_PERMISSION_REFERENCE',''),
            'note':'Use of native /clock was confirmed by the user on 2026-10-13. '
                   'Clock aliases and forbidden actor/judge/Gazebo state inputs remain audited.'},
        team_time={'source':os.environ.get('TEAM_TIME_SOURCE','ros_clock'),
            'flight_transport':'baseline native ROS scheduling',
            'business_timers':'native rospy simulation time from /clock'},
        runtime_baseline={'source_commit':os.environ.get('ROBOCUP_RUNTIME_BASELINE',
            '721927c5c948e22b956f91a7fdec1394e29ade98'),
            'scope':'Restored flight, perception and team algorithms; startup audit and fleet sensor accounting retained.',
            'note':'source_sha256 records exact working-tree files; git_head may additionally contain orchestration fixes.'},
        sensor_inventory=inventory,
        search_segments=os.environ.get('SEARCH_SEGMENTS','0'),
        motion_safety=os.environ.get('MOTION_SAFETY','0'),
        south_patrol=os.environ.get('SOUTH_PATROL','0'),
        southeast_inner_patrol=os.environ.get('SOUTH_PATROL_SOUTHEAST','0'),
        northeast_inner_patrol=os.environ.get('SOUTH_PATROL_NORTHEAST','0'),
        east_persistent_patrol=os.environ.get('EAST_PATROL','0'),
        east_split_patrol=os.environ.get('EAST_SPLIT_PATROL','0'),
        takeoff_require_camera=os.environ.get('TAKEOFF_REQUIRE_CAMERA','0'),
        orbit_radius_close_m=os.environ.get('ORBIT_RADIUS_CLOSE','8.0'),
        early_view_pursuit=os.environ.get('EARLY_VIEW_PURSUIT','0'),
        localized_search_follow=os.environ.get('LOCALIZED_SEARCH_FOLLOW','0'),
        candidate_image_max_tilt_deg=os.environ.get('CANDIDATE_IMAGE_MAX_TILT_DEG','8'),
        immediate_visible_dispatch=os.environ.get('IMMEDIATE_VISIBLE_DISPATCH','0'),
        pixel_view_recovery=os.environ.get('PIXEL_VIEW_RECOVERY','0'),
        pixel_recovery_approach=os.environ.get('PIXEL_RECOVERY_APPROACH','0'),
        coupled_view_geometry=os.environ.get('COUPLED_VIEW_GEOMETRY','0'),
        view_space_backoff=os.environ.get('VIEW_SPACE_BACKOFF','0'),
        bridge_fresh_image_slot=os.environ.get('BRIDGE_FRESH_IMAGE_SLOT','0'),
        search_road_first_pass=os.environ.get('SEARCH_ROAD_FIRST_PASS','0'),
        search_directed_coverage=os.environ.get('SEARCH_DIRECTED_COVERAGE','0'),
        routine_search_bounds=os.environ.get('ROUTINE_SEARCH_BOUNDS','0'),
        routine_observation_heading=os.environ.get('ROUTINE_OBSERVATION_HEADING','0'),
        takeoff_progress_recovery_early=os.environ.get('TAKEOFF_PROGRESS_RECOVERY_EARLY','0'),
        report_flow_diagnostics=os.environ.get('REPORT_FLOW_DIAGNOSTICS','0'),
        edge_search_review=os.environ.get('EDGE_SEARCH_REVIEW','0'),
        building_subpoint_review=os.environ.get('BUILDING_SUBPOINT_REVIEW','0'),
        routine_search_region={'x_min':-50.0,'x_max':130.0,'y_min':-60.0,'y_max':60.0,
            'basis':'Public official actor activity bounds; intersected with sensor map and flight limits. Not actor state.'},
        flight_boundary={key:os.environ.get(key, default) for key, default in (
            ('FLIGHT_X_MIN','-50.0'), ('FLIGHT_X_MAX','150.0'),
            ('FLIGHT_Y_MIN','-60.0'), ('FLIGHT_Y_MAX','60.0'),
            ('FLIGHT_INSET','2.0'), ('MAP_GUARD_MARGIN','2.0'))},
        report_policy=os.environ.get('BRIDGE_REPORT_POLICY','fit'),
        weights={'path':str(chosen),'sha256':digest(chosen)} if chosen else None,
        note='Disk snapshot before process startup; includes dirty working-tree content. '
             'Effective process parameters are logged by each node. No runtime reload is implied.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    temporary = args.output.with_suffix('.tmp')
    temporary.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    temporary.replace(args.output)
    print('[manifest] %s: %d source files' % (args.output,len(result['source_sha256'])))


if __name__ == '__main__':
    main()
