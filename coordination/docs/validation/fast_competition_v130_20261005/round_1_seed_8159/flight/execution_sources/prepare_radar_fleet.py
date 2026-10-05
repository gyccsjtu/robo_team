#!/usr/bin/env python3
"""Derive isolated radar aircraft wiring from the actual PX4 build scripts.

This prepares wiring, not spawn clearance or permission to fly.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path


def assignment_base(text, variable):
    pattern = r'^\s*' + re.escape(variable) + r'=\$\(\(\s*(\d+)\s*\+\s*px4_instance\s*\)\)\s*$'
    matches = re.findall(pattern, text, re.MULTILINE)
    if len(matches) != 1:
        raise ValueError('Unsupported or ambiguous PX4 assignment: ' + variable)
    return int(matches[0])


def derive(build):
    init = Path(build) / 'etc/init.d-posix'
    sources = {name: (init / name).read_text() for name in
               ('px4-rc.mavlink', 'px4-rc.simulator', 'rcS')}
    mav = sources['px4-rc.mavlink']
    local = assignment_base(mav, 'udp_offboard_port_local')
    remote = assignment_base(mav, 'udp_offboard_port_remote')
    gimbal_px4 = assignment_base(mav, 'udp_onboard_gimbal_port_local')
    gimbal_remote = assignment_base(mav, 'udp_onboard_gimbal_port_remote')
    tcp = assignment_base(sources['px4-rc.simulator'], 'simulator_tcp_port')
    if not re.search(r'param set MAV_SYS_ID\s+\$\(\(px4_instance\+1\)\)', sources['rcS']):
        raise ValueError('Unverified MAV_SYS_ID mapping')
    if not re.search(r'mavlink start[^\n]*-u \$udp_offboard_port_local[^\n]*-o \$udp_offboard_port_remote', mav):
        raise ValueError('Offboard link does not use the derived ports')
    rows = []
    for instance in range(1, 7):
        uid = 'uav_%d' % instance
        rows.append(dict(uav_id=uid, model_name=uid, px4_instance=instance,
                         mavros_namespace='/%s/mavros' % uid,
                         scan_topic='/%s/scan' % uid,
                         mavlink_system_id=instance + 1,
                         px4_local_port=local + instance,
                         mavros_local_port=remote + instance,
                         px4_gimbal_port=gimbal_px4 + instance,
                         gimbal_local_port=gimbal_remote + instance,
                         simulator_tcp_port=tcp + instance,
                         simulator_udp_port=14560 + instance,
                         fcu_url='udp://:%d@127.0.0.1:%d' % (remote + instance, local + instance),
                         lock_key=uid))
    ports = [row[key] for row in rows for key in
             ('px4_local_port', 'mavros_local_port', 'simulator_tcp_port', 'simulator_udp_port',
              'px4_gimbal_port', 'gimbal_local_port')]
    if len(set(ports)) != len(ports) or not all(1024 <= p <= 65535 for p in ports):
        raise ValueError('Fleet ports overlap or are invalid')
    return dict(schema_version=2, purpose='RADAR_FLEET_WIRING_ONLY',
                px4_build=str(Path(build).resolve()),
                source_sha256={name: hashlib.sha256(text.encode()).hexdigest()
                               for name, text in sources.items()},
                simulator_udp_origin='adapter SDF base 14560; must be matched when generating models',
                spawn_clearance_verified=False, flight_ready=False, uavs=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-build', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    manifest = derive(args.px4_build)
    # Never overwrite another run's wiring evidence.
    with Path(args.output).open('x') as stream:
        json.dump(manifest, stream, indent=2)
        stream.write('\n')
    print('Prepared six-aircraft wiring; spawn clearance and flight readiness remain unverified')


if __name__ == '__main__':
    main()
