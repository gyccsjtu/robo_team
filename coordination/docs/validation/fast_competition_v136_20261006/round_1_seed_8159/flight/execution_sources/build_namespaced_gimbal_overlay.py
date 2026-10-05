"""Build a per-aircraft MAVLink gimbal overlay without editing official sources.

Uses the verified runtime's compiler/linker flags, writes only a new output
directory, and records original/patched sources and commands for reproduction.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Unexpected pinned gimbal source: ' + old[:70])
    return text.replace(old, new)


def patched_sources(cpp, header):
    header = replace_once(header, 'private: static constexpr uint8_t ourSysid {1};',
        'private: uint8_t ourSysid {1};\n'
        '    private: uint16_t px4UdpPort {13030};\n'
        '    private: uint16_t localUdpPort {0};')
    cpp = replace_once(cpp, 'this->sdf = _sdf;', '''this->sdf = _sdf;
  if (_sdf->HasElement("mavlink_system_id")) {
    const int id = _sdf->Get<int>("mavlink_system_id");
    const int remote = _sdf->Get<int>("px4_udp_port");
    const int local = _sdf->Get<int>("gimbal_udp_port");
    if (id < 1 || id > 255 || remote < 1024 || remote > 65535 ||
        local < 1024 || local > 65535 || local == remote) {
      gzthrow("Invalid per-aircraft gimbal wiring");
    }
    this->ourSysid = static_cast<uint8_t>(id);
    this->px4UdpPort = static_cast<uint16_t>(remote);
    this->localUdpPort = static_cast<uint16_t>(local);
  }''')
    cpp = replace_once(cpp, 'this->myaddr.sin_port = htons(0);',
                       'this->myaddr.sin_port = htons(this->localUdpPort);')
    cpp = replace_once(cpp, 'dest_addr.sin_port = htons(13030);',
                       'dest_addr.sin_port = htons(this->px4UdpPort);')
    cpp = replace_once(cpp, 'void GimbalControllerPlugin::RxThread()\n{',
        'void GimbalControllerPlugin::RxThread()\n{\n'
        '  mavlink_message_t receiveBuffer {};\n'
        '  mavlink_status_t receiveStatus {};')
    cpp = replace_once(cpp, 'mavlink_parse_char(mavlinkChannel, buffer[i], &msg, &status)',
        'mavlink_frame_char_buffer(&receiveBuffer, &receiveStatus, buffer[i], &msg, &status) == MAVLINK_FRAMING_OK')
    cpp = replace_once(cpp, 'mavlink_msg_gimbal_device_set_attitude_decode(&msg, &set_attitude);',
        'mavlink_msg_gimbal_device_set_attitude_decode(&msg, &set_attitude);\n'
        '  if ((set_attitude.target_system != 0 && set_attitude.target_system != ourSysid) ||\n'
        '      (set_attitude.target_component != 0 && set_attitude.target_component != ourCompid)) return;')
    cpp = replace_once(cpp, 'void GimbalControllerPlugin::HandleAutopilotStateForGimbalDevice(const mavlink_message_t& msg)\n{',
        'void GimbalControllerPlugin::HandleAutopilotStateForGimbalDevice(const mavlink_message_t& msg)\n{\n'
        '  if (msg.sysid != ourSysid) return;')
    return cpp, header


def build(runtime, output):
    runtime, output = Path(runtime).resolve(), Path(output).resolve()
    entries = json.loads((runtime/'gazebo/compile_commands.json').read_text())
    entry = next(e for e in entries if e['file'].endswith('/gazebo_gimbal_controller_plugin.cpp'))
    source = Path(entry['file']); header = source.parent.parent/'include/gazebo_gimbal_controller_plugin.hh'
    original = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (source, header)}
    expected = ('c85fe7e0c4cb33dd83ca8959052d93716ebdd8b34912d65ec30a94403af017b4',
                '579d77857942eda99114715f79f514b86658248cbd3f77f39dda52505a28bcd5')
    if tuple(original.values()) != expected:
        raise ValueError('Gimbal source differs from the verified pinned runtime')
    cpp, hpp = patched_sources(source.read_text(), header.read_text())
    output.mkdir(parents=True, exist_ok=False)
    (output/'source').mkdir(); (output/'include').mkdir()
    cpp_path = output/'source'/source.name; cpp_path.write_text(cpp)
    (output/'include'/header.name).write_text(hpp)
    obj = output/'gimbal.o'; library = output/'librobocup_namespaced_gimbal.so'
    compile_cmd = shlex.split(entry['command'])
    compile_cmd.insert(1, '-I'+str(output/'include'))
    compile_cmd[compile_cmd.index('-o')+1] = str(obj)
    compile_cmd[compile_cmd.index('-c')+1] = str(cpp_path)
    raw_link = subprocess.check_output(['ninja','-t','commands','gazebo_gimbal_controller_plugin'],
        cwd=entry['directory'], text=True).splitlines()[-1]
    if not raw_link.startswith(': && ') or not raw_link.endswith(' && :'):
        raise ValueError('Unexpected runtime linker command')
    link_cmd = shlex.split(raw_link[5:-5])
    link_cmd[link_cmd.index('-o')+1] = str(library)
    link_cmd = [str(obj) if token.endswith('gazebo_gimbal_controller_plugin.cpp.o') else token
                for token in link_cmd]
    with (output/'build.log').open('w') as log:
        for cmd in (compile_cmd,link_cmd):
            subprocess.run(cmd,cwd=entry['directory'],stdout=log,stderr=log,check=True)
    if original != {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (source,header)}:
        raise RuntimeError('Official gimbal source changed during build')
    manifest = dict(revision='v1.21_per_aircraft_gimbal', original_source_sha256=original,
        compiler_command=compile_cmd, linker_command=link_cmd, runtime=str(runtime),
        library=str(library), library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
        patched_source_sha256={str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (cpp_path,output/'include'/header.name)},
        sensor_parameters_changed=False, official_sources_modified=False)
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    return manifest


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime',required=True); p.add_argument('--output',required=True)
    args = p.parse_args()
    print(json.dumps(build(args.runtime,args.output),indent=2))
