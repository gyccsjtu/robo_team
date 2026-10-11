#!/usr/bin/env python3
"""Exercise fresh preparation and read-only reuse in a disposable Linux fixture.

Only reads the official sources. No simulator is started and no live overlay
is accessed. All generated symlinks and startup copies remain below /tmp.
"""
import contextlib
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile


REPO = Path(__file__).resolve().parents[2]
OFFICIAL_PX4 = Path('/root/third_party_official/PX4-Autopilot')
OFFICIAL_XT = Path('/root/third_party_official/XTDrone_official')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def invoke(module, target, *options):
    old_argv = sys.argv
    output = io.StringIO()
    try:
        sys.argv = [str(module.__file__), '--output', str(target)] + list(options)
        with contextlib.redirect_stdout(output):
            module.main()
    finally:
        sys.argv = old_argv
    return json.loads(output.getvalue())


def snapshot(root):
    result = {}
    for path in sorted(root.rglob('*')):
        info = path.lstat()
        if path.is_symlink():
            value = dict(kind='symlink', target=os.readlink(str(path)))
        elif path.is_file():
            value = dict(kind='file', sha256=digest(path))
        else:
            value = dict(kind='directory')
        value['mtime_ns'] = info.st_mtime_ns
        result[str(path.relative_to(root))] = value
    return result


def main():
    if sys.platform != 'linux':
        raise RuntimeError('Run this disposable fixture in Ubuntu-20.04, not Windows')
    script = REPO/'coordination/scripts/container/prepare_botsort_sim_runtime.py'
    spec = importlib.util.spec_from_file_location('botsort_prepare_fixture', str(script))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    startup_base = Path('build/px4_sitl_default/etc/init.d-posix')
    source_specs = [
        (OFFICIAL_PX4, startup_base/'px4-rc.mavlink'),
        (OFFICIAL_PX4, startup_base/'airframes/6011_typhoon_h480.post'),
        (OFFICIAL_PX4, startup_base/'airframes/6012_typhoon_h480_ctrlalloc.post'),
        (OFFICIAL_PX4, Path('Tools/sitl_gazebo/worlds/robocup.world')),
        (OFFICIAL_XT, Path('sitl_config/models/typhoon_h480/typhoon_h480.sdf')),
        (OFFICIAL_XT, Path('sitl_config/models/hokuyo_lidar/model.sdf')),
        (OFFICIAL_XT, Path('sitl_config/models/stereo_camera/model.sdf')),
    ]
    source_before = {str(base/relative): digest(base/relative)
                     for base, relative in source_specs}
    with tempfile.TemporaryDirectory(prefix='botsort_prepare_fixture_', dir='/tmp') as tmp:
        root = Path(tmp)
        fixture_px4 = root/'official_px4'
        fixture_xt = root/'official_xt'
        fixture_px4.mkdir()
        fixture_xt.mkdir()
        for base, relative in source_specs:
            destination = (fixture_px4 if base == OFFICIAL_PX4 else fixture_xt)/relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(str(base/relative), str(destination))
        (fixture_px4/'Tools/sitl_gazebo/models/typhoon_h480').mkdir(parents=True)
        (fixture_px4/'build/px4_sitl_default/bin').mkdir()
        (fixture_px4/'build/px4_sitl_default/bin/px4').write_text('fixture; not executable\n')
        (fixture_px4/'build/px4_sitl_default/build_gazebo').mkdir()
        (fixture_px4/'build/build_reference').write_text('fixture compiled asset\n')
        (fixture_xt/'robocup').mkdir()
        (fixture_xt/'robocup/fixture_actor.py').write_text('# fixture actor; never launched\n')

        module.PX4 = fixture_px4
        module.XT = fixture_xt
        module.ALLOWED = root/'private_allowed'
        target = module.ALLOWED/'rebuild'
        fresh = invoke(module, target)
        build = target/'PX4_Firmware/build'
        startup = build/'px4_sitl_default/etc/init.d-posix'
        assert build.is_dir() and not build.is_symlink()
        assert all(not path.is_symlink() for path in startup.rglob('*'))
        assert (build/'px4_sitl_default/bin').is_symlink()
        ports = module.verify_private_ports(target)
        assert len(ports['assignments']) == 6 and ports['official_files_unchanged']
        initial = snapshot(target)
        reused = invoke(module, target)
        assert reused['reused_without_writes'] and snapshot(target) == initial
        verified = invoke(module, target, '--verify-only')
        assert verified['private_startup_ports_valid'] and snapshot(target) == initial
        invoke(module, target, '--isolate-px4-ports')
        assert snapshot(target) == initial

        incomplete = module.ALLOWED/'incomplete'
        incomplete.mkdir()
        sentinel = incomplete/'untouched.txt'
        sentinel.write_text('preserve incomplete preparation\n')
        incomplete_before = snapshot(incomplete)
        try:
            invoke(module, incomplete)
        except FileNotFoundError:
            pass
        else:
            raise AssertionError('Incomplete overlay was unexpectedly accepted')
        assert snapshot(incomplete) == incomplete_before

        source_after = {str(base/relative): digest(base/relative)
                        for base, relative in source_specs}
        assert source_after == source_before
        result = dict(
            fixture_only=True, simulator_started=False, live_overlay_accessed=False,
            fresh_auto_private_startup=True, six_port_assignments=True,
            physical_startup_copy=True, compiled_binaries_read_only_links=True,
            repeat_files_hash_and_mtime_unchanged=True,
            verify_only_files_hash_and_mtime_unchanged=True,
            compatibility_isolate_files_hash_and_mtime_unchanged=True,
            incomplete_overlay_rejected_without_writes=True,
            official_input_sha256_unchanged=True,
            tracked_overlay_entries=len(initial),
            model_sha256=fresh['model_sha256'], world_sha256=fresh['world_sha256'],
            official_source_sha256=source_before,
        )
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
