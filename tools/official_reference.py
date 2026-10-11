#!/usr/bin/env python3
"""Pinned upstream bytes for offline environment checks, never flight input."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request


def contract():
    return json.loads(Path(__file__).with_suffix('.json').read_text())


def digest(data):
    return hashlib.sha256(data).hexdigest()


def cache_dir(spec):
    return Path.home()/'.cache/robocup_official'/spec['commit']


def pinned_blobs(root, names):
    spec = contract()
    blobs = {}
    for name in names:
        for path in (Path(root)/'robocup'/name, cache_dir(spec)/name):
            if path.is_file():
                data = path.read_bytes()
                if digest(data) == spec['sha256'][name]:
                    blobs[name] = data
                    break
        if name not in blobs:
            raise ValueError('Missing verified official bytes for %s; run tools/official_reference.py --download before environment preparation' % name)
    return blobs


def check_local(root, names=('control_actor.py', 'score_cal.py')):
    spec = contract()
    failures = []
    for name in names:
        path = Path(root)/'robocup'/name
        if not path.is_file() or digest(path.read_bytes()) != spec['sha256'][name]:
            failures.append(name)
    return failures


def download():
    spec = contract()
    destination = cache_dir(spec)
    destination.mkdir(parents=True, exist_ok=True)
    for name, expected in spec['sha256'].items():
        path = destination/name
        if path.is_file() and digest(path.read_bytes()) == expected:
            continue
        data = urllib.request.urlopen(spec['raw_base']+name, timeout=25).read()
        if digest(data) != expected:
            raise ValueError('Upstream checksum differs: '+name)
        temporary = path.with_suffix(path.suffix+'.tmp')
        temporary.write_bytes(data)
        temporary.replace(path)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path.home()/'XTDrone')
    parser.add_argument('--download', action='store_true', help='Fetch verified cache only; never overwrite the official environment')
    args = parser.parse_args()
    try:
        if args.download:
            print('[official] verified cache: %s' % download())
            return 0
        failures = check_local(args.root)
        if failures:
            print('[official] FAIL pinned %s: %s' % (contract()['commit'], ', '.join(failures)))
            return 1
        print('[official] PASS actor/judge exact bytes at '+contract()['commit'])
        return 0
    except (OSError, ValueError) as exc:
        print('[official] FAIL '+str(exc))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
