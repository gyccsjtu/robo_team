#!/usr/bin/env python3
"""Check each locked distribution against built wheel metadata and record hashes."""
import argparse
import email
import hashlib
import json
from pathlib import Path
import zipfile
from packaging.requirements import Requirement
from packaging.utils import canonicalize_name


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lock', type=Path, required=True)
    parser.add_argument('--wheelhouse', type=Path, required=True)
    args = parser.parse_args()
    locks = {}
    for line in args.lock.read_text().splitlines():
        if line.strip() and not line.startswith('#'):
            req = Requirement(line)
            locks[canonicalize_name(req.name)] = req
    found, records = {}, []
    for wheel in sorted(args.wheelhouse.glob('*.whl')):
        with zipfile.ZipFile(wheel) as archive:
            metadata = next(p for p in archive.namelist() if p.endswith('.dist-info/METADATA'))
            info = email.message_from_bytes(archive.read(metadata))
        name, version = canonicalize_name(info['Name']), info['Version']
        if name not in locks or version not in locks[name].specifier or name in found:
            raise ValueError('unexpected, duplicate or unlocked wheel: '+wheel.name)
        found[name] = version
        records.append(dict(filename=wheel.name, name=name, version=version,
                            sha256=hashlib.sha256(wheel.read_bytes()).hexdigest()))
    if set(found) != set(locks):
        raise ValueError('missing locked wheels: '+str(sorted(set(locks)-set(found))))
    (args.wheelhouse/'wheelhouse_manifest.json').write_text(json.dumps(records, indent=2)+'\n')
    print('[wheelhouse] %d locked distributions hashed' % len(records))


if __name__ == '__main__':
    main()
