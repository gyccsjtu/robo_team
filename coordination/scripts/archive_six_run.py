#!/usr/bin/env python3
"""Preserve an already completed isolated flight, including executed source hashes."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    source, output = Path(args.run).resolve(), Path(args.output).resolve()
    result = source / 'result.json'
    if not result.is_file() or not source.is_dir() or output.exists():
        raise ValueError('A completed run and new output directory are required')
    output.mkdir(parents=True)
    for name in ('result.json', 'execution_sources.json', 'swarm_source_manifest.json', 'startup_clearance.json'):
        path = source / name
        if path.is_file():
            shutil.copy2(path, output / name)
    members = []
    directories = {'execution_sources', 'swarm_sources', 'models', 'algorithm'}
    for path in sorted(source.iterdir()):
        if path.name in directories or (path.is_file() and path.suffix in {'.json', '.jsonl', '.world', '.launch', '.log', '.sdf', '.png', '.py'}):
            members.append(path)
    archive = output / 'evidence.tar.gz'
    with tarfile.open(archive, 'w:gz') as stream:
        for path in members:
            stream.add(path, arcname=path.name)
    (output / 'archive_manifest.json').write_text(json.dumps(dict(
        source=str(source), archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
        members=[p.name for p in members],
        note='Executed source is the frozen run copy; subsequent workspace edits are not flight evidence.'), indent=2))
    print(json.dumps(dict(output=str(output), archive_bytes=archive.stat().st_size)))


if __name__ == '__main__':
    main()
