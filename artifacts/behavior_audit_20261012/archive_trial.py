"""Archive a stopped, instrumented GUI trial without reading simulator truth."""
from pathlib import Path
import csv, gzip, hashlib, json, re, sys, tarfile

folder = Path(sys.argv[1])
manifest = json.loads((folder/'manifest.json').read_text())
marker = json.loads((folder/'finish_marker.json').read_text())
nearest = None
with gzip.open(folder/'sensors.jsonl.gz', 'rt') as stream:
    for line in stream:
        row = json.loads(line)
        distance = abs(row['wall'] - marker['judge_mtime'])
        if nearest is None or distance < nearest[0]:
            nearest = distance, row['ros']
marker.update(end_ros=nearest[1], sensor_wall_alignment_s=nearest[0])
(folder/'finish_marker.json').write_text(json.dumps(marker, indent=2)+'\n')
csv_dir = folder/'flight_csv'
csv_dir.mkdir(exist_ok=True)
for source in Path('/home/gycc/robocup_logs').glob('*.csv'):
    with source.open() as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or 'wall_time' not in reader.fieldnames:
            continue
        with (csv_dir/source.name).open('w') as output:
            writer = csv.DictWriter(output, fieldnames=reader.fieldnames)
            writer.writeheader()
            for row in reader:
                try:
                    if float(row['wall_time']) >= manifest['created_wall']:
                        writer.writerow(row)
                except (ValueError, TypeError):
                    pass
judge = (folder/'judge.txt').read_text()
matches = re.findall(r'actor_(\d) is OK\s+Time usage: ([\d.]+)', judge)
result = dict(runtime_commit=manifest['commit'], log_dir=manifest['log_dir'],
    finished='Mission finished' in judge, official_eliminated=len({a for a, _ in matches}),
    eliminations=[dict(actor=int(a), seconds=float(t)) for a, t in matches],
    completion_seconds=float(matches[-1][1]) if len({a for a, _ in matches}) == 6 else None,
    PR_FAR_DISCOVERY=manifest['PR_FAR_DISCOVERY'], gui=manifest['gui'])
(folder/'result.json').write_text(json.dumps(result, indent=2)+'\n')
with tarfile.open(folder/'runtime_evidence.tar.gz', 'w:gz') as archive:
    for source in Path(manifest['log_dir']).glob('*'):
        if source.is_file(): archive.add(source, arcname='logs/'+source.name)
    for source in csv_dir.glob('*.csv'):
        archive.add(source, arcname='flight_csv/'+source.name)
    index = folder/'matched_frames/frames.jsonl'
    if index.exists(): archive.add(index, arcname='native_images/frames.jsonl')
    for source in sorted((folder/'matched_frames').glob('*.png'))[:4]:
        archive.add(source, arcname='native_images/'+source.name)
records = []
for source in folder.rglob('*'):
    if source.is_file() and source.name != 'evidence_manifest.json':
        records.append(dict(path=str(source.relative_to(folder)), bytes=source.stat().st_size,
            sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
(folder/'evidence_manifest.json').write_text(json.dumps(records, indent=2)+'\n')
print(json.dumps(dict(result=result, end_ros=marker['end_ros']), indent=2))
