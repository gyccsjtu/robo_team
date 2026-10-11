#!/usr/bin/env python3
"""Prepare/audit reviewed six-class data, then warm-start the existing detector.

Manifest JSONL: image, label, group, split, reviewed=true, reviewer,
sha256 and label_sha256.
One recording/layout group must stay in one split. Test images are never used
for checkpoint selection. Unreviewed capture proposals are rejected.
"""
import argparse
from collections import Counter, defaultdict
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time

NAMES = ['red', 'green', 'blue', 'white', 'brown', 'distractor_person']
ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_labels(path):
    boxes = []
    for line in Path(path).read_text().splitlines():
        if not line.strip():
            continue
        parts = line.split()
        if len(parts) != 5:
            raise ValueError('Expected class cx cy width height: '+str(path))
        cls, cx, cy, w, h = map(float, parts)
        if not all(math.isfinite(x) for x in (cls, cx, cy, w, h)):
            raise ValueError('Non-finite label: '+str(path))
        if cls != int(cls) or not 0 <= cls < len(NAMES):
            raise ValueError('Invalid class: '+str(path))
        if (w <= 0 or h <= 0 or cx-w/2 < -1e-6 or cy-h/2 < -1e-6 or
                cx+w/2 > 1+1e-6 or cy+h/2 > 1+1e-6):
            raise ValueError('Box outside image: '+str(path))
        boxes.append((int(cls), cx, cy, w, h))
    return boxes


def audit_manifest(path, require_complete=True):
    base = Path(path).resolve().parent
    records, groups, hashes, counts = [], {}, {}, defaultdict(Counter)
    for line_no, line in enumerate(Path(path).read_text().splitlines(), 1):
        if not line.strip():
            continue
        row = json.loads(line)
        if row.get('reviewed') is not True or not str(row.get('reviewer', '')).strip():
            raise ValueError('Unreviewed frame on manifest line %d' % line_no)
        group, split = row.get('group'), row.get('split')
        if not isinstance(group, str) or not group or split not in ('train', 'val', 'test'):
            raise ValueError('Missing collection group or split')
        if group in groups and groups[group] != split:
            raise ValueError('Recording/layout group leaks across splits: '+group)
        groups[group] = split
        image, label = base/row['image'], base/row['label']
        if not image.is_file() or not label.is_file():
            raise ValueError('Missing image/label on line %d' % line_no)
        digest = hashlib.sha256(image.read_bytes()).hexdigest()
        if digest != row.get('sha256'):
            raise ValueError('Reviewed image checksum changed: '+str(image))
        if hashlib.sha256(label.read_bytes()).hexdigest() != row.get('label_sha256'):
            raise ValueError('Reviewed label checksum changed: '+str(label))
        if digest in hashes:
            raise ValueError('Duplicate image (including cross-split leakage): '+str(image))
        hashes[digest] = split
        boxes = read_labels(label)
        counts[split].update(box[0] for box in boxes)
        row = dict(row, image=str(image.resolve()), label=str(label.resolve()))
        records.append(row)
    if not records:
        raise ValueError('Empty dataset')
    missing = ['%s:%s' % (split, name) for split in ('train', 'val', 'test')
               for i, name in enumerate(NAMES) if counts[split][i] == 0]
    if require_complete and missing:
        raise ValueError('Each class needs independently reviewed train/val/test data; missing '+', '.join(missing))
    return records, {'images': len(records), 'groups': len(groups),
                     'counts': {s: {NAMES[k]: v for k, v in c.items()} for s, c in counts.items()},
                     'missing_classes': missing}


def prepare(manifest, output):
    records, report = audit_manifest(manifest)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    for i, row in enumerate(records):
        split = row['split']
        for kind in ('images', 'labels'):
            (output/kind/split).mkdir(parents=True, exist_ok=True)
        # Symlinks avoid duplicating the dataset and never alter source labels.
        name = '%06d' % i
        (output/'images'/split/(name+Path(row['image']).suffix)).symlink_to(row['image'])
        (output/'labels'/split/(name+'.txt')).symlink_to(row['label'])
    data = dict(path=str(output), train='images/train', val='images/val',
                test='images/test', names=dict(enumerate(NAMES)))
    import yaml
    (output/'data.yaml').write_text(yaml.safe_dump(data, sort_keys=False))
    (output/'reviewed_manifest.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    (output/'audit.json').write_text(json.dumps(report, indent=2)+'\n')
    return output/'data.yaml'


def audit_prepared_dataset(data):
    """Audit the files YOLO will actually open, not only a nearby manifest."""
    import yaml
    data = Path(data).resolve()
    base = data.parent
    config = yaml.safe_load(data.read_text())
    expected = dict(path=str(base), train='images/train', val='images/val',
                    test='images/test', names=dict(enumerate(NAMES)))
    if config != expected:
        raise ValueError('Prepared dataset YAML must exactly match local paths and six-class names; extra fields forbidden')
    manifest = base/'reviewed_manifest.jsonl'
    rows, report = audit_manifest(manifest)
    expected_files = defaultdict(set)
    for i, row in enumerate(rows):
        for kind, key, suffix in (('images', 'image', Path(row['image']).suffix),
                                  ('labels', 'label', '.txt')):
            folder = base/kind/row['split']
            item = folder/('%06d%s' % (i, suffix))
            expected_files[folder].add(item)
            if not item.is_file() or item.resolve() != Path(row[key]).resolve():
                raise ValueError('Prepared image/label differs from reviewed source: '+str(item))
    for folder, expected_set in expected_files.items():
        if set(folder.iterdir()) != expected_set:
            raise ValueError('Extra or missing files in prepared dataset: '+str(folder))
    evidence = dict(audit_wall_s=time.time(), data=str(data), data_sha256=sha256(data),
                    manifest=str(manifest), manifest_sha256=sha256(manifest),
                    names=NAMES, summary=report,
                    splits={split: [dict(sha256=r['sha256'], label_sha256=r['label_sha256'],
                                         group=r['group']) for r in rows if r['split']==split]
                            for split in ('train', 'val', 'test')})
    return rows, evidence


def finalize_provenance(run_dir, input_audit, timing='before_training'):
    """Bind a finished checkpoint to verified input files and the actual run args.

    Older in-flight runs can be audited during training, but that timing is
    recorded explicitly; this does not claim that an earlier audit occurred.
    """
    import yaml
    from ultralytics import YOLO
    run_dir = Path(run_dir).resolve()
    before = json.loads(Path(input_audit).read_text())
    args = yaml.safe_load((run_dir/'args.yaml').read_text())
    if args.get('resume') or args.get('fraction', 1.0) != 1.0 or args.get('split', 'val') != 'val':
        raise ValueError('Unsupported actual run split, fraction or resume provenance')
    _, after = audit_prepared_dataset(args['data'])
    for key in ('data', 'data_sha256', 'manifest_sha256', 'splits', 'names'):
        if before[key] != after[key]:
            raise ValueError('Dataset changed since input audit: '+key)
    checkpoints = {}
    for name in ('best.pt', 'last.pt'):
        path = run_dir/'weights'/name
        if not path.is_file():
            continue
        model = YOLO(str(path))
        if list(model.names.values()) != NAMES:
            raise ValueError('Candidate checkpoint is not the required six-class model')
        checkpoint_args = model.ckpt.get('train_args', {})
        if Path(checkpoint_args.get('data', '')).resolve() != Path(after['data']):
            raise ValueError('Checkpoint was trained with another data configuration')
        if Path(checkpoint_args.get('model', '')).resolve() != Path(args['model']).resolve():
            raise ValueError('Checkpoint warm-start differs from recorded training args')
        checkpoints[name] = sha256(path)
    if 'best.pt' not in checkpoints:
        raise ValueError('No completed best checkpoint')
    path = run_dir/'training_provenance.json'
    record = json.loads(path.read_text()) if path.exists() else {}
    record.update(provenance_version=2, input_audit_timing=timing,
                  input_audit_sha256=sha256(input_audit), dataset_inputs=after,
                  initial_input_audit_wall_s=before['audit_wall_s'],
                  args_sha256=sha256(run_dir/'args.yaml'), actual_arguments=args,
                  checkpoint_sha256=checkpoints, class_names=NAMES,
                  warmstart_sha256=sha256(args['model']), finalized_wall_s=time.time())
    path.write_text(json.dumps(record, indent=2)+'\n')
    return path


def verify_candidate_provenance(weights, holdout_rows, independent=True):
    weights = Path(weights).resolve()
    path = weights.parent.parent/'training_provenance.json'
    if not path.is_file():
        raise ValueError('Candidate lacks verified training provenance')
    record = json.loads(path.read_text())
    if (record.get('provenance_version') != 2 or record.get('class_names') != NAMES or
            record.get('checkpoint_sha256', {}).get(weights.name) != sha256(weights)):
        raise ValueError('Candidate checkpoint/class provenance mismatch')
    inputs = record['dataset_inputs']
    forbidden = {'train', 'val'} if independent else {'train'}
    used = {r['sha256'] for s in forbidden for r in inputs['splits'][s]}
    if used.intersection(r['sha256'] for r in holdout_rows):
        raise ValueError('Evaluation images were used for training or checkpoint selection')
    return dict(path=str(path), sha256=sha256(path), independent_test=independent,
                input_audit_timing=record['input_audit_timing'])


def import_review(review_json, output):
    """Turn explicitly accepted image reviews into checksummed YOLO labels."""
    from PIL import Image
    rows = json.loads(Path(review_json).read_text())
    accepted = [r for r in rows if r.get('reviewed') is True and r.get('review_status') == 'accepted']
    if not accepted:
        raise ValueError('No explicitly accepted frames in review')
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    records=[]
    for i,row in enumerate(accepted):
        image=Path(row['image']).resolve()
        if not str(row.get('reviewer','')).strip():
            raise ValueError('Accepted frame needs reviewer name')
        if hashlib.sha256(image.read_bytes()).hexdigest()!=row['sha256']:
            raise ValueError('Reviewed image checksum changed')
        with Image.open(image) as im:
            w,h=im.size
        if (row.get('width'),row.get('height'))!=(w,h):
            raise ValueError('Review dimensions differ from original image')
        lines=[]
        for b in row['boxes']:
            cls=b['class_id'];x1,y1,x2,y2=map(float,b['xyxy'])
            if (not isinstance(cls,int) or isinstance(cls,bool) or not 0<=cls<6 or
                    not all(math.isfinite(x) for x in (x1,y1,x2,y2)) or
                    not 0<=x1<x2<=w or not 0<=y1<y2<=h):
                raise ValueError('Invalid reviewed box')
            lines.append('%d %.8f %.8f %.8f %.8f\n' % (cls,(x1+x2)/2/w,(y1+y2)/2/h,(x2-x1)/w,(y2-y1)/h))
        label=output/('%06d.txt'%i)
        label.write_text(''.join(lines))
        records.append(dict(image=str(image),label=str(label),group=row['group'],split=row['split'],
                            reviewed=True,reviewer=row['reviewer'],sha256=row['sha256'],
                            label_sha256=hashlib.sha256(label.read_bytes()).hexdigest()))
    manifest=output/'reviewed_manifest.jsonl'
    manifest.write_text(''.join(json.dumps(r)+'\n' for r in records))
    audit_manifest(manifest,require_complete=False)
    return manifest


def probe_device(device):
    import torch
    if device == 'cpu':
        return dict(torch=torch.__version__, device='cpu')
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA unavailable; choose --device cpu explicitly or fix the isolated training environment')
    index = int(str(device).split(',')[0])
    # is_available() alone returns true even for an unsupported sm_120 wheel.
    x = torch.ones((2, 2), device='cuda:%d' % index)
    value = float((x@x).sum().item())
    del x
    torch.cuda.empty_cache()
    return dict(torch=torch.__version__, cuda=torch.version.cuda,
                device=torch.cuda.get_device_name(index), small_tensor=value)


def train(args):
    data = Path(args.data).resolve()
    _, input_audit = audit_prepared_dataset(data)
    if not Path(args.weights).is_file():
        raise ValueError('Warm-start checkpoint does not exist: '+args.weights)
    if shutil.disk_usage(Path(args.project).resolve().parent).free < 1024**3:
        raise RuntimeError('At least 1 GiB free space is required before training')
    os.environ.setdefault('CUBLAS_WORKSPACE_CONFIG', ':4096:8')
    device = probe_device(args.device)
    from ultralytics import YOLO
    import torch
    model = YOLO(args.weights)
    if list(model.names.values()) != NAMES[:5]:
        raise ValueError('Warm-start weights must use the original five-class order')
    experiment = Path(args.project).resolve()/args.name
    if experiment.exists():
        raise ValueError('Refusing to overwrite a prior training run: '+str(experiment))
    input_path = experiment.parent/(args.name+'_input_audit.json')
    input_path.write_text(json.dumps(input_audit, indent=2)+'\n')
    config = dict(data=str(data), epochs=args.epochs, imgsz=640, batch=args.batch,
                  workers=2, device=args.device, project=str(experiment.parent),
                  name=args.name, plots=False, val=True, patience=12, seed=0,
                  cache=False, exist_ok=False, save=True, save_period=-1,
                  # Hue changes can relabel the competition's color classes.
                  hsv_h=0.0, fliplr=.5, flipud=0., close_mosaic=10)
    record = dict(start_wall_s=time.time(), warmstart=str(Path(args.weights).resolve()),
                  warmstart_sha256=hashlib.sha256(Path(args.weights).read_bytes()).hexdigest(),
                  device=device, dataset=input_audit['summary'], arguments=config)
    try:
        model.train(**config)
    except (torch.cuda.OutOfMemoryError, RuntimeError) as exc:
        if 'out of memory' not in str(exc).lower() or args.batch != 8:
            raise
        del model
        gc.collect(); torch.cuda.empty_cache()
        config.update(batch=4, name=args.name+'_batch4')
        if (experiment.parent/config['name']).exists():
            raise ValueError('OOM retry output already exists')
        record['oom_retry_batch'] = 4
        model = YOLO(args.weights)
        model.train(**config)
    save_dir = Path(model.trainer.save_dir)
    record.update(finish_wall_s=time.time(), save_dir=str(save_dir), final_batch=config['batch'])
    (save_dir/'training_provenance.json').write_text(json.dumps(record, indent=2)+'\n')
    finalize_provenance(save_dir, input_path)
    print(json.dumps(record, indent=2))
    print('Candidate weights only; independent actor-recall/decoy and live acceptance required before deployment.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    a = sub.add_parser('audit')
    a.add_argument('--manifest', required=True)
    a.add_argument('--allow-incomplete', action='store_true')
    ap = sub.add_parser('audit-prepared')
    ap.add_argument('--data', required=True); ap.add_argument('--output', required=True)
    fp = sub.add_parser('finalize-provenance')
    fp.add_argument('--run', required=True); fp.add_argument('--input-audit', required=True)
    fp.add_argument('--timing', choices=['before_training', 'during_training'], required=True)
    b = sub.add_parser('prepare')
    b.add_argument('--manifest', required=True); b.add_argument('--output', required=True)
    r = sub.add_parser('import-review')
    r.add_argument('--review', required=True); r.add_argument('--output', required=True)
    t = sub.add_parser('train')
    t.add_argument('--data', required=True)
    t.add_argument('--weights', default=str(ROOT/'weights/best_yolo11n_bino_v1.pt'))
    t.add_argument('--project', required=True)
    t.add_argument('--name', default='yolo11n_bino_distractor_v3')
    t.add_argument('--device', default='0')
    t.add_argument('--epochs', type=int, default=60)
    t.add_argument('--batch', type=int, choices=[4, 8], default=8)
    q = sub.add_parser('probe'); q.add_argument('--device', default='0')
    args = p.parse_args()
    if args.action == 'audit':
        print(json.dumps(audit_manifest(args.manifest, not args.allow_incomplete)[1], indent=2))
    elif args.action == 'prepare':
        print(prepare(args.manifest, args.output))
    elif args.action == 'import-review':
        print(import_review(args.review, args.output))
    elif args.action == 'probe':
        print(json.dumps(probe_device(args.device), indent=2))
    elif args.action == 'audit-prepared':
        evidence = audit_prepared_dataset(args.data)[1]
        Path(args.output).write_text(json.dumps(evidence, indent=2)+'\n')
        print(json.dumps(evidence['summary'], indent=2))
    elif args.action == 'finalize-provenance':
        print(finalize_provenance(args.run, args.input_audit, args.timing))
    else:
        if not 1 <= args.epochs <= 60:
            p.error('epochs must be between 1 and 60')
        train(args)


if __name__ == '__main__':
    main()
