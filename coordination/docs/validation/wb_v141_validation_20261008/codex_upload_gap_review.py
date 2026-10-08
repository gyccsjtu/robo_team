"""Offline: derive upload spans and gaps from actual rows, never bucket edges."""
import argparse
import collections
import json
from pathlib import Path


def review(round_dir):
    path = Path(round_dir) / 'flight/algorithm/bridge_trace.jsonl'
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    by_tag = collections.defaultdict(list)
    for row in rows:
        if row.get('kind') == 'upload':
            by_tag[row['tag']].append(float(row['receipt_s']))
    spans = []
    for tag, samples in by_tag.items():
        samples.sort()
        spans.append(dict(tag=tag, count=len(samples), first=samples[0], last=samples[-1],
                          gaps_gt_1s=[dict(first=a, last=b, seconds=b-a)
                                     for a,b in zip(samples,samples[1:]) if b-a > 1.+1e-8]))
    spans.sort(key=lambda row: row['first'])
    gaps = []
    for first, second in zip(spans, spans[1:]):
        lo, hi = first['last'], second['first']
        if hi <= lo:
            continue
        window = [row for row in rows if lo < row.get('receipt_s', -1) < hi]
        inputs = [row for row in window if row.get('kind') in ('input','navigation_candidate')]
        skips = [row for row in window if row.get('kind') == 'skip']
        gaps.append(dict(first=lo, last=hi, seconds=hi-lo,
                         input_count=len(inputs),
                         input_colors=dict(collections.Counter(row.get('observation',{}).get('target_id')
                                                                for row in inputs)),
                         alive_inputs=sum(row.get('alive') is True for row in inputs),
                         skip_reasons=dict(collections.Counter('%s:%s' % (row.get('tag'),row.get('reason'))
                                                               for row in skips)),
                         uploads_in_gap=sum(row.get('kind') == 'upload' for row in window)))
    return dict(round=str(round_dir), upload_spans=spans, between_spans=gaps,
                meaning='Publication evidence only; not scheduling causality or exact judge replay')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--round', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    result = review(args.round)
    Path(args.out).write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
