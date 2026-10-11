"""Camera evidence shared by dispatch and local tracking; no geometry inference."""
import json
import math

VISUAL_TTL = 1.0


def parse_visual_evidence(raw, now, uav_ids):
    data = json.loads(raw)
    if data.get('real_detection') is not True:
        raise ValueError('not a real camera detection')
    tid, uid = str(data['target_id']), str(data['source_uav'])
    sample, x, y = (float(data[k]) for k in ('sample_s', 'x', 'y'))
    confidence = float(data['confidence'])
    if (tid not in ('t0', 't1', 't2', 't3', 't4', 't5') or uid not in uav_ids or
            not all(math.isfinite(v) for v in (sample, x, y, confidence)) or
            sample <= 0 or not 0.0 <= now - sample <= VISUAL_TTL or
            not 0.0 < confidence <= 1.0):
        raise ValueError('invalid or stale camera sample')
    rng = data.get('range_m')
    rng = float(rng) if rng is not None else None
    eligible = (data.get('official_view_ok') is True and rng is not None and
                math.isfinite(rng) and 0.0 < rng <= 12.0)
    return dict(target_id=tid, source_uav=uid, sample_s=sample, x=x, y=y,
                confidence=confidence, eligible=eligible)
