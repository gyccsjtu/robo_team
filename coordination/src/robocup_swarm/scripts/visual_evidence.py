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
    result = dict(target_id=tid, source_uav=uid, sample_s=sample, x=x, y=y,
                  confidence=confidence, eligible=eligible)
    switch = data.get('hypothesis_switch')
    if switch is not None:
        if not isinstance(switch, dict) or tid in ('t4', 't5') or not eligible:
            raise ValueError('invalid spatial hypothesis event')
        event = {k: float(switch[k]) for k in
                 ('switch_s', 'old_x', 'old_y', 'new_x', 'new_y', 'trial_until_s')}
        event['source_uav'] = str(switch['source_uav'])
        age = sample-event['switch_s']
        if (not all(math.isfinite(event[k]) for k in event if k != 'source_uav') or
                event['source_uav'] not in uav_ids or event['switch_s'] <= 0. or
                not 0. <= age < 25. or
                abs(event['trial_until_s']-event['switch_s']-25.) > 1e-6 or
                math.hypot(event['new_x']-event['old_x'],
                           event['new_y']-event['old_y']) <= 10. or
                math.hypot(x-event['new_x'], y-event['new_y']) > 3.+2.3*age):
            raise ValueError('stale or inconsistent spatial hypothesis event')
        result['hypothesis_switch'] = event
    return result
