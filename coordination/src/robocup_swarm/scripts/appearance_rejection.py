"""Camera-instance negative evidence, never a colour or world-position ban."""
import json
import math


def parse_rejections(raw, now, allowed_sources=None):
    packet = json.loads(raw)
    source, sample = packet['source_uav'], float(packet['sample_s'])
    rows = packet['rejected_candidates']
    if (packet.get('schema_version') != 1 or
            packet.get('reason') != 'confirmed_secondary_distractor' or
            packet.get('real_detection') is not True or
            not isinstance(source, str) or not source or len(source) > 128 or
            (allowed_sources is not None and source not in allowed_sources) or
            not math.isfinite(sample) or not 0. < sample <= now+.05 or
            now-sample > .8 or not isinstance(rows, list) or len(rows) > 72):
        raise ValueError('invalid appearance rejection frame')
    result = []
    for row in rows:
        cid = row['candidate_id']
        confidence = float(row['confidence'])
        positive = float(row['last_positive_s'])
        times = [float(t) for t in row['evidence_samples']]
        if (not isinstance(cid, str) or not cid.strip() or len(cid) > 128 or
                not math.isfinite(confidence) or not .85 <= confidence <= 1. or
                not math.isfinite(positive) or not 0. < positive < sample or
                sample-positive > .8 or not 3 <= len(times) <= 32 or
                not all(math.isfinite(t) and t > 0. for t in times) or
                abs(times[-1]-sample) > 1e-6 or
                not .30-1e-6 <= times[-1]-times[0] <= 1.+1e-6 or
                any(not 0. < b-a <= .40+1e-6 for a,b in zip(times,times[1:]))):
            raise ValueError('unconfirmed or unscoped appearance rejection')
        result.append(dict(source_uav=source, candidate_id=cid, sample_s=sample))
    return result


class RejectionBook:
    """Block queued older positives; new image evidence may recover an instance.

    A retained entry does not prohibit a place, colour, another camera, or any
    newer positive image. Consumers serialize record/check with their state.
    """
    def __init__(self):
        self.samples = {}

    def record(self, event, now):
        self.samples = {k:t for k,t in self.samples.items() if now-t <= 30.}
        key = (event['source_uav'], event['candidate_id'])
        sample = event['sample_s']
        if sample <= self.samples.get(key, -math.inf):
            return False
        self.samples[key] = sample
        if len(self.samples) > 384:
            self.samples = dict(sorted(self.samples.items(), key=lambda kv:kv[1])[-384:])
        return True

    def blocks(self, source, candidate_id, sample):
        return sample <= self.samples.get((source, candidate_id), -math.inf)
