"""Independent appearance evidence; never changes the primary box or color.

The verifier sees the same full native image. Its geometry is used only to
associate one physical instance, not to locate an actor or replace v1 boxes.
No motion, world position, referee state, or fixed object coordinates enter.
"""
import math


def overlap(a, b):
    iw = max(0., min(a[2], b[2])-max(a[0], b[0]))
    ih = max(0., min(a[3], b[3])-max(a[1], b[1]))
    union = ((a[2]-a[0])*(a[3]-a[1]) +
             (b[2]-b[0])*(b[3]-b[1]) - iw*ih)
    return iw*ih/max(1e-12, union)


def same_instance(a, b):
    """Two detectors can regress different edges of the same visible body."""
    if not all(math.isfinite(v) for v in (*a, *b)):
        return False
    aw, ah, bw, bh = a[2]-a[0], a[3]-a[1], b[2]-b[0], b[3]-b[1]
    if min(aw, ah, bw, bh) <= 0.:
        return False
    return (overlap(a, b) >= .75 and .65 <= aw/bw <= 1.5 and
            .65 <= ah/bh <= 1.5 and
            abs(a[0]+a[2]-b[0]-b[2]) <= .3*min(aw, bw) and
            abs(a[1]+a[3]-b[1]-b[3]) <= .3*min(ah, bh))


class AppearanceVerifier:
    """Apply unique negative evidence with bounded temporal association.

    >=.50 negative confidence with >=.20 margin against the verifier's actor
    classes creates an ambiguous candidate (attention only). Three >=.75
    negatives spanning .30s classify a distractor. Scores from two different
    models are never subtracted. Unmatched and spatially ambiguous pairs do
    not veto original actors; a nearby actor must not inherit the veto.
    """
    def __init__(self):
        self.history = []
        self.last_sample = -float('inf')

    def apply(self, primary_boxes, verified_boxes, verified_names, sample_s):
        if not math.isfinite(sample_s):
            raise ValueError('Nonfinite image timestamp')
        labels = (dict(enumerate(verified_names)) if isinstance(verified_names, (tuple, list))
                  else verified_names)
        decoy_ids = {int(i) for i, name in labels.items() if name == 'distractor_person'}
        if len(decoy_ids) != 1:
            raise ValueError('Appearance verifier requires one distractor_person class')
        rows = [(int(b.cls), tuple(float(v) for v in b.xyxy[0]), float(b.conf))
                for b in verified_boxes if math.isfinite(float(b.conf))]
        decoys = [r for r in rows if r[0] in decoy_ids and r[2] >= .50]
        actors = [r for r in rows if r[0] not in decoy_ids and r[2] >= .30]
        primary = list(primary_boxes)
        boxes = [tuple(float(v) for v in b.xyxy[0]) for b in primary]
        matches = [[j for j, (_, db, _) in enumerate(decoys) if same_instance(ab, db)]
                   for ab in boxes]
        owners = {j: sum(j in choice for choice in matches) for j in range(len(decoys))}
        fresh = sample_s > self.last_sample
        previous = [h for h in self.history if 0. < sample_s-h['sample'] <= .6]
        history_owners = {j: sum(int(b.cls) == h['cls'] and overlap(ab, h['box']) >= .5
                                for b, ab in zip(primary, boxes))
                          for j, h in enumerate(previous)}
        following, used, kept, evidence = [], set(), [], []
        for i, (actor, ab) in enumerate(zip(primary, boxes)):
            if len(matches[i]) != 1 or owners[matches[i][0]] != 1:
                kept.append(actor)
                continue
            _, db, dc = decoys[matches[i][0]]
            ac = max((cf for _, bb, cf in actors if same_instance(ab, bb)), default=0.)
            if dc-ac < .20-1e-9:
                kept.append(actor)
                continue
            actor.appearance_status = 'ambiguous'
            actor.distractor_confidence = dc
            samples = []
            candidates = [(j, h) for j, h in enumerate(previous) if j not in used and
                          history_owners[j] == 1 and h['cls'] == int(actor.cls) and
                          overlap(ab, h['box']) >= .5]
            if fresh and dc >= .75:
                if len(candidates) == 1:
                    j, history = candidates[0]
                    used.add(j)
                    samples = [s for s in history['samples'] if sample_s-s <= 1.1]
                samples.append(sample_s)
                following.append(dict(cls=int(actor.cls), box=ab, sample=sample_s,
                                      samples=samples[-3:]))
            if len(samples) >= 3 and samples[-1]-samples[-3] >= .30-1e-9:
                actor.appearance_status = 'distractor'
            evidence.append(dict(box=list(ab), verifier_box=list(db), confidence=dc,
                                 actor_confidence=ac, status=actor.appearance_status))
            if actor.appearance_status != 'distractor':
                kept.append(actor)
        if fresh:
            self.history, self.last_sample = following, sample_s
        return kept, evidence
