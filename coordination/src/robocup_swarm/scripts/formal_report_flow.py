"""Passive evidence of our own report flow, never judge success or accuracy.

Images are the parsed ``visual_evidence`` records (``eligible is True``).
Only a real ``published`` telemetry message matched to the same camera,
candidate and original image can update the book. This module has no ROS,
publishing, planning, ownership or elimination side effects.
"""
import math


_TAGS = {'t0': 'green', 't1': 'blue', 't2': 'brown', 't3': 'white',
         't4': 'red2', 't5': 'red1'}
_IMAGE_TTL = 2.0
_PENDING_TTL = .5
_LIMIT = 16
_EPS = 1e-9


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('not a finite number')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('not a finite number')
    return value


class ReportFlowHealth:
    """A bounded diagnostic book. ``fresh`` grants no control/report authority."""

    def __init__(self, uav_ids):
        self.uav_ids = frozenset(uav_ids)
        self._images = {tid: {} for tid in _TAGS}
        self._pending = {tid: {} for tid in _TAGS}
        self._sources = {tid: {} for tid in _TAGS}
        self._received = {tid: {} for tid in _TAGS}  # One watermark per legal UAV.
        self._water = {}  # Matched image/report high-water marks survive reset.
        self._barriers = {}
        self._latest = {}
        self._now = 0.

    def _time(self, now):
        now = _number(now)
        if now < 0. or now < self._now-_EPS:
            raise ValueError('invalid or reversed diagnostic clock')
        self._now = max(self._now, now)
        for tid in _TAGS:
            self._images[tid] = {k: v for k, v in self._images[tid].items()
                                 if now-v['sample_s'] <= _IMAGE_TTL+_EPS}
            self._pending[tid] = {k: v for k, v in self._pending[tid].items()
                                  if now < v['deadline_s']}
        return now

    def _identity(self, data):
        if not isinstance(data, dict):
            raise ValueError('not a record')
        tid, source, candidate = (data[k] for k in
                                 ('target_id', 'source_uav', 'candidate_id'))
        if (tid not in _TAGS or source not in self.uav_ids or
                not isinstance(candidate, str) or not candidate.strip() or
                len(candidate) > 128):
            raise ValueError('unknown target, camera or candidate')
        sample = _number(data['sample_s'])
        if sample <= 0. or sample < self._barriers.get(tid, 0.):
            raise ValueError('image predates current task barrier')
        x, y = _number(data['x']), _number(data['y'])
        return tid, (source, candidate, sample), x, y

    def _parse_publish(self, payload, now):
        tid, key, x, y = self._identity(payload)
        if payload.get('event') != 'published':
            raise ValueError('not an actual report publication')
        if 'tag' in payload and payload['tag'] != _TAGS[tid]:
            raise ValueError('target/colour stream mismatch')
        policy = payload.get('report_policy')
        if policy not in ('fresh', 'fit'):
            raise ValueError('unknown report policy')
        report = _number(payload['report_s'])
        sample = key[2]
        limit = .4 if policy == 'fresh' else .6
        if (not -_EPS <= report-sample <= limit+_EPS or
                not -_EPS <= now-report <= .6+_EPS or
                not -_EPS <= now-sample <= limit+.6+_EPS or
                report < self._barriers.get(tid, 0.)):
            raise ValueError('old or future report/image')
        current = self._sources[tid].get(key[0])
        if current is not None and sample <= current[0] and key[1] != current[1]:
            raise ValueError('publication belongs to an already replaced instance')
        water = self._water.get(tid)
        if water is not None and (sample <= water[0] or report <= water[1]):
            raise ValueError('duplicate or reversed original publication')
        return dict(event='published', target_id=tid, tag=_TAGS[tid], source_uav=key[0],
                    candidate_id=key[1], sample_s=sample, report_s=report,
                    x=x, y=y, report_policy=policy)

    @staticmethod
    def _cap(book):
        while len(book) > _LIMIT:
            del book[next(iter(book))]

    def _match(self, payload, now):
        try:
            record = self._parse_publish(payload, now)
        except (KeyError, TypeError, ValueError):
            return False
        tid = record['target_id']
        key = (record['source_uav'], record['candidate_id'], record['sample_s'])
        image = self._images[tid].get(key)
        current = self._sources[tid].get(key[0])
        if (image is None or current is None or current[1] != key[1] or
                math.hypot(image['x']-record['x'], image['y']-record['y']) > .75+_EPS):
            return False
        record['matched_s'] = now
        self._water[tid] = (record['sample_s'], record['report_s'])
        self._latest[tid] = record
        return True

    def remember_image(self, data, now):
        """Remember one newly accepted image, and resolve pending telemetry.

        Returns whether a new eligible image was recorded. A duplicate cannot
        update camera freshness, identity or any pending message's deadline.
        """
        try:
            now = self._time(now)
            tid, key, x, y = self._identity(data)
            if (data.get('eligible') is not True or
                    not 0. <= now-key[2] <= _IMAGE_TTL+_EPS):
                return False
            current = self._sources[tid].get(key[0])
            if current is not None and key[2] <= current[0]:
                return False
        except (KeyError, TypeError, ValueError):
            return False
        self._sources[tid][key[0]] = (key[2], key[1])
        self._images[tid][key] = dict(sample_s=key[2], x=x, y=y)
        self._cap(self._images[tid])
        pending = self._pending[tid].pop(key, None)
        if pending is not None:
            self._match(pending['record'], now)
        return True

    def remember_publish(self, payload, now):
        """Return True only if this original publication matches an image."""
        try:
            now = self._time(now)
            record = self._parse_publish(payload, now)
        except (KeyError, TypeError, ValueError):
            return False
        tid = record['target_id']
        key = (record['source_uav'], record['candidate_id'], record['sample_s'])
        received = self._received[tid].get(key[0])
        if received is not None and (key[2] <= received[0] or
                                     record['report_s'] <= received[1]):
            return False
        # Receipt and successful matching have separate high-water marks.
        # Expiring an unmatched message must not license its replay to obtain
        # another deadline. _match deliberately checks only matched water.
        self._received[tid][key[0]] = (key[2], record['report_s'])
        pending = self._pending[tid]
        if key in pending:
            return False  # A repeated message cannot extend its .5 s budget.
        if key in self._images[tid]:
            return self._match(record, now)
        pending[key] = dict(record=record, deadline_s=now+_PENDING_TTL)
        self._cap(pending)
        return False

    def latest(self, tid, now):
        """Return last matched evidence with ages, including prolonged silence.

        None means no matched publication in this task. A stale record stays
        visible so diagnostics can distinguish thirty seconds of silence from
        never reporting. Neither the record nor ``fresh`` confirms accuracy.
        """
        try:
            now = self._time(now)
        except (TypeError, ValueError):
            return None
        if not isinstance(tid, str) or tid not in _TAGS:
            return None
        record = self._latest.get(tid)
        if record is None:
            return None
        result = dict(record)
        current = self._sources[tid].get(record['source_uav'])
        result['identity_current'] = bool(current and current[1] == record['candidate_id'])
        result['report_age_s'] = now-record['report_s']
        result['sample_age_s'] = now-record['sample_s']
        limit = .4 if record['report_policy'] == 'fresh' else .6
        result['fresh'] = (result['identity_current'] and
                           0. <= result['report_age_s'] <= .6+_EPS and
                           0. <= result['sample_age_s'] <= limit+_EPS)
        return result

    def reset(self, tid, barrier_s=None):
        """Start a new task/hypothesis without licensing old messages again."""
        if not isinstance(tid, str) or tid not in _TAGS:
            return
        barrier = self._now if barrier_s is None else _number(barrier_s)
        if barrier < 0.:
            raise ValueError('invalid reset barrier')
        # The caller can reset on a new hypothesis before remembering its
        # first image. Its validated switch time may exceed our last callback.
        self._now = max(self._now, barrier)
        self._barriers[tid] = max(self._barriers.get(tid, 0.), barrier)
        self._images[tid].clear()
        self._pending[tid].clear()
        self._latest.pop(tid, None)
