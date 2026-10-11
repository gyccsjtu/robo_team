"""Single-camera BoT-SORT adapter; identity never grants reporting quality.

The pinned tracker uses frame steps internally. This adapter limits retention
by original image seconds, isolates clothing classes in every matching stage,
and only attaches identities to detections actually matched in this image.
"""
import math
import importlib
import uuid
from types import SimpleNamespace

import numpy as np


class Detections:
    """Minimal non-mutating Results-like input in original image coordinates."""

    def __init__(self, rows):
        self.rows = np.asarray(rows, dtype=np.float32).reshape(-1, 6)

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, index):
        return Detections(self.rows[index])

    @property
    def xyxy(self):
        return self.rows[:, :4].copy()

    @property
    def xywh(self):
        result = self.xyxy
        result[:, :2] = (self.rows[:, :2] + self.rows[:, 2:4]) / 2.
        result[:, 2:4] = self.rows[:, 2:4] - self.rows[:, :2]
        return result

    @property
    def conf(self):
        return self.rows[:, 4]

    @property
    def cls(self):
        return self.rows[:, 5]


class _FrameWarp:
    """Use one background estimate for all color trackers in the same image."""
    method = 'shared_sparseOptFlow'

    def __init__(self, warp):
        self.warp = warp

    def apply(self, image, boxes):
        return self.warp


class ImageMOT:
    def __init__(self, classes, source, session=None, low_conf=.10,
                 class_thresholds=None, max_absence_s=3.0, max_frame_gap_s=3.0,
                 gmc_method='sparseOptFlow', tracker_factory=None, gmc_factory=None):
        if not 0. < low_conf < 1. or max_absence_s <= 0. or max_frame_gap_s <= 0.:
            raise ValueError('Invalid image tracking thresholds')
        if tracker_factory is None:
            prefix = (__package__+'.' if __package__ else '')+'vendor.ultralytics_trackers.'
            tracker_factory = importlib.import_module(prefix+'bot_sort').BOTSORT
        if gmc_factory is None:
            prefix = (__package__+'.' if __package__ else '')+'vendor.ultralytics_trackers.'
            gmc_factory = importlib.import_module(prefix+'utils.gmc').GMC
        self.classes, self.source = tuple(classes), str(source)
        self.session = session or uuid.uuid4().hex[:12]
        self.low_conf = float(low_conf)
        self.thresholds = class_thresholds or {c: .40 for c in self.classes}
        self.max_absence_s, self.max_frame_gap_s = max_absence_s, max_frame_gap_s
        self._tracker_factory, self._gmc_factory = tracker_factory, gmc_factory
        self.gmc_method = gmc_method
        self.last_sample = -1.
        self.last_cue_sample = -1.
        self.epoch = 0
        self.observations = {}
        self.cue_history = {}
        self.stats = dict(frames=0, resets=0, matched=0, gmc_failures=0)
        self._initialize()

    def _initialize(self):
        self.trackers = {}
        for cls in self.classes:
            threshold = float(self.thresholds[cls])
            if not self.low_conf < threshold <= 1.:
                raise ValueError('Low association confidence must be below class creation threshold')
            args = SimpleNamespace(track_high_thresh=threshold,
                track_low_thresh=self.low_conf, new_track_thresh=threshold,
                # The adapter, not a nominal update rate, expires IDs by
                # original image seconds in _expire().
                track_buffer=float('inf'), match_thresh=.8, fuse_score=True,
                gmc_method='none', proximity_thresh=.5,
                appearance_thresh=.8, with_reid=False, model='auto')
            self.trackers[cls] = self._tracker_factory(args)
        self.gmc = self._gmc_factory(method=self.gmc_method)

    def _identity(self, cls, track_id):
        return '%s:%s:%d:%d' % (cls, self.session, self.epoch, int(track_id))

    def _expire(self, cls, tracker, sample_s):
        for pool in ('tracked_stracks', 'lost_stracks'):
            keep = []
            for track in getattr(tracker, pool):
                key = self._identity(cls, track.track_id)
                seen = self.observations.get(key, -float('inf'))
                if sample_s - seen <= self.max_absence_s:
                    keep.append(track)
                else:
                    track.mark_removed()
                    self.cue_history.pop(key, None)
            setattr(tracker, pool, keep)

    def annotate(self, image, boxes, sample_s):
        """Attach image_track_id in-place to project-owned box adapters only.

        Inputs originate from ActorDistractorResolver, not YOLO tensors. No
        tracker-predicted bbox is returned as a real detection or projection.
        """
        boxes = list(boxes)
        for box in boxes:
            box.image_track_id = None
        if not math.isfinite(sample_s) or sample_s <= self.last_sample:
            return False
        if self.last_sample >= 0. and sample_s-self.last_sample > self.max_frame_gap_s:
            self.epoch += 1
            self.stats['resets'] += 1
            self.observations.clear()
            self.cue_history.clear()
            self._initialize()
        self.last_sample = float(sample_s)
        self.stats['frames'] += 1
        groups = {c: [] for c in self.classes}
        all_boxes = []
        for i, box in enumerate(boxes):
            cid, confidence = int(box.cls), float(box.conf)
            xyxy = [float(x) for x in box.xyxy[0]]
            if (not 0 <= cid < len(self.classes) or
                    not all(math.isfinite(x) for x in xyxy+[confidence]) or
                    not self.low_conf <= confidence <= 1. or
                    xyxy[2] <= xyxy[0] or xyxy[3] <= xyxy[1]):
                continue
            groups[self.classes[cid]].append((i, xyxy+[confidence, cid]))
            all_boxes.append(xyxy)
        try:
            warp = self.gmc.apply(image, np.asarray(all_boxes, dtype=np.float32).reshape(-1, 4))
            if warp is None or np.shape(warp) != (2, 3) or not np.isfinite(warp).all():
                raise ValueError('Invalid camera motion estimate')
        except Exception:
            # Identity is a degraded estimate, never proof of valid geometry.
            self.stats['gmc_failures'] += 1
            warp = np.eye(2, 3, dtype=np.float32)
        for cls, members in groups.items():
            tracker = self.trackers[cls]
            self._expire(cls, tracker, sample_s)
            tracker.gmc = _FrameWarp(warp)
            tracker.update(Detections([row for _, row in members]), image)
            # Include tentative tracks as image identity only. World creation
            # still has strict geometry, confidence and independent hit gates.
            used_indices = set()
            for track in tracker.tracked_stracks:
                if track.frame_id != tracker.frame_id:
                    continue
                index = int(track.idx)
                if not 0 <= index < len(members) or index in used_indices:
                    continue
                used_indices.add(index)
                key = self._identity(cls, track.track_id)
                boxes[members[index][0]].image_track_id = key
                self.observations[key] = float(sample_s)
                self.stats['matched'] += 1
        self.observations = {k: t for k, t in self.observations.items()
                             if sample_s-t <= self.max_absence_s}
        return True

    def cues(self, sample_s, boxes):
        """Build bounded pixel attention from actual eligible observations."""
        result = []
        if sample_s != self.last_sample or sample_s <= self.last_cue_sample:
            return result
        self.last_cue_sample = float(sample_s)
        for box in boxes:
            key = box.get('image_track_id')
            if not key or self.observations.get(key) != sample_s:
                continue
            history = [p for p in self.cue_history.get(key, [])
                       if 0. <= sample_s-p[0] <= 2.5]
            if history and sample_s-history[-1][0] > 1.2:
                history = []
            if not history or sample_s > history[-1][0]:
                history.append((float(sample_s), float(box['conf'])))
            self.cue_history[key] = history
            # Keep the shortest sufficient real-image sequence.
            first = 0
            for start in range(max(0, len(history)-2)):
                if history[-1][0]-history[start][0] < .3-1e-9:
                    break
                first = start
            history = history[first:]
            if len(history) < 3 or history[-1][0]-history[0][0] < .3-1e-9:
                continue
            normal = min(p[1] for p in history) >= .85
            bounded = min(p[1] for p in history) >= .45 and box['conf'] >= .55
            if not (normal or bounded):
                continue
            cue = dict(candidate_id=key, sample_s=float(sample_s),
                bearing=box['bearing'], confidence=box['conf'], hits=len(history),
                u=box['u'], v=box['v'], attention_only=(not normal or
                box.get('appearance_status', 'actor') != 'actor'))
            for field in ('box', 'width', 'height', 'camera_tilt',
                          'appearance_status', 'appearance_confidence',
                          'distractor_confidence'):
                if field in box:
                    cue[field] = box[field]
            result.append(cue)
        self.cue_history = {k: h for k, h in self.cue_history.items()
                            if h and sample_s-h[-1][0] <= self.max_absence_s}
        return result


def image_world_matches(tracks, detections, sample_s, confirm_hits, height_gate,
                        base_gate=3.0, gate_min=1.5, gate_mult=3.0, max_gap_s=3.0):
    """Bind known image identity; use a reachable set from last real position.

    A second independent greedy position association must not swap MOT IDs.
    Reachability remains bounded and rejects large projection jumps even when
    a tracker keeps the same ID. Ambiguous duplicate identities match neither.
    """
    track_ids, detection_ids = {}, {}
    for t in tracks:
        key = getattr(t, 'image_track_id', None)
        if key:
            track_ids.setdefault(key, []).append(t)
    for index, d in enumerate(detections):
        if d.get('image_track_id'):
            detection_ids.setdefault(d['image_track_id'], []).append(index)
    result = {}
    for key, old in track_ids.items():
        current = detection_ids.get(key, [])
        if len(old) != 1 or len(current) != 1:
            continue
        t, j = old[0], current[0]
        d = detections[j]
        gap = sample_s-t.observed_s
        if (not 0. < gap <= max_gap_s or t.cls != d['cls'] or
                (t.hits < confirm_hits and not d['strict']) or
                abs(d['h']-t.h) > height_gate):
            continue
        x, y = t.observation_xy()
        distance = math.hypot(d['xyz'][0]-x, d['xyz'][1]-y)
        # Preserve the existing short-image-gap projection jump guard. The
        # last real position (instead of a possibly wrong coast prediction)
        # allows a turn after occlusion without widening its radius.
        gate = (max(gate_min, min(base_gate, gate_min+gate_mult*gap))
                if t.hits >= confirm_hits else base_gate)
        if math.isfinite(distance) and distance < gate:
            result[t.id] = (j, distance, gate)
    return result
