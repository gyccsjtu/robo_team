"""Bounded, image-only optical-flow tracker for shadow evaluation.

This module cannot publish, locate a person in world coordinates, or authorize
flight. A tracked rectangle is *not* a measured footpoint. In particular all
eligibility flags stay false even when optical flow succeeds. YOLO remains the
only source of class/identity anchors; optical-flow updates never refresh its
age or confidence. This estimates apparent image motion (including camera
motion), not a person's world velocity.
"""
import math
import time
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class _Track:
    identity: str
    label: str
    box: np.ndarray
    seed_box: np.ndarray
    polygon: np.ndarray
    confidence: float
    seed_s: float
    sample_s: float
    gray: np.ndarray
    points: np.ndarray
    template: np.ndarray
    histogram: np.ndarray


class FastVisualTracker:
    """Track detector-anchored IDs for at most ``max_age_s`` seconds.

    ``seed`` accepts dicts with id, label, box (xyxy), and confidence. It only
    corrects the listed IDs. Use ``step(image, t)`` before ``seed(image, t, ...)``
    when both operations refer to the same image. Repeated/older frames do not
    modify tracking state. Lost tracks require a new genuine detector seed.
    """

    def __init__(self, max_age_s=0.6, max_gap_s=0.3, max_tracks=6):
        if not (math.isfinite(max_age_s) and max_age_s > 0 and
                math.isfinite(max_gap_s) and max_gap_s > 0):
            raise ValueError("tracking time limits must be finite and positive")
        if isinstance(max_tracks, bool) or int(max_tracks) != max_tracks or max_tracks < 1:
            raise ValueError("max_tracks must be a positive integer")
        self.max_age_s = float(max_age_s)
        self.max_gap_s = float(max_gap_s)
        self.max_tracks = int(max_tracks)
        self._tracks = {}
        self._latest_s = -math.inf
        self._last_seed_s = -math.inf

    @staticmethod
    def _image(image):
        if (not isinstance(image, np.ndarray) or image.dtype != np.uint8 or
                image.ndim != 3 or image.shape[2] != 3 or min(image.shape[:2]) < 2):
            raise ValueError("image must be a complete uint8 BGR image")
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    @staticmethod
    def _result(identity, label, box, sample_s, seed_s, status, reason,
                quality=0., processing_ms=0., real_detection=False):
        raw_box = np.asarray(box).reshape(-1).tolist()
        safe_box = [float(v) if isinstance(v, (float, int)) and math.isfinite(v) else None
                    for v in raw_box[:4]]
        safe_box += [None] * (4 - len(safe_box))
        return dict(id=str(identity), label=str(label),
                    box=safe_box, sample_s=float(sample_s),
                    seed_s=float(seed_s), quality=float(quality), status=status,
                    reason=reason, real_detection=bool(real_detection),
                    control_eligible=False, report_eligible=False,
                    footpoint_valid=False, processing_ms=float(processing_ms))

    def _track_result(self, track, stamp_s, status, reason, **kwargs):
        return self._result(track.identity, track.label, track.box, stamp_s,
                            track.seed_s, status, reason, **kwargs)

    @staticmethod
    def _upper_roi(box):
        x1, y1, x2, y2 = box
        w, h = x2 - x1, y2 - y1
        # Exclude silhouette edges, the ground, and walking feet. These points
        # can support a rectangle estimate but cannot validate its bottom.
        return (int(math.ceil(x1 + .15 * w)), int(math.ceil(y1 + .10 * h)),
                int(math.floor(x2 - .15 * w)), int(math.floor(y1 + .68 * h)))

    @classmethod
    def _appearance(cls, image, gray, box):
        x1, y1, x2, y2 = cls._upper_roi(box)
        patch = cv2.resize(gray[y1:y2, x1:x2], (24, 32), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(image[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256])
        cv2.normalize(hist, hist, alpha=1., norm_type=cv2.NORM_L1)
        return patch, hist

    @staticmethod
    def _box_reason(box, shape):
        if box.shape != (4,) or not np.isfinite(box).all():
            return "invalid_box"
        x1, y1, x2, y2 = box
        h, w = shape[:2]
        if x2 - x1 < 12 or y2 - y1 < 24 or (x2 - x1) * (y2 - y1) < 320:
            return "small_box"
        if x1 < 0 or y1 < 0 or x2 > w or y2 > h:
            return "box_outside_image"
        return ""

    @staticmethod
    def _spread_ok(points, box):
        if len(points) < 8:
            return False
        spread = np.ptp(points.reshape(-1, 2), axis=0)
        return spread[0] >= .20 * (box[2] - box[0]) and spread[1] >= .20 * (box[3] - box[1])

    def _expire(self, stamp_s):
        output = []
        for identity, track in list(self._tracks.items()):
            reason = "detector_anchor_expired" if stamp_s - track.seed_s > self.max_age_s + 1e-9 else ""
            if not reason and stamp_s - track.sample_s > self.max_gap_s + 1e-9:
                reason = "image_gap"
            if reason:
                output.append(self._track_result(track, stamp_s, "expired", reason))
                del self._tracks[identity]
        return output

    def seed(self, image_bgr, stamp_s, detections):
        """Anchor listed identities with actual detections; return diagnostics."""
        started = time.perf_counter()
        stamp_s = float(stamp_s)
        if not math.isfinite(stamp_s):
            raise ValueError("image stamp must be finite")
        detections = list(detections)
        if stamp_s < self._latest_s or stamp_s <= self._last_seed_s:
            return [self._result(d.get("id", ""), d.get("label", ""),
                                 d.get("box", [0., 0., 0., 0.]), stamp_s, stamp_s,
                                 "rejected", "non_increasing_seed") for d in detections]
        gray = self._image(image_bgr)
        self._latest_s = stamp_s
        self._last_seed_s = stamp_s
        output = self._expire(stamp_s)
        seen = set()
        for detection in detections:
            identity = str(detection["id"])
            label = str(detection["label"])
            box = np.asarray(detection["box"], dtype=np.float32)
            confidence = float(detection["confidence"])
            reason = self._box_reason(box, gray.shape)
            if not identity or not label or not math.isfinite(confidence) or not 0. <= confidence <= 1.:
                reason = "invalid_detection"
            if identity in seen:
                reason = "duplicate_id_in_seed"
            seen.add(identity)
            if not reason and identity not in self._tracks and len(self._tracks) >= self.max_tracks:
                reason = "capacity"
            points = None
            if not reason:
                mask = np.zeros(gray.shape, dtype=np.uint8)
                x1, y1, x2, y2 = box
                w, h = x2 - x1, y2 - y1
                # Plain uniforms often have no corners inside the torso.
                # Include head/shoulder contours within the measured box,
                # while excluding feet/ground. Keep the stricter inner-torso
                # appearance comparison and all flow/inlier thresholds.
                mask[int(np.ceil(y1)):int(np.floor(y1 + .80*h)),
                     int(np.ceil(x1 + .03*w)):int(np.floor(x2 - .03*w))] = 255
                points = cv2.goodFeaturesToTrack(gray, maxCorners=60,
                                                qualityLevel=.012, minDistance=3,
                                                mask=mask, blockSize=3)
                if points is None or not self._spread_ok(points, box):
                    reason = "insufficient_person_texture"
            if reason:
                # A new genuine measurement which cannot safely seed flow must
                # not leave an older flow hypothesis alive under this ID.
                if reason != "duplicate_id_in_seed":
                    self._tracks.pop(identity, None)
                output.append(self._result(identity, label, box, stamp_s, stamp_s,
                                           "rejected", reason, real_detection=True))
                continue
            template, histogram = self._appearance(image_bgr, gray, box)
            self._tracks[identity] = _Track(identity, label, box.copy(), box.copy(),
                                           np.array([[box[0], box[1]], [box[2], box[1]],
                                                     [box[2], box[3]], [box[0], box[3]]], dtype=np.float32),
                                           confidence, stamp_s, stamp_s, gray,
                                           points, template, histogram)
            output.append(self._result(identity, label, box, stamp_s, stamp_s,
                                       "seeded", "genuine_detector_anchor",
                                       quality=1., real_detection=True))
        elapsed = (time.perf_counter() - started) * 1000.
        for row in output:
            row["processing_ms"] = elapsed
        return output

    def _advance(self, track, image, gray, stamp_s):
        if gray.shape != track.gray.shape:
            return None, "image_size_changed", 0.
        lk = dict(winSize=(21, 21), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
        next_points, forward_ok, error = cv2.calcOpticalFlowPyrLK(track.gray, gray, track.points, None, **lk)
        if next_points is None or forward_ok is None:
            return None, "forward_flow_failed", 0.
        back, back_ok, _ = cv2.calcOpticalFlowPyrLK(gray, track.gray, next_points, None, **lk)
        if back is None or back_ok is None:
            return None, "backward_flow_failed", 0.
        old = track.points.reshape(-1, 2)
        new = next_points.reshape(-1, 2)
        fb = np.linalg.norm(back.reshape(-1, 2) - old, axis=1)
        valid = ((forward_ok.ravel() == 1) & (back_ok.ravel() == 1) &
                 np.isfinite(new).all(axis=1) & np.isfinite(fb) & (fb <= 1.2) &
                 (error.ravel() <= 25.))
        if valid.sum() < 8 or valid.mean() < .55:
            return None, "flow_inconsistent", 0.
        matrix, inliers = cv2.estimateAffinePartial2D(old[valid], new[valid], method=cv2.RANSAC,
                                                    ransacReprojThreshold=1.5, maxIters=500,
                                                    confidence=.99, refineIters=10)
        if matrix is None or inliers is None or not np.isfinite(matrix).all():
            return None, "affine_fit_failed", 0.
        inlier = inliers.ravel().astype(bool)
        if inlier.sum() < 8 or inlier.mean() < .65:
            return None, "motion_not_coherent", 0.
        scale = math.hypot(matrix[0, 0], matrix[1, 0])
        angle = abs(math.atan2(matrix[1, 0], matrix[0, 0]))
        if not .8 <= scale <= 1.25 or angle > math.radians(18):
            return None, "implausible_frame_transform", 0.
        x1, y1, x2, y2 = track.box
        # Transform the original quadrilateral, never the last axis-aligned
        # envelope: repeated camera rotations would inflate that envelope.
        warped = cv2.transform(track.polygon.reshape(-1, 1, 2), matrix).reshape(-1, 2)
        box = np.array([*warped.min(axis=0), *warped.max(axis=0)], dtype=np.float32)
        reason = self._box_reason(box, gray.shape)
        if reason:
            return None, reason, 0.
        old_center = (track.box[:2] + track.box[2:]) * .5
        center = (box[:2] + box[2:]) * .5
        if np.linalg.norm(center - old_center) > max(24., .8 * (y2 - y1)):
            return None, "implausible_displacement", 0.
        ratios = (box[2:] - box[:2]) / (track.seed_box[2:] - track.seed_box[:2])
        if (ratios < .65).any() or (ratios > 1.5).any():
            return None, "anchor_scale_drift", 0.
        kept = new[valid][inlier].astype(np.float32).reshape(-1, 1, 2)
        if not self._spread_ok(kept, box):
            return None, "feature_support_collapsed", 0.
        patch, histogram = self._appearance(image, gray, box)
        correlation = float(cv2.matchTemplate(patch, track.template, cv2.TM_CCOEFF_NORMED)[0, 0])
        histogram_similarity = 1. - float(cv2.compareHist(track.histogram, histogram, cv2.HISTCMP_BHATTACHARYYA))
        if not math.isfinite(correlation) or correlation < .20 or histogram_similarity < .45:
            return None, "appearance_changed", 0.
        # Do not replenish points from the new box: doing that repeatedly can
        # turn lost foreground texture into a self-sustaining background track.
        quality = float(np.clip(valid.mean() * inlier.mean() *
                                min(1., max(0., correlation)) * histogram_similarity, 0., 1.))
        track.box = box
        track.polygon = warped
        track.points = kept
        track.gray = gray
        track.sample_s = stamp_s
        return track, "image_features_matched", quality

    def step(self, image_bgr, stamp_s):
        """Process a new image. Failures remove tracks until detector reseeding."""
        started = time.perf_counter()
        stamp_s = float(stamp_s)
        if not math.isfinite(stamp_s):
            raise ValueError("image stamp must be finite")
        if stamp_s <= self._latest_s:
            return [self._track_result(track, stamp_s, "rejected", "non_increasing_image")
                    for track in self._tracks.values()]
        gray = self._image(image_bgr)
        self._latest_s = stamp_s
        output = self._expire(stamp_s)
        for identity, track in list(self._tracks.items()):
            try:
                updated, reason, quality = self._advance(track, image_bgr, gray, stamp_s)
            except cv2.error:
                updated, reason, quality = None, "optical_flow_error", 0.
            if updated is None:
                output.append(self._track_result(track, stamp_s, "lost", reason))
                del self._tracks[identity]
            else:
                output.append(self._track_result(track, stamp_s, "tracked", reason, quality=quality))
        ambiguous = set()
        tracks = list(self._tracks.values())
        for i, a in enumerate(tracks):
            for b in tracks[i + 1:]:
                overlap = np.maximum(0., np.minimum(a.box[2:], b.box[2:]) -
                                     np.maximum(a.box[:2], b.box[:2]))
                intersection = float(np.prod(overlap))
                union = float(np.prod(a.box[2:] - a.box[:2]) +
                              np.prod(b.box[2:] - b.box[:2]) - intersection)
                if intersection <= 0.:
                    continue
                pa, pb = a.points.reshape(-1, 2), b.points.reshape(-1, 2)
                distances2 = np.sum((pa[:, None] - pb[None, :]) ** 2, axis=2)
                shared = min(int((distances2.min(axis=1) <= 4.).sum()),
                             int((distances2.min(axis=0) <= 4.).sum()))
                if intersection / max(union, 1.) > .70 or shared >= max(8, .5 * min(len(pa), len(pb))):
                    ambiguous.update((a.identity, b.identity))
        for identity in ambiguous:
            del self._tracks[identity]
        for row in output:
            if row["id"] in ambiguous and row["status"] == "tracked":
                row.update(status="lost", reason="multiple_tracks_ambiguous", quality=0.)
        elapsed = (time.perf_counter() - started) * 1000.
        for row in output:
            row["processing_ms"] = elapsed
        return output
