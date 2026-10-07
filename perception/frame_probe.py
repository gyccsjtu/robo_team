#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Per-frame detection denominator: one row for every frame the detector processed.

Why this exists (2026-10-07, improvement task N7)
-------------------------------------------------
Every other row in ``perception_<uav>.csv`` is event driven: a raw box that survived
the geometry gate (``yolo_detection``), a track that was refused (``track_reject``),
a target that was published (``track_publish``). A frame where the model returns
nothing writes **no row at all**. So "the 15 s continuity criterion breaks because of
detection gaps" could not be attributed to either of its two very different causes:

  * the model genuinely did not fire (a recall / resolution problem), or
  * the model fired and our own ROI / range / height gates dropped it (a gate problem).

Both are invisible in the event stream. This module builds the missing row.

Design notes
------------
* Pure Python, no ROS, no cv2 -> unit testable off the robot
  (``perception/tests/test_frame_probe.py``).
* The file is deliberately **not** named ``perception*.csv``: ``audit_fast_city.py``
  and ``replay_image_motion.py`` glob ``perception*.csv`` as the event stream and
  would be fed a completely different row shape.
* Nothing here touches control. It only records.
"""

FIELDS = [
    # when / how long
    'ros_time', 'image_stamp', 'image_age_s', 'inference_s',
    # what the model emitted this frame, before any of our filtering
    'raw_boxes', 'raw_cls', 'raw_in_roi', 'roi_drop',
    'raw_w_max', 'raw_h_max', 'raw_conf_max',
    # what survived our own geometry gate (loose band), and what it dropped
    'geo_pass', 'geo_reject',
    # where the camera was, so a miss can be split into "not looking there" vs "looked and missed"
    'cam_x', 'cam_y', 'cam_z', 'cam_yaw', 'img_w', 'img_h',
    # which inference backend answered
    'shared',
]


def summarize_raw_boxes(boxes, roi_top=0.0, roi_bottom=None):
    """Summarize one frame of raw model output.

    ``boxes`` yields ``(class_name, confidence, x1, y1, x2, y2)`` in image pixels,
    i.e. exactly what YOLO returned, before ROI clipping or geometry gating.

    Returns the counters that make one frame comparable with the next:
      raw_boxes    how many boxes the model emitted at all
      raw_cls      per-class counts, e.g. ``'blue:2|white:1'`` (sorted, so it diffs cleanly)
      raw_in_roi   how many sit inside the vertical ROI band we later apply
      roi_drop     how many our own ROI band removes before geometry even sees them
      raw_w_max / raw_h_max / raw_conf_max
                   the largest / most confident in-ROI box ("how small was the best
                   chance this frame gave us" -- the quantity the recall question needs)
    """
    counts = {}
    total = in_roi = 0
    w_max = h_max = conf_max = 0.0
    for name, confidence, x1, y1, x2, y2 in boxes:
        total += 1
        counts[name] = counts.get(name, 0) + 1
        if roi_bottom is not None and (y2 < roi_top or y1 > roi_bottom):
            continue
        in_roi += 1
        width, height = x2 - x1, y2 - y1
        if width > w_max:
            w_max = width
        if height > h_max:
            h_max = height
        if confidence > conf_max:
            conf_max = confidence
    return dict(
        raw_boxes=total,
        raw_cls='|'.join('%s:%d' % item for item in sorted(counts.items())),
        raw_in_roi=in_roi,
        roi_drop=total - in_roi,
        raw_w_max=round(w_max, 1),
        raw_h_max=round(h_max, 1),
        raw_conf_max=round(conf_max, 4),
    )


def frame_row(summary, ros_time, image_stamp, image_age_s, inference_s,
              geo_pass, geo_reject, camera_xyz, camera_yaw, image_wh, shared):
    """Join a :func:`summarize_raw_boxes` result with the per-frame scalars.

    Kept separate from the caller so the row shape is testable without ROS.
    """
    row = dict(summary)
    row.update(
        ros_time=ros_time,
        image_stamp=image_stamp,
        image_age_s=image_age_s,
        inference_s=inference_s,
        geo_pass=geo_pass,
        geo_reject=geo_reject,
        cam_x=camera_xyz[0],
        cam_y=camera_xyz[1],
        cam_z=camera_xyz[2],
        cam_yaw=camera_yaw,
        img_w=image_wh[0],
        img_h=image_wh[1],
        shared=shared,
    )
    missing = [name for name in FIELDS if name not in row]
    if missing:
        raise ValueError('FRAME_ROW_MISSING_FIELDS:%s' % ','.join(missing))
    return row
