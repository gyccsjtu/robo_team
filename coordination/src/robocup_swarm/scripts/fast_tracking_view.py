"""Strict identity and time gates for experimental image-only yaw feedback.

This module produces no world position, evidence, assignment, or official report.
An optical-flow result can only refine an existing own-camera target bearing.
"""
import math


FAST_IMAGE_MAX_AGE = .15
FAST_SEED_MAX_AGE = .6
REAL_POSITION_MAX_AGE = .6
FAST_MIN_QUALITY = .35
FAST_MAX_BEARING_DELTA = .25


def fast_yaw_enabled(environ, uav_id):
    if environ.get('FAST_VISUAL_YAW', '0') != '1':
        return False
    selected = {v.strip() for v in environ.get('FAST_VISUAL_YAW_UAVS', '').split(',')
                if v.strip()}
    return not selected or uav_id in selected or uav_id.rsplit('_', 1)[-1] in selected


def validate_fast_view(data, source, tid, real_view, now, previous=None):
    """Validate a newer image against an already bound, real camera view.

    The seed identifier is image-tracker identity, while world_candidate_id is
    the existing localization identity. Matching clothing colour is insufficient.
    Re-validating accepted data at output time expires it without changing the
    real view or refreshing its localization timestamp.
    """
    try:
        if (data.get('schema') != 1 or data.get('source_uav') != source or
                data.get('status') != 'tracked' or
                data.get('real_detection') is not False or
                data.get('report_eligible') is not False or
                data.get('footpoint_valid') is not False or
                data.get('yaw_only') is not True):
            raise ValueError('contract')
        if real_view is None or real_view.get('target_id') != tid:
            raise ValueError('no_assigned_real_view')
        candidate = data.get('seed_candidate_id')
        session = data.get('candidate_session')
        instance = real_view.get('world_candidate_id')
        if (not isinstance(candidate, str) or not candidate or
                not isinstance(session, str) or not session or ':' in session or
                real_view.get('candidate_id') != candidate or
                not isinstance(instance, str) or not instance.startswith(session+':') or
                data.get('seed_id') != session+':'+candidate):
            raise ValueError('identity')
        if ('bound_world_candidate_id' in data and
                data['bound_world_candidate_id'] != instance):
            raise ValueError('world_identity_changed')
        if 'target_id' in data and data['target_id'] != tid:
            raise ValueError('assignment_changed')
        sample, seed, quality, bearing, width, height, fx = (
            float(data[k]) for k in
            ('sample_s', 'seed_s', 'quality', 'bearing', 'width', 'height', 'fx'))
        real_sample = float(real_view['sample_s'])
        position_sample = float(real_view.get('position_sample_s', real_sample))
        real_bearing = float(real_view['bearing'])
        box = tuple(map(float, data['box']))
        if len(box) != 4 or not all(math.isfinite(v) for v in
                (now, sample, seed, quality, bearing, width, height, fx,
                 real_sample, position_sample, real_bearing, *box)):
            raise ValueError('nonfinite')
        if not (0. <= now-sample <= FAST_IMAGE_MAX_AGE and
                0. <= now-seed <= FAST_SEED_MAX_AGE and
                0. <= now-position_sample <= REAL_POSITION_MAX_AGE and
                0. <= now-real_sample <= .5 and
                0. < seed < sample and seed >= position_sample and
                sample > real_sample):
            raise ValueError('age_or_order')
        if previous is not None and sample <= previous['sample_s']:
            raise ValueError('duplicate_or_old')
        if not FAST_MIN_QUALITY <= quality <= 1.:
            raise ValueError('quality')
        x1, y1, x2, y2 = box
        if (width != real_view['width'] or height != real_view['height'] or
                width <= 0. or height <= 0. or fx <= 0. or
                not (0. <= x1 < x2 <= width and 0. <= y1 < y2 <= height)):
            raise ValueError('box')
        delta = math.atan2(math.sin(bearing-real_bearing),
                           math.cos(bearing-real_bearing))
        if abs(delta) > FAST_MAX_BEARING_DELTA:
            raise ValueError('bearing_jump')
        return dict(data, sample_s=sample, seed_s=seed, quality=quality,
                    bearing=bearing, box=box, target_id=tid,
                    bound_world_candidate_id=instance)
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError('malformed') from exc
