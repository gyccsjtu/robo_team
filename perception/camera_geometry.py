"""Camera metadata validation and timestamp alignment, independent of ROS."""
import math


def calibration(width, height, intrinsic, distortion):
    if type(width) is not int or type(height) is not int or min(width, height) <= 0:
        raise ValueError('CAMERA_DIMENSIONS')
    if len(intrinsic) != 9 or not all(math.isfinite(v) for v in intrinsic):
        raise ValueError('CAMERA_INTRINSICS')
    fx, fy, cx, cy = intrinsic[0], intrinsic[4], intrinsic[2], intrinsic[5]
    if (min(fx, fy) <= 0 or not 0 <= cx <= width or not 0 <= cy <= height
            or intrinsic[1] != 0 or intrinsic[3] != 0
            or list(intrinsic[6:]) != [0., 0., 1.]):
        raise ValueError('CAMERA_INTRINSICS')
    if any(not math.isfinite(v) or abs(v) > 1e-10 for v in distortion):
        raise ValueError('RAW_DISTORTION_REQUIRES_RECTIFICATION')
    return fx, fy, cx, cy, width, height


def aligned_translation(stamp, first, second, maximum_age=.5):
    """Samples are (sample_s,x,y,z). Extrapolation uses seconds, never ratios."""
    if not all(math.isfinite(v) for v in (stamp,)+tuple(first)+tuple(second)):
        raise ValueError('POSE_FINITE')
    dt = second[0]-first[0]
    offset_s = stamp-second[0]
    if dt <= 0 or not -maximum_age <= offset_s <= .1:
        raise ValueError('POSE_TIME_ALIGNMENT')
    return tuple(second[i]+(second[i]-first[i])*offset_s/dt for i in (1, 2, 3))
