"""Explicit clock and sensor receipts for offline single-step flight fixtures."""
import math
from types import SimpleNamespace
from unittest.mock import patch


def install_clock_fixture(case, ros, wall=1000.):
    for name, value in (('is_time_valid', True), ('time_epoch', 0),
                        ('wall_monotonic', wall)):
        change = patch.object(ros, name, return_value=value)
        change.start()
        case.addCleanup(change.stop)


def prepare_flight_fixture(agent, ros, renew_pose=False):
    """Declare received telemetry; do not hide an explicitly stale pose/scan."""
    sample = ros.Time.now().to_sec()
    if renew_pose:
        agent._pose_sample_s = sample
        velocity = getattr(agent, '_measured_velocity', (sample, (0., 0.)))[1]
        agent._measured_velocity = (sample, velocity)
    agent.__dict__.setdefault('_pose_sample_s', sample)
    agent.__dict__.setdefault('_measured_velocity', (sample, (0., 0.)))
    agent.__dict__.setdefault('_scan_t', sample)
    agent.__dict__.setdefault('_scan', SimpleNamespace(
        angle_min=-math.pi, angle_increment=math.pi/180., range_min=.1,
        range_max=20., ranges=[float('inf')]*361))
    agent.__dict__.setdefault('yaw', 0.)
    agent.__dict__.setdefault('_epoch_pose_samples', 1)
    agent.__dict__.setdefault('_epoch_pose_required', 1)
    for source in ('pose', 'velocity', 'scan'):
        setattr(agent, '_wall_%s_received' % source, ros.wall_monotonic())
        setattr(agent, '_wall_%s_epoch' % source, ros.time_epoch())
