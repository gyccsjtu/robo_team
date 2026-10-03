"""Prefer a camera that currently sees the target, within dispatch range."""
import math


def tracker_rank(uid, distance, observations, now, limit):
    # Outside-range observers cannot displace an eligible nearby aircraft.
    fresh = any(owner == uid and math.isfinite(stamp) and 0 <= now-stamp <= 1.
                for owner, stamp in observations)
    eligible_observer = fresh and distance < limit
    return (0 if eligible_observer else 1, distance, uid)
