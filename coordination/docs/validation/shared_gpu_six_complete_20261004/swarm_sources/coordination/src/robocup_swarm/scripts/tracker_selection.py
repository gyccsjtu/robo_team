"""Prefer a camera that currently sees the target, within dispatch range."""
import math


def tracker_rank(uid, distance, observations, now, limit):
    # Outside-range observers cannot displace an eligible nearby aircraft.
    fresh = any(owner == uid and math.isfinite(stamp) and 0 <= now-stamp <= 1.
                for owner, stamp in observations)
    eligible_observer = fresh and distance < limit
    return (0 if eligible_observer else 1, distance, uid)


def takeover_candidate(owner, candidates, observations, now, limit):
    """An actual camera may request takeover after its owner loses its view.

    This selects an intent only; it does not release a target or route lock.
    """
    if any(uid == owner and math.isfinite(stamp) and 0 <= now-stamp <= 1.5
           for uid, stamp in observations):
        return None
    eligible = [(distance, uid) for uid, distance in candidates if uid != owner
                and math.isfinite(distance) and 0 <= distance < limit
                and tracker_rank(uid, distance, observations, now, limit)[0] == 0]
    return min(eligible)[1] if eligible else None
