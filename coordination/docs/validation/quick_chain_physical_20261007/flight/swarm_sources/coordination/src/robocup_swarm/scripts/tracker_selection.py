"""Prefer a camera that currently sees the target, within dispatch range."""
import math


def tracker_rank(uid, distance, observations, now, limit):
    # Outside-range observers cannot displace an eligible nearby aircraft.
    fresh = any(owner == uid and math.isfinite(stamp) and 0 <= now-stamp <= 1.
                for owner, stamp in observations)
    eligible_observer = fresh and distance < limit
    return (0 if eligible_observer else 1, distance, uid)


def held_position(held, now, hold_s):
    """Last known position to keep dispatching while the target is unobserved.

    Observation freshness (OBS_TTL, 1 s) and time-to-reach are different scales:
    measured ground speed is ~0.6 m/s while dispatched aircraft sit 13-61 m from
    their target, so a target that stops being observed is undispatchable long
    before any aircraft could arrive. ``hold_s`` bounds how long the remembered
    position may still be used. ``hold_s <= 0`` keeps the previous behaviour -
    no hold at all - so the caller can enable this without changing any safety
    gate, speed limit or release condition.
    """
    if hold_s <= 0.0 or held is None or now is None:
        return None
    try:
        x, y, stamp = held
        age = now - stamp
    except (TypeError, ValueError):
        return None
    for value in (x, y, age):
        if not math.isfinite(value):
            return None
    if not 0.0 <= age <= hold_s:
        return None
    return (x, y)


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
