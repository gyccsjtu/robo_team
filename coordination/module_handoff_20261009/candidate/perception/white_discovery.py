"""Early white/blue originals may guide approach, never official reporting."""
import math


def white_discovery(track, now, height_min, height_max, score_min):
    values = (now, track.observed_s, track.h, track.rng, track.score_ema, *track.raw_xy)
    return (track.cls == 'white' and track.miss == 0 and track.hits >= 2
            and track.person_support.hits >= 2 and track.person_support.current_verified
            and all(math.isfinite(v) for v in values)
            and 0 <= now-track.observed_s <= 1.
            and height_min <= track.h <= height_max
            and 0 < track.rng <= 22. and track.score_ema >= score_min)


def blue_discovery(track, now, height_min, height_max, score_min):
    values=(now,track.observed_s,track.h,track.rng,track.score_ema,*track.raw_xy)
    return (track.cls=='blue' and track.miss==0 and track.hits>=2
            and track.person_support.hits>=2 and track.person_support.current_verified
            and all(math.isfinite(v) for v in values) and 0 <= now-track.observed_s <= 1.
            and height_min <= track.h <= height_max and 0 < track.rng <= 45.
            and track.score_ema >= score_min and track.blue_identity is not None
            and track.blue_identity.approach_allowed(now))
