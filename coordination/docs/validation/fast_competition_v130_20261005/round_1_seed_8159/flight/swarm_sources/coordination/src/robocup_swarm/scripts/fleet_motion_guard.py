"""Pure measured fleet-motion cache and final planar protective guard."""
import math


class MotionCache:
    def __init__(self, run_id, fleet):
        if not run_id or not fleet or len(set(fleet)) != len(fleet):
            raise ValueError('Explicit run and unique fleet required')
        self.run_id, self.fleet, self.samples = run_id, tuple(fleet), {}

    def receive(self, message, now):
        try:
            uid = message['uav_id']
            if (message['schema_version'] != 1 or message['run_id'] != self.run_id
                    or uid not in self.fleet or message['frame'] != 'world_enu_xy'):
                return False
            seq, stamp = message['seq'], message['sample_s']
            position, velocity = message['position_xy'], message['velocity_xy']
            if (type(seq) is not int or seq <= 0 or len(position) != 2 or len(velocity) != 2
                    or not all(math.isfinite(v) for v in (*position, *velocity, stamp, now))
                    or not 0 <= now - stamp <= .5):
                return False
            old = self.samples.get(uid)
            if old and (seq <= old['seq'] or stamp <= old['sample_s']):
                return False
            self.samples[uid] = dict(message)
            return True
        except (KeyError, TypeError, ValueError):
            return False


def protect(request, uid, cache, now, separation=4.5, latency=.5, brake=.5, horizon=3.):
    stop = lambda reason: dict(velocity_xy=(0., 0.), reason=reason)
    if len(request) != 2 or not all(math.isfinite(x) for x in (*request, now)):
        return stop('INVALID_REQUEST')
    if uid not in cache.fleet or any(k not in cache.samples or not 0 <= now-cache.samples[k]['sample_s'] <= .5 for k in cache.fleet):
        return stop('MISSING_OR_STALE_FLEET_MOTION')
    own = cache.samples[uid]
    ox, oy = own['position_xy']
    vx, vy = own['velocity_xy']
    own_age = now - own['sample_s']
    ox, oy = ox + vx*own_age, oy + vy*own_age
    for peer_id in cache.fleet:
        if peer_id == uid:
            continue
        peer = cache.samples[peer_id]
        age = now - peer['sample_s']
        px, py = peer['position_xy']
        pvx, pvy = peer['velocity_xy']
        # Account for sample age rather than treating old coordinates as current.
        dx, dy = px + pvx*age - ox, py + pvy*age - oy
        distance = math.hypot(dx, dy)
        if distance <= separation:
            return stop('SEPARATION_PROTECTIVE_STOP')
        ux, uy = dx/distance, dy/distance
        own_closing = max(0., vx*ux + vy*uy)
        peer_closing = max(0., -pvx*ux - pvy*uy)
        stop_budget = (own_closing+peer_closing)*latency + (own_closing**2+peer_closing**2)/(2*brake)
        if stop_budget >= distance-separation:
            return stop('RELATIVE_BRAKING_REQUIRED')
        rvx, rvy = pvx-request[0], pvy-request[1]
        speed2 = rvx*rvx + rvy*rvy
        nearest_t = min(horizon, max(0., -(dx*rvx + dy*rvy)/speed2)) if speed2 > 1e-9 else 0.
        if math.hypot(dx+rvx*nearest_t, dy+rvy*nearest_t) < separation:
            return stop('PREDICTED_APPROACH_STOP')
    return dict(velocity_xy=tuple(request), reason='CLEAR')
