"""Contestant flight limits, independent of the padded sensor map.

The local official robocup/map_generator.py draws its perimeter at
x=-50/130 and y=-50/50. Actor plugin clamps and map storage bounds are NOT
flight limits. No actor positions, obstacle truth or judge input is read here.
"""
import math

FLIGHT_X_MIN, FLIGHT_X_MAX = -50.0, 130.0
FLIGHT_Y_MIN, FLIGHT_Y_MAX = -50.0, 50.0
FLIGHT_INSET = 2.0


def flight_bounds():
    return FLIGHT_X_MIN, FLIGHT_X_MAX, FLIGHT_Y_MIN, FLIGHT_Y_MAX


def inside_flight(x, y, inset=0.0):
    return (FLIGHT_X_MIN + inset <= x <= FLIGHT_X_MAX - inset and
            FLIGHT_Y_MIN + inset <= y <= FLIGHT_Y_MAX - inset)


def flight_waypoint(x, y, inset=FLIGHT_INSET):
    if not all(math.isfinite(v) for v in (x, y, inset)):
        raise ValueError('nonfinite flight waypoint')
    inset = max(0.0, min(inset, (FLIGHT_X_MAX-FLIGHT_X_MIN)/4.0,
                         (FLIGHT_Y_MAX-FLIGHT_Y_MIN)/4.0))
    return (min(max(x, FLIGHT_X_MIN+inset), FLIGHT_X_MAX-inset),
            min(max(y, FLIGHT_Y_MIN+inset), FLIGHT_Y_MAX-inset))


def boundary_velocity(x, y, vx, vy, margin=FLIGHT_INSET,
                      braking_accel=1.2, response_s=0.6):
    """Cap outward components by v*T + v²/(2*a) <= remaining room.

    Tangential and inward motion remain available. The inset reserves space
    for the airframe and localization error. This is a command envelope, not
    proof of actual vehicle braking; flight logs must validate the response.
    """
    if not all(math.isfinite(v) for v in (x, y, vx, vy)):
        return 0.0, 0.0
    def cap(position, velocity, lo, hi):
        room = (hi-position if velocity > 0 else position-lo) - margin
        room = max(0.0, room)
        at = braking_accel * response_s
        limit = math.sqrt(at*at + 2.0*braking_accel*room) - at
        return math.copysign(min(abs(velocity), limit), velocity)
    return (cap(x, vx, FLIGHT_X_MIN, FLIGHT_X_MAX),
            cap(y, vy, FLIGHT_Y_MIN, FLIGHT_Y_MAX))


class FlightGrid:
    """Navigation-only mask over sensed occupancy; never changes sensor data.

    Both A* and local grid guards use the inset. Raw occupancy queries retain
    sensed data so a vehicle inside the inset can find a clear retreat inward.
    Sensor publication and optical LOS must continue to use the original map.
    """
    def __init__(self, grid):
        self.source = grid

    def __getattr__(self, name):
        return getattr(self.source, name)

    def is_free(self, cell):
        return (self.source.is_free(cell) and
                inside_flight(*self.source.cell_to_world(cell), inset=FLIGHT_INSET))

    def is_free_raw(self, cell):
        raw = getattr(self.source, 'is_free_raw', self.source.is_free)
        return raw(cell)


def validate_environment(env):
    expected = dict(zip(('FLIGHT_X_MIN', 'FLIGHT_X_MAX',
                         'FLIGHT_Y_MIN', 'FLIGHT_Y_MAX', 'FLIGHT_INSET'),
                        flight_bounds() + (FLIGHT_INSET,)))
    for key, value in expected.items():
        if float(env.get(key, value)) != value:
            raise ValueError('%s must be %s, got %s' % (key, value, env[key]))


if __name__ == '__main__':
    import os
    import sys
    try:
        validate_environment(os.environ)
    except ValueError as exc:
        print('[flight boundary] FAIL %s' % exc, file=sys.stderr)
        sys.exit(1)
    print(' '.join(str(v) for v in flight_bounds() + (FLIGHT_INSET,)))
