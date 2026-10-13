"""Observe sensed free ground beside buildings from safe external viewpoints."""
import math

from search_observation import cell_points, ground_distance_band, in_view
from swarm_task import ACTOR_SEARCH_BOUNDS, LineOfSight


DIRECTED_PURPOSES = ('south_patrol', 'building_review', 'coverage_view')


def boundary_corner(coverage, key, point):
    """Identify real corners of the search envelope, not every cell corner."""
    if not reserved_subpoint(coverage, key, point):
        return False
    x0, x1, y0, y1 = coverage.cell_bounds(key)
    return ((math.isclose(x0, coverage.x_min) and math.isclose(point[0], x0+.01)) or
            (math.isclose(x1, coverage.x_max) and math.isclose(point[0], x1-.01))) and (
            (math.isclose(y0, coverage.y_min) and math.isclose(point[1], y0+.01)) or
            (math.isclose(y1, coverage.y_max) and math.isclose(point[1], y1-.01)))


def reserved_subpoint(coverage, key, point):
    """A building review may observe only this cell's existing nine samples."""
    cell = coverage.cell(key)
    if cell is None or not all(math.isfinite(v) for v in point):
        return False
    xmin, xmax, ymin, ymax = ACTOR_SEARCH_BOUNDS
    if (not xmin <= point[0] <= xmax or not ymin <= point[1] <= ymax or
            coverage.world_to_cell(*point) != key):
        return False
    return any(math.dist(point, sample) <= 1e-6
               for sample in cell_points(coverage, cell))


def valid_view(raw, inflated, look, point, flight_waypoint, altitude=4.5):
    """Ground must be raw-free; aircraft clearance remains inflated + 1 m."""
    if (raw is None or inflated is None or
            not all(math.isfinite(v) for v in (*look, *point)) or
            math.dist(point, flight_waypoint(*point)) > 1e-6):
        return False
    ground = raw.world_to_cell(look)
    if ground is None or not raw.is_free(ground):
        return False  # LOS deliberately exempts its endpoint; do not trust that.
    for dx, dy in ((0., 0.), (1., 0.), (-1., 0.), (0., 1.), (0., -1.),
                   (.7, .7), (.7, -.7), (-.7, .7), (-.7, -.7)):
        cell = inflated.world_to_cell((point[0]+dx, point[1]+dy))
        if cell is None or not inflated.is_free(cell):
            return False
    bearing = math.atan2(look[1]-point[1], look[0]-point[0])
    if not in_view(*point, bearing, *look, altitude, 0.):
        return False
    los = LineOfSight(lambda ix, iy: not raw.is_free((ix, iy)),
                      raw.resolution, raw.origin)
    return los.visible(*point, *look)


def view_options(coverage, key, raw, inflated, flight_waypoint,
                 altitude=4.5, radius=8.95, samples=None, heading=None):
    """Bounded candidates; actual aircraft-to-viewpoint paths are checked later."""
    cell = coverage.cell(key)
    if cell is None:
        return ()
    result = []
    for look in (cell_points(coverage, cell) if samples is None else samples):
        if not reserved_subpoint(coverage, key, look):
            continue
        ground = raw.world_to_cell(look) if raw is not None else None
        if ground is None or not raw.is_free(ground):
            continue
        angles = ([index*math.pi/4. for index in range(8)] if heading is None else
                  [heading+math.pi+offset for offset in (0., math.pi/8., -math.pi/8.)])
        for angle in angles:
            point = (look[0]+radius*math.cos(angle),
                     look[1]+radius*math.sin(angle))
            if valid_view(raw, inflated, look, point, flight_waypoint, altitude):
                result.append((point, look))
    return tuple(result)


def measured_view_options(coverage, key, raw, inflated, flight_waypoint,
                          altitude, heading=None, samples=None):
    """Neutral hover must see a reserved sample at the measured camera height.

    This proposes a view, never image evidence. Unknown/invalid height has no
    fallback. The midpoint leaves equal vertical margins inside the <=9 m band.
    """
    if isinstance(altitude, bool) or not isinstance(altitude, (int, float)):
        return ()
    band = ground_distance_band(altitude, 0.)
    if band is None:
        return ()
    return view_options(coverage, key, raw, inflated, flight_waypoint,
                        altitude=altitude, radius=sum(band)*.5,
                        samples=samples, heading=heading)
