"""Track definition: cone positions and centerline from fs_track.sdf."""

import numpy as np

# ---------------------------------------------------------------------------
# Cone positions extracted from fs_track.sdf  (x, y) — z is ground-level
# Blue = inner boundary, Yellow = outer boundary, Orange = start/finish
# ---------------------------------------------------------------------------

BLUE_CONES = np.array([
    # Right straight (inner, x=18.5)
    [18.5, -10], [18.5, -5], [18.5, 0], [18.5, 5], [18.5, 10],
    # Top curve (inner, r=18.5 from center (0,10))
    [16.02, 14.25], [9.25, 18.02], [0, 20], [-9.25, 18.02], [-16.02, 14.25],
    # Left straight (inner, x=-18.5)
    [-18.5, 10], [-18.5, 5], [-18.5, 0], [-18.5, -5], [-18.5, -10],
    # Bottom curve (inner, r=18.5 from center (0,-10))
    [-16.02, -14.25], [-9.25, -18.02], [0, -20], [9.25, -18.02], [16.02, -14.25],
])

YELLOW_CONES = np.array([
    # Right straight (outer, x=21.5)
    [21.5, -10], [21.5, -5], [21.5, 0], [21.5, 5], [21.5, 10],
    # Top curve (outer, r=21.5 from center (0,10))
    [18.62, 16.75], [10.75, 20.62], [0, 23], [-10.75, 20.62], [-18.62, 16.75],
    # Left straight (outer, x=-21.5)
    [-21.5, 10], [-21.5, 5], [-21.5, 0], [-21.5, -5], [-21.5, -10],
    # Bottom curve (outer, r=21.5 from center (0,-10))
    [-18.62, -16.75], [-10.75, -20.62], [0, -23], [10.75, -20.62], [18.62, -16.75],
])

ORANGE_CONES = np.array([
    [18.0, -0.5], [18.0, 0.5], [22.0, -0.5], [22.0, 0.5],
])

# ---------------------------------------------------------------------------
# Track geometry — derived from actual cone positions
# ---------------------------------------------------------------------------
# The centerline is the midpoint between corresponding blue/yellow cones,
# densified by linear interpolation.  Direction: counterclockwise.
# Start/finish at (20, 0) facing +Y.

# Spawn pose
SPAWN_X, SPAWN_Y, SPAWN_YAW = 20.0, 0.0, np.pi / 2

# ---------------------------------------------------------------------------
# Centerline from cone midpoints
# ---------------------------------------------------------------------------
# Midpoint vertices (20 points forming a closed polygon)
_MIDPOINTS = (BLUE_CONES + YELLOW_CONES) / 2.0

# Find the index of the midpoint closest to spawn (20, 0)
_spawn_dists = np.hypot(_MIDPOINTS[:, 0] - SPAWN_X, _MIDPOINTS[:, 1] - SPAWN_Y)
_spawn_idx = int(np.argmin(_spawn_dists))

# Reorder midpoints so index 0 is the one nearest the spawn
_MIDPOINTS_ORDERED = np.roll(_MIDPOINTS, -_spawn_idx, axis=0)

NUM_INTERP_PER_SEGMENT = 100  # interpolation points between each pair of vertices


def _densify_midpoints():
    """Densify the midpoint polygon into a smooth centerline."""
    n = len(_MIDPOINTS_ORDERED)
    points = []
    cum_s = [0.0]
    for i in range(n):
        p0 = _MIDPOINTS_ORDERED[i]
        p1 = _MIDPOINTS_ORDERED[(i + 1) % n]
        seg_len = np.hypot(p1[0] - p0[0], p1[1] - p0[1])
        for j in range(NUM_INTERP_PER_SEGMENT):
            t = j / NUM_INTERP_PER_SEGMENT
            x = p0[0] + t * (p1[0] - p0[0])
            y = p0[1] + t * (p1[1] - p0[1])
            points.append((x, y))
            if len(points) > 1:
                dx = points[-1][0] - points[-2][0]
                dy = points[-1][1] - points[-2][1]
                cum_s.append(cum_s[-1] + np.hypot(dx, dy))
    return np.array(points), np.array(cum_s)


CENTERLINE_XY, CENTERLINE_S = _densify_midpoints()
TRACK_LENGTH = CENTERLINE_S[-1] + np.hypot(
    CENTERLINE_XY[0, 0] - CENTERLINE_XY[-1, 0],
    CENTERLINE_XY[0, 1] - CENTERLINE_XY[-1, 1],
)


def project_to_centerline(x, y):
    """Find nearest centerline point.

    Returns
    -------
    s : float
        Distance along the track (0 .. TRACK_LENGTH).
    cte : float
        Cross-track error (distance from centerline).
    """
    dx = CENTERLINE_XY[:, 0] - x
    dy = CENTERLINE_XY[:, 1] - y
    dists_sq = dx * dx + dy * dy
    idx = np.argmin(dists_sq)
    return float(CENTERLINE_S[idx]), float(np.sqrt(dists_sq[idx]))


# ---------------------------------------------------------------------------
# Track boundary checking — uses line segments between consecutive cones
# ---------------------------------------------------------------------------
# Half track width = average distance from centerline to boundary midpoint
HALF_TRACK_WIDTH = 1.5  # nominal — actual varies but cones are ~1.5m from midpoints

# Precompute segment arrays for vectorized distance computation
# Each boundary is N segments: starts[i] -> ends[i]
_BLUE_STARTS = BLUE_CONES
_BLUE_ENDS = np.roll(BLUE_CONES, -1, axis=0)
_YELLOW_STARTS = YELLOW_CONES
_YELLOW_ENDS = np.roll(YELLOW_CONES, -1, axis=0)
# Combined: all boundary segments (blue + yellow)
_ALL_STARTS = np.vstack([_BLUE_STARTS, _YELLOW_STARTS])
_ALL_ENDS = np.vstack([_BLUE_ENDS, _YELLOW_ENDS])
_ALL_DX = _ALL_ENDS[:, 0] - _ALL_STARTS[:, 0]
_ALL_DY = _ALL_ENDS[:, 1] - _ALL_STARTS[:, 1]
_ALL_LEN_SQ = _ALL_DX ** 2 + _ALL_DY ** 2


def _point_in_polygon_np(px, py, poly_x, poly_y):
    """Vectorized ray casting point-in-polygon."""
    n = len(poly_x)
    x1 = poly_x
    y1 = poly_y
    x2 = np.roll(poly_x, -1)
    y2 = np.roll(poly_y, -1)
    # Edge crosses the ray from (px, py) rightward?
    cond1 = (y1 > py) != (y2 > py)
    x_intersect = (x2 - x1) * (py - y1) / (y2 - y1) + x1
    cond2 = px < x_intersect
    crossings = np.sum(cond1 & cond2)
    return crossings % 2 == 1


def is_inside_track(x, y):
    """Check if (x,y) is between the blue and yellow cone boundaries."""
    in_yellow = _point_in_polygon_np(x, y, YELLOW_CONES[:, 0], YELLOW_CONES[:, 1])
    in_blue = _point_in_polygon_np(x, y, BLUE_CONES[:, 0], BLUE_CONES[:, 1])
    return bool(in_yellow and not in_blue)


def dist_to_boundary(x, y):
    """Signed distance to nearest track boundary segment.

    Returns
    -------
    float
        Positive if inside the track, negative if outside.
        Magnitude is the distance to the nearest boundary line segment.
    """
    # Vectorized distance to all segments at once
    apx = x - _ALL_STARTS[:, 0]
    apy = y - _ALL_STARTS[:, 1]
    t = (apx * _ALL_DX + apy * _ALL_DY) / _ALL_LEN_SQ
    t = np.clip(t, 0.0, 1.0)
    proj_x = _ALL_STARTS[:, 0] + t * _ALL_DX
    proj_y = _ALL_STARTS[:, 1] + t * _ALL_DY
    dists = np.hypot(x - proj_x, y - proj_y)
    d = float(dists.min())
    if not is_inside_track(x, y):
        return -d
    return d
