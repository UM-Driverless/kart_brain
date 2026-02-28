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
# Track geometry
# ---------------------------------------------------------------------------
# Oval: two 20m straights + two semicircles of radius 20m (centerline)
# Direction: counterclockwise.  Start/finish at (20, 0) facing +Y.
#
# Parameterization (s = distance along centerline from start):
#   s ∈ [0,  10)          right straight  (20, 0)→(20, 10)
#   s ∈ [10, 10+πR)       top semicircle  center (0,10), θ 0→π
#   s ∈ [10+πR, 30+πR)    left straight   (-20,10)→(-20,-10)
#   s ∈ [30+πR, 30+2πR)   bottom semicircle center (0,-10), θ π→2π
#   s ∈ [30+2πR, 40+2πR)  right straight  (20,-10)→(20, 0)

CURVE_RADIUS = 20.0
STRAIGHT_LEN = 20.0
CURVE_LEN = np.pi * CURVE_RADIUS            # ≈62.83 m
TRACK_LENGTH = 2 * STRAIGHT_LEN + 2 * CURVE_LEN  # ≈165.66 m

# Segment boundaries
_S1 = 10.0                   # end of first half-straight
_S2 = _S1 + CURVE_LEN        # end of top semicircle
_S3 = _S2 + STRAIGHT_LEN     # end of left straight
_S4 = _S3 + CURVE_LEN        # end of bottom semicircle
# _S5 = _S4 + 10.0 == TRACK_LENGTH

# Spawn pose
SPAWN_X, SPAWN_Y, SPAWN_YAW = 20.0, 0.0, np.pi / 2

# ---------------------------------------------------------------------------
# Dense centerline for projection
# ---------------------------------------------------------------------------
NUM_CENTERLINE_POINTS = 2000


def _centerline_point(s):
    """Return (x, y) for a given distance-along-track parameter."""
    s = s % TRACK_LENGTH
    if s < _S1:
        return 20.0, s
    elif s < _S2:
        theta = (s - _S1) / CURVE_RADIUS
        return CURVE_RADIUS * np.cos(theta), 10.0 + CURVE_RADIUS * np.sin(theta)
    elif s < _S3:
        return -20.0, 10.0 - (s - _S2)
    elif s < _S4:
        theta = np.pi + (s - _S3) / CURVE_RADIUS
        return CURVE_RADIUS * np.cos(theta), -10.0 + CURVE_RADIUS * np.sin(theta)
    else:
        return 20.0, -10.0 + (s - _S4)


def _generate_centerline():
    s_values = np.linspace(0, TRACK_LENGTH, NUM_CENTERLINE_POINTS, endpoint=False)
    xy = np.array([_centerline_point(s) for s in s_values])
    return xy, s_values


CENTERLINE_XY, CENTERLINE_S = _generate_centerline()


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
