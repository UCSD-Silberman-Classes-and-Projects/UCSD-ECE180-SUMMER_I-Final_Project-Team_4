"""Dual-LD19 processing: mask -> transform to robot frame -> merge -> sectorize.

Because the UNO Q sits centrally amid wiring, one LiDAR cannot see 360. We run a
FRONT and a REAR LD19; each covers the arc the central clutter blocks for the
other. Each unit also sees its own body, the clutter, and the other LiDAR as
phantom returns -- those fixed arcs are masked out per unit before merging.

Downstream (evasion policy) never sees raw points: everything is reduced to a
handful of per-sector minimum distances.
"""

import math

from .. import config


def _angle_in_arc(angle, start, end):
    """True if angle (deg) lies within [start, end], handling wraparound."""
    a = angle % 360
    s, e = start % 360, end % 360
    if s <= e:
        return s <= a <= e
    return a >= s or a <= e   # arc wraps through 0


def apply_masks(points, masks):
    """Drop points whose angle falls inside any masked arc (LiDAR frame)."""
    if not masks:
        return points
    kept = []
    for p in points:
        if any(_angle_in_arc(p.angle_deg, s, e) for (s, e) in masks):
            continue
        kept.append(p)
    return kept


def to_robot_frame(points, offset):
    """Transform points from a LiDAR's local frame into the robot frame.

    offset = (x, y, yaw) of the LiDAR relative to robot center (m, m, rad).
    Returns list of (angle_deg_robot, distance_m) tuples.
    """
    ox, oy, oyaw = offset
    out = []
    for p in points:
        # point position in LiDAR frame
        a = math.radians(p.angle_deg)
        px = p.distance_m * math.cos(a)
        py = p.distance_m * math.sin(a)
        # rotate by yaw, then translate by the mounting offset
        rx = px * math.cos(oyaw) - py * math.sin(oyaw) + ox
        ry = px * math.sin(oyaw) + py * math.cos(oyaw) + oy
        dist = math.hypot(rx, ry)
        ang = math.degrees(math.atan2(ry, rx))
        out.append((ang, dist))
    return out


def merge(front_points, rear_points):
    """Mask, transform, and combine both units into one robot-frame point list."""
    f = apply_masks(front_points, config.FRONT_LIDAR_MASKS)
    r = apply_masks(rear_points, config.REAR_LIDAR_MASKS)
    fr = to_robot_frame(f, config.FRONT_LIDAR_OFFSET)
    rr = to_robot_frame(r, config.REAR_LIDAR_OFFSET)
    return fr + rr


def sectorize(robot_points, sectors=config.SECTORS):
    """Reduce a merged point list to nearest-obstacle distance per sector.

    Returns a dict {sector_name: min_distance_m}; a sector with no points
    returns math.inf (nothing seen there).
    """
    result = {name: math.inf for name in sectors}
    for ang, dist in robot_points:
        for name, (start, end) in sectors.items():
            if _angle_in_arc(ang, start, end):
                if dist < result[name]:
                    result[name] = dist
    return result


def process(front_points, rear_points):
    """Full pipeline: two raw scans -> six sector minimums."""
    return sectorize(merge(front_points, rear_points))


def is_cornered(sectors, threshold=config.CORNER_DISTANCE):
    """Corner condition: walls converging ahead on both front-side sectors."""
    return (sectors["front_left"] < threshold and
            sectors["front_right"] < threshold)


def range_at_bearing(robot_points, bearing_deg,
                     half_window_deg=config.BEARING_WINDOW_DEG):
    """Nearest LiDAR return within +/- half_window_deg of a bearing.

    This is what pairs the camera with the LiDAR: the camera says WHICH
    direction the pursuer is, this says HOW FAR away the nearest thing in
    that direction is. Returns math.inf when nothing is seen there (which
    range_to_proximity treats as "far").

    Note the window is a compromise: too narrow and a person between scan
    points is missed, too wide and a wall beside them wins the min(). Tune
    config.BEARING_WINDOW_DEG on the real robot.
    """
    best = math.inf
    lo = bearing_deg - half_window_deg
    hi = bearing_deg + half_window_deg
    for ang, dist in robot_points:
        if _angle_in_arc(ang, lo, hi) and dist < best:
            best = dist
    return best


def track_at_bearing(robot_points, bearing_deg,
                     tol_deg=config.LIDAR_TRACK_TOL_DEG,
                     max_m=None,
                     range_hint_m=None,
                     range_gate_m=config.LIDAR_TRACK_RANGE_GATE_M):
    """Re-find the pursuer in a scan near a REMEMBERED bearing.

    Angles are in the same frame as robot_points (atan2 convention:
    positive = CCW/left). The camera's blind arc points backward -- exactly
    where a successfully evaded pursuer ends up -- so this is what keeps the
    pursuit alive without the camera: given the last known chassis bearing
    (and last known range), return (bearing_deg, distance_m) of the best
    return in a window around it. Feeding the result back in on the next
    revolution makes this a nearest-neighbour tracker that follows a walking
    person from scan to scan.

    The range gate is essential, not a refinement: a flat wall offers a
    continuum of returns, so if the angular window slips during a hard turn,
    an ungated search lands on the wall and then follows ITSELF along it
    indefinitely. With a hint, only returns within +/-range_gate_m of the
    last known range qualify, and among those the one closest in ANGLE to
    the prediction wins (the gate has already established plausibility).

    Returns (bearing_deg, distance_m) or None if nothing qualifies.
    """
    if max_m is None:
        max_m = config.RANGE_FAR_M * 2
    hint = None
    if range_hint_m is not None and math.isfinite(range_hint_m):
        hint = range_hint_m
    best = None
    for ang, dist in robot_points:
        if dist > max_m:
            continue
        da = abs((ang - bearing_deg + 180.0) % 360.0 - 180.0)
        if da > tol_deg:
            continue
        if hint is not None and abs(dist - hint) > range_gate_m:
            continue
        score = da if hint is not None else dist
        if best is None or score < best[0]:
            best = (score, ang, dist)
    if best is None:
        return None
    return best[1], best[2]


def merged_points(front_points, rear_points):
    """Masked + transformed robot-frame points, before sectorizing.

    process() throws the individual points away, but range_at_bearing needs
    them, so expose the intermediate step rather than scanning twice.
    """
    return merge(front_points, rear_points)
