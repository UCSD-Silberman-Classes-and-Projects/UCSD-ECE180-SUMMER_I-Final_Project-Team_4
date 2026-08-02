"""Pure geometry: turn a person bounding box into bearing and proximity.

No I/O, no state -- trivially unit-testable with hand-written boxes.
A bbox is (x_min, y_min, x_max, y_max) in pixel coordinates.
"""

import math

from .. import config


def bbox_to_bearing(bbox, frame_width=config.FRAME_WIDTH):
    """Return bearing in [-1, 1]: -1 = far left, 0 = centered, +1 = far right.

    Uses the horizontal center of the box relative to the frame center.
    """
    x_min, _, x_max, _ = bbox
    box_center_x = 0.5 * (x_min + x_max)
    normalized = box_center_x / frame_width          # 0..1 across the frame
    return 2.0 * normalized - 1.0                     # -1..1


def bbox_to_proximity(bbox, frame_height=config.FRAME_HEIGHT):
    """Return proximity in [0, 1]: 0 = far, 1 = very close.

    Uses box height as a distance proxy (a standing person's real height is
    roughly constant indoors, so a taller box means they are closer).
    """
    _, y_min, _, y_max = bbox
    box_frac = (y_max - y_min) / frame_height
    lo, hi = config.PROXIMITY_MIN_BOX_FRAC, config.PROXIMITY_MAX_BOX_FRAC
    proximity = (box_frac - lo) / (hi - lo)
    return max(0.0, min(1.0, proximity))              # clamp to [0, 1]

# ---------------------------------------------------------------------------
# Servo-mounted camera geometry
#
# With the camera on a pan servo, a bearing measured in the frame is relative
# to WHERE THE CAMERA IS POINTING, not to the chassis. Everything downstream
# (evasion, LiDAR range lookup) needs chassis-relative angles, so these three
# functions do the conversion: pixels -> degrees off the camera axis ->
# degrees off the chassis nose -> a steering signal.
# ---------------------------------------------------------------------------


def bbox_to_camera_bearing_deg(bbox, frame_width=config.FRAME_WIDTH,
                               hfov_deg=config.CAMERA_HFOV_DEG):
    """Angle of the box centre off the camera's optical axis, in degrees.

    Negative = left of centre, positive = right. Uses the true pinhole
    (tangent) mapping rather than a linear one: a linear mapping is a couple
    of degrees off near the frame edges, which matters here because that
    error feeds straight into where we aim the servo.
    """
    x_min, _, x_max, _ = bbox
    centre_x = 0.5 * (x_min + x_max)
    offset = centre_x / frame_width - 0.5          # -0.5 .. +0.5
    focal = 0.5 / math.tan(math.radians(hfov_deg) / 2.0)
    return math.degrees(math.atan2(offset, focal))


def camera_to_robot_bearing(camera_bearing_deg, servo_deg,
                            centre_deg=config.SERVO_CENTER_DEG,
                            servo_dir=config.SERVO_DIR):
    """Convert a camera-frame bearing into a chassis-relative bearing.

    servo_deg is where the pan servo is pointing; centre_deg is the angle at
    which the camera looks straight down the chassis. servo_dir (+1/-1)
    absorbs which way your linkage turns, so a mirrored mount is a config
    change rather than a code change.

    Returns degrees in [-180, 180): 0 = dead ahead, positive = robot's right.
    """
    pan_off_nose = servo_dir * (servo_deg - centre_deg)
    return wrap180(pan_off_nose + camera_bearing_deg)


def wrap180(angle_deg):
    """Normalize an angle to [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def robot_bearing_to_steer_signal(bearing_deg, saturate_deg=90.0):
    """Turn a chassis-relative pursuer bearing into evasion.escape()'s `bearing`.

    escape() computes `heading = -bearing`, so this returns the value that
    makes the robot steer to put the pursuer BEHIND it:

        pursuer dead ahead (0 deg)   -> turn hard (spin away)
        pursuer on the right (+90)   -> turn left
        pursuer behind (180)         -> drive straight, already fleeing
        pursuer on the left (-90)    -> turn right

    The naive `-bearing/180` gets the behind case badly wrong (it would order
    a hard turn while the robot is already pointed the right way), so we
    steer on the error between where we face and where we WANT to face, which
    is directly away from the pursuer.
    """
    # Angle we want the nose pointing: directly away from the pursuer.
    desired = wrap180(bearing_deg + 180.0)
    # escape() negates, so pre-negate to get `heading == desired/saturate`.
    signal = -desired / saturate_deg
    return max(-1.0, min(1.0, signal))


def range_to_proximity(range_m,
                       near_m=config.RANGE_NEAR_M,
                       far_m=config.RANGE_FAR_M):
    """Map a measured LiDAR range (metres) to proximity in [0, 1].

    Replaces the bbox-height proxy now that the LiDAR gives a real distance:
    1.0 = on top of us, 0.0 = far away or not seen (range inf).
    """
    if range_m is None or math.isinf(range_m):
        return 0.0
    if far_m <= near_m:
        return 0.0
    proximity = (far_m - range_m) / (far_m - near_m)
    return max(0.0, min(1.0, proximity))
