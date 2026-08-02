"""Escape policy. Pure function so it can be swapped without touching the loop.

Input:  pursuer bearing, pursuer proximity, LiDAR sector distances.
Output: (heading, speed) where heading is in [-1, 1] (-1 hard left, +1 hard
right) and speed is a PWM magnitude.

Start reactive: steer to keep the pursuer behind you, flee faster as they close,
and override toward open space if LiDAR says you are being cornered. A learned
policy can later replace the body of escape() with the same signature.
"""

import math

from .. import config
from ..sensing import lidar


def escape(bearing, proximity, sectors):
    """Return (heading, speed).

    heading: [-1, 1], steering command (differential drive maps it to L/R PWM)
    speed:   PWM magnitude in [0, MAX_SPEED]
    """
    # 1) Base flee: turn AWAY from the pursuer. If they are on our right
    #    (bearing > 0), steer left (negative heading), and vice versa.
    heading = -bearing

    # 2) Speed scales with how close they are -- closer pursuer, faster flee.
    speed = config.BASE_SPEED + config.PROXIMITY_SPEED_GAIN * proximity
    speed = min(speed, config.MAX_SPEED)

    # 3) Corner override: if boxed in ahead, steer toward the most open side
    #    regardless of the pursuer -- escaping the trap takes priority.
    if lidar.is_cornered(sectors):
        heading = _toward_open_side(sectors)

    # 4) Soft-stop scaling: if something is close directly ahead, ease speed so
    #    the turn can take effect before we hit it. (Hard stop lives on the STM32.)
    if sectors["front"] < config.SAFETY_DISTANCE:
        speed *= 0.3

    return _clamp_heading(heading), speed


def _toward_open_side(sectors):
    """Pick the side with more clearance; return a strong heading toward it."""
    left_clear = min(sectors["front_left"], sectors["left"])
    right_clear = min(sectors["front_right"], sectors["right"])
    # more clearance on the left -> steer left (negative heading)
    return -0.9 if left_clear >= right_clear else 0.9


def _clamp_heading(h):
    return max(-1.0, min(1.0, h))


def heading_to_pwm(heading, speed):
    """Map (heading, speed) to (left_pwm, right_pwm) for differential drive.

    MOTOR_TURN_SIGN absorbs which physical side each motor channel drives,
    so a robot wired the other way round is a config change (measured by
    scripts/sign_check.py), not a code change. With the wrong sign the
    steering loop runs POSITIVE feedback: every turn command rotates the
    chassis toward the pursuer instead of away, the bearing never migrates
    behind, heading saturates, and the robot orbits the pursuer in donuts
    with the pan servo still faithfully holding them in frame.
    """
    turn = config.TURN_GAIN * config.MOTOR_TURN_SIGN * heading
    left = speed + turn
    right = speed - turn
    # clamp both wheels into valid PWM range
    left = int(max(-config.MAX_SPEED, min(config.MAX_SPEED, left)))
    right = int(max(-config.MAX_SPEED, min(config.MAX_SPEED, right)))
    return left, right


# ---------------------------------------------------------------------------
# Flee integration: pursuit memory + stateful escape policy
#
# Everything below works in the LiDAR/robot math frame: bearings in degrees,
# positive = CCW/left of the nose (same as lidar.to_robot_frame's atan2).
# main.py converts the camera's right-positive bearing at the boundary.
# ---------------------------------------------------------------------------


def flee_heading(bearing_ccw_deg):
    """Steering command [-1, 1] that turns the nose AWAY from the pursuer.

    Steers on the error between where we face and where we WANT to face
    (directly away), so pursuer-behind means drive straight, not spin:

        pursuer dead ahead (0)      -> hard turn (either way)
        pursuer left (+90 CCW)      -> +1  (turn right)
        pursuer behind (+/-180)     ->  0  (already fleeing, floor it)
        pursuer right (-90 CCW)     -> -1  (turn left)

    heading > 0 makes the LEFT wheel faster (heading_to_pwm), i.e. a
    clockwise/right turn.
    """
    desired = (bearing_ccw_deg + 180.0 + 180.0) % 360.0 - 180.0
    return _clamp_heading(-desired / 90.0)


class PursuerTracker:
    """Keeps the pursuit alive through camera-blind moments.

    Fleeing WORKS by putting the pursuer behind the robot -- inside the
    camera's rear blind arc -- so a robot that stops the moment the camera
    goes blind stops exactly when it is winning. This stores the last
    confirmed chassis bearing + range and keeps them usable while blind:

    - rotate(dyaw_ccw_deg): the bearing lives in the CHASSIS frame, so every
      commanded turn rotates it. The caller dead-reckons yaw from the PWM
      differential it just commanded (config.YAW_DPS_PER_PWM_DIFF). Without
      this, the 180-deg turn-away slides the LiDAR search window off the
      person within a few control ticks.
    - update(bearing, range, now): a fresh fix from EITHER sensor (camera
      geometry or lidar.track_at_bearing) refreshes the clock, so the chase
      continues as long as any sensor still sees them.
    - get(now): predicted (bearing, range), or None once every sensor has
      been silent for FLEE_MEMORY_S -- at which point the robot stops and
      returns to scanning (driving blind on stale data is how you flee into
      the pursuer's shins).
    """

    def __init__(self, horizon_s=None):
        self.horizon = (config.FLEE_MEMORY_S
                        if horizon_s is None else horizon_s)
        self._bearing = None
        self._range = None
        self._stamp = None

    def update(self, bearing_ccw_deg, range_m, now):
        self._bearing = (bearing_ccw_deg + 180.0) % 360.0 - 180.0
        if range_m is not None and math.isfinite(range_m):
            self._range = range_m
        else:
            self._range = None
        self._stamp = now

    def rotate(self, dyaw_ccw_deg):
        """Chassis rotated CCW by dyaw: every chassis bearing rotates the
        opposite way."""
        if self._bearing is not None:
            self._bearing = ((self._bearing - dyaw_ccw_deg + 180.0)
                             % 360.0 - 180.0)

    def get(self, now):
        if self._bearing is None or now - self._stamp > self.horizon:
            return None
        return self._bearing, self._range

    def clear(self):
        self._bearing = None
        self._range = None
        self._stamp = None


class EscapePolicy:
    """Stateful flee policy: obstacle dodging + corner-escape commitment.

    Wraps the same building blocks as escape() but fixes two failure modes
    a pure per-tick function cannot (both were fatal in closed-loop
    simulation before being fixed):

    1. INTERPOLATED dodge. When an obstacle sits in the forward cone, the
       flee heading and an ADDITIVE dodge can exactly cancel (pursuer
       behind-left says steer left, wall ahead-right says steer right ->
       net zero -> the robot crawls dead straight into the wall in stable
       equilibrium). Blending (1-w)*flee + w*dodge guarantees the dodge
       dominates as the obstacle closes.
    2. Corner LATCH. In a real corner the left/right clearances trade the
       lead as the robot rotates, so re-picking the open side every tick
       oscillates +0.9/-0.9 and the robot jitters in place until tagged.
       The first tick is_cornered fires, the chosen side is latched and
       held at full lock -- ignoring the flee heading, which is what the
       latch is fighting -- until the whole forward cone opens past
       AVOID_DISTANCE (under a second of commitment at our turn rate).
    """

    def __init__(self):
        self._corner_side = None

    def step(self, bearing_ccw_deg, proximity, sectors, pursuer_range=None):
        """bearing_ccw_deg: pursuer chassis bearing (deg, +CCW/left) or None
        while searching. pursuer_range: LiDAR range to the pursuer (m), used
        to tell the pursuer apart from a wall. Returns (heading, speed);
        wheels stay stopped while searching -- 0 would read as 'pursuer dead
        ahead' and cause a spin.
        """
        if bearing_ccw_deg is None:
            self._corner_side = None
            return 0.0, 0.0

        heading = flee_heading(bearing_ccw_deg)
        prox = 0.5 if proximity is None else proximity
        speed = config.BASE_SPEED + config.PROXIMITY_SPEED_GAIN * prox
        speed = min(speed, config.MAX_SPEED)

        ahead = min(sectors["front"], sectors["front_left"],
                    sectors["front_right"])

        # Is the nearest thing in the forward cone actually the PURSUER? If
        # so it is not an obstacle: braking or dodging for the person you
        # are fleeing brakes you to a stationary spin instead of letting the
        # escape arc carry you away. flee_heading already turns us off them,
        # so we suppress the brake, the dodge, AND the corner latch for that
        # tick. A real wall (nearer than the pursuer, or off their bearing)
        # still trips all three normally.
        pursuer_ahead = (
            pursuer_range is not None
            and math.isfinite(pursuer_range)
            and abs(bearing_ccw_deg) <= config.PURSUER_FRONT_CONE_DEG
            and abs(ahead - pursuer_range) <= config.PURSUER_MATCH_TOL_M)
        obstacle_ahead = math.inf if pursuer_ahead else ahead
        front_clear = math.inf if pursuer_ahead else sectors["front"]

        if self._corner_side is not None:
            if obstacle_ahead >= config.AVOID_DISTANCE:
                self._corner_side = None          # escaped: resume fleeing
        elif not pursuer_ahead and lidar.is_cornered(sectors):
            self._corner_side = _toward_open_side(sectors)

        if self._corner_side is not None:
            heading = self._corner_side
        elif obstacle_ahead < config.AVOID_DISTANCE:
            weight = 1.0 - obstacle_ahead / config.AVOID_DISTANCE
            heading = ((1.0 - weight) * heading
                       + weight * _toward_open_side(sectors))

        if front_clear < config.SAFETY_DISTANCE:
            speed *= 0.3

        return _clamp_heading(heading), speed


class SearchInvestigator:
    """While SEARCHING: use the LiDAR to decide where the CAMERA should look.

    The camera sweep alone is deaf to the one sensor that already sees 360
    degrees -- so a pursuer approaching through the camera's rear blind arc
    would reach a still-scanning robot untouched. This closes that hole
    while keeping the identity gate intact: a close LiDAR return triggers an
    INVESTIGATION -- aim the camera at it if the pan can reach, otherwise
    rotate the chassis in place until it can -- and the camera then decides
    who it is. Nothing flees until identity is confirmed; a chair gets
    looked at once and then ignored unless something at that bearing comes
    CLOSER (an approaching person re-triggers continuously, a wall never
    does).

    step(points, now) -> None, ("aim", bearing_ccw) or ("spin", bearing_ccw)
    Callers only consult this while the scanner is searching AND the pursuit
    tracker is empty. reset() when a pursuit starts.
    """

    # Pan reach off the nose (deg): servo range each side of centre.
    _PAN_REACH = min(config.SERVO_MAX_DEG - config.SERVO_CENTER_DEG,
                     config.SERVO_CENTER_DEG - config.SERVO_MIN_DEG)

    def __init__(self):
        self._goal = None            # (bearing_ccw at trigger time)
        self._goal_t = None
        self._done = []              # [(bearing_ccw, dist)] already checked

    def reset(self):
        self._goal = None
        self._goal_t = None
        self._done = []

    def _nearest(self, points):
        best = None
        for ang, dist in points:
            if dist < config.CURIOSITY_RANGE_M:
                if best is None or dist < best[1]:
                    a = (ang + 180.0) % 360.0 - 180.0
                    best = (a, dist)
        return best

    def _already_checked(self, bearing, dist):
        for b, d in self._done:
            da = abs((bearing - b + 180.0) % 360.0 - 180.0)
            if da < 30.0 and dist > d - config.CURIOSITY_RECLOSER_M:
                return True
        return False

    def _mark_checked(self, bearing, dist):
        self._done = [(b, d) for (b, d) in self._done
                      if abs((bearing - b + 180.0) % 360.0 - 180.0) >= 30.0]
        self._done.append((bearing, dist))

    def step(self, points, now):
        near = self._nearest(points)
        if near is None:
            self._goal = None
            return None
        bearing, dist = near

        if self._goal is None:
            if self._already_checked(bearing, dist):
                return None
            self._goal = bearing
            self._goal_t = now

        # Follow the CURRENT nearest return while the goal is active (the
        # person is walking; the trigger bearing goes stale immediately).
        if abs(bearing) <= self._PAN_REACH:
            if now - self._goal_t > config.CURIOSITY_AIM_S:
                # Camera has dwelt on it and the detector said nothing:
                # log it as checked and let the sweep move on.
                self._mark_checked(bearing, dist)
                self._goal = None
                return None
            return ("aim", bearing)
        return ("spin", bearing)


class CaughtDetector:
    """Debounced end-of-run detector.

    The instantaneous caught condition (LiDAR range under CAUGHT_RANGE_M, or
    proximity over CAUGHT_PROXIMITY) is one noisy measurement: the range
    window can catch a chair edge or wall corner for a revolution or two
    even with the pursuer far away, and ending the run on a single such
    tick makes the demo die at random. This requires the condition to hold
    CONTINUOUSLY for CAUGHT_HOLD_S: any clean tick resets the clock. A real
    capture -- the pursuer standing over the robot -- sustains the
    condition trivially; a one-revolution glitch cannot.

        update(caught_now, now) -> True once the debounced capture is
        confirmed. held_for(now) reports the current streak (for logging).
    """

    def __init__(self, hold_s=None):
        self.hold_s = config.CAUGHT_HOLD_S if hold_s is None else hold_s
        self._since = None

    def update(self, caught_now, now):
        if not caught_now:
            self._since = None
            return False
        if self._since is None:
            self._since = now
        return (now - self._since) >= self.hold_s

    def held_for(self, now):
        return 0.0 if self._since is None else now - self._since
