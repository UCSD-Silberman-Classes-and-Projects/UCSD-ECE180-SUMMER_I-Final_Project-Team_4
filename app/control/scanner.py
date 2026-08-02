"""Camera pan control: sweep for the pursuer, then stay locked on them.

Two states, and the whole point of the module is the handoff between them:

    SEARCHING  slowly sweep the servo end to end, running detection +
               the identity gate on every frame, until an ENROLLED person
               is found. Strangers never end the sweep.
    TRACKING   steer the servo to keep that person centred in frame. If they
               are lost for TRACK_LOST_FRAMES, resume sweeping FROM THE
               CURRENT ANGLE (not from the end stop) so we re-acquire near
               where they were last seen.

The servo angle is state the MPU owns, mirrored to the MCU each iteration.
We deliberately do NOT read the angle back over the Bridge every frame: that
would be a blocking round-trip inside the control loop. Instead we keep a
MODEL of the true angle: the firmware slews deterministically toward the
last command at a known rate (SERVO_SLEW_DPS, which must match
SERVO_SLEW_DEG_PER_STEP / loop period in servo_cam.cpp), so the MPU can
replay that motion between frames and know, closely enough, where the
camera is really pointing right now. Callers may re-anchor the model with
resync() from an occasional get_servo() round-trip.

Why the model matters -- the anti-shake fix: the camera measures the person
relative to where the servo ACTUALLY points, but a naive controller adds
that error to the COMMANDED angle every frame. The servo is far slower than
the frame rate, so the same error is re-added several times while the horn
is still in transit and the setpoint winds up far past the person; then the
sign flips and it winds up the other way -- the left-right shake. The cure
is to stop integrating: every frame we re-aim ABSOLUTELY at
(estimated true angle) + gain * (smoothed offset). Repeated sightings of a
stationary person then converge on one setpoint instead of stacking up.

update() is pure decision logic given a detection; it never touches hardware
itself, so it unit-tests without a servo, camera, or board.
"""

import time

from .. import config
from ..perception import geometry

SEARCHING = "searching"
TRACKING = "tracking"


class CameraScanner:
    """Owns the pan servo angle and the search/track state machine."""

    def __init__(self,
                 min_deg=None, max_deg=None, centre_deg=None,
                 scan_step_deg=None, track_gain=None,
                 deadzone_deg=None, lost_frames=None):
        self.min_deg = min_deg if min_deg is not None else config.SERVO_MIN_DEG
        self.max_deg = max_deg if max_deg is not None else config.SERVO_MAX_DEG
        self.centre_deg = centre_deg if centre_deg is not None else config.SERVO_CENTER_DEG
        self.scan_step = scan_step_deg if scan_step_deg is not None else config.SCAN_STEP_DEG
        self.track_gain = track_gain if track_gain is not None else config.TRACK_GAIN
        self.deadzone = deadzone_deg if deadzone_deg is not None else config.TRACK_DEADZONE_DEG
        self.lost_limit = lost_frames if lost_frames is not None else config.TRACK_LOST_FRAMES

        self.state = SEARCHING
        self.angle = self.centre_deg      # commanded servo angle, degrees
        self.est_angle = self.centre_deg  # modelled TRUE angle (slew replay)
        self._last_now = None
        self._sweep_dir = +1
        self._lost = 0
        self._ema = None                  # smoothed camera-frame bearing
        self._correcting = False          # deadband hysteresis latch
        self.target_name = None           # who we are tracking, once known

    # ------------------------------------------------------------------
    def update(self, detection_bbox, pursuer_name, now=None):
        """Advance one control iteration.

        detection_bbox: bbox of the ENROLLED pursuer this frame, or None if
                        they were not seen (nobody there, or only strangers).
        pursuer_name:   identity the gate settled on, or None.
        now:            monotonic seconds; injectable for tests. Used only
                        to advance the servo-position model.

        Returns (servo_angle_deg, robot_bearing_deg or None). The bearing is
        None while searching or when the pursuer is not currently visible --
        callers must treat that as "no pursuer" rather than "bearing 0",
        which would look like the pursuer is dead ahead.
        """
        self._advance_model(time.monotonic() if now is None else now)
        if detection_bbox is None:
            return self._handle_missing()
        return self._handle_seen(detection_bbox, pursuer_name)

    def _advance_model(self, now):
        """Replay the firmware's slew: est_angle chases the last command at
        SERVO_SLEW_DPS. This is what the camera is ACTUALLY doing."""
        if self._last_now is not None:
            dt = max(0.0, now - self._last_now)
            step = config.SERVO_SLEW_DPS * dt
            diff = self.angle - self.est_angle
            self.est_angle += max(-step, min(step, diff))
        self._last_now = now

    def resync(self, actual_deg):
        """Re-anchor the model from a real get_servo() read-back. Cheap to
        call every second or two; unnecessary every frame."""
        self.est_angle = float(actual_deg)

    def override_target(self, servo_deg):
        """External aim command -- used by the pursuit tracker to point the
        camera at its predicted bearing while the camera is blind, so it
        re-acquires the instant the person swings back into view. Setting
        self.angle (the command) rather than bypassing the scanner keeps the
        slew model truthful: est_angle keeps chasing what the firmware is
        actually being told. Returns the clamped command to send."""
        self.angle = self._clamp(float(servo_deg))
        return self.angle

    # ------------------------------------------------------------------
    def _handle_seen(self, bbox, pursuer_name):
        self._lost = 0
        self.target_name = pursuer_name or self.target_name
        self.state = TRACKING

        cam_bearing = geometry.bbox_to_camera_bearing_deg(bbox)

        # Smooth the detector's frame-to-frame centroid jitter before it
        # reaches the servo. Seeded with the first sample so acquisition is
        # not sluggish.
        if self._ema is None:
            self._ema = cam_bearing
        else:
            a = config.TRACK_SMOOTH
            self._ema = a * cam_bearing + (1.0 - a) * self._ema

        # Deadband with hysteresis: start correcting only past deadzone, but
        # once correcting, keep going until nearly centred -- otherwise the
        # servo parks at the deadzone EDGE and re-triggers endlessly.
        if not self._correcting and abs(self._ema) > self.deadzone:
            self._correcting = True
        elif self._correcting and abs(self._ema) < 0.4 * self.deadzone:
            self._correcting = False

        # ABSOLUTE re-aim, not an increment: the offset was measured from
        # where the camera really points (est_angle), so that is the base it
        # must be applied to. Because each frame recomputes the same
        # absolute setpoint for a stationary person, nothing accumulates and
        # the windup oscillation cannot happen. Gain < 1 damps against
        # detection noise; the sub-degree residue vanishes over frames.
        if self._correcting:
            self.angle = self._clamp(
                self.est_angle
                + config.SERVO_DIR * self.track_gain * self._ema)

        # Report the bearing from the modelled TRUE pointing angle, not the
        # command the servo has not finished executing yet.
        robot_bearing = geometry.camera_to_robot_bearing(cam_bearing,
                                                         self.est_angle)
        return self.angle, robot_bearing

    def _handle_missing(self):
        if self.state == TRACKING:
            self._lost += 1
            if self._lost < self.lost_limit:
                # Hold position briefly: a one-frame detection dropout is
                # common, and sweeping away immediately would lose a pursuer
                # who is still right there.
                return self.angle, None
            # Genuinely lost -- resume sweeping from here, not from an end
            # stop, so we re-acquire near where they vanished.
            self.state = SEARCHING
            self.target_name = None
            self._lost = 0
            self._ema = None
            self._correcting = False

        self._sweep()
        return self.angle, None

    def _sweep(self):
        """Ping-pong the servo across its range, one small step per call."""
        nxt = self.angle + self._sweep_dir * self.scan_step
        if nxt >= self.max_deg:
            nxt = self.max_deg
            self._sweep_dir = -1
        elif nxt <= self.min_deg:
            nxt = self.min_deg
            self._sweep_dir = +1
        self.angle = nxt

    def _clamp(self, deg):
        return max(self.min_deg, min(self.max_deg, deg))

    @property
    def is_tracking(self):
        return self.state == TRACKING
