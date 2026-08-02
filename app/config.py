"""All tunable constants in one place.

During tuning you will be twiddling these constantly; keep them here rather than
scattered through the code so tuning stays sane.
"""

# ---- Camera / perception ----
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_INDEX = 0                # /dev/video0 (find with: v4l2-ctl --list-devices)
# Capture source: an int V4L2 index (USB webcam) OR a stream URL string
# (ESP32-CAM, e.g. "http://192.168.4.1:81/stream"). Scripts accept --camera to
# override this without editing the file.
CAMERA_SOURCE = CAMERA_INDEX
DETECT_SCORE_THRESHOLD = 0.5    # min confidence to count a person detection
PERSON_CLASS_ID = 0             # class index for "person" in the TFLite model
DETECT_THREADS = 4              # interpreter threads; the UNO Q's Cortex-A53
                                 # is quad-core, and 1 thread measured 6 FPS vs
                                 # the 10 FPS gate. Try 2 or 3 if the LiDAR
                                 # driver(s) also compete for CPU once running.
# Logitech C920 horizontal field of view. Used to convert a pixel offset in
# the frame into an angle off the camera's axis, which only means something
# once combined with where the pan servo is pointing.
CAMERA_HFOV_DEG = 70.42

# ---- Camera pan servo (HS-225MG) ----
# Mechanical limits must match sketch/src/servo_cam.cpp; the firmware clamps
# too, these are here so the MPU sweep never asks for an angle it cannot get.
SERVO_MIN_DEG = 10
SERVO_MAX_DEG = 170
SERVO_CENTER_DEG = 90           # camera looking straight down the chassis
# +1 if increasing servo angle turns the camera toward the robot's LEFT
# (i.e. toward negative robot bearing); -1 if your linkage is mirrored.
# Get this wrong and the robot will flee TOWARD the pursuer -- verify with
# scripts/servo_test.py --verify before driving.
SERVO_DIR = +1

# Search sweep: how far the servo steps per control iteration while hunting.
# At ~12-15 FPS, 3.5 deg/frame is ~45-50 deg/s -- a full 10..170 sweep in
# about 3.3 s. The old 1.5 (~22 deg/s, 7 s per sweep) let a pursuer starting
# on the flank walk all the way up and tag the still-scanning robot in the
# flee simulation. 3.5 stays well under the servo slew (100 deg/s) and under
# ~4 deg/frame the detector still gets usable frames; drop it back toward 2
# only if sweep-blur measurably hurts detection on the real camera.
SCAN_STEP_DEG = 3.5
# Tracking: proportional gain from "how far off-centre the pursuer is" to
# "how far to move the servo". Below 1.0 to avoid overshoot/oscillation.
TRACK_GAIN = 0.3                # fraction of the (smoothed) off-centre error
                                # corrected per re-aim; <1 damps detector noise
TRACK_SMOOTH = 0.3              # EMA factor on the camera-frame bearing:
                                # 1.0 = raw (jittery), 0.3 = heavy smoothing
SERVO_SLEW_DPS = 100.0          # firmware slew rate the MPU-side model replays.
                                # MUST match servo_cam.cpp: SERVO_SLEW_DEG_PER_STEP
                                # (2 deg) / loop period (~20 ms) = 100 deg/s.
                                # If you change one, change the other.
TRACK_DEADZONE_DEG = 3.0        # don't twitch the servo for tiny errors
# Frames the pursuer may be missing before we give up and resume sweeping.
TRACK_LOST_FRAMES = 12

# ---- Range from LiDAR ----
# The camera says WHICH direction the pursuer is; the LiDAR says HOW FAR.
# We look for the nearest return within this half-angle of the pursuer's
# bearing -- wide enough to catch a person, narrow enough to not pick up a
# wall beside them.
BEARING_WINDOW_DEG = 8.0
RANGE_NEAR_M = 0.5              # at/below this -> proximity 1.0
RANGE_FAR_M = 4.0               # at/above this -> proximity 0.0
CAUGHT_RANGE_M = 0.30           # pursuer this close counts as caught.
                                # Was 0.6, which false-triggered constantly:
                                # the range comes from a +/-8 deg window
                                # around the pursuer's bearing, and one scan
                                # revolution where a chair edge or wall
                                # corner strays into that window under the
                                # threshold ended the run instantly.
CAUGHT_HOLD_S = 1.5             # the caught condition must hold
                                # CONTINUOUSLY this long before the run
                                # ends. A single-tick threshold on a noisy
                                # measurement is a false-positive machine; a
                                # real capture (person standing over the
                                # robot) trivially sustains 1.5 s, while a
                                # one-revolution range glitch cannot. Only
                                # MEASURED fixes (camera/lidar) count --
                                # dead-reckoned memory ranges are stale
                                # extrapolations, not evidence of capture.

# ---- Proximity model ----
# Person box height (as a fraction of frame height) used as a distance proxy.
# Larger box = closer person. Tune these two anchors on the real setup.
PROXIMITY_MIN_BOX_FRAC = 0.15   # far  -> proximity ~0
PROXIMITY_MAX_BOX_FRAC = 0.90   # near -> proximity ~1

# ---- "Caught" definition ----
# Loss condition: pursuer proximity crosses this threshold.
CAUGHT_PROXIMITY = 0.85

# ---- LiDAR ----
LIDAR_SCAN_HZ = 10              # LD19 nominal scan rate

# Serial ports. BOTH of our CP2102 USB-UART adapters report the SAME USB
# serial number ("0001" -- see lsusb), so /dev/ttyUSB0 vs /dev/ttyUSB1 is
# decided by enumeration order and CAN SWAP between boots, silently
# exchanging front and rear. Use the /dev/serial/by-path/ names instead:
# they encode the physical hub socket and only change if a plug is moved.
# Run  python3 -m scripts.lidar_test --identify  to determine which path is
# which unit; it prints these two lines ready to paste.
FRONT_LIDAR_PORT = None         # e.g. "/dev/serial/by-path/platform-...usb-0:1.2:1.0-port0"
REAR_LIDAR_PORT = None          # e.g. "/dev/serial/by-path/platform-...usb-0:1.3:1.0-port0"

# The LD19 spins CLOCKWISE (viewed from the top), so its native angles run
# CW -- opposite to the CCW-positive convention used everywhere downstream.
# The driver flips angles at the source when this is True. Verify on real
# hardware with  scripts.lidar_test --watch front : an object placed to the
# unit's LEFT (of its arrow/connector-forward axis) must read near +90.
LD19_NATIVE_CW = True

# Per-unit angular masks: arcs where each LiDAR is IGNORED. Points inside
# are dropped. IMPORTANT: these are in each unit's OWN LOCAL frame (masks
# are applied BEFORE the robot-frame transform in lidar.merge), and local
# 0 deg = that unit's forward axis, +CCW.
#
# Policy: keep only the FRONT SEMICIRCLE of each unit -- everything within
# +/-90 deg of its forward direction -- and drop the rear semicircle. Since
# the front unit points forward and the rear unit points backward, together
# they still cover the full 360 around the robot (each owns the half it
# faces), while each unit's own view of the chassis/other-unit clutter
# (which sits behind it, near local 180) is discarded. That combined
# coverage is why masking the rear halves does NOT create a blind spot
# behind the robot -- the REAR unit's front semicircle looks there.
#
# The masked arc is local 90..270 (the rear semicircle); it does not wrap
# through 0, so [-90,+90] survives. To keep a wider cone later, narrow this
# toward (120, 240); to keep a narrower one, widen toward (60, 300).
#
# WHY +/-120 AND NOT EXACTLY +/-90: exact semicircles tile the circle with
# ZERO overlap, and the two units are mounted ~20 cm apart -- so at close
# range, PARALLAX pulls each unit's surviving half of a target away from the
# seam and the tiles do not meet. Measured in simulation: a person 0.35 m
# dead abeam (robot-frame +90) subtends local 78..134 at the front unit and
# their CENTRE sits at front-unit local ~106 (the 10 cm mount offset shifts
# bearings by ~16 deg at that range); an exact-90 mask keeps only a sliver
# that parallax maps to robot-frame ~60, the rear unit mirrors it to ~120,
# and range_at_bearing(+90) returns inf FOR A PERSON HALF A METRE AWAY --
# the pursuer becomes invisible precisely when acquired on the flank.
# Keeping +/-120 per unit closes the seam down to arm's-length range while
# still cutting the whole 120-deg rear wedge each unit must ignore (its own
# cable/chassis clutter sits at local ~180, comfortably inside the mask).
FRONT_LIDAR_MASKS = [(120, 240)]   # keep +/-120 deg of the front unit's nose
REAR_LIDAR_MASKS = [(120, 240)]    # keep +/-120 deg of the rear unit's nose
# Mounting offsets from robot center (meters, radians). Fill from measurement.
FRONT_LIDAR_OFFSET = (0.10, 0.0, 0.0)      # (x, y, yaw)
REAR_LIDAR_OFFSET = (-0.10, 0.0, 3.14159)  # rear unit faces backward

# Sector boundaries (deg, robot frame, 0 = straight ahead, +CCW).
# Each sector reports the nearest obstacle distance within its arc.
SECTORS = {
    "front":       (-20, 20),
    "front_left":  (20, 60),
    "front_right": (-60, -20),
    "left":        (60, 120),
    "right":       (-120, -60),
    "rear":        (120, 240),   # wraps through 180
}

# ---- Corner detection ----
CORNER_DISTANCE = 0.40          # m; walls closer than this in both front sides = corner
# When fleeing, the PURSUER themselves shows up as a close return in the
# front LiDAR sectors. Obstacle avoidance must not treat them as a wall: if
# it does, the robot brakes for the person it is fleeing and dodges them,
# which at close range turns into a stationary spin instead of a getaway
# arc. These two tolerances decide "the near thing ahead IS the pursuer, not
# an obstacle" -- pursuer bearing within the front cone AND the front
# clearance matching the pursuer's own range. Fleeing (flee_heading) still
# turns us away from them; we just stop braking/dodging for them.
PURSUER_FRONT_CONE_DEG = 65.0   # front + front_left/right span about +/-60
PURSUER_MATCH_TOL_M = 0.40      # front clearance within this of pursuer range
AVOID_DISTANCE = 0.9            # m; obstacle in the forward cone closer than
                                # this starts bending the flee heading toward
                                # the open side (a chair mid-room never trips
                                # the corner override, so it needs this)

# ---- Camera -> LiDAR pursuit hand-off (fleeing) ----
# Fleeing WORKS by putting the pursuer behind the robot -- inside the
# camera's rear blind arc -- so the chase must survive camera blindness.
# While blind, the last known bearing is kept alive and re-tracked against
# the 360-deg LiDAR each revolution.
FLEE_MEMORY_S = 3.0             # bearing survives this long with NO sensor
                                # support at all before the robot stops+scans
LIDAR_TRACK_TOL_DEG = 15.0      # window for re-finding the pursuer in the
                                # scan (wider than BEARING_WINDOW_DEG: a
                                # walking person drifts between revolutions)
LIDAR_TRACK_RANGE_GATE_M = 0.6  # a re-fix must be within this of the last
                                # known range; without the gate the tracker
                                # grabs the wall behind the person the moment
                                # the angular window slips (walls offer a
                                # continuum of returns and the window then
                                # follows ITSELF along the wall)
YAW_DPS_PER_PWM_DIFF = 1.2      # SIGNED dead-reckoning constant: chassis
                                # yaw deg/s (CCW positive) per unit
                                # of (right_pwm - left_pwm). The tracked
                                # bearing lives in the CHASSIS frame; during
                                # the turn-away the chassis out-rotates the
                                # servo, so this rotation compensation is
                                # what keeps the LiDAR window on the person.
                                # CALIBRATE on the robot: command
                                # set_motion(-120, 120), time N full spins,
                                # value = 360*N / seconds / 240.

# ---- Search-time investigation ("what's that near me?") ----
# While SEARCHING, the LiDAR already sees everything around the robot, but a
# camera-only sweep ignores it -- so a pursuer approaching from the camera's
# rear blind arc could walk right up to a robot that never even turns
# around. Investigation fixes that WITHOUT weakening the identity gate: a
# close LiDAR return makes the robot POINT THE CAMERA at it (or rotate in
# place if it is outside the pan range) so the camera can check who it is.
# No fleeing happens until identity is confirmed; strangers and chairs get
# looked at once and ignored.
CURIOSITY_RANGE_M = 1.5         # investigate returns closer than this
CURIOSITY_RECLOSER_M = 0.2      # re-investigate the same direction only if
                                # it got this much closer (an approaching
                                # person re-triggers ~2x/s; a chair never
                                # re-triggers)
CURIOSITY_AIM_S = 1.2           # camera dwell on an investigated blob
                                # before the sweep resumes
CURIOSITY_SPIN_HEADING = 0.7    # in-place spin strength while turning the
                                # chassis toward a blind-arc blob
SAFETY_DISTANCE = 0.25          # m; LiDAR soft-stop distance on the MPU

# ---- Evasion policy ----
BASE_SPEED = 120                # PWM baseline when fleeing
MAX_SPEED = 255                 # PWM cap
PROXIMITY_SPEED_GAIN = 135      # extra PWM scaled by proximity (closer -> faster)
TURN_GAIN = 100                 # how hard bearing maps into differential steering
# Per-motor polarity. If ONE motor is wired backwards, a spin command
# (opposite-sign wheels) makes the robot TRANSLATE and a straight command
# makes it SPIN -- so it does donuts the instant it tries to flee, while
# camera tracking (which never moves the wheels) looks fine. Set the
# offending wheel to -1. scripts/sign_check.py drives each wheel alone and
# tells you which. This is a per-wheel DIRECTION fix; MOTOR_TURN_SIGN below
# is a different thing (whole-chassis turn convention) and cannot repair a
# single inverted motor.
MOTOR_LEFT_SIGN = +1            # set to -1 if the LEFT wheel runs backwards
MOTOR_RIGHT_SIGN = +1           # set to -1 if the RIGHT wheel runs backwards
# Channel mapping. If commanding the RIGHT wheel moves the LEFT one (and
# vice versa), the two motor outputs are crossed -- set this True to swap
# them back in software instead of re-plugging the leads. This swap is
# applied at the bridge BEFORE the per-wheel signs above, so measure/set
# the swap first, then the signs. (Distinct faults: a swap crosses which
# wheel obeys which command; a sign flips one wheel's direction. Donuts can
# come from either -- sign_check phase 1 drives each wheel alone and tells
# them apart, since a swapped channel makes the OTHER wheel move.)
MOTOR_SWAP_LR = False           # True if left/right motor outputs are crossed
MOTOR_TURN_SIGN = +1            # +1 or -1: which way the CHASSIS physically
                                # turns for a wheel differential. Convention:
                                # heading > 0 must turn the robot RIGHT
                                # (clockwise from above). If the robot turns
                                # LEFT instead -- symptom: it does DONUTS
                                # around the pursuer with the camera still
                                # locked on them, because steering saturates
                                # instead of converging -- set this to -1.
                                # No software test can catch this: it depends
                                # on which physical side each motor channel
                                # drives. MEASURE it with:
                                #     python3 -m scripts.sign_check
                                # which also measures YAW_DPS_PER_PWM_DIFF.

# ---- Control loop ----
LOOP_HZ_TARGET = 10             # aspirational loop rate; gated by detector FPS


# ---- Identity gate (target re-ID nice-to-have) ----
# The robot only flees ENROLLED pursuers; strangers are ignored. Gating turns
# on automatically once anyone is enrolled (scripts/enroll.py); set
# IDENTITY_GATE = False to force it off (e.g. for the FPS benchmark).
IDENTITY_GATE = True
IDENTITY_DIR = "data/identities"
ID_MATCH_THRESHOLD = 0.62       # min correlation to accept a match at all
ID_MATCH_MARGIN = 0.08          # winner must beat the runner-up by this much,
                                # else the frame is "too close to call" and no
                                # one is named. THE main fix for confusing
                                # similar/distant people -- raise to be stricter.
ID_VOTE_WINDOW = 8              # frames of score history per person
ID_MIN_VOTES = 3                # frames needed before a decision is allowed
ID_LOST_FRAMES_RESET = 10       # consecutive no-person frames before vote reset
# Torso crop as fractions of the person bbox (sample shirt, not arms/background)
ID_TORSO_X_FRAC = 0.20          # trim this fraction off each side
ID_TORSO_Y_TOP_FRAC = 0.18      # top of torso band (below the head)
ID_TORSO_Y_BOT_FRAC = 0.52      # bottom of torso band (above the hips)
ID_MIN_PATCH_PX = 20            # torso crop smaller than this is unusable
ID_MIN_PERSON_H_FRAC = 0.35     # person bbox must fill >=35% of frame height;
                                # smaller = too far to identify reliably, so
                                # the gate stays silent instead of guessing.
                                # Lower toward 0.25 if you need longer range and
                                # accept more risk; raise for stricter, closer-only.
ID_MIN_SHIRT_PX = 400           # torso must have at least this many non-dark
                                # pixels for a trustworthy colour histogram
ID_IOU_MATCH = 0.30             # bbox IoU to treat a detection as the same person
ID_CANDIDATE_MAX_MISSED = 15    # frames a candidate survives without a match
# Track lock: once a candidate holds >= ID_LOCK_SCORE for ID_LOCK_SECONDS
# straight, commit to that identity permanently (for the life of the track) and
# stop re-questioning it -- gives the servo cam a stable target.
ID_LOCK_SCORE = 0.90            # sustained score required to commit
ID_LOCK_SECONDS = 2.0           # how long it must hold before locking
ID_H_BINS = 30                  # hue histogram bins
ID_S_BINS = 32                  # saturation histogram bins
