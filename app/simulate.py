"""Hardware-free 2D flee simulation -- the REAL pipeline in a fake room.

This is not a re-implementation of the robot's logic. Only three things are
simulated: physics (differential-drive kinematics), the camera's answer
(a bounding box for the enrolled person when they are inside the FOV), and
the raw LiDAR returns (ray-cast against walls / chairs / the person).
Everything between sensing and wheels is the actual production code path:

    CameraScanner                sweep/track state machine + servo model
    geometry.bbox_to_camera_bearing_deg / camera_to_robot_bearing
    lidar.merged_points          BOTH units, real +/-90 masks, real offsets
    lidar.sectorize / range_at_bearing / track_at_bearing
    evasion.PursuerTracker       camera->LiDAR hand-off, yaw dead-reckoning
    evasion.EscapePolicy         flee + dodge + corner latch
    evasion.heading_to_pwm       differential steering

So an integration test on this sim genuinely covers the code that runs on
the robot; if a sign flips anywhere in that chain, the simulated robot
drives INTO the pursuer or a wall and the test fails. (The camera-vs-LiDAR
sign mismatch this loop used to have does exactly that here.)

Frames: the world and robot theta use the math convention (CCW positive).
The camera chain reports right-positive bearings; the sim converts exactly
where app/main.py converts, because FleeSim.step IS main.py's loop with the
hardware calls swapped for the fakes.

Used by tests/test_flee_integration.py and scripts/flee_sim.py.
"""

import math
from collections import namedtuple

from . import config
from .control import evasion
from .control.scanner import CameraScanner
from .perception import geometry
from .sensing import lidar

# ---- physics constants (sim only; not robot tunables) ----
MAX_SPEED_MPS = 0.8       # wheel speed at PWM 255
TRACK_WIDTH_M = 0.15      # distance between wheel pairs
ROBOT_RADIUS_M = 0.12     # collision footprint
PERSON_RADIUS_M = 0.18    # a pair of legs, as the LiDAR sees them
CAMERA_SEE_M = 6.0        # detector range in the sim
LIDAR_STEP_DEG = 2        # synthetic angular resolution
LIDAR_MAX_M = 10.0

Point = namedtuple("Point", ["angle_deg", "distance_m", "intensity"])
Circle = namedtuple("Circle", ["x", "y", "r"])            # chairs, people
Segment = namedtuple("Segment", ["x1", "y1", "x2", "y2"])  # walls


def rectangle_room(w, h):
    """Four wall segments enclosing (0,0)..(w,h)."""
    return [Segment(0, 0, w, 0), Segment(w, 0, w, h),
            Segment(w, h, 0, h), Segment(0, h, 0, 0)]


# ---- ray casting ----

def _ray_segment(px, py, dx, dy, s):
    ex, ey = s.x2 - s.x1, s.y2 - s.y1
    denom = dx * ey - dy * ex
    if abs(denom) < 1e-12:
        return None
    t = ((s.x1 - px) * ey - (s.y1 - py) * ex) / denom
    u = ((s.x1 - px) * dy - (s.y1 - py) * dx) / denom
    if t > 1e-9 and 0.0 <= u <= 1.0:
        return t
    return None


def _ray_circle(px, py, dx, dy, c):
    fx, fy = px - c.x, py - c.y
    b = 2 * (fx * dx + fy * dy)
    cc = fx * fx + fy * fy - c.r * c.r
    disc = b * b - 4 * cc
    if disc < 0:
        return None
    sq = math.sqrt(disc)
    for t in ((-b - sq) / 2, (-b + sq) / 2):
        if t > 1e-9:
            return t
    return None


def _point_segment_dist(px, py, s):
    ex, ey = s.x2 - s.x1, s.y2 - s.y1
    ln2 = ex * ex + ey * ey
    t = max(0.0, min(1.0, ((px - s.x1) * ex + (py - s.y1) * ey) / ln2))
    return math.hypot(px - (s.x1 + t * ex), py - (s.y1 + t * ey))


class World:
    """Static walls + chairs; people are added per-cast so they can move."""

    def __init__(self, walls, chairs=()):
        self.walls = list(walls)
        self.chairs = list(chairs)

    def cast(self, px, py, ang_rad, extra_circles=(), max_m=LIDAR_MAX_M):
        dx, dy = math.cos(ang_rad), math.sin(ang_rad)
        best = max_m
        for s in self.walls:
            t = _ray_segment(px, py, dx, dy, s)
            if t is not None and t < best:
                best = t
        for c in list(self.chairs) + list(extra_circles):
            t = _ray_circle(px, py, dx, dy, c)
            if t is not None and t < best:
                best = t
        return best if best < max_m else None

    def clearance(self, x, y):
        """Distance from (x, y) to the nearest wall or chair SURFACE."""
        best = math.inf
        for s in self.walls:
            best = min(best, _point_segment_dist(x, y, s))
        for c in self.chairs:
            best = min(best, math.hypot(x - c.x, y - c.y) - c.r)
        return best


class Robot:
    def __init__(self, x, y, theta_rad):
        self.x, self.y, self.theta = x, y, theta_rad

    def drive(self, left_pwm, right_pwm, dt, world=None):
        vl = left_pwm / 255.0 * MAX_SPEED_MPS
        vr = right_pwm / 255.0 * MAX_SPEED_MPS
        v = (vl + vr) / 2.0
        omega = (vr - vl) / TRACK_WIDTH_M   # +CCW: right faster -> left turn
        self.theta += omega * dt
        nx = self.x + v * math.cos(self.theta) * dt
        ny = self.y + v * math.sin(self.theta) * dt
        if world is not None and world.clearance(nx, ny) < ROBOT_RADIUS_M:
            return True     # solid world: motion blocked, collision recorded
        self.x, self.y = nx, ny
        return False


class SimServo:
    """The firmware's deterministic slew, driven by sim time. Tracks the
    TRUE horn angle; the scanner keeps its own estimate of the same thing,
    and the sim measures the camera from the truth."""

    def __init__(self, angle=config.SERVO_CENTER_DEG):
        self.angle = float(angle)

    def slew_toward(self, target_deg, dt):
        step = config.SERVO_SLEW_DPS * dt
        diff = max(-step, min(step, target_deg - self.angle))
        self.angle += diff


def synth_scan(world, robot, offset, people):
    """One synthetic LD19 revolution IN THE UNIT'S LOCAL FRAME (the frame
    the driver outputs: degrees 0..360, CCW, 0 = unit's forward axis).

    offset = (ox, oy, oyaw) exactly as in config.*_LIDAR_OFFSET. Returned
    Points carry local angles, so the production lidar.merged_points() must
    apply the real +/-90 masks and the real frame transform to make sense
    of them -- which is the point.
    """
    ox, oy, oyaw = offset
    ux = robot.x + ox * math.cos(robot.theta) - oy * math.sin(robot.theta)
    uy = robot.y + ox * math.sin(robot.theta) + oy * math.cos(robot.theta)
    uh = robot.theta + oyaw
    pts = []
    for a in range(0, 360, LIDAR_STEP_DEG):
        d = world.cast(ux, uy, uh + math.radians(a), extra_circles=people)
        if d is not None:
            pts.append(Point(float(a), d, 200))
    return pts


def camera_bbox(robot, servo_true_deg, person_xy, person_dist):
    """What the identity-gated detector would hand the scanner: a bounding
    box for the ENROLLED person, or None when outside the FOV / too far.

    Inverts geometry.bbox_to_camera_bearing_deg exactly (tangent model), so
    the production pixel->bearing math is what gets exercised.
    """
    if person_dist > CAMERA_SEE_M:
        return None
    # Person bearing off the chassis nose, in the CAMERA chain's convention
    # (positive = robot's right):
    bearing_ccw = math.degrees(math.atan2(person_xy[1] - robot.y,
                                          person_xy[0] - robot.x)
                               - robot.theta)
    bearing_right = geometry.wrap180(-bearing_ccw)
    cam_off = geometry.wrap180(
        bearing_right - config.SERVO_DIR
        * (servo_true_deg - config.SERVO_CENTER_DEG))
    if abs(cam_off) > config.CAMERA_HFOV_DEG / 2:
        return None
    focal = 0.5 / math.tan(math.radians(config.CAMERA_HFOV_DEG) / 2.0)
    frac = 0.5 + math.tan(math.radians(cam_off)) * focal
    cx = frac * config.FRAME_WIDTH
    half_w = 40.0
    # Box height shrinks with distance (only the bbox-proximity fallback
    # ever reads it; range normally comes from the LiDAR).
    h = config.FRAME_HEIGHT * max(0.1, min(0.95, 0.9 / max(person_dist, 0.5)))
    y0 = (config.FRAME_HEIGHT - h) / 2
    return (cx - half_w, y0, cx + half_w, y0 + h)


class FleeSim:
    """app/main.py's control loop, transplanted into the fake room.

    Step order, frame conversions, and every decision mirror main.py; the
    hardware calls (camera, bridge, serial LiDARs) are replaced by the fakes
    above. history records one dict per tick for tests and plotting.
    """

    def __init__(self, world, robot, pursuer_xy, pursuer_speed=0.45,
                 dt=0.08, use_front=True, use_rear=True,
                 pursuer_moves=True, enrolled_visible=True):
        self.world = world
        self.robot = robot
        self.px, self.py = pursuer_xy
        self.pursuer_speed = pursuer_speed
        self.dt = dt
        self.use_front = use_front
        self.use_rear = use_rear
        self.pursuer_moves = pursuer_moves
        self.enrolled_visible = enrolled_visible
        self.scanner = CameraScanner()
        self.servo = SimServo(self.scanner.angle)
        self.tracker = evasion.PursuerTracker()
        self.policy = evasion.EscapePolicy()
        self.investigator = evasion.SearchInvestigator()
        self._last_pwm = (0, 0)
        self.t = 0.0
        self.history = []

    def pursuer_distance(self):
        return math.hypot(self.px - self.robot.x, self.py - self.robot.y)

    def step(self):
        r, w = self.robot, self.world
        person = Circle(self.px, self.py, PERSON_RADIUS_M)

        # --- perception (simulated sensing, real geometry) ---
        d_true = self.pursuer_distance()
        bbox = (camera_bbox(r, self.servo.angle, (self.px, self.py), d_true)
                if self.enrolled_visible else None)
        name = "jaafar" if bbox is not None else None

        # --- pan servo: sweep until found, then track (real code) ---
        servo_deg, bearing_deg = self.scanner.update(bbox, name, now=self.t)

        # --- dual LiDAR through the REAL mask/merge/sectorize path ---
        front = (synth_scan(w, r, config.FRONT_LIDAR_OFFSET, [person])
                 if self.use_front else [])
        rear = (synth_scan(w, r, config.REAR_LIDAR_OFFSET, [person])
                if self.use_rear else [])
        points = lidar.merged_points(front, rear)
        sectors = lidar.sectorize(points)

        # --- camera -> LiDAR pursuit hand-off (mirrors main.py) ---
        lp, rp = self._last_pwm
        self.tracker.rotate((rp - lp) * config.YAW_DPS_PER_PWM_DIFF
                            * self.dt)
        source = None
        bearing_ccw = None
        pursuer_range = math.inf
        if bearing_deg is not None:
            bearing_ccw = -bearing_deg          # camera(+right) -> CCW
            # Range for the camera's bearing. When the tracker already has
            # a range estimate, reuse the range-gated angle-closest match
            # (a chair or wall edge inside the +/-8 deg window can sit
            # NEARER than the person, and a raw min() would report its
            # distance instead; the gate keeps the selection on returns
            # consistent with the pursuit). First fix has no hint -> min().
            prev = self.tracker.get(self.t)
            hinted = None
            if prev is not None and prev[1] is not None:
                hinted = lidar.track_at_bearing(
                    points, bearing_ccw,
                    tol_deg=config.BEARING_WINDOW_DEG,
                    range_hint_m=prev[1])
            pursuer_range = (hinted[1] if hinted is not None
                             else lidar.range_at_bearing(points, bearing_ccw))
            self.tracker.update(bearing_ccw, pursuer_range, self.t)
            source = "camera"
        else:
            predicted = self.tracker.get(self.t)
            if predicted is not None:
                pb, pr = predicted
                fix = lidar.track_at_bearing(points, pb, range_hint_m=pr)
                if fix is not None:
                    bearing_ccw, pursuer_range = fix
                    self.tracker.update(bearing_ccw, pursuer_range, self.t)
                    source = "lidar"
                else:
                    bearing_ccw = pb
                    pursuer_range = pr if pr is not None else math.inf
                    source = "memory"
                servo_deg = self.scanner.override_target(
                    config.SERVO_CENTER_DEG
                    + config.SERVO_DIR * (-bearing_ccw))

        # --- search-time investigation (LiDAR tells the camera where to
        # look; identity gate untouched -- nothing flees until confirmed) ---
        investigate = None
        if bearing_ccw is None and source is None:
            investigate = self.investigator.step(points, self.t)
            if investigate is not None and investigate[0] == "aim":
                servo_deg = self.scanner.override_target(
                    config.SERVO_CENTER_DEG
                    + config.SERVO_DIR * (-investigate[1]))
        else:
            self.investigator.reset()

        proximity = geometry.range_to_proximity(pursuer_range)
        if proximity == 0.0 and bbox is not None:
            proximity = geometry.bbox_to_proximity(bbox)

        # --- decide & act (real policy, fake physics) ---
        heading, speed = self.policy.step(bearing_ccw, proximity, sectors,
                                          pursuer_range=pursuer_range)
        if investigate is not None and investigate[0] == "spin":
            # Rotate in place to bring a blind-arc blob into camera reach.
            heading = (-config.CURIOSITY_SPIN_HEADING
                       if investigate[1] > 0
                       else config.CURIOSITY_SPIN_HEADING)
            speed = 0.0
        left, right = evasion.heading_to_pwm(heading, speed)
        self._last_pwm = (left, right)
        collided = r.drive(left, right, self.dt, world=w)
        self.servo.slew_toward(servo_deg, self.dt)   # firmware slews horn

        # --- pursuer walks straight at the robot, tags at arm's length ---
        if self.pursuer_moves:
            d = self.pursuer_distance()
            stop_at = 0.35
            if d > stop_at:
                step = min(self.pursuer_speed * self.dt, d - stop_at)
                self.px += (r.x - self.px) / d * step
                self.py += (r.y - self.py) / d * step

        self.t += self.dt
        rec = dict(t=self.t, x=r.x, y=r.y, theta=r.theta,
                   px=self.px, py=self.py,
                   state=self.scanner.state, servo=self.servo.angle,
                   bearing_ccw=bearing_ccw, range_m=pursuer_range,
                   source=source,
                   clearance=w.clearance(r.x, r.y),
                   pursuer_dist=self.pursuer_distance(),
                   heading=heading, speed=speed, pwm=(left, right),
                   investigate=(investigate[0] if investigate else None),
                   collided=collided)
        self.history.append(rec)
        return rec

    def run(self, seconds):
        for _ in range(int(seconds / self.dt)):
            self.step()
        return self.history


def classroom(seed_chairs=True):
    """The standard test room: 6 x 6 m with a couple of chairs mid-floor."""
    chairs = [Circle(2.0, 4.2, 0.25), Circle(4.3, 2.2, 0.25)] \
        if seed_chairs else []
    return World(rectangle_room(6.0, 6.0), chairs)
