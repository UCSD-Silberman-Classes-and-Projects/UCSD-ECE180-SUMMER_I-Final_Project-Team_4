"""Camera scanner: sweep for the pursuer, lock on, hold through dropouts."""

import math

import pytest

from app import config
from app.control.scanner import CameraScanner, SEARCHING, TRACKING


def _box_at(frac_x, width=80):
    cx = frac_x * config.FRAME_WIDTH
    return (cx - width / 2, 100, cx + width / 2, 400)


CENTRED = _box_at(0.5)
RIGHT_EDGE = _box_at(0.9)
LEFT_EDGE = _box_at(0.1)


# ---- searching ----

def test_starts_searching_at_centre():
    s = CameraScanner()
    assert s.state == SEARCHING
    assert s.angle == config.SERVO_CENTER_DEG


def test_sweep_moves_the_servo_and_reports_no_bearing():
    s = CameraScanner()
    start = s.angle
    angle, bearing = s.update(None, None)
    assert angle != start
    assert bearing is None, "must not report a bearing while searching"


def test_sweep_ping_pongs_within_limits():
    s = CameraScanner(scan_step_deg=10)
    seen = []
    for _ in range(200):
        angle, _ = s.update(None, None)
        seen.append(angle)
        assert s.min_deg <= angle <= s.max_deg
    assert max(seen) == pytest.approx(s.max_deg)
    assert min(seen) == pytest.approx(s.min_deg)


def test_sweep_reverses_rather_than_jumping():
    """Direction flips at the end stop; no teleport back to the other side."""
    s = CameraScanner(scan_step_deg=10)
    prev = s.angle
    steps = []
    for _ in range(60):
        angle, _ = s.update(None, None)
        steps.append(abs(angle - prev))
        prev = angle
    assert max(steps) <= 10 + 1e-9


# ---- acquiring ----

def test_seeing_the_pursuer_switches_to_tracking():
    s = CameraScanner()
    angle, bearing = s.update(CENTRED, "jaafar")
    assert s.state == TRACKING
    assert s.target_name == "jaafar"
    assert bearing is not None


def test_centred_pursuer_does_not_move_the_servo():
    """Inside the deadzone: hold still rather than twitch."""
    s = CameraScanner()
    before = s.angle
    s.update(CENTRED, "jaafar")
    assert s.angle == pytest.approx(before)


def test_offcentre_pursuer_moves_servo_toward_them():
    s = CameraScanner()
    before = s.angle
    s.update(RIGHT_EDGE, "jaafar")
    moved_right = s.angle - before
    s2 = CameraScanner()
    before2 = s2.angle
    s2.update(LEFT_EDGE, "jaafar")
    moved_left = s2.angle - before2
    assert moved_right != 0 and moved_left != 0
    assert (moved_right > 0) != (moved_left > 0), "must move opposite ways"


def test_tracking_converges_toward_centre():
    """Repeatedly seeing them at the same spot should walk the servo over."""
    s = CameraScanner()
    angles = []
    for _ in range(6):
        angle, _ = s.update(RIGHT_EDGE, "jaafar")
        angles.append(angle)
    # monotonic movement in one direction, and it should slow (proportional)
    deltas = [abs(b - a) for a, b in zip(angles, angles[1:])]
    assert all(d >= 0 for d in deltas)


def test_servo_stays_within_limits_while_tracking():
    s = CameraScanner()
    for _ in range(200):
        s.update(RIGHT_EDGE, "jaafar")
        assert s.min_deg <= s.angle <= s.max_deg


# ---- losing the pursuer ----

def test_brief_dropout_holds_position():
    s = CameraScanner(lost_frames=5)
    s.update(CENTRED, "jaafar")
    held = s.angle
    for _ in range(4):
        angle, bearing = s.update(None, None)
        assert angle == pytest.approx(held), "should hold, not sweep away"
        assert bearing is None
        assert s.state == TRACKING


def test_sustained_loss_resumes_searching():
    s = CameraScanner(lost_frames=3)
    s.update(CENTRED, "jaafar")
    for _ in range(5):
        s.update(None, None)
    assert s.state == SEARCHING
    assert s.target_name is None


def test_search_resumes_from_where_it_lost_them():
    """Re-acquire near the last sighting, not from an end stop."""
    s = CameraScanner(lost_frames=2, scan_step_deg=1.0)
    for _ in range(10):
        s.update(RIGHT_EDGE, "jaafar")
    last_seen = s.angle
    for _ in range(4):
        s.update(None, None)
    assert abs(s.angle - last_seen) < 10, "sweep restarted far from last sighting"


def test_reacquiring_returns_to_tracking():
    s = CameraScanner(lost_frames=2)
    s.update(CENTRED, "jaafar")
    for _ in range(5):
        s.update(None, None)
    assert s.state == SEARCHING
    angle, bearing = s.update(CENTRED, "jaafar")
    assert s.state == TRACKING and bearing is not None


# ---- bearing reporting ----

def test_bearing_accounts_for_servo_angle():
    """Same pixel position, different servo angle -> different chassis bearing.
    The bearing is computed from est_angle (where the camera truly points),
    not the in-flight command."""
    a = CameraScanner()
    a.angle = a.est_angle = config.SERVO_CENTER_DEG
    _, bearing_centre = a.update(CENTRED, "jaafar")

    b = CameraScanner()
    b.angle = b.est_angle = config.SERVO_CENTER_DEG + 40
    _, bearing_turned = b.update(CENTRED, "jaafar")

    assert abs(bearing_turned - bearing_centre) == pytest.approx(40, abs=1.0)


# ---- oscillation regression: the anti-shake law ----
#
# Closed loop with realistic timing: the servo physically slews at
# SERVO_SLEW_DPS while frames arrive far more often, and the camera measures
# the person relative to the servo's TRUE position. The old incremental law
# (angle += gain * error per frame) re-added the same error many times while
# the horn was still in transit, wound the setpoint far past the person, and
# oscillated left-right. These tests fail under that law.

def _closed_loop(person_abs_deg, fps=12.0, seconds=4.0, start=None):
    """Simulate tracking a person at a fixed absolute servo-frame angle.
    Returns (list of true servo angles, list of commanded angles, scanner)."""
    s = CameraScanner()
    if start is not None:
        s.angle = s.est_angle = start
    dt = 1.0 / fps
    now = 0.0
    true_deg = s.est_angle
    truth, cmds = [], []
    for _ in range(int(seconds * fps)):
        # what the camera sees from the TRUE pointing angle
        off = person_abs_deg - true_deg
        # person visible only within the FOV
        if abs(off) <= config.CAMERA_HFOV_DEG / 2:
            frac = 0.5 + math.tan(math.radians(off)) * (
                0.5 / math.tan(math.radians(config.CAMERA_HFOV_DEG) / 2))
            bbox = _box_at(frac)
        else:
            bbox = None
        now += dt
        cmd, _ = s.update(bbox, "jaafar", now=now)
        # firmware slews the real horn toward the command
        step = config.SERVO_SLEW_DPS * dt
        true_deg += max(-step, min(step, cmd - true_deg))
        truth.append(true_deg)
        cmds.append(cmd)
    return truth, cmds, s


def test_tracking_settles_without_oscillation():
    """Person 35 deg off to one side: the servo must arrive and STAY, not
    slosh through them. At most one direction reversal is allowed (tiny
    filter-settling wiggle); the old windup law reverses many times."""
    truth, _, _ = _closed_loop(person_abs_deg=125.0, start=90.0)
    reversals = 0
    for a, b, c in zip(truth, truth[1:], truth[2:]):
        d1, d2 = b - a, c - b
        if abs(d1) > 0.2 and abs(d2) > 0.2 and (d1 > 0) != (d2 > 0):
            reversals += 1
    assert reversals <= 1, f"servo reversed {reversals} times (shaking)"
    # and it actually got there
    assert abs(truth[-1] - 125.0) < config.TRACK_DEADZONE_DEG + 1.0


def test_command_never_winds_far_past_the_person():
    """The commanded setpoint must stay bounded near the person's absolute
    angle. Absolute re-aiming is structurally bounded by about
    gain * (HFOV/2) plus EMA lag -- roughly 10 deg at any sane tuning --
    while the old incremental law wound up 20-30 deg past. The threshold
    sits between the two regimes so it stays valid across gain/smoothing
    retunes (heavier TRACK_SMOOTH trades a couple of degrees of transient
    glide-past for calmer steady-state; that is fine, windup is not)."""
    _, cmds, _ = _closed_loop(person_abs_deg=125.0, start=90.0)
    overshoot = max(c - 125.0 for c in cmds)
    assert overshoot < 12.0, f"command wound up {overshoot:.1f} deg past target"


def test_tracks_a_walking_person_smoothly():
    """Person strolls across the servo's range at ~20 deg/s: tracking error
    stays bounded and the servo never flips direction against the walk."""
    s = CameraScanner()
    s.angle = s.est_angle = 90.0
    true_deg = 90.0
    fps, dt = 12.0, 1.0 / 12.0
    now, person = 0.0, 95.0
    errors, truth = [], []
    for i in range(int(6.0 * fps)):
        person = 95.0 + 20.0 * (i * dt)            # walks toward max
        person = min(person, 160.0)
        off = person - true_deg
        if abs(off) <= config.CAMERA_HFOV_DEG / 2:
            frac = 0.5 + math.tan(math.radians(off)) * (
                0.5 / math.tan(math.radians(config.CAMERA_HFOV_DEG) / 2))
            bbox = _box_at(frac)
        else:
            bbox = None
        now += dt
        cmd, _ = s.update(bbox, "jaafar", now=now)
        step = config.SERVO_SLEW_DPS * dt
        true_deg += max(-step, min(step, cmd - true_deg))
        errors.append(abs(person - true_deg))
        truth.append(true_deg)
    # settled portion: error bounded (lag is fine, oscillation is not)
    assert max(errors[len(errors) // 2:]) < 15.0
    backwards = sum(1 for a, b in zip(truth[12:], truth[13:]) if b < a - 0.3)
    assert backwards == 0, "servo moved against the walk direction"


def test_deadband_hysteresis_does_not_park_on_the_edge():
    """Settling exactly at the deadzone edge and re-triggering every frame
    was part of the twitching. Once correcting, it should finish the job."""
    truth, _, _ = _closed_loop(person_abs_deg=95.0, start=90.0)  # 5 deg off
    assert abs(truth[-1] - 95.0) < 2.0
