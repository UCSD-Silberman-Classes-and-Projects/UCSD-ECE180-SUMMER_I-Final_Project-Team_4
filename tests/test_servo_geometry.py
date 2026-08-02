"""Servo-mounted camera geometry: pixels -> camera angle -> chassis angle -> steer."""

import math

import pytest

from app import config
from app.perception import geometry


def _box_at(frac_x, width=80):
    """Person box whose centre sits at frac_x across the frame."""
    cx = frac_x * config.FRAME_WIDTH
    return (cx - width / 2, 100, cx + width / 2, 400)


# ---- pixels -> degrees off the camera axis ----

def test_centred_box_is_zero_degrees():
    assert geometry.bbox_to_camera_bearing_deg(_box_at(0.5)) == pytest.approx(0.0)


def test_frame_edges_are_half_the_fov():
    half = config.CAMERA_HFOV_DEG / 2
    assert geometry.bbox_to_camera_bearing_deg(_box_at(1.0)) == pytest.approx(half, abs=0.01)
    assert geometry.bbox_to_camera_bearing_deg(_box_at(0.0)) == pytest.approx(-half, abs=0.01)


def test_camera_bearing_sign_and_monotonicity():
    vals = [geometry.bbox_to_camera_bearing_deg(_box_at(f))
            for f in (0.0, 0.25, 0.5, 0.75, 1.0)]
    assert vals == sorted(vals)
    assert vals[0] < 0 < vals[-1]


# ---- camera angle + servo angle -> chassis angle ----

def test_centred_servo_passes_camera_bearing_through():
    b = geometry.camera_to_robot_bearing(12.0, config.SERVO_CENTER_DEG)
    assert b == pytest.approx(12.0 * 1 + 0.0) or b == pytest.approx(12.0)


def test_servo_offset_adds_to_bearing():
    """Camera centred in frame but servo turned 30 deg -> pursuer is 30 deg off the nose."""
    off = config.SERVO_CENTER_DEG + 30
    b = geometry.camera_to_robot_bearing(0.0, off)
    assert abs(b) == pytest.approx(30.0)


def test_servo_dir_flips_the_sign():
    off = config.SERVO_CENTER_DEG + 30
    plus = geometry.camera_to_robot_bearing(0.0, off, servo_dir=+1)
    minus = geometry.camera_to_robot_bearing(0.0, off, servo_dir=-1)
    assert plus == pytest.approx(-minus)


def test_wrap180_normalizes():
    assert geometry.wrap180(190) == pytest.approx(-170)
    assert geometry.wrap180(-190) == pytest.approx(170)
    assert geometry.wrap180(0) == pytest.approx(0)


# ---- chassis angle -> steering signal fed to evasion.escape() ----
# escape() computes heading = -bearing, so we check the RESULTING heading.

def _heading_for(bearing_deg):
    return -geometry.robot_bearing_to_steer_signal(bearing_deg)


def test_pursuer_behind_means_drive_straight():
    """Already fleeing the right way: near-zero steering."""
    assert _heading_for(180.0) == pytest.approx(0.0, abs=0.02)
    assert abs(_heading_for(170.0)) < 0.15


def test_pursuer_on_the_right_turns_left():
    # positive heading = right turn (see evasion.heading_to_pwm), so left is negative
    assert _heading_for(90.0) < -0.9


def test_pursuer_on_the_left_turns_right():
    assert _heading_for(-90.0) > 0.9


def test_pursuer_dead_ahead_turns_hard():
    """The case a naive -bearing/180 gets wrong: must spin away, not go straight."""
    assert abs(_heading_for(0.0)) == pytest.approx(1.0)


def test_steer_signal_is_bounded():
    for deg in range(-180, 181, 5):
        s = geometry.robot_bearing_to_steer_signal(float(deg))
        assert -1.0 <= s <= 1.0


# ---- LiDAR range -> proximity ----

def test_range_to_proximity_anchors():
    assert geometry.range_to_proximity(config.RANGE_NEAR_M) == pytest.approx(1.0)
    assert geometry.range_to_proximity(config.RANGE_FAR_M) == pytest.approx(0.0)


def test_closer_is_higher_proximity():
    near = geometry.range_to_proximity(1.0)
    far = geometry.range_to_proximity(3.0)
    assert near > far


def test_unseen_pursuer_is_not_close():
    """inf range must read as far away, never as 'on top of us'."""
    assert geometry.range_to_proximity(math.inf) == 0.0
    assert geometry.range_to_proximity(None) == 0.0


def test_proximity_clamped_beyond_anchors():
    assert geometry.range_to_proximity(0.05) == 1.0
    assert geometry.range_to_proximity(50.0) == 0.0
