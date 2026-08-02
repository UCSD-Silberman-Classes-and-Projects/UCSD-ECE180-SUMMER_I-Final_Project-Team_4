"""Escape policy: flee direction, speed scaling, corner override, PWM mapping."""

import math

import pytest

from app import config
from app.control import evasion


@pytest.fixture
def clear():
    """All sectors wide open."""
    return {name: math.inf for name in config.SECTORS}


def test_flees_away_from_pursuer(clear):
    # Pursuer on the right (+bearing) -> steer left (-heading), and vice versa.
    right, _ = evasion.escape(0.8, 0.5, clear)
    left, _ = evasion.escape(-0.8, 0.5, clear)
    assert right < 0
    assert left > 0


def test_heading_is_clamped(clear):
    for bearing in (-5.0, 5.0):
        heading, _ = evasion.escape(bearing, 0.5, clear)
        assert -1.0 <= heading <= 1.0


def test_closer_pursuer_means_faster(clear):
    _, slow = evasion.escape(0.0, 0.1, clear)
    _, fast = evasion.escape(0.0, 0.9, clear)
    assert fast > slow
    assert slow >= config.BASE_SPEED


def test_speed_never_exceeds_max(clear):
    _, speed = evasion.escape(0.0, 1.0, clear)
    assert speed <= config.MAX_SPEED


def test_corner_override_beats_pursuer_bearing(clear):
    """Boxed in ahead: escaping the trap outranks fleeing the pursuer."""
    cornered = dict(clear)
    cornered["front_left"] = config.CORNER_DISTANCE / 2
    cornered["front_right"] = config.CORNER_DISTANCE / 2
    # Pursuer dead ahead would normally give heading ~0; the override must not.
    heading, _ = evasion.escape(0.0, 0.5, cornered)
    assert abs(heading) == pytest.approx(0.9)


def test_corner_override_steers_toward_the_open_side(clear):
    left_open = dict(clear)
    left_open["front_left"] = 0.35
    left_open["left"] = 3.0          # roomy on the left
    left_open["front_right"] = 0.10
    left_open["right"] = 0.15        # tight on the right
    heading, _ = evasion.escape(0.0, 0.5, left_open)
    assert heading < 0               # negative heading = steer left

    right_open = dict(clear)
    right_open["front_left"] = 0.10
    right_open["left"] = 0.15
    right_open["front_right"] = 0.35
    right_open["right"] = 3.0
    heading, _ = evasion.escape(0.0, 0.5, right_open)
    assert heading > 0


def test_obstacle_ahead_slows_down(clear):
    blocked = dict(clear)
    blocked["front"] = config.SAFETY_DISTANCE / 2
    _, slow = evasion.escape(0.0, 0.5, blocked)
    _, normal = evasion.escape(0.0, 0.5, clear)
    assert slow < normal


def test_pwm_straight_ahead_is_balanced():
    left, right = evasion.heading_to_pwm(0.0, 200)
    assert left == right == 200


def test_pwm_differential_matches_heading_sign():
    # Positive heading = steer right => left wheel faster than right.
    left, right = evasion.heading_to_pwm(0.5, 200)
    assert left > right
    left, right = evasion.heading_to_pwm(-0.5, 200)
    assert left < right


def test_pwm_stays_in_range():
    for heading in (-1.0, 0.0, 1.0):
        for speed in (0, config.MAX_SPEED):
            left, right = evasion.heading_to_pwm(heading, speed)
            assert -config.MAX_SPEED <= left <= config.MAX_SPEED
            assert -config.MAX_SPEED <= right <= config.MAX_SPEED


def test_full_chain_pursuer_right_turns_left(clear):
    """End-to-end: pursuer on the right should spin the left wheel faster."""
    heading, speed = evasion.escape(0.6, 0.7, clear)
    left, right = evasion.heading_to_pwm(heading, speed)
    assert left < right
