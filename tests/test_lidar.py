"""LiDAR pipeline: arc math, masking, frame transform, sectorizing, corners."""

import math
from collections import namedtuple

import pytest

from app import config
from app.sensing import lidar

# lidar.py only ever reads .angle_deg / .distance_m, so tests use a local
# stand-in rather than importing ld19_driver (which needs the lds2d package
# and is only installable on the board).
Point = namedtuple("Point", ["angle_deg", "distance_m", "intensity"])


def ring(distance=2.0, step=5):
    """A full circle of returns at a fixed distance."""
    return [Point(a, distance, 200) for a in range(0, 360, step)]


def test_angle_in_arc_normal_and_wrapped():
    assert lidar._angle_in_arc(30, 20, 60)
    assert not lidar._angle_in_arc(70, 20, 60)
    # Arc wrapping through zero.
    assert lidar._angle_in_arc(350, 340, 20)
    assert lidar._angle_in_arc(10, 340, 20)
    assert not lidar._angle_in_arc(180, 340, 20)


def test_angle_in_arc_handles_negative_angles():
    # Sector definitions in config use negative degrees.
    assert lidar._angle_in_arc(-30, -60, -20)
    assert not lidar._angle_in_arc(-10, -60, -20)


def test_masks_drop_only_masked_arcs():
    pts = ring(step=10)
    kept = lidar.apply_masks(pts, [(150, 210)])
    assert all(not lidar._angle_in_arc(p.angle_deg, 150, 210) for p in kept)
    assert 0 < len(kept) < len(pts)


def test_no_masks_is_a_passthrough():
    pts = ring()
    assert lidar.apply_masks(pts, []) == pts


def test_transform_translates_along_x():
    """A point 1 m dead ahead of a LiDAR mounted 0.1 m forward is 1.1 m out."""
    pts = [Point(0.0, 1.0, 200)]
    out = lidar.to_robot_frame(pts, (0.10, 0.0, 0.0))
    angle, dist = out[0]
    assert dist == pytest.approx(1.10)
    assert angle == pytest.approx(0.0)


def test_transform_rear_unit_yaw_flips_direction():
    """The rear LiDAR faces backward: its 'ahead' is the robot's behind."""
    pts = [Point(0.0, 1.0, 200)]
    out = lidar.to_robot_frame(pts, (-0.10, 0.0, math.pi))
    angle, dist = out[0]
    assert dist == pytest.approx(1.10)
    assert abs(angle) == pytest.approx(180.0, abs=1e-6)


def test_sectorize_reports_nearest_per_sector():
    # Something close dead ahead, everything else far.
    pts = [(0.0, 0.5), (10.0, 3.0), (90.0, 2.0), (-90.0, 4.0)]
    sectors = lidar.sectorize(pts)
    assert sectors["front"] == pytest.approx(0.5)   # nearest of the two front pts
    assert sectors["left"] == pytest.approx(2.0)
    assert sectors["right"] == pytest.approx(4.0)


def test_empty_sector_is_infinite():
    sectors = lidar.sectorize([])
    assert all(v == math.inf for v in sectors.values())


def test_process_covers_every_sector():
    sectors = lidar.process(ring(), ring())
    assert set(sectors) == set(config.SECTORS)
    assert all(v < math.inf for v in sectors.values())


def test_is_cornered_requires_both_front_sides():
    close, far = config.CORNER_DISTANCE / 2, config.CORNER_DISTANCE * 3
    base = {name: math.inf for name in config.SECTORS}

    both = dict(base, front_left=close, front_right=close)
    one = dict(base, front_left=close, front_right=far)
    neither = dict(base, front_left=far, front_right=far)

    assert lidar.is_cornered(both)
    assert not lidar.is_cornered(one)
    assert not lidar.is_cornered(neither)


# ---- range at a bearing (pairs camera direction with LiDAR distance) ----

def test_range_at_bearing_finds_the_nearest_return_there():
    pts = [(0.0, 3.0), (2.0, 1.5), (90.0, 0.4)]
    assert lidar.range_at_bearing(pts, 0.0, half_window_deg=5) == pytest.approx(1.5)


def test_range_at_bearing_ignores_other_directions():
    """A close wall to the side must not be reported as the pursuer's range."""
    pts = [(90.0, 0.3), (0.0, 2.5)]
    assert lidar.range_at_bearing(pts, 0.0, half_window_deg=5) == pytest.approx(2.5)


def test_range_at_bearing_returns_inf_when_nothing_is_there():
    assert lidar.range_at_bearing([(90.0, 1.0)], 0.0, half_window_deg=5) == math.inf
    assert lidar.range_at_bearing([], 0.0) == math.inf


def test_range_at_bearing_handles_wraparound():
    """A pursuer just past 180 deg must still be found."""
    pts = [(179.0, 2.0), (-179.0, 1.2)]
    assert lidar.range_at_bearing(pts, 180.0, half_window_deg=5) == pytest.approx(1.2)


def test_range_at_bearing_window_width_matters():
    pts = [(10.0, 1.0)]
    assert lidar.range_at_bearing(pts, 0.0, half_window_deg=5) == math.inf
    assert lidar.range_at_bearing(pts, 0.0, half_window_deg=15) == pytest.approx(1.0)


def test_merged_points_exposes_robot_frame_points():
    pts = ring(distance=2.0, step=10)
    merged = lidar.merged_points(pts, pts)
    assert merged and len(merged[0]) == 2      # (angle, distance) tuples
