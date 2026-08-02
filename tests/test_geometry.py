"""Bounding box -> bearing / proximity."""

import pytest

from app import config
from app.perception import geometry


def _box(cx, height_frac=0.5, width=80):
    """Box centered at cx px with the given height as a fraction of the frame."""
    h = height_frac * config.FRAME_HEIGHT
    y1 = (config.FRAME_HEIGHT - h) / 2
    return (cx - width / 2, y1, cx + width / 2, y1 + h)


def test_centered_box_has_zero_bearing():
    assert geometry.bbox_to_bearing(_box(config.FRAME_WIDTH / 2)) == pytest.approx(0.0)


def test_bearing_sign_and_limits():
    # Left half is negative, right half positive, edges saturate at -1 / +1.
    assert geometry.bbox_to_bearing(_box(0)) == pytest.approx(-1.0)
    assert geometry.bbox_to_bearing(_box(config.FRAME_WIDTH)) == pytest.approx(1.0)
    assert geometry.bbox_to_bearing(_box(config.FRAME_WIDTH * 0.25)) < 0
    assert geometry.bbox_to_bearing(_box(config.FRAME_WIDTH * 0.75)) > 0


def test_bearing_is_monotonic_left_to_right():
    xs = [0, 160, 320, 480, 640]
    bearings = [geometry.bbox_to_bearing(_box(x)) for x in xs]
    assert bearings == sorted(bearings)


def test_proximity_clamps_to_unit_range():
    tiny = _box(320, height_frac=0.01)      # far below the near anchor
    huge = _box(320, height_frac=1.0)       # beyond the far anchor
    assert geometry.bbox_to_proximity(tiny) == 0.0
    assert geometry.bbox_to_proximity(huge) == 1.0


def test_proximity_hits_anchors():
    lo, hi = config.PROXIMITY_MIN_BOX_FRAC, config.PROXIMITY_MAX_BOX_FRAC
    assert geometry.bbox_to_proximity(_box(320, height_frac=lo)) == pytest.approx(0.0)
    assert geometry.bbox_to_proximity(_box(320, height_frac=hi)) == pytest.approx(1.0)


def test_taller_box_means_closer():
    near = geometry.bbox_to_proximity(_box(320, height_frac=0.7))
    far = geometry.bbox_to_proximity(_box(320, height_frac=0.3))
    assert near > far
