"""Shared test fixtures.

Keeps synthetic-scene construction in one place so the perception and identity
tests build people the same way.
"""

from collections import namedtuple

import numpy as np
import pytest

Detection = namedtuple("Detection", ["bbox", "score", "cls"])

# Distinct shirt colors (BGR) for identity tests.
BLUE = (200, 60, 30)
RED = (30, 40, 200)
GREEN = (60, 200, 60)
ORANGE = (20, 120, 240)


@pytest.fixture
def scene():
    """Build a synthetic frame plus detections for a list of people.

    people: list of (shirt_bgr, x_center_px, box_width_px)
    Returns (frame, [Detection, ...]). Shirts get gaussian noise so they behave
    like real fabric rather than a single exact RGB value.
    """
    rng = np.random.default_rng(11)

    def _make(people, height=480, width=640):
        frame = np.full((height, width, 3), 200, np.uint8)
        dets = []
        for shirt, cx, box_w in people:
            x1, x2 = int(cx - box_w // 2), int(cx + box_w // 2)
            y1, y2 = 60, 420
            patch = (np.array(shirt, np.float32)
                     + rng.normal(0, 15, (y2 - y1, x2 - x1, 3)))
            frame[y1:y2, x1:x2] = np.clip(patch, 0, 255).astype(np.uint8)
            dets.append(Detection(bbox=(x1, y1, x2, y2), score=0.9, cls=0))
        return frame, dets

    return _make
