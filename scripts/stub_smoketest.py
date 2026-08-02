"""Offline smoke test of the whole MPU control path with no camera, no hardware.

Feeds synthetic person bounding boxes + open LiDAR sectors through geometry,
evasion, and PWM conversion, so the control path can be exercised on any laptop
before the robot exists. Complements benchmark_fps.py (which needs a real camera
and model) and the build order's step 1 ("Bridge with a stub").

    python -m scripts.stub_smoketest
"""

import math

from app import config
from app.perception import geometry
from app.control import evasion
from app.control.bridge_client import StubBridge


def main():
    bridge = StubBridge()
    open_sectors = {name: math.inf for name in config.SECTORS}

    print("Pursuer sweeps left -> right; heading should flee the opposite way:")
    for cx_frac in (0.1, 0.3, 0.5, 0.7, 0.9):
        cx = cx_frac * config.FRAME_WIDTH
        bbox = (cx - 40, 100, cx + 40, 400)
        bearing = geometry.bbox_to_bearing(bbox)
        proximity = geometry.bbox_to_proximity(bbox)
        heading, speed = evasion.escape(bearing, proximity, open_sectors)
        left, right = evasion.heading_to_pwm(heading, speed)
        bridge.set_motion(left, right)
        print(f"  bearing {bearing:+.2f} -> heading {heading:+.2f}  "
              f"(L={left}, R={right})")

    print("\nCornered ahead (both front sides blocked): expect a hard turn:")
    cornered = dict(open_sectors)
    cornered["front_left"] = 0.2
    cornered["front_right"] = 0.2
    heading, speed = evasion.escape(0.0, 0.6, cornered)
    print(f"  heading {heading:+.2f}, speed {speed:.0f}")
    print("\nstub smoke test OK")


if __name__ == "__main__":
    main()
