"""Thin wrapper over lds2d's driver for the LDRobot LD19.

lds2d (https://pypi.org/project/lds2d/, Apache-2.0) is a maintained,
plain-pyserial Python port of the kaiaai/LDS C++ library. Its LDROBOT-LD19
driver parses the 47-byte, 0x54-header, 12-points-per-packet protocol
directly (angle in centidegrees, distance in mm, CRC8-validated), ported
from LDRobot's own reference source. This was chosen over hand-rolling a
parser per the project's stated preference for a maintained implementation.

Caveat worth knowing: lds2d's own README marks LD19 support "spec" rather
than hardware-verified -- it's ported from kaiaai/LDS and unit-tested
against synthetic packets, but not yet confirmed against a physical LD19 by
lds2d's maintainers. Validate this wrapper's output against a known object
with scripts/lidar_viz.py before trusting it for navigation.

Angle convention here: degrees, 0 = the LiDAR's forward axis, CCW positive.
lds2d's ScanPoint.angle_deg is 0-360, increasing in the sensor's own scan
direction; verify with lidar_viz.py which way that actually runs on your
unit (a known object placed to one side should land at the expected sign),
and flip the sign in _parse() below if it turns out to run CW instead.
"""

from collections import namedtuple

from lds2d import Lidar

Point = namedtuple("Point", ["angle_deg", "distance_m", "intensity"])


class LD19:
    def __init__(self, port, baud=230400):
        self._lidar = Lidar.open("LDROBOT-LD19", port, baud=baud)
        self._scans = self._lidar.scans()

    def read_scan(self):
        """Block until one full 360 revolution is assembled; return list[Point].

        Drops zero-distance "no return" points (lds2d's valid_points) rather
        than passing them through: a raw 0 distance would otherwise always
        win the nearest-obstacle-per-sector reduction in sensing/lidar.py.
        """
        scan = next(self._scans)
        return [self._parse(p) for p in scan.valid_points]

    def _parse(self, point):
        return Point(point.angle_deg, point.dist_mm / 1000.0, point.quality)

    def close(self):
        self._lidar.close()
