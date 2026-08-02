"""Best-effort telemetry snapshot for laptop-side viewers (see
laptop/radar_view.py).

Writes the current control-loop state to a local file as JSON, atomically
(write to a .tmp path, then os.replace) so a concurrent reader never sees a
half-written file. A viewer just polls this file over SSH -- reusing the
access already used to work the board, no new listening port or protocol.

Opt-in only (app/main.py's --telemetry-file flag, off by default): a JSON
encode + small file write per control-loop iteration is cheap, but there is
no reason to pay it on a run nobody is watching live. Never raises: a full
disk or missing directory must not be able to stall the control loop over
something this optional.
"""

import json
import math
import os


def write_frame(path, lidar_points, bearing_deg, pursuer_range_m,
                heading, speed, proximity):
    """lidar_points: [(angle_deg, dist_m), ...], robot frame (+CCW/left,
    same convention as app/control/evasion.py's PursuerTracker/flee_heading).
    bearing_deg: pursuer chassis bearing in that same frame, or None.
    """
    try:
        # float() everything: the LD19 driver (lds2d) hands back numpy
        # float32 for angle/distance, which json can't serialize -- cast to
        # native Python floats rather than trust every caller's dtype.
        payload = {
            "lidar_points": [(float(a), float(d)) for a, d in lidar_points],
            "bearing_deg": None if bearing_deg is None else float(bearing_deg),
            "pursuer_range_m": (None if not math.isfinite(pursuer_range_m)
                                else float(pursuer_range_m)),
            "heading": float(heading),
            "speed": float(speed),
            "proximity": float(proximity),
        }
        tmp = f"{path}.tmp"
        with open(tmp, "w") as f:
            json.dump(payload, f)
        os.replace(tmp, path)
    except (OSError, TypeError, ValueError):
        pass
