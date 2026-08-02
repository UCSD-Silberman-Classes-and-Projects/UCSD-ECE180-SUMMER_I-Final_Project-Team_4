"""Structured per-frame run logging, for time-to-capture evaluation and debug.

Writes one CSV row per loop iteration. After a run, time-to-capture is the
timestamp of the first row where caught == 1 (minus the run start).
"""

import csv
import time


class RunLogger:
    FIELDS = ["t", "fps", "bearing", "proximity", "caught", "identity",
              "front", "front_left", "front_right", "left", "right", "rear",
              "left_pwm", "right_pwm"]

    def __init__(self, path):
        self._f = open(path, "w", newline="")
        self._w = csv.DictWriter(self._f, fieldnames=self.FIELDS)
        self._w.writeheader()
        self._start = time.time()

    def log(self, fps, bearing, proximity, caught, sectors, left_pwm,
            right_pwm, identity_name=None):
        row = {
            "identity": identity_name or "",
            "t": round(time.time() - self._start, 3),
            "fps": round(fps, 2),
            "bearing": round(bearing, 3) if bearing is not None else "",
            "proximity": round(proximity, 3) if proximity is not None else "",
            "caught": int(caught),
            "left_pwm": left_pwm,
            "right_pwm": right_pwm,
        }
        for name in ("front", "front_left", "front_right", "left", "right", "rear"):
            v = sectors.get(name, float("inf"))
            row[name] = round(v, 3) if v != float("inf") else ""
        self._w.writerow(row)

    def close(self):
        self._f.close()
