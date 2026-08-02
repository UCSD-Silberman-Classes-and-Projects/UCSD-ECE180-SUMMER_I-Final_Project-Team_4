"""Verify the dual-LD19 pipeline: masks, merge, and sector reduction.

Use this during LiDAR bring-up to confirm that (a) the per-unit masks actually
null out the central-clutter / other-LiDAR phantom returns, and (b) the merged
scan lines up in the robot frame. Prints sector minimums each scan; if you have
matplotlib, it also scatter-plots the merged points.

    python -m scripts.lidar_viz --front /dev/ttyUSB0 --rear /dev/ttyUSB1
"""

import argparse

from app.sensing.ld19_driver import LD19
from app.sensing import lidar


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--front", default="/dev/ttyUSB0")
    ap.add_argument("--rear", default="/dev/ttyUSB1")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    front, rear = LD19(args.front), LD19(args.rear)

    try:
        while True:
            fpts, rpts = front.read_scan(), rear.read_scan()
            merged = lidar.merge(fpts, rpts)
            sectors = lidar.sectorize(merged)

            line = "  ".join(
                f"{k}={v:.2f}" if v != float('inf') else f"{k}=--"
                for k, v in sectors.items()
            )
            flag = "  [CORNERED]" if lidar.is_cornered(sectors) else ""
            print(line + flag)

            if args.plot:
                _plot(merged)
    except KeyboardInterrupt:
        pass
    finally:
        front.close()
        rear.close()


def _plot(merged):
    try:
        import math
        import matplotlib.pyplot as plt
        xs = [d * math.cos(math.radians(a)) for a, d in merged]
        ys = [d * math.sin(math.radians(a)) for a, d in merged]
        plt.clf()
        plt.scatter(xs, ys, s=2)
        plt.scatter([0], [0], c="red", marker="s")  # robot center
        plt.gca().set_aspect("equal")
        plt.pause(0.001)
    except ImportError:
        pass


if __name__ == "__main__":
    main()
