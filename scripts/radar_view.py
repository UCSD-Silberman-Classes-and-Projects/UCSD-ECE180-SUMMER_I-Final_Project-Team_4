"""Radar-style dashboard: LiDAR, where the camera thinks the pursuer is, and
the planned escape path, all overlaid on one map. Runs on your computer, not
the board -- lives outside app/ and scripts/ (both part of the board's own
checkout/deploy) in laptop/, a laptop-only tools tree. Only imports the
hardware-free, pure parts of app/ (config, geometry, sensing.lidar,
control.evasion), never camera/detector/LD19/Bridge.

    python -m laptop.radar_view --stub                      # simulated data
    python -m laptop.radar_view --host chuckduino            # live, over SSH

Live mode expects the board to be running app/main.py with
--telemetry-file /some/path (see app/telemetry.py); this script polls that
file with `ssh <host> cat <path>`, reusing whatever SSH access already
reaches the board (works over Tailscale same as anything else SSH).

One polar plot, robot at the centre, nose pointing up (0 deg = ahead,
positive = CCW/left -- same convention as app/control/evasion.py's
PursuerTracker/flee_heading and the LiDAR's own robot-frame points, so
nothing here needs to re-derive or guess a sign):

    gray dots    LiDAR returns
    red star     pursuer bearing/range (what the camera+LiDAR fusion thinks)
    orange curve projected path for the next PATH_HORIZON_S seconds if the
                 current heading/speed held steady

In live mode heading/speed/proximity come straight from the board's real
EscapePolicy (sent in the telemetry payload) rather than being recomputed
here -- that state (corner-latch, pursuit memory) lives on the board and
can't be faithfully reconstructed from a single snapshot. In --stub mode,
where there is no board, a local EscapePolicy instance fills in for demo
purposes.
"""

import argparse
import json
import math
import subprocess
import time
from dataclasses import dataclass, field

from app import config
from app.control import evasion
from app.perception import geometry
from app.sensing import lidar


@dataclass
class Frame:
    """One telemetry snapshot. This is the shape get_live_frame() returns."""
    lidar_points: list = field(default_factory=list)   # [(angle_deg, dist_m), ...], +CCW/left robot frame
    bearing_deg: float = None                          # pursuer bearing, same frame, or None if not seen
    pursuer_range_m: float = math.inf                   # LiDAR range at that bearing
    heading: float = None                               # from the board's real EscapePolicy (live mode only)
    speed: float = None
    proximity: float = None


# ---------------------------------------------------------------------------
# Telemetry link: poll the file app/telemetry.py writes on the board, over SSH.
# ---------------------------------------------------------------------------

def get_live_frame(host, remote_path):
    """Pull one telemetry snapshot from the board over SSH.

    Reuses the SSH access already used to work the board -- no new socket
    server or protocol, just `ssh host cat path`. Returns None on any
    hiccup (board mid-write despite the atomic rename on its side, SSH
    timeout, stale/missing file); the caller just skips that tick.

    Repeated `ssh` invocations each pay a fresh handshake unless connections
    are multiplexed -- for a much snappier live view, add to ~/.ssh/config:

        Host <your-board-host>
            ControlMaster auto
            ControlPersist 60s
    """
    try:
        result = subprocess.run(
            ["ssh", host, "cat", remote_path],
            capture_output=True, text=True, timeout=2.0)
    except (subprocess.TimeoutExpired, OSError):
        return None
    if result.returncode != 0 or not result.stdout.strip():
        return None
    try:
        data = json.loads(result.stdout)
    except json.JSONDecodeError:
        return None   # caught the board mid-write; rare given the atomic rename

    range_m = data.get("pursuer_range_m")
    return Frame(
        lidar_points=[tuple(p) for p in data.get("lidar_points", [])],
        bearing_deg=data.get("bearing_deg"),
        pursuer_range_m=math.inf if range_m is None else range_m,
        heading=data.get("heading", 0.0),
        speed=data.get("speed", 0.0),
        proximity=data.get("proximity", 0.0),
    )


# ---------------------------------------------------------------------------
# Simulated data, for --stub -- building/checking the map with no board.
# ---------------------------------------------------------------------------

def simulate_frame(t):
    """Fake a Frame: a boxy room boundary and a pursuer circling the robot."""
    lidar_points = []
    for a in range(0, 360, 3):
        rad = math.radians(a)
        # distance to a 4x4 m square room wall, robot at its centre
        wall = min(abs(2.0 / math.cos(rad)) if abs(math.cos(rad)) > 1e-3 else math.inf,
                   abs(2.0 / math.sin(rad)) if abs(math.sin(rad)) > 1e-3 else math.inf)
        lidar_points.append((a, wall))

    bearing_deg = geometry.wrap180(60 * t)     # pursuer slowly orbiting
    pursuer_range_m = 1.5 + 0.5 * math.sin(t)
    return Frame(lidar_points, bearing_deg, pursuer_range_m)


def decide(frame, policy):
    """Frame -> (heading, speed, proximity), via the same EscapePolicy the
    board runs -- used for --stub only; live frames carry the board's own
    real values instead (see Frame.heading/speed/proximity)."""
    sectors = lidar.sectorize(frame.lidar_points)
    if frame.bearing_deg is None:
        return 0.0, 0.0, 0.0
    proximity = geometry.range_to_proximity(frame.pursuer_range_m)
    heading, speed = policy.step(frame.bearing_deg, proximity, sectors,
                                 pursuer_range=frame.pursuer_range_m)
    return heading, speed, proximity


# ---------------------------------------------------------------------------
# Path projection: not a calibrated model -- evasion's heading/speed are
# unitless ([-1, 1] / PWM), with no recorded real-world m/s or deg/s mapping
# anywhere in the codebase, so these two constants are visualization guesses.
# ---------------------------------------------------------------------------

NOMINAL_MAX_SPEED_MPS = 0.5
MAX_TURN_RATE_DEG_S = 90.0
PATH_HORIZON_S = 1.5
PATH_STEPS = 20


def project_path(heading, speed):
    """Short-horizon path preview as (theta_deg, r_m) pairs in the same
    +CCW/left convention as bearing_deg and the LiDAR points, ready to plot
    directly on a polar axes configured with theta_zero_location="N" and
    the default (CCW) theta_direction.

    Sign, straight from app/control/evasion.py's flee_heading() docstring:
    "heading > 0 makes the LEFT wheel faster ... i.e. a clockwise/right
    turn." A right turn moves the nose toward NEGATIVE theta in the
    +CCW/left convention used here, hence the minus sign below.
    """
    v = (speed / config.MAX_SPEED) * NOMINAL_MAX_SPEED_MPS
    dt = PATH_HORIZON_S / PATH_STEPS

    phi, x, y = 0.0, 0.0, 0.0    # phi: nose heading in the +CCW/left frame
    thetas, rs = [0.0], [0.0]
    for _ in range(PATH_STEPS):
        phi -= heading * MAX_TURN_RATE_DEG_S * dt
        x += v * math.cos(math.radians(phi)) * dt
        y += v * math.sin(math.radians(phi)) * dt
        thetas.append(math.degrees(math.atan2(y, x)))
        rs.append(math.hypot(x, y))
    return thetas, rs


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def draw(ax, frame, heading, speed, proximity):
    ax.clear()
    ax.set_theta_zero_location("N")   # 0 deg = up = ahead
    # theta_direction left at its default (CCW/1): matches the +CCW/left
    # convention shared by lidar_points, bearing_deg, and project_path().
    ax.set_rlim(0, 4)

    if frame.lidar_points:
        thetas = [math.radians(a) for a, _ in frame.lidar_points]
        rs = [d for _, d in frame.lidar_points]
        ax.scatter(thetas, rs, s=4, c="gray", label="LiDAR")

    if frame.bearing_deg is not None and not math.isinf(frame.pursuer_range_m):
        ax.scatter([math.radians(frame.bearing_deg)], [frame.pursuer_range_m],
                   c="red", marker="*", s=220, label="pursuer", zorder=4)

    path_theta, path_r = project_path(heading, speed)
    ax.plot([math.radians(t) for t in path_theta], path_r,
           c="orange", lw=2, label="planned path", zorder=3)

    ax.scatter([0], [0], c="tab:blue", marker="^", s=140, zorder=5)
    ax.legend(loc="upper right", bbox_to_anchor=(1.25, 1.1), fontsize=8)
    bearing_txt = "--" if frame.bearing_deg is None else f"{frame.bearing_deg:+.0f} deg"
    ax.set_title(f"bearing={bearing_txt}   heading={heading:+.2f}   "
                f"speed={speed:.0f}   prox={proximity:.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stub", action="store_true",
                    help="simulated data, no board needed")
    ap.add_argument("--host",
                    help="SSH target for the board (e.g. chuckduino, or a "
                         "Tailscale hostname/IP) -- required unless --stub")
    ap.add_argument("--remote-path", default="/tmp/mousebot_telemetry.json",
                    help="path app/main.py's --telemetry-file wrote on the "
                         "board (must match on both sides)")
    args = ap.parse_args()

    if not args.stub and not args.host:
        raise SystemExit("--host is required (or pass --stub for simulated data)")

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        raise SystemExit(
            "matplotlib is required for this script (not in requirements.txt "
            "-- optional viz dependency, same as scripts/lidar_viz.py).\n"
            "  pip install matplotlib")

    plt.ion()
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(projection="polar")

    policy = evasion.EscapePolicy()   # --stub only; live frames bring their own
    t0 = time.time()
    try:
        while plt.fignum_exists(fig.number):
            if args.stub:
                frame = simulate_frame(time.time() - t0)
                heading, speed, proximity = decide(frame, policy)
            else:
                frame = get_live_frame(args.host, args.remote_path)
                if frame is None:
                    plt.pause(0.2)
                    continue
                heading, speed, proximity = frame.heading, frame.speed, frame.proximity
            draw(ax, frame, heading, speed, proximity)
            plt.pause(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close(fig)


if __name__ == "__main__":
    main()
