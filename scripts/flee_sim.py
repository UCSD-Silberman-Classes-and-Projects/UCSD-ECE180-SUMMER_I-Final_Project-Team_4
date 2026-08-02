"""Run the flee simulation and plot the chase -- the class-demo visual.

The robot runs the REAL pipeline (scanner, geometry, dual-LiDAR masks,
pursuit tracker, escape policy); only physics and raw sensor returns are
simulated. See app/simulate.py.

Usage (from the repo root, venv active):

    python3 -m scripts.flee_sim                       # default: ahead
    python3 -m scripts.flee_sim --scenario behind
    python3 -m scripts.flee_sim --scenario corner_start --seconds 60
    python3 -m scripts.flee_sim --plot flee.png       # save the figure
    python3 -m scripts.flee_sim --all                 # stats for every scenario

Scenarios: ahead, flank, corner_start, chair_line, behind.

For the live presentation:
    python3 -m pytest tests/test_flee_integration.py -v   # the proof (~1 min)
    python3 -m scripts.flee_sim --scenario ahead --plot demo.png   # the picture
"""

import argparse
import collections
import math
import sys

from app import simulate as S

SCENARIOS = {
    "ahead": ((3.0, 3.0, -90), (3.0, 1.2),
              "pursuer dead ahead; robot must turn 180 and hand the chase "
              "from camera to LiDAR"),
    "flank": ((3.0, 3.0, 90), (0.8, 3.0),
              "pursuer approaches from the side, outside the initial view; "
              "LiDAR-guided investigation acquires them"),
    "corner_start": ((2.0, 2.0, 45), (4.0, 4.0),
                     "robot starts boxed near a corner; the escape latch "
                     "commits to a side and gets out"),
    "chair_line": ((3.0, 3.0, 90), (3.0, 5.0),
                   "the flee path threads between two chairs"),
    "behind": ((3.0, 3.0, 90), (3.0, 1.0),
               "pursuer sneaks up through the camera's blind arc; the robot "
               "spins in place to identify, then flees"),
}

SOURCE_COLORS = {"camera": "#1D9E75", "lidar": "#378ADD",
                 "memory": "#EF9F27", None: "#BBBBBB"}


def run_scenario(name, seconds, pursuer_speed):
    pose, pxy, blurb = SCENARIOS[name]
    world = S.classroom()
    robot = S.Robot(pose[0], pose[1], math.radians(pose[2]))
    sim = S.FleeSim(world, robot, pxy, pursuer_speed=pursuer_speed)
    sim.run(seconds)
    return sim, blurb


def print_stats(name, sim, blurb):
    h = sim.history
    dmin = min(x["pursuer_dist"] for x in h)
    dend = h[-1]["pursuer_dist"]
    clr = min(x["clearance"] for x in h)
    hits = sum(x["collided"] for x in h)
    srcs = collections.Counter(x["source"] for x in h)
    tracked = sum(v for k, v in srcs.items() if k)
    print(f"\n=== {name}: {blurb} ===")
    print(f"  duration            {h[-1]['t']:5.1f} s   "
          f"({len(h)} control ticks)")
    print(f"  closest approach    {dmin:5.2f} m   "
          f"{'TAGGED' if dmin <= 0.40 else '(never tagged; tag = 0.40 m)'}")
    print(f"  final separation    {dend:5.2f} m")
    print(f"  min wall clearance  {clr:5.2f} m   "
          f"({hits} collision{'s' if hits != 1 else ''})")
    if tracked:
        cam = srcs.get("camera", 0)
        lid = srcs.get("lidar", 0)
        mem = srcs.get("memory", 0)
        print(f"  pursuit carried by  camera {100*cam//tracked}%  "
              f"lidar {100*lid//tracked}%  memory {100*mem//tracked}%")
    inv = collections.Counter(x.get("investigate") for x in h)
    if inv.get("aim") or inv.get("spin"):
        print(f"  investigations      aim {inv.get('aim', 0)} ticks, "
              f"spin {inv.get('spin', 0)} ticks")
    return dmin > 0.40 and hits == 0


def plot(sim, name, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plot "
              "(pip install matplotlib)")
        return
    h = sim.history
    fig, ax = plt.subplots(figsize=(7, 7))
    for s in sim.world.walls:
        ax.plot([s.x1, s.x2], [s.y1, s.y2], color="#333333", lw=2)
    for c in sim.world.chairs:
        ax.add_patch(plt.Circle((c.x, c.y), c.r, color="#8B6F47",
                                alpha=0.8, label="_chair"))
    # Robot path, coloured by what carried the pursuit at each tick.
    for a, b in zip(h[:-1], h[1:]):
        ax.plot([a["x"], b["x"]], [a["y"], b["y"]],
                color=SOURCE_COLORS[b["source"]], lw=2.2)
    ax.plot([x["px"] for x in h], [x["py"] for x in h],
            color="#E4572E", lw=1.4, ls="--", label="pursuer")
    ax.plot(h[0]["x"], h[0]["y"], "o", color="#111111", ms=9,
            label="robot start")
    ax.plot(h[-1]["x"], h[-1]["y"], "s", color="#111111", ms=9,
            label="robot end")
    ax.plot(h[0]["px"], h[0]["py"], "o", color="#E4572E", ms=9)
    handles, labels = ax.get_legend_handles_labels()
    from matplotlib.lines import Line2D
    for src, col in (("camera", "#1D9E75"), ("lidar", "#378ADD"),
                     ("memory", "#EF9F27"), ("searching", "#BBBBBB")):
        handles.append(Line2D([0], [0], color=col, lw=3))
        labels.append(f"pursuit via {src}" if src != "searching" else src)
    ax.legend(handles, labels, loc="upper left", fontsize=8)
    dmin = min(x["pursuer_dist"] for x in h)
    ax.set_title(f"mousebot flee -- '{name}'  "
                 f"(closest approach {dmin:.2f} m, "
                 f"{sum(x['collided'] for x in h)} collisions)")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    print(f"  plot saved -> {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--scenario", choices=sorted(SCENARIOS), default="ahead")
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--pursuer-speed", type=float, default=0.45,
                    help="pursuer walking speed, m/s (default 0.45)")
    ap.add_argument("--plot", metavar="PNG",
                    help="save a trajectory figure to this file")
    ap.add_argument("--all", action="store_true",
                    help="run every scenario, stats only")
    args = ap.parse_args()

    if args.all:
        ok = True
        for name in SCENARIOS:
            sim, blurb = run_scenario(name, args.seconds, args.pursuer_speed)
            ok &= print_stats(name, sim, blurb)
        sys.exit(0 if ok else 1)

    sim, blurb = run_scenario(args.scenario, args.seconds,
                              args.pursuer_speed)
    ok = print_stats(args.scenario, sim, blurb)
    if args.plot:
        plot(sim, args.scenario, args.plot)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
