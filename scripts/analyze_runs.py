"""Compute time-to-capture and supporting stats from run logs.

Time-to-capture is the project's primary evaluation metric, and RunLogger
already writes everything needed for it -- this turns those CSVs into the
numbers and plots that go in the report.

    python -m scripts.analyze_runs data/runs/*.csv
    python -m scripts.analyze_runs data/runs/*.csv --plot out.png

Per run it reports: survival time (time-to-capture, or the full run length if
the pursuer never got within CAUGHT_PROXIMITY), loop FPS, how long the pursuer
was actually visible, how often the corner override could have fired, and --
when the identity gate is on -- who was being fled. Across runs it reports
mean/median/min/max survival, which is the headline number: a policy change is
an improvement only if it raises survival time across several runs, not one.

Reads only the CSV, so it runs anywhere (no camera, board, or LiDAR needed).
"""

import argparse
import csv
import math
import statistics
import sys

from app import config


def _f(row, key):
    """Parse a possibly-blank CSV cell as float; blank/inf -> None."""
    v = row.get(key, "")
    if v == "" or v is None:
        return None
    try:
        return float(v)
    except ValueError:
        return None


def load_run(path):
    """Read one run CSV into a list of dict rows."""
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def summarize(path):
    """Reduce one run CSV to a stats dict, or None if the file has no rows."""
    rows = load_run(path)
    if not rows:
        return None

    times = [_f(r, "t") or 0.0 for r in rows]
    duration = max(times)

    # Time-to-capture: timestamp of the first caught==1 row. If the pursuer
    # never got close enough, the robot survived the whole run -- record the
    # duration and flag it, so escaped runs are not mistaken for instant
    # captures when averaging.
    caught_t = None
    for r in rows:
        if (r.get("caught") or "0").strip() == "1":
            caught_t = _f(r, "t")
            break
    escaped = caught_t is None
    survival = duration if escaped else caught_t

    fps = [v for v in (_f(r, "fps") for r in rows) if v]
    visible = [r for r in rows if _f(r, "proximity") not in (None, 0.0)]

    # Frames where both front-side sectors were inside CORNER_DISTANCE, i.e.
    # the corner override in evasion.escape() would have taken control.
    cornered = 0
    for r in rows:
        fl, fr = _f(r, "front_left"), _f(r, "front_right")
        if (fl is not None and fr is not None
                and fl < config.CORNER_DISTANCE and fr < config.CORNER_DISTANCE):
            cornered += 1

    names = {r.get("identity", "") for r in rows} - {""}

    return {
        "path": path,
        "rows": len(rows),
        "duration": duration,
        "survival": survival,
        "escaped": escaped,
        "fps_mean": statistics.mean(fps) if fps else 0.0,
        "fps_min": min(fps) if fps else 0.0,
        "visible_frac": len(visible) / len(rows),
        "cornered_frac": cornered / len(rows),
        "max_proximity": max((_f(r, "proximity") or 0.0) for r in rows),
        "identities": sorted(names),
    }


def print_run(s):
    tag = "ESCAPED (never caught)" if s["escaped"] else "caught"
    print(f"\n{s['path']}")
    print(f"  survival time    {s['survival']:6.1f} s   [{tag}]")
    print(f"  run length       {s['duration']:6.1f} s   ({s['rows']} frames)")
    print(f"  loop FPS         {s['fps_mean']:6.1f} mean, {s['fps_min']:.1f} min"
          + ("   <-- BELOW 10 FPS GATE" if s["fps_mean"] < 10 else ""))
    print(f"  pursuer visible  {s['visible_frac'] * 100:5.1f}% of frames")
    print(f"  cornered         {s['cornered_frac'] * 100:5.1f}% of frames")
    print(f"  peak proximity   {s['max_proximity']:6.2f}   "
          f"(caught threshold {config.CAUGHT_PROXIMITY})")
    if s["identities"]:
        print(f"  fleeing          {', '.join(s['identities'])}")


def print_aggregate(stats):
    surv = [s["survival"] for s in stats]
    escapes = sum(1 for s in stats if s["escaped"])
    print("\n" + "=" * 52)
    print(f"AGGREGATE over {len(stats)} run(s)")
    print(f"  mean survival    {statistics.mean(surv):6.1f} s")
    print(f"  median survival  {statistics.median(surv):6.1f} s")
    print(f"  range            {min(surv):6.1f} s .. {max(surv):.1f} s")
    if len(surv) > 1:
        print(f"  std dev          {statistics.stdev(surv):6.1f} s")
    print(f"  escaped          {escapes}/{len(stats)} runs")
    print(f"  mean loop FPS    {statistics.mean(s['fps_mean'] for s in stats):6.1f}")
    if len(surv) < 3:
        print("\n  NOTE: with fewer than 3 runs these numbers are noise. Collect"
              "\n  several runs per configuration before claiming an improvement.")


def plot(stats, out_path):
    """Bar chart of survival time per run; escaped runs marked."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n[plot] matplotlib not installed; skipping (pip install matplotlib)")
        return
    labels = [s["path"].split("/")[-1].replace(".csv", "") for s in stats]
    values = [s["survival"] for s in stats]
    colors = ["tab:green" if s["escaped"] else "tab:red" for s in stats]
    fig, ax = plt.subplots(figsize=(max(6, len(stats) * 1.2), 4))
    ax.bar(labels, values, color=colors)
    ax.axhline(statistics.mean(values), ls="--", c="gray",
               label=f"mean {statistics.mean(values):.1f} s")
    ax.set_ylabel("survival time (s)")
    ax.set_title("Time-to-capture per run (green = escaped)")
    ax.legend()
    plt.xticks(rotation=30, ha="right")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"\n[plot] wrote {out_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+", help="run CSV files")
    ap.add_argument("--plot", metavar="PNG", help="write a survival bar chart")
    ap.add_argument("--quiet", action="store_true",
                    help="aggregate only, skip per-run detail")
    args = ap.parse_args()

    stats = []
    for path in args.logs:
        try:
            s = summarize(path)
        except OSError as exc:
            print(f"[skip] {path}: {exc}", file=sys.stderr)
            continue
        if s is None:
            print(f"[skip] {path}: no data rows", file=sys.stderr)
            continue
        stats.append(s)
        if not args.quiet:
            print_run(s)

    if not stats:
        sys.exit("no usable run logs")
    print_aggregate(stats)
    if args.plot:
        plot(stats, args.plot)


if __name__ == "__main__":
    main()
