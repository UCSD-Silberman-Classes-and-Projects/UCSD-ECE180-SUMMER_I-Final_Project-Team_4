"""LD19 bring-up and testing. Run these IN ORDER on first hookup:

    python3 -m scripts.lidar_test --list        # 1. both adapters visible?
    python3 -m scripts.lidar_test --identify    # 2. which port is which unit
    python3 -m scripts.lidar_test --watch front # 3. sane data + angle signs
    python3 -m scripts.lidar_test --verify      # 4. health check both units

--identify exists because both CP2102 adapters report USB serial "0001"
(see lsusb): they are indistinguishable by ID, and /dev/ttyUSB0/1 ordering
can swap between boots. The script tells them apart PHYSICALLY: you cover
one unit with your hand, it sees which port's scan went near-blind, and it
prints the /dev/serial/by-path lines to paste into app/config.py. by-path
names encode the hub socket, so they stay correct across reboots as long as
the plugs stay put.

--watch shows a live 16-wedge console radar plus health counters. Use it to
verify the angle convention: stand to the unit's LEFT (relative to the
direction its arrow / cable-exit faces) and your echo must appear around
+90 deg. If it shows -90, flip LD19_NATIVE_CW in config.py.

No wheels are ever commanded by this script.
"""

import argparse
import sys
import time

sys.path.insert(0, ".")

from app import config                                    # noqa: E402
from app.sensing.ld19_driver import (LD19,                # noqa: E402
                                     find_candidate_ports)


def cmd_list():
    ports = find_candidate_ports()
    if not ports:
        print("No CP210x serial adapters found. Check power to the LiDAR "
              "hats and the USB hub, then re-run.")
        return 1
    print(f"Found {len(ports)} candidate port(s):")
    for p in ports:
        print(f"  {p}")
    if len(ports) < 2:
        print("Expected 2 (front + rear). If only one shows, swap cables "
              "to find the dead link/adapter.")
    if any("/by-path/" not in p for p in ports):
        print("note: raw /dev/ttyUSBn names shown -- /dev/serial/by-path "
              "missing on this system, so port order may vary per boot.")
    return 0


def _mean_range(scan):
    return sum(p.distance_m for p in scan) / max(len(scan), 1)


def cmd_identify():
    ports = find_candidate_ports()
    if len(ports) < 2:
        print("Need both adapters visible first -- run --list.")
        return 1
    ports = ports[:2]
    print(f"Opening {ports[0]}\n    and {ports[1]} ...")
    units = [LD19(p) for p in ports]
    try:
        for u in units:
            u.read_scan()          # wait until both are streaming

        print("\nBoth units streaming. Now: CUP YOUR HANDS AROUND THE "
              "**FRONT** unit\n(the one mounted at the robot's nose), "
              "covering its window,\nand hold for ~3 seconds...")
        covered = _wait_for_covered(units)
        if covered is None:
            print("Couldn't detect a covered unit -- hands close enough to "
                  "the window? Try again.")
            return 1

        front = units[covered]
        rear = units[1 - covered]
        print(f"\nGot it. The unit you covered is on:\n    {front.port}")
        print("\nPaste these into app/config.py:\n")
        print(f'FRONT_LIDAR_PORT = "{front.port}"')
        print(f'REAR_LIDAR_PORT = "{rear.port}"')
        return 0
    finally:
        for u in units:
            u.close()


def _wait_for_covered(units, timeout=20.0, near_m=0.12):
    """Return the index of the unit whose scan collapses to near-field
    returns (a cupped hand), or None on timeout."""
    baseline = [_mean_range(u.read_scan()) for u in units]
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.3)
        for i, u in enumerate(units):
            scan = u.read_scan()
            if not scan:
                continue
            near = sum(1 for p in scan if p.distance_m < near_m)
            if near > len(scan) * 0.5 and _mean_range(scan) \
                    < baseline[i] * 0.4:
                return i
    return None


def _resolve(name):
    if name == "front":
        port = config.FRONT_LIDAR_PORT
    elif name == "rear":
        port = config.REAR_LIDAR_PORT
    else:
        return name
    if not port:
        raise SystemExit(f"config.{name.upper()}_LIDAR_PORT is not set -- "
                         "run --identify first, or pass a device path.")
    return port


WEDGES = 16


def cmd_calibrate(name):
    """Single-object angle-sign check, in plain language.

    Place ONE object (a box, a chair, your leg) about 0.5-1 m from the unit
    and clearly off to ONE side of its forward axis -- ideally near 90 deg
    left or right. This reports which way the driver thinks that object is,
    and whether that matches the CCW convention the rest of the robot uses,
    so you know whether to flip LD19_NATIVE_CW. It averages several
    revolutions so a little hand-wobble does not matter.
    """
    port = _resolve(name)
    print(f"Calibrating {name} ({port}).")
    print("Put ONE object ~0.5-1 m away, off to the unit's LEFT or RIGHT of\n"
          "its forward axis (the direction the connector/arrow points).\n"
          "Averaging 20 revolutions; hold it still...\n")
    unit = LD19(port)

    # The chassis sits a few cm behind each unit and is ALWAYS the nearest
    # return, so a plain "nearest point" search just finds the robot's own
    # body (that is the ~170 deg / 0.05 m 'BEHIND' reading you saw). Exclude
    # returns closer than a hand's reach AND anything in the masked body arc,
    # so the object you actually placed is what gets measured.
    body_lo, body_hi = 120.0, 240.0     # skip this local arc (robot body)
    min_m, max_m = 0.15, 2.5            # a placed object, not the chassis/wall

    def _candidate(p):
        return (min_m <= p.distance_m <= max_m
                and not (body_lo <= p.angle_deg <= body_hi))

    try:
        unit.read_scan()
        import math
        xs, ys, dists = [], [], []
        empty = 0
        while len(dists) < 20:
            scan = unit.read_scan()
            cands = [p for p in scan if _candidate(p)]
            if not cands:
                empty += 1
                if empty > 60:
                    print("No object found in front/side arc. Place one "
                          f"{min_m:.2f}-{max_m:.1f} m away, off to a side,\n"
                          "and NOT directly behind the unit, then re-run.")
                    return 1
                time.sleep(0.05)
                continue
            near = min(cands, key=lambda p: p.distance_m)
            a = math.radians(near.angle_deg)
            xs.append(math.cos(a))
            ys.append(math.sin(a))
            dists.append(near.distance_m)
            time.sleep(0.05)
    finally:
        unit.close()

    import math
    mean_deg = math.degrees(math.atan2(sum(ys), sum(xs))) % 360.0
    mean_d = sum(dists) / len(dists)
    print(f"Nearest object read at ~{mean_deg:.0f} deg, {mean_d:.2f} m "
          "(driver's CCW local frame).\n")

    # Interpret against the convention: 0 = forward, +90 = left, +270/-90 = right.
    def side_of(deg):
        d = (deg + 180) % 360 - 180
        if -45 <= d <= 45:
            return "in FRONT of"
        if 45 < d <= 135:
            return "to the LEFT of"
        if -135 <= d < -45:
            return "to the RIGHT of"
        return "BEHIND"

    driver_side = side_of(mean_deg)
    print(f"By the driver's numbers the object is {driver_side} the unit.")
    print("Look at where it ACTUALLY is:")
    print(f"  - if that matches   -> convention is correct, "
          f"leave LD19_NATIVE_CW = {config.LD19_NATIVE_CW}")
    print("  - if left/right are SWAPPED -> flip LD19_NATIVE_CW in "
          f"app/config.py (currently {config.LD19_NATIVE_CW}) and re-run")
    print("  - if front/back are swapped -> the unit's forward axis is not "
          "what you think; check the mounting arrow")
    return 0


def cmd_watch(name):
    port = _resolve(name)
    print(f"Watching {name} ({port}); Ctrl-C to stop.")
    print("Wedge labels are LOCAL frame, CCW; 0 = unit's forward axis, "
          "+90 = LEFT, 270 = RIGHT.")
    unit = LD19(port)
    try:
        while True:
            scan = unit.read_scan()
            st = unit.stats()
            nearest = [float("inf")] * WEDGES
            for p in scan:
                nearest[int(p.angle_deg // (360 / WEDGES)) % WEDGES] = min(
                    nearest[int(p.angle_deg // (360 / WEDGES)) % WEDGES],
                    p.distance_m)
            close = min(scan, key=lambda p: p.distance_m)
            row = " ".join(
                f"{a * (360 // WEDGES):>4d}:" + (
                    " ---" if d == float("inf") else f"{d:4.1f}")
                for a, d in enumerate(nearest))
            print(f"\r{st['scan_hz']:4.1f} Hz  {st['points_last_rev']:3d} "
                  f"pts/rev  crc={st['crc_errors']}  "
                  f"nearest {close.distance_m:4.2f} m @ "
                  f"{close.angle_deg:5.1f}deg | {row[:110]}", end="")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print()
    finally:
        unit.close()
    return 0


def cmd_verify(seconds=5.0):
    """Automated health check on both configured units."""
    problems = []
    units = {}
    for name in ("front", "rear"):
        port = _resolve(name)
        try:
            units[name] = LD19(port)
        except Exception as e:
            print(f"[FAIL] {name}: cannot open {port}: {e}")
            return 1

    try:
        print(f"Collecting {seconds:.0f}s from both units...")
        for u in units.values():
            u.read_scan()
        time.sleep(seconds)

        for name, u in units.items():
            st = u.stats()
            scan = u.read_scan()
            ok = True

            hz = st["scan_hz"]
            if not 7.0 <= hz <= 13.0:
                ok = False
                problems.append(f"{name}: scan rate {hz:.1f} Hz (expect ~10;"
                                " check 5 V supply if it sags)")
            expect_revs = seconds * hz * 0.7
            if st["revolutions"] < expect_revs:
                ok = False
                problems.append(f"{name}: only {st['revolutions']} revs in "
                                f"{seconds:.0f}s -- dropped data?")
            total_pkts = max(st["revolutions"] * 38, 1)
            if st["crc_errors"] > total_pkts * 0.01:
                ok = False
                problems.append(f"{name}: {st['crc_errors']} CRC errors -- "
                                "noisy wiring / baud mismatch?")
            if st["points_last_rev"] < 250:
                ok = False
                problems.append(f"{name}: {st['points_last_rev']} pts/rev "
                                "is thin (expect ~450 with returns)")
            span = max(p.angle_deg for p in scan) - min(p.angle_deg
                                                        for p in scan)
            if span < 270:
                ok = False
                problems.append(f"{name}: angular span only {span:.0f} deg")

            print(f"[{'OK ' if ok else 'BAD'}] {name}: {hz:4.1f} Hz, "
                  f"{st['points_last_rev']} pts/rev, "
                  f"crc={st['crc_errors']}, resyncs={st['resyncs']}")

        # The two ports must be DIFFERENT physical units: with identical USB
        # serials, misconfig can open the same device twice via two names.
        a = units["front"].read_scan()
        b = units["rear"].read_scan()
        if a and b and abs(_mean_range(a) - _mean_range(b)) < 1e-9 \
                and len(a) == len(b):
            problems.append("front and rear returned identical scans -- "
                            "both port names may resolve to the SAME device")
    finally:
        for u in units.values():
            u.close()

    if problems:
        print("\nProblems:")
        for p in problems:
            print(f"  - {p}")
        return 1
    print("\nBoth LiDARs healthy.")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--list", action="store_true")
    g.add_argument("--identify", action="store_true")
    g.add_argument("--watch", metavar="FRONT|REAR|/dev/...")
    g.add_argument("--calibrate", metavar="FRONT|REAR|/dev/...",
                   help="single-object angle-sign check in plain language")
    g.add_argument("--verify", action="store_true")
    args = ap.parse_args()

    if args.list:
        sys.exit(cmd_list())
    if args.identify:
        sys.exit(cmd_identify())
    if args.watch:
        sys.exit(cmd_watch(args.watch.lower()))
    if args.calibrate:
        sys.exit(cmd_calibrate(args.calibrate.lower()))
    if args.verify:
        sys.exit(cmd_verify())


if __name__ == "__main__":
    main()
