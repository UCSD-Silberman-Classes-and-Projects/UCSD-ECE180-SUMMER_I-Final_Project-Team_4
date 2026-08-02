"""Bring up the camera pan servo, and verify which way it turns.

Run this BEFORE any driving test. Getting config.SERVO_DIR backwards means
every bearing is mirrored, which makes the robot flee TOWARD the pursuer --
the single most dangerous sign error in the project, and invisible until the
robot is moving.

    python3 -m scripts.servo_test --centre       # park at SERVO_CENTER_DEG
    python3 -m scripts.servo_test --angle 130    # go to one angle and hold
    python3 -m scripts.servo_test --sweep        # ping-pong the full range
    python3 -m scripts.servo_test --verify       # guided SERVO_DIR check

The firmware slew-limits the motion, so the servo eases toward each target
rather than snapping; that is expected, not lag in this script.
"""

import argparse
import time

from app import config
from app.control.bridge_client import BridgeClient, StubBridge


def _bridge(stub):
    return StubBridge() if stub else BridgeClient()


def do_centre(bridge):
    print(f"centring at {config.SERVO_CENTER_DEG} deg "
          f"(camera should look straight down the chassis)")
    bridge.set_servo(config.SERVO_CENTER_DEG)
    time.sleep(1.5)


def do_angle(bridge, angle):
    angle = max(config.SERVO_MIN_DEG, min(config.SERVO_MAX_DEG, angle))
    print(f"moving to {angle} deg")
    bridge.set_servo(angle)
    time.sleep(1.5)


def do_sweep(bridge, cycles, step, dwell):
    print(f"sweeping {config.SERVO_MIN_DEG}..{config.SERVO_MAX_DEG} deg, "
          f"{cycles} cycle(s). Ctrl-C to stop.")
    lo, hi = config.SERVO_MIN_DEG, config.SERVO_MAX_DEG
    try:
        for _ in range(cycles):
            for angle in list(range(lo, hi + 1, step)) + list(range(hi, lo - 1, -step)):
                bridge.set_servo(angle)
                time.sleep(dwell)
    except KeyboardInterrupt:
        pass
    do_centre(bridge)


def do_verify(bridge):
    """Guided check that config.SERVO_DIR matches the physical linkage."""
    print("\n=== SERVO_DIR verification ===")
    print(f"config.SERVO_DIR is currently {config.SERVO_DIR:+d}\n")
    do_centre(bridge)
    input("Camera should now be pointing STRAIGHT AHEAD. Press Enter...")

    high = min(config.SERVO_CENTER_DEG + 45, config.SERVO_MAX_DEG)
    print(f"\nmoving to {high} deg (an INCREASE from centre)...")
    bridge.set_servo(high)
    time.sleep(2.0)

    print("\nStanding behind the robot, looking the way it drives:")
    print("  [l] the camera turned to the robot's LEFT")
    print("  [r] the camera turned to the robot's RIGHT")
    answer = input("which way did it turn? [l/r] ").strip().lower()
    do_centre(bridge)

    if answer.startswith("l"):
        expected = +1
        print("\nIncreasing angle -> camera looks LEFT -> SERVO_DIR should be +1")
    elif answer.startswith("r"):
        expected = -1
        print("\nIncreasing angle -> camera looks RIGHT -> SERVO_DIR should be -1")
    else:
        print("unrecognised answer; nothing checked")
        return

    if expected == config.SERVO_DIR:
        print(f"config.SERVO_DIR = {config.SERVO_DIR:+d} is CORRECT. Nothing to change.")
    else:
        print(f"MISMATCH: config.SERVO_DIR is {config.SERVO_DIR:+d} but should be "
              f"{expected:+d}.\nEdit app/config.py and set SERVO_DIR = {expected:+d} "
              f"before running any driving test.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--centre", action="store_true", help="park at centre")
    ap.add_argument("--angle", type=int, help="go to a specific angle (deg)")
    ap.add_argument("--sweep", action="store_true", help="ping-pong the range")
    ap.add_argument("--verify", action="store_true",
                    help="guided check that SERVO_DIR matches the hardware")
    ap.add_argument("--cycles", type=int, default=2)
    ap.add_argument("--step", type=int, default=5)
    ap.add_argument("--dwell", type=float, default=0.05)
    ap.add_argument("--stub", action="store_true", help="no hardware, print only")
    args = ap.parse_args()

    bridge = _bridge(args.stub)
    try:
        if args.verify:
            do_verify(bridge)
        elif args.sweep:
            do_sweep(bridge, args.cycles, args.step, args.dwell)
        elif args.angle is not None:
            do_angle(bridge, args.angle)
        else:
            do_centre(bridge)
    finally:
        if not args.stub:
            bridge.set_servo(config.SERVO_CENTER_DEG)


if __name__ == "__main__":
    main()
