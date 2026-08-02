"""Measure the physical signs no software test can see -- ON THE ROBOT.

Symptom this exists for: the robot recognises the pursuer, then does DONUTS
instead of driving away (camera still locked on), and "works decent" only
while the wheels are stopped.

There are TWO independent hardware faults that both cause donuts, and this
script separates them -- in the right order, because the second measurement
is meaningless until the first is fixed:

  PHASE 1  Per-wheel polarity (MOTOR_LEFT_SIGN / MOTOR_RIGHT_SIGN).
           If ONE motor is wired backwards, a spin command translates and a
           straight command spins. Tell-tale: when asked to spin in place,
           the robot drives in a straight line instead. Drives each wheel
           ALONE so you can read its true direction.

  PHASE 2  Turn convention + rate (MOTOR_TURN_SIGN, YAW_DPS_PER_PWM_DIFF).
           Once both wheels drive the right way, spin in place and time it.

  PHASE 3  Servo direction (SERVO_DIR). A mirrored horn mirrors every
           bearing -- same donut symptom from the camera side.

Corrections found in phase 1 are applied for the rest of THIS run, so the
later phases work even before you have edited the file.

USAGE -- robot on the floor, ~1 m clearance (it can spin AND briefly
translate); watch it from ABOVE:

    python3 -m scripts.sign_check            # full sequence
    python3 -m scripts.sign_check --wheels   # phase 1 only
    python3 -m scripts.sign_check --spin      # phase 2 only
    python3 -m scripts.sign_check --servo     # phase 3 only
    python3 -m scripts.sign_check --pwm 100   # gentler drive
"""

import argparse
import time

from app import config
from app.control.bridge_client import BridgeClient


SPIN_TURNS = 2


def _pulse(bridge, left, right, secs=1.0):
    bridge.set_motion(left, right)
    time.sleep(secs)
    bridge.stop()


def _ask(prompt, choices):
    ans = ""
    while ans not in choices:
        ans = input(prompt).lower().strip()
    return ans


def wheel_polarity(bridge, pwm):
    """Drive each wheel alone; detect a channel swap AND each wheel's
    polarity. Returns (swap, left_sign, right_sign)."""
    print("=" * 68)
    print("PHASE 1 -- channel mapping + per-wheel polarity")
    print("=" * 68)
    print("\nDriving ONE channel at a time. Watch which wheel ACTUALLY moves")
    print("and which way. 'Forward' = top of the wheel moves toward the nose.")
    print("If the wheel that moves is the OTHER side from the one commanded,")
    print("your motor channels are crossed -- this catches that.\n")

    observed = {}   # commanded side -> (which wheel moved, direction)
    for name, cmd in (("LEFT", (pwm, 0)), ("RIGHT", (0, pwm))):
        input(f"Press Enter to drive the {name} channel...")
        _pulse(bridge, *cmd, secs=1.2)
        which = _ask("Which wheel MOVED -- [l]eft, [r]ight, or [n]either? ",
                     ("l", "r", "n"))
        if which == "n":
            print(f"\n*** Nothing moved for the {name} channel: dead motor,")
            print("    loose connector, or unpowered driver. Fix that first.\n")
            observed[name] = (name[0].lower(), +1)   # assume in-place, fwd
            continue
        direction = _ask("Did it go [f]orward or [b]ackward? ", ("f", "b"))
        observed[name] = (which, +1 if direction == "f" else -1)

    # Swap if the LEFT command moved the right wheel (and vice versa).
    left_moved = observed["LEFT"][0]
    right_moved = observed["RIGHT"][0]
    swap = (left_moved == "r" and right_moved == "l")

    # After un-swapping, attribute each polarity to its true wheel.
    left_sign = observed["RIGHT"][1] if swap else observed["LEFT"][1]
    right_sign = observed["LEFT"][1] if swap else observed["RIGHT"][1]

    print()
    fixes = []
    if swap:
        print("Channel SWAP detected -- the commands are crossed.")
        fixes.append("    MOTOR_SWAP_LR = True")
    if left_sign != 1:
        fixes.append(f"    MOTOR_LEFT_SIGN = {left_sign:+d}")
    if right_sign != 1:
        fixes.append(f"    MOTOR_RIGHT_SIGN = {right_sign:+d}")
    if fixes:
        print("Paste into app/config.py:")
        for line in fixes:
            print(line)
        print("(Applying for the rest of this run so the next phases work.)")
    else:
        print("Channels correct and both wheels drive forward. Nothing to fix.")

    config.MOTOR_SWAP_LR = swap
    config.MOTOR_LEFT_SIGN = left_sign
    config.MOTOR_RIGHT_SIGN = right_sign
    print()
    return swap, left_sign, right_sign


def spin_check(bridge, pwm):
    print("=" * 68)
    print("PHASE 2 -- turn direction + rate")
    print("=" * 68)
    print("\nNow both wheels are corrected, so this should be a real in-place")
    print(f"spin. Commanding a differential of {2*pwm} PWM.\n")
    input("Clear ~1 m around the robot, then press Enter to spin...")
    _pulse(bridge, pwm, -pwm, secs=1.5)
    ans = _ask("Did it spin [c]lockwise or [a]nticlockwise (from above)? ",
               ("c", "a"))
    turn_sign = +1 if ans == "c" else -1
    print("\nWiring matches the code convention."
          if turn_sign == +1 else
          "\nMirrored turn convention -- set MOTOR_TURN_SIGN = -1.")
    print()

    print(f"Timing {SPIN_TURNS} full revolutions. Press Enter to START, then")
    print(f"Enter again the instant it completes {SPIN_TURNS} turns.")
    input("Enter to start...")
    bridge.set_motion(pwm, -pwm)
    t0 = time.monotonic()
    input(f"... Enter after {SPIN_TURNS} revolutions ...")
    elapsed = time.monotonic() - t0
    bridge.stop()
    rate = 360.0 * SPIN_TURNS / max(elapsed, 0.1)
    yaw_const = turn_sign * rate / (2 * pwm)
    print(f"\nMeasured {rate:.0f} deg/s at |R-L| = {2*pwm} PWM.\n")
    print("Paste into app/config.py:")
    print(f"    MOTOR_TURN_SIGN = {turn_sign:+d}")
    print(f"    YAW_DPS_PER_PWM_DIFF = {yaw_const:.2f}")
    print()
    return turn_sign, yaw_const


def servo_check(bridge):
    print("=" * 68)
    print("PHASE 3 -- servo direction (SERVO_DIR)")
    print("=" * 68)
    print("\nStand BEHIND the robot, looking where its nose points.")
    input("Press Enter to centre the camera...")
    bridge.set_servo(config.SERVO_CENTER_DEG)
    time.sleep(1.5)
    target = config.SERVO_CENTER_DEG + config.SERVO_DIR * 40
    print(f"\nCommanding {target:.0f}. With SERVO_DIR = {config.SERVO_DIR:+d}"
          " correct, the camera pans to the robot's RIGHT.\n")
    bridge.set_servo(target)
    time.sleep(1.5)
    ans = _ask("Which way did the camera pan? [l]eft / [r]ight: ", ("l", "r"))
    bridge.set_servo(config.SERVO_CENTER_DEG)
    print("\nCorrect. Keep SERVO_DIR = %+d." % config.SERVO_DIR
          if ans == "r" else
          "\nMirrored servo -- paste:  SERVO_DIR = %+d" % (-config.SERVO_DIR))
    print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wheels", action="store_true", help="phase 1 only")
    ap.add_argument("--spin", action="store_true", help="phase 2 only")
    ap.add_argument("--servo", action="store_true", help="phase 3 only")
    ap.add_argument("--pwm", type=int, default=120)
    args = ap.parse_args()
    all_phases = not (args.wheels or args.spin or args.servo)

    bridge = BridgeClient()
    try:
        if args.wheels or all_phases:
            wheel_polarity(bridge, args.pwm)
        if args.spin or all_phases:
            spin_check(bridge, args.pwm)
        if args.servo or all_phases:
            servo_check(bridge)
        print("Done. Edit app/config.py with the values above, then:")
        print("    python3 -m app.main")
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
