"""Top-level evasion loop (runs on the UNO Q MPU).

    capture -> detect -> identity gate -> pan servo (search or track)
            -> LiDAR range at the pursuer's bearing
            -> evasion policy -> PWM -> bridge -> log -> repeat

Division of labour, now that the camera is on a pan servo and the ultrasonic
is gone:

    camera  says WHICH DIRECTION the pursuer is (bearing), over a much wider
            arc than a fixed camera could cover, because the servo sweeps
    LiDAR   says HOW FAR they are, by looking up the nearest return at that
            bearing, and separately keeps the robot out of corners
    STM32    holds the servo angle and fails safe if this loop dies

Run with --stub to exercise the whole loop with no hardware.
"""

import argparse
import math
import time

from . import config
from .perception.camera import Camera
from .perception.detector import PersonDetector
from .perception import geometry
from .perception import identity
from .sensing import lidar
from .sensing.ld19_driver import LD19
from .control import evasion
from .control.scanner import CameraScanner
from .control.bridge_client import BridgeClient, StubBridge
from .utils.logging import RunLogger


def build_bridge(stub):
    return StubBridge() if stub else BridgeClient()


def run(model_path, front_port, rear_port, log_path, stub=False,
        no_drive=False):
    camera = Camera()
    detector = PersonDetector(model_path)
    bridge = build_bridge(stub)
    logger = RunLogger(log_path)
    scanner = CameraScanner()

    # Identity gate: on when anyone is enrolled (see scripts/enroll.py).
    # Every detected person is scored each frame and the best-matching
    # enrolled one is chosen, so a stranger cannot hold the tracker hostage
    # and cannot end the search sweep.
    id_db = identity.IdentityDB()
    gate = config.IDENTITY_GATE and len(id_db) > 0
    selector = identity.PursuerSelector(id_db) if gate else None
    lost_frames = 0
    resync_countdown = 20
    if gate:
        print(f"[identity] gate ON -- fleeing only: {id_db.names()}")
    else:
        print("[identity] gate off -- fleeing any person")
    if no_drive:
        print("[safety] --no-drive: servo will track, wheels stay stopped")

    front_lidar = rear_lidar = None
    if not stub:
        if not front_port or not rear_port:
            raise SystemExit(
                "LiDAR ports not set. Run  python3 -m scripts.lidar_test "
                "--identify  and paste the two lines it prints into "
                "app/config.py (or pass --front-port/--rear-port).")
        front_lidar = LD19(front_port)
        rear_lidar = LD19(rear_port)

    tracker = evasion.PursuerTracker()
    policy = evasion.EscapePolicy()
    investigator = evasion.SearchInvestigator()
    caught_detector = evasion.CaughtDetector()
    last_pwm = (0, 0)
    last_t = time.monotonic()

    try:
        while True:
            t0 = time.time()

            # --- perception ---
            frame = camera.read()
            if frame is None:
                continue

            pursuer = None
            if gate:
                detections = detector.detect(frame)
                person, pursuer = selector.select(frame, detections)
                if person is None:
                    lost_frames = 0 if detections else lost_frames + 1
                    if lost_frames >= config.ID_LOST_FRAMES_RESET:
                        selector.reset()
                else:
                    lost_frames = 0
            else:
                person = detector.best(frame)
                pursuer = "person" if person is not None else None

            # --- pan servo: sweep until the pursuer is found, then track ---
            bbox = person.bbox if person is not None else None
            servo_deg, bearing_deg = scanner.update(bbox, pursuer)
            # Re-anchor the scanner's servo-position model occasionally so
            # small MCU loop-timing differences cannot accumulate. One
            # blocking read-back every ~20 frames (~2 s) is noise in the
            # budget; every frame would not be.
            resync_countdown -= 1
            if resync_countdown <= 0 and not stub:
                scanner.resync(bridge.get_servo())
                resync_countdown = 20

            # --- LiDAR: sectors for corner avoidance, range for proximity ---
            if stub:
                points = []
                sectors = {name: math.inf for name in config.SECTORS}
            else:
                front_scan = front_lidar.read_scan()
                rear_scan = rear_lidar.read_scan()
                points = lidar.merged_points(front_scan, rear_scan)
                sectors = lidar.sectorize(points)

            # --- camera -> LiDAR pursuit hand-off ---
            # FRAME NOTE (this fixes a live sign bug): the camera chain
            # (camera_to_robot_bearing) reports bearings with positive =
            # robot's RIGHT, but LiDAR robot_points use the atan2 convention,
            # positive = CCW/LEFT. Everything below the boundary -- the
            # tracker, track/range lookups, the escape policy -- works in
            # the LiDAR (CCW) frame; the camera bearing is negated once,
            # here, on entry. Feeding the right-positive bearing straight
            # into range_at_bearing (as this loop previously did) looked up
            # the pursuer's range on the MIRRORED side of the robot.
            now = time.monotonic()
            dt = max(0.0, now - last_t)
            last_t = now
            # Dead-reckon chassis yaw from the PWM commanded last tick: the
            # turn-away out-rotates the servo, so the camera WILL go blind
            # mid-turn, and the chassis-frame bearing must rotate with us.
            tracker.rotate((last_pwm[1] - last_pwm[0])
                           * config.YAW_DPS_PER_PWM_DIFF * dt)

            source = None
            bearing_ccw = None
            pursuer_range = math.inf
            if bearing_deg is not None:
                bearing_ccw = -bearing_deg          # camera(+right) -> CCW
                # Range for the camera's bearing. When the tracker already
                # has a range estimate, reuse the range-gated angle-closest
                # match (a chair or wall edge inside the +/-8 deg window
                # can sit NEARER than the person, and a raw min() would
                # report its distance instead). First fix: no hint -> min().
                prev = tracker.get(now)
                hinted = None
                if prev is not None and prev[1] is not None:
                    hinted = lidar.track_at_bearing(
                        points, bearing_ccw,
                        tol_deg=config.BEARING_WINDOW_DEG,
                        range_hint_m=prev[1])
                pursuer_range = (hinted[1] if hinted is not None else
                                 lidar.range_at_bearing(points, bearing_ccw))
                tracker.update(bearing_ccw, pursuer_range, now)
                source = "camera"
            else:
                predicted = tracker.get(now)
                if predicted is not None:
                    pb, pr = predicted
                    fix = lidar.track_at_bearing(points, pb, range_hint_m=pr)
                    if fix is not None:
                        bearing_ccw, pursuer_range = fix
                        tracker.update(bearing_ccw, pursuer_range, now)
                        source = "lidar"
                    else:
                        bearing_ccw = pb
                        pursuer_range = pr if pr is not None else math.inf
                        source = "memory"
                    # Aim the camera at the prediction (clamped into the pan
                    # range) so it re-acquires the moment the person swings
                    # back into reachable bearings. Convert CCW -> the servo
                    # frame (right-positive) on the way out.
                    servo_deg = scanner.override_target(
                        config.SERVO_CENTER_DEG
                        + config.SERVO_DIR * (-bearing_ccw))

            # --- search-time investigation: the LiDAR tells the camera
            # where to look while sweeping. Identity gate untouched --
            # nothing flees until the camera confirms who it is; a chair
            # gets one look, an approaching person re-triggers until seen.
            investigate = None
            if bearing_ccw is None and source is None:
                investigate = investigator.step(points, now)
                if investigate is not None and investigate[0] == "aim":
                    servo_deg = scanner.override_target(
                        config.SERVO_CENTER_DEG
                        + config.SERVO_DIR * (-investigate[1]))
            else:
                investigator.reset()
            bridge.set_servo(servo_deg)

            proximity = geometry.range_to_proximity(pursuer_range)
            if proximity == 0.0 and person is not None and bbox is not None:
                # LiDAR silent on that bearing: fall back to box height.
                proximity = geometry.bbox_to_proximity(bbox)

            # --- decide & act ---
            heading, speed = policy.step(bearing_ccw, proximity, sectors,
                                        pursuer_range=pursuer_range)
            if investigate is not None and investigate[0] == "spin":
                # Rotate in place to bring a blind-arc blob into camera
                # reach; forward speed stays zero until identity confirms.
                heading = (-config.CURIOSITY_SPIN_HEADING
                           if investigate[1] > 0
                           else config.CURIOSITY_SPIN_HEADING)
                speed = 0.0
            steer_signal = heading            # for the log line below
            left_pwm, right_pwm = evasion.heading_to_pwm(heading, speed)
            last_pwm = (left_pwm, right_pwm)
            if no_drive:
                left_pwm = right_pwm = 0
            bridge.set_motion(left_pwm, right_pwm)

            # --- eval / logging ---
            # Instantaneous condition, counted only from MEASURED fixes:
            # memory-source ranges are dead-reckoned extrapolations of a
            # stale reading and must not end the run. The detector then
            # requires the condition to hold CAUGHT_HOLD_S continuously --
            # a single-revolution range glitch (chair edge or wall corner
            # straying into the bearing window) resets and never ends it.
            caught_now = (source in ("camera", "lidar")
                          and (proximity >= config.CAUGHT_PROXIMITY
                               or pursuer_range <= config.CAUGHT_RANGE_M))
            caught = caught_detector.update(caught_now, now)
            held = caught_detector.held_for(now)
            if caught_now and not caught:
                print(f"[caught?] range={pursuer_range:.2f} m via={source} "
                      f"holding {held:.1f}/{config.CAUGHT_HOLD_S:.1f} s",
                      end="\r")
            fps = 1.0 / max(time.time() - t0, 1e-6)
            logger.log(fps, steer_signal, proximity, caught_now,
                       sectors, left_pwm, right_pwm, identity_name=pursuer)
            if caught:
                print(f"\n[caught] range={pursuer_range:.2f} m "
                      f"proximity={proximity:.2f} via={source} "
                      f"sustained {held:.1f} s -- run ends")
                break

    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
        camera.release()
        logger.close()
        if front_lidar:
            front_lidar.close()
        if rear_lidar:
            rear_lidar.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="models/person_detect.tflite")
    ap.add_argument("--front-port", default=config.FRONT_LIDAR_PORT,
                    help="serial port of the FRONT LD19; defaults to "
                         "config.FRONT_LIDAR_PORT (set it with "
                         "scripts.lidar_test --identify). Raw /dev/ttyUSBn "
                         "works but can swap units between boots.")
    ap.add_argument("--rear-port", default=config.REAR_LIDAR_PORT,
                    help="serial port of the REAR LD19; see --front-port")
    ap.add_argument("--log", default="run.csv")
    ap.add_argument("--stub", action="store_true",
                    help="run with no hardware (fake bridge, no LiDAR)")
    ap.add_argument("--no-drive", action="store_true",
                    help="track with the servo but keep the wheels stopped "
                         "(safe first test on the real robot)")
    args = ap.parse_args()
    run(args.model, args.front_port, args.rear_port, args.log,
        stub=args.stub, no_drive=args.no_drive)


if __name__ == "__main__":
    main()
