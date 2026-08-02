"""Test the full sweep-and-track behaviour with the wheels disabled.

This is the safe rehearsal for the real run: the camera sweeps, finds the
enrolled pursuer, and the servo tracks them, but no motor command is ever
sent. Watch it in a browser like scripts/watch.py, with the search/track
state and bearings drawn on the frame.

    python3 -m scripts.track_test --camera 0
    # then open http://<board>.local:8080/ from your laptop

Add --lidar to also show the measured range at the pursuer's bearing, which
is what proximity (and therefore "caught") is computed from:

    python3 -m scripts.track_test --camera 0 --lidar \
        --front-port /dev/ttyUSB0 --rear-port /dev/ttyUSB1

What to check, in order:
  1. with nobody in view, the servo sweeps end to end, slowly and smoothly
  2. a STRANGER walking through does not stop the sweep
  3. you (enrolled) stop the sweep, and the box turns green with your name
  4. the servo follows you as you walk across and around the robot
  5. bearing reads ~0 when you are in front, and grows toward +/-90 as you
     move to a side -- if the sign is backwards, fix config.SERVO_DIR
     (see scripts/servo_test.py --verify)
  6. with --lidar, range should fall as you approach
"""

import argparse
import http.server
import socketserver
import threading
import time

import cv2

from app import config
from app.perception.camera import Camera, parse_source
from app.perception.detector import PersonDetector
from app.perception import identity, geometry
from app.control.scanner import CameraScanner, TRACKING
from app.control.bridge_client import BridgeClient, StubBridge

_lock = threading.Lock()
_latest = None

GREEN = (0, 200, 0)
RED = (0, 0, 220)
AMBER = (0, 200, 255)
WHITE = (255, 255, 255)


def _draw(frame, detections, chosen, name, scanner, bearing_deg,
          pursuer_range, fps):
    h, w = frame.shape[:2]
    cv2.line(frame, (w // 2, 0), (w // 2, h), (60, 60, 60), 1)

    for det in detections:
        is_chosen = chosen is not None and det.bbox == chosen.bbox
        colour = GREEN if is_chosen else RED
        x1, y1, x2, y2 = (int(v) for v in det.bbox)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
        label = (name or "pursuer") if is_chosen else "not enrolled"
        cv2.putText(frame, label, (x1, max(16, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)

    state_colour = GREEN if scanner.state == TRACKING else AMBER
    lines = [
        f"state   {scanner.state.upper()}",
        f"servo   {scanner.angle:6.1f} deg",
        f"bearing {('%+7.1f deg' % bearing_deg) if bearing_deg is not None else '   --'}",
    ]
    if pursuer_range is not None:
        rng = "  --" if pursuer_range == float("inf") else f"{pursuer_range:5.2f} m"
        prox = geometry.range_to_proximity(pursuer_range)
        lines.append(f"range   {rng}   prox {prox:.2f}")
    lines.append(f"fps     {fps:5.1f}")

    cv2.rectangle(frame, (0, 0), (250, 22 * len(lines) + 10), (25, 25, 25), -1)
    for i, text in enumerate(lines):
        cv2.putText(frame, text, (8, 20 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, state_colour if i == 0 else WHITE, 1, cv2.LINE_AA)
    return frame


def _loop(camera, detector, selector, gate, bridge, scanner,
          front_lidar, rear_lidar, quality):
    global _latest
    from app.sensing import lidar as lidar_mod
    frames, t0 = 0, time.time()
    resync = [20]      # frames until the next get_servo() model re-anchor

    while True:
        frame = camera.read()
        if frame is None:
            time.sleep(0.05)
            continue
        frames += 1

        detections = detector.detect(frame)
        if gate:
            chosen, name = selector.select(frame, detections)
        else:
            chosen = max(detections, key=lambda d: d.score) if detections else None
            name = "person" if chosen else None

        bbox = chosen.bbox if chosen is not None else None
        servo_deg, bearing_deg = scanner.update(bbox, name)
        bridge.set_servo(servo_deg)          # servo moves; wheels never do
        resync[0] -= 1
        if resync[0] <= 0:
            scanner.resync(bridge.get_servo())   # re-anchor the slew model
            resync[0] = 20

        pursuer_range = None
        if front_lidar is not None and bearing_deg is not None:
            points = lidar_mod.merged_points(front_lidar.read_scan(),
                                             rear_lidar.read_scan())
            pursuer_range = lidar_mod.range_at_bearing(points, bearing_deg)

        fps = frames / max(time.time() - t0, 1e-6)
        _draw(frame, detections, chosen, name, scanner, bearing_deg,
              pursuer_range, fps)

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            with _lock:
                _latest = buf.tobytes()


class _Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path != "/":
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _lock:
                    jpeg = _latest
                if jpeg is not None:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                time.sleep(0.03)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default=None)
    ap.add_argument("--model", default="models/person_detect.tflite")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--quality", type=int, default=60)
    ap.add_argument("--lidar", action="store_true",
                    help="also measure range at the pursuer's bearing")
    ap.add_argument("--front-port", default=config.FRONT_LIDAR_PORT,
                    help="serial port of the FRONT LD19; defaults to "
                         "config.FRONT_LIDAR_PORT (set it with "
                         "scripts.lidar_test --identify). Raw /dev/ttyUSBn "
                         "works but can swap units between boots.")
    ap.add_argument("--rear-port", default=config.REAR_LIDAR_PORT,
                    help="serial port of the REAR LD19; see --front-port")
    ap.add_argument("--stub", action="store_true",
                    help="no Bridge (servo commands printed, not sent)")
    args = ap.parse_args()

    camera = Camera(parse_source(args.camera))
    detector = PersonDetector(args.model)
    bridge = StubBridge() if args.stub else BridgeClient()
    scanner = CameraScanner()

    db = identity.IdentityDB()
    gate = config.IDENTITY_GATE and len(db) > 0
    selector = identity.PursuerSelector(db) if gate else None
    print(f"[track] identity gate {'ON: ' + str(db.names()) if gate else 'off'}")
    print("[track] WHEELS ARE NEVER COMMANDED in this script")

    front_lidar = rear_lidar = None
    if args.lidar:
        from app.sensing.ld19_driver import LD19
        if not args.front_port or not args.rear_port:
            raise SystemExit(
                "LiDAR ports not set. Run  python3 -m scripts.lidar_test "
                "--identify  and paste the two lines it prints into "
                "app/config.py (or pass --front-port/--rear-port).")
        front_lidar = LD19(args.front_port)
        rear_lidar = LD19(args.rear_port)
        print("[track] LiDAR range enabled")

    threading.Thread(
        target=_loop,
        args=(camera, detector, selector, gate, bridge, scanner,
              front_lidar, rear_lidar, args.quality),
        daemon=True).start()

    print(f"[track] open http://<this-board>.local:{args.port}/ -- Ctrl-C to stop")
    try:
        with socketserver.ThreadingTCPServer(("", args.port), _Handler) as srv:
            srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.set_servo(config.SERVO_CENTER_DEG)
        camera.release()
        if front_lidar:
            front_lidar.close()
        if rear_lidar:
            rear_lidar.close()


if __name__ == "__main__":
    main()
