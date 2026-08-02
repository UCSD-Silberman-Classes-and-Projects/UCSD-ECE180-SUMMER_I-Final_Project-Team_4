"""Stream the UNO Q's camera view live to a browser over WiFi.

No new dependencies (uses Python's built-in http.server) and much smoother
than X11 forwarding for actually watching video. Two modes:

    # raw camera feed, whatever source you point it at
    python3 -m scripts.watch --camera 0

    # annotated feed: boxes, identity gate, [LOCKED] tag -- the same overlay
    # vision_preview.py draws, but live instead of saved to disk
    python3 -m scripts.watch --camera 0 --annotate

Then, from your Mac, open in any browser:

    http://<board-ip-or-hostname>.local:8080/

Ctrl-C on the board to stop. Bandwidth-friendly by design: JPEG quality and
target FPS are both capped (see --quality / --fps) since this runs alongside
detection and you do not want the stream competing for the same CPU/network
that the evasion loop needs.
"""

import argparse
import http.server
import socketserver
import threading
import time

import cv2

from app.perception.camera import Camera, parse_source

_lock = threading.Lock()
_latest_jpeg = None


def _capture_loop(camera, annotate, detector, gate_selector, gate_db,
                  target_fps, jpeg_quality):
    global _latest_jpeg
    period = 1.0 / target_fps
    while True:
        t0 = time.time()
        frame = camera.read()
        if frame is None:
            time.sleep(0.05)
            continue

        if annotate:
            from scripts.vision_preview import annotate as draw_overlay
            detections = detector.detect(frame)
            if gate_selector is not None:
                chosen, name = gate_selector.select(frame, detections)
                locked = gate_selector.locked_identity()
            else:
                chosen = max(detections, key=lambda d: d.score) if detections else None
                name, locked = None, None
            draw_overlay(frame, detections, chosen, name, target_fps,
                        gate_selector is not None, locked)

        ok, buf = cv2.imencode(".jpg", frame,
                               [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        if ok:
            with _lock:
                _latest_jpeg = buf.tobytes()

        elapsed = time.time() - t0
        if elapsed < period:
            time.sleep(period - elapsed)


class _Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass  # quiet; the console is busy enough

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
                    jpeg = _latest_jpeg
                if jpeg is not None:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                time.sleep(0.03)
        except (BrokenPipeError, ConnectionResetError):
            pass   # browser tab closed -- expected, not an error


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default=None,
                    help="V4L2 index (e.g. 0) or stream URL; "
                         "defaults to config.CAMERA_SOURCE")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--fps", type=float, default=10.0,
                    help="stream frame rate cap (default 10, kept modest so "
                         "it doesn't fight the detector for CPU/bandwidth)")
    ap.add_argument("--quality", type=int, default=60,
                    help="JPEG quality 1-100 (lower = less bandwidth)")
    ap.add_argument("--annotate", action="store_true",
                    help="draw detection boxes / identity gate, like "
                         "vision_preview.py, instead of the raw feed")
    ap.add_argument("--model", default="models/person_detect.tflite")
    args = ap.parse_args()

    camera = Camera(parse_source(args.camera))
    print(f"[watch] capturing from {camera.source!r} at {camera.actual_size()}")

    detector = gate_selector = gate_db = None
    if args.annotate:
        from app.perception.detector import PersonDetector
        from app.perception import identity
        from app import config
        detector = PersonDetector(args.model)
        gate_db = identity.IdentityDB()
        if config.IDENTITY_GATE and len(gate_db) > 0:
            gate_selector = identity.PursuerSelector(gate_db)
            print(f"[watch] identity gate ON: {gate_db.names()}")
        else:
            print("[watch] identity gate off (nobody enrolled, or --no-gate)")

    thread = threading.Thread(
        target=_capture_loop,
        args=(camera, args.annotate, detector, gate_selector, gate_db,
              args.fps, args.quality),
        daemon=True)
    thread.start()

    print(f"[watch] open http://<this-board>.local:{args.port}/ in a browser")
    print("[watch] Ctrl-C to stop")
    try:
        with socketserver.ThreadingTCPServer(("", args.port), _Handler) as srv:
            srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        camera.release()


if __name__ == "__main__":
    main()
