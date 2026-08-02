"""Verify a camera source before running anything else on it.

The first thing to run on the board after plugging in a webcam or pointing at
an ESP32-CAM stream. Confirms the source opens, reports the resolution it
actually delivers (not the one we asked for), measures capture-only frame rate,
and optionally saves a frame so you can confirm it is pointing where you think.

    # USB webcam on the UNO Q (find the index with: v4l2-ctl --list-devices)
    python3 -m scripts.test_camera --camera 0 --save shot.jpg

    # ESP32-CAM MJPEG stream
    python3 -m scripts.test_camera --camera http://192.168.4.1:81/stream --save shot.jpg

Capture FPS measured here is the CEILING for the whole pipeline: detection can
only ever be slower. If this number is already under the 10 FPS gate, fix the
camera before touching the model.
"""

import argparse
import time

import cv2

from app.perception.camera import Camera, parse_source


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default=None,
                    help="V4L2 index (e.g. 0) or stream URL; "
                         "defaults to config.CAMERA_SOURCE")
    ap.add_argument("--frames", type=int, default=60,
                    help="frames to time (default 60)")
    ap.add_argument("--save", metavar="PATH", help="write one frame here")
    args = ap.parse_args()

    source = parse_source(args.camera)
    print(f"opening {source!r} ...")
    try:
        cam = Camera(source)
    except RuntimeError as exc:
        raise SystemExit(f"FAILED: {exc}")

    kind = "network stream" if cam.is_stream else "local V4L2 device"
    print(f"opened OK  ({kind})")

    frame = None
    for _ in range(10):                      # let the stream settle
        frame = cam.read()
        if frame is not None:
            break
    if frame is None:
        cam.release()
        raise SystemExit("FAILED: opened but delivered no frames. "
                         "For a stream, check the URL in a browser; for USB, "
                         "check that nothing else is holding the device.")

    h, w = frame.shape[:2]
    print(f"frame size: {w}x{h}  (driver reports {cam.actual_size()})")

    dropped = 0
    t0 = time.time()
    for _ in range(args.frames):
        if cam.read() is None:
            dropped += 1
    elapsed = time.time() - t0
    fps = args.frames / elapsed if elapsed > 0 else 0.0

    print(f"capture rate: {fps:.1f} FPS over {args.frames} frames"
          f"{f' ({dropped} dropped)' if dropped else ''}")
    if fps < 10:
        print("  WARNING: below the 10 FPS gate before detection even runs.")
        if cam.is_stream:
            print("  For a stream: lower the sender's resolution/quality, or "
                  "move closer to the access point.")
        else:
            print("  For USB: try a lower FRAME_WIDTH/HEIGHT, or check whether "
                  "the camera is falling back to a slow MJPEG/YUYV mode.")

    if args.save:
        cv2.imwrite(args.save, frame)
        print(f"wrote {args.save}")

    cam.release()
    print("done")


if __name__ == "__main__":
    main()
