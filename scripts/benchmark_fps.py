"""Week-1 gate: measure person-detection FPS on the MPU.

The whole project hinges on this clearing ~10 FPS. Run it on the actual board,
and re-run with the LiDAR driver(s) active since they share the processor.

    python -m scripts.benchmark_fps --model models/person_detect.tflite --frames 200
"""

import argparse
import time

from app.perception.camera import Camera, parse_source
from app.perception.detector import PersonDetector


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="models/person_detect.tflite")
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--camera", default=None,
                    help="V4L2 index (e.g. 0) or stream URL; "
                         "defaults to config.CAMERA_SOURCE")
    args = ap.parse_args()

    camera = Camera(parse_source(args.camera))
    detector = PersonDetector(args.model)

    # warm up (first inference allocates / JITs)
    for _ in range(5):
        f = camera.read()
        if f is not None:
            detector.detect(f)

    n, t0 = 0, time.time()
    while n < args.frames:
        f = camera.read()
        if f is None:
            continue
        detector.detect(f)
        n += 1
    elapsed = time.time() - t0

    camera.release()
    print(f"{n} frames in {elapsed:.2f}s -> {n / elapsed:.2f} FPS")
    print("GATE:", "PASS" if n / elapsed >= 10 else "FAIL (below 10 FPS)")


if __name__ == "__main__":
    main()
