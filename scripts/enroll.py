"""Enroll a team member so the identity gate flees only them.

Run once per pursuer, in the demo room, wearing demo-day clothes (the gate
identifies clothing). Wear clearly different shirt colors from each other.
Stand 2-3 m from the camera with your full body in frame and slowly turn and
move around while it collects torso appearance samples.

    python -m scripts.enroll --name charlie
    python -m scripts.enroll --name jaafar --model models/person_detect.tflite

Signatures land in data/identities/<name>.npz. Re-enroll after changing
clothes. Delete a person's .npz to un-enroll them; the gate switches off
automatically when nobody is enrolled.
"""

import argparse
import time

from app import config
from app.perception.camera import Camera, parse_source
from app.perception.detector import PersonDetector
from app.perception import identity


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True, help="pursuer name, e.g. charlie")
    ap.add_argument("--model", default="models/person_detect.tflite")
    ap.add_argument("--camera", default=None,
                    help="V4L2 index (e.g. 0) or stream URL; "
                         "defaults to config.CAMERA_SOURCE")
    ap.add_argument("--samples", type=int, default=120)
    ap.add_argument("--gap", type=float, default=0.12,
                    help="seconds between accepted samples")
    args = ap.parse_args()

    name = args.name.strip().lower()
    camera = Camera(parse_source(args.camera))
    detector = PersonDetector(args.model)
    db = identity.IdentityDB()

    samples, last_t = [], 0.0
    total = args.samples
    print(f"[enroll] collecting {total} torso samples for '{name}'.")
    print("[enroll] VARY yourself as it captures -- the wider the variety, the")
    print("[enroll] more sure the robot will be later. Follow the prompts:")
    # Quarters of the capture each ask for a different condition, so the set
    # spans the pose/distance/lighting the gate will actually see.
    prompts = [
        "  >> stand at NORMAL distance, face the camera",
        "  >> turn slowly LEFT and RIGHT (side profiles)",
        "  >> step BACK a few paces (farther away)",
        "  >> move around the space (different lighting angles)",
    ]
    shown = -1
    try:
        while len(samples) < total:
            quarter = min(3, len(samples) * 4 // total)
            if quarter != shown:
                shown = quarter
                print(prompts[quarter])
            frame = camera.read()
            if frame is None:
                continue
            person = detector.best(frame)
            if person is None or time.time() - last_t < args.gap:
                continue
            sig = identity.torso_signature(frame, person.bbox)
            if sig is None:
                continue    # too far / too small -- step closer
            samples.append(sig)
            last_t = time.time()
            if len(samples) % 20 == 0:
                print(f"[enroll] {len(samples)}/{total}")
    except KeyboardInterrupt:
        print("[enroll] aborted; nothing saved")
        return
    finally:
        camera.release()

    path = db.save(name, samples)
    print(f"[enroll] saved {len(samples)} signatures -> {path}")
    print(f"[enroll] enrolled pursuers: {db.names()}")


if __name__ == "__main__":
    main()
