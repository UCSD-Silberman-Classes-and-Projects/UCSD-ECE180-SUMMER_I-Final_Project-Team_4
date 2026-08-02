"""See what the perception stack sees: boxes, bearing, proximity, identity.

The one tool for eyeballing the vision pipeline. Draws every person detection,
marks which one the identity gate selected, and overlays the exact numbers the
evasion policy will receive -- so a wrong bearing sign or a mistuned proximity
anchor is visible immediately instead of showing up later as strange driving.

    # live camera
    python -m scripts.vision_preview

    # offline, on frames saved by collect_frames.py (no camera needed)
    python -m scripts.vision_preview --source data/frames

    # single image, write the annotated result to disk (works headless)
    python -m scripts.vision_preview --source shot.jpg --save out.jpg

    # diagnose a new model file: input size, dtype, output tensor order
    python -m scripts.vision_preview --inspect

Keys in the live window: q quits, s saves the current annotated frame.

Overlay colours:
    green box   the selected pursuer (or any person when the gate is off)
    red box     a detected person the identity gate rejected
    yellow line frame centre; the bearing readout is relative to this
"""

import argparse
import glob
import os
import time

import cv2

from app import config
from app.perception import geometry
from app.perception import identity

GREEN = (0, 200, 0)
RED = (0, 0, 220)
YELLOW = (0, 220, 220)
WHITE = (255, 255, 255)


def load_detector(model_path):
    """Import and build the detector, with an actionable error if no TFLite
    runtime is available. tflite-runtime is discontinued and has no wheels
    for newer Python/aarch64 (including the UNO Q's Debian trixie / Python
    3.13, and macOS); ai-edge-litert is the maintained, lightweight
    replacement, with tensorflow as the heaviest but most universal fallback.
    detector.py already tries all three in that order -- this just improves
    the message when none are installed."""
    try:
        from app.perception.detector import PersonDetector
    except ImportError as exc:
        raise SystemExit(
            f"No TFLite interpreter available ({exc}).\n"
            "  Recommended (UNO Q or laptop):  pip install ai-edge-litert\n"
            "  Heaviest fallback:               pip install tensorflow")
    return PersonDetector(model_path)


def inspect_model(model_path):
    """Print tensor layout -- use this when detections look wrong.

    detector._read_outputs() assumes the output order [boxes, classes, scores],
    and _preprocess() assumes a uint8 input. Both vary between models; this
    prints what your file actually wants so you can fix the indices once.
    """
    det = load_detector(model_path)
    print(f"model: {model_path}")
    for d in det.input_details:
        print(f"  input   {d['index']}  shape={list(d['shape'])}  dtype={d['dtype'].__name__}")
    for i, d in enumerate(det.output_details):
        print(f"  output  idx {i} (tensor {d['index']})  "
              f"shape={list(d['shape'])}  dtype={d['dtype'].__name__}  name={d['name']}")
    print("\ndetector._read_outputs() currently reads output idx 0=boxes, "
          "1=classes, 2=scores.")
    print("A 4-element shape like [1, N, 4] is the boxes tensor; two [1, N] "
          "tensors are classes and scores; a [1] tensor is the detection count."
          "\nIf the order above differs, adjust _read_outputs() in "
          "app/perception/detector.py.")


def annotate(frame, detections, chosen, name, fps, gate_on,
             selector_locked=None):
    """Draw boxes and the numbers the control loop will act on."""
    h, w = frame.shape[:2]
    cv2.line(frame, (w // 2, 0), (w // 2, h), YELLOW, 1)

    for det in detections:
        is_chosen = chosen is not None and det.bbox == chosen.bbox
        colour = GREEN if is_chosen else RED
        x1, y1, x2, y2 = (int(v) for v in det.bbox)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)

        label = f"{det.score:.2f}"
        if is_chosen and name:
            locked = selector_locked and name == selector_locked
            label = (f"[LOCKED] {name}" if locked
                     else f"{name} {det.score:.2f}")
        elif gate_on and not is_chosen:
            # Show WHY this person was rejected, to guide tuning.
            h_frac = (det.bbox[3] - det.bbox[1]) / h
            if h_frac < config.ID_MIN_PERSON_H_FRAC:
                label = f"too far ({h_frac:.0%})"
            else:
                label = f"not you? ({det.score:.2f})"
        cv2.putText(frame, label, (x1, max(16, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)

        if is_chosen:
            bearing = geometry.bbox_to_bearing(det.bbox)
            proximity = geometry.bbox_to_proximity(det.bbox)
            box_frac = (y2 - y1) / h
            side = "LEFT" if bearing < -0.05 else "RIGHT" if bearing > 0.05 else "AHEAD"
            lines = [
                f"bearing   {bearing:+.2f}  ({side})",
                f"proximity {proximity:.2f}"
                + ("  CAUGHT" if proximity >= config.CAUGHT_PROXIMITY else ""),
                f"box height {box_frac:.2f} of frame",
            ]
            for i, text in enumerate(lines):
                cv2.putText(frame, text, (10, h - 14 - 22 * (len(lines) - 1 - i)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)

    status = f"{len(detections)} person(s)  {fps:.1f} fps"
    status += "  gate ON" if gate_on else "  gate off"
    cv2.rectangle(frame, (0, 0), (w, 26), (30, 30, 30), -1)
    cv2.putText(frame, status, (10, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 2)
    return frame


def iter_frames(source, camera_arg=None):
    """Yield (name, frame) from a live camera, an image file, or a directory."""
    if source == "cam":
        from app.perception.camera import Camera, parse_source
        cam = Camera(parse_source(camera_arg))
        print(f"[preview] capturing from {cam.source!r} at {cam.actual_size()}")
        try:
            while True:
                frame = cam.read()
                if frame is None:
                    continue
                yield "live", frame
        finally:
            cam.release()
    elif os.path.isdir(source):
        paths = sorted(glob.glob(os.path.join(source, "*.jpg"))
                       + glob.glob(os.path.join(source, "*.png")))
        if not paths:
            raise SystemExit(f"no .jpg/.png files in {source}")
        for path in paths:
            frame = cv2.imread(path)
            if frame is not None:
                yield os.path.basename(path), frame
    else:
        frame = cv2.imread(source)
        if frame is None:
            raise SystemExit(f"could not read image: {source}")
        yield os.path.basename(source), frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="models/person_detect.tflite")
    ap.add_argument("--source", default="cam",
                    help="'cam' (use --camera), an image file, or a directory")
    ap.add_argument("--camera", default=None,
                    help="capture source: V4L2 index (e.g. 0) or stream URL "
                         "(e.g. http://192.168.4.1:81/stream). "
                         "Defaults to config.CAMERA_SOURCE.")
    ap.add_argument("--save", metavar="PATH",
                    help="write annotated output here (file, or dir for many)")
    ap.add_argument("--no-window", action="store_true",
                    help="skip the GUI (headless boards); use with --save")
    ap.add_argument("--inspect", action="store_true",
                    help="print model tensor layout and exit")
    ap.add_argument("--no-gate", action="store_true",
                    help="ignore enrolled identities; show every person as chosen")
    args = ap.parse_args()

    if not os.path.exists(args.model):
        raise SystemExit(
            f"model not found: {args.model}\n"
            "Download a TFLite person-detection model into models/ first "
            "(see README).")

    if args.inspect:
        inspect_model(args.model)
        return

    detector = load_detector(args.model)
    db = identity.IdentityDB()
    gate_on = config.IDENTITY_GATE and len(db) > 0 and not args.no_gate
    selector = identity.PursuerSelector(db) if gate_on else None
    print(f"[preview] identity gate {'ON: ' + str(db.names()) if gate_on else 'off'}")

    save_dir = None
    if args.save and (os.path.isdir(args.save) or args.source != "cam"
                      and os.path.isdir(args.source)):
        save_dir = args.save
        os.makedirs(save_dir, exist_ok=True)

    show = not args.no_window
    frames = seen = 0
    t0 = time.time()

    try:
        for name, frame in iter_frames(args.source, args.camera):
            frames += 1
            detections = detector.detect(frame)
            seen += bool(detections)

            if gate_on:
                chosen, who = selector.select(frame, detections)
            else:
                chosen = max(detections, key=lambda d: d.score) if detections else None
                who = None
            locked = selector.locked_identity() if gate_on else None

            fps = frames / max(time.time() - t0, 1e-6)
            annotate(frame, detections, chosen, who, fps, gate_on, locked)

            if save_dir:
                cv2.imwrite(os.path.join(save_dir, f"annotated_{name}"), frame)
            elif args.save:
                cv2.imwrite(args.save, frame)
                print(f"[preview] wrote {args.save}")

            if show:
                cv2.imshow("vision preview", frame)
                key = cv2.waitKey(0 if args.source != "cam" else 1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("s"):
                    out = f"preview_{int(time.time())}.jpg"
                    cv2.imwrite(out, frame)
                    print(f"[preview] saved {out}")
    except KeyboardInterrupt:
        pass
    finally:
        if show:
            cv2.destroyAllWindows()

    print(f"[preview] {frames} frames, a person was detected in {seen} "
          f"({100 * seen / max(frames, 1):.0f}%)")


if __name__ == "__main__":
    main()
