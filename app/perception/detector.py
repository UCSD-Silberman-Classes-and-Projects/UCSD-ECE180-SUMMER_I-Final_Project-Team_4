"""TFLite person detection.

Clean surface: detect(frame) -> list[Detection]. Swap the model file without
touching the main loop. Filters to the person class above a score threshold.
"""

from collections import namedtuple

import numpy as np

try:
    # On-device: prefer the standalone tflite-runtime if present.
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    try:
        # tflite-runtime is discontinued and has no wheels for newer
        # Python/aarch64 combos; ai-edge-litert is Google's maintained
        # successor and is what `pip install ai-edge-litert` gives you on
        # the UNO Q (Debian trixie ships Python 3.13, which tflite-runtime
        # never got a wheel for).
        from ai_edge_litert.interpreter import Interpreter
    except ImportError:
        try:
            # Heaviest fallback: full TensorFlow. Works everywhere, including
            # a Mac dev machine, at the cost of a large install.
            from tensorflow.lite.python.interpreter import Interpreter
        except ImportError as exc:
            raise ImportError(
                "No TFLite interpreter available. Install one of:\n"
                "  pip install ai-edge-litert   (recommended, lightweight)\n"
                "  pip install tensorflow       (heavier, works everywhere)"
            ) from exc

from .. import config

Detection = namedtuple("Detection", ["bbox", "score", "cls"])


class PersonDetector:
    def __init__(self, model_path,
                 score_threshold=config.DETECT_SCORE_THRESHOLD,
                 person_class=config.PERSON_CLASS_ID,
                 num_threads=None):
        # The UNO Q's Cortex-A53 is quad-core; a single-threaded interpreter
        # leaves 3 cores idle and was measured to fail the 10 FPS gate
        # (6 FPS) on this board. num_threads=None uses config.DETECT_THREADS.
        threads = num_threads if num_threads is not None else config.DETECT_THREADS
        self.interpreter = Interpreter(model_path=model_path,
                                       num_threads=threads)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        _, self.in_h, self.in_w, _ = self.input_details[0]["shape"]
        self.score_threshold = score_threshold
        self.person_class = person_class

    def detect(self, frame):
        """Run inference on a BGR frame. Return person detections only.

        Bounding boxes are returned in the ORIGINAL frame's pixel coordinates.
        """
        h, w = frame.shape[:2]
        inp = self._preprocess(frame)
        self.interpreter.set_tensor(self.input_details[0]["index"], inp)
        self.interpreter.invoke()

        boxes, classes, scores = self._read_outputs()
        detections = []
        for box, cls, score in zip(boxes, classes, scores):
            if int(cls) != self.person_class or score < self.score_threshold:
                continue
            # TFLite detection boxes are usually [ymin, xmin, ymax, xmax] in 0..1.
            ymin, xmin, ymax, xmax = box
            bbox = (xmin * w, ymin * h, xmax * w, ymax * h)
            detections.append(Detection(bbox=bbox, score=float(score), cls=int(cls)))
        return detections

    def best(self, frame):
        """Convenience: return the single highest-score person, or None."""
        dets = self.detect(frame)
        return max(dets, key=lambda d: d.score) if dets else None

    def _preprocess(self, frame):
        import cv2
        resized = cv2.resize(frame, (self.in_w, self.in_h))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        # Assumes a uint8 quantized model; adjust if the model wants float input.
        return np.expand_dims(rgb.astype(np.uint8), axis=0)

    def _read_outputs(self):
        # Output tensor order varies by model; adjust indices to match yours.
        out = self.output_details
        boxes = self.interpreter.get_tensor(out[0]["index"])[0]
        classes = self.interpreter.get_tensor(out[1]["index"])[0]
        scores = self.interpreter.get_tensor(out[2]["index"])[0]
        return boxes, classes, scores
