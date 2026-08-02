# Models

`.gitignore` excludes `models/*.tflite`, so these binaries are NOT in the repo.
Each machine fetches them once:

```bash
curl -L -o models/person_detect.tflite \
  https://raw.githubusercontent.com/google-coral/test_data/master/ssd_mobilenet_v2_coco_quant_postprocess.tflite

# optional alternative, and the labels file for debugging
curl -L -o models/person_detect_mobiledet.tflite \
  https://raw.githubusercontent.com/google-coral/test_data/master/ssdlite_mobiledet_coco_qat_postprocess.tflite
curl -L -o models/coco_labels.txt \
  https://raw.githubusercontent.com/google-coral/test_data/master/coco_labels.txt
```

(If you would rather commit them -- 6 MB is well within GitHub's limits and
guarantees every teammate and the demo machine have the identical file --
use `git add -f models/person_detect.tflite`.)

## person_detect.tflite  (default)

SSD MobileNet V2, COCO, uint8-quantized, with the detection post-process op
baked in. 300x300 input, 6.2 MB. From the Coral test-data set (Apache 2.0,
same model family as the standard TFLite object-detection examples).

Verified against `app/perception/detector.py` with no code changes:

| property | value | matches detector.py |
|---|---|---|
| input | `[1, 300, 300, 3]` uint8 | yes -- `_preprocess()` feeds uint8 |
| output 0 | `[1, 20, 4]` boxes (ymin, xmin, ymax, xmax, normalized) | yes |
| output 1 | `[1, 20]` classes | yes |
| output 2 | `[1, 20]` scores | yes |
| output 3 | `[1]` count | unused |
| person class id | 0 | matches `config.PERSON_CLASS_ID` |

Sanity results: a photo of a person scores 0.84 and yields exactly one person
detection; a photo of a cat yields zero person detections (it is classified as
`cat`, class 16, and filtered out). Measured 22 ms/frame single-threaded on an
x86 container -- expect meaningfully slower on the UNO Q's Cortex-A53; run
`scripts/benchmark_fps.py` on the board for the number that matters.

## person_detect_mobiledet.tflite  (alternative)

SSDLite MobileDet, COCO, uint8-quantized, 320x320 input, 4.3 MB. Same tensor
layout and same person class id, so it is a drop-in swap:

    python -m scripts.vision_preview --model models/person_detect_mobiledet.tflite
    python -m scripts.benchmark_fps --model models/person_detect_mobiledet.tflite

MobileDet is usually the more accurate of the two per unit of compute, and it
returns up to 100 detections instead of 20. If the default misses people at
distance, benchmark this one on the board before doing anything more elaborate.
It did produce a duplicate lower-confidence box on the test image, so if you
switch, sanity-check `DETECT_SCORE_THRESHOLD` (the identity gate's
`PursuerSelector` handles duplicates fine -- it scores every box).

## coco_labels.txt

The 80 COCO class names in model order; line 1 is `person`. Only needed for
debugging (e.g. "what did it think that was?"), not by the app.

## Swapping in your own model

Any TFLite detector with the `TFLite_Detection_PostProcess` op works. Check it
first with:

    python -m scripts.vision_preview --inspect --model models/your_model.tflite

If the input dtype is float32, or the output order differs, adjust
`_preprocess()` / `_read_outputs()` in `app/perception/detector.py`.
