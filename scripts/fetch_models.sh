#!/usr/bin/env bash
# Download the person-detection models (gitignored, so fetch once per machine).
# Usage: bash scripts/fetch_models.sh
set -euo pipefail
BASE="https://raw.githubusercontent.com/google-coral/test_data/master"
mkdir -p models
echo "fetching default model (SSD MobileNet V2, 6.2 MB)..."
curl -fL -o models/person_detect.tflite "$BASE/ssd_mobilenet_v2_coco_quant_postprocess.tflite"
echo "fetching alternative (SSDLite MobileDet, 4.3 MB)..."
curl -fL -o models/person_detect_mobiledet.tflite "$BASE/ssdlite_mobiledet_coco_qat_postprocess.tflite"
echo "fetching COCO labels..."
curl -fL -o models/coco_labels.txt "$BASE/coco_labels.txt"
echo
echo "done. verify with:  python -m scripts.vision_preview --inspect"
