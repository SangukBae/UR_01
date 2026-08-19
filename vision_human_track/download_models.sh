#!/bin/bash
# Downloads the two MediaPipe Tasks model files this service needs.
# Not committed to git (binary, ~13MB combined) - run this once instead,
# locally or as part of the Docker build (see Dockerfile).
set -e
cd "$(dirname "$0")"
mkdir -p models

curl -fsSL -o models/pose_landmarker_lite.task \
  "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task"
curl -fsSL -o models/hand_landmarker.task \
  "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"

echo "Downloaded MediaPipe models to $(pwd)/models"
