# Build context is the repo root (UR_01/), not this directory -- see
# docker-compose.yml's `build.context: ..`. That's needed to pull in
# ../yolo/smoke_test.py (a teammate's object-detection module, reused here
# rather than duplicated) alongside this service's own src/.

FROM python:3.10-slim

# libgl1/libglib2.0-0: required by opencv even in headless mode.
# libegl1/libgles2: mediapipe's native lib dlopens these even when only
# running CPU inference (no GPU delegate actually used).
# libusb-1.0-0/libudev1: pyrealsense2's bundled librealsense needs these at
# runtime to talk to a RealSense over USB (CAMERA_BACKEND=realsense).
# curl: only needed to fetch the MediaPipe model files below (python:3.10-
# slim doesn't include it).
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 libglib2.0-0 libegl1 libgles2 libusb-1.0-0 libudev1 curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY vision_human_track/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Model files aren't committed to git (binary, ~13MB) - fetched at build
# time instead, so a fresh clone builds with nothing extra to hand-carry.
COPY vision_human_track/download_models.sh .
RUN chmod +x download_models.sh && ./download_models.sh

COPY vision_human_track/src/ src/
COPY yolo/smoke_test.py yolo/smoke_test.py

# Bakes the default YOLO weight into the image so a fresh container's first
# /detect/live call (with YOLO_ENABLED=true) doesn't need outbound network
# access -- same reasoning as download_models.sh above. Override at runtime
# with YOLO_MODEL=yolo26s.pt/m/l/x; a size not baked in here falls back to
# ultralytics' own auto-download (needs network) on first use -- see
# src/api.py's `_load_models`. Only fetched if YOLO_ENABLED is set at build
# time; skip entirely for a human/hand-only deployment to save the
# download + image size.
ARG YOLO_ENABLED=true
ARG YOLO_MODEL=yolo26n.pt
RUN if [ "$YOLO_ENABLED" = "true" ]; then \
      mkdir -p yolo && cd yolo && \
      python3 -c "from ultralytics import YOLO; YOLO('${YOLO_MODEL}')"; \
    fi

EXPOSE 8000

WORKDIR /app/src
CMD ["uvicorn", "api:app", "--host", "0.0.0.0", "--port", "8000"]
