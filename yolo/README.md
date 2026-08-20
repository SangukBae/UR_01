# Object detection with YOLO

This smoke test runs YOLO26 in Docker, detects every recognized object in
`test.png`, prints a structured JSON result, and saves an annotated image. It
exits successfully only when the model loads, inference completes, at least one
object is detected, and the annotated image is written.

## Install NVIDIA Docker support

First install Docker and the NVIDIA driver. Confirm that the host can see the
GPU:

```bash
nvidia-smi
```

On Ubuntu or Debian, add NVIDIA's package repository and install the NVIDIA
Container Toolkit:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
```

Check that the GPU devices are available to Docker:

```bash
nvidia-ctk cdi list
```

The current CDI method requires Docker 28.2 or newer and NVIDIA Container
Toolkit 1.18 or newer.

## Pull the YOLO image

```bash
docker pull ultralytics/ultralytics:latest
```

## Run with the GPU

From this directory:

```bash
docker run --rm --ipc=host \
  --device nvidia.com/gpu=all \
  -e YOLO_DEVICE=0 \
  -v "$PWD":/workspace \
  -w /workspace \
  ultralytics/ultralytics:latest \
  python smoke_test.py
```

The model weights (`yolo26n.pt`) are downloaded automatically if needed.

The script uses `test.png` by default. To detect another image, pass its path:

```bash
docker run --rm --ipc=host \
  --device nvidia.com/gpu=all \
  -e YOLO_DEVICE=0 \
  -v "$PWD":/workspace \
  -w /workspace \
  ultralytics/ultralytics:latest \
  python smoke_test.py my_image.jpg
```

To run without an NVIDIA GPU, omit `--device` and `-e YOLO_DEVICE=0`. The smoke
test uses the CPU by default.

## Result structure

On success, the script prints a JSON object with run metadata and every
detection. YOLO's numeric `class_id` is mapped to the corresponding
`class_name`:

```json
{
  "status": "passed",
  "model": "yolo26n.pt",
  "device": "cpu",
  "input_image": "/workspace/test.png",
  "annotated_image": "/workspace/test_detected.jpg",
  "detection_count": 1,
  "detections": [
    {
      "class_id": 47,
      "class_name": "apple",
      "instance_name": "apple1",
      "confidence": 0.643,
      "xyxy": [217.5, 108.4, 351.4, 243.1]
    }
  ]
}
```

The top-level fields are:

- `status`: `passed` after all smoke-test checks succeed
- `model`: model path or name selected with `YOLO_MODEL`
- `device`: inference device selected with `YOLO_DEVICE`
- `input_image`: absolute path to the source image
- `annotated_image`: absolute path to the generated JPEG
- `detection_count`: number of entries in `detections`
- `detections`: one record per detected object

Each detection contains the numeric and human-readable class, a per-class
instance name, confidence rounded to three decimals, and an `xyxy` bounding
box. `xyxy` contains pixel coordinates:

- `x1`, `y1`: top-left corner
- `x2`, `y2`: bottom-right corner

If any check fails, Python exits non-zero and prints an exception instead of a
successful JSON result.

`instance_name` numbers detections separately within each class, producing names
such as `apple1`, `apple2`, and `person1`. For a video stream these numbers are
recalculated on every frame; persistent identities require object tracking.

The output name is derived from the input name by adding `_detected`. For
example, `test.png` produces `test_detected.jpg`. Every detected box is labelled
with its `instance_name` and confidence, such as `apple1 0.88` and
`apple2 0.86`; the code does not filter the output to apples. A pretrained model
can only name classes included in its training dataset—for example, the COCO
model recognizes `apple` but has no `grape` class.
