FROM python:3.10-slim

# libgl1/libglib2.0-0: required by opencv even in headless mode.
# libegl1/libgles2: mediapipe's native lib dlopens these even when only
# running CPU inference (no GPU delegate actually used).
# curl: only needed to fetch the model files below (python:3.10-slim
# doesn't include it).
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 libglib2.0-0 libegl1 libgles2 curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Model files aren't committed to git (binary, ~13MB) - fetched at build
# time instead, so a fresh clone builds with nothing extra to hand-carry.
COPY download_models.sh .
RUN chmod +x download_models.sh && ./download_models.sh

COPY src/ src/

EXPOSE 8000

WORKDIR /app/src
CMD ["uvicorn", "api:app", "--host", "0.0.0.0", "--port", "8000"]
