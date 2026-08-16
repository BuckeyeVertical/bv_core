#!/bin/bash
# Usage: ./run_docker.sh [IMAGE_NAME]
# If no IMAGE_NAME is provided, use the local simulation image.
IMAGE_NAME="${1:-bv-mission:latest}"
CONTAINER_NAME="bv-mission"
HOST_DIR="$(cd ../../.. && pwd)"
CONTAINER_SRC_DIR="/bv_ws"


docker run -it \
  --name "$CONTAINER_NAME" \
  --privileged \
  -v "$HOST_DIR":"$CONTAINER_SRC_DIR" \
  -p 8765:8765 \
  "$IMAGE_NAME" \
  bash
