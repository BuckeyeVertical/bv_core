#!/bin/bash
# Usage: ./run_docker.sh [IMAGE_NAME]
# If no IMAGE_NAME is provided, defaults to "bv-raspi:dev"
IMAGE_NAME="${1:-bv-raspi:dev}"
CONTAINER_NAME="bvdev"
HOST_DIR="$(cd ../../.. && pwd)"
CONTAINER_SRC_DIR="/bv_ws"

docker rm $CONTAINER_NAME

docker run -it --rm \
  --name "$CONTAINER_NAME" \
  --privileged \
  -v "$HOST_DIR":"$CONTAINER_SRC_DIR" \
  -p 8765:8765 \
  "$IMAGE_NAME" \
  bash -lc 'mkdir -p /bv_ws/logs; nohup bash /bv_ws/container/launch.sh > /bv_ws/logs/foxglove_bridge.log 2>&1 & exec bash'