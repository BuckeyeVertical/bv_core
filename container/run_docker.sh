#!/bin/bash

IMAGE_NAME="dreamer_docker" # Leaving this as dreamer_docker for now because I don't want to build a redundant docker on the raspi
CONTAINER_NAME="dreamer_container"
HOST_DIR="/home/eashan/bv_ws"
CONTAINER_SRC_DIR="/bv_ws"

docker rm $CONTAINER_NAME

docker run -it \
    --name "$CONTAINER_NAME" \
    --network host \
    --privileged \
    -v "$HOST_DIR":"$CONTAINER_SRC_DIR" \
    "$IMAGE_NAME" \
    /bv_ws/BuckeyeVerticalSoftwareV2/containers/raspi/launch.sh

