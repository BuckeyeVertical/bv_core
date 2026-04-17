#!/bin/bash

source /opt/ros/humble/setup.bash
if [ -f "/opt/foxglove-sdk/install/setup.bash" ]; then
  source /opt/foxglove-sdk/install/setup.bash
fi
if [ -f "/bv_ws/install/setup.bash" ]; then
  source /bv_ws/install/setup.bash
fi

exec "$@"
