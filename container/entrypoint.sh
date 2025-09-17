#!/bin/bash

source /opt/ros/humble/setup.bash
if [ -f "/bv_ws/install/setup.bash" ]; then
  source /bv_ws/install/setup.bash
fi

exec "$@"