#!/bin/bash

source /opt/ros/humble/setup.bash
if [ -f "/bv_ws/BuckeyeVerticalSoftwareV2/install/setup.bash" ]; then
  source /bv_ws/BuckeyeVerticalSoftwareV2/install/setup.bash
fi

exec "$@"