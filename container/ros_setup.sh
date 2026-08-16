#!/usr/bin/env bash

source /opt/ros/humble/setup.bash

if [[ -f /bv_ws/install/setup.bash ]]; then
    source /bv_ws/install/setup.bash
elif [[ -f /bv_ws/install_sim/setup.bash ]]; then
    source /bv_ws/install_sim/setup.bash
fi
