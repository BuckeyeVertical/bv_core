#!/bin/bash

# Terminal colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color
PORT="/dev/ttyUSB0"
LAUNCH_FILE="traj_test_launch"

# Create logs directory with timestamp
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
# LOG_DIR="/ros2_ws/logs/run_${TIMESTAMP}"
LOG_DIR="/bv_ws/logs"
mkdir -p "${LOG_DIR}"

# MicroXRCE Agent
echo "Starting MicroXRCEAgent"
MicroXRCEAgent serial --dev $PORT -b 921600 > "${LOG_DIR}/microxrce.log" 2>&1 &

# Navigate to workspace
cd /bv_ws/BuckeyeVerticalSoftwareV2

# Build packages
echo -e "${YELLOW}Building ROS packages...${NC}"
colcon build

# Source workspace
source /bv_ws/BuckeyeVerticalSoftwareV2/install/local_setup.bash

# Launch application
echo -e "${GREEN}Build complete. Starting ROS application...${NC}"
pwd
ls /bv_ws/BuckeyeVerticalSoftwareV2/install/px4_ros_com/lib/px4_ros_com
ros2 run px4_ros_com ${LAUNCH_FILE} 2>&1 | tee "${LOG_DIR}/ros.log"
