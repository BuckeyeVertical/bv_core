
#!/usr/bin/env bash
set -Eeuo pipefail

# Start Foxglove Bridge
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml 
#send_buffer_limit:=100000000