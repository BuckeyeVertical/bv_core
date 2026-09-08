# Run the Mission on the Jetson

## Maximize mission overhead

Run these commands from a normal SSH session before starting the mission:

```bash
sudo systemctl isolate multi-user.target
sudo systemctl stop docker.service docker.socket containerd.service
pkill -u "$USER" -f '/\.vscode-server/'
```

## Safety checks

`mission.launch.py` arms the aircraft automatically. Before starting it:

- Clear the aircraft and propeller area.
- Confirm the Herelink is connected and its Loiter/RTL controls work.
- Confirm PX4 reports no battery, GPS, EKF, or other preflight failures.
- Confirm the camera and PX4 serial port exist:

```bash
ls -l /dev/video0 /dev/ttyTHS*
```

Verify the camera can capture and decode eight full-resolution frames:
The command should exit without errors.
```bash
gst-launch-1.0 -q \
  v4l2src device=/dev/video0 num-buffers=8 ! \
  'image/jpeg,width=3840,height=2160,framerate=30/1' ! \
  jpegdec ! fakesink
```

Check one frame visually
```bash
ssh bvorinnano@192.168.144.2 "
rm -f /tmp/camera_frame.jpg
gst-launch-1.0 -q \
  v4l2src device=/dev/video0 num-buffers=1 ! \
  'image/jpeg,width=3840,height=2160,framerate=30/1' ! \
  jpegparse ! \
  filesink location=/tmp/camera_frame.jpg
"

scp bvorinnano@192.168.144.2:/tmp/camera_frame.jpg .
```

AFTER starting MAVROS below, verify PX4 and GPS before launching the mission:

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/global_position/global --once
```

Require `connected: true`, `armed: false`, and coordinates near the flight area.

The repository readiness checker automates those read-only checks plus the
camera/device checks above. Run it on the Jetson after MAVROS is started:

```bash
cd ~/bv_ws/src/bv_core
python3 scripts/mission_ready_check.py
```

It never arms, changes mode, uploads a mission, or launches the mission stack.
It exits nonzero when an automated check fails and prints the remaining manual
confirmations. The one-frame visual check above remains a separate manual step.

## Run the mission

Terminal 1 — MAVROS:

```bash
ssh bvorinnano@192.168.144.2
source ~/bv_ws/.venv/bin/activate
source ~/bv_ws/install/setup.bash

ros2 launch mavros px4.launch \
  fcu_url:=serial:///dev/ttyTHS1:921600
```

Terminal 2 — mission:

```bash
ssh bvorinnano@192.168.144.2
source ~/bv_ws/.venv/bin/activate
source ~/bv_ws/install/setup.bash
cd bv_ws
colcon build
export BV_MISSION_CONFIG=real_params.yaml
ros2 launch bv_core mission.launch.py
```

Open the GCS from the laptop:

- Herelink Wi-Fi: `http://192.168.144.2:8765`
- Direct USB-C: `http://192.168.55.1:8765`

For an in-flight abort, command Loiter or RTL from the Herelink. Do not use
`Ctrl-C` as a flight abort.
