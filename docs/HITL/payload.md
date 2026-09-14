# Payload Drop

## Bench test

Remove the propellers.

Terminal 1 — MAVROS:

```bash
ssh jetson-usbc
source ~/bv_ws/.venv/bin/activate
source ~/bv_ws/install/setup.bash

ros2 launch mavros px4.launch \
  fcu_url:=serial:///dev/ttyTHS1:921600
```

Terminal 2 — commands:

```bash
ros2 run bv_core test_servo show            # configured values
ros2 run bv_core test_servo rest            # plate hold, clamp open

ros2 run bv_core test_servo plate 1685      # hold both
ros2 run bv_core test_servo plate 1360      # beacon drop
ros2 run bv_core test_servo plate 2050      # bottle drop

ros2 run bv_core test_servo clamp 1900      # unclamped
ros2 run bv_core test_servo clamp 1577      # clamped, bottle
ros2 run bv_core test_servo clamp 1710      # clamped, beacon

ros2 run bv_core test_servo drop bottle     # full drop from the config
ros2 run bv_core test_servo drop beacon

ros2 run bv_core test_servo drop bottle 200:3   # one rhythm: toggle_ms:seconds
ros2 run bv_core test_servo drop bottle 150:3
ros2 run bv_core test_servo drop bottle 100:3
```

A drop takes 1 s of braking with the plate holding, then the plate releases
and the clamp brakes through that payload's own `brake_phases`, then both
servos go back to rest.
The last line should read `Drop complete: N/N commands confirmed by PX4`.
