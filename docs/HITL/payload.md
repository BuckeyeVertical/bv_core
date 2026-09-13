# Payload Drop

Two servos: the **plate** holds both payloads up, and the **clamp** brakes the
line. Values are in the `payload:` block of `config/real_params.yaml`. The
mission only drops when `payload.enabled: true`.

## Flight controller (QGC)

| Parameter | Plate | Clamp |
|---|---|---|
| `PWM_MAIN_FUNC` | 302 | 301 |
| `PWM_MAIN_MIN` / `MAX` | 800 / 2200 | 800 / 2200 |
| `PWM_MAIN_DIS` | 1685 | 1900 |
| `PWM_MAIN_TIM` | 50 | 50 |

`COM_PREARM_MODE = 2` lets the servos move while disarmed.

## Update the Jetson

From the laptop, fix the Jetson clock (a wrong clock makes the build silently
keep old code), then sync:

```bash
cd ~/Code/bv_ws/src/bv_core
./scripts/sync_jetson_clock.sh
./scripts/sync_jetson.sh jetson-usbc
```

On the Jetson, rebuild:

```bash
cd ~/bv_ws
colcon build --packages-select bv_core
source install/setup.bash
```

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

Terminal 2 — watch the outputs (channel 1 = clamp, channel 2 = plate):

```bash
ros2 topic echo /mavros/rc/out --field channels
```

Terminal 3 — commands:

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
and the clamp brakes through `brake_phases`, then both servos go back to rest.
The last line should read `Drop complete: N/N commands confirmed by PX4`.
