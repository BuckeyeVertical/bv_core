# Payload Release

Each delivery drops one payload from the hover altitude (150 ft) using two servos
on the flight controller:

- The **plate** sits under both payloads and holds them up. Moving it one way drops
  the beacon (for the tent), and moving it the other way drops the bottle (for the
  person).
- The **clamp** grips the line to slow the fall. It does not hold anything up.
  During a drop it toggles between clamped and unclamped, faster as the payload
  gets closer to the ground. Each payload has its own clamped position.

`mission_node` runs the sequence in the DEPLOY state. The logic is in
`bv_core/payload.py`, and the values are in the `payload:` block of the mission
config.

## Positions

All positions are pulse widths in µs. They were measured on QGC's **Actuators** page
by typing a value into an output's disarmed field and watching the servo move. The
outputs run at 50 Hz.

| Servo | Position | µs |
|---|---|---|
| Plate | Hold both payloads | 1685 |
| Plate | Drop beacon (tent) | 1360 |
| Plate | Drop bottle (person) | 2050 |
| Clamp | Unclamped (also its rest position) | 1900 |
| Clamp | Clamped for the bottle | 1577 |
| Clamp | Clamped for the beacon | 1300 (placeholder, needs measuring) |

To change a position, measure it in QGC the same way, then type the new number into
`real_params.yaml` and rebuild.

## Drop sequence

At t = 0 the plate moves to that payload's drop position and the clamp closes on
that payload's clamped position. The clamp then toggles between clamped and
unclamped through three phases. Each phase lasts `drop_ft / speed_ftps` seconds:

| Phase | Distance | Speed | Toggle interval | Duration |
|---|---|---|---|---|
| 1 | 75 ft | 15.0 ft/s | 200 ms | 5.00 s |
| 2 | 50 ft | 13.3 ft/s | 150 ms | 3.76 s |
| 3 | 25 ft | 12.5 ft/s | 100 ms | 2.00 s |

Every phase starts clamped. After the last phase the clamp opens (1900) and the
plate stays at the drop position. The drone holds over the target for the full
10.76 s, then resumes the scan. Deliveries can happen in either order. If RTL
interrupts a drop, the clamp opens.

At startup `mission_node` warns if the phase distances don't add up to the delivery
altitude.

## How the pulse reaches the servo

PX4 does not take a pulse width in a command. `MAV_CMD_DO_SET_ACTUATOR` carries a
value between -1 and 1 for each "Peripheral via Actuator Set N" output, and PX4
maps that value onto the output's `PWM_MAIN_MINn` to `PWM_MAIN_MAXn`. The code does
the reverse conversion using `pwm_range_us`, so **`pwm_range_us` must equal those
two parameters**.

If a position is outside `pwm_range_us`, `mission_node` refuses to start and names
the position. It does not let PX4 quietly limit the pulse to the edge of the range.

## Flight controller setup

Below, the plate is on MAIN output *p* and the clamp is on MAIN output *c*. If the
servos are on the AUX rail, the parameters are named `PWM_AUX_*` instead.

| Parameter | Plate (*p*) | Clamp (*c*) | Why |
|---|---|---|---|
| `PWM_MAIN_FUNCp` / `FUNCc` | 301 | 302 | Actuator Set 1 and 2. These must match `actuator_set`. |
| `PWM_MAIN_MINp` / `MINc` | 800 | 800 | Must equal `pwm_range_us[0]`. |
| `PWM_MAIN_MAXp` / `MAXc` | 2200 | 2200 | Must equal `pwm_range_us[1]`. |
| `PWM_MAIN_DISp` / `DISc` | **1685** | **1900** | The pulse sent while disarmed: plate holding, clamp open. |
| `PWM_MAIN_TIMx` | 50 | 50 | 50 Hz. Don't share a timer group with a DShot or 400 Hz motor. |

Reboot the flight controller after changing `FUNC` or `TIM`.

**The disarmed pulse matters.** PX4 only sends commanded values to these outputs
while the vehicle is armed. At power-up, before arming, and after landing, each
output sends its `DIS` pulse instead. The plate must hold the payloads at that
pulse. `ros2 run bv_core test_servo show` prints the current `DIS` values.

`mission_node` also sends the hold and unclamped positions once at startup, before
arming.

Commands go out as MAVLink broadcasts, so MAVROS doesn't wait for acknowledgements.
MAVROS refuses a new command while an earlier one of the same type is still waiting
for its acknowledgement, which can take up to 5 s. One lost acknowledgement would
otherwise freeze the clamp in the middle of a drop.

## Bench testing

Remove the propellers. Then either arm the vehicle or set `COM_PREARM_MODE = 2`
(Always) so the outputs respond while disarmed. Start MAVROS and run:

```bash
ros2 run bv_core test_servo show            # configured pulses, range check, DIS values
ros2 run bv_core test_servo rest            # plate hold + clamp open
ros2 run bv_core test_servo plate 1685      # one servo to a pulse
ros2 run bv_core test_servo clamp 1577
ros2 run bv_core test_servo drop bottle     # full 10.76 s sequence (or: beacon)
```

The tool reads the mission config selected by `BV_MISSION_CONFIG` (default
`real_params.yaml`). It ignores `payload.enabled` and the startup range check. Set
`COM_PREARM_MODE` back to its flight value when you're done.

## Turning the payload off

Set `payload.enabled: false` to fly without payload hardware. The drone still flies
to each target and holds there. It sends no servo commands, skips the range check,
and DEPLOY finishes immediately.

## Changing a value: rebuild

`mission_node` reads the **installed** copy of the config, not the one in `src/`.
After editing a YAML file, or after `scripts/sync_jetson.sh` brings new commits to
the Jetson, rebuild before launching:

```bash
cd ~/bv_ws && colcon build --packages-select bv_core && source install/setup.bash
```
