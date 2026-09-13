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
| Clamp | Clamped for the beacon | 1710 |

To change a position, measure it in QGC the same way, then type the new number into
`real_params.yaml` and rebuild.

## Drop sequence

**Pre-brake (`pre_drop_s`, 1 s):** the clamp starts braking while the plate still
holds. It toggles between that payload's clamped position and unclamped at the
first phase's 200 ms rhythm (the first phase that actually runs). The rhythm is continuous for the whole drop: every
flip comes one interval after the previous one, so the clamp never pauses where
the interval changes.

**Drop:** after the pre-brake, the plate moves to that payload's drop position.
The braking phases then run in full; the pre-brake doesn't shorten them. Each
phase is `{duration_s, toggle_ms}`:

| Phase | `toggle_ms` | `duration_s` | Worked out from |
|---|---|---|---|
| 1 | 200 | 5.0 | 75 ft at 15 ft/s |
| 2 | 150 | 3.76 | 50 ft at 13.3 ft/s |
| 3 | 100 | 2.0 | 25 ft at 12.5 ft/s |

A phase with `duration_s: 0` is skipped, so zeroing two phases tests the third
on its own. `toggle_ms` must stay above 0, and together the phases must last more
than 0 s. Otherwise `mission_node` refuses to start, because the plate would move
out and straight back before anything fell.

After the last phase the clamp opens (1900) and the
plate returns to hold (1685), so the payload still aboard is gripped again. The
drone holds over the target for the whole drop, 1 s plus 10.76 s = 11.76 s, then
resumes the scan.
Deliveries can happen in either order. If RTL interrupts a drop, both servos go
back to rest the same way.

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
| `PWM_MAIN_FUNCp` / `FUNCc` | 302 | 301 | Plate = Actuator Set 2, clamp = Actuator Set 1. These must match `actuator_set`. |
| `PWM_MAIN_MINp` / `MINc` | 800 | 800 | Must equal `pwm_range_us[0]`. |
| `PWM_MAIN_MAXp` / `MAXc` | 2200 | 2200 | Must equal `pwm_range_us[1]`. |
| `PWM_MAIN_DISp` / `DISc` | **1685** | **1900** | The pulse sent while disarmed: plate holding, clamp open. |
| `PWM_MAIN_TIMx` | 50 | 50 | 50 Hz. Don't share a timer group with a DShot or 400 Hz motor. |

Reboot the flight controller after changing `FUNC` or `TIM`.

**The disarmed pulse matters.** Each output sends its `DIS` pulse at power-up,
until the first command arrives. With `COM_PREARM_MODE = 2` (this aircraft's
setting) the servos then follow commands even while disarmed. With any other
value they stay at `DIS` whenever the vehicle is disarmed. The plate must hold the payloads at that
pulse. `ros2 run bv_core test_servo show` prints the current `DIS` values.

`mission_node` also sends the hold and unclamped positions once at startup, before
arming.

Commands are addressed to PX4. Don't switch them to broadcast: PX4 on the flight
controller answers a broadcast `MAV_CMD_DO_SET_ACTUATOR` with UNSUPPORTED and leaves
the outputs alone, even though SITL accepts it. An addressed command makes MAVROS
wait for PX4's reply, which takes about 10 ms over serial, and MAVROS refuses a
second command of the same type until then. So only one command is ever in
flight. Newer positions wait for the reply, and a command PX4 didn't confirm is
re-sent after 0.2 s. The drop timing follows the clock either way.

This is all in one class, `PayloadWriter` (`bv_core/payload_writer.py`). The
mission's DEPLOY and `test_servo drop` both use it, with the same `DropSequence`,
so a bench drop runs exactly the code that flies. Only the trigger differs.

## Bench testing

Remove the propellers. The servos only respond while disarmed if
`COM_PREARM_MODE = 2` (Always). That is this aircraft's normal setting, used in
flight too, so there's nothing to change. With another value you'd have to arm to
bench-test. Start MAVROS and run:

```bash
ros2 run bv_core test_servo show            # configured pulses, range check, DIS values
ros2 run bv_core test_servo rest            # plate hold + clamp open
ros2 run bv_core test_servo plate 1685      # one servo to a pulse
ros2 run bv_core test_servo clamp 1577
ros2 run bv_core test_servo drop bottle     # full 11.76 s sequence (or: beacon)
ros2 run bv_core test_servo drop bottle 150:3   # braking at 150 ms for 3 s only
```

Brake phases after the payload name (`toggle_ms:duration_s`, one or more, e.g.
`200:5 150:3.76 100:2`) replace the configured ones for that drop only. The 1 s
pre-brake still runs first, at the first given rhythm. The YAML isn't touched.

The tool reads the mission config selected by `BV_MISSION_CONFIG` (default
`real_params.yaml`). It ignores `payload.enabled` and the startup range check.

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
