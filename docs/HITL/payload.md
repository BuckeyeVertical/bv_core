# Payload Release

Each delivery drops one payload from the hover altitude (150 ft) with two servos
on the flight controller:

- The **slider** holds both payloads in its middle position. Moving it one way
  drops the beacon (for the tent) and moving it the other way drops the bottle
  (for the person).
- The **brake** sits at rest. During a drop it toggles between its pulse position
  and rest, with shorter intervals as the payload gets closer to the ground.

`mission_node` runs the sequence in the DEPLOY state. The logic is in
`bv_core/payload.py` and the values are in the `payload:` block of the mission
config.

## Drop sequence

At t = 0 the slider moves to the release position and the brake moves to its pulse
position. The brake then toggles through three phases. Each phase lasts
`drop_ft / speed_ftps` seconds:

| Phase | Distance | Speed | Toggle interval | Duration |
|---|---|---|---|---|
| 1 | 75 ft | 15.0 ft/s | 200 ms | 5.00 s |
| 2 | 50 ft | 13.3 ft/s | 150 ms | 3.76 s |
| 3 | 25 ft | 12.5 ft/s | 100 ms | 2.00 s |

Every phase starts on the pulse position. When the last phase ends, the brake rests
and the slider stays at the release position, as in the Arduino test. The drone
holds over the target for the full 10.76 s and then resumes the scan. If RTL
interrupts a drop, the brake goes to rest.

At startup `mission_node` warns if the phase distances do not add up to the
delivery altitude.

## Positions: one zero plus fixed offsets

Each servo is configured as a `zero_deg` plus a fixed offset for each position.
The offsets come from the mechanism's geometry and never change. The zero depends
on where the horn happens to sit on the servo spline.

```yaml
slider:
  zero_deg: 100          # the beacon release, wherever the horn puts it
  beacon_offset_deg: 0
  hold_offset_deg: 30
  bottle_offset_deg: 70
brake:
  zero_deg: 107          # the pulse position
  pulse_offset_deg: 0
  rest_offset_deg: 18
```

With a zero of 100, which is how the horn was mounted for the Arduino test, the
slider positions are 100 / 130 / 170°. When the horn is re-mounted, you change
only `zero_deg` and all three positions move together.

## Degrees to PWM

PX4 does not accept a pulse width in the command. `MAV_CMD_DO_SET_ACTUATOR` carries
a value from -1 to 1 for each "Peripheral via Actuator Set N" output. PX4 maps that
range onto the output's `PWM_MAIN_MINn` to `PWM_MAIN_MAXn`. The conversion has two
steps:

1. Degrees to µs uses Arduino's mapping, `544 + deg × 1856 / 180`
   (`arduino_pulse_range_us`). The flight controller therefore sends the pulses
   that were tested on the bench.
2. µs to the -1..1 value uses `pwm_range_us`, which **must equal the flight
   controller's `PWM_MAIN_MINn` and `PWM_MAIN_MAXn`**.

If a position is outside its servo's `pwm_range_us`, `mission_node` refuses to
start and names the position. It does not clamp silently, because a clamped
release can look fine on the ground and then fail to drop the payload in flight.

PX4 limits `PWM_MAIN_MAX` to 2200 µs, so 800–2200 µs covers about **25–161°**.
The slider needs 70° of travel, so its zero must be between about **25° and 90°**.
A zero of about **58°** leaves the same margin on both ends (58 / 88 / 128°). With
the Arduino mounting (zero 100) the bottle needs 170° = 2297 µs, which is out of
range. The fix is to re-zero the slider.

## Re-zeroing a servo

Do this when you move a horn to a different spline tooth, or to bring the slider
into range. Take the propellers off and allow bench control first (see
[Bench testing](#bench-testing)).

1. **Command the target zero.** For the slider:
   `ros2 run bv_core test_servo slider 58`.
2. **Re-seat the horn.** With the servo holding that angle, pull the horn off and
   push it back on the tooth that puts the mechanism closest to its **beacon
   release** position. Splines only allow steps of roughly 7–15°, so it will be
   close but not exact.
3. **Fine-tune.** Nudge the angle (`test_servo slider 56`, `test_servo slider 60`,
   and so on) until the mechanism sits exactly at beacon release. **That angle is
   the new `zero_deg`.**
4. **Enter it** in `real_params.yaml`, then rebuild:
   `colcon build --packages-select bv_core`.
5. **Check** with `ros2 run bv_core test_servo show`. It prints every position with
   its pulse and whether it is in range. Then run `test_servo rest` (the slider
   should hold both payloads) and `test_servo drop bottle` / `drop beacon`.
6. **Update the disarmed pulse** (`PWM_MAIN_DISs`) to the new hold pulse printed by
   `test_servo show`.

The brake is re-zeroed the same way, using its pulse position as the zero.

## Flight controller setup

The instructions assume the slider is on MAIN output *s* and the brake is on MAIN
output *b*. Use the outputs the servos are actually plugged into.

| Parameter | Slider (*s*) | Brake (*b*) | Why |
|---|---|---|---|
| `PWM_MAIN_FUNCs` / `FUNCb` | 301 | 302 | Actuator Set 1 and 2. These must match `actuator_set`. |
| `PWM_MAIN_MINs` / `MINb` | 800 | 800 | Must equal `pwm_range_us[0]`. |
| `PWM_MAIN_MAXs` / `MAXb` | 2200 | 2200 | Must equal `pwm_range_us[1]`. |
| `PWM_MAIN_DISs` / `DISb` | slider **hold** pulse | brake **rest** pulse | Pulse sent while disarmed. Get both from `test_servo show`. They change whenever a zero changes. |

**The disarmed pulse matters.** PX4 only sends commanded values to peripheral
outputs while the vehicle is armed. While disarmed, including at power-up and after
landing, each output sends its `PWM_MAIN_DISn` pulse. If that pulse is 0 (no
signal), the slider goes limp and can drop a payload on the ground. After any
re-zero, set the disarmed pulses again from `test_servo show`.

`mission_node` sends the hold and rest positions once at startup, before arming.
PX4 stores them and applies them when the vehicle arms.

Commands are sent as MAVLink broadcasts, so MAVROS does not wait for ACKs. MAVROS
refuses a command while the previous one of the same type is waiting for its ACK,
and that wait can last 5 s. One lost ACK would otherwise freeze the brake in the
middle of a drop.

## Bench testing

Remove the propellers. Then either arm the vehicle or set `COM_PREARM_MODE = 2`
(Always) so peripheral outputs respond while disarmed. Start MAVROS and run:

```bash
ros2 run bv_core test_servo show            # positions, pulses, range check, DIS values
ros2 run bv_core test_servo rest            # slider hold + brake rest
ros2 run bv_core test_servo slider 58       # try an angle
ros2 run bv_core test_servo brake 107
ros2 run bv_core test_servo drop bottle     # full 10.76 s sequence (or: beacon)
```

The tool reads the mission config selected by `BV_MISSION_CONFIG` (default
`real_params.yaml`). It ignores `payload.enabled` and the startup range check, and
warns when an angle is out of range. Set `COM_PREARM_MODE` back to its flight value
afterwards.

## Turning the payload off

Set `payload.enabled: false` to fly without payload hardware. The drone still flies
to each target and holds, but it sends no servo commands and skips the range check,
and DEPLOY finishes immediately.
