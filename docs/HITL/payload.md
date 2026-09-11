# Payload Release

Each delivery drops one payload from the hover altitude (150 ft) with two servos
on the flight controller:

- The **slider** holds both payloads at its zero position. Moving it one way drops
  the beacon (for the tent) and moving it the other way drops the bottle (for the
  person).
- The **brake** rests at its zero. During a drop it toggles between its pulse
  position and rest, with shorter intervals as the payload gets closer to the ground.

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

## Positions: degrees, zero plus offset

The degree values are the angles that were tested with Arduino `Servo.write()`.
Each servo is configured as one zero plus offsets:

```yaml
slider:
  zero_deg: 130          # hold
  beacon_offset_deg: -30 # 100 deg
  bottle_offset_deg: 40  # 170 deg
brake:
  zero_deg: 125          # rest
  pulse_offset_deg: -18  # 107 deg
```

**Re-zeroing:** after you re-mount a servo horn, find its new hold or rest angle
with the bench tool and change that servo's `zero_deg`. The offsets depend on the
mechanism, so they stay the same.

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

| Position | Degrees | Pulse |
|---|---|---|
| Slider hold | 130 | 1884 µs |
| Beacon release | 100 | 1575 µs |
| Bottle release | 170 | 2297 µs |
| Brake rest | 125 | 1833 µs |
| Brake pulse | 107 | 1647 µs |

If a position is outside its servo's `pwm_range_us`, `mission_node` refuses to
start and names the position. It does not clamp silently, because a clamped
release can look fine on the ground and then fail to drop the payload in flight.

> **The bottle release does not currently pass this check.** PX4 limits
> `PWM_MAIN_MAX` to 2200 µs (about 160°), and 170° needs 2297 µs. Fix it one of two
> ways:
> - Use the bench tool to find the smallest angle that still drops the bottle, then
>   lower `bottle_offset_deg`.
> - Raise `PWM_MAIN_MAXn` above 2200 from the PX4 console
>   (`param set PWM_MAIN_MAX5 2350`) and change `pwm_range_us` to match. The
>   firmware reads the value without clamping it and only QGroundControl enforces
>   2200, but this setting is outside PX4's documented range, so verify it on the
>   bench.

## Flight controller setup

The instructions assume the slider is on MAIN output *s* and the brake is on MAIN
output *b*. Use the outputs the servos are actually plugged into.

| Parameter | Slider (*s*) | Brake (*b*) | Why |
|---|---|---|---|
| `PWM_MAIN_FUNCs` / `FUNCb` | 301 | 302 | Actuator Set 1 and 2. These must match `actuator_set`. |
| `PWM_MAIN_MINs` / `MINb` | 800 | 800 | Must equal `pwm_range_us[0]`. |
| `PWM_MAIN_MAXs` / `MAXb` | 2200 | 2200 | Must equal `pwm_range_us[1]`. |
| `PWM_MAIN_DISs` / `DISb` | **1884** | **1833** | Pulse sent while disarmed: slider hold and brake rest. |

**The disarmed pulse matters.** PX4 only sends commanded values to peripheral
outputs while the vehicle is armed. While disarmed, including at power-up and after
landing, each output sends its `PWM_MAIN_DISn` pulse. If that pulse is 0 (no
signal), the slider goes limp and can drop a payload on the ground. If you re-zero
a servo, recompute its disarmed pulse as `544 + zero_deg × 1856 / 180`.

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
ros2 run bv_core test_servo rest            # slider hold + brake rest
ros2 run bv_core test_servo slider 160      # try an angle
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
and DEPLOY finishes immediately. `sim_params.yaml` ships with the payload disabled.
