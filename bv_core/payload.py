"""Payload release: the slider and brake servo sequence, driven through PX4.

Pure logic, no ROS, so it is unit-tested standalone. mission_node owns the timer
and the MAVROS client; this module only answers "where should each servo be at
time t" and "what MAVLink command puts them there".

The hardware, as tested with an Arduino before moving to the flight controller:

    slider  holds both payloads in the middle; one side drops the beacon, the
            other the bottle
    brake   rests; during a drop it toggles between its pulse position and
            rest, faster as the payload nears the ground

Positions are configured in degrees as one zero per servo plus a fixed offset
for each position (slider: beacon 0, hold 30, bottle 70). The offsets are the
mechanism's geometry and never change; the zero is wherever the horn happens to
sit on the spline. Re-mounting a horn means measuring and entering its new zero,
and it is also how the positions are moved into the flight controller's range.

Degrees become a pulse width the way Arduino's Servo.write() does it, so the
flight controller reproduces the pulses the team tested with. PX4 does not take a
pulse width: MAV_CMD_DO_SET_ACTUATOR carries -1..1 per "Peripheral via Actuator
Set N" output, which PX4 maps onto that output's PWM_MAIN_MINn..PWM_MAIN_MAXn. So
the second step converts the pulse into that range, which must match the FC.
"""

import math
from dataclasses import dataclass

# MAVLink command. PX4 ignores MAV_CMD_DO_SET_SERVO; this is the one it handles.
MAV_CMD_DO_SET_ACTUATOR = 187
NUM_ACTUATOR_SETS = 6

# Which payload each detector class receives. Indices follow CLASS_NAMES
# ("person", "tent"): water bottle to the mannequin, strobe beacon to the tent.
PAYLOAD_BY_CLASS = {0: 'bottle', 1: 'beacon'}
PAYLOADS = ('bottle', 'beacon')

FT_PER_M = 1.0 / 0.3048
ALTITUDE_WARNING_TOLERANCE_FT = 2.0


def degrees_to_us(degrees, pulse_range_us):
    """Pulse width for an angle, as Arduino's Servo.write() computes it."""
    low, high = pulse_range_us
    return low + degrees * (high - low) / 180.0


def us_to_actuator_value(pulse_us, pwm_range_us):
    """PX4 actuator value (-1..1) that makes an output emit this pulse."""
    low, high = pwm_range_us
    return 2.0 * (pulse_us - low) / (high - low) - 1.0


def payload_for_class(class_id):
    """The payload a detector class receives, or None for an unknown class."""
    if class_id is None:
        return None
    return PAYLOAD_BY_CLASS.get(int(class_id))


@dataclass(frozen=True)
class ServoConfig:
    name: str
    actuator_set: int
    zero_deg: float
    pwm_range_us: tuple


@dataclass(frozen=True)
class BrakePhase:
    drop_ft: float
    speed_ftps: float
    toggle_ms: float

    @property
    def duration_s(self):
        return self.drop_ft / self.speed_ftps

    @property
    def toggle_s(self):
        return self.toggle_ms / 1000.0


@dataclass(frozen=True)
class PayloadConfig:
    slider: ServoConfig
    brake: ServoConfig
    slider_offsets_deg: dict   # 'hold', 'beacon', 'bottle'
    brake_offsets_deg: dict    # 'rest', 'pulse'
    arduino_pulse_range_us: tuple
    brake_phases: tuple

    @property
    def slider_hold_deg(self):
        return self.slider.zero_deg + self.slider_offsets_deg['hold']

    def release_deg(self, payload):
        return self.slider.zero_deg + self.slider_offsets_deg[payload]

    @property
    def brake_rest_deg(self):
        return self.brake.zero_deg + self.brake_offsets_deg['rest']

    @property
    def brake_pulse_deg(self):
        return self.brake.zero_deg + self.brake_offsets_deg['pulse']

    @property
    def total_duration_s(self):
        return sum(phase.duration_s for phase in self.brake_phases)

    @property
    def drop_ft(self):
        return sum(phase.drop_ft for phase in self.brake_phases)

    def positions(self):
        """Every commanded position as (servo, label, degrees)."""
        return [
            (self.slider, 'hold', self.slider_hold_deg),
            *((self.slider, f'{payload} release', self.release_deg(payload))
              for payload in PAYLOADS),
            (self.brake, 'rest', self.brake_rest_deg),
            (self.brake, 'pulse', self.brake_pulse_deg),
        ]

    def actuator_value(self, servo, degrees):
        pulse = degrees_to_us(degrees, self.arduino_pulse_range_us)
        return us_to_actuator_value(pulse, servo.pwm_range_us)


def load_payload_config(mission_config):
    """Parse and validate the mission config's ``payload`` block.

    Returns None when the payload is disabled; nothing else in the block is read
    then, so a test flight works before the numbers are filled in.

    Raises ValueError for anything malformed, and for any position the flight
    controller's PWM range cannot reach. A servo that is quietly clamped to the
    nearest reachable pulse can look fine on the ground and never release in
    the air, so an unreachable position stops the mission before takeoff.
    """
    block = mission_config.get('payload')
    if not isinstance(block, dict):
        raise ValueError(
            "mission config is missing the 'payload' block "
            "(set payload.enabled: false to fly without one)")

    enabled = block.get('enabled')
    if not isinstance(enabled, bool):
        raise ValueError(
            f"payload.enabled must be true or false, got {enabled!r}")
    if not enabled:
        return None

    config = parse_payload_block(block)
    problems = unreachable_positions(config)
    if problems:
        raise ValueError(
            "payload positions the flight controller cannot reach: "
            + "; ".join(problems)
            + ". Re-mount the servo horn and enter its new zero_deg "
            "(docs/HITL/payload.md), or widen PWM_MAIN_MINn/MAXn on the FC "
            "and pwm_range_us to match.")
    return config


def parse_payload_block(block):
    """Parse the payload block's values, ignoring enabled and reachability.

    For the bench tool, which must work while the payload is disabled or a
    position is still out of range: finding a reachable position is its job.
    """
    block = _mapping(block, 'payload')
    arduino_range = _pulse_range(
        block.get('arduino_pulse_range_us'), 'payload.arduino_pulse_range_us')
    slider_block = _mapping(block.get('slider'), 'payload.slider')
    brake_block = _mapping(block.get('brake'), 'payload.brake')
    slider = _servo('slider', slider_block)
    brake = _servo('brake', brake_block)
    if slider.actuator_set == brake.actuator_set:
        raise ValueError(
            "payload.slider.actuator_set and payload.brake.actuator_set must "
            f"differ (both are {slider.actuator_set})")

    config = PayloadConfig(
        slider=slider,
        brake=brake,
        slider_offsets_deg={
            name: _number(slider_block, f'{name}_offset_deg', 'payload.slider')
            for name in ('hold', *PAYLOADS)
        },
        brake_offsets_deg={
            name: _number(brake_block, f'{name}_offset_deg', 'payload.brake')
            for name in ('rest', 'pulse')
        },
        arduino_pulse_range_us=arduino_range,
        brake_phases=_phases(block.get('brake_phases')),
    )
    return config


def unreachable_positions(config):
    """One message per position outside its servo's PWM range."""
    problems = []
    for servo, label, degrees in config.positions():
        problem = unreachable_reason(config, servo, degrees)
        if problem:
            problems.append(f"{servo.name} {label} {problem}")
    return problems


def unreachable_reason(config, servo, degrees):
    """Why a servo cannot reach this angle, or None when it can."""
    pulse = degrees_to_us(degrees, config.arduino_pulse_range_us)
    low, high = servo.pwm_range_us
    if low <= pulse <= high:
        return None
    return (f"{degrees:g} deg needs {pulse:.0f} us, outside pwm_range_us "
            f"[{low:g}, {high:g}]")


def altitude_mismatch_warning(config, altitude_m):
    """Warn when the brake phases were tuned for a different drop height."""
    altitude_ft = altitude_m * FT_PER_M
    if abs(config.drop_ft - altitude_ft) <= ALTITUDE_WARNING_TOLERANCE_FT:
        return None
    return (
        f"payload.brake_phases cover a {config.drop_ft:g} ft drop but delivery "
        f"altitude is {altitude_ft:.0f} ft: the brake timing will not match "
        f"the payload's descent")


def actuator_params(config, slider_deg=None, brake_deg=None):
    """MAV_CMD_DO_SET_ACTUATOR and its seven params for the given positions.

    A servo left as None gets NaN, which PX4 treats as "leave unchanged", so
    commanding one servo never disturbs the other.
    """
    params = [math.nan] * 7
    for servo, degrees in ((config.slider, slider_deg),
                           (config.brake, brake_deg)):
        if degrees is not None:
            params[servo.actuator_set - 1] = config.actuator_value(
                servo, degrees)
    # Actuator set index: param1..6 address sets 1..6. A float, because the
    # CommandLong fields are float32 and rosidl rejects an int.
    params[6] = 0.0
    return MAV_CMD_DO_SET_ACTUATOR, params


class DropSequence:
    """Servo positions over one drop, as a function of elapsed time.

    Mirrors the tested Arduino dropBeacon()/dropBottle(): the slider moves to
    the release and the brake to its pulse position at t=0; the brake then
    toggles between pulse and rest at each phase's interval, restarting on
    pulse at each phase boundary; when the last phase ends the brake rests and
    the slider stays at the release position.
    """

    def __init__(self, config, payload):
        if payload not in PAYLOADS:
            raise ValueError(f"unknown payload {payload!r}")
        self.config = config
        self.payload = payload
        self.slider_deg = config.release_deg(payload)

    def positions_at(self, elapsed_s):
        """(slider_deg, brake_deg, done) at elapsed_s into the drop."""
        phase_start = 0.0
        for phase in self.config.brake_phases:
            phase_end = phase_start + phase.duration_s
            if elapsed_s < phase_end:
                toggles = int(max(0.0, elapsed_s - phase_start)
                              // phase.toggle_s)
                brake = (self.config.brake_pulse_deg if toggles % 2 == 0
                         else self.config.brake_rest_deg)
                return self.slider_deg, brake, False
            phase_start = phase_end
        return self.slider_deg, self.config.brake_rest_deg, True


# -- parsing helpers -------------------------------------------------------

def _mapping(value, where):
    if not isinstance(value, dict):
        raise ValueError(f"{where} must be a mapping")
    return value


def _number(block, key, where):
    value = block.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{where}.{key} must be a number, got {value!r}")
    if not math.isfinite(value):
        raise ValueError(f"{where}.{key} must be finite, got {value!r}")
    return float(value)


def _pulse_range(value, where):
    if (not isinstance(value, (list, tuple)) or len(value) != 2
            or not all(isinstance(v, (int, float))
                       and not isinstance(v, bool) for v in value)):
        raise ValueError(f"{where} must be [min_us, max_us], got {value!r}")
    low, high = float(value[0]), float(value[1])
    if not 0.0 < low < high:
        raise ValueError(
            f"{where} must satisfy 0 < min_us < max_us, got {value!r}")
    return low, high


def _servo(name, block):
    where = f'payload.{name}'
    actuator_set = block.get('actuator_set')
    if (isinstance(actuator_set, bool) or not isinstance(actuator_set, int)
            or not 1 <= actuator_set <= NUM_ACTUATOR_SETS):
        raise ValueError(
            f"{where}.actuator_set must be 1-{NUM_ACTUATOR_SETS} "
            f"(PX4 'Peripheral via Actuator Set N'), got {actuator_set!r}")
    return ServoConfig(
        name=name,
        actuator_set=actuator_set,
        zero_deg=_number(block, 'zero_deg', where),
        pwm_range_us=_pulse_range(
            block.get('pwm_range_us'), f'{where}.pwm_range_us'),
    )


def _phases(value):
    if not isinstance(value, list) or not value:
        raise ValueError("payload.brake_phases must be a non-empty list")
    phases = []
    for index, raw in enumerate(value):
        where = f'payload.brake_phases[{index}]'
        raw = _mapping(raw, where)
        values = {key: _number(raw, key, where)
                  for key in ('drop_ft', 'speed_ftps', 'toggle_ms')}
        for key, number in values.items():
            if number <= 0.0:
                raise ValueError(
                    f"{where}.{key} must be positive, got {number:g}")
        phases.append(BrakePhase(**values))
    return tuple(phases)
