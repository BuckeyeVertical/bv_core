"""Payload release: the plate and clamp servo sequence, driven through PX4.

Pure logic, no ROS, so it is unit-tested standalone. mission_node owns the timer
and the MAVROS client; this module only answers "where should each servo be at
time t" and "what MAVLink command puts them there".

The hardware:

    plate   sits under both payloads and holds them up; moving it one way drops
            the beacon, the other way the bottle
    clamp   grips the line to slow the payload's fall (it holds nothing up);
            during a drop it toggles between clamped and unclamped, faster as
            the payload nears the ground. Each payload has its own clamped
            position, since they weigh different amounts.

Positions are pulse widths in microseconds, read straight off QGC's Actuators
page (type a value into an output's disarmed field and watch the servo).

PX4 does not take a pulse width in a command: MAV_CMD_DO_SET_ACTUATOR carries
-1..1 per "Peripheral via Actuator Set N" output, which PX4 maps onto that
output's PWM_MAIN_MINn..PWM_MAIN_MAXn. pwm_range_us must therefore equal those
two parameters, and every position must lie inside it.
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

# Config keys per servo, without the _us suffix.
PLATE_POSITIONS = ('hold', 'beacon_drop', 'bottle_drop')
CLAMP_POSITIONS = ('unclamped', 'bottle_clamped', 'beacon_clamped')

FT_PER_M = 1.0 / 0.3048
ALTITUDE_WARNING_TOLERANCE_FT = 2.0


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
    pwm_range_us: tuple
    positions_us: dict   # position name -> pulse width


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
    plate: ServoConfig
    clamp: ServoConfig
    brake_phases: tuple

    @property
    def plate_hold_us(self):
        return self.plate.positions_us['hold']

    def plate_drop_us(self, payload):
        return self.plate.positions_us[f'{payload}_drop']

    @property
    def unclamped_us(self):
        return self.clamp.positions_us['unclamped']

    def clamped_us(self, payload):
        return self.clamp.positions_us[f'{payload}_clamped']

    @property
    def total_duration_s(self):
        return sum(phase.duration_s for phase in self.brake_phases)

    @property
    def drop_ft(self):
        return sum(phase.drop_ft for phase in self.brake_phases)

    def positions(self):
        """Every configured position as (servo, name, pulse_us)."""
        return [(servo, name, pulse)
                for servo in (self.plate, self.clamp)
                for name, pulse in servo.positions_us.items()]

    def actuator_value(self, servo, pulse_us):
        return us_to_actuator_value(pulse_us, servo.pwm_range_us)


def load_payload_config(mission_config):
    """Parse and validate the mission config's ``payload`` block.

    Returns None when the payload is disabled; nothing else in the block is read
    then, so a test flight works before the numbers are filled in.

    Raises ValueError for anything malformed, and for any position outside its
    servo's PWM range. PX4 would clamp such a position to the range's edge,
    which can look fine on the ground and never release in the air, so it
    stops the mission before takeoff instead.
    """
    if 'payload' not in mission_config:
        raise ValueError(
            "mission config has no 'payload' block. If the YAML file does "
            "contain one, rebuild (colcon build --packages-select bv_core): "
            "mission_node reads the installed copy of the config")
    block = mission_config['payload']
    if block is None:
        raise ValueError(
            "payload block is empty: indent the lines under 'payload:' "
            "(e.g. '  enabled: false')")
    if not isinstance(block, dict):
        raise ValueError(
            f"payload must be a mapping, got {block!r}: check that the lines "
            f"under 'payload:' are indented")

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
            "payload positions outside the flight controller's PWM range: "
            + "; ".join(problems)
            + ". Fix the position, or set PWM_MAIN_MINn/MAXn on the FC and "
            "pwm_range_us to match.")
    return config


def parse_payload_block(block):
    """Parse the payload block's values, ignoring enabled and reachability.

    For the bench tool, which must work while the payload is disabled or a
    position is still out of range.
    """
    block = _mapping(block, 'payload')
    plate = _servo('plate', block, PLATE_POSITIONS)
    clamp = _servo('clamp', block, CLAMP_POSITIONS)
    if plate.actuator_set == clamp.actuator_set:
        raise ValueError(
            "payload.plate.actuator_set and payload.clamp.actuator_set must "
            f"differ (both are {plate.actuator_set})")
    return PayloadConfig(
        plate=plate,
        clamp=clamp,
        brake_phases=_phases(block.get('brake_phases')),
    )


def unreachable_positions(config):
    """One message per position outside its servo's PWM range."""
    problems = []
    for servo, name, pulse in config.positions():
        problem = unreachable_reason(servo, pulse)
        if problem:
            problems.append(f"{servo.name} {name} {problem}")
    return problems


def unreachable_reason(servo, pulse_us):
    """Why a servo cannot emit this pulse, or None when it can."""
    low, high = servo.pwm_range_us
    if low <= pulse_us <= high:
        return None
    return f"{pulse_us:g} us is outside pwm_range_us [{low:g}, {high:g}]"


def altitude_mismatch_warning(config, altitude_m):
    """Warn when the brake phases were tuned for a different drop height."""
    altitude_ft = altitude_m * FT_PER_M
    if abs(config.drop_ft - altitude_ft) <= ALTITUDE_WARNING_TOLERANCE_FT:
        return None
    return (
        f"payload.brake_phases cover a {config.drop_ft:g} ft drop but delivery "
        f"altitude is {altitude_ft:.0f} ft: the clamp timing will not match "
        f"the payload's descent")


def actuator_params(config, plate_us=None, clamp_us=None):
    """MAV_CMD_DO_SET_ACTUATOR and its seven params for the given pulses.

    A servo left as None gets NaN, which PX4 treats as "leave unchanged", so
    commanding one servo never disturbs the other.
    """
    params = [math.nan] * 7
    for servo, pulse in ((config.plate, plate_us), (config.clamp, clamp_us)):
        if pulse is not None:
            params[servo.actuator_set - 1] = config.actuator_value(
                servo, pulse)
    # Actuator set index: param1..6 address sets 1..6. A float, because the
    # CommandLong fields are float32 and rosidl rejects an int.
    params[6] = 0.0
    return MAV_CMD_DO_SET_ACTUATOR, params


class DropSequence:
    """Servo pulses over one drop, as a function of elapsed time.

    At t=0 the plate moves to this payload's drop position and the clamp
    grips. The clamp then toggles between this payload's clamped position and
    unclamped at each phase's interval, restarting clamped at each phase
    boundary. When the last phase ends the clamp opens and the plate returns
    to hold, so the payload still aboard is gripped again for the flight to
    the next target.
    """

    def __init__(self, config, payload):
        if payload not in PAYLOADS:
            raise ValueError(f"unknown payload {payload!r}")
        self.config = config
        self.payload = payload
        self.plate_us = config.plate_drop_us(payload)
        self.clamped_us = config.clamped_us(payload)

    def positions_at(self, elapsed_s):
        """(plate_us, clamp_us, done) at elapsed_s into the drop."""
        phase_start = 0.0
        for phase in self.config.brake_phases:
            phase_end = phase_start + phase.duration_s
            if elapsed_s < phase_end:
                toggles = int(max(0.0, elapsed_s - phase_start)
                              // phase.toggle_s)
                clamp = (self.clamped_us if toggles % 2 == 0
                         else self.config.unclamped_us)
                return self.plate_us, clamp, False
            phase_start = phase_end
        return self.config.plate_hold_us, self.config.unclamped_us, True


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


def _servo(name, payload_block, position_names):
    where = f'payload.{name}'
    block = _mapping(payload_block.get(name), where)
    actuator_set = block.get('actuator_set')
    if (isinstance(actuator_set, bool) or not isinstance(actuator_set, int)
            or not 1 <= actuator_set <= NUM_ACTUATOR_SETS):
        raise ValueError(
            f"{where}.actuator_set must be 1-{NUM_ACTUATOR_SETS} "
            f"(PX4 'Peripheral via Actuator Set N'), got {actuator_set!r}")
    return ServoConfig(
        name=name,
        actuator_set=actuator_set,
        pwm_range_us=_pulse_range(
            block.get('pwm_range_us'), f'{where}.pwm_range_us'),
        positions_us={
            position: _number(block, f'{position}_us', where)
            for position in position_names
        },
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
