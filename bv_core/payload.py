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

import bisect
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
    duration_s: float   # 0 skips the phase
    toggle_ms: float

    @property
    def toggle_s(self):
        return self.toggle_ms / 1000.0


@dataclass(frozen=True)
class PayloadConfig:
    plate: ServoConfig
    clamp: ServoConfig
    pre_drop_s: float   # clamp brakes this long before the plate moves
    brake_phases: dict  # payload -> tuple of BrakePhase

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

    def brake_duration_s(self, payload):
        """This payload's braking phases, after the plate moves."""
        return sum(phase.duration_s for phase in self.brake_phases[payload])

    def total_duration_s(self, payload):
        """This payload's whole drop: pre-brake plus its braking phases."""
        return self.pre_drop_s + self.brake_duration_s(payload)

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
    pre_drop_s = _number(block, 'pre_drop_s', 'payload')
    if pre_drop_s < 0.0:
        raise ValueError(
            f"payload.pre_drop_s must not be negative, got {pre_drop_s:g}")
    return PayloadConfig(
        plate=plate,
        clamp=clamp,
        pre_drop_s=pre_drop_s,
        brake_phases=_payload_phases(block.get('brake_phases')),
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

    For the first pre_drop_s the plate stays at hold while the clamp already
    brakes. Then the plate moves to this payload's drop position for the
    braking phases, which run in full; a phase lasting 0 s is skipped. When
    the last phase ends the clamp opens and the plate returns to hold, so the
    payload still aboard is gripped again for the flight to the next target.

    The clamp starts clamped and flips between this payload's clamped
    position and unclamped in one continuous rhythm: every flip comes one
    interval after the previous one, the interval being that of the phase
    (the pre-brake uses the first phase that runs) in which it began. Restarting a
    phase on "clamped" instead held the clamp for two intervals whenever the
    previous phase also ended clamped, a visible pause at each phase change.
    """

    def __init__(self, config, payload):
        if payload not in PAYLOADS:
            raise ValueError(f"unknown payload {payload!r}")
        self.config = config
        self.payload = payload
        self.plate_us = config.plate_drop_us(payload)
        self.clamped_us = config.clamped_us(payload)
        self.phases = config.brake_phases[payload]
        self.total_s = config.total_duration_s(payload)
        self._flip_times = self._compute_flip_times()

    def _compute_flip_times(self):
        """Elapsed times at which the clamp changes position."""
        phases = [p for p in self.phases if p.duration_s > 0.0]
        # (start_s, toggle_s) per segment, the pre-brake first.
        segments = []
        start = 0.0
        if self.config.pre_drop_s > 0.0:
            segments.append((0.0, phases[0].toggle_s))
            start = self.config.pre_drop_s
        for phase in phases:
            segments.append((start, phase.toggle_s))
            start += phase.duration_s
        end = self.total_s

        flips = []
        t = 0.0
        while True:
            toggle_s = next(toggle for seg_start, toggle in reversed(segments)
                            if seg_start <= t)
            t += toggle_s
            if t >= end:
                return flips
            flips.append(t)

    def positions_at(self, elapsed_s):
        """(plate_us, clamp_us, done) at elapsed_s into the drop."""
        if elapsed_s >= self.total_s:
            return self.config.plate_hold_us, self.config.unclamped_us, True
        plate = (self.config.plate_hold_us
                 if elapsed_s < self.config.pre_drop_s else self.plate_us)
        flips = bisect.bisect_right(self._flip_times, elapsed_s)
        clamp = (self.clamped_us if flips % 2 == 0
                 else self.config.unclamped_us)
        return plate, clamp, False


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


def _payload_phases(value):
    """{payload: phases} from brake_phases: one list per payload."""
    if not isinstance(value, dict):
        raise ValueError(
            "payload.brake_phases must give each payload its own list: "
            "brake_phases: {bottle: [...], beacon: [...]}")
    missing = [payload for payload in PAYLOADS if payload not in value]
    if missing:
        raise ValueError(
            f"payload.brake_phases has no list for {', '.join(missing)} "
            f"(needs bottle and beacon)")
    return {payload: parse_brake_phases(
                value[payload], where=f'payload.brake_phases.{payload}')
            for payload in PAYLOADS}


def parse_brake_phases(value, where='brake phases'):
    """One payload's phases, from its YAML list or 'toggle_ms:duration_s' text.

    duration_s of 0 skips a phase; toggle_ms must be positive; together the
    phases must last more than 0 s, or the plate would move out and straight
    back before anything fell.
    """
    if isinstance(value, list) and value and all(
            isinstance(item, str) for item in value):
        value = [_phase_from_text(item) for item in value]
    if not isinstance(value, list) or not value:
        raise ValueError(f"{where} must be a non-empty list")
    phases = []
    for index, raw in enumerate(value):
        item = f'{where}[{index}]'
        raw = _mapping(raw, item)
        if 'duration_s' not in raw and 'drop_ft' in raw:
            raise ValueError(
                f"{item} uses drop_ft/speed_ftps; phases are now "
                f"{{duration_s: <seconds>, toggle_ms: <ms>}} (duration_s = "
                f"drop_ft / speed_ftps)")
        duration_s = _number(raw, 'duration_s', item)
        toggle_ms = _number(raw, 'toggle_ms', item)
        if duration_s < 0.0:
            raise ValueError(
                f"{item}.duration_s must not be negative, got {duration_s:g}")
        if toggle_ms <= 0.0:
            raise ValueError(
                f"{item}.toggle_ms must be positive, got {toggle_ms:g}")
        phases.append(BrakePhase(duration_s=duration_s, toggle_ms=toggle_ms))
    if sum(phase.duration_s for phase in phases) <= 0.0:
        raise ValueError(
            f"{where} must last more than 0 s in total: with every phase "
            f"skipped the plate would move out and straight back")
    return tuple(phases)


def _phase_from_text(text):
    """'150:3' -> {'toggle_ms': 150.0, 'duration_s': 3.0}."""
    try:
        toggle, duration = text.split(':')
        return {'toggle_ms': float(toggle), 'duration_s': float(duration)}
    except ValueError:
        raise ValueError(
            f"brake phase {text!r} must be toggle_ms:duration_s, e.g. 150:3"
        ) from None
