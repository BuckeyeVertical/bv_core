"""Tests for the payload release configuration and drop sequence."""

import math

import pytest

from bv_core.payload import (
    DropSequence,
    MAV_CMD_DO_SET_ACTUATOR,
    actuator_params,
    altitude_mismatch_warning,
    load_payload_config,
    parse_payload_block,
    payload_for_class,
    unreachable_positions,
    unreachable_reason,
    us_to_actuator_value,
)


def _block(**overrides):
    """A payload block with the values measured in QGC."""
    block = {
        'enabled': True,
        'plate': {
            'actuator_set': 1,
            'pwm_range_us': [800, 2200],
            'hold_us': 1685,
            'beacon_drop_us': 1360,
            'bottle_drop_us': 2050,
        },
        'clamp': {
            'actuator_set': 2,
            'pwm_range_us': [800, 2200],
            'unclamped_us': 1900,
            'bottle_clamped_us': 1577,
            'beacon_clamped_us': 1300,
        },
        'pre_drop_s': 1.0,
        'brake_phases': [
            {'drop_ft': 75, 'speed_ftps': 15.0, 'toggle_ms': 200},
            {'drop_ft': 50, 'speed_ftps': 13.3, 'toggle_ms': 150},
            {'drop_ft': 25, 'speed_ftps': 12.5, 'toggle_ms': 100},
        ],
    }
    block.update(overrides)
    return {'payload': block}


def _config(**overrides):
    return load_payload_config(_block(**overrides))


class TestConversion:
    def test_pulse_to_px4_actuator_value(self):
        assert us_to_actuator_value(800, (800, 2200)) == pytest.approx(-1.0)
        assert us_to_actuator_value(1500, (800, 2200)) == pytest.approx(0.0)
        assert us_to_actuator_value(2200, (800, 2200)) == pytest.approx(1.0)
        assert us_to_actuator_value(1685, (800, 2200)) == pytest.approx(
            2 * 885 / 1400 - 1)


class TestLoading:
    def test_positions_are_the_configured_pulses(self):
        cfg = _config()
        assert cfg.plate_hold_us == 1685
        assert cfg.plate_drop_us('beacon') == 1360
        assert cfg.plate_drop_us('bottle') == 2050
        assert cfg.unclamped_us == 1900
        assert cfg.clamped_us('bottle') == 1577
        assert cfg.clamped_us('beacon') == 1300

    def test_phase_durations_come_from_distance_over_speed(self):
        cfg = _config()
        durations = [phase.duration_s for phase in cfg.brake_phases]
        assert durations == pytest.approx([5.0, 50 / 13.3, 2.0])
        assert cfg.brake_duration_s == pytest.approx(10.759, abs=0.001)
        # The 1 s pre-brake comes on top; it doesn't shorten the braking.
        assert cfg.total_duration_s == pytest.approx(11.759, abs=0.001)
        assert cfg.drop_ft == pytest.approx(150)

    def test_pre_drop_is_required_and_not_negative(self):
        block = _block()
        del block['payload']['pre_drop_s']
        with pytest.raises(ValueError, match='pre_drop_s'):
            load_payload_config(block)
        with pytest.raises(ValueError, match='pre_drop_s'):
            _config(pre_drop_s=-1)
        assert _config(pre_drop_s=0).total_duration_s == pytest.approx(
            10.759, abs=0.001)

    def test_disabled_needs_nothing_else(self):
        assert load_payload_config({'payload': {'enabled': False}}) is None

    def test_missing_block_says_to_rebuild(self):
        with pytest.raises(ValueError, match='colcon build'):
            load_payload_config({})

    def test_unindented_block_is_explained(self):
        # "payload:" followed by unindented lines parses as an empty value.
        with pytest.raises(ValueError, match='indent'):
            load_payload_config({'payload': None, 'enabled': False})
        with pytest.raises(ValueError, match='indent'):
            load_payload_config({'payload': 'enabled: false'})

    def test_enabled_must_be_boolean(self):
        with pytest.raises(ValueError, match='enabled'):
            load_payload_config({'payload': {'enabled': 'yes'}})

    def test_every_position_is_required(self):
        block = _block()
        del block['payload']['clamp']['beacon_clamped_us']
        with pytest.raises(ValueError, match='beacon_clamped_us'):
            load_payload_config(block)

    def test_servos_need_distinct_actuator_sets(self):
        block = _block()
        block['payload']['clamp']['actuator_set'] = 1
        with pytest.raises(ValueError, match='actuator_set'):
            load_payload_config(block)

    def test_actuator_set_must_exist_on_px4(self):
        block = _block()
        block['payload']['plate']['actuator_set'] = 7
        with pytest.raises(ValueError, match='actuator_set'):
            load_payload_config(block)

    def test_brake_phases_required(self):
        with pytest.raises(ValueError, match='brake_phases'):
            _config(brake_phases=[])

    def test_phase_values_must_be_positive(self):
        with pytest.raises(ValueError, match='speed_ftps'):
            _config(brake_phases=[
                {'drop_ft': 75, 'speed_ftps': 0, 'toggle_ms': 200}])


class TestRangeCheck:
    def test_measured_values_fit_the_default_range(self):
        assert unreachable_positions(_config()) == []

    def test_unreachable_position_names_the_position(self):
        block = _block()
        block['payload']['plate']['bottle_drop_us'] = 2300
        with pytest.raises(ValueError) as error:
            load_payload_config(block)
        message = str(error.value)
        assert 'bottle_drop' in message and '2300' in message
        assert '2200' in message
        assert 'beacon_drop' not in message

    def test_bench_parse_skips_enabled_and_reachability(self):
        block = _block(enabled=False)['payload']
        block['plate']['bottle_drop_us'] = 2300
        cfg = parse_payload_block(block)
        assert cfg.plate_drop_us('bottle') == 2300
        problems = unreachable_positions(cfg)
        assert len(problems) == 1 and 'bottle_drop' in problems[0]

    def test_unreachable_reason_for_a_single_pulse(self):
        cfg = _config()
        assert unreachable_reason(cfg.plate, 1685) is None
        assert '2200' in unreachable_reason(cfg.plate, 2300)

    def test_range_must_be_ordered(self):
        block = _block()
        block['payload']['clamp']['pwm_range_us'] = [2000, 1000]
        with pytest.raises(ValueError, match='pwm_range_us'):
            load_payload_config(block)


class TestPayloadSelection:
    def test_person_gets_bottle_and_tent_gets_beacon(self):
        assert payload_for_class(0) == 'bottle'
        assert payload_for_class(1) == 'beacon'

    def test_unknown_class_has_no_payload(self):
        assert payload_for_class(-1) is None
        assert payload_for_class(2) is None
        assert payload_for_class(None) is None


class TestDropSequence:
    """pre_drop_s = 1.0: brake on the held plate first, then the drop."""

    @staticmethod
    def clamp_runs(seq, until_s, step_s=0.001):
        """Lengths (s) of each stretch where the clamp holds one position."""
        runs, last, since = [], None, 0.0
        steps = int(until_s / step_s)
        for i in range(steps):
            t = i * step_s
            clamp = seq.positions_at(t)[1]
            if last is not None and clamp != last:
                runs.append(t - since)
                since = t
            last = clamp
        return runs

    def test_pre_brake_clamps_while_the_plate_still_holds(self):
        seq = DropSequence(_config(), 'bottle')
        assert seq.positions_at(0.0) == (1685, 1577, False)

    def test_pre_brake_toggles_at_the_first_phase_rhythm(self):
        seq = DropSequence(_config(), 'bottle')
        assert seq.positions_at(0.19)[:2] == (1685, 1577)
        assert seq.positions_at(0.21)[:2] == (1685, 1900)
        assert seq.positions_at(0.41)[:2] == (1685, 1577)
        assert seq.positions_at(0.99)[0] == 1685

    def test_plate_drops_after_the_pre_brake(self):
        seq = DropSequence(_config(), 'bottle')
        assert seq.positions_at(0.99)[0] == 1685
        assert seq.positions_at(1.01)[0] == 2050

    def test_beacon_uses_its_own_drop_and_clamp(self):
        seq = DropSequence(_config(), 'beacon')
        assert seq.positions_at(0.0) == (1685, 1300, False)
        assert seq.positions_at(0.21)[1] == 1900
        assert seq.positions_at(1.01)[0] == 1360

    def test_rhythm_never_pauses_at_a_phase_change(self):
        # Each stretch lasts exactly one toggle interval of the phase it
        # starts in: 200 ms, then 150 ms, then 100 ms. Never two in a row,
        # which is what a pause at a phase change looked like.
        cfg = _config()
        runs = self.clamp_runs(DropSequence(cfg, 'bottle'),
                               cfg.total_duration_s - 0.01)
        assert max(runs) <= 0.2 + 0.002
        assert runs[:5] == pytest.approx([0.2] * 5, abs=0.002)
        assert any(run == pytest.approx(0.15, abs=0.002) for run in runs)
        assert runs[-3:] == pytest.approx([0.1] * 3, abs=0.002)

    def test_braking_phases_keep_their_full_length(self):
        cfg = _config()
        seq = DropSequence(cfg, 'bottle')
        # The plate is at the drop position for the full 10.76 s of braking.
        assert seq.positions_at(1.01)[0] == 2050
        assert seq.positions_at(cfg.total_duration_s - 0.01)[0] == 2050
        assert cfg.total_duration_s - cfg.pre_drop_s == pytest.approx(
            10.759, abs=0.001)

    def test_ends_unclamped_with_plate_back_at_hold(self):
        cfg = _config()
        seq = DropSequence(cfg, 'bottle')
        plate, _, done = seq.positions_at(cfg.total_duration_s - 0.001)
        assert (plate, done) == (2050, False)
        assert seq.positions_at(cfg.total_duration_s) == (1685, 1900, True)
        assert seq.positions_at(60.0) == (1685, 1900, True)

    def test_no_pre_brake_drops_at_once(self):
        seq = DropSequence(_config(pre_drop_s=0), 'bottle')
        assert seq.positions_at(0.0) == (2050, 1577, False)

    def test_unknown_payload_rejected(self):
        with pytest.raises(ValueError):
            DropSequence(_config(), 'anvil')


class TestCommand:
    def test_only_the_named_actuator_sets_are_touched(self):
        cfg = _config()
        command, params = actuator_params(cfg, plate_us=1685)
        assert command == MAV_CMD_DO_SET_ACTUATOR
        assert len(params) == 7
        assert params[0] == pytest.approx(us_to_actuator_value(1685, (800, 2200)))
        assert all(math.isnan(p) for p in params[1:6])
        assert params[6] == 0  # actuator set index
        # CommandLong fields are float32; rosidl rejects an int.
        assert all(isinstance(p, float) for p in params)

    def test_both_servos_in_one_command(self):
        cfg = _config()
        _, params = actuator_params(cfg, plate_us=2050, clamp_us=1577)
        assert params[0] == pytest.approx(us_to_actuator_value(2050, (800, 2200)))
        assert params[1] == pytest.approx(us_to_actuator_value(1577, (800, 2200)))


class TestAltitudeWarning:
    def test_matching_altitude_is_quiet(self):
        # 45.72 m is exactly 150 ft, the phases' total drop.
        assert altitude_mismatch_warning(_config(), 45.72) is None

    def test_mismatched_altitude_explains_itself(self):
        warning = altitude_mismatch_warning(_config(), 60.96)
        assert warning is not None
        assert '150' in warning and '200' in warning
