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
        assert cfg.total_duration_s == pytest.approx(10.759, abs=0.001)
        assert cfg.drop_ft == pytest.approx(150)

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
    def test_starts_with_drop_and_clamp(self):
        seq = DropSequence(_config(), 'bottle')
        assert seq.positions_at(0.0) == (2050, 1577, False)

    def test_beacon_uses_its_own_drop_and_clamp(self):
        seq = DropSequence(_config(), 'beacon')
        assert seq.positions_at(0.0) == (1360, 1300, False)
        assert seq.positions_at(0.21)[1] == 1900

    def test_phase_one_toggles_every_200ms(self):
        seq = DropSequence(_config(), 'bottle')
        assert seq.positions_at(0.19)[1] == 1577
        assert seq.positions_at(0.21)[1] == 1900
        assert seq.positions_at(0.41)[1] == 1577

    def test_phase_two_toggles_every_150ms_and_starts_clamped(self):
        seq = DropSequence(_config(), 'bottle')
        assert seq.positions_at(5.01)[1] == 1577
        assert seq.positions_at(5.16)[1] == 1900
        assert seq.positions_at(5.31)[1] == 1577

    def test_phase_three_toggles_every_100ms(self):
        seq = DropSequence(_config(), 'bottle')
        start = 5.0 + 50 / 13.3
        assert seq.positions_at(start + 0.01)[1] == 1577
        assert seq.positions_at(start + 0.11)[1] == 1900
        assert seq.positions_at(start + 0.21)[1] == 1577

    def test_ends_unclamped_with_plate_back_at_hold(self):
        cfg = _config()
        seq = DropSequence(cfg, 'bottle')
        # The plate holds the drop position for the whole brake sequence...
        plate, _, done = seq.positions_at(cfg.total_duration_s - 0.001)
        assert (plate, done) == (2050, False)
        # ...then returns to hold, so the other payload is gripped again.
        assert seq.positions_at(cfg.total_duration_s) == (1685, 1900, True)
        assert seq.positions_at(60.0) == (1685, 1900, True)

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
