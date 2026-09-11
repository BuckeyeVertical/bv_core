"""Tests for the payload release configuration and drop sequence."""

import math

import pytest

from bv_core.payload import (
    DropSequence,
    MAV_CMD_DO_SET_ACTUATOR,
    actuator_params,
    altitude_mismatch_warning,
    degrees_to_us,
    load_payload_config,
    parse_payload_block,
    payload_for_class,
    unreachable_positions,
    unreachable_reason,
    us_to_actuator_value,
)


def _block(**overrides):
    """A payload block matching the tested Arduino values."""
    block = {
        'enabled': True,
        'arduino_pulse_range_us': [544, 2400],
        'slider': {
            'actuator_set': 1,
            'zero_deg': 100,
            'beacon_offset_deg': 0,
            'hold_offset_deg': 30,
            'bottle_offset_deg': 70,
            'pwm_range_us': [544, 2400],
        },
        'brake': {
            'actuator_set': 2,
            'zero_deg': 107,
            'pulse_offset_deg': 0,
            'rest_offset_deg': 18,
            'pwm_range_us': [544, 2400],
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
    def test_arduino_servo_library_mapping(self):
        # Servo.write(deg) on an AVR board: 0-180 deg -> 544-2400 us.
        assert degrees_to_us(0, (544, 2400)) == pytest.approx(544)
        assert degrees_to_us(180, (544, 2400)) == pytest.approx(2400)
        assert degrees_to_us(130, (544, 2400)) == pytest.approx(1884.4, abs=0.1)
        assert degrees_to_us(170, (544, 2400)) == pytest.approx(2296.9, abs=0.1)

    def test_pulse_to_px4_actuator_value(self):
        assert us_to_actuator_value(1000, (1000, 2000)) == pytest.approx(-1.0)
        assert us_to_actuator_value(1500, (1000, 2000)) == pytest.approx(0.0)
        assert us_to_actuator_value(2000, (1000, 2000)) == pytest.approx(1.0)

    def test_matching_ranges_reduce_to_degrees_over_ninety(self):
        # With the FC range equal to the Arduino range, value = deg / 90 - 1.
        for deg in (100, 107, 125, 130, 170):
            us = degrees_to_us(deg, (544, 2400))
            assert us_to_actuator_value(us, (544, 2400)) == pytest.approx(
                deg / 90.0 - 1.0)


class TestLoading:
    def test_positions_are_zero_plus_offset(self):
        cfg = _config()
        assert cfg.slider_hold_deg == 130
        assert cfg.release_deg('beacon') == 100
        assert cfg.release_deg('bottle') == 170
        assert cfg.brake_rest_deg == 125
        assert cfg.brake_pulse_deg == 107

    def test_rezero_moves_every_position_of_that_servo(self):
        # Re-mounting the horn so the beacon release sits at 58 deg keeps the
        # 0/30/70 spacing and brings the bottle release inside PX4's range.
        block = _block()
        block['payload']['slider']['zero_deg'] = 58
        block['payload']['slider']['pwm_range_us'] = [800, 2200]
        cfg = load_payload_config(block)
        assert cfg.release_deg('beacon') == 58
        assert cfg.slider_hold_deg == 88
        assert cfg.release_deg('bottle') == 128
        # The brake keeps its own zero.
        assert cfg.brake_rest_deg == 125
        assert cfg.brake_pulse_deg == 107

    def test_old_schema_names_the_missing_offset(self):
        block = _block()
        del block['payload']['slider']['hold_offset_deg']
        with pytest.raises(ValueError, match='hold_offset_deg'):
            load_payload_config(block)

    def test_phase_durations_come_from_distance_over_speed(self):
        cfg = _config()
        durations = [phase.duration_s for phase in cfg.brake_phases]
        assert durations == pytest.approx([5.0, 50 / 13.3, 2.0])
        assert cfg.total_duration_s == pytest.approx(10.759, abs=0.001)
        assert cfg.drop_ft == pytest.approx(150)

    def test_disabled_needs_nothing_else(self):
        assert load_payload_config({'payload': {'enabled': False}}) is None

    def test_missing_block_is_an_error(self):
        with pytest.raises(ValueError, match='payload'):
            load_payload_config({})

    def test_enabled_must_be_boolean(self):
        with pytest.raises(ValueError, match='enabled'):
            load_payload_config({'payload': {'enabled': 'yes'}})

    def test_servos_need_distinct_actuator_sets(self):
        block = _block()
        block['payload']['brake']['actuator_set'] = 1
        with pytest.raises(ValueError, match='actuator_set'):
            load_payload_config(block)

    def test_actuator_set_must_exist_on_px4(self):
        block = _block()
        block['payload']['slider']['actuator_set'] = 7
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
    def test_positions_inside_fc_range_pass(self):
        _config()  # Arduino-equal range: everything reachable

    def test_unreachable_position_names_the_position(self):
        block = _block()
        block['payload']['slider']['pwm_range_us'] = [800, 2200]
        with pytest.raises(ValueError) as error:
            load_payload_config(block)
        message = str(error.value)
        assert 'bottle' in message
        assert '170' in message
        assert '2200' in message
        # Reachable positions are not reported.
        assert 'beacon' not in message

    def test_bench_parse_skips_enabled_and_reachability(self):
        block = _block(enabled=False)['payload']
        block['slider']['pwm_range_us'] = [800, 2200]
        cfg = parse_payload_block(block)
        assert cfg.release_deg('bottle') == 170
        problems = unreachable_positions(cfg)
        assert len(problems) == 1 and 'bottle' in problems[0]

    def test_unreachable_reason_for_a_single_angle(self):
        cfg = _config()
        assert unreachable_reason(cfg, cfg.slider, 130) is None
        block = _block()['payload']
        block['slider']['pwm_range_us'] = [800, 2200]
        narrow = parse_payload_block(block)
        assert '2200' in unreachable_reason(narrow, narrow.slider, 170)

    def test_range_must_be_ordered(self):
        block = _block()
        block['payload']['brake']['pwm_range_us'] = [2000, 1000]
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
    def test_starts_with_release_and_brake_pulse(self):
        seq = DropSequence(_config(), 'bottle')
        assert seq.positions_at(0.0) == (170, 107, False)

    def test_beacon_uses_its_own_release(self):
        seq = DropSequence(_config(), 'beacon')
        assert seq.positions_at(0.0)[0] == 100

    def test_phase_one_toggles_every_200ms(self):
        seq = DropSequence(_config(), 'bottle')
        assert seq.positions_at(0.19)[1] == 107
        assert seq.positions_at(0.21)[1] == 125
        assert seq.positions_at(0.41)[1] == 107

    def test_phase_two_toggles_every_150ms_and_starts_on_pulse(self):
        seq = DropSequence(_config(), 'bottle')
        assert seq.positions_at(5.01)[1] == 107
        assert seq.positions_at(5.16)[1] == 125
        assert seq.positions_at(5.31)[1] == 107

    def test_phase_three_toggles_every_100ms(self):
        seq = DropSequence(_config(), 'bottle')
        start = 5.0 + 50 / 13.3
        assert seq.positions_at(start + 0.01)[1] == 107
        assert seq.positions_at(start + 0.11)[1] == 125
        assert seq.positions_at(start + 0.21)[1] == 107

    def test_ends_with_brake_at_rest_and_slider_left_released(self):
        cfg = _config()
        seq = DropSequence(cfg, 'bottle')
        assert seq.positions_at(cfg.total_duration_s - 0.001)[2] is False
        assert seq.positions_at(cfg.total_duration_s) == (170, 125, True)
        assert seq.positions_at(60.0) == (170, 125, True)

    def test_unknown_payload_rejected(self):
        with pytest.raises(ValueError):
            DropSequence(_config(), 'anvil')


class TestCommand:
    def test_only_the_named_actuator_sets_are_touched(self):
        cfg = _config()
        command, params = actuator_params(cfg, slider_deg=130)
        assert command == MAV_CMD_DO_SET_ACTUATOR
        assert len(params) == 7
        assert params[0] == pytest.approx(130 / 90 - 1)
        assert all(math.isnan(p) for p in params[1:6])
        assert params[6] == 0  # actuator set index
        # CommandLong fields are float32; rosidl rejects an int.
        assert all(isinstance(p, float) for p in params)

    def test_both_servos_in_one_command(self):
        cfg = _config()
        _, params = actuator_params(cfg, slider_deg=170, brake_deg=107)
        assert params[0] == pytest.approx(170 / 90 - 1)
        assert params[1] == pytest.approx(107 / 90 - 1)


class TestAltitudeWarning:
    def test_matching_altitude_is_quiet(self):
        # 45.72 m is exactly 150 ft, the phases' total drop.
        assert altitude_mismatch_warning(_config(), 45.72) is None

    def test_mismatched_altitude_explains_itself(self):
        warning = altitude_mismatch_warning(_config(), 60.96)
        assert warning is not None
        assert '150' in warning and '200' in warning
