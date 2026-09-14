"""Argument handling and drop schedule for the test_servo bench tool."""

import pytest

from bv_core.payload import parse_payload_block
from bv_core.test_servo import describe, parse_args

from test_payload import _block


class TestParseArgs:
    def test_single_servo(self):
        assert parse_args(['plate', '1685']) == ('servo', 'plate', 1685.0)
        assert parse_args(['clamp', '1577.5']) == ('servo', 'clamp', 1577.5)

    def test_rest_and_drop(self):
        assert parse_args(['show']) == ('show',)
        assert parse_args(['rest']) == ('rest',)
        assert parse_args(['drop', 'bottle']) == ('drop', 'bottle', None, None)
        assert parse_args(['drop', 'beacon']) == ('drop', 'beacon', None, None)

    def test_drop_with_phases_from_the_command_line(self):
        action = parse_args(['drop', 'bottle', '150:3'])
        assert action[:2] == ('drop', 'bottle') and action[3] is None
        assert [(p.toggle_ms, p.duration_s) for p in action[2]] == [(150, 3.0)]

    def test_drop_with_clamped_pulse_from_the_command_line(self):
        assert parse_args(['drop', 'beacon', '2050']) == (
            'drop', 'beacon', None, 2050.0)
        action = parse_args(['drop', 'beacon', '2050', '200:10', '100:2'])
        assert action[:2] == ('drop', 'beacon') and action[3] == 2050.0
        assert [(p.toggle_ms, p.duration_s) for p in action[2]] == [
            (200, 10.0), (100, 2.0)]

    @pytest.mark.parametrize('argv', [
        ['drop', 'bottle', 'abc'],             # not a pulse or a phase
        ['drop', 'bottle', '2050', '1900'],    # second pulse
        ['drop', 'bottle', '150:3', '2050'],   # pulse after the phases
        ['drop', 'bottle', '150:x'],
    ])
    def test_bad_drop_arguments_print_usage(self, argv):
        with pytest.raises(SystemExit, match='usage'):
            parse_args(argv)

    @pytest.mark.parametrize('argv', [
        [], ['plate'], ['plate', 'abc'], ['slider', '1500'],
        ['drop', 'anvil'], ['rest', 'now'],
    ])
    def test_bad_arguments_print_usage(self, argv):
        with pytest.raises(SystemExit, match='usage'):
            parse_args(argv)


class TestShow:
    def test_lists_positions_and_disarmed_values(self):
        text = '\n'.join(describe(parse_payload_block(_block()['payload'])))
        assert 'plate hold: 1685 us [ok]' in text
        assert 'plate bottle_drop: 2050 us [ok]' in text
        assert 'clamp beacon_clamped: 1300 us [ok]' in text
        # Disarmed pulses to enter on the FC: plate hold, clamp open.
        assert 'PWM_MAIN_DIS (plate): 1685' in text
        assert 'PWM_MAIN_DIS (clamp): 1900' in text
        assert 'OUT OF RANGE' not in text
        # Each payload's brake phases and total drop time.
        assert 'bottle brake: 200 ms x 5 s, 150 ms x 3.76 s, 100 ms x 2 s' in text
        assert 'beacon brake:' in text and '11.76 s total' in text

    def test_flags_unreachable_positions(self):
        block = _block()['payload']
        block['plate']['bottle_drop_us'] = 2300
        lines = describe(parse_payload_block(block))
        flagged = [line for line in lines if 'OUT OF RANGE' in line]
        assert len(flagged) == 1 and 'bottle_drop' in flagged[0]
