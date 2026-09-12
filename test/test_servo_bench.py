"""Argument handling and drop schedule for the test_servo bench tool."""

import pytest

from bv_core.payload import parse_payload_block
from bv_core.test_servo import describe, drop_schedule, parse_args

from test_payload import _block


class TestParseArgs:
    def test_single_servo(self):
        assert parse_args(['plate', '1685']) == ('servo', 'plate', 1685.0)
        assert parse_args(['clamp', '1577.5']) == ('servo', 'clamp', 1577.5)

    def test_rest_and_drop(self):
        assert parse_args(['show']) == ('show',)
        assert parse_args(['rest']) == ('rest',)
        assert parse_args(['drop', 'bottle']) == ('drop', 'bottle')
        assert parse_args(['drop', 'beacon']) == ('drop', 'beacon')

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

    def test_flags_unreachable_positions(self):
        block = _block()['payload']
        block['plate']['bottle_drop_us'] = 2300
        lines = describe(parse_payload_block(block))
        flagged = [line for line in lines if 'OUT OF RANGE' in line]
        assert len(flagged) == 1 and 'bottle_drop' in flagged[0]


class TestDropSchedule:
    def test_matches_the_mission_sequence(self):
        config = parse_payload_block(_block()['payload'])
        schedule = drop_schedule(config, 'bottle')
        assert schedule[0] == (0.0, 2050, 1577)
        assert schedule[1][1:] == (2050, 1900)
        assert schedule[1][0] == pytest.approx(0.2)
        # Ends unclamped, sent no later than the sequence end.
        assert schedule[-1][1:] == (2050, 1900)
        assert schedule[-1][0] <= config.total_duration_s + 0.02
        # Only changes are listed.
        pairs = [entry[1:] for entry in schedule]
        assert all(a != b for a, b in zip(pairs, pairs[1:]))
