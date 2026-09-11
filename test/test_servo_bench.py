"""Argument handling and drop schedule for the test_servo bench tool."""

import pytest

from bv_core.payload import parse_payload_block
from bv_core.test_servo import drop_schedule, parse_args

from test_payload import _block


class TestParseArgs:
    def test_single_servo(self):
        assert parse_args(['slider', '130']) == ('servo', 'slider', 130.0)
        assert parse_args(['brake', '107.5']) == ('servo', 'brake', 107.5)

    def test_rest_and_drop(self):
        assert parse_args(['rest']) == ('rest',)
        assert parse_args(['drop', 'bottle']) == ('drop', 'bottle')
        assert parse_args(['drop', 'beacon']) == ('drop', 'beacon')

    @pytest.mark.parametrize('argv', [
        [], ['slider'], ['slider', 'abc'], ['elbow', '90'],
        ['drop', 'anvil'], ['rest', 'now'],
    ])
    def test_bad_arguments_print_usage(self, argv):
        with pytest.raises(SystemExit, match='usage'):
            parse_args(argv)


class TestDropSchedule:
    def test_matches_the_mission_sequence(self):
        config = parse_payload_block(_block()['payload'])
        schedule = drop_schedule(config, 'bottle')
        assert schedule[0] == (0.0, 170, 107)
        assert schedule[1][1:] == (170, 125)
        assert schedule[1][0] == pytest.approx(0.2)
        # Ends with the brake at rest, sent no later than the sequence end.
        assert schedule[-1][1:] == (170, 125)
        assert schedule[-1][0] <= config.total_duration_s + 0.02
        # Only changes are listed.
        pairs = [entry[1:] for entry in schedule]
        assert all(a != b for a, b in zip(pairs, pairs[1:]))
