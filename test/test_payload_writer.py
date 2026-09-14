"""PayloadWriter: the shared code that sends payload positions to PX4."""

import math
from unittest.mock import Mock

from bv_core.payload import MAV_CMD_DO_SET_ACTUATOR, load_payload_config
from bv_core.payload_writer import RETRY_SEC, PayloadWriter, actuator_request

from fake_mavros import Clock, FakeCommandClient, sent


def config():
    return load_payload_config({'payload': {
        'enabled': True,
        'plate': {'actuator_set': 2, 'pwm_range_us': [800, 2200],
                  'hold_us': 1685, 'beacon_drop_us': 1360,
                  'bottle_drop_us': 2050},
        'clamp': {'actuator_set': 1, 'pwm_range_us': [800, 2200],
                  'unclamped_us': 1900, 'bottle_clamped_us': 1577,
                  'beacon_clamped_us': 1710},
        'pre_drop_s': 1.0,
        'brake_phases': {
            'bottle': [{'duration_s': 3.0, 'toggle_ms': 200}],
            'beacon': [{'duration_s': 3.0, 'toggle_ms': 200}],
        },
    }})


def writer(clock=None):
    client = FakeCommandClient()
    return PayloadWriter(config(), client, Mock(), clock=clock or Clock()), client


def test_request_is_addressed_do_set_actuator():
    request = actuator_request(config(), plate_us=1685, clamp_us=1900)
    assert request.command == MAV_CMD_DO_SET_ACTUATOR
    # PX4 on the flight controller ignores the broadcast form.
    assert request.broadcast is False
    assert request.param7 == 0
    assert all(math.isnan(p) for p in (
        request.param3, request.param4, request.param5, request.param6))


def test_sends_and_records_the_confirmation():
    w, client = writer()
    w.set(1685, 1900)
    assert sent(client) == [(1685, 1900)]
    assert w.confirmed == (1685, 1900) and w.settled
    assert (w.sent, w.accepted) == (1, 1)


def test_confirmed_positions_are_not_resent():
    w, client = writer()
    w.set(1685, 1900)
    w.flush()
    w.set(1685, 1900)
    assert len(sent(client)) == 1


def test_one_command_in_flight_then_the_latest_positions():
    w, client = writer()
    client.auto_reply = None                  # PX4 has not answered yet
    w.set(2050, 1577)
    w.set(2050, 1900)
    w.set(2050, 1577)
    w.set(2050, 1900)
    assert sent(client) == [(2050, 1577)]     # MAVROS allows only one
    assert not w.settled
    # The reply arrives: the writer sends what is wanted NOW, skipping the
    # positions that went stale in between.
    client.futures[0].finish(success=True)
    assert sent(client) == [(2050, 1577), (2050, 1900)]


def test_unconfirmed_command_is_resent_after_a_pause():
    clock = Clock()
    w, client = writer(clock)
    client.auto_reply = {'success': False, 'result': 3}
    w.set(1685, 1900)
    assert len(sent(client)) == 1
    w.flush()                                 # too soon: no resend
    assert len(sent(client)) == 1
    clock.now += RETRY_SEC + 0.01
    client.auto_reply = {'success': True}
    w.flush()
    assert sent(client) == [(1685, 1900), (1685, 1900)]
    assert w.settled and w.failures == 0
    w.logger.warn.assert_called_once()


def test_reply_exception_is_logged_not_raised():
    w, client = writer()
    client.auto_reply = {'error': RuntimeError('link down')}
    w.set(1685, 1900)
    assert not w.pending
    assert w.confirmed is None
    w.logger.warn.assert_called_once()


def test_nothing_is_sent_before_anything_is_wanted():
    w, client = writer()
    w.flush()
    assert sent(client) == [] and w.settled


def test_stats_record_reply_time_and_how_late_each_send_was():
    clock = Clock()
    w, client = writer(clock)
    client.auto_reply = None
    w.set(2050, 1577)                         # wanted and sent at t=0
    clock.now += 0.012
    client.futures[0].finish(success=True)    # PX4 replies after 12 ms
    assert abs(w.max_reply_s - 0.012) < 1e-9
    assert abs(w.max_late_s) < 1e-9           # sent the moment it was wanted


def test_stats_count_positions_skipped_while_waiting_for_px4():
    clock = Clock()
    w, client = writer(clock)
    client.auto_reply = None
    w.set(2050, 1577)                         # in flight
    clock.now += 0.2
    w.set(2050, 1900)                         # waits for the reply...
    clock.now += 0.2
    w.set(1685, 1900)                         # ...replaced before it went out
    assert w.skipped == 1
    clock.now += 0.05
    client.futures[0].finish(success=True)    # now sends the latest
    assert sent(client) == [(2050, 1577), (1685, 1900)]
    # The latest was wanted 0.05 s before the reply let it go out.
    assert abs(w.max_late_s - 0.05) < 1e-9


def test_nothing_resent_when_px4_already_holds_the_latest():
    w, client = writer()
    client.auto_reply = None
    w.set(2050, 1577)                         # in flight
    w.set(2050, 1900)                         # skipped...
    w.set(2050, 1577)                         # ...and back where PX4 is going
    client.futures[0].finish(success=True)
    assert sent(client) == [(2050, 1577)] and w.settled
    assert w.skipped == 1


def test_command_log_lines_when_asked():
    clock = Clock()
    client = FakeCommandClient()
    logger = Mock()
    w = PayloadWriter(config(), client, logger, clock=clock, log_commands=True)
    w.set(2050, 1577)
    lines = [call.args[0] for call in logger.info.call_args_list]
    assert any('send #1' in line and '2050' in line and '1577' in line
               for line in lines)
    assert any('confirmed #1' in line for line in lines)


def test_reset_stats_starts_a_new_count():
    w, client = writer()
    w.set(1685, 1900)
    w.reset_stats()
    assert (w.sent, w.accepted, w.skipped, w.max_reply_s, w.max_late_s) == (
        0, 0, 0, 0.0, 0.0)
    assert 'sent 0' in w.summary()
