"""mission_node's DEPLOY state driving the payload through PX4.

Runs the real MissionRunner methods on a stand-in object, like the other
mission tests: MissionRunner.__init__ needs MAVROS and the installed config.
The clock is patched so the drop sequence advances deterministically, and a
fake MAVROS client decides when (and whether) PX4 confirms each command.
"""

import math
import unittest
from types import MethodType, SimpleNamespace
from unittest.mock import Mock, patch

from bv_core import mission as mission_module
from bv_core.mission import MissionRunner, PAYLOAD_RETRY_SEC, STATE_DEPLOY
from bv_core.payload import MAV_CMD_DO_SET_ACTUATOR, load_payload_config


def payload_config():
    return load_payload_config({'payload': {
        'enabled': True,
        'plate': {'actuator_set': 2, 'pwm_range_us': [800, 2200],
                  'hold_us': 1685, 'beacon_drop_us': 1360,
                  'bottle_drop_us': 2050},
        'clamp': {'actuator_set': 1, 'pwm_range_us': [800, 2200],
                  'unclamped_us': 1900, 'bottle_clamped_us': 1577,
                  'beacon_clamped_us': 1300},
        'pre_drop_s': 1.0,
        'brake_phases': [
            {'duration_s': 5.0, 'toggle_ms': 200},
            {'duration_s': 3.76, 'toggle_ms': 150},
            {'duration_s': 2.0, 'toggle_ms': 100},
        ],
    }})


class FakeFuture:
    def __init__(self, client):
        self.client = client
        self.callback = None
        self.response = None
        self.error = None

    def add_done_callback(self, callback):
        self.callback = callback
        if self.client.auto_reply is not None:
            self.finish(**self.client.auto_reply)

    def finish(self, success=True, result=0, error=None):
        self.response = SimpleNamespace(success=success, result=result)
        self.error = error
        self.callback(self)

    def result(self):
        if self.error is not None:
            raise self.error
        return self.response


class FakeCommandClient:
    """MAVROS /mavros/cmd/command. Replies at once unless told otherwise."""

    def __init__(self):
        self.requests = []
        self.futures = []
        self.auto_reply = {'success': True}

    def call_async(self, request):
        self.requests.append(request)
        future = FakeFuture(self)
        self.futures.append(future)
        return future


def mission(payload=None, class_id=0, state='deliver'):
    node = SimpleNamespace(
        current_state=state, is_transitioning=True, payload=payload,
        current_target_class_id=class_id,
        objects_delivered_count=0, num_objects_to_find=2,
        command_client=FakeCommandClient(), log=Mock(),
        get_logger=Mock(return_value=Mock()), publish_mission_state=Mock(),
        create_timer=Mock(side_effect=lambda *_args: Mock()),
        destroy_timer=Mock(), on_deploy_complete=Mock(),
        approval_gate=None, set_velocity=Mock(), set_flight_mode=Mock(),
        rtl_velocity=5.0,
        _drop_sequence=None, _deploy_timer=None,
        _payload_desired=None, _payload_confirmed=None,
        _payload_pending=False, _payload_retry_at=0.0, _payload_failures=0,
    )
    for name in ('enter_deploy_state', '_on_deploy_tick', '_stop_drop',
                 'set_payload_positions', 'send_payload_rest_positions',
                 '_flush_payload', '_on_payload_reply', 'enter_rtl_state'):
        setattr(node, name, MethodType(getattr(MissionRunner, name), node))
    return node


def decode(value, low=800, high=2200):
    return None if math.isnan(value) else round(
        low + (value + 1.0) / 2.0 * (high - low))


def sent(node):
    """(plate_us, clamp_us) per command. Clamp is set 1, plate set 2."""
    return [(decode(r.param2), decode(r.param1))
            for r in node.command_client.requests]


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


class DeployTestCase(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        patcher = patch.object(mission_module.time, 'monotonic', self.clock)
        patcher.start()
        self.addCleanup(patcher.stop)

    def run_drop(self, node, until_s, step_s=0.02):
        start = self.clock.now
        while self.clock.now - start < until_s and node._drop_sequence:
            self.clock.now += step_s
            node._on_deploy_tick()


class TestDeploy(DeployTestCase):
    def test_person_gets_bottle_clamp_then_bottle_drop(self):
        node = mission(payload_config(), class_id=0)
        node.enter_deploy_state()
        self.assertEqual(node.current_state, STATE_DEPLOY)
        # Pre-brake: the clamp grips while the plate still holds.
        self.assertEqual(sent(node), [(1685, 1577)])
        node.create_timer.assert_called_once()
        self.run_drop(node, 1.01)
        self.assertEqual(sent(node)[-1][0], 2050)

    def test_tent_gets_beacon_clamp_then_beacon_drop(self):
        node = mission(payload_config(), class_id=1)
        node.enter_deploy_state()
        self.assertEqual(sent(node)[0], (1685, 1300))
        self.run_drop(node, 1.01)
        self.assertEqual(sent(node)[-1][0], 1360)
        # The beacon's own clamped value is used throughout.
        self.assertEqual({clamp for _, clamp in sent(node)}, {1300, 1900})

    def test_commands_are_addressed_do_set_actuator(self):
        # PX4 on the flight controller rejects the broadcast form as
        # UNSUPPORTED and leaves the outputs alone; addressed commands work.
        node = mission(payload_config())
        node.enter_deploy_state()
        request = node.command_client.requests[0]
        self.assertEqual(request.command, MAV_CMD_DO_SET_ACTUATOR)
        self.assertFalse(request.broadcast)
        self.assertEqual(request.param7, 0)
        self.assertTrue(all(math.isnan(p) for p in (
            request.param3, request.param4, request.param5, request.param6)))

    def test_plate_holds_during_the_pre_brake_then_drops(self):
        node = mission(payload_config())
        node.enter_deploy_state()
        self.run_drop(node, 0.99)
        pre = sent(node)
        self.assertEqual([clamp for _, clamp in pre],
                         [1577, 1900, 1577, 1900, 1577])
        self.assertTrue(all(plate == 1685 for plate, _ in pre))
        self.run_drop(node, 1.0)
        after = sent(node)[len(pre):]
        # The clamp keeps one continuous rhythm through the plate moving:
        # every command flips it, none repeats the previous position.
        clamps = [clamp for _, clamp in pre + after]
        self.assertTrue(all(a != b for a, b in zip(clamps, clamps[1:])))
        self.assertEqual(set(clamps), {1577, 1900})
        # Every command re-asserts the drop, so a lost command cannot
        # leave the plate holding the payload.
        self.assertTrue(all(plate == 2050 for plate, _ in after))

    def test_sends_only_on_change(self):
        node = mission(payload_config())
        node.enter_deploy_state()
        self.run_drop(node, 0.19, step_s=0.01)
        self.assertEqual(len(sent(node)), 1)

    def test_completes_unclamped_with_plate_back_at_hold(self):
        node = mission(payload_config())
        node.enter_deploy_state()
        # 1 s pre-brake + 10.76 s of braking: still running at 11.5 s.
        self.run_drop(node, 11.5)
        node.on_deploy_complete.assert_not_called()
        self.run_drop(node, 0.5)
        node.on_deploy_complete.assert_called_once()
        self.assertEqual(sent(node)[-1], (1685, 1900))
        self.assertIsNone(node._drop_sequence)
        node.destroy_timer.assert_called()

    def test_disabled_payload_skips_straight_to_complete(self):
        node = mission(payload=None)
        node.enter_deploy_state()
        self.assertEqual(sent(node), [])
        node.create_timer.assert_not_called()
        node.on_deploy_complete.assert_called_once()

    def test_unknown_class_skips_the_drop(self):
        node = mission(payload_config(), class_id=-1)
        node.enter_deploy_state()
        self.assertEqual(sent(node), [])
        node.on_deploy_complete.assert_called_once()
        node.get_logger().error.assert_called()

    def test_tick_after_leaving_deploy_stops_without_completing(self):
        node = mission(payload_config())
        node.enter_deploy_state()
        node.current_state = 'return'
        self.clock.now += 0.5
        node._on_deploy_tick()
        self.assertIsNone(node._drop_sequence)
        node.on_deploy_complete.assert_not_called()

    def test_rtl_mid_drop_returns_to_rest(self):
        node = mission(payload_config())
        node.enter_deploy_state()
        node.enter_rtl_state(command_mode=False)
        self.assertIsNone(node._drop_sequence)
        self.assertEqual(sent(node)[-1], (1685, 1900))
        node.on_deploy_complete.assert_not_called()

    def test_rtl_without_payload_sends_nothing(self):
        node = mission(payload=None, state='scan')
        node.enter_rtl_state(command_mode=False)
        self.assertEqual(sent(node), [])


class TestPayloadWriter(DeployTestCase):
    def test_one_command_in_flight_then_the_latest_positions(self):
        node = mission(payload_config())
        node.command_client.auto_reply = None     # PX4 has not answered yet
        node.enter_deploy_state()
        self.run_drop(node, 0.3)                  # clamp has toggled open
        self.assertEqual(sent(node), [(1685, 1577)])
        # The reply arrives: the writer sends where the clamp is NOW,
        # skipping any positions it missed in between.
        node.command_client.futures[0].finish(success=True)
        self.assertEqual(sent(node), [(1685, 1577), (1685, 1900)])

    def test_unconfirmed_command_is_resent_after_a_pause(self):
        node = mission(payload_config())
        node.command_client.auto_reply = {'success': False, 'result': 3}
        node.send_payload_rest_positions()
        self.assertEqual(len(sent(node)), 1)
        node._flush_payload()                     # too soon: no resend
        self.assertEqual(len(sent(node)), 1)
        self.clock.now += PAYLOAD_RETRY_SEC + 0.01
        node.command_client.auto_reply = {'success': True}
        node._flush_payload()
        self.assertEqual(sent(node), [(1685, 1900), (1685, 1900)])
        self.assertEqual(node._payload_confirmed, (1685, 1900))
        node.get_logger().warn.assert_called_once()

    def test_confirmed_positions_are_not_resent(self):
        node = mission(payload_config())
        node.send_payload_rest_positions()
        node._flush_payload()
        node.send_payload_rest_positions()
        self.assertEqual(len(sent(node)), 1)

    def test_reply_exception_is_logged_not_raised(self):
        node = mission(payload_config())
        node.command_client.auto_reply = {'error': RuntimeError('link down')}
        node.send_payload_rest_positions()
        self.assertFalse(node._payload_pending)
        self.assertIsNone(node._payload_confirmed)
        node.get_logger().warn.assert_called_once()


class TestRestPositions(DeployTestCase):
    def test_startup_sends_hold_and_rest(self):
        node = mission(payload_config(), state='takeoff')
        node.send_payload_rest_positions()
        self.assertEqual(sent(node), [(1685, 1900)])

    def test_startup_with_payload_disabled_sends_nothing(self):
        node = mission(payload=None, state='takeoff')
        node.send_payload_rest_positions()
        self.assertEqual(sent(node), [])


if __name__ == '__main__':
    unittest.main()
