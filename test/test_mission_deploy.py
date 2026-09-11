"""mission_node's DEPLOY state driving the payload through PX4.

Runs the real MissionRunner methods on a stand-in object, like the other
mission tests: MissionRunner.__init__ needs MAVROS and the installed config.
The clock is patched so the drop sequence advances deterministically.
"""

import math
import unittest
from types import MethodType, SimpleNamespace
from unittest.mock import Mock, patch

from bv_core import mission as mission_module
from bv_core.mission import MissionRunner, STATE_DEPLOY
from bv_core.payload import MAV_CMD_DO_SET_ACTUATOR, load_payload_config


def payload_config():
    return load_payload_config({'payload': {
        'enabled': True,
        'arduino_pulse_range_us': [544, 2400],
        'slider': {'actuator_set': 1, 'zero_deg': 130,
                   'beacon_offset_deg': -30, 'bottle_offset_deg': 40,
                   'pwm_range_us': [544, 2400]},
        'brake': {'actuator_set': 2, 'zero_deg': 125,
                  'pulse_offset_deg': -18, 'pwm_range_us': [544, 2400]},
        'brake_phases': [
            {'drop_ft': 75, 'speed_ftps': 15.0, 'toggle_ms': 200},
            {'drop_ft': 50, 'speed_ftps': 13.3, 'toggle_ms': 150},
            {'drop_ft': 25, 'speed_ftps': 12.5, 'toggle_ms': 100},
        ],
    }})


def mission(payload=None, class_id=0, state='deliver'):
    node = SimpleNamespace(
        current_state=state, is_transitioning=True, payload=payload,
        current_target_class_id=class_id,
        objects_delivered_count=0, num_objects_to_find=2,
        command_client=Mock(), log=Mock(),
        get_logger=Mock(return_value=Mock()), publish_mission_state=Mock(),
        create_timer=Mock(side_effect=lambda *_args: Mock()),
        destroy_timer=Mock(), on_deploy_complete=Mock(),
        approval_gate=None, set_velocity=Mock(), set_flight_mode=Mock(),
        rtl_velocity=5.0,
        _drop_sequence=None, _deploy_timer=None,
    )
    for name in ('enter_deploy_state', '_on_deploy_tick', '_stop_drop',
                 'send_payload_actuators', 'send_payload_rest_positions',
                 'on_payload_command_complete', 'enter_rtl_state'):
        setattr(node, name, MethodType(getattr(MissionRunner, name), node))
    return node


def sent(node):
    """(slider_deg, brake_deg) per command, decoded from actuator values."""
    decoded = []
    for call in node.command_client.call_async.call_args_list:
        request = call.args[0]
        values = []
        for value in (request.param1, request.param2):
            values.append(None if math.isnan(value)
                          else round((value + 1.0) * 90.0))
        decoded.append(tuple(values))
    return decoded


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


class TestDeploy(unittest.TestCase):
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

    def test_person_gets_bottle_release_and_brake_pulse_immediately(self):
        node = mission(payload_config(), class_id=0)
        node.enter_deploy_state()
        self.assertEqual(node.current_state, STATE_DEPLOY)
        self.assertEqual(sent(node), [(170, 107)])
        node.create_timer.assert_called_once()

    def test_tent_gets_beacon_release(self):
        node = mission(payload_config(), class_id=1)
        node.enter_deploy_state()
        self.assertEqual(sent(node)[0], (100, 107))

    def test_commands_are_broadcast_do_set_actuator(self):
        # Broadcast: MAVROS does not wait for an ACK, so one lost ACK cannot
        # block the next brake toggle for its 5 s ACK timeout.
        node = mission(payload_config())
        node.enter_deploy_state()
        request = node.command_client.call_async.call_args.args[0]
        self.assertEqual(request.command, MAV_CMD_DO_SET_ACTUATOR)
        self.assertTrue(request.broadcast)
        self.assertEqual(request.param7, 0)
        self.assertTrue(all(math.isnan(p) for p in (
            request.param3, request.param4, request.param5, request.param6)))

    def test_brake_toggles_and_slider_is_repeated(self):
        node = mission(payload_config())
        node.enter_deploy_state()
        self.run_drop(node, 1.0)
        commands = sent(node)
        self.assertEqual([brake for _, brake in commands],
                         [107, 125, 107, 125, 107, 125])
        # Every command re-asserts the release, so a dropped packet cannot
        # leave the slider holding the payload.
        self.assertTrue(all(slider == 170 for slider, _ in commands))

    def test_sends_only_on_change(self):
        node = mission(payload_config())
        node.enter_deploy_state()
        self.run_drop(node, 0.19, step_s=0.01)
        self.assertEqual(len(sent(node)), 1)

    def test_completes_after_the_full_sequence_with_brake_at_rest(self):
        node = mission(payload_config())
        node.enter_deploy_state()
        self.run_drop(node, 10.0)
        node.on_deploy_complete.assert_not_called()
        self.run_drop(node, 1.0)
        node.on_deploy_complete.assert_called_once()
        self.assertEqual(sent(node)[-1], (170, 125))
        self.assertIsNone(node._drop_sequence)
        node.destroy_timer.assert_called()

    def test_disabled_payload_skips_straight_to_complete(self):
        node = mission(payload=None)
        node.enter_deploy_state()
        node.command_client.call_async.assert_not_called()
        node.create_timer.assert_not_called()
        node.on_deploy_complete.assert_called_once()

    def test_unknown_class_skips_the_drop(self):
        node = mission(payload_config(), class_id=-1)
        node.enter_deploy_state()
        node.command_client.call_async.assert_not_called()
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

    def test_rtl_mid_drop_rests_the_brake(self):
        node = mission(payload_config())
        node.enter_deploy_state()
        node.enter_rtl_state(command_mode=False)
        self.assertIsNone(node._drop_sequence)
        self.assertEqual(sent(node)[-1], (None, 125))
        node.on_deploy_complete.assert_not_called()

    def test_rtl_without_payload_sends_nothing(self):
        node = mission(payload=None, state='scan')
        node.enter_rtl_state(command_mode=False)
        node.command_client.call_async.assert_not_called()


class TestRestPositions(unittest.TestCase):
    def test_startup_sends_hold_and_rest(self):
        node = mission(payload_config(), state='takeoff')
        node.send_payload_rest_positions()
        self.assertEqual(sent(node), [(130, 125)])

    def test_startup_with_payload_disabled_sends_nothing(self):
        node = mission(payload=None, state='takeoff')
        node.send_payload_rest_positions()
        node.command_client.call_async.assert_not_called()


class TestCommandResult(unittest.TestCase):
    def test_failed_command_is_logged_not_raised(self):
        node = mission(payload_config())
        future = Mock(result=Mock(side_effect=RuntimeError('link down')))
        node.on_payload_command_complete(future)
        node.get_logger().warn.assert_called()

    def test_rejected_command_is_logged(self):
        node = mission(payload_config())
        future = Mock(result=lambda: SimpleNamespace(success=False, result=4))
        node.on_payload_command_complete(future)
        node.get_logger().warn.assert_called()


if __name__ == '__main__':
    unittest.main()
