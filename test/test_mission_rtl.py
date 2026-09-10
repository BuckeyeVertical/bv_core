"""RTL interruption and completion without constructing a ROS node."""

import unittest
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

from bv_core.mission import MissionRunner


def mission(state='localize'):
    node = SimpleNamespace(
        current_state=state, in_auto_mission=True, rtl_completed=False,
        is_transitioning=True, approval_gate=Mock(), log=Mock(),
        get_logger=Mock(return_value=Mock()), publish_mission_state=Mock(),
        set_flight_mode=Mock(), destroy_timer=Mock(), arm_vehicle=Mock(),
        reset_all_servos_to_default=Mock(), set_velocity=Mock(),
        create_timer=Mock(return_value=Mock()), desired_velocity=3.0,
        last_waypoint_reached=None, expected_final_waypoint_index=0,
        has_armed=True, enter_deliver_state=Mock(),
        scan_route_pending=False, _pending_scan_confirmation=None,
        _localize_timer=Mock(), _localize_retry_timer=Mock(),
        _velocity_delay_timer=Mock(),
    )
    for name in (
        'enter_rtl_state', 'on_vehicle_state_changed', 'handle_state_completion',
        'on_waypoint_push_complete', 'on_arm_complete', 'on_set_mode_complete',
        'on_vision_localization_complete', '_on_approval_granted',
    ):
        setattr(node, name, MethodType(getattr(MissionRunner, name), node))
    return node


class TestMissionRTL(unittest.TestCase):
    def test_completion_requires_disarming_and_is_reported_once(self):
        node = mission('scan')
        node.enter_rtl_state()
        node.set_flight_mode.assert_called_once_with('AUTO.RTL')
        node.on_vehicle_state_changed(SimpleNamespace(mode='AUTO.RTL', armed=True))
        node.handle_state_completion()  # A leftover waypoint cannot imply landing.
        self.assertFalse(node.rtl_completed)
        logger = node.get_logger()
        self.assertFalse(any('COMPLETE' in str(c) for c in logger.info.call_args_list))
        for _ in range(2):
            node.on_vehicle_state_changed(SimpleNamespace(mode='AUTO.LOITER', armed=False))
        self.assertTrue(node.rtl_completed)
        self.assertEqual(sum('RETURN COMPLETE - DISARMED' in str(c)
                             for c in logger.info.call_args_list), 1)

    def test_external_rtl_cancels_pending_work_in_each_active_phase(self):
        for state in ('takeoff', 'lap', 'scan', 'localize', 'deliver', 'deploy'):
            with self.subTest(state=state):
                node = mission(state)
                timers = [node._localize_timer, node._localize_retry_timer,
                          node._velocity_delay_timer]
                node.on_vehicle_state_changed(SimpleNamespace(mode='AUTO.RTL', armed=True))
                self.assertEqual(node.current_state, 'return')
                node.approval_gate.cancel.assert_called_once_with('rtl')
                node.set_flight_mode.assert_not_called()
                node.publish_mission_state.assert_called_once()
                for timer in timers:
                    timer.cancel.assert_called_once()
                    node.destroy_timer.assert_any_call(timer)
                self.assertIsNone(node._localize_timer)
                self.assertIsNone(node._localize_retry_timer)
                self.assertIsNone(node._velocity_delay_timer)

    def test_late_replies_and_approval_cannot_resume_mission(self):
        node = mission()
        node.on_vehicle_state_changed(SimpleNamespace(mode='AUTO.RTL', armed=True))
        future = Mock()
        for name in ('on_waypoint_push_complete', 'on_arm_complete',
                     'on_vision_localization_complete'):
            getattr(node, name)(future)
        node._on_approval_granted()
        future.result.assert_not_called()
        node.on_set_mode_complete(Mock(result=lambda: SimpleNamespace(mode_sent=True)))
        node.set_flight_mode.assert_not_called()
        node.arm_vehicle.assert_not_called()
        node.enter_deliver_state.assert_not_called()
        node.reset_all_servos_to_default.assert_not_called()

    def test_failed_rtl_mode_request_is_still_logged(self):
        node = mission('return')
        node.on_set_mode_complete(Mock(result=lambda: SimpleNamespace(mode_sent=False)))
        node.get_logger().error.assert_called_once_with('Mode change failed!')
        node.set_flight_mode.assert_not_called()

    def test_queued_velocity_callback_does_nothing_after_rtl(self):
        node = mission('scan')
        node.on_set_mode_complete(Mock(result=lambda: SimpleNamespace(mode_sent=True)))
        callback = node.create_timer.call_args.args[1]
        node.on_vehicle_state_changed(SimpleNamespace(mode='AUTO.RTL', armed=True))
        callback()
        node.set_velocity.assert_not_called()

    def test_normal_mission_keeps_existing_behavior(self):
        node = mission('scan')
        node.on_waypoint_push_complete(Mock(result=lambda: SimpleNamespace(success=True)))
        node.set_flight_mode.assert_called_once_with('AUTO.MISSION')
        node.on_set_mode_complete(Mock(result=lambda: SimpleNamespace(mode_sent=True)))
        self.assertFalse(node.is_transitioning)
        node.reset_all_servos_to_default.assert_called_once()
        node.create_timer.call_args.args[1]()
        node.set_velocity.assert_called_once_with(3.0)

    def test_loiter_response_keeps_detection_transition_locked(self):
        node = mission('scan')

        node.on_set_mode_complete(
            Mock(result=lambda: SimpleNamespace(mode_sent=True)),
            'AUTO.LOITER',
        )

        self.assertTrue(node.is_transitioning)
        node.reset_all_servos_to_default.assert_not_called()
        node.create_timer.assert_not_called()

    def test_set_flight_mode_forwards_requested_mode_to_callback(self):
        future = Mock()
        node = SimpleNamespace(
            set_mode_client=Mock(call_async=Mock(return_value=future)),
            on_set_mode_complete=Mock(),
            get_logger=Mock(return_value=Mock()),
        )
        node.set_flight_mode = MethodType(MissionRunner.set_flight_mode, node)

        node.set_flight_mode('AUTO.LOITER')
        callback = future.add_done_callback.call_args.args[0]
        completed = Mock()
        callback(completed)

        node.on_set_mode_complete.assert_called_once_with(
            completed, 'AUTO.LOITER')

    def test_loiter_is_not_an_external_rtl(self):
        node = mission()
        node.on_vehicle_state_changed(SimpleNamespace(mode='AUTO.LOITER', armed=True))
        self.assertEqual(node.current_state, 'localize')
        node.approval_gate.cancel.assert_not_called()


if __name__ == '__main__':
    unittest.main()
