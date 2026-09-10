#!/usr/bin/env python3
"""Regression tests for delayed or duplicate confirmed detections."""

import os
import sys
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_msgs.msg import ConfirmedDetection  # noqa: E402

from bv_core.mission import confirmation_rejection_reason  # noqa: E402
from bv_core.mission import MissionRunner  # noqa: E402


def _confirmation(detection_id='candidate-1', stamp_ns=2_000_000_050):
    message = ConfirmedDetection()
    message.detection_id = detection_id
    message.class_id = 0
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    return message


def test_fresh_confirmation_is_accepted():
    assert confirmation_rejection_reason(
        _confirmation(), 2_000_000_000, set()) is None


def test_confirmation_from_prior_scan_is_stale():
    assert confirmation_rejection_reason(
        _confirmation(stamp_ns=1_999_999_999),
        2_000_000_000,
        set(),
    ) == 'stale'


def test_already_handled_confirmation_is_duplicate():
    assert confirmation_rejection_reason(
        _confirmation(), 2_000_000_000, {'candidate-1'}) == 'duplicate'


def test_confirmation_during_scan_upload_is_processed_when_route_is_ready():
    node = SimpleNamespace(
        current_state='scan',
        scan_started_ns=2_000_000_000,
        scan_route_pending=True,
        _pending_scan_confirmation=None,
        is_transitioning=True,
        handled_confirmation_ids=set(),
        objects_delivered_count=0,
        num_objects_to_find=2,
        current_lat=None,
        current_lon=None,
        confirmed_detection_class_id=-1,
        confirmed_detection_id='',
        confirmed_detection_coords=None,
        get_logger=Mock(return_value=Mock()),
        log=Mock(),
        set_flight_mode=Mock(),
        create_timer=Mock(side_effect=lambda *_args: Mock()),
        reset_all_servos_to_default=Mock(),
        set_velocity=Mock(),
        desired_velocity=3.0,
        _localize_timer=None,
        _velocity_delay_timer=None,
        in_auto_mission=False,
        last_waypoint_reached=None,
        expected_final_waypoint_index=5,
        handle_state_completion=Mock(),
    )
    for name in (
        'on_object_detected', '_drain_pending_scan_confirmation',
        'on_set_mode_complete',
    ):
        setattr(node, name, MethodType(getattr(MissionRunner, name), node))

    message = _confirmation()
    message.latitude = 36.2
    message.longitude = -96.0
    node.on_object_detected(message)

    assert node._pending_scan_confirmation is message
    assert node.handled_confirmation_ids == set()

    node.on_set_mode_complete(
        Mock(result=lambda: SimpleNamespace(mode_sent=True)),
        'AUTO.MISSION',
    )

    assert node.scan_route_pending is False
    assert node._pending_scan_confirmation is None
    assert node.handled_confirmation_ids == {'candidate-1'}
    assert node.confirmed_detection_id == 'candidate-1'
    node.set_flight_mode.assert_called_once_with('AUTO.LOITER')
