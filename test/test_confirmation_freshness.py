#!/usr/bin/env python3
"""Regression tests for delayed or duplicate confirmed detections."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_msgs.msg import ConfirmedDetection  # noqa: E402

from bv_core.mission import confirmation_rejection_reason  # noqa: E402


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
