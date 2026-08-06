#!/usr/bin/env python3
"""Tests for ApprovalGate — uses a stub node, no rclpy spin required.

Requires bv_msgs on the path:  source ~/bv_ws/install/setup.bash
"""

import os
import sys

from builtin_interfaces.msg import Time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_msgs.srv import DetectionDecision  # noqa: E402

from bv_core.approval_gate import ApprovalGate  # noqa: E402


class _StubTimer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class _StubPublisher:
    def __init__(self):
        self.published = []
        self.subscribers = 1

    def publish(self, msg):
        self.published.append(msg)

    def get_subscription_count(self):
        return self.subscribers


class _StubLogger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(('info', msg))

    def warn(self, msg):
        self.lines.append(('warn', msg))

    def error(self, msg):
        self.lines.append(('error', msg))


class _StubClock:
    def now(self):
        return self

    def to_msg(self):
        return Time()


class StubNode:
    """Minimal stand-in for rclpy.node.Node."""

    def __init__(self):
        self.publisher = _StubPublisher()
        self.timers = []
        self.services = {}
        self._logger = _StubLogger()
        self._clock = _StubClock()

    def create_publisher(self, msg_type, topic, qos):
        return self.publisher

    def create_service(self, srv_type, name, callback):
        self.services[name] = callback
        return object()

    def create_timer(self, period, callback):
        timer = _StubTimer(period, callback)
        self.timers.append(timer)
        return timer

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock


@pytest.fixture
def gate_env():
    node = StubNode()
    gate = ApprovalGate(node, timeout_sec=180.0)
    calls = {'approve': 0, 'reject': []}

    def on_approve():
        calls['approve'] += 1

    def on_reject(lat, lon, class_id, reason):
        calls['reject'].append((lat, lon, class_id, reason))

    def make_request():
        return gate.request(
            class_id=0, lat=38.3877, lon=-76.4190, alt=15.2, confidence=0.94,
            drone_lat=38.3876, drone_lon=-76.4191, annotated_crop=b'\xff\xd8fake',
            on_approve=on_approve, on_reject=on_reject)

    return node, gate, calls, make_request


def _decide(node, detection_id, approved, reason=''):
    request = DetectionDecision.Request()
    request.detection_id = detection_id
    request.approved = approved
    request.reason = reason
    response = DetectionDecision.Response()
    return node.services['/detection_decision'](request, response)


class TestRequest:
    def test_publishes_pending_with_generated_id(self, gate_env):
        node, gate, _, make_request = gate_env
        detection_id = make_request()
        assert detection_id
        assert gate.is_pending()
        msg = node.publisher.published[-1]
        assert msg.detection_id == detection_id
        assert msg.class_id == 0
        assert msg.timeout_sec == pytest.approx(180.0)
        assert bytes(msg.annotated_crop.data) == b'\xff\xd8fake'
        assert msg.annotated_crop.format == 'jpeg'

    def test_arms_timeout_timer(self, gate_env):
        node, _, _, make_request = gate_env
        make_request()
        assert len(node.timers) == 1
        assert node.timers[0].period == pytest.approx(180.0)

    def test_zero_timeout_arms_no_timer(self):
        node = StubNode()
        gate = ApprovalGate(node, timeout_sec=0.0)
        gate.request(class_id=0, lat=0.0, lon=0.0, alt=0.0, confidence=0.0,
                     drone_lat=0.0, drone_lon=0.0, annotated_crop=b'',
                     on_approve=lambda: None, on_reject=lambda *a: None)
        assert node.timers == []


class TestDecision:
    def test_approve_invokes_callback_and_clears(self, gate_env):
        node, gate, calls, make_request = gate_env
        detection_id = make_request()
        response = _decide(node, detection_id, True)
        assert response.accepted is True
        assert calls['approve'] == 1
        assert not gate.is_pending()

    def test_reject_passes_location_and_reason(self, gate_env):
        node, gate, calls, make_request = gate_env
        detection_id = make_request()
        response = _decide(node, detection_id, False, 'shadow')
        assert response.accepted is True
        assert calls['reject'] == [(38.3877, -76.4190, 0, 'shadow')]
        assert not gate.is_pending()

    def test_stale_id_is_refused_and_pending_survives(self, gate_env):
        node, gate, calls, make_request = gate_env
        make_request()
        response = _decide(node, 'not-the-active-id', True)
        assert response.accepted is False
        assert 'mismatch' in response.message
        assert calls['approve'] == 0
        assert gate.is_pending()      # a stale click must not clear a live pending

    def test_decision_with_nothing_pending_is_refused(self, gate_env):
        node, _, calls, _ = gate_env
        response = _decide(node, 'anything', True)
        assert response.accepted is False
        assert calls['approve'] == 0

    def test_clearing_publishes_empty_pending(self, gate_env):
        node, _, _, make_request = gate_env
        detection_id = make_request()
        _decide(node, detection_id, True)
        assert node.publisher.published[-1].detection_id == ''

    def test_decision_cancels_the_timer(self, gate_env):
        node, _, _, make_request = gate_env
        detection_id = make_request()
        _decide(node, detection_id, True)
        assert node.timers[0].cancelled is True


class TestTimeout:
    def test_timeout_approves(self, gate_env):
        node, gate, calls, make_request = gate_env
        make_request()
        node.timers[0].callback()          # fire the timer
        assert calls['approve'] == 1       # fails OPEN: deploys, does not skip
        assert calls['reject'] == []
        assert not gate.is_pending()

    def test_timeout_with_nothing_pending_is_a_noop(self, gate_env):
        node, gate, calls, make_request = gate_env
        detection_id = make_request()
        _decide(node, detection_id, False)
        node.timers[0].callback()          # late fire after the FSM moved on
        assert calls['approve'] == 0


class TestCancel:
    def test_cancel_clears_without_invoking_callbacks(self, gate_env):
        node, gate, calls, make_request = gate_env
        make_request()
        gate.cancel('rtl')
        assert not gate.is_pending()
        assert calls['approve'] == 0
        assert calls['reject'] == []
        assert node.publisher.published[-1].detection_id == ''
