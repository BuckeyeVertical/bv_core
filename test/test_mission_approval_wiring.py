#!/usr/bin/env python3
"""Tests for mission_node's approval callbacks and its contract with ApprovalGate.

These drive `MissionRunner._on_approval_granted` / `_on_approval_rejected` — the real
functions, taken off the real class — against a lightweight stand-in that carries only
the attributes those two methods touch. `MissionRunner.__init__` reads the installed
config and builds MAVROS clients, so it is never called here.

The last class is the important one: it wires the real ApprovalGate to mission_node's
real bound methods and drives a real verdict through the gate's service callback. The
gate SWALLOWS exceptions raised by these callbacks, so a signature drift (a renamed
keyword, a wrong arity) would otherwise fail silently and strand the aircraft loitering
in STATE_LOCALIZE. These tests therefore assert observable side effects, never the
absence of an exception.

Requires bv_msgs on the path:  source ~/bv_ws/install/setup.bash
"""

import os
import sys

from builtin_interfaces.msg import Time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_msgs.msg import ObjectLocations  # noqa: E402
from bv_msgs.srv import DetectionDecision, LocalizeObject  # noqa: E402

from bv_core.approval_gate import ApprovalGate  # noqa: E402
from bv_core.mission import (  # noqa: E402
    MissionRunner, STATE_DELIVER, STATE_LOCALIZE, STATE_RTL,
)


class _StubLogger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(('info', msg))

    def warn(self, msg):
        self.lines.append(('warn', msg))

    def error(self, msg):
        self.lines.append(('error', msg))


class _RecordingPublisher:
    """Publisher that appends to a shared call-order log."""

    def __init__(self, calls):
        self.calls = calls
        self.published = []

    def publish(self, msg):
        self.published.append(msg)
        self.calls.append('publish')

    def get_subscription_count(self):
        return 1


class FakeMission:
    """Stand-in carrying exactly the attributes the two callbacks touch.

    The callbacks are the genuine functions lifted off MissionRunner, so a change to
    either method body is exercised here without instantiating a ROS node.
    """

    _on_approval_granted = MissionRunner._on_approval_granted
    _on_approval_rejected = MissionRunner._on_approval_rejected
    on_vision_localization_complete = MissionRunner.on_vision_localization_complete

    def __init__(self, state=STATE_LOCALIZE):
        self.current_state = state
        self.calls = []
        self.rejected_object_pub = _RecordingPublisher(self.calls)
        self._logger = _StubLogger()
        self.current_target_coords = (38.3877, -76.4190, 15.2)
        self.current_target_class_id = 1
        self.confirmed_detection_class_id = 1
        # Only read by on_vision_localization_complete's success path.
        self.approval_gate = None
        self.localization_retry_count = 0
        self._localize_retry_timer = None
        self.current_lat = 38.3876
        self.current_lon = -76.4191

    def get_logger(self):
        return self._logger

    def enter_deliver_state(self):
        self.calls.append('enter_deliver_state')
        self.current_state = STATE_DELIVER

    def enter_scan_state(self):
        self.calls.append('enter_scan_state')
        self.current_state = 'scan'


class TestApprovalGranted:
    def test_delivers(self):
        mission = FakeMission()
        mission._on_approval_granted()
        assert mission.calls == ['enter_deliver_state']

    def test_publishes_nothing(self):
        mission = FakeMission()
        mission._on_approval_granted()
        assert mission.rejected_object_pub.published == []

    @pytest.mark.parametrize('state', [STATE_RTL, STATE_DELIVER, 'scan'])
    def test_late_verdict_outside_localize_is_ignored(self, state):
        mission = FakeMission(state=state)
        mission._on_approval_granted()
        assert mission.calls == []
        assert mission.current_state == state


class TestApprovalRejected:
    def test_publishes_before_transitioning_to_scan(self):
        # Ordering requirement from Task 4's review: enter_scan_state() publishes
        # mission_state='scan', on which filtering_node re-arms every non-deployed
        # class. If scan landed first, filtering could re-confirm the same blob and
        # overwrite the position the rejection exists to suppress.
        mission = FakeMission()
        mission._on_approval_rejected(38.3877, -76.4190, 1, 'shadow')
        assert mission.calls == ['publish', 'enter_scan_state']
        assert mission.calls.index('publish') < mission.calls.index('enter_scan_state')

    def test_published_message_carries_the_rejected_location(self):
        mission = FakeMission()
        mission._on_approval_rejected(38.3877010611, -76.41905421, 1, 'shadow')
        msg = mission.rejected_object_pub.published[-1]
        assert isinstance(msg, ObjectLocations)
        assert msg.latitude == pytest.approx(38.3877010611)
        assert msg.longitude == pytest.approx(-76.41905421)
        assert msg.class_id == 1

    def test_message_fields_are_coerced_to_the_ros_types(self):
        # ObjectLocations is float64/float64/int32; ints or numpy scalars arriving
        # from the gate must not raise on assignment.
        mission = FakeMission()
        mission._on_approval_rejected(38, -76, True, '')
        msg = mission.rejected_object_pub.published[-1]
        assert msg.latitude == pytest.approx(38.0)
        assert msg.class_id == 1

    def test_clears_the_target_and_confirmation(self):
        mission = FakeMission()
        mission._on_approval_rejected(38.3877, -76.4190, 1, 'shadow')
        assert mission.current_target_coords is None
        assert mission.current_target_class_id is None
        assert mission.confirmed_detection_class_id == -1

    def test_logs_the_class_name_and_reason(self):
        mission = FakeMission()
        mission._on_approval_rejected(38.3877, -76.4190, 1, 'shadow')
        line = mission.get_logger().lines[-1][1]
        assert 'tent' in line
        assert 'shadow' in line

    def test_out_of_range_class_id_does_not_raise(self):
        mission = FakeMission()
        mission._on_approval_rejected(38.3877, -76.4190, 99, '')
        assert 'unknown' in mission.get_logger().lines[-1][1]
        assert mission.calls == ['publish', 'enter_scan_state']

    @pytest.mark.parametrize('state', [STATE_RTL, STATE_DELIVER, 'scan'])
    def test_late_verdict_outside_localize_is_ignored(self, state):
        mission = FakeMission(state=state)
        mission._on_approval_rejected(38.3877, -76.4190, 1, 'shadow')
        assert mission.calls == []
        assert mission.rejected_object_pub.published == []
        assert mission.current_target_coords is not None
        assert mission.confirmed_detection_class_id == 1


# Live contract against the real gate

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

    def publish(self, msg):
        self.published.append(msg)

    def get_subscription_count(self):
        return 1


class _StubClock:
    def now(self):
        return self

    def to_msg(self):
        return Time()


class GateStubNode:
    """Minimal rclpy.node.Node stand-in, for constructing a real ApprovalGate."""

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

    def destroy_timer(self, timer):
        pass

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock


class _FakeFuture:
    def __init__(self, response):
        self._response = response

    def result(self):
        return self._response


def _localize_response(crop=b'\xff\xd8fake'):
    response = LocalizeObject.Response()
    response.success = True
    response.latitude = 38.3877010611
    response.longitude = -76.4190542
    response.altitude = 15.2
    response.class_id = 1
    response.confidence = 0.94
    response.annotated_crop.format = 'jpeg'
    response.annotated_crop.data = crop
    return response


def _wired(crop=b'\xff\xd8fake'):
    """A real ApprovalGate armed through mission_node's real request call site.

    Nothing here restates the keyword arguments: `on_vision_localization_complete`
    builds the `gate.request(...)` call itself, so a renamed or dropped keyword in
    mission.py surfaces as a TypeError right here.
    """
    node = GateStubNode()
    mission = FakeMission()
    mission.approval_gate = ApprovalGate(node, timeout_sec=180.0)
    mission.on_vision_localization_complete(_FakeFuture(_localize_response(crop)))
    detection_id = node.publisher.published[-1].detection_id
    return node, mission.approval_gate, mission, detection_id


def _decide(node, detection_id, approved, reason=''):
    request = DetectionDecision.Request()
    request.detection_id = detection_id
    request.approved = approved
    request.reason = reason
    return node.services['/detection_decision'](request, DetectionDecision.Response())


class TestLiveGateContract:
    """The gate swallows callback exceptions, so assert side effects, not silence."""

    def test_request_accepts_mission_nodes_call_site_arguments(self):
        node, gate, mission, detection_id = _wired()
        assert detection_id
        assert gate.is_pending()
        # The gate holds, rather than delivering, while a verdict is outstanding.
        assert mission.calls == []
        assert mission.current_state == STATE_LOCALIZE

        pending = node.publisher.published[-1]
        assert pending.class_id == 1
        assert pending.latitude == pytest.approx(38.3877010611)
        assert pending.longitude == pytest.approx(-76.4190542)
        assert pending.altitude == pytest.approx(15.2)
        assert pending.confidence == pytest.approx(0.94)
        assert pending.drone_latitude == pytest.approx(38.3876)
        assert pending.drone_longitude == pytest.approx(-76.4191)
        assert bytes(pending.annotated_crop.data) == b'\xff\xd8fake'

    def test_empty_crop_still_arms_the_gate_and_warns(self):
        # vision_node leaves format='jpeg' on crop failure, so mission.py must key
        # "no image" off len(data), not off format.
        node, gate, mission, _ = _wired(crop=b'')
        assert gate.is_pending()
        assert bytes(node.publisher.published[-1].annotated_crop.data) == b''
        assert any(level == 'warn' and 'annotated crop' in text
                   for level, text in mission.get_logger().lines)

    def test_gate_disabled_delivers_without_touching_the_gate(self):
        # The governing property: with Approval_required false the gate is None and
        # localization completion runs straight into delivery, as it always has.
        mission = FakeMission()
        assert mission.approval_gate is None
        mission.on_vision_localization_complete(_FakeFuture(_localize_response()))
        assert mission.calls == ['enter_deliver_state']
        assert mission.current_target_coords == pytest.approx(
            (38.3877010611, -76.4190542, 15.2))
        assert mission.current_target_class_id == 1

    def test_real_reject_verdict_runs_the_callback_to_completion(self):
        # The stranding case: if on_reject's arity or the keyword names ever drift,
        # the gate clears the pending, cancels the timer, catches the TypeError, and
        # the aircraft loiters in STATE_LOCALIZE forever. These assertions are the
        # only thing that would catch it.
        node, gate, mission, detection_id = _wired()
        response = _decide(node, detection_id, False, 'shadow')

        assert response.accepted is True
        assert mission.calls == ['publish', 'enter_scan_state']
        msg = mission.rejected_object_pub.published[-1]
        assert msg.latitude == pytest.approx(38.3877010611)
        assert msg.longitude == pytest.approx(-76.4190542)
        assert msg.class_id == 1
        assert mission.current_target_coords is None
        assert mission.current_target_class_id is None
        assert mission.confirmed_detection_class_id == -1
        assert not gate.is_pending()
        assert not any(level == 'error' for level, _ in node.get_logger().lines)

    def test_real_approve_verdict_runs_the_callback_to_completion(self):
        node, gate, mission, detection_id = _wired()
        response = _decide(node, detection_id, True)

        assert response.accepted is True
        assert mission.calls == ['enter_deliver_state']
        assert not gate.is_pending()
        assert not any(level == 'error' for level, _ in node.get_logger().lines)

    def test_timeout_fires_the_same_deliver_path_as_an_approval(self):
        # Fail-open: an absent operator must still deploy, via the identical call.
        node, gate, mission, _ = _wired()
        node.timers[0].callback()

        assert mission.calls == ['enter_deliver_state']
        assert not gate.is_pending()
        assert not any(level == 'error' for level, _ in node.get_logger().lines)

    def test_cancel_on_rtl_invokes_neither_callback(self):
        node, gate, mission, _ = _wired()
        gate.cancel('rtl')

        assert mission.calls == []
        assert not gate.is_pending()
